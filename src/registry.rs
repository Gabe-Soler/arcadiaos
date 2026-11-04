//! Capability registry: which robot can do what, how fast, and at what
//! energy cost, plus the travel and work cost models built on those specs.
//!
//! Everything is derived from the robot specs in `hello`, so new robot designs
//! join without code changes. A capability is usable when it has a positive
//! rate (and, for hauling and charging, a positive capacity); a rejection for
//! a missing capability disables it for that robot.
//!
//! Each robot also carries performance factors ([`Perf`]): wear and degraded
//! faults make it slower and hungrier than its spec, and every cost model here
//! applies them, so estimates, reserves and charge targets follow the robot's
//! actual condition.

use crate::geom;
use arcadia_protocol::{CapabilityName, CapabilitySpec, GridCoord, RobotSpec, Scenario};
use std::collections::{BTreeMap, BTreeSet};

/// Capabilities that do landing-pad work, in pipeline order.
pub const WORK_CAPABILITIES: [CapabilityName; 6] = [
    CapabilityName::Scan,
    CapabilityName::Dig,
    CapabilityName::Haul,
    CapabilityName::Grade,
    CapabilityName::Compact,
    CapabilityName::Sinter,
];

/// Static performance of one robot.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotCaps {
    /// Robot id.
    pub id: String,
    /// Driving speed, m/s.
    pub speed_m_s: f64,
    /// Driving energy, Wh/m.
    pub drive_wh_per_m: f64,
    /// Always-on draw, W.
    pub idle_w: f64,
    /// Battery capacity, Wh.
    pub battery_wh: f64,
    caps: BTreeMap<CapabilityName, CapabilitySpec>,
}

/// How a robot currently performs against its spec. Wear and degraded faults
/// push every factor away from 1; a repair or service brings them back.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Perf {
    /// Driving speed factor (at most 1).
    pub speed: f64,
    /// Drive energy factor (at least 1).
    pub drive_energy: f64,
    /// Work rate factor for everything but scans (at most 1).
    pub work_rate: f64,
    /// Work energy factor for everything but scans (at least 1).
    pub work_energy: f64,
    /// Scan rate factor (at most 1).
    pub scan_rate: f64,
    /// Factor on all energy drawn (at least 1).
    pub energy: f64,
}

impl Default for Perf {
    fn default() -> Self {
        Self {
            speed: 1.0,
            drive_energy: 1.0,
            work_rate: 1.0,
            work_energy: 1.0,
            scan_rate: 1.0,
            energy: 1.0,
        }
    }
}

impl Perf {
    /// The factors with each clamped to a sane range, so a bad estimate can
    /// never make a robot look free or infinitely slow.
    pub fn sanitized(self) -> Self {
        let slow = |v: f64| {
            if v.is_finite() {
                v.clamp(0.2, 1.0)
            } else {
                1.0
            }
        };
        let hungry = |v: f64| {
            if v.is_finite() {
                v.clamp(1.0, 3.0)
            } else {
                1.0
            }
        };
        Self {
            speed: slow(self.speed),
            drive_energy: hungry(self.drive_energy),
            work_rate: slow(self.work_rate),
            work_energy: hungry(self.work_energy),
            scan_rate: slow(self.scan_rate),
            energy: hungry(self.energy),
        }
    }
}

/// A charging station as the planner sees it.
#[derive(Debug, Clone, PartialEq)]
pub struct StationInfo {
    /// Station id.
    pub id: String,
    /// Where robots charge.
    pub position: GridCoord,
    /// Power per slot, W.
    pub slot_power_w: f64,
    /// Robots that can charge at once.
    pub slots: u32,
}

/// Who can do what, at what rate and cost.
#[derive(Debug, Clone, PartialEq)]
pub struct Registry {
    /// Cell edge length, m.
    pub cell_size_m: f64,
    robots: Vec<RobotCaps>,
    stations: Vec<StationInfo>,
    disabled: BTreeSet<(usize, CapabilityName)>,
    perf: Vec<Perf>,
}

fn finite_pos(v: f64) -> f64 {
    if v.is_finite() && v > 0.0 { v } else { 0.0 }
}

impl Registry {
    /// Build from the scenario. Robot indices follow the fleet (first spec
    /// wins for a repeated id).
    pub fn new(scenario: &Scenario) -> Self {
        let mut seen = BTreeSet::new();
        let robots = scenario
            .robots
            .iter()
            .filter(|s| seen.insert(s.robot_id.clone()))
            .map(Self::caps_of)
            .collect();
        let stations = scenario
            .stations
            .iter()
            .map(|s| StationInfo {
                id: s.station_id.clone(),
                position: s.position,
                slot_power_w: finite_pos(s.max_power_w) / f64::from(s.slots.max(1)),
                slots: s.slots,
            })
            .collect();
        let cell = scenario.site.cell_size_m;
        let robots: Vec<RobotCaps> = robots;
        Self {
            cell_size_m: if cell.is_finite() && cell > 0.0 {
                cell
            } else {
                1.0
            },
            perf: vec![Perf::default(); robots.len()],
            robots,
            stations,
            disabled: BTreeSet::new(),
        }
    }

    fn caps_of(spec: &RobotSpec) -> RobotCaps {
        let mut caps = BTreeMap::new();
        for c in &spec.capabilities {
            caps.entry(c.name).or_insert_with(|| c.clone());
        }
        RobotCaps {
            id: spec.robot_id.clone(),
            speed_m_s: finite_pos(spec.max_speed_m_s),
            drive_wh_per_m: finite_pos(spec.drive_energy_wh_per_m),
            idle_w: finite_pos(spec.idle_power_w),
            battery_wh: finite_pos(spec.battery_capacity_wh),
            caps,
        }
    }

    /// Number of robots.
    pub fn len(&self) -> usize {
        self.robots.len()
    }

    /// Whether there are no robots.
    pub fn is_empty(&self) -> bool {
        self.robots.is_empty()
    }

    /// Robot `r`.
    pub fn robot(&self, r: usize) -> &RobotCaps {
        &self.robots[r]
    }

    /// Charging stations.
    pub fn stations(&self) -> &[StationInfo] {
        &self.stations
    }

    fn spec(&self, r: usize, cap: CapabilityName) -> Option<&CapabilitySpec> {
        self.robots.get(r)?.caps.get(&cap)
    }

    /// Whether robot `r` can do `cap` work.
    pub fn can(&self, r: usize, cap: CapabilityName) -> bool {
        if self.disabled.contains(&(r, cap)) || self.robots.get(r).is_none() {
            return false;
        }
        let Some(spec) = self.spec(r, cap) else {
            return false;
        };
        match cap {
            CapabilityName::Relay => true,
            CapabilityName::Haul | CapabilityName::Charge => {
                finite_pos(spec.rate) > 0.0 && spec.capacity.is_some_and(|c| finite_pos(c) > 0.0)
            }
            _ => finite_pos(spec.rate) > 0.0,
        }
    }

    /// Every robot that can do `cap`, in index order.
    pub fn capable(&self, cap: CapabilityName) -> Vec<usize> {
        (0..self.robots.len())
            .filter(|&r| self.can(r, cap))
            .collect()
    }

    /// Robot `r`'s current performance factors.
    pub fn perf(&self, r: usize) -> Perf {
        self.perf.get(r).copied().unwrap_or_default()
    }

    /// Set robot `r`'s performance factors (sanitized).
    pub fn set_perf(&mut self, r: usize, perf: Perf) {
        if let Some(p) = self.perf.get_mut(r) {
            *p = perf.sanitized();
        }
    }

    /// The spec work rate of `cap`, ignoring wear and faults (0 when unable).
    pub fn spec_rate(&self, r: usize, cap: CapabilityName) -> f64 {
        if !self.can(r, cap) {
            return 0.0;
        }
        self.spec(r, cap).map_or(0.0, |s| finite_pos(s.rate))
    }

    /// Work rate, units per second, as the robot performs now (0 when unable).
    pub fn rate(&self, r: usize, cap: CapabilityName) -> f64 {
        let p = self.perf(r);
        let factor = match cap {
            CapabilityName::Scan => p.scan_rate,
            CapabilityName::Charge | CapabilityName::Relay => 1.0,
            _ => p.work_rate,
        };
        self.spec_rate(r, cap) * factor
    }

    /// Energy per work unit as the robot performs now, Wh.
    pub fn energy_per_unit(&self, r: usize, cap: CapabilityName) -> f64 {
        let p = self.perf(r);
        let factor = match cap {
            CapabilityName::Scan => p.energy,
            CapabilityName::Charge | CapabilityName::Relay => 1.0,
            _ => p.work_energy * p.energy,
        };
        self.spec(r, cap)
            .map_or(0.0, |s| finite_pos(s.energy_wh_per_unit))
            * factor
    }

    /// Driving speed as the robot performs now, m/s.
    pub fn speed_m_s(&self, r: usize) -> f64 {
        self.robots[r].speed_m_s * self.perf(r).speed
    }

    /// Drive energy per metre as the robot performs now, Wh/m.
    pub fn drive_wh_per_m(&self, r: usize) -> f64 {
        let p = self.perf(r);
        self.robots[r].drive_wh_per_m * p.drive_energy * p.energy
    }

    /// Idle energy over `seconds` as the robot performs now, Wh.
    pub fn idle_wh(&self, r: usize, seconds: f64) -> f64 {
        seconds.max(0.0) * self.robots[r].idle_w * self.perf(r).energy / 3600.0
    }

    /// Haul payload capacity, kg.
    pub fn haul_capacity_kg(&self, r: usize) -> f64 {
        if !self.can(r, CapabilityName::Haul) {
            return 0.0;
        }
        self.spec(r, CapabilityName::Haul)
            .and_then(|s| s.capacity)
            .map_or(0.0, finite_pos)
    }

    /// Mobile charging power, W.
    pub fn charger_power_w(&self, r: usize) -> f64 {
        self.spec_rate(r, CapabilityName::Charge) * 3600.0
    }

    /// Mobile charger tank capacity, Wh.
    pub fn tank_capacity_wh(&self, r: usize) -> f64 {
        if !self.can(r, CapabilityName::Charge) {
            return 0.0;
        }
        self.spec(r, CapabilityName::Charge)
            .and_then(|s| s.capacity)
            .map_or(0.0, finite_pos)
    }

    /// Stop offering `cap` to robot `r` (the simulator says it cannot).
    pub fn disable(&mut self, r: usize, cap: CapabilityName) {
        self.disabled.insert((r, cap));
    }

    /// Driving time from `a` to `b`, s.
    pub fn travel_s(&self, r: usize, a: GridCoord, b: GridCoord) -> f64 {
        let speed = self.speed_m_s(r);
        let d = geom::dist(a, b) * self.cell_size_m;
        if d == 0.0 {
            0.0
        } else if speed > 0.0 {
            d / speed
        } else {
            f64::INFINITY
        }
    }

    /// Driving energy from `a` to `b`, including idle draw on the way, Wh.
    pub fn travel_wh(&self, r: usize, a: GridCoord, b: GridCoord) -> f64 {
        let d = geom::dist(a, b) * self.cell_size_m;
        let t = self.travel_s(r, a, b);
        if t.is_finite() {
            d * self.drive_wh_per_m(r) + self.idle_wh(r, t)
        } else {
            f64::INFINITY
        }
    }

    /// Time for robot `r` to tow another robot from `a` to `b`, s.
    pub fn tow_s(&self, r: usize, a: GridCoord, b: GridCoord, speed_factor: f64) -> f64 {
        let t = self.travel_s(r, a, b);
        if speed_factor > 0.0 {
            t / speed_factor
        } else {
            f64::INFINITY
        }
    }

    /// Energy for robot `r` to tow another robot from `a` to `b`, Wh.
    pub fn tow_wh(
        &self,
        r: usize,
        a: GridCoord,
        b: GridCoord,
        speed_factor: f64,
        energy_factor: f64,
    ) -> f64 {
        let d = geom::dist(a, b) * self.cell_size_m;
        let t = self.tow_s(r, a, b, speed_factor);
        if t.is_finite() {
            d * self.drive_wh_per_m(r) * energy_factor.max(1.0) + self.idle_wh(r, t)
        } else {
            f64::INFINITY
        }
    }

    /// Time for `units` of `cap` work in one step, s.
    pub fn work_s(&self, r: usize, cap: CapabilityName, units: f64, min_work_s: f64) -> f64 {
        let rate = self.rate(r, cap);
        if rate > 0.0 {
            (units.max(0.0) / rate).max(min_work_s)
        } else {
            f64::INFINITY
        }
    }

    /// Energy for `units` of `cap` work in one step, including idle draw, Wh.
    pub fn work_wh(&self, r: usize, cap: CapabilityName, units: f64, min_work_s: f64) -> f64 {
        let t = self.work_s(r, cap, units, min_work_s);
        units.max(0.0) * self.energy_per_unit(r, cap) + self.idle_wh(r, t)
    }

    /// Station charging power for one slot, best over all stations, W.
    pub fn best_slot_power_w(&self) -> f64 {
        self.stations
            .iter()
            .map(|s| s.slot_power_w)
            .fold(0.0, f64::max)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use arcadia_protocol::*;

    pub fn cap(name: CapabilityName, rate: f64, e: f64, capacity: Option<f64>) -> CapabilitySpec {
        CapabilitySpec {
            name,
            rate,
            energy_wh_per_unit: e,
            capacity,
            range_m: None,
        }
    }

    pub fn robot(id: &str, speed: f64, caps: Vec<CapabilitySpec>) -> RobotSpec {
        RobotSpec {
            robot_id: id.into(),
            model: None,
            mass_kg: 100.0,
            battery_capacity_wh: 5000.0,
            max_speed_m_s: speed,
            drive_energy_wh_per_m: 0.5,
            idle_power_w: 36.0,
            capabilities: caps,
        }
    }

    pub fn scenario(robots: Vec<RobotSpec>) -> Scenario {
        Scenario {
            scenario_id: "t".into(),
            start_t: 0.0,
            time_limit_t: SOL_S,
            sol_length_s: SOL_S,
            site: SiteMap {
                site_id: "s".into(),
                name: None,
                width: 10,
                height: 10,
                cell_size_m: 2.0,
                cells: vec![],
            },
            robots,
            stations: vec![ChargeStation {
                station_id: "st".into(),
                position: GridCoord { x: 0, y: 0 },
                max_power_w: 6000.0,
                slots: 2,
            }],
            depots: vec![],
            power: PowerSystem {
                mode: PowerMode::Hybrid,
                fission_w: 10_000.0,
                solar_peak_w: 20_000.0,
                storage_capacity_wh: 100_000.0,
                essential_load_w: 2_000.0,
            },
            base_station: BaseStation {
                position: GridCoord { x: 0, y: 0 },
                range_m: 1000.0,
            },
            policy: Policy {
                policy_version: 1,
                default_level: AuthorityLevel::AUTONOMOUS,
                levels: Default::default(),
                level3: Level3Rule {
                    max_wait_s: 60.0,
                    default_fallback: Fallback::Abort,
                },
            },
        }
    }

    #[test]
    fn capabilities_rates_and_costs() {
        use CapabilityName::*;
        let mut reg = Registry::new(&scenario(vec![
            robot(
                "a",
                0.5,
                vec![
                    cap(Dig, 0.01, 100.0, None),
                    cap(Haul, 2.0, 0.01, Some(200.0)),
                ],
            ),
            robot(
                "b",
                0.25,
                vec![
                    cap(Haul, 1.0, 0.01, None),
                    cap(Charge, 0.5, 0.0, Some(800.0)),
                ],
            ),
        ]));
        assert!(reg.can(0, Dig) && reg.can(0, Haul));
        assert!(!reg.can(1, Haul), "haul without capacity is unusable");
        assert!(reg.can(1, Charge));
        assert_eq!(reg.capable(Haul), vec![0]);
        assert_eq!(reg.haul_capacity_kg(0), 200.0);
        assert_eq!(reg.charger_power_w(1), 1800.0);
        assert_eq!(reg.tank_capacity_wh(1), 800.0);
        assert_eq!(reg.best_slot_power_w(), 3000.0);

        let (a, b) = (GridCoord { x: 0, y: 0 }, GridCoord { x: 3, y: 4 });
        // 5 cells × 2 m at 0.5 m/s.
        assert_eq!(reg.travel_s(0, a, b), 20.0);
        assert!((reg.travel_wh(0, a, b) - (10.0 * 0.5 + 20.0 * 36.0 / 3600.0)).abs() < 1e-12);
        assert_eq!(reg.work_s(0, Dig, 0.02, 5.0), 5.0);
        assert_eq!(reg.work_s(0, Dig, 0.1, 5.0), 10.0);
        assert!((reg.work_wh(0, Dig, 0.1, 5.0) - (10.0 + 0.1)).abs() < 1e-12);

        reg.disable(0, Dig);
        assert!(!reg.can(0, Dig));
        assert_eq!(reg.rate(0, Dig), 0.0);
        assert!(reg.work_s(0, Dig, 1.0, 5.0).is_infinite());
    }

    #[test]
    fn performance_factors_flow_into_every_cost() {
        use CapabilityName::*;
        let mut reg = Registry::new(&scenario(vec![robot(
            "a",
            0.5,
            vec![
                cap(Dig, 0.01, 100.0, None),
                cap(Scan, 4.0, 0.05, None),
                cap(Charge, 0.5, 0.0, Some(800.0)),
            ],
        )]));
        let (a, b) = (GridCoord { x: 0, y: 0 }, GridCoord { x: 3, y: 4 });
        let base_s = reg.travel_s(0, a, b);
        let base_wh = reg.travel_wh(0, a, b);
        let base_dig = reg.work_wh(0, Dig, 0.1, 5.0);
        reg.set_perf(
            0,
            Perf {
                speed: 0.5,
                drive_energy: 1.5,
                work_rate: 0.5,
                work_energy: 2.0,
                scan_rate: 0.25,
                energy: 1.2,
            },
        );
        assert_eq!(reg.travel_s(0, a, b), 2.0 * base_s);
        assert!(reg.travel_wh(0, a, b) > base_wh * 1.5);
        assert_eq!(reg.rate(0, Dig), 0.005);
        assert_eq!(reg.rate(0, Scan), 1.0);
        assert_eq!(reg.spec_rate(0, Dig), 0.01);
        assert!(reg.work_wh(0, Dig, 0.1, 5.0) > 2.0 * base_dig);
        assert_eq!(reg.charger_power_w(0), 1800.0, "charging is not affected");
        // Tows are slower and hungrier than driving alone.
        assert_eq!(reg.tow_s(0, a, b, 0.5), 2.0 * reg.travel_s(0, a, b));
        assert!(reg.tow_wh(0, a, b, 0.5, 2.0) > reg.travel_wh(0, a, b));
        // Nonsense factors are clamped.
        reg.set_perf(
            0,
            Perf {
                speed: 0.0,
                energy: f64::NAN,
                ..Perf::default()
            },
        );
        assert_eq!(reg.perf(0).speed, 0.2);
        assert_eq!(reg.perf(0).energy, 1.0);
    }
}
