//! Capability registry: which robot can do what, how fast, and at what
//! energy cost, plus the travel and work cost models built on those specs.
//!
//! Everything is derived from the robot specs in `hello`, so new robot designs
//! join without code changes. A capability is usable when it has a positive
//! rate (and, for hauling and charging, a positive capacity); a rejection for
//! a missing capability disables it for that robot.

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
        Self {
            cell_size_m: if cell.is_finite() && cell > 0.0 {
                cell
            } else {
                1.0
            },
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

    /// Work rate, units per second (0 when unable).
    pub fn rate(&self, r: usize, cap: CapabilityName) -> f64 {
        if !self.can(r, cap) {
            return 0.0;
        }
        self.spec(r, cap).map_or(0.0, |s| finite_pos(s.rate))
    }

    /// Energy per work unit, Wh.
    pub fn energy_per_unit(&self, r: usize, cap: CapabilityName) -> f64 {
        self.spec(r, cap)
            .map_or(0.0, |s| finite_pos(s.energy_wh_per_unit))
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
        self.rate(r, CapabilityName::Charge) * 3600.0
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
        let speed = self.robots[r].speed_m_s;
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
        let rc = &self.robots[r];
        let d = geom::dist(a, b) * self.cell_size_m;
        let t = self.travel_s(r, a, b);
        if t.is_finite() {
            d * rc.drive_wh_per_m + t * rc.idle_w / 3600.0
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
        units.max(0.0) * self.energy_per_unit(r, cap) + t * self.robots[r].idle_w / 3600.0
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
}
