//! Energy policy: reserves, charging decisions, station contention and the
//! mobile charger.
//!
//! The simulator draws a step's energy when the step starts, and a step the
//! battery cannot cover drains the robot and leaves it offline. So every task
//! is checked before it is issued: the battery must cover the task (with a
//! safety factor) *and* the trip from the task's end to the nearest charger,
//! plus a margin for telemetry noise. Charging targets the energy the robot's
//! remaining planned work needs rather than always filling up, which avoids
//! wasted time at the plug on the last charge.

use crate::config::Config;
use crate::geom;
use crate::registry::Registry;
use crate::world::WorldModel;
use arcadia_protocol::GridCoord;
use std::collections::BTreeMap;

/// Where a robot charges.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ChargeSource {
    /// Station by index.
    Station(usize),
    /// Mobile charger robot by index.
    Mobile(usize),
}

/// A planned charge.
#[derive(Debug, Clone, PartialEq)]
pub struct ChargePlan {
    /// Source.
    pub source: ChargeSource,
    /// Station or charger id for the command.
    pub source_id: String,
    /// Battery level to stop at, Wh.
    pub until_wh: f64,
    /// Where the robot charges.
    pub at: GridCoord,
    /// Expected time away from work, s.
    pub est_s: f64,
    /// Expected energy delivered, Wh.
    pub energy_wh: f64,
    /// Keep charging after the battery is full to refill the robot's own
    /// mobile-charger tank (sent without `until_wh`).
    pub fill_tank: bool,
}

/// A mobile charger whose tank falls below this fraction refills when idle.
const TANK_REFILL_FRAC: f64 = 0.5;

/// Robots already using or heading to each station, with expected end times.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct StationLoad {
    /// Expected end time of every robot charging or queued, per station.
    pub ends: BTreeMap<usize, Vec<f64>>,
}

impl StationLoad {
    /// Expected wait for a slot at station `s` on arrival at `arrive_t`, s.
    pub fn wait_s(&self, s: usize, slots: u32, arrive_t: f64) -> f64 {
        let mut ends: Vec<f64> = self.ends.get(&s).cloned().unwrap_or_default();
        let slots = slots.max(1) as usize;
        if ends.len() < slots {
            return 0.0;
        }
        ends.sort_by(f64::total_cmp);
        // The robot gets the slot freed by the (n - slots + 1)-th finisher.
        (ends[ends.len() - slots] - arrive_t).max(0.0)
    }

    /// Note a robot charging at `s` until `end_t`.
    pub fn add(&mut self, s: usize, end_t: f64) {
        self.ends.entry(s).or_default().push(end_t);
    }
}

/// Energy bookkeeping across decisions.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Energy {
    tank_wh: BTreeMap<usize, f64>,
    /// Mobile chargers reserved for a client: charger → client.
    pub reserved: BTreeMap<usize, usize>,
}

impl Energy {
    /// Start with every mobile charger's tank full.
    pub fn new(reg: &Registry) -> Self {
        let tank_wh = (0..reg.len())
            .filter(|&r| reg.tank_capacity_wh(r) > 0.0)
            .map(|r| (r, reg.tank_capacity_wh(r)))
            .collect();
        Self {
            tank_wh,
            reserved: BTreeMap::new(),
        }
    }

    /// Battery margin kept for telemetry noise and estimate error, Wh.
    pub fn margin_wh(reg: &Registry, r: usize, cfg: &Config) -> f64 {
        (cfg.energy_margin_frac * reg.robot(r).battery_wh).max(cfg.energy_margin_min_wh)
    }

    /// Energy to keep in hand at `at`: the trip to the nearest station plus
    /// the margin. Without stations only the margin applies.
    pub fn reserve_wh(reg: &Registry, r: usize, at: GridCoord, cfg: &Config) -> f64 {
        let trip = reg
            .stations()
            .iter()
            .map(|s| reg.travel_wh(r, at, s.position))
            .fold(f64::INFINITY, f64::min);
        let trip = if trip.is_finite() { trip } else { 0.0 };
        trip * cfg.reserve_factor + Self::margin_wh(reg, r, cfg)
    }

    /// Whether robot `r` can afford a task costing `cost_wh` that ends at `end`.
    pub fn affordable(
        world: &WorldModel,
        reg: &Registry,
        r: usize,
        cost_wh: f64,
        end: GridCoord,
        cfg: &Config,
    ) -> bool {
        let battery = world.fleet.get(r).battery_wh();
        cost_wh.is_finite()
            && battery - cost_wh * cfg.estimate_factor >= Self::reserve_wh(reg, r, end, cfg)
    }

    /// Whether robot `r` is below its low-water mark where it stands.
    pub fn low(world: &WorldModel, reg: &Registry, r: usize, cfg: &Config) -> bool {
        let e = world.fleet.get(r);
        let Some(pos) = e.position() else {
            return false;
        };
        e.battery_wh() < Self::reserve_wh(reg, r, pos, cfg) + Self::margin_wh(reg, r, cfg)
    }

    /// Estimated energy left in mobile charger `c`'s tank, Wh.
    pub fn tank(&self, c: usize) -> f64 {
        self.tank_wh.get(&c).copied().unwrap_or(0.0)
    }

    /// Record energy a mobile charger handed out.
    pub fn record_delivery(&mut self, c: usize, wh: f64) {
        if let Some(t) = self.tank_wh.get_mut(&c) {
            *t = (*t - wh.max(0.0)).max(0.0);
        }
    }

    /// A charger topped up its tank at a station.
    pub fn refill(&mut self, reg: &Registry, c: usize) {
        if let Some(t) = self.tank_wh.get_mut(&c) {
            *t = reg.tank_capacity_wh(c);
        }
    }

    /// Battery level to charge robot `r` to, given the energy its remaining
    /// planned work needs (`need_wh`, if known): enough for that work, the
    /// drive back from the charger and the reserve, never less than a
    /// worthwhile top-up and never more than full.
    pub fn charge_target(
        world: &WorldModel,
        reg: &Registry,
        r: usize,
        need_wh: Option<f64>,
        cfg: &Config,
    ) -> f64 {
        let e = world.fleet.get(r);
        let cap = reg.robot(r).battery_wh;
        let battery = e.battery_wh();
        let pos = e.position().unwrap_or(GridCoord { x: 0, y: 0 });
        let wanted = match need_wh {
            Some(need) if need.is_finite() => {
                let trip_back = reg
                    .stations()
                    .iter()
                    .map(|s| reg.travel_wh(r, s.position, pos))
                    .fold(f64::INFINITY, f64::min);
                let trip_back = if trip_back.is_finite() {
                    trip_back
                } else {
                    0.0
                };
                need * (1.0 + cfg.charge_headroom_frac)
                    + trip_back
                    + Self::reserve_wh(reg, r, pos, cfg)
                    + Self::margin_wh(reg, r, cfg)
            }
            _ => cap,
        };
        wanted.max(battery + cfg.min_charge_frac * cap).min(cap)
    }

    /// Plan a charge for robot `r` up to `until_wh`. Considers every station
    /// (with queueing) and every free mobile charger in `chargers`; picks the
    /// quickest the robot can reach. `None` when nothing is reachable or no
    /// charge is needed.
    pub fn plan_charge(
        &self,
        world: &WorldModel,
        reg: &Registry,
        r: usize,
        until_wh: f64,
        load: &StationLoad,
        chargers: &[usize],
    ) -> Option<ChargePlan> {
        let e = world.fleet.get(r);
        let pos = e.position()?;
        let battery = e.battery_wh();
        let cap = reg.robot(r).battery_wh;
        let until = until_wh.min(cap);
        if until <= battery + 1.0 {
            return None;
        }
        let fraction = world.charge_fraction();
        let mut best: Option<(f64, ChargePlan)> = None;
        let mut consider = |cost: f64, plan: ChargePlan| {
            if best.as_ref().is_none_or(|(c, _)| cost < *c - 1e-9) {
                best = Some((cost, plan));
            }
        };
        for (s, st) in reg.stations().iter().enumerate() {
            let trip_wh = reg.travel_wh(r, pos, st.position);
            let usable = st.slots > 0 && st.slot_power_w > 0.0;
            if !usable || !trip_wh.is_finite() || battery < trip_wh * 1.05 + 1.0 {
                continue;
            }
            let travel = reg.travel_s(r, pos, st.position);
            let wait = load.wait_s(s, st.slots, world.now + travel);
            let energy = until - (battery - trip_wh);
            let charge_s = energy / (st.slot_power_w * fraction) * 3600.0;
            let est = travel + wait + charge_s;
            // Coming back is part of the cost of choosing a far station.
            consider(
                est + travel,
                ChargePlan {
                    source: ChargeSource::Station(s),
                    source_id: st.id.clone(),
                    until_wh: until,
                    at: st.position,
                    est_s: est,
                    energy_wh: energy,
                    fill_tank: false,
                },
            );
        }
        for &c in chargers {
            let ce = world.fleet.get(c);
            let (Some(cpos), true) = (ce.position(), ce.operable()) else {
                continue;
            };
            let power = reg.charger_power_w(c);
            let trip_wh = reg.travel_wh(r, pos, cpos);
            if c == r || power <= 0.0 || !trip_wh.is_finite() || battery < trip_wh * 1.05 + 1.0 {
                continue;
            }
            let wanted = until - (battery - trip_wh);
            let energy = wanted.min(self.tank(c));
            if energy < 0.5 * wanted || energy <= 1.0 {
                continue;
            }
            let travel = reg.travel_s(r, pos, cpos);
            let est = travel + energy / power * 3600.0;
            consider(
                est + travel,
                ChargePlan {
                    source: ChargeSource::Mobile(c),
                    source_id: reg.robot(c).id.clone(),
                    until_wh: (battery - trip_wh + energy).min(cap),
                    at: cpos,
                    est_s: est,
                    energy_wh: energy,
                    fill_tank: false,
                },
            );
        }
        best.map(|(_, p)| p)
    }

    /// Plan a station visit for mobile charger `c` that fills its battery and
    /// then its tank. `None` when its tank is still well stocked or no station
    /// is reachable.
    pub fn plan_tank_refill(
        &self,
        world: &WorldModel,
        reg: &Registry,
        c: usize,
        load: &StationLoad,
    ) -> Option<ChargePlan> {
        let tank_cap = reg.tank_capacity_wh(c);
        if tank_cap <= 0.0 || self.tank(c) >= TANK_REFILL_FRAC * tank_cap {
            return None;
        }
        let e = world.fleet.get(c);
        let pos = e.position()?;
        let battery = e.battery_wh();
        let cap = reg.robot(c).battery_wh;
        let fraction = world.charge_fraction();
        let mut best: Option<(f64, ChargePlan)> = None;
        for (s, st) in reg.stations().iter().enumerate() {
            let trip_wh = reg.travel_wh(c, pos, st.position);
            let usable = st.slots > 0 && st.slot_power_w > 0.0;
            if !usable || !trip_wh.is_finite() || battery < trip_wh * 1.05 + 1.0 {
                continue;
            }
            let travel = reg.travel_s(c, pos, st.position);
            let energy = (cap - (battery - trip_wh)).max(0.0) + (tank_cap - self.tank(c));
            let est = travel
                + load.wait_s(s, st.slots, world.now + travel)
                + energy / (st.slot_power_w * fraction) * 3600.0;
            if best.as_ref().is_none_or(|(b, _)| est < *b - 1e-9) {
                best = Some((
                    est,
                    ChargePlan {
                        source: ChargeSource::Station(s),
                        source_id: st.id.clone(),
                        until_wh: cap,
                        at: st.position,
                        est_s: est,
                        energy_wh: energy,
                        fill_tank: true,
                    },
                ));
            }
        }
        best.map(|(_, p)| p)
    }

    /// The mobile charger nearest to `at` that is free to come to a stranded
    /// robot, if any.
    pub fn rescuer(
        &self,
        world: &WorldModel,
        reg: &Registry,
        free: &[usize],
        at: GridCoord,
        client: usize,
    ) -> Option<usize> {
        free.iter()
            .copied()
            .filter(|&c| c != client && reg.charger_power_w(c) > 0.0 && self.tank(c) > 1.0)
            .filter(|&c| world.fleet.get(c).operable())
            .filter_map(|c| {
                world
                    .fleet
                    .get(c)
                    .position()
                    .map(|p| (geom::dist(p, at), c))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)))
            .map(|(_, c)| c)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::tests::{cap, robot, scenario};
    use arcadia_protocol::*;

    fn state(id: &str, x: u32, y: u32, battery: f64) -> RobotState {
        RobotState {
            robot_id: id.into(),
            position: GridCoord { x, y },
            status: RobotStatus::Idle,
            battery_wh: battery,
            payload_kg: 0.0,
            current_command: None,
            health: HealthTelemetry {
                wheel_slip: 0.0,
                motor_current_ratio: 1.0,
                dust_coverage: 0.0,
                temperature_c: -50.0,
            },
            in_relay_coverage: true,
        }
    }

    fn world() -> (WorldModel, Registry) {
        let mut sc = scenario(vec![
            robot(
                "w",
                0.1,
                vec![cap(CapabilityName::Sinter, 0.01, 60.0, None)],
            ),
            robot(
                "c",
                0.5,
                vec![
                    cap(CapabilityName::Charge, 0.5, 0.0, Some(3000.0)),
                    cap(CapabilityName::Haul, 3.0, 0.01, Some(400.0)),
                ],
            ),
        ]);
        sc.site.width = 60;
        sc.site.height = 60;
        sc.site.cell_size_m = 1.0;
        let mut w = WorldModel::new(&sc);
        w.fleet
            .update(&[state("w", 40, 40, 2000.0), state("c", 41, 40, 9000.0)]);
        (w, Registry::new(&sc))
    }

    #[test]
    fn reserve_and_affordability() {
        let (w, reg) = world();
        let at = GridCoord { x: 40, y: 40 };
        let reserve = Energy::reserve_wh(&reg, 0, at, &Config::default());
        let trip = reg.travel_wh(0, at, GridCoord { x: 0, y: 0 });
        // Trip to the station with its safety factor, plus 2.5% of 5 kWh.
        assert!((reserve - (trip * 1.25 + 125.0)).abs() < 1e-9, "{reserve}");
        assert!(Energy::affordable(
            &w,
            &reg,
            0,
            100.0,
            at,
            &Config::default()
        ));
        assert!(!Energy::affordable(
            &w,
            &reg,
            0,
            1900.0,
            at,
            &Config::default()
        ));
        assert!(!Energy::affordable(
            &w,
            &reg,
            0,
            f64::INFINITY,
            at,
            &Config::default()
        ));
        assert!(!Energy::low(&w, &reg, 0, &Config::default()));
    }

    #[test]
    fn charge_target_covers_remaining_work_without_overfilling() {
        let (w, reg) = world();
        let cfg = Config::default();
        let full = Energy::charge_target(&w, &reg, 0, None, &cfg);
        assert_eq!(full, 5000.0);
        let small = Energy::charge_target(&w, &reg, 0, Some(100.0), &cfg);
        // At least the minimum worthwhile charge, never above capacity.
        assert!(
            small >= 2000.0 + 0.15 * 5000.0 - 1e-9 && small < full,
            "{small}"
        );
        let big = Energy::charge_target(&w, &reg, 0, Some(1e9), &cfg);
        assert_eq!(big, 5000.0);
    }

    #[test]
    fn low_tanks_are_refilled_at_a_station() {
        let (w, reg) = world();
        let mut energy = Energy::new(&reg);
        let load = StationLoad::default();
        assert!(
            energy.plan_tank_refill(&w, &reg, 1, &load).is_none(),
            "tank still full"
        );
        assert!(
            energy.plan_tank_refill(&w, &reg, 0, &load).is_none(),
            "not a charger"
        );
        energy.record_delivery(1, 2000.0);
        let plan = energy.plan_tank_refill(&w, &reg, 1, &load).unwrap();
        assert!(plan.fill_tank);
        assert_eq!(plan.source, ChargeSource::Station(0));
        assert_eq!(plan.until_wh, 5000.0);
        assert!(plan.energy_wh >= 2000.0);
    }

    #[test]
    fn picks_the_quickest_source_and_respects_queues() {
        let (w, reg) = world();
        let mut energy = Energy::new(&reg);
        assert_eq!(energy.tank(1), 3000.0);
        // The station is far away; the mobile charger is next door.
        let plan = energy
            .plan_charge(&w, &reg, 0, 3000.0, &StationLoad::default(), &[1])
            .unwrap();
        assert_eq!(plan.source, ChargeSource::Mobile(1));
        assert_eq!(plan.source_id, "c");
        // Without the charger it goes to the station.
        let plan = energy
            .plan_charge(&w, &reg, 0, 3000.0, &StationLoad::default(), &[])
            .unwrap();
        assert_eq!(plan.source, ChargeSource::Station(0));
        // An empty tank cannot help.
        energy.record_delivery(1, 3000.0);
        let plan = energy
            .plan_charge(&w, &reg, 0, 3000.0, &StationLoad::default(), &[1])
            .unwrap();
        assert_eq!(plan.source, ChargeSource::Station(0));
        energy.refill(&reg, 1);
        assert_eq!(energy.tank(1), 3000.0);
        // Nothing to do when already above the target.
        assert!(
            energy
                .plan_charge(&w, &reg, 0, 1500.0, &StationLoad::default(), &[])
                .is_none()
        );

        let mut load = StationLoad::default();
        assert_eq!(load.wait_s(0, 2, 100.0), 0.0);
        load.add(0, 500.0);
        load.add(0, 300.0);
        assert_eq!(load.wait_s(0, 2, 100.0), 200.0);
        assert_eq!(
            energy.rescuer(&w, &reg, &[1], GridCoord { x: 40, y: 40 }, 0),
            Some(1)
        );
    }
}
