//! Recovery: bringing disabled robots back to work.
//!
//! A robot can be stuck in soft ground, failed (a disabling fault), offline
//! (an empty battery), or a mix of these. Recovery is planned one step at a
//! time from the robot's current condition, like the pad plan, so a surprise
//! on the way (a tow that fails, a second fault) simply replans:
//!
//! | Condition | Steps considered |
//! |---|---|
//! | on a depot cell, stuck or failed | `service` in place |
//! | offline at a station, or beside an idle mobile charger | `charge` in place |
//! | stuck | field `repair`; `tow` to a firm cell nearby (or to a station if the battery is empty) |
//! | failed, field repairable | field `repair`; `tow` to a depot for `service` |
//! | failed, not field repairable | `tow` to a depot for `service`; `cannibalize` a donor where the policy allows it |
//! | offline | bring a mobile charger over; `tow` to a station |
//!
//! Each candidate is costed in seconds until the robot is back: the helper's
//! drive, the work or tow itself, queueing at the depot or station, and the
//! service or charge that follows. The cheapest permitted option wins; ties
//! go to the lower helper index. A helper whose battery cannot cover the step
//! and its reserve is sent to charge first.
//!
//! Every option names the policy action it uses, and the coordinator's
//! authority layer gates it through [`Ctx::may`]: levels 1 and 2 are taken
//! (level 2 with a notify), higher levels only with an approval in hand.
//! Cannibalizing (level 3 by default) is gated per donor by
//! [`Ctx::may_strip`], because an approval covers the strip it asked for;
//! [`best_cannibalize`] finds the option worth asking Earth about.

use crate::config::Config;
use crate::energy::{ChargeSource, Energy, StationLoad};
use crate::executor::TowPurpose;
use crate::geom;
use crate::health::Health;
use crate::registry::Registry;
use crate::world::WorldModel;
use arcadia_protocol::{CapabilityName, GridCoord, RobotStatus, Surface};
use std::collections::BTreeSet;

/// Robots within this many cells of each other can work on each other.
pub const REACH_CELLS: f64 = 1.5;

/// Search radius for a firm cell to tow a stuck robot onto, cells.
const FIRM_SEARCH_CELLS: i64 = 6;

/// How far a stuck robot is towed towards a station when no firm cell is known, cells.
const FREE_TOW_CELLS: f64 = 3.0;

/// Work to strip a donor and fit its parts, in repair units (interface
/// assumption: 0.5 to strip, 1.0 to install).
const CANNIBALIZE_UNITS: f64 = 1.5;

/// One step that moves a disabled robot towards working again.
#[derive(Debug, Clone, PartialEq)]
pub enum Step {
    /// The robot is on a depot cell: send it `service`.
    ServiceInPlace {
        /// Depot index.
        depot: usize,
    },
    /// The robot is offline at a charge source: send it `charge`.
    ChargeInPlace {
        /// The source it is at.
        source: ChargeSource,
    },
    /// A repairer drives over and repairs it.
    Repair {
        /// The repairing robot.
        repairer: usize,
    },
    /// A tower tows it.
    Tow {
        /// The towing robot.
        tower: usize,
        /// Destination.
        to: GridCoord,
        /// What the robot needs there.
        purpose: TowPurpose,
    },
    /// A mobile charger drives over so the robot can charge from it.
    BringCharger {
        /// The mobile charger.
        charger: usize,
    },
    /// A repairer strips a donor for parts and fits them to the robot.
    Cannibalize {
        /// The repairing robot.
        executor: usize,
        /// The robot stripped (lost afterwards).
        donor: usize,
    },
}

impl Step {
    /// The policy action this step uses, for the authority gate.
    pub fn action(&self) -> &'static str {
        match self {
            Step::ServiceInPlace { .. } => "service",
            Step::ChargeInPlace { .. } => "charge",
            Step::Repair { .. } => "repair",
            Step::Tow { .. } => "tow",
            Step::BringCharger { .. } => "move_to",
            Step::Cannibalize { .. } => "cannibalize",
        }
    }

    /// The robot that carries the step out, when it is not the robot itself.
    pub fn helper(&self) -> Option<usize> {
        match self {
            Step::ServiceInPlace { .. } | Step::ChargeInPlace { .. } => None,
            Step::Repair { repairer } => Some(*repairer),
            Step::Tow { tower, .. } => Some(*tower),
            Step::BringCharger { charger } => Some(*charger),
            Step::Cannibalize { executor, .. } => Some(*executor),
        }
    }
}

/// A costed recovery step.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    /// The robot being recovered.
    pub target: usize,
    /// What to do next.
    pub step: Step,
    /// Expected time until the robot is back, s.
    pub cost_s: f64,
    /// Battery the helper needs for the step and its reserve afterwards, Wh.
    pub helper_need_wh: f64,
    /// Whether the helper's battery covers that now.
    pub affordable: bool,
}

/// Everything recovery planning reads.
#[derive(Clone, Copy)]
pub struct Ctx<'a> {
    /// World model.
    pub world: &'a WorldModel,
    /// Capability registry.
    pub reg: &'a Registry,
    /// Energy bookkeeping (mobile charger tanks and reservations).
    pub energy: &'a Energy,
    /// Health estimates (repair work).
    pub health: &'a Health,
    /// Tuning.
    pub cfg: &'a Config,
    /// Station queues.
    pub load: &'a StationLoad,
    /// Expected wait at each depot for an arrival at a given time, s.
    pub depot_wait: &'a dyn Fn(usize, f64) -> f64,
    /// Expected service duration, s.
    pub service_s: f64,
    /// Robots engaged by commands in flight.
    pub engaged: &'a BTreeSet<usize>,
    /// Robots with a command in flight.
    pub running: &'a BTreeSet<usize>,
    /// Actions ArcadiaOS may take now without asking Earth.
    pub may: &'a dyn Fn(&str) -> bool,
    /// Donors the robot being recovered may now be given parts from.
    pub may_strip: &'a dyn Fn(usize) -> bool,
}

impl Ctx<'_> {
    fn pos(&self, r: usize) -> Option<GridCoord> {
        self.world.fleet.get(r).position()
    }

    fn battery(&self, r: usize) -> f64 {
        self.world.fleet.get(r).battery_wh()
    }

    /// Battery a helper needs for a step costing `wh` that ends at `end`.
    fn need(&self, r: usize, wh: f64, end: GridCoord) -> f64 {
        wh * self.cfg.estimate_factor + Energy::reserve_wh(self.reg, r, end, self.cfg)
    }

    /// Extra time for a helper to charge up to `need` first, s.
    fn charge_first_s(&self, r: usize, need: f64) -> f64 {
        let short = need - self.battery(r);
        if short <= 0.0 {
            return 0.0;
        }
        let power = self.reg.best_slot_power_w().max(1.0);
        short / power * 3600.0 + 1800.0
    }

    /// A station's index if `at` is its cell and it can charge.
    fn station_at(&self, at: GridCoord) -> Option<usize> {
        self.reg
            .stations()
            .iter()
            .position(|s| s.position == at && s.slots > 0 && s.slot_power_w > 0.0)
    }

    /// A depot's index if `at` is its cell.
    fn depot_at(&self, at: GridCoord) -> Option<usize> {
        self.world.depots.iter().position(|d| d.position == at)
    }

    /// An idle mobile charger within reach of `at` that `client` may use.
    fn charger_beside(&self, at: GridCoord, client: usize) -> Option<usize> {
        (0..self.reg.len())
            .filter(|&c| c != client && self.reg.charger_power_w(c) > 0.0)
            .filter(|&c| self.energy.tank(c) > 1.0)
            .filter(|&c| self.world.fleet.get(c).operable())
            .filter(|&c| !self.running.contains(&c) && !self.engaged.contains(&c))
            .filter(|&c| self.energy.reserved.get(&c).is_none_or(|&k| k == client))
            .find(|&c| {
                self.pos(c)
                    .is_some_and(|p| geom::dist(p, at) <= REACH_CELLS)
            })
    }
}

/// Energy a recovered robot should be charged to, Wh: full, or what a mobile
/// charger's tank allows.
pub fn recovery_charge_wh(ctx: &Ctx, target: usize, source: ChargeSource) -> f64 {
    let cap = ctx.reg.robot(target).battery_wh;
    let battery = ctx.battery(target);
    match source {
        ChargeSource::Station(_) => cap,
        ChargeSource::Mobile(c) => (battery + ctx.energy.tank(c)).min(cap),
    }
}

/// Plan the next recovery step for disabled robot `target`, using robots in
/// `helpers`. `None` when the robot needs nothing, is already being helped,
/// or nothing permitted can help it now.
pub fn plan(ctx: &Ctx, target: usize, helpers: &[usize]) -> Option<Plan> {
    let world = ctx.world;
    let reg = ctx.reg;
    let cfg = ctx.cfg;
    let e = world.fleet.get(target);
    if !e.recoverable() || ctx.engaged.contains(&target) || ctx.running.contains(&target) {
        return None;
    }
    let pos = e.position()?;
    let status = e.status();
    let stuck = status == RobotStatus::Stuck;
    let offline = status == RobotStatus::Offline;
    let faulty = e.has_disabled_fault();
    let field_repairable = e.disabled_faults_field_repairable();
    let empty = e.battery_empty(Energy::margin_wh(reg, target, cfg));
    let now = world.now;

    // Steps the robot can take where it stands.
    if (stuck || faulty)
        && let Some(d) = ctx.depot_at(pos)
        && (ctx.may)("service")
    {
        return Some(Plan {
            target,
            step: Step::ServiceInPlace { depot: d },
            cost_s: (ctx.depot_wait)(d, now) + ctx.service_s,
            helper_need_wh: 0.0,
            affordable: true,
        });
    }
    if offline && (ctx.may)("charge") {
        let source = ctx
            .station_at(pos)
            .map(ChargeSource::Station)
            .or_else(|| ctx.charger_beside(pos, target).map(ChargeSource::Mobile));
        if let Some(source) = source {
            return Some(Plan {
                target,
                step: Step::ChargeInPlace { source },
                cost_s: 0.0,
                helper_need_wh: 0.0,
                affordable: true,
            });
        }
    }

    let mut best: Option<Plan> = None;
    let mut consider = |p: Plan| {
        let allowed = match p.step {
            Step::Cannibalize { donor, .. } => (ctx.may_strip)(donor),
            _ => (ctx.may)(p.step.action()),
        };
        if !allowed {
            return;
        }
        let better = match &best {
            None => true,
            // An affordable option beats one that needs a charge first.
            Some(b) if p.affordable != b.affordable => p.affordable,
            Some(b) => p.cost_s < b.cost_s - 1e-9,
        };
        if better {
            best = Some(p);
        }
    };

    // Field repair: frees a stuck robot and clears field-repairable faults.
    if stuck || (faulty && field_repairable) {
        let units = ctx.health.repair_units(world, target, cfg);
        for &r in helpers {
            if r == target || !reg.can(r, CapabilityName::Repair) {
                continue;
            }
            let Some(rpos) = ctx.pos(r) else { continue };
            let work_wh = reg.work_wh(r, CapabilityName::Repair, units, cfg.min_work_s);
            if units <= 0.0 || work_wh > cfg.repair_max_battery_frac * reg.robot(r).battery_wh {
                continue;
            }
            let travel_s = reg.travel_s(r, rpos, pos);
            let need = ctx.need(r, reg.travel_wh(r, rpos, pos) + work_wh, pos);
            let affordable = ctx.battery(r) >= need;
            let cost = travel_s
                + reg.work_s(r, CapabilityName::Repair, units, cfg.min_work_s)
                + ctx.charge_first_s(r, need);
            if cost.is_finite() {
                consider(Plan {
                    target,
                    step: Step::Repair { repairer: r },
                    cost_s: cost,
                    helper_need_wh: need,
                    affordable,
                });
            }
        }
    }

    // Towing: to a depot for service, to a station for charge, or out of soft ground.
    let mut destinations: Vec<(GridCoord, TowPurpose, f64)> = Vec::new();
    if faulty {
        for (d, depot) in world.depots.iter().enumerate() {
            let wait = (ctx.depot_wait)(d, now);
            destinations.push((depot.position, TowPurpose::Depot(d), wait + ctx.service_s));
        }
    } else if empty {
        for (s, st) in reg.stations().iter().enumerate() {
            if st.slots == 0 || st.slot_power_w <= 0.0 {
                continue;
            }
            let wh = (reg.robot(target).battery_wh - ctx.battery(target)).max(0.0);
            let charge_s = wh / (st.slot_power_w * world.charge_fraction()) * 3600.0;
            let wait = ctx.load.wait_s(s, st.slots, now);
            destinations.push((st.position, TowPurpose::Station(s), wait + charge_s));
        }
    } else if stuck {
        destinations.push((free_tow_target(world, reg, pos), TowPurpose::Free, 0.0));
    }
    for &t in helpers {
        if t == target || !reg.can(t, CapabilityName::Haul) {
            continue;
        }
        let Some(tpos) = ctx.pos(t) else { continue };
        for &(to, purpose, after_s) in &destinations {
            let tow_s = reg.tow_s(t, pos, to, cfg.tow_speed_factor);
            let wh = reg.travel_wh(t, tpos, pos)
                + reg.tow_wh(t, pos, to, cfg.tow_speed_factor, cfg.tow_energy_factor);
            let need = ctx.need(t, wh, to);
            let affordable = ctx.battery(t) >= need;
            let cost = reg.travel_s(t, tpos, pos) + tow_s + after_s + ctx.charge_first_s(t, need);
            if cost.is_finite() {
                consider(Plan {
                    target,
                    step: Step::Tow {
                        tower: t,
                        to,
                        purpose,
                    },
                    cost_s: cost,
                    helper_need_wh: need,
                    affordable,
                });
            }
        }
    }

    // A mobile charger for an offline robot (a stuck one must be freed first).
    if offline {
        for &c in helpers {
            if c == target || reg.charger_power_w(c) <= 0.0 || ctx.energy.tank(c) <= 1.0 {
                continue;
            }
            if ctx.energy.reserved.get(&c).is_some_and(|&k| k != target) {
                continue;
            }
            let Some(cpos) = ctx.pos(c) else { continue };
            let wh = recovery_charge_wh(ctx, target, ChargeSource::Mobile(c)) - ctx.battery(target);
            if wh <= 1.0 {
                continue;
            }
            let need = ctx.need(c, reg.travel_wh(c, cpos, pos), pos);
            let affordable = ctx.battery(c) >= need;
            let cost = reg.travel_s(c, cpos, pos)
                + wh / reg.charger_power_w(c) * 3600.0
                + ctx.charge_first_s(c, need);
            if cost.is_finite() {
                consider(Plan {
                    target,
                    step: Step::BringCharger { charger: c },
                    cost_s: cost,
                    helper_need_wh: need,
                    affordable,
                });
            }
        }
    }

    // Parts from a donor, where the authority layer allows that donor.
    if faulty {
        for p in cannibalize_options(ctx, target, pos, helpers) {
            consider(p);
        }
    }
    best
}

/// Every way of restoring failed robot `target` (at `pos`) with parts from a
/// donor, costed like the other options.
fn cannibalize_options(ctx: &Ctx, target: usize, pos: GridCoord, helpers: &[usize]) -> Vec<Plan> {
    let (reg, cfg) = (ctx.reg, ctx.cfg);
    let mut out = Vec::new();
    for &r in helpers {
        if r == target || !reg.can(r, CapabilityName::Repair) {
            continue;
        }
        let Some(rpos) = ctx.pos(r) else { continue };
        for donor in donors(ctx, target, r) {
            let Some(dpos) = ctx.pos(donor) else { continue };
            let units = CANNIBALIZE_UNITS;
            let wh = reg.travel_wh(r, rpos, dpos)
                + reg.travel_wh(r, dpos, pos)
                + reg.work_wh(r, CapabilityName::Repair, units, cfg.min_work_s);
            let need = ctx.need(r, wh, pos);
            let cost = reg.travel_s(r, rpos, dpos)
                + reg.travel_s(r, dpos, pos)
                + reg.work_s(r, CapabilityName::Repair, units, cfg.min_work_s)
                + ctx.charge_first_s(r, need);
            if cost.is_finite() {
                out.push(Plan {
                    target,
                    step: Step::Cannibalize { executor: r, donor },
                    cost_s: cost,
                    helper_need_wh: need,
                    affordable: ctx.battery(r) >= need,
                });
            }
        }
    }
    out
}

/// The cheapest cannibalization for disabled robot `target`, whether or not
/// it is allowed yet: what ArcadiaOS would ask Earth to approve. Affordable
/// options beat ones that need the helper to charge first.
pub fn best_cannibalize(ctx: &Ctx, target: usize, helpers: &[usize]) -> Option<Plan> {
    let e = ctx.world.fleet.get(target);
    if !e.recoverable()
        || !e.has_disabled_fault()
        || ctx.engaged.contains(&target)
        || ctx.running.contains(&target)
    {
        return None;
    }
    let pos = e.position()?;
    cannibalize_options(ctx, target, pos, helpers)
        .into_iter()
        .min_by(|a, b| {
            b.affordable
                .cmp(&a.affordable)
                .then(a.cost_s.total_cmp(&b.cost_s))
                .then(a.step.helper().cmp(&b.step.helper()))
        })
}

/// Robots that may be stripped to restore `target`: disabled (a disabling
/// fault or an empty battery), not lost or engaged, and whose every
/// capability another working robot also has.
pub fn donors(ctx: &Ctx, target: usize, executor: usize) -> Vec<usize> {
    let world = ctx.world;
    let reg = ctx.reg;
    (0..reg.len())
        .filter(|&d| d != target && d != executor)
        .filter(|&d| {
            let e = world.fleet.get(d);
            e.recoverable()
                && !ctx.engaged.contains(&d)
                && (e.has_disabled_fault() || e.battery_empty(Energy::margin_wh(reg, d, ctx.cfg)))
        })
        .filter(|&d| {
            crate::registry::WORK_CAPABILITIES
                .iter()
                .chain([CapabilityName::Repair, CapabilityName::Charge].iter())
                .filter(|&&cap| reg.can(d, cap))
                .all(|&cap| {
                    (0..reg.len())
                        .any(|o| o != d && reg.can(o, cap) && world.fleet.get(o).operable())
                })
        })
        .collect()
}

/// Where to tow a stuck robot at `at`: the nearest cell known to be firm
/// (compacted or stabilized) within a few cells, else a few cells towards
/// the nearest station.
pub fn free_tow_target(world: &WorldModel, reg: &Registry, at: GridCoord) -> GridCoord {
    let site = &world.site;
    let mut best: Option<(f64, u32, u32)> = None;
    for dy in -FIRM_SEARCH_CELLS..=FIRM_SEARCH_CELLS {
        for dx in -FIRM_SEARCH_CELLS..=FIRM_SEARCH_CELLS {
            if dx == 0 && dy == 0 {
                continue;
            }
            let (x, y) = (i64::from(at.x) + dx, i64::from(at.y) + dy);
            if x < 0 || y < 0 {
                continue;
            }
            let c = GridCoord {
                x: x as u32,
                y: y as u32,
            };
            let firm = site
                .get(c)
                .is_some_and(|k| matches!(k.surface, Surface::Compacted | Surface::Stabilized));
            if !firm {
                continue;
            }
            let d = geom::dist(at, c);
            if best.is_none_or(|(bd, by, bx)| (d, c.y, c.x) < (bd, by, bx)) {
                best = Some((d, c.y, c.x));
            }
        }
    }
    if let Some((_, y, x)) = best {
        return GridCoord { x, y };
    }
    let station = reg.stations().iter().map(|s| s.position).min_by(|a, b| {
        geom::dist(at, *a)
            .total_cmp(&geom::dist(at, *b))
            .then(a.y.cmp(&b.y))
            .then(a.x.cmp(&b.x))
    });
    let toward = match station {
        Some(s) if geom::dist(at, s) > FREE_TOW_CELLS => s,
        // Already close to the station (or none): move a few cells towards the site centre.
        _ => GridCoord {
            x: site.width / 2,
            y: site.height / 2,
        },
    };
    let d = geom::dist(at, toward);
    if d <= 0.0 {
        return GridCoord {
            x: (at.x + 2).min(site.width.saturating_sub(1)),
            y: at.y,
        };
    }
    let f = (FREE_TOW_CELLS / d).min(1.0);
    let x = f64::from(at.x) + (f64::from(toward.x) - f64::from(at.x)) * f;
    let y = f64::from(at.y) + (f64::from(toward.y) - f64::from(at.y)) * f;
    GridCoord {
        x: (x.round().max(0.0) as u32).min(site.width.saturating_sub(1)),
        y: (y.round().max(0.0) as u32).min(site.height.saturating_sub(1)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::tests::{cap, robot, scenario};
    use arcadia_protocol::*;

    fn state(id: &str, x: u32, y: u32, status: RobotStatus, battery: f64) -> RobotState {
        RobotState {
            robot_id: id.into(),
            position: GridCoord { x, y },
            status,
            battery_wh: battery,
            payload_kg: 0.0,
            current_command: None,
            health: HealthTelemetry {
                wheel_slip: 0.02,
                motor_current_ratio: 1.0,
                dust_coverage: 0.0,
                temperature_c: -50.0,
            },
            faults: vec![],
            in_relay_coverage: true,
            telemetry_t: 0.0,
        }
    }

    fn fault(component: &str, severity: Severity, field_repairable: bool) -> ActiveFault {
        ActiveFault {
            component: component.into(),
            severity,
            cause: FaultCause::Random,
            since_t: 0.0,
            field_repairable,
        }
    }

    /// A worker, a scout that repairs, and a hauler that tows and charges.
    fn setup() -> (WorldModel, Registry) {
        let mut sc = scenario(vec![
            robot(
                "worker",
                0.15,
                vec![cap(CapabilityName::Sinter, 0.01, 60.0, None)],
            ),
            robot(
                "scout",
                0.5,
                vec![
                    cap(CapabilityName::Scan, 4.0, 0.05, None),
                    cap(CapabilityName::Repair, 0.0005, 400.0, None),
                ],
            ),
            robot(
                "hauler",
                0.3,
                vec![
                    cap(CapabilityName::Haul, 3.0, 0.01, Some(400.0)),
                    cap(CapabilityName::Charge, 0.5, 0.0, Some(8000.0)),
                ],
            ),
        ]);
        sc.site.width = 60;
        sc.site.height = 60;
        sc.site.cell_size_m = 1.0;
        sc.stations[0].position = GridCoord { x: 10, y: 10 };
        sc.depots = vec![Depot {
            depot_id: "dp".into(),
            position: GridCoord { x: 8, y: 14 },
        }];
        let w = WorldModel::new(&sc);
        (w, Registry::new(&sc))
    }

    struct Fixture {
        energy: Energy,
        health: Health,
        cfg: Config,
        load: StationLoad,
        engaged: BTreeSet<usize>,
        running: BTreeSet<usize>,
    }

    impl Fixture {
        fn new(reg: &Registry) -> Self {
            Self {
                energy: Energy::new(reg),
                health: Health::new(reg.len()),
                cfg: Config::default(),
                load: StationLoad::default(),
                engaged: BTreeSet::new(),
                running: BTreeSet::new(),
            }
        }
    }

    fn no_wait(_: usize, _: f64) -> f64 {
        0.0
    }

    fn up_to_level_2(w: &WorldModel) -> impl Fn(&str) -> bool + '_ {
        |action: &str| w.policy.level_for(action).get() <= 2
    }

    fn never(_: usize) -> bool {
        false
    }

    fn ctx<'a>(
        w: &'a WorldModel,
        reg: &'a Registry,
        f: &'a Fixture,
        may: &'a dyn Fn(&str) -> bool,
    ) -> Ctx<'a> {
        Ctx {
            world: w,
            reg,
            energy: &f.energy,
            health: &f.health,
            cfg: &f.cfg,
            load: &f.load,
            depot_wait: &no_wait,
            service_s: 7200.0,
            engaged: &f.engaged,
            running: &f.running,
            may,
            may_strip: &never,
        }
    }

    #[test]
    fn a_field_repairable_failure_is_repaired_by_the_scout() {
        let (mut w, reg) = setup();
        let mut worker = state("worker", 30, 30, RobotStatus::Failed, 3000.0);
        worker.faults = vec![fault("tool", Severity::Disabled, true)];
        w.fleet.update(&[
            worker,
            state("scout", 28, 30, RobotStatus::Idle, 1400.0),
            state("hauler", 40, 40, RobotStatus::Idle, 9000.0),
        ]);
        let f = Fixture::new(&reg);
        let p = plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 0, &[1, 2]).unwrap();
        assert_eq!(p.step, Step::Repair { repairer: 1 });
        assert!(p.affordable);
        // About 2000 s of repair work plus a short drive.
        assert!(p.cost_s > 2000.0 && p.cost_s < 2200.0, "{}", p.cost_s);
        // Without the scout the hauler tows it to the depot.
        let p = plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 0, &[2]).unwrap();
        assert!(matches!(
            p.step,
            Step::Tow {
                tower: 2,
                purpose: TowPurpose::Depot(0),
                ..
            }
        ));
    }

    #[test]
    fn a_fault_that_needs_the_depot_is_towed_there_then_serviced() {
        let (mut w, reg) = setup();
        let mut worker = state("worker", 30, 30, RobotStatus::Failed, 3000.0);
        worker.faults = vec![fault("battery", Severity::Disabled, false)];
        w.fleet.update(&[
            worker.clone(),
            state("scout", 28, 30, RobotStatus::Idle, 1400.0),
            state("hauler", 40, 40, RobotStatus::Idle, 9000.0),
        ]);
        let f = Fixture::new(&reg);
        let p = plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 0, &[1, 2]).unwrap();
        assert_eq!(
            p.step,
            Step::Tow {
                tower: 2,
                to: GridCoord { x: 8, y: 14 },
                purpose: TowPurpose::Depot(0)
            }
        );
        assert_eq!(p.step.action(), "tow");
        // Once on the depot cell it is serviced where it stands.
        worker.position = GridCoord { x: 8, y: 14 };
        w.fleet.update(&[worker]);
        let p = plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 0, &[1, 2]).unwrap();
        assert_eq!(p.step, Step::ServiceInPlace { depot: 0 });
        assert_eq!(p.step.helper(), None);
    }

    #[test]
    fn stuck_robots_are_freed_and_offline_ones_recharged() {
        let (mut w, reg) = setup();
        w.fleet.update(&[
            state("worker", 30, 30, RobotStatus::Stuck, 3000.0),
            state("scout", 50, 50, RobotStatus::Idle, 1400.0),
            state("hauler", 31, 30, RobotStatus::Idle, 9000.0),
        ]);
        let f = Fixture::new(&reg);
        // The hauler is next door: a short tow beats the scout's long repair.
        let p = plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 0, &[1, 2]).unwrap();
        assert!(
            matches!(
                p.step,
                Step::Tow {
                    tower: 2,
                    purpose: TowPurpose::Free,
                    ..
                }
            ),
            "{p:?}"
        );

        // Offline with the charger beside it: charge in place.
        w.fleet
            .update(&[state("worker", 30, 30, RobotStatus::Offline, 0.0)]);
        let p = plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 0, &[1]).unwrap();
        assert_eq!(
            p.step,
            Step::ChargeInPlace {
                source: ChargeSource::Mobile(2)
            }
        );
        // Charger far away: bring it over.
        w.fleet
            .update(&[state("hauler", 50, 10, RobotStatus::Idle, 9000.0)]);
        let p = plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 0, &[1, 2]).unwrap();
        assert!(
            matches!(
                p.step,
                Step::BringCharger { charger: 2 } | Step::Tow { tower: 2, .. }
            ),
            "{p:?}"
        );
        // At the station: charge there.
        w.fleet
            .update(&[state("worker", 10, 10, RobotStatus::Offline, 0.0)]);
        let p = plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 0, &[]).unwrap();
        assert_eq!(
            p.step,
            Step::ChargeInPlace {
                source: ChargeSource::Station(0)
            }
        );
    }

    #[test]
    fn engaged_or_running_robots_are_left_alone_and_helpers_charge_first() {
        let (mut w, reg) = setup();
        let mut worker = state("worker", 30, 30, RobotStatus::Failed, 3000.0);
        worker.faults = vec![fault("joints", Severity::Disabled, true)];
        w.fleet.update(&[
            worker,
            state("scout", 28, 30, RobotStatus::Idle, 200.0),
            state("hauler", 40, 40, RobotStatus::Idle, 9000.0),
        ]);
        let mut f = Fixture::new(&reg);
        // The scout cannot afford the repair now; the hauler can tow.
        let p = plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 0, &[1, 2]).unwrap();
        assert!(matches!(p.step, Step::Tow { .. }), "{p:?}");
        let p = plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 0, &[1]).unwrap();
        assert_eq!(p.step, Step::Repair { repairer: 1 });
        assert!(!p.affordable && p.helper_need_wh > 200.0);
        f.engaged.insert(0);
        assert!(plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 0, &[1, 2]).is_none());
        // Healthy robots need nothing.
        assert!(plan(&ctx(&w, &reg, &f, &up_to_level_2(&w)), 2, &[1]).is_none());
    }

    #[test]
    fn cannibalizing_needs_a_redundant_donor_and_the_authority_layer() {
        let (mut w, reg) = setup();
        let mut worker = state("worker", 30, 30, RobotStatus::Failed, 3000.0);
        worker.faults = vec![fault("tool", Severity::Disabled, false)];
        let mut hauler = state("hauler", 40, 40, RobotStatus::Offline, 0.0);
        hauler.faults = vec![fault("wheels", Severity::Disabled, false)];
        w.fleet.update(&[
            worker,
            state("scout", 28, 30, RobotStatus::Idle, 1400.0),
            hauler,
        ]);
        w.depots.clear();
        let f = Fixture::new(&reg);
        // The hauler is the only tower and charger, so it is no donor.
        let may = up_to_level_2(&w);
        assert!(donors(&ctx(&w, &reg, &f, &may), 0, 1).is_empty());
        assert!(best_cannibalize(&ctx(&w, &reg, &f, &may), 0, &[1]).is_none());
        assert!(plan(&ctx(&w, &reg, &f, &may), 0, &[1]).is_none());
    }

    #[test]
    fn a_redundant_donor_is_used_only_once_the_strip_is_allowed() {
        // A second sinterer, failed beyond field repair, is a redundant donor
        // while a third one works.
        let sinterer = |id: &str| {
            robot(
                id,
                0.15,
                vec![cap(CapabilityName::Sinter, 0.01, 60.0, None)],
            )
        };
        let mut sc = scenario(vec![
            sinterer("worker"),
            robot(
                "scout",
                0.5,
                vec![
                    cap(CapabilityName::Scan, 4.0, 0.05, None),
                    cap(CapabilityName::Repair, 0.0005, 400.0, None),
                ],
            ),
            sinterer("spare"),
            sinterer("other"),
        ]);
        sc.site.width = 60;
        sc.site.height = 60;
        let mut w = WorldModel::new(&sc);
        let reg = Registry::new(&sc);
        let mut worker = state("worker", 30, 30, RobotStatus::Failed, 3000.0);
        worker.faults = vec![fault("tool", Severity::Disabled, false)];
        let mut spare = state("spare", 31, 31, RobotStatus::Failed, 3000.0);
        spare.faults = vec![fault("wheels", Severity::Disabled, false)];
        w.fleet.update(&[
            worker,
            state("scout", 29, 30, RobotStatus::Idle, 1400.0),
            spare,
            state("other", 10, 10, RobotStatus::Idle, 3000.0),
        ]);
        let f = Fixture::new(&reg);
        let may = up_to_level_2(&w);
        // No depot and no tower: stripping the spare is the only way back,
        // but it is not allowed yet, so nothing is planned...
        let c = ctx(&w, &reg, &f, &may);
        assert!(plan(&c, 0, &[1]).is_none());
        // ... though it is what to ask Earth about.
        let ask = best_cannibalize(&c, 0, &[1]).unwrap();
        assert_eq!(
            ask.step,
            Step::Cannibalize {
                executor: 1,
                donor: 2
            }
        );
        assert_eq!(ask.step.action(), "cannibalize");
        // Once that strip is allowed it is planned.
        let spare_only = |d: usize| d == 2;
        let c = Ctx {
            may_strip: &spare_only,
            ..ctx(&w, &reg, &f, &may)
        };
        assert_eq!(plan(&c, 0, &[1]).unwrap().step, ask.step);
    }

    #[test]
    fn stuck_robots_are_towed_onto_firm_ground() {
        let (mut w, reg) = setup();
        let at = GridCoord { x: 30, y: 30 };
        // Nothing firm known: a few cells towards the station.
        let to = free_tow_target(&w, &reg, at);
        assert!(
            geom::dist(at, to) <= 3.5 && to.x < 30 && to.y < 30,
            "{to:?}"
        );
        // A compacted cell nearby wins.
        let c = GridCoord { x: 32, y: 31 };
        w.site.get_mut(c).unwrap().surface = Surface::Compacted;
        assert_eq!(free_tow_target(&w, &reg, at), c);
    }
}
