//! The coordinator: one decision per observation.
//!
//! Each observation is folded into the world model (robot states, then events
//! in order: command outcomes, scan results, faults, environment reports,
//! Earth messages), the health estimates and the power forecast. Then the
//! decision is built:
//!
//! 1. refresh the capacity plan when the work or the fleet has changed;
//! 2. derive the ready tasks from the plan (dependency order per cell);
//! 3. recover disabled robots: service or charge in place, field repair,
//!    tow, or a mobile charger brought over ([`crate::recovery`]);
//! 4. maintenance so pressing it cannot wait for a natural break
//!    ([`crate::health`]);
//! 5. send robots below their energy low-water mark to charge;
//! 6. run the auction over free robots (and robots on interruptible jobs);
//! 7. give leftover robots maintenance at this natural break, a charge when
//!    work waits on energy, an opportunistic or parking top-up (held back
//!    when it would drain the site bank at night or in a storm), or a move
//!    towards upcoming work;
//! 8. ask for a wake-up while anything waits on time.
//!
//! Extension points: week 6 adds an authority gate where commands are issued
//! (recovery options already name their policy action and pass through
//! [`crate::recovery::permitted`]), Earth messages in [`State::on_uplink`],
//! and coverage-aware planning.

use crate::allocator::{self, AuctionResult, Bidder, Context};
use crate::capacity::{CapacityPlan, Workload};
use crate::config::Config;
use crate::diag::Diag;
use crate::energy::{ChargePlan, ChargeSource, Energy, StationLoad};
use crate::executor::{Assignment, Executor, Job, Outcome, PlannedUse, Sent, TowPurpose};
use crate::geom;
use crate::health::{Health, Maintenance};
use crate::planner::{Planner, SpoilKind};
use crate::power::PowerModel;
use crate::recovery::{self, Step};
use crate::registry::Registry;
use crate::tasks::{Stage, Task};
use crate::world::WorldModel;
use arcadia_protocol::{
    Action, CapabilityName, Command, Decision, EarthToSite, End, Event, EventKind, FailReason,
    GridCoord, Hello, Objective, Observation, RejectReason, RobotStatus, Uplink,
};
use std::collections::{BTreeMap, BTreeSet};

/// Capabilities whose robots wait at the work front when idle.
const FRONT_CAPABILITIES: [CapabilityName; 5] = [
    CapabilityName::Scan,
    CapabilityName::Dig,
    CapabilityName::Grade,
    CapabilityName::Compact,
    CapabilityName::Sinter,
];

/// Ready role tasks needed before an opportunistic charge is cut short.
const INTERRUPT_CHARGE_TASKS: usize = 4;

/// Consecutive rejections after which a robot rests before new work.
const REJECTION_STREAK_LIMIT: u32 = 3;

/// Back-off after a recovery or maintenance command fails or is refused, s.
const RECOVERY_BACKOFF_S: f64 = 120.0;

/// Decision-making core. The session loop in [`crate::run`] feeds it observations.
#[derive(Debug, Clone, Default)]
pub struct Coordinator {
    cfg: Config,
    diag: Diag,
    state: Option<State>,
    timing: Timing,
}

/// Wall-clock time spent deciding, for diagnostics only (never an input to a decision).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Timing {
    total_ms: f64,
    max_ms: f64,
}

/// Everything the coordinator knows once `hello` has arrived.
#[derive(Debug, Clone)]
struct State {
    world: WorldModel,
    reg: Registry,
    planner: Option<Planner>,
    capacity: CapacityPlan,
    capacity_dirty: bool,
    energy: Energy,
    exec: Executor,
    health: Health,
    power: PowerModel,
    /// Expected depot service time, s (learned from acceptances).
    service_s: f64,
    /// Active faults per robot at the last observation, to notice changes.
    fault_marks: Vec<usize>,
    same_time: (f64, u32),
    decisions: u64,
}

/// Working state of one decision.
#[derive(Debug, Clone, Default)]
struct Round {
    now: f64,
    cmds: Vec<Command>,
    /// Operable robots with nothing in flight.
    free: Vec<usize>,
    /// Robots on a job a better one may replace.
    interruptible: Vec<usize>,
    load: StationLoad,
    /// Expected end of every service under way or coming, per depot.
    depots: BTreeMap<usize, Vec<f64>>,
    workload: Workload,
    /// Station power drawn by charging ArcadiaOS has scheduled, W.
    charging_w: f64,
}

impl Round {
    fn take(&mut self, r: usize) {
        self.free.retain(|&x| x != r);
        self.interruptible.retain(|&x| x != r);
    }

    /// Expected wait at depot `d` for an arrival at `arrive_t`, s.
    fn depot_wait(&self, d: usize, capacity: u32, arrive_t: f64) -> f64 {
        let mut ends = self.depots.get(&d).cloned().unwrap_or_default();
        let capacity = capacity.max(1) as usize;
        if ends.len() < capacity {
            return 0.0;
        }
        ends.sort_by(f64::total_cmp);
        (ends[ends.len() - capacity] - arrive_t).max(0.0)
    }
}

/// Counters reported at the end of a run.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Summary {
    /// Decisions made.
    pub decisions: u64,
    /// Commands sent (cancels included).
    pub commands: u64,
    /// Commands completed.
    pub completed: u64,
    /// Commands failed (cancellations excluded).
    pub failed: u64,
    /// Commands rejected.
    pub rejected: u64,
    /// Pad cells believed finished.
    pub cells_finished: usize,
    /// Pad cells.
    pub cells_total: usize,
}

impl Coordinator {
    /// A coordinator with default tuning and diagnostics from `ARCADIAOS_LOG`.
    pub fn new() -> Self {
        Self::with_config(Config::default(), Diag::new(crate::diag::Level::from_env()))
    }

    /// A coordinator with explicit tuning and diagnostics.
    pub fn with_config(cfg: Config, diag: Diag) -> Self {
        Self {
            cfg,
            diag,
            state: None,
            timing: Timing::default(),
        }
    }

    /// Take in the scenario.
    pub fn start(&mut self, hello: Box<Hello>) {
        let world = WorldModel::new(&hello.scenario);
        let reg = Registry::new(&hello.scenario);
        self.diag.info(world.now, || {
            format!(
                "run {}: {} robots, {} stations, {} depots, site {}x{} cells of {} m, power {:?}",
                hello.run_id,
                reg.len(),
                world.stations.len(),
                world.depots.len(),
                world.site.width,
                world.site.height,
                world.site.cell_size_m,
                hello.scenario.power.mode
            )
        });
        self.state = Some(State {
            energy: Energy::new(&reg),
            health: Health::new(reg.len()),
            power: PowerModel::new(hello.scenario.power.clone(), hello.scenario.sol_length_s),
            service_s: self.cfg.service_s,
            fault_marks: vec![0; reg.len()],
            world,
            reg,
            planner: None,
            capacity: CapacityPlan::default(),
            capacity_dirty: true,
            exec: Executor::new(),
            same_time: (f64::NAN, 0),
            decisions: 0,
        });
    }

    /// Decide what to do about `obs`.
    pub fn decide(&mut self, obs: &Observation) -> Decision {
        let started = std::time::Instant::now();
        let mut decision = Decision::empty(obs.seq);
        if let Some(st) = self.state.as_mut() {
            st.observe(obs, &self.cfg, &self.diag);
            st.decisions += 1;
            if st.planner.is_some() {
                let (commands, wake_at) = st.dispatch(&self.cfg, &self.diag);
                decision.commands = commands;
                decision.wake_at = wake_at;
            }
        }
        let ms = started.elapsed().as_secs_f64() * 1000.0;
        self.timing.total_ms += ms;
        self.timing.max_ms = self.timing.max_ms.max(ms);
        decision
    }

    /// The session ended.
    pub fn finish(&mut self, end: &End) {
        let s = self.summary();
        let timing = self.timing;
        self.diag.info(end.t, || {
            format!(
                "ended {:?}: {} decisions, {} commands ({} completed, {} failed, {} rejected), pad {}/{} cells; decisions took {:.3} ms on average, {:.3} ms at most",
                end.reason,
                s.decisions,
                s.commands,
                s.completed,
                s.failed,
                s.rejected,
                s.cells_finished,
                s.cells_total,
                timing.total_ms / s.decisions.max(1) as f64,
                timing.max_ms
            )
        });
        if let Some(st) = &self.state {
            for r in 0..st.reg.len() {
                self.diag.info(end.t, || {
                    st.health.describe(&st.world, &st.reg, r, &self.cfg)
                });
            }
        }
    }

    /// Counters so far.
    pub fn summary(&self) -> Summary {
        let Some(st) = &self.state else {
            return Summary::default();
        };
        let progress = st
            .planner
            .as_ref()
            .map(|p| p.progress(&st.world, &self.cfg))
            .unwrap_or_default();
        Summary {
            decisions: st.decisions,
            commands: st.exec.stats.issued,
            completed: st.exec.stats.completed,
            failed: st.exec.stats.failed,
            rejected: st.exec.stats.rejected,
            cells_finished: progress.finished,
            cells_total: progress.pad_cells,
        }
    }

    /// The current plan, once a goal has arrived.
    pub fn planner(&self) -> Option<&Planner> {
        self.state.as_ref().and_then(|s| s.planner.as_ref())
    }

    /// The current world model, once `hello` has arrived.
    pub fn world(&self) -> Option<&WorldModel> {
        self.state.as_ref().map(|s| &s.world)
    }

    /// The current capacity plan.
    pub fn capacity(&self) -> Option<&CapacityPlan> {
        self.state.as_ref().map(|s| &s.capacity)
    }

    /// The current health estimates.
    pub fn health(&self) -> Option<&Health> {
        self.state.as_ref().map(|s| &s.health)
    }
}

/// What a command sent now to robot `r` is expected to cost and expose.
fn sent(world: &WorldModel, r: usize, est_s: f64, planned: PlannedUse) -> Sent {
    let e = world.fleet.get(r);
    Sent {
        now: world.now,
        est_s,
        battery_wh: e.battery_wh(),
        payload_kg: e.payload_kg(),
        planned,
    }
}

/// Exposure of awarded work, with the calibration its estimate carries.
fn work_use(
    reg: &Registry,
    health: &Health,
    r: usize,
    scan: bool,
    plan: &allocator::WorkPlan,
) -> PlannedUse {
    let busy = (plan.duration_s - plan.travel_s).max(0.0);
    let (energy_factor, duration_factor) = health.applied_calibration(r);
    PlannedUse {
        drive_m: plan.travel_s * reg.speed_m_s(r),
        drive_s: plan.travel_s,
        work_s: if scan { 0.0 } else { busy },
        scan_s: if scan { busy } else { 0.0 },
        energy_factor,
        duration_factor,
    }
}

/// Exposure of driving robot `r` from `a` to `b`.
fn drive_use(reg: &Registry, r: usize, a: GridCoord, b: GridCoord) -> PlannedUse {
    PlannedUse {
        drive_m: geom::dist(a, b) * reg.cell_size_m,
        drive_s: reg.travel_s(r, a, b),
        ..PlannedUse::default()
    }
}

impl State {
    fn observe(&mut self, obs: &Observation, cfg: &Config, diag: &Diag) {
        let t = obs.t;
        self.world.now = t;
        if self.same_time.0 == t {
            self.same_time.1 += 1;
        } else {
            self.same_time = (t, 1);
        }
        self.world.fleet.update(&obs.robots);
        for e in &obs.events {
            self.on_event(e, cfg, diag);
        }
        self.health.observe(&self.world, cfg);
        self.power.observe(&self.world);
        for r in 0..self.reg.len() {
            self.reg.set_perf(r, self.health.perf(&self.world, r, cfg));
            // A fault appearing or clearing changes the robot's rates: re-plan roles.
            let faults = self.world.fleet.get(r).faults().len();
            if self.fault_marks.get(r).is_some_and(|&m| m != faults) {
                self.fault_marks[r] = faults;
                self.capacity_dirty = true;
            }
        }
    }

    fn on_event(&mut self, e: &Event, cfg: &Config, diag: &Diag) {
        self.world.observe(e);
        match &e.kind {
            EventKind::EarthMessage { message, .. } => self.on_uplink(message, cfg, diag),
            EventKind::ScanResult {
                command_id, cells, ..
            } => {
                let issued = self.exec.issued_t(command_id).unwrap_or(e.t);
                self.world.site.apply_scan(cells, issued, e.t);
                if let Some(p) = self.planner.as_mut() {
                    p.after_scan(&self.world, cells, cfg);
                }
            }
            EventKind::RobotFault {
                robot_id,
                component,
                severity,
                field_repairable,
                cause,
                ..
            } => {
                diag.info(e.t, || {
                    format!(
                        "{robot_id}: {severity:?} {cause:?} fault in {component}{}",
                        if *field_repairable {
                            ""
                        } else {
                            " (depot only)"
                        }
                    )
                });
                if let Some(r) = self.world.fleet.index_of(robot_id) {
                    self.health.fault(r);
                }
            }
            _ => {
                if let EventKind::CommandAccepted {
                    command_id,
                    estimated_duration_s,
                    ..
                } = &e.kind
                    && let Some(a) = self.exec.assignment_of(command_id)
                    && matches!(a.job, Job::Service { .. })
                    && a.planned.drive_m <= 0.0
                    && *estimated_duration_s > 0.0
                {
                    // A robot already at the depot: the estimate is the service itself.
                    self.service_s = *estimated_duration_s;
                }
                if let Some((a, outcome)) = self.exec.on_event(e) {
                    self.on_outcome(a, outcome, e.t, cfg, diag);
                }
            }
        }
    }

    fn on_uplink(&mut self, message: &EarthToSite, cfg: &Config, diag: &Diag) {
        match &message.body {
            Uplink::Goal {
                goal_id,
                objective: Objective::LandingPad { pad },
                ..
            } => {
                if self.planner.is_some() {
                    diag.info(self.world.now, || format!("ignoring extra goal {goal_id}"));
                    return;
                }
                let planner = Planner::new(goal_id, pad, &self.world, &self.reg, cfg);
                diag.info(self.world.now, || {
                    format!(
                        "goal {goal_id}: pad {}x{} at ({},{}) to {:.2} m, {} sections, berm {} cells, {} stockpile sites",
                        geom::width(&planner.pad.area),
                        geom::height(&planner.pad.area),
                        planner.pad.area.min.x,
                        planner.pad.area.min.y,
                        planner.pad.target_elevation_m,
                        planner.sections.len(),
                        planner.berm.len(),
                        planner.material.piles.len()
                    )
                });
                self.planner = Some(planner);
                self.capacity_dirty = true;
            }
            Uplink::PolicyUpdate { policy } => {
                self.world.policy = policy.clone();
            }
            _ => {}
        }
    }

    fn on_outcome(&mut self, a: Assignment, outcome: Outcome, t: f64, cfg: &Config, diag: &Diag) {
        let r = a.robot;
        let id = self.reg.robot(r).id.clone();
        let name = |i: usize| self.reg.robot(i).id.clone();
        let label = match &a.job {
            Job::Repair { target, preventive } => format!(
                "repair of {}{}",
                name(*target),
                if *preventive { " (preventive)" } else { "" }
            ),
            Job::Tow { target, to, .. } => {
                format!("tow of {} to ({},{})", name(*target), to.x, to.y)
            }
            Job::Cannibalize {
                target,
                beneficiary,
            } => format!("cannibalizing {} for {}", name(*target), name(*beneficiary)),
            job => job.label(),
        };
        match &outcome {
            Outcome::Completed { .. } => {
                if matches!(
                    a.job,
                    Job::Work { .. } | Job::Charge { .. } | Job::Move { .. }
                ) {
                    diag.debug(t, || format!("{id}: done {label}"));
                } else {
                    diag.info(t, || format!("{id}: done {label}"));
                }
            }
            Outcome::Failed { reason, detail, .. } if *reason != FailReason::Cancelled => {
                diag.info(t, || {
                    format!(
                        "{id}: {label} failed: {reason:?} {}",
                        detail.as_deref().unwrap_or("")
                    )
                });
            }
            Outcome::Rejected { reason, detail } => {
                diag.info(t, || {
                    format!(
                        "{id}: {label} rejected: {reason:?} {}",
                        detail.as_deref().unwrap_or("")
                    )
                });
            }
            Outcome::Failed { .. } => {}
        }
        // Wear accounting: what the command exposed the robot to.
        let (fraction, used_wh) = match &outcome {
            Outcome::Completed { energy_used_wh } => (1.0, *energy_used_wh),
            Outcome::Failed { energy_used_wh, .. } => {
                let planned = (a.est_end_t - a.issued_t).max(1.0);
                (
                    ((t - a.issued_t) / planned).clamp(0.0, 1.0),
                    *energy_used_wh,
                )
            }
            Outcome::Rejected { .. } => (0.0, 0.0),
        };
        if fraction > 0.0 || used_wh > 0.0 {
            self.health.record_use(r, &a.planned, fraction, used_wh);
        }
        if let (Job::Work { plan, .. }, Outcome::Completed { energy_used_wh }) = (&a.job, &outcome)
        {
            // Learn against the estimate without the calibration it carried.
            let (fe, fd) = (
                a.planned.energy_factor.max(1e-9),
                a.planned.duration_factor.max(1e-9),
            );
            self.health.calibrate(
                r,
                plan.energy_wh / fe,
                *energy_used_wh,
                plan.duration_s / fd,
                t - a.issued_t,
            );
        }
        // Refused again and again, for whatever reason: give the robot a rest.
        if matches!(outcome, Outcome::Rejected { .. })
            && self.exec.rejection_streak(r) >= REJECTION_STREAK_LIMIT
        {
            self.exec.back_off_robot(r, t + cfg.retry_backoff_s / 5.0);
        }
        let done = matches!(outcome, Outcome::Completed { .. });
        let cancelled = matches!(
            outcome,
            Outcome::Failed {
                reason: FailReason::Cancelled,
                ..
            }
        );
        match a.job.clone() {
            Job::Work { task, plan } => match outcome {
                Outcome::Completed { .. } => self.apply_work(&a, &task, &plan, t, cfg, diag),
                Outcome::Failed { reason, .. } => self.work_failed(r, &task, &plan, reason, t, cfg),
                Outcome::Rejected { reason, .. } => self.work_rejected(r, &task, reason, t, cfg),
            },
            Job::Charge { plan, .. } => {
                if let ChargeSource::Mobile(c) = plan.source {
                    self.energy.reserved.remove(&c);
                    if done {
                        self.energy.record_delivery(c, plan.energy_wh);
                    }
                }
                if done && plan.fill_tank {
                    self.energy.refill(&self.reg, r);
                }
                if !done {
                    self.exec.back_off_robot(r, t + 60.0);
                }
            }
            Job::Move { rescue_for, .. } => {
                if !done {
                    if rescue_for.is_some() {
                        self.energy.reserved.remove(&r);
                    }
                    if !cancelled {
                        self.exec.back_off_robot(r, t + 60.0);
                    }
                }
            }
            Job::Repair { target, .. } => {
                if done {
                    self.health.repaired(target, cfg);
                    self.capacity_dirty = true;
                } else if !cancelled {
                    self.exec.back_off_robot(r, t + RECOVERY_BACKOFF_S);
                }
            }
            Job::Tow { .. } => {
                if !done && !cancelled {
                    self.exec.back_off_robot(r, t + RECOVERY_BACKOFF_S);
                }
            }
            Job::Service { .. } => {
                if done {
                    self.health.serviced(r);
                    self.capacity_dirty = true;
                } else if !cancelled {
                    self.exec.back_off_robot(r, t + RECOVERY_BACKOFF_S);
                }
            }
            Job::Cannibalize { beneficiary, .. } => {
                if done {
                    self.health.serviced(beneficiary);
                    self.capacity_dirty = true;
                } else if !cancelled {
                    self.exec.back_off_robot(r, t + RECOVERY_BACKOFF_S);
                }
            }
        }
    }

    fn apply_work(
        &mut self,
        a: &Assignment,
        task: &Task,
        plan: &allocator::WorkPlan,
        t: f64,
        cfg: &Config,
        diag: &Diag,
    ) {
        let Some(planner) = self.planner.as_mut() else {
            return;
        };
        let site = &mut self.world.site;
        let c = task.cell();
        match task.stage {
            Stage::Dig => {
                let spoil = site.record_dig(c, task.elevation_m, t, cfg);
                let Some(dig) = &plan.dig else { return };
                let overflow = (spoil - dig.room_kg).max(0.0);
                if overflow <= 0.0 {
                    return;
                }
                match dig.spoil {
                    Some(s) => match s.kind {
                        SpoilKind::Pile(i) => {
                            planner.material.record_spoil(i, overflow);
                            site.record_deposit(s.at, overflow, false, t, cfg);
                        }
                        SpoilKind::Berm(i) => {
                            site.record_deposit(s.at, overflow, true, t, cfg);
                            planner.record_berm_deposit(i, overflow);
                            planner.material.side_cast_kg += overflow;
                        }
                        SpoilKind::Fill => {
                            site.record_deposit(s.at, overflow, false, t, cfg);
                            // The amount is an estimate: measure before grading.
                            site.mark_stale(s.at);
                            planner.material.side_cast_kg += overflow;
                        }
                    },
                    None => planner.material.spoil_kg += overflow,
                }
            }
            Stage::Fill | Stage::Berm => {
                let Some(h) = &plan.haul else { return };
                let capacity = self.reg.haul_capacity_kg(a.robot);
                let dumped = a.payload_kg.max(h.mass_kg.min(capacity));
                let loaded = (dumped - a.payload_kg).max(0.0);
                if let Some(i) = h.pile {
                    planner.material.record_load(i, loaded);
                    site.record_removal(h.from, loaded, t, cfg);
                }
                site.record_deposit(h.to, dumped, h.berm.is_some(), t, cfg);
                if let Some(i) = h.berm {
                    planner.record_berm_deposit(i, dumped);
                }
            }
            Stage::Grade => {
                site.record_grade(c, task.elevation_m, t);
                planner.clear_rework(c);
            }
            Stage::Compact => site.record_compact(c, cfg.compact_settle_m, t),
            Stage::Sinter => site.record_sinter(c, t),
            Stage::Recheck => {
                let actions = planner.after_recheck(&self.world, cfg);
                diag.info(t, || {
                    format!(
                        "recheck round {}: {actions} corrective actions",
                        planner.recheck.rounds
                    )
                });
            }
            Stage::Survey | Stage::Verify | Stage::BermCheck => {}
        }
    }

    fn work_failed(
        &mut self,
        r: usize,
        task: &Task,
        plan: &allocator::WorkPlan,
        reason: FailReason,
        t: f64,
        cfg: &Config,
    ) {
        if reason == FailReason::Cancelled {
            return;
        }
        let haul = matches!(task.stage, Stage::Fill | Stage::Berm);
        if reason == FailReason::Blocked {
            if haul {
                // Nothing left to load there: stop counting on that pile.
                if let (Some(p), Some(h)) = (self.planner.as_mut(), plan.haul.as_ref())
                    && let Some(i) = h.pile
                    && let Some(pile) = p.material.piles.get_mut(i)
                {
                    pile.spoil_kg = 0.0;
                    pile.borrowed_kg = pile.max_borrow_kg;
                }
            } else if task.changes_ground() {
                // The cell is not in the state we believed: measure it again.
                self.world.site.mark_stale(task.cell());
                if task.stage == Stage::Grade
                    && let Some(p) = self.planner.as_mut()
                {
                    p.tighten(task.cell());
                }
            }
        }
        // A fault or getting stuck says nothing about the task: once the robot
        // is back it may retry at once, and other robots are free to take it.
        if !matches!(reason, FailReason::Fault | FailReason::Stuck) {
            self.exec.back_off(r, task.key, t + cfg.retry_backoff_s);
        }
    }

    fn work_rejected(&mut self, r: usize, task: &Task, reason: RejectReason, t: f64, cfg: &Config) {
        match reason {
            RejectReason::MissingCapability => {
                self.reg.disable(r, task.capability());
                self.capacity_dirty = true;
            }
            RejectReason::InvalidTarget => {
                if task.changes_ground() && !matches!(task.stage, Stage::Fill | Stage::Berm) {
                    self.world.site.mark_stale(task.cell());
                }
                if task.stage == Stage::Grade
                    && let Some(p) = self.planner.as_mut()
                {
                    p.tighten(task.cell());
                }
                self.exec.back_off(r, task.key, t + cfg.retry_backoff_s);
            }
            RejectReason::UnknownRobot => {
                self.world.fleet.get_mut(r).gone = true;
                self.capacity_dirty = true;
            }
            RejectReason::Busy => self.exec.back_off_robot(r, t + 30.0),
            RejectReason::RobotDisabled | RejectReason::OutOfCoverage => {
                self.exec.back_off_robot(r, t + 60.0);
            }
            // Authority enforcement arrives in week 6; until then pad work is level 1.
            RejectReason::ApprovalRequired | RejectReason::Vetoed => {
                self.exec.back_off(r, task.key, t + cfg.retry_backoff_s);
            }
        }
    }

    fn dispatch(&mut self, cfg: &Config, diag: &Diag) -> (Vec<Command>, Option<f64>) {
        let now = self.world.now;
        let next_cooldown = self.exec.prune_cooldowns(now);
        if self.same_time.1 > cfg.max_same_time_decisions {
            diag.info(now, || "many decisions at one sim time; pausing".into());
            return (Vec::new(), Some(now + 60.0));
        }
        let Some(planner) = self.planner.as_mut() else {
            return (Vec::new(), None);
        };

        // Everything looks done but the run goes on: something is off.
        let progress = planner.progress(&self.world, cfg);
        let recheck_in_flight = self
            .exec
            .all_active()
            .any(|a| matches!(&a.job, Job::Work { task, .. } if task.stage == Stage::Recheck));
        if progress.complete() && !planner.recheck.wanted && !recheck_in_flight {
            diag.info(now, || {
                "pad looks complete but the run continues; rechecking".into()
            });
            planner.request_recheck();
        }

        let n = self.reg.len();
        let available: Vec<bool> = (0..n).map(|r| self.world.fleet.get(r).operable()).collect();
        let workload = planner.workload(&self.world, &self.reg, cfg);
        if self.capacity_dirty || self.capacity.stale(&workload, &available) {
            self.capacity = CapacityPlan::solve(&workload, &self.reg, &available, cfg);
            self.capacity_dirty = false;
            diag.info(now, || describe_capacity(&self.capacity, &self.reg));
        }
        self.release_chargers(cfg);

        let mut round = self.round(workload, now);
        self.recover(&mut round, cfg, diag);
        self.maintain(&mut round, None, cfg, diag);
        let stranded = self.charge_low(&mut round, cfg, diag);
        let result = self.auction(&mut round, &stranded, cfg, diag);
        self.maintain(&mut round, Some(&result), cfg, diag);
        self.leftovers(&mut round, &result, &stranded, cfg, diag);

        let mut wake = next_cooldown.filter(|&w| w > now);
        let waiting =
            !round.free.is_empty() || (0..n).any(|r| self.world.fleet.get(r).recoverable());
        if (self.exec.idle() || waiting) && !progress.complete() {
            let idle_wake = now + cfg.idle_wake_s;
            wake = Some(wake.map_or(idle_wake, |w| w.min(idle_wake)));
        }
        (round.cmds, wake)
    }

    /// Release mobile chargers whose client no longer needs them.
    fn release_chargers(&mut self, cfg: &Config) {
        let reserved: Vec<(usize, usize)> =
            self.energy.reserved.iter().map(|(&c, &k)| (c, k)).collect();
        for (c, client) in reserved {
            let ce = self.world.fleet.get(client);
            let client_ok = ce.operable() || ce.recoverable();
            let charging = matches!(
                self.exec.active(client).map(|a| &a.job),
                Some(Job::Charge { plan, .. }) if plan.source == ChargeSource::Mobile(c)
            );
            let charger_moving = self.exec.active(c).is_some();
            let wanted = ce.status() == RobotStatus::Offline
                || Energy::low(&self.world, &self.reg, client, cfg);
            if !client_ok
                || !self.world.fleet.get(c).operable()
                || (!charging && !charger_moving && !wanted)
            {
                self.energy.reserved.remove(&c);
            }
        }
    }

    /// Set up the working state of a decision.
    fn round(&self, workload: Workload, now: f64) -> Round {
        let mut round = Round {
            now,
            workload,
            ..Round::default()
        };
        for a in self.exec.all_active() {
            match &a.job {
                Job::Charge { plan, .. } => {
                    if let ChargeSource::Station(s) = plan.source {
                        round.load.add(s, a.est_end_t);
                        round.charging_w += self.reg.stations()[s].slot_power_w;
                    }
                }
                Job::Service { depot, .. } => {
                    round.depots.entry(*depot).or_default().push(a.est_end_t);
                }
                Job::Tow {
                    purpose: TowPurpose::Depot(d),
                    ..
                } => {
                    round
                        .depots
                        .entry(*d)
                        .or_default()
                        .push(a.est_end_t + self.service_s);
                }
                _ => {}
            }
        }
        let engaged = self.exec.engaged();
        for r in 0..self.reg.len() {
            let e = self.world.fleet.get(r);
            if !e.operable()
                || self.exec.robot_cooling(r, now)
                || self.energy.reserved.contains_key(&r)
                || engaged.contains(&r)
            {
                continue;
            }
            // A command this coordinator does not know about is still running.
            let unknown_command = e
                .state
                .as_ref()
                .is_some_and(|s| s.current_command.is_some());
            match self.exec.active(r) {
                None if unknown_command => {}
                None => round.free.push(r),
                Some(a) if a.interruptible() => round.interruptible.push(r),
                Some(_) => {}
            }
        }
        round
    }

    /// Whether robot `r` could do any remaining work, were it working.
    fn has_work(&self, r: usize, workload: &Workload) -> bool {
        crate::registry::WORK_CAPABILITIES
            .iter()
            .any(|&cap| self.reg.can(r, cap) && workload.get(&cap).is_some_and(|w| w.steps > 0.0))
    }

    /// Cancel robot `r`'s interruptible job so it can take another.
    fn free_up(&mut self, round: &mut Round, r: usize, diag: &Diag) {
        if self.exec.active(r).is_some_and(Assignment::interruptible)
            && let Some((cancel, old)) = self.exec.cancel(r, &self.reg.robot(r).id)
        {
            if let Job::Charge { plan, .. } = &old.job
                && let ChargeSource::Mobile(c) = plan.source
            {
                self.energy.reserved.remove(&c);
            }
            if let Job::Move {
                rescue_for: Some(_),
                ..
            } = &old.job
            {
                self.energy.reserved.remove(&r);
            }
            diag.debug(round.now, || {
                format!(
                    "{}: cancel {} for recovery",
                    self.reg.robot(r).id,
                    old.job.label()
                )
            });
            round.cmds.push(cancel);
        }
    }

    /// Step 3: bring disabled robots back, most needed first.
    fn recover(&mut self, round: &mut Round, cfg: &Config, diag: &Diag) {
        let now = round.now;
        let n = self.reg.len();
        let mut targets: Vec<(bool, usize)> = (0..n)
            .filter(|&r| self.world.fleet.get(r).recoverable())
            .filter(|&r| !self.exec.robot_cooling(r, now))
            .map(|r| (self.has_work(r, &round.workload), r))
            .collect();
        // Robots whose capabilities still have work come first.
        targets.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
        for (_, x) in targets {
            let engaged = self.exec.engaged();
            let running: BTreeSet<usize> = self.exec.all_active().map(|a| a.robot).collect();
            let awaited = self.energy.reserved.values().any(|&k| k == x);
            let mut helpers: Vec<usize> = round
                .free
                .iter()
                .chain(round.interruptible.iter())
                .copied()
                .filter(|&h| h != x)
                .collect();
            helpers.sort_unstable();
            helpers.dedup();
            let capacity = cfg.depot_capacity;
            let depot_wait = |d: usize, t: f64| round.depot_wait(d, capacity, t);
            let plan = {
                let ctx = recovery::Ctx {
                    world: &self.world,
                    reg: &self.reg,
                    energy: &self.energy,
                    health: &self.health,
                    cfg,
                    load: &round.load,
                    depot_wait: &depot_wait,
                    service_s: self.service_s,
                    engaged: &engaged,
                    running: &running,
                };
                recovery::plan(&ctx, x, &helpers)
            };
            let Some(plan) = plan else { continue };
            // A mobile charger is on its way: wait for it.
            if awaited && plan.step.helper().is_some() {
                continue;
            }
            if !plan.affordable {
                if let Some(h) = plan.step.helper()
                    && round.free.contains(&h)
                {
                    self.charge_for(round, h, plan.helper_need_wh, cfg, diag);
                }
                continue;
            }
            self.issue_recovery(round, plan, cfg, diag);
        }
    }

    /// Send helper `h` to charge up to `need_wh` (and a little more).
    fn charge_for(&mut self, round: &mut Round, h: usize, need_wh: f64, cfg: &Config, diag: &Diag) {
        let cap = self.reg.robot(h).battery_wh;
        let until = (need_wh + cfg.min_charge_frac * cap).min(cap);
        let chargers = chargers_for(h, &round.free, &self.energy, &self.exec, &self.reg);
        if let Some(plan) =
            self.energy
                .plan_charge(&self.world, &self.reg, h, until, &round.load, &chargers)
        {
            issue_charge(
                &self.world,
                &self.reg,
                &mut self.energy,
                &mut self.exec,
                round,
                h,
                plan,
                false,
                diag,
            );
        }
    }

    fn issue_recovery(
        &mut self,
        round: &mut Round,
        plan: recovery::Plan,
        cfg: &Config,
        diag: &Diag,
    ) {
        let x = plan.target;
        let now = round.now;
        let Some(xpos) = self.world.fleet.get(x).position() else {
            return;
        };
        let xid = self.reg.robot(x).id.clone();
        if let Some(h) = plan.step.helper() {
            self.free_up(round, h, diag);
        }
        diag.info(now, || {
            let who = plan
                .step
                .helper()
                .map(|h| format!("{} ", self.reg.robot(h).id))
                .unwrap_or_default();
            format!(
                "recover {xid} ({:?}): {who}{:?}, about {:.0} s",
                self.world.fleet.get(x).status(),
                plan.step,
                plan.cost_s
            )
        });
        match plan.step {
            Step::ServiceInPlace { depot } => {
                let est = plan.cost_s;
                let action = Action::Service {
                    depot: self.world.depots[depot].depot_id.clone(),
                };
                let job = Job::Service {
                    depot,
                    preventive: false,
                };
                round.depots.entry(depot).or_default().push(now + est);
                let s = sent(&self.world, x, est, PlannedUse::default());
                round.cmds.push(self.exec.issue(x, &xid, job, action, s));
            }
            Step::ChargeInPlace { source } => {
                let until = {
                    let ctx_load = &round.load;
                    let engaged = BTreeSet::new();
                    let running = BTreeSet::new();
                    let depot_wait = |_: usize, _: f64| 0.0;
                    let ctx = recovery::Ctx {
                        world: &self.world,
                        reg: &self.reg,
                        energy: &self.energy,
                        health: &self.health,
                        cfg,
                        load: ctx_load,
                        depot_wait: &depot_wait,
                        service_s: self.service_s,
                        engaged: &engaged,
                        running: &running,
                    };
                    recovery::recovery_charge_wh(&ctx, x, source)
                };
                let battery = self.world.fleet.get(x).battery_wh();
                let (source_id, power, wait) = match source {
                    ChargeSource::Station(s) => {
                        let st = &self.reg.stations()[s];
                        (
                            st.id.clone(),
                            st.slot_power_w * self.world.charge_fraction(),
                            round.load.wait_s(s, st.slots, now),
                        )
                    }
                    ChargeSource::Mobile(c) => (
                        self.reg.robot(c).id.clone(),
                        self.reg.charger_power_w(c),
                        0.0,
                    ),
                };
                let energy_wh = (until - battery).max(0.0);
                let plan = ChargePlan {
                    source,
                    source_id,
                    until_wh: until,
                    at: xpos,
                    est_s: wait + energy_wh / power.max(1.0) * 3600.0,
                    energy_wh,
                    fill_tank: false,
                };
                issue_charge(
                    &self.world,
                    &self.reg,
                    &mut self.energy,
                    &mut self.exec,
                    round,
                    x,
                    plan,
                    false,
                    diag,
                );
            }
            Step::Repair { repairer } => {
                let hpos = self.world.fleet.get(repairer).position().unwrap_or(xpos);
                let units = self.health.repair_units(&self.world, x, cfg);
                let work_s =
                    self.reg
                        .work_s(repairer, CapabilityName::Repair, units, cfg.min_work_s);
                let mut planned = drive_use(&self.reg, repairer, hpos, xpos);
                planned.work_s = work_s;
                let est = planned.drive_s + work_s;
                let job = Job::Repair {
                    target: x,
                    preventive: false,
                };
                let action = Action::Repair {
                    target_robot: xid.clone(),
                };
                let s = sent(&self.world, repairer, est, planned);
                let id = self.reg.robot(repairer).id.clone();
                round
                    .cmds
                    .push(self.exec.issue(repairer, &id, job, action, s));
                round.take(repairer);
            }
            Step::Tow { tower, to, purpose } => {
                let tpos = self.world.fleet.get(tower).position().unwrap_or(xpos);
                let tow_s = self.reg.tow_s(tower, xpos, to, cfg.tow_speed_factor);
                let mut planned = drive_use(&self.reg, tower, tpos, xpos);
                planned.drive_m += geom::dist(xpos, to) * self.reg.cell_size_m;
                planned.drive_s += tow_s;
                let est = planned.drive_s;
                if let TowPurpose::Depot(d) = purpose {
                    round
                        .depots
                        .entry(d)
                        .or_default()
                        .push(now + est + self.service_s);
                }
                let job = Job::Tow {
                    target: x,
                    to,
                    purpose,
                };
                let action = Action::Tow {
                    target_robot: xid.clone(),
                    to,
                };
                let s = sent(&self.world, tower, est, planned);
                let id = self.reg.robot(tower).id.clone();
                round.cmds.push(self.exec.issue(tower, &id, job, action, s));
                round.take(tower);
            }
            Step::BringCharger { charger } => {
                let cpos = self.world.fleet.get(charger).position().unwrap_or(xpos);
                let planned = drive_use(&self.reg, charger, cpos, xpos);
                let job = Job::Move {
                    to: xpos,
                    anticipatory: false,
                    rescue_for: Some(x),
                };
                let s = sent(&self.world, charger, planned.drive_s, planned);
                let id = self.reg.robot(charger).id.clone();
                round.cmds.push(self.exec.issue(
                    charger,
                    &id,
                    job,
                    Action::MoveTo { target: xpos },
                    s,
                ));
                self.energy.reserved.insert(charger, x);
                round.take(charger);
            }
            Step::Cannibalize { executor, donor } => {
                let job = Job::Cannibalize {
                    target: donor,
                    beneficiary: x,
                };
                let action = Action::Cannibalize {
                    target_robot: self.reg.robot(donor).id.clone(),
                    beneficiary_robot: Some(xid.clone()),
                };
                let s = sent(&self.world, executor, plan.cost_s, PlannedUse::default());
                let id = self.reg.robot(executor).id.clone();
                round
                    .cmds
                    .push(self.exec.issue(executor, &id, job, action, s));
                round.take(executor);
            }
        }
        round.take(x);
    }

    /// Steps 4 and 7: preventive maintenance. Before the auction (no
    /// `result`) it considers every free robot, and treats one that must stop
    /// to charge as at a natural break; after it, the robots the auction left
    /// with nothing to do or blocked on energy, which are at a natural break.
    fn maintain(
        &mut self,
        round: &mut Round,
        result: Option<&AuctionResult>,
        cfg: &Config,
        diag: &Diag,
    ) {
        let candidates: Vec<usize> = match result {
            Some(res) => round
                .free
                .iter()
                .copied()
                .filter(|r| {
                    res.role_bids.get(r).copied().unwrap_or(0) == 0
                        || res.energy_blocked.contains(r)
                })
                .collect(),
            None => round.free.clone(),
        };
        for r in candidates {
            if !round.free.contains(&r) {
                continue;
            }
            let natural_break = result.is_some() || Energy::low(&self.world, &self.reg, r, cfg);
            let e = self.world.fleet.get(r);
            let Some(pos) = e.position() else { continue };
            let hours = self.capacity.loads_s.get(r).copied().unwrap_or(0.0) / 3600.0;
            if hours <= 0.0 {
                continue;
            }
            let util = self.capacity.utilization(r);
            let weight = |u: f64| u.clamp(0.2, 1.0);
            // Field repair by the best free repairer.
            let units = self.health.repair_units(&self.world, r, cfg);
            let mut repair: Option<(f64, usize, f64)> = None;
            for &h in &round.free {
                if h == r || !self.reg.can(h, CapabilityName::Repair) {
                    continue;
                }
                let Some(hpos) = self.world.fleet.get(h).position() else {
                    continue;
                };
                let work_s = self
                    .reg
                    .work_s(h, CapabilityName::Repair, units, cfg.min_work_s);
                let work_wh = self
                    .reg
                    .work_wh(h, CapabilityName::Repair, units, cfg.min_work_s);
                let wh = self.reg.travel_wh(h, hpos, pos) + work_wh;
                if work_wh > cfg.repair_max_battery_frac * self.reg.robot(h).battery_wh
                    || !Energy::affordable(&self.world, &self.reg, h, wh, pos, cfg)
                {
                    continue;
                }
                let busy = self.reg.travel_s(h, hpos, pos) + work_s;
                let cost = busy * weight(util) + busy * weight(self.capacity.utilization(h));
                if repair.is_none_or(|(c, _, _)| cost < c - 1e-9) {
                    repair = Some((cost, h, work_s));
                }
            }
            // Depot service.
            let mut service: Option<(f64, usize, f64)> = None;
            for (d, depot) in self.world.depots.iter().enumerate() {
                let travel = self.reg.travel_s(r, pos, depot.position);
                let wh = self.reg.travel_wh(r, pos, depot.position);
                if !Energy::affordable(&self.world, &self.reg, r, wh, depot.position, cfg) {
                    continue;
                }
                let wait = round.depot_wait(d, cfg.depot_capacity, round.now + travel);
                let cost = 2.0 * travel + wait + self.service_s;
                if service.is_none_or(|(c, _, _)| cost < c - 1e-9) {
                    service = Some((cost, d, travel + wait + self.service_s));
                }
            }
            let Some(case) = self.health.maintenance(
                &self.world,
                &self.reg,
                r,
                hours,
                util,
                repair.map(|x| x.0),
                service.map(|x| x.0),
                cfg,
            ) else {
                continue;
            };
            if !case.worth_it(natural_break, cfg) {
                continue;
            }
            let rid = self.reg.robot(r).id.clone();
            diag.info(round.now, || {
                format!(
                    "maintenance for {rid} ({}): {:?}, benefit {:.0} s against {:.0} s; {}",
                    if natural_break {
                        "natural break"
                    } else {
                        "pressing"
                    },
                    case.kind,
                    case.benefit_s,
                    case.cost_s,
                    self.health.describe(&self.world, &self.reg, r, cfg)
                )
            });
            match case.kind {
                Maintenance::Repair => {
                    let Some((_, h, work_s)) = repair else {
                        continue;
                    };
                    let hpos = self.world.fleet.get(h).position().unwrap_or(pos);
                    let mut planned = drive_use(&self.reg, h, hpos, pos);
                    planned.work_s = work_s;
                    let est = planned.drive_s + work_s;
                    let job = Job::Repair {
                        target: r,
                        preventive: true,
                    };
                    let action = Action::Repair {
                        target_robot: rid.clone(),
                    };
                    let s = sent(&self.world, h, est, planned);
                    let hid = self.reg.robot(h).id.clone();
                    round.cmds.push(self.exec.issue(h, &hid, job, action, s));
                    round.take(h);
                    round.take(r);
                }
                Maintenance::Service => {
                    let Some((_, d, est)) = service else { continue };
                    let depot = &self.world.depots[d];
                    let planned = drive_use(&self.reg, r, pos, depot.position);
                    round.depots.entry(d).or_default().push(round.now + est);
                    let job = Job::Service {
                        depot: d,
                        preventive: true,
                    };
                    let action = Action::Service {
                        depot: depot.depot_id.clone(),
                    };
                    let s = sent(&self.world, r, est, planned);
                    round.cmds.push(self.exec.issue(r, &rid, job, action, s));
                    round.take(r);
                }
            }
        }
    }

    /// Step 5: robots at their low-water mark charge before anything else.
    /// Returns the robots that cannot reach any charger.
    fn charge_low(&mut self, round: &mut Round, cfg: &Config, diag: &Diag) -> Vec<usize> {
        let mut stranded = Vec::new();
        for r in round.free.clone() {
            if !round.free.contains(&r) || !Energy::low(&self.world, &self.reg, r, cfg) {
                continue;
            }
            // A mobile charger is on its way: wait for it.
            if self.charger_coming(r) {
                round.take(r);
                continue;
            }
            let chargers = chargers_for(r, &round.free, &self.energy, &self.exec, &self.reg);
            let need = self
                .capacity
                .energy_need_wh(r, &round.workload, &self.reg, cfg);
            let until = self.charge_until(r, Some(need), cfg);
            self.preempt_optional(round, r, diag);
            let plan =
                self.energy
                    .plan_charge(&self.world, &self.reg, r, until, &round.load, &chargers);
            if self.deliver_charger(round, r, until, plan.as_ref(), cfg, diag) {
                continue;
            }
            match plan {
                Some(plan) => issue_charge(
                    &self.world,
                    &self.reg,
                    &mut self.energy,
                    &mut self.exec,
                    round,
                    r,
                    plan,
                    false,
                    diag,
                ),
                None => stranded.push(r),
            }
        }
        stranded
    }

    /// Whether a mobile charger is driving to robot `r`.
    fn charger_coming(&self, r: usize) -> bool {
        self.energy.reserved.iter().any(|(&c, &k)| {
            k == r
                && matches!(
                    self.exec.active(c).map(|a| &a.job),
                    Some(Job::Move { rescue_for: Some(x), .. }) if *x == r
                )
        })
    }

    /// Send a free mobile charger to robot `r`, which charges where it stands,
    /// when that costs less time than `best` (its own trip to a charger and
    /// back, queueing included), counting the charger's time at its
    /// utilisation. Returns whether a charger was sent.
    fn deliver_charger(
        &mut self,
        round: &mut Round,
        r: usize,
        until: f64,
        best: Option<&ChargePlan>,
        cfg: &Config,
        diag: &Diag,
    ) -> bool {
        let e = self.world.fleet.get(r);
        let Some(pos) = e.position() else {
            return false;
        };
        let wanted = until.min(self.reg.robot(r).battery_wh) - e.battery_wh();
        if wanted <= 1.0 {
            return false;
        }
        let best_cost = best.map_or(f64::INFINITY, |p| p.est_s + self.reg.travel_s(r, p.at, pos));
        let mut pick: Option<(f64, usize)> = None;
        for &c in &round.free {
            let power = self.reg.charger_power_w(c);
            let tank = self.energy.tank(c);
            if c == r || power <= 0.0 || tank < 0.5 * wanted || tank <= 1.0 {
                continue;
            }
            if self.energy.reserved.contains_key(&c) {
                continue;
            }
            let Some(cpos) = self.world.fleet.get(c).position() else {
                continue;
            };
            // A charger already beside the robot is one of its normal options.
            if geom::dist(cpos, pos) <= recovery::REACH_CELLS {
                continue;
            }
            let trip_wh = self.reg.travel_wh(c, cpos, pos);
            if !Energy::affordable(&self.world, &self.reg, c, trip_wh, pos, cfg) {
                continue;
            }
            let travel = self.reg.travel_s(c, cpos, pos);
            let charge_s = wanted.min(tank) / power * 3600.0;
            let charger_time = (2.0 * travel + charge_s) * self.capacity.utilization(c);
            let cost = travel + charge_s + charger_time;
            if cost.is_finite()
                && cost < best_cost - 60.0
                && pick.is_none_or(|(b, _)| cost < b - 1e-9)
            {
                pick = Some((cost, c));
            }
        }
        let Some((cost, c)) = pick else {
            return false;
        };
        let cpos = self.world.fleet.get(c).position().unwrap_or(pos);
        diag.info(round.now, || {
            format!(
                "{} drives over to charge {} ({:.0} s against {:.0} s at the best charger)",
                self.reg.robot(c).id,
                self.reg.robot(r).id,
                cost,
                best_cost
            )
        });
        let planned = drive_use(&self.reg, c, cpos, pos);
        let job = Job::Move {
            to: pos,
            anticipatory: false,
            rescue_for: Some(r),
        };
        let s = sent(&self.world, c, planned.drive_s, planned);
        let cid = self.reg.robot(c).id.clone();
        round.cmds.push(
            self.exec
                .issue(c, &cid, job, Action::MoveTo { target: pos }, s),
        );
        self.energy.reserved.insert(c, r);
        round.take(c);
        round.take(r);
        true
    }

    /// Charge target for robot `r`: what its remaining work needs (charging
    /// past that only keeps a busy robot at the plug).
    fn charge_until(&self, r: usize, need: Option<f64>, cfg: &Config) -> f64 {
        Energy::charge_target(&self.world, &self.reg, r, need, cfg)
    }

    /// Step 6: the auction over free robots and interruptible jobs.
    fn auction(
        &mut self,
        round: &mut Round,
        stranded: &[usize],
        cfg: &Config,
        diag: &Diag,
    ) -> AuctionResult {
        let now = round.now;
        let Some(planner) = self.planner.as_ref() else {
            return AuctionResult::default();
        };
        let busy = self.exec.busy();
        let tasks = planner.tasks(&self.world, &busy, cfg);
        let mut crowd: BTreeMap<(CapabilityName, usize), usize> = BTreeMap::new();
        for a in self.exec.all_active() {
            if let Job::Work { task, .. } = &a.job
                && let Some(s) = task.section
            {
                *crowd.entry((task.capability(), s)).or_default() += 1;
            }
        }
        let bidders: Vec<Bidder> = round
            .free
            .iter()
            .filter(|r| !stranded.contains(r))
            .map(|&robot| Bidder {
                robot,
                roles_only: false,
            })
            .chain(round.interruptible.iter().map(|&robot| Bidder {
                robot,
                roles_only: true,
            }))
            .collect();
        let result = {
            let ctx = Context {
                world: &self.world,
                reg: &self.reg,
                planner,
                capacity: &self.capacity,
                cfg,
                busy: &busy,
                crowd: &crowd,
            };
            let exec = &self.exec;
            allocator::auction(&ctx, &bidders, &tasks, &|r, k| exec.cooling(r, k, now))
        };
        for award in &result.awards {
            let r = award.robot;
            if let Some(a) = self.exec.active(r) {
                // Cut an opportunistic charge short only when work has piled up.
                let charging = matches!(a.job, Job::Charge { .. });
                if charging
                    && result.role_bids.get(&r).copied().unwrap_or(0) < INTERRUPT_CHARGE_TASKS
                {
                    continue;
                }
                if let Some((cancel, old)) = self.exec.cancel(r, &self.reg.robot(r).id) {
                    if let Job::Charge { plan, .. } = &old.job
                        && let ChargeSource::Mobile(c) = plan.source
                    {
                        self.energy.reserved.remove(&c);
                    }
                    diag.debug(now, || {
                        format!(
                            "{}: cancel {} for better work",
                            self.reg.robot(r).id,
                            old.job.label()
                        )
                    });
                    round.cmds.push(cancel);
                }
            }
            let task = tasks[award.task].clone();
            let action = award.plan.action.clone();
            let scan = task.capability() == CapabilityName::Scan;
            let planned = work_use(&self.reg, &self.health, r, scan, &award.plan);
            let est = award.plan.duration_s;
            let job = Job::Work {
                task,
                plan: Box::new(award.plan.clone()),
            };
            diag.debug(now, || {
                format!(
                    "{}: {} (score {:.0})",
                    self.reg.robot(r).id,
                    job.label(),
                    award.score
                )
            });
            let s = sent(&self.world, r, est, planned);
            let id = self.reg.robot(r).id.clone();
            round.cmds.push(self.exec.issue(r, &id, job, action, s));
            round.free.retain(|&x| x != r);
        }
        result
    }

    /// Step 7: leftover robots charge when work is waiting on energy, top up
    /// while there is nothing to do, keep enough charge for the night, or
    /// wait near upcoming work.
    fn leftovers(
        &mut self,
        round: &mut Round,
        result: &AuctionResult,
        stranded: &[usize],
        cfg: &Config,
        diag: &Diag,
    ) {
        let now = round.now;
        let front = self
            .planner
            .as_ref()
            .and_then(|p| p.front(&self.world, cfg));
        for r in round.free.clone() {
            if !round.free.contains(&r) {
                continue;
            }
            let e = self.world.fleet.get(r);
            let Some(pos) = e.position() else { continue };
            let battery = e.battery_wh();
            let payload = e.payload_kg();
            let cap_wh = self.reg.robot(r).battery_wh;
            let need = Some(
                self.capacity
                    .energy_need_wh(r, &round.workload, &self.reg, cfg),
            );
            let blocked = result.energy_blocked.contains(&r) || stranded.contains(&r);
            let idle_work = result.role_bids.get(&r).copied().unwrap_or(0) == 0;
            let opportunistic = !blocked
                && idle_work
                && battery < cfg.opportunistic_charge_frac * cap_wh
                && need.is_some_and(|need| {
                    need > battery - Energy::reserve_wh(&self.reg, r, pos, cfg)
                });
            if blocked || opportunistic {
                if self.charger_coming(r) {
                    round.take(r);
                    continue;
                }
                let chargers = chargers_for(r, &round.free, &self.energy, &self.exec, &self.reg);
                let until = self.charge_until(r, need, cfg);
                if blocked {
                    self.preempt_optional(round, r, diag);
                }
                let plan = self.energy.plan_charge(
                    &self.world,
                    &self.reg,
                    r,
                    until,
                    &round.load,
                    &chargers,
                );
                if blocked && self.deliver_charger(round, r, until, plan.as_ref(), cfg, diag) {
                    continue;
                }
                // Optional top-ups stay off the grid when it cannot spare the energy.
                let plan = plan.filter(|p| blocked || self.optional_station_ok(round, p, cfg));
                if let Some(plan) = plan {
                    issue_charge(
                        &self.world,
                        &self.reg,
                        &mut self.energy,
                        &mut self.exec,
                        round,
                        r,
                        plan,
                        opportunistic,
                        diag,
                    );
                    continue;
                }
                let rescue_underway = self.energy.reserved.values().any(|&client| client == r);
                if blocked && !rescue_underway {
                    // Nothing reachable: bring a mobile charger over.
                    let candidates: Vec<usize> =
                        round.free.iter().copied().filter(|&c| c != r).collect();
                    if let Some(c) =
                        self.energy
                            .rescuer(&self.world, &self.reg, &candidates, pos, r)
                    {
                        let cpos = self.world.fleet.get(c).position().unwrap_or(pos);
                        let planned = drive_use(&self.reg, c, cpos, pos);
                        let job = Job::Move {
                            to: pos,
                            anticipatory: false,
                            rescue_for: Some(r),
                        };
                        diag.info(now, || {
                            format!("{} rescues {}", self.reg.robot(c).id, self.reg.robot(r).id)
                        });
                        let s = sent(&self.world, c, planned.drive_s, planned);
                        let cid = self.reg.robot(c).id.clone();
                        round.cmds.push(self.exec.issue(
                            c,
                            &cid,
                            job,
                            Action::MoveTo { target: pos },
                            s,
                        ));
                        self.energy.reserved.insert(c, r);
                        round.take(c);
                    }
                    continue;
                }
            }
            // An idle mobile charger with a low tank goes to refill it.
            if idle_work
                && let Some(plan) =
                    self.energy
                        .plan_tank_refill(&self.world, &self.reg, r, &round.load)
                && self.optional_station_ok(round, &plan, cfg)
            {
                issue_charge(
                    &self.world,
                    &self.reg,
                    &mut self.energy,
                    &mut self.exec,
                    round,
                    r,
                    plan,
                    true,
                    diag,
                );
                continue;
            }
            // Nothing to do: keep enough charge to idle through the night, and
            // take free energy while the bank overflows.
            if idle_work && !blocked && self.park(round, r, pos, battery, cfg, diag) {
                continue;
            }
            // Wait near the work front if this robot works the pad surface.
            let Some(to) = front else { continue };
            let works_front = FRONT_CAPABILITIES
                .iter()
                .any(|&c| self.capacity.is_role(r, c, cfg.role_min_share));
            if !works_front || geom::dist(pos, to) <= 3.0 {
                continue;
            }
            let trip = self.reg.travel_wh(r, pos, to);
            if !Energy::affordable(&self.world, &self.reg, r, trip, to, cfg) {
                continue;
            }
            let job = Job::Move {
                to,
                anticipatory: true,
                rescue_for: None,
            };
            diag.debug(now, || format!("{}: {}", self.reg.robot(r).id, job.label()));
            let planned = drive_use(&self.reg, r, pos, to);
            let s = Sent {
                now,
                est_s: planned.drive_s,
                battery_wh: battery,
                payload_kg: payload,
                planned,
            };
            let id = self.reg.robot(r).id.clone();
            round.cmds.push(
                self.exec
                    .issue(r, &id, job, Action::MoveTo { target: to }, s),
            );
            round.free.retain(|&x| x != r);
        }
    }

    /// Battery robot `r` at `at` needs to sit idle for the parking horizon
    /// (heater included on cold nights) and still reach a charger, Wh.
    fn park_need_wh(&self, r: usize, at: GridCoord, cfg: &Config) -> f64 {
        let now = self.world.now;
        let horizon = cfg.park_horizon_s;
        let cold = self.power.cold_s(now, now + horizon, cfg);
        Energy::reserve_wh(&self.reg, r, at, cfg)
            + Energy::margin_wh(&self.reg, r, cfg)
            + self.reg.idle_wh(r, horizon)
            + self.reg.idle_wh(r, cold) * cfg.heater_idle_factor
    }

    /// Parking and surplus charging for an idle robot. Returns whether a
    /// charge was issued.
    fn park(
        &mut self,
        round: &mut Round,
        r: usize,
        pos: GridCoord,
        battery: f64,
        cfg: &Config,
        diag: &Diag,
    ) -> bool {
        let cap = self.reg.robot(r).battery_wh;
        let survival = battery < self.park_need_wh(r, pos, cfg);
        let surplus = self.power.surplus(&self.world, round.charging_w, cfg)
            && battery < cfg.surplus_charge_frac * cap;
        if !survival && !surplus {
            return false;
        }
        let Some(plan) = self
            .energy
            .plan_charge(&self.world, &self.reg, r, cap, &round.load, &[])
        else {
            return false;
        };
        let ChargeSource::Station(s) = plan.source else {
            return false;
        };
        let st = &self.reg.stations()[s];
        let occupied = round.load.ends.get(&s).map_or(0, Vec::len);
        // Leave a slot for robots with work, unless the battery would not last.
        let slot_ok = if survival {
            occupied < st.slots as usize
        } else {
            occupied + 1 < st.slots as usize
        };
        if !slot_ok || (!survival && !self.optional_station_ok(round, &plan, cfg)) {
            return false;
        }
        diag.info(round.now, || {
            format!(
                "{} parks charged ({}): {:.0} Wh",
                self.reg.robot(r).id,
                if survival {
                    "night reserve"
                } else {
                    "solar surplus"
                },
                battery
            )
        });
        issue_charge(
            &self.world,
            &self.reg,
            &mut self.energy,
            &mut self.exec,
            round,
            r,
            plan,
            true,
            diag,
        );
        true
    }

    /// Robot `r` needs a station slot for work: cut short an optional charge
    /// (a top-up or a parking charge) holding the last free slot at the
    /// station it would use, so working robots never queue behind idle ones.
    fn preempt_optional(&mut self, round: &mut Round, r: usize, diag: &Diag) {
        let Some(pos) = self.world.fleet.get(r).position() else {
            return;
        };
        let nearest = self
            .reg
            .stations()
            .iter()
            .enumerate()
            .filter(|(_, st)| st.slots > 0 && st.slot_power_w > 0.0)
            .min_by(|a, b| {
                geom::dist(pos, a.1.position)
                    .total_cmp(&geom::dist(pos, b.1.position))
                    .then(a.0.cmp(&b.0))
            })
            .map(|(s, st)| (s, st.slots as usize));
        let Some((s, slots)) = nearest else { return };
        if round.load.ends.get(&s).map_or(0, Vec::len) < slots {
            return;
        }
        // The optional session there that would run longest.
        let victim = self
            .exec
            .all_active()
            .filter(|a| a.robot != r)
            .filter(|a| {
                matches!(&a.job, Job::Charge { plan, opportunistic: true } if plan.source == ChargeSource::Station(s))
            })
            .max_by(|a, b| a.est_end_t.total_cmp(&b.est_end_t).then(b.robot.cmp(&a.robot)))
            .map(|a| (a.robot, a.est_end_t));
        let Some((v, end_t)) = victim else { return };
        if let Some((cancel, old)) = self.exec.cancel(v, &self.reg.robot(v).id) {
            diag.info(round.now, || {
                format!(
                    "{}: cut {} short for {}",
                    self.reg.robot(v).id,
                    old.job.label(),
                    self.reg.robot(r).id
                )
            });
            round.cmds.push(cancel);
            if let Some(ends) = round.load.ends.get_mut(&s)
                && let Some(i) = ends.iter().position(|&t| t == end_t)
            {
                ends.remove(i);
            }
            round.charging_w = (round.charging_w - self.reg.stations()[s].slot_power_w).max(0.0);
        }
    }

    /// Whether an optional charge may draw on the site grid now.
    fn optional_station_ok(&self, round: &Round, plan: &ChargePlan, cfg: &Config) -> bool {
        let ChargeSource::Station(s) = plan.source else {
            return true;
        };
        let power = self.reg.stations()[s].slot_power_w;
        self.power
            .optional_charging_ok(&self.world, round.charging_w, power, plan.est_s, cfg)
    }
}

/// Mobile chargers robot `r` may use now: free ones, plus any reserved for it
/// that have arrived and are idle.
fn chargers_for(
    r: usize,
    free: &[usize],
    energy: &Energy,
    exec: &Executor,
    reg: &Registry,
) -> Vec<usize> {
    let mut out: Vec<usize> = free
        .iter()
        .copied()
        .filter(|&c| c != r && reg.charger_power_w(c) > 0.0)
        .collect();
    for (&c, &client) in &energy.reserved {
        if client == r && exec.active(c).is_none() && !out.contains(&c) {
            out.push(c);
        }
    }
    out.sort_unstable();
    out
}

#[allow(clippy::too_many_arguments)]
fn issue_charge(
    world: &WorldModel,
    reg: &Registry,
    energy: &mut Energy,
    exec: &mut Executor,
    round: &mut Round,
    r: usize,
    plan: ChargePlan,
    opportunistic: bool,
    diag: &Diag,
) {
    let now = world.now;
    match plan.source {
        ChargeSource::Station(s) => {
            round.load.add(s, now + plan.est_s);
            round.charging_w += reg.stations()[s].slot_power_w;
        }
        ChargeSource::Mobile(c) => {
            energy.reserved.insert(c, r);
            round.take(c);
        }
    }
    let action = Action::Charge {
        source: plan.source_id.clone(),
        until_wh: (!plan.fill_tank).then_some(plan.until_wh),
    };
    let est = plan.est_s;
    let planned = world
        .fleet
        .get(r)
        .position()
        .map(|p| drive_use(reg, r, p, plan.at))
        .unwrap_or_default();
    let job = Job::Charge {
        plan,
        opportunistic,
    };
    let e = world.fleet.get(r);
    diag.info(now, || {
        format!(
            "{} ({:.0} Wh): {}",
            reg.robot(r).id,
            e.battery_wh(),
            job.label()
        )
    });
    let s = sent(world, r, est, planned);
    round
        .cmds
        .push(exec.issue(r, &reg.robot(r).id, job, action, s));
    round.take(r);
}

fn describe_capacity(plan: &CapacityPlan, reg: &Registry) -> String {
    let mut parts = Vec::new();
    for r in 0..reg.len() {
        let roles: Vec<String> = crate::registry::WORK_CAPABILITIES
            .iter()
            .filter_map(|&c| {
                let s = plan.share(r, c);
                (s > 0.005).then(|| format!("{c:?} {:.0}%", s * 100.0).to_lowercase())
            })
            .collect();
        if !roles.is_empty() {
            parts.push(format!("{} [{}]", reg.robot(r).id, roles.join(", ")));
        }
    }
    format!(
        "capacity plan{}: makespan {:.1} h; {}",
        if plan.optimal { "" } else { " (fallback)" },
        plan.makespan_s / 3600.0,
        parts.join("; ")
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::diag::Level;
    use crate::registry::tests::{cap, robot, scenario};
    use arcadia_protocol::*;

    fn hello() -> Box<Hello> {
        let mut sc = scenario(vec![
            robot(
                "scout",
                0.5,
                vec![cap(CapabilityName::Scan, 4.0, 0.05, None)],
            ),
            robot(
                "worker",
                0.2,
                vec![
                    cap(CapabilityName::Dig, 0.01, 50.0, None),
                    cap(CapabilityName::Grade, 0.05, 5.0, None),
                    cap(CapabilityName::Compact, 0.1, 4.0, None),
                    cap(CapabilityName::Sinter, 0.02, 30.0, None),
                    cap(CapabilityName::Haul, 2.0, 0.01, Some(300.0)),
                ],
            ),
        ]);
        sc.site.cell_size_m = 1.0;
        Box::new(Hello {
            protocol_version: PROTOCOL_VERSION.into(),
            simulator: SoftwareInfo {
                name: "t".into(),
                version: "0".into(),
            },
            run_id: "r".into(),
            seed: 0,
            scenario: sc,
        })
    }

    fn state(id: &str, x: u32, y: u32) -> RobotState {
        RobotState {
            robot_id: id.into(),
            position: GridCoord { x, y },
            status: RobotStatus::Idle,
            battery_wh: 5000.0,
            payload_kg: 0.0,
            current_command: None,
            health: HealthTelemetry {
                wheel_slip: 0.0,
                motor_current_ratio: 1.0,
                dust_coverage: 0.0,
                temperature_c: -50.0,
            },
            faults: vec![],
            in_relay_coverage: true,
            telemetry_t: 0.0,
        }
    }

    fn goal() -> Event {
        Event {
            t: 0.0,
            kind: EventKind::EarthMessage {
                sent_t: 0.0,
                message: EarthToSite {
                    message_id: "m".into(),
                    body: Uplink::Goal {
                        goal_id: "pad".into(),
                        priority: 1,
                        deadline_t: None,
                        objective: Objective::LandingPad {
                            pad: LandingPadSpec {
                                area: GridRect {
                                    min: GridCoord { x: 5, y: 5 },
                                    max: GridCoord { x: 7, y: 7 },
                                },
                                target_elevation_m: -10.0,
                                surface_tolerance_m: 0.05,
                                max_slope_deg: 2.0,
                                stabilize: true,
                                berm: None,
                            },
                        },
                    },
                },
            },
        }
    }

    #[test]
    fn first_decision_surveys_and_sends_the_worker_to_the_front() {
        let mut os = Coordinator::with_config(Config::default(), Diag::new(Level::Off));
        os.start(hello());
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal()],
            robots: vec![state("scout", 1, 1), state("worker", 1, 2)],
        });
        assert_eq!(d.seq, 0);
        assert!(os.planner().is_some());
        assert!(
            os.capacity()
                .unwrap()
                .is_role(1, CapabilityName::Sinter, 0.05)
        );
        let scan = d.commands.iter().find(|c| c.robot_id == "scout").unwrap();
        assert!(matches!(scan.action, Action::Scan { .. }));
        // Nothing is ready for the worker yet, so it heads for the work front.
        let worker = d.commands.iter().find(|c| c.robot_id == "worker").unwrap();
        assert!(
            matches!(worker.action, Action::MoveTo { target } if target.x >= 5 && target.y >= 5)
        );
        let s = os.summary();
        assert_eq!((s.decisions, s.commands, s.cells_total), (1, 2, 9));
    }

    #[test]
    fn mobile_chargers_reserved_for_a_client_are_offered_to_it() {
        let sc = scenario(vec![
            robot(
                "a",
                0.5,
                vec![cap(CapabilityName::Sinter, 0.01, 60.0, None)],
            ),
            robot(
                "c",
                0.5,
                vec![cap(CapabilityName::Charge, 0.5, 0.0, Some(1000.0))],
            ),
            robot(
                "d",
                0.5,
                vec![cap(CapabilityName::Charge, 0.5, 0.0, Some(1000.0))],
            ),
        ]);
        let reg = Registry::new(&sc);
        let mut energy = Energy::new(&reg);
        let exec = Executor::new();
        assert_eq!(chargers_for(0, &[0, 2], &energy, &exec, &reg), vec![2]);
        energy.reserved.insert(1, 0);
        assert_eq!(chargers_for(0, &[0, 2], &energy, &exec, &reg), vec![1, 2]);
        assert_eq!(
            chargers_for(2, &[0], &energy, &exec, &reg),
            Vec::<usize>::new()
        );
    }

    /// A worker, a scout that repairs, and a hauler that tows and charges, on
    /// a 30 × 30 site with the station at (0,0) and a depot at (2,0).
    fn crew_hello() -> Box<Hello> {
        let mut sc = scenario(vec![
            robot(
                "worker",
                0.15,
                vec![
                    cap(CapabilityName::Compact, 0.1, 4.0, None),
                    cap(CapabilityName::Sinter, 0.01, 60.0, None),
                ],
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
                    cap(CapabilityName::Dig, 0.01, 50.0, None),
                    cap(CapabilityName::Grade, 0.05, 5.0, None),
                    cap(CapabilityName::Haul, 3.0, 0.01, Some(400.0)),
                    cap(CapabilityName::Charge, 0.5, 0.0, Some(8000.0)),
                ],
            ),
        ]);
        sc.site.width = 30;
        sc.site.height = 30;
        sc.site.cell_size_m = 1.0;
        sc.depots = vec![Depot {
            depot_id: "dp".into(),
            position: GridCoord { x: 2, y: 0 },
        }];
        let mut h = hello();
        h.scenario = sc;
        h
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

    fn crew_os() -> Coordinator {
        let mut os = Coordinator::with_config(Config::default(), Diag::new(Level::Off));
        os.start(crew_hello());
        os
    }

    fn command_for<'a>(d: &'a Decision, id: &str) -> Option<&'a Command> {
        d.commands.iter().find(|c| c.robot_id == id)
    }

    #[test]
    fn a_depot_only_fault_is_towed_to_the_depot_and_serviced_there() {
        let mut os = crew_os();
        let mut worker = state("worker", 12, 12);
        worker.status = RobotStatus::Failed;
        worker.faults = vec![fault("tool", Severity::Disabled, false)];
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal()],
            robots: vec![worker.clone(), state("scout", 6, 6), state("hauler", 8, 8)],
        });
        let tow = command_for(&d, "hauler").expect("the hauler tows");
        assert_eq!(
            tow.action,
            Action::Tow {
                target_robot: "worker".into(),
                to: GridCoord { x: 2, y: 0 }
            }
        );
        assert!(command_for(&d, "worker").is_none());
        // The tow done, the worker stands on the depot cell and is serviced.
        let tow_id = tow.command_id.clone();
        worker.position = GridCoord { x: 2, y: 0 };
        let d = os.decide(&Observation {
            seq: 1,
            t: 400.0,
            events: vec![Event {
                t: 400.0,
                kind: EventKind::CommandCompleted {
                    command_id: tow_id,
                    robot_id: "hauler".into(),
                    energy_used_wh: 40.0,
                },
            }],
            robots: vec![worker, state("scout", 6, 6), state("hauler", 2, 0)],
        });
        let service = command_for(&d, "worker").expect("the worker is serviced");
        assert_eq!(service.action, Action::Service { depot: "dp".into() });
    }

    #[test]
    fn an_offline_robot_at_the_station_charges_where_it_is() {
        let mut os = crew_os();
        let mut worker = state("worker", 0, 0);
        worker.status = RobotStatus::Offline;
        worker.battery_wh = 0.0;
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal()],
            robots: vec![worker, state("scout", 6, 6), state("hauler", 8, 8)],
        });
        let charge = command_for(&d, "worker").expect("the worker charges");
        assert!(matches!(
            &charge.action,
            Action::Charge { source, until_wh: Some(u) } if source == "st" && *u > 4000.0
        ));
    }

    #[test]
    fn an_offline_robot_in_the_field_gets_the_mobile_charger() {
        // A weak station: the mobile charger is the quicker source.
        let mut h = crew_hello();
        h.scenario.stations[0].max_power_w = 1000.0;
        let mut os = Coordinator::with_config(Config::default(), Diag::new(Level::Off));
        os.start(h);
        let mut worker = state("worker", 20, 20);
        worker.status = RobotStatus::Offline;
        worker.battery_wh = 0.0;
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal()],
            robots: vec![worker, state("scout", 6, 6), state("hauler", 22, 21)],
        });
        let mv = command_for(&d, "hauler").expect("the charger drives over");
        assert_eq!(
            mv.action,
            Action::MoveTo {
                target: GridCoord { x: 20, y: 20 }
            }
        );
    }

    #[test]
    fn a_stuck_and_failed_robot_next_to_the_scout_is_repaired() {
        // A field repair frees it and clears the fault in one go; a tow would
        // still leave a depot service to do.
        let mut os = crew_os();
        let mut worker = state("worker", 20, 20);
        worker.status = RobotStatus::Stuck;
        worker.faults = vec![fault("joints", Severity::Disabled, true)];
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal()],
            robots: vec![worker, state("scout", 20, 21), state("hauler", 1, 28)],
        });
        let cmd = command_for(&d, "scout").expect("the scout helps");
        assert_eq!(
            cmd.action,
            Action::Repair {
                target_robot: "worker".into()
            }
        );
        assert!(os.health().is_some());
    }

    #[test]
    fn an_idle_robot_low_before_the_night_parks_charged() {
        let mut os = crew_os();
        // The worker has nothing ready yet and enough charge for its share of
        // the work, but not for twelve idle hours with the heater on.
        let mut worker = state("worker", 6, 6);
        worker.battery_wh = 800.0;
        let t = 0.7 * SOL_S;
        let d = os.decide(&Observation {
            seq: 0,
            t,
            events: vec![goal()],
            robots: vec![worker, state("scout", 6, 7), state("hauler", 8, 8)],
        });
        let charge = command_for(&d, "worker").expect("the worker parks charged");
        assert_eq!(
            charge.action,
            Action::Charge {
                source: "st".into(),
                until_wh: Some(5000.0)
            }
        );
        // A full battery needs no parking charge.
        let mut os = crew_os();
        let d = os.decide(&Observation {
            seq: 0,
            t,
            events: vec![goal()],
            robots: vec![
                state("worker", 6, 6),
                state("scout", 6, 7),
                state("hauler", 8, 8),
            ],
        });
        assert!(
            !command_for(&d, "worker").is_some_and(|c| matches!(c.action, Action::Charge { .. }))
        );
    }

    #[test]
    fn a_free_mobile_charger_drives_to_a_slow_robot_in_the_field() {
        let mut sc = scenario(vec![
            robot(
                "slow",
                0.05,
                vec![cap(CapabilityName::Sinter, 0.01, 60.0, None)],
            ),
            robot(
                "cart",
                0.5,
                vec![cap(CapabilityName::Charge, 0.5, 0.0, Some(8000.0))],
            ),
        ]);
        sc.site.width = 30;
        sc.site.height = 30;
        sc.site.cell_size_m = 1.0;
        sc.stations[0].max_power_w = 2000.0;
        let mut h = hello();
        h.scenario = sc;
        let mut os = Coordinator::with_config(Config::default(), Diag::new(Level::Off));
        os.start(h);
        let mut slow = state("slow", 28, 28);
        slow.battery_wh = 200.0;
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal()],
            robots: vec![slow, state("cart", 18, 28)],
        });
        // The cart comes over rather than the slow robot driving anywhere.
        let mv = command_for(&d, "cart").expect("the cart drives over");
        assert_eq!(
            mv.action,
            Action::MoveTo {
                target: GridCoord { x: 28, y: 28 }
            }
        );
        assert!(command_for(&d, "slow").is_none());
        // Once it has arrived the robot charges from it where it stands.
        let id = mv.command_id.clone();
        let mut slow = state("slow", 28, 28);
        slow.battery_wh = 190.0;
        let d = os.decide(&Observation {
            seq: 1,
            t: 20.0,
            events: vec![Event {
                t: 20.0,
                kind: EventKind::CommandCompleted {
                    command_id: id,
                    robot_id: "cart".into(),
                    energy_used_wh: 3.0,
                },
            }],
            robots: vec![slow, state("cart", 28, 28)],
        });
        let charge = command_for(&d, "slow").expect("the robot charges");
        assert!(matches!(&charge.action, Action::Charge { source, .. } if source == "cart"));
    }

    #[test]
    fn lost_robots_are_never_commanded() {
        let mut os = crew_os();
        let mut hauler = state("hauler", 8, 8);
        hauler.status = RobotStatus::Lost;
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal()],
            robots: vec![state("worker", 6, 6), state("scout", 6, 7), hauler],
        });
        assert!(command_for(&d, "hauler").is_none());
        assert!(!os.world().unwrap().fleet.get(2).operable());
        let cap = os.capacity().unwrap();
        assert_eq!(cap.share(2, CapabilityName::Haul), 0.0);
    }
}
