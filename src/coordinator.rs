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
//!    tow, a mobile charger brought over, or (with Earth's approval, or the
//!    level-3 fallback) parts from a donor ([`crate::recovery`]);
//! 4. maintenance so pressing it cannot wait for a natural break
//!    ([`crate::health`]);
//! 5. station a relay, or stand one down, where coverage needs it;
//! 6. send robots below their energy low-water mark to charge;
//! 7. run the auction over free robots (and robots on interruptible jobs);
//! 8. give leftover robots maintenance at this natural break, a charge when
//!    work waits on energy, an opportunistic or parking top-up (held back
//!    when it would drain the site bank at night or in a storm), or a move
//!    towards upcoming work;
//! 9. report to Earth: a notify for every level-2 action, approval requests,
//!    and a periodic status report ([`crate::authority`]);
//! 10. ask for a wake-up while anything waits on time.
//!
//! Every action passes the authority layer: level 1 is taken, level 2 is
//! taken and notified, level 3 and 4 only with an approval in hand. Robots
//! out of coverage are never commanded; their snapshots are stale and their
//! events arrive late.

use crate::allocator::{self, AuctionResult, Bidder, Context};
use crate::authority::{self, Authority, Reach, Stakes};
use crate::capacity::{CapacityPlan, Workload};
use crate::config::Config;
use crate::coverage::Coverage;
use crate::diag::Diag;
use crate::energy::{ChargePlan, ChargeSource, Energy, StationLoad};
use crate::executor::{Assignment, Executor, Job, Outcome, PlannedUse, Sent, TowPurpose};
use crate::geom;
use crate::health::{Health, Maintenance};
use crate::planner::{Planner, SpoilKind};
use crate::power::PowerModel;
use crate::recovery::{self, Step};
use crate::registry::Registry;
use crate::tasks::{Stage, Task, TaskKey};
use crate::world::WorldModel;
use arcadia_protocol::{
    Action, CapabilityName, Command, Decision, EarthToSite, End, Event, EventKind, FailReason,
    Fallback, GoalProgress, GridCoord, Hello, Objective, Observation, RejectReason, RobotStatus,
    Uplink,
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

/// Robot commands that are routine work: if a policy puts one at level 3 or
/// 4, ArcadiaOS asks Earth for a standing approval before using it again.
const ROUTINE_ACTIONS: [&str; 14] = [
    "move_to", "scan", "dig", "grade", "haul", "compact", "sinter", "charge", "relay", "wait",
    "cancel", "repair", "service", "tow",
];

/// Approval purpose for a standing approval of a routine action.
const ROUTINE: &str = "routine";

/// Approval purpose for charging from the site's energy reserve.
const RESERVE: &str = "charging from the site energy reserve";

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
    /// Policy levels, approvals and messages to Earth.
    authority: Authority,
    /// The covered part of the site at this decision.
    coverage: Coverage,
    /// Robots known to be lost, to notice new losses.
    lost_marks: Vec<bool>,
    /// Tasks released by a disabled robot whose reassignment awaits Earth:
    /// task → (the robot that had it, approval purpose).
    held: BTreeMap<TaskKey, (usize, String)>,
    /// A request was sent that the next decision may act on at once.
    act_soon: bool,
    /// Earth was told the site is charging from its energy reserve.
    reserve_noted: bool,
    /// Strips ArcadiaOS asked Earth about: (donor, robot to restore).
    strips: Vec<(usize, usize)>,
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
    /// Routine actions at level 3 or 4 without a standing approval.
    blocked: BTreeSet<&'static str>,
    /// New station charging would draw on the site's energy reserve, which
    /// the policy does not allow without approval: the supply to spare, W.
    reserve_spare_w: Option<f64>,
    /// Charging was held back because of the reserve.
    reserve_wanted: bool,
    /// Station charges issued in this decision.
    station_charges: usize,
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
        let coverage = Coverage::new(&world.base_station, world.site.cell_size_m, &[]);
        self.state = Some(State {
            energy: Energy::new(&reg),
            health: Health::new(reg.len()),
            power: PowerModel::new(hello.scenario.power.clone(), hello.scenario.sol_length_s),
            service_s: self.cfg.service_s,
            fault_marks: vec![0; reg.len()],
            lost_marks: vec![false; reg.len()],
            world,
            reg,
            planner: None,
            capacity: CapacityPlan::default(),
            capacity_dirty: true,
            exec: Executor::new(),
            same_time: (f64::NAN, 0),
            decisions: 0,
            authority: Authority::default(),
            coverage,
            held: BTreeMap::new(),
            act_soon: false,
            reserve_noted: false,
            strips: Vec::new(),
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
            decision.to_earth = st.authority.take_outbox();
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

    /// The authority layer: requests, answers and traffic to Earth.
    pub fn authority(&self) -> Option<&Authority> {
        self.state.as_ref().map(|s| &s.authority)
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
        for r in 0..self.reg.len() {
            let lost = self.world.fleet.get(r).lost();
            if lost && !self.lost_marks[r] {
                diag.info(t, || format!("{} is lost", self.reg.robot(r).id));
                self.authority.major_event();
                self.capacity_dirty = true;
            }
            self.lost_marks[r] = lost;
        }
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
                diag.info(self.world.now, || {
                    format!(
                        "policy version {} in effect: {}",
                        policy.policy_version,
                        policy
                            .levels
                            .iter()
                            .filter(|(a, l)| self.world.policy.level_for(a) != **l)
                            .map(|(a, l)| format!("{a} {}", l.get()))
                            .collect::<Vec<_>>()
                            .join(", ")
                    )
                });
                self.world.policy = policy.clone();
                self.authority.counts.policy_updates += 1;
                self.authority.major_event();
            }
            Uplink::Approve { request_id, .. } => {
                diag.info(self.world.now, || format!("Earth approves {request_id}"));
                self.authority.on_answer(request_id, true, self.world.now);
            }
            Uplink::Veto { request_id, .. } => {
                diag.info(self.world.now, || format!("Earth vetoes {request_id}"));
                self.authority.on_answer(request_id, false, self.world.now);
            }
            Uplink::Reprioritize { priorities } => {
                // One goal: priorities are recorded, and nothing else changes.
                for p in priorities {
                    self.authority
                        .priorities
                        .insert(p.goal_id.clone(), p.priority);
                }
            }
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
                Outcome::Failed { reason, .. } => {
                    self.work_failed(r, &task, &plan, reason, t, cfg, diag)
                }
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
            // A relay ends only by failing; a cancellation is the planner's own.
            Job::Relay { .. } => {
                if !cancelled {
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

    #[allow(clippy::too_many_arguments)]
    fn work_failed(
        &mut self,
        r: usize,
        task: &Task,
        plan: &allocator::WorkPlan,
        reason: FailReason,
        t: f64,
        cfg: &Config,
        diag: &Diag,
    ) {
        if reason == FailReason::Cancelled {
            return;
        }
        if matches!(reason, FailReason::Fault | FailReason::Stuck) {
            self.released(r, task, reason, cfg, diag);
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
            // The authority layer asks Earth before acting, so these should not
            // happen; if one does, the task waits out a back-off.
            RejectReason::ApprovalRequired | RejectReason::Vetoed => {
                self.exec.back_off(r, task.key, t + cfg.retry_backoff_s);
            }
        }
    }

    fn dispatch(&mut self, cfg: &Config, diag: &Diag) -> (Vec<Command>, Option<f64>) {
        let now = self.world.now;
        let next_cooldown = self.exec.prune_cooldowns(now);
        self.coverage = self.coverage_now();
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
        if (self.capacity_dirty || self.capacity.stale(&workload, &available))
            && self.replan_allowed(&available, cfg, diag)
        {
            self.capacity = CapacityPlan::solve(&workload, &self.reg, &available, cfg);
            self.capacity_dirty = false;
            diag.info(now, || describe_capacity(&self.capacity, &self.reg));
        }
        self.release_chargers(cfg);

        let mut round = self.round(workload, now);
        round.blocked = self.blocked_actions();
        round.reserve_spare_w = self.reserve_lock(cfg);
        let tasks = self
            .planner
            .as_ref()
            .map(|p| p.tasks(&self.world, &self.exec.busy(), cfg))
            .unwrap_or_default();
        self.held.retain(|k, _| tasks.iter().any(|t| t.key == *k));
        self.recover(&mut round, cfg, diag);
        self.maintain(&mut round, None, cfg, diag);
        self.relay(&mut round, &tasks, cfg, diag);
        let stranded = self.charge_low(&mut round, cfg, diag);
        let result = self.auction(&mut round, &tasks, &stranded, cfg, diag);
        self.maintain(&mut round, Some(&result), cfg, diag);
        self.leftovers(&mut round, &result, &stranded, cfg, diag);
        self.report(&mut round, cfg, diag);

        let mut wake = next_cooldown.filter(|&w| w > now);
        let waiting =
            !round.free.is_empty() || (0..n).any(|r| self.world.fleet.get(r).recoverable());
        if (self.exec.idle() || waiting) && !progress.complete() {
            let idle_wake = now + cfg.idle_wake_s;
            wake = Some(wake.map_or(idle_wake, |w| w.min(idle_wake)));
        }
        // Look again when Earth's answer is due, or at once after asking
        // with no time to wait (the fallback then applies).
        let mut earth = self.authority.next_deadline(now);
        if std::mem::take(&mut self.act_soon) {
            earth = Some(earth.map_or(now + 1.0, |t| t.min(now + 1.0)));
        }
        if let Some(t) = earth.filter(|&t| t > now) {
            wake = Some(wake.map_or(t, |w| w.min(t)));
        }
        (round.cmds, wake)
    }

    /// The covered part of the site now: the base station, and the relays
    /// ArcadiaOS has holding at their positions.
    fn coverage_now(&self) -> Coverage {
        let holding: Vec<(GridCoord, f64)> = self
            .exec
            .all_active()
            .filter_map(|a| {
                let Job::Relay { at } = a.job else {
                    return None;
                };
                let s = self.world.fleet.get(a.robot).state.as_ref()?;
                let holding = s.position == at
                    && s.status == RobotStatus::Working
                    && s.current_command.as_deref() == Some(a.command_id.as_str());
                let range = self.relay_range(a.robot);
                (holding && range > 0.0).then_some((at, range))
            })
            .collect();
        Coverage::new(
            &self.world.base_station,
            self.world.site.cell_size_m,
            &holding,
        )
    }

    /// Robot `r`'s relay range, m (0 without a relay capability that has one).
    fn relay_range(&self, r: usize) -> f64 {
        self.world
            .fleet
            .get(r)
            .spec
            .capability(CapabilityName::Relay)
            .and_then(|c| c.range_m)
            .filter(|&m| m.is_finite())
            .unwrap_or(0.0)
    }

    /// Routine actions the policy puts at level 3 or 4 without a standing approval.
    fn blocked_actions(&self) -> BTreeSet<&'static str> {
        let policy = &self.world.policy;
        let purpose = routine_purpose(policy);
        ROUTINE_ACTIONS
            .iter()
            .copied()
            .filter(|a| {
                authority::level(policy, a) >= 3
                    && self
                        .authority
                        .authorization(a, &purpose, self.world.now)
                        .is_none()
            })
            .collect()
    }

    /// The supply to spare for new station charging while the site's energy
    /// reserve is locked; `None` when it is not. The reserve is the bottom
    /// `bank_floor_frac` of the site bank. Charging that draws on it commits
    /// the reserve (`commit_energy_reserve`), which at level 3 or 4 waits for
    /// Earth's approval.
    fn reserve_lock(&self, cfg: &Config) -> Option<f64> {
        let p = self.world.power.as_ref()?;
        let low = p.storage_capacity_wh > 0.0
            && p.storage_wh <= cfg.bank_floor_frac * p.storage_capacity_wh;
        if !low
            || authority::level(&self.world.policy, "commit_energy_reserve") <= 2
            || self
                .authority
                .authorization("commit_energy_reserve", RESERVE, self.world.now)
                .is_some()
        {
            return None;
        }
        Some((p.solar_w + p.fission_w - p.demand_w).max(0.0))
    }

    /// Whether the capacity plan may be re-solved now. Re-solving for a
    /// changed set of working robots is a `replan`: notified at level 2,
    /// and at levels 3 and 4 held (the old plan stands) until Earth agrees.
    fn replan_allowed(&mut self, available: &[bool], cfg: &Config, diag: &Diag) -> bool {
        if !self.capacity.fleet_changed(available) {
            return true;
        }
        let changes: Vec<String> = (0..self.reg.len())
            .filter(|&r| self.capacity.solved_available.get(r) != available.get(r))
            .map(|r| {
                let id = &self.reg.robot(r).id;
                if available.get(r).copied().unwrap_or(false) {
                    format!("{id} back")
                } else {
                    format!("without {id}")
                }
            })
            .collect();
        let what = format!("roles re-planned {}", changes.join(", "));
        match authority::level(&self.world.policy, "replan") {
            1 => true,
            2 => {
                self.authority.note("replan", what, vec![]);
                true
            }
            _ => {
                let purpose = format!("replan {}", changes.join(", "));
                let now = self.world.now;
                if self
                    .authority
                    .authorization("replan", &purpose, now)
                    .is_some()
                {
                    return true;
                }
                if !self.authority.asked("replan", &purpose) {
                    let stakes = Stakes {
                        benefit_s: cfg.routine_benefit_s,
                        wait_cost_per_s: 1.0,
                        risk_s: 0.0,
                    };
                    self.ask_earth("replan", &purpose, what, stakes, cfg, diag);
                }
                false
            }
        }
    }

    /// Ask Earth about `action` for `purpose`. At level 3 the level-3 rule
    /// sets the deadline and the fallback (and may say asking is pointless);
    /// at level 4 only an approval counts, so the request always goes out,
    /// to be answered when Earth can. Returns whether a request went out.
    fn ask_earth(
        &mut self,
        action: &str,
        purpose: &str,
        summary: String,
        stakes: Stakes,
        cfg: &Config,
        diag: &Diag,
    ) -> bool {
        let now = self.world.now;
        let policy = self.world.policy.clone();
        let max_wait = policy.level3.max_wait_s.max(0.0);
        let reach = self.reach(cfg);
        let ask = if authority::level(&policy, action) >= 4 {
            let wait = reach.answer_in_s(now) * cfg.answer_margin.max(1.0);
            authority::Ask {
                decide_by_t: now + wait.min(max_wait),
                fallback: Fallback::Abort,
            }
        } else {
            match authority::level3_rule(&stakes, reach, now, max_wait, cfg.answer_margin) {
                Some(a) => a,
                None => return false,
            }
        };
        diag.info(now, || {
            format!(
                "asking Earth to {action} ({summary}): decide by {:.0}, fallback {:?}",
                ask.decide_by_t, ask.fallback
            )
        });
        self.authority
            .ask(&policy, action, purpose, summary, ask, now);
        if ask.decide_by_t <= now {
            self.act_soon = true;
        }
        true
    }

    /// Robot `r` was disabled during `task`, which other robots may now take
    /// over: `reassign_task`. Notified at level 2; at levels 3 and 4 the task
    /// is held for `r` until Earth agrees.
    fn released(&mut self, r: usize, task: &Task, reason: FailReason, cfg: &Config, diag: &Diag) {
        let what = format!(
            "{} ({},{}) released by {} after {reason:?}",
            task.stage.name(),
            task.cell().x,
            task.cell().y,
            self.reg.robot(r).id
        );
        match authority::level(&self.world.policy, "reassign_task") {
            1 => {}
            2 => self.authority.note("reassign_task", what, vec![]),
            _ => {
                let purpose = format!("reassign {:?}", task.key);
                self.held.insert(task.key, (r, purpose.clone()));
                if !self.authority.asked("reassign_task", &purpose) {
                    let stakes = Stakes {
                        benefit_s: cfg.routine_benefit_s,
                        wait_cost_per_s: 1.0,
                        risk_s: 0.0,
                    };
                    self.ask_earth("reassign_task", &purpose, what, stakes, cfg, diag);
                }
            }
        }
    }

    /// Step 9: what Earth hears from this decision. Level-2 commands are
    /// notified, one notify per action; level 3 and 4 commands carry their
    /// approval; standing approvals are asked for; and a status report goes
    /// out when one is due.
    fn report(&mut self, round: &mut Round, cfg: &Config, diag: &Diag) {
        let now = round.now;
        let policy = self.world.policy.clone();
        let purpose = routine_purpose(&policy);
        let mut notified: BTreeMap<&'static str, Vec<String>> = BTreeMap::new();
        for c in &mut round.cmds {
            let action = c.action.name();
            match authority::level(&policy, action) {
                1 => {}
                2 => notified
                    .entry(action)
                    .or_default()
                    .push(c.command_id.clone()),
                _ => {
                    if c.approval_id.is_none() {
                        c.approval_id = self.authority.authorization(action, &purpose, now);
                    }
                }
            }
        }
        for (action, ids) in notified {
            self.authority.notify_commands(action, ids);
        }
        for action in round.blocked.clone() {
            if !self.authority.asked(action, &purpose) {
                let level = authority::level(&policy, action);
                let stakes = Stakes {
                    benefit_s: cfg.routine_benefit_s,
                    wait_cost_per_s: 1.0,
                    risk_s: 0.0,
                };
                let summary = format!("standing approval to {action}, now level {level}");
                self.ask_earth(action, &purpose, summary, stakes, cfg, diag);
            }
        }
        // The energy reserve: ask before charging from it, or tell Earth.
        let bank_low = self.world.power.as_ref().is_some_and(|p| {
            p.storage_capacity_wh > 0.0
                && p.storage_wh <= cfg.bank_floor_frac * p.storage_capacity_wh
        });
        if round.reserve_wanted
            && !self
                .authority
                .pending("commit_energy_reserve", RESERVE, now)
        {
            let stakes = Stakes {
                benefit_s: cfg.routine_benefit_s,
                wait_cost_per_s: 1.0,
                risk_s: cfg.cannibalize_risk_s,
            };
            let summary = "charge robots from the last quarter of the site bank".to_string();
            if !self.authority.asked("commit_energy_reserve", RESERVE) {
                self.ask_earth("commit_energy_reserve", RESERVE, summary, stakes, cfg, diag);
            }
        }
        if bank_low
            && round.station_charges > 0
            && !self.reserve_noted
            && authority::level(&policy, "commit_energy_reserve") == 2
        {
            self.authority.note(
                "commit_energy_reserve",
                "charging from the last quarter of the site bank".into(),
                vec![],
            );
            self.reserve_noted = true;
        }
        if !bank_low {
            self.reserve_noted = false;
        }
        if self
            .authority
            .status_due(now, self.world.link_up(), cfg.status_period_s)
        {
            let (summary, progress) = self.status(cfg);
            diag.info(now, || format!("status report: {summary}"));
            self.authority.report_status(now, summary, progress);
        }
    }

    /// A status report: progress against the goal and the state of the crew.
    fn status(&self, cfg: &Config) -> (String, Vec<GoalProgress>) {
        let p = self
            .planner
            .as_ref()
            .map(|pl| pl.progress(&self.world, cfg))
            .unwrap_or_default();
        let n = self.reg.len();
        let count = |f: &dyn Fn(usize) -> bool| (0..n).filter(|&r| f(r)).count();
        let working = count(&|r| self.world.fleet.get(r).operable());
        let disabled = count(&|r| self.world.fleet.get(r).recoverable());
        let lost = count(&|r| self.world.fleet.get(r).lost());
        let sol = (self.world.now - self.world.start_t) / self.world.sol_length_s.max(1.0);
        let summary = format!(
            "sol {sol:.2}: pad {}/{} cells finished, {} verified; berm {}/{} confirmed; {working} robots working, {disabled} disabled, {lost} lost",
            p.finished, p.pad_cells, p.verified, p.berm_confirmed, p.berm_cells
        );
        let total = (p.pad_cells + p.berm_cells).max(1) as f64;
        let fraction = ((p.finished + p.berm_confirmed) as f64 / total).clamp(0.0, 1.0);
        let progress = self
            .planner
            .as_ref()
            .map(|pl| {
                vec![GoalProgress {
                    goal_id: pl.goal_id.clone(),
                    fraction_complete: fraction,
                }]
            })
            .unwrap_or_default();
        (summary, progress)
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
                || !self.in_contact(r)
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

    /// Whether the site can hear robot `r`: its snapshot is live, not the
    /// stale last-contact state of a robot out of coverage.
    fn in_contact(&self, r: usize) -> bool {
        self.world
            .fleet
            .get(r)
            .state
            .as_ref()
            .is_some_and(|s| s.in_relay_coverage)
    }

    /// Whether robot `r` could do any remaining work, were it working.
    fn has_work(&self, r: usize, workload: &Workload) -> bool {
        crate::registry::WORK_CAPABILITIES
            .iter()
            .any(|&cap| self.reg.can(r, cap) && workload.get(&cap).is_some_and(|w| w.steps > 0.0))
    }

    /// Cancel robot `r`'s interruptible job so it can take another.
    fn free_up(&mut self, round: &mut Round, r: usize, diag: &Diag) {
        if round.blocked.contains("cancel") {
            return;
        }
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

    /// Step 3: bring disabled robots back, most needed first. Robots out of
    /// contact are left alone: their snapshots are stale.
    fn recover(&mut self, round: &mut Round, cfg: &Config, diag: &Diag) {
        let now = round.now;
        let n = self.reg.len();
        let mut targets: Vec<(bool, usize)> = (0..n)
            .filter(|&r| self.world.fleet.get(r).recoverable())
            .filter(|&r| self.in_contact(r))
            .filter(|&r| !self.exec.robot_cooling(r, now))
            .map(|r| (self.has_work(r, &round.workload), r))
            .collect();
        // Robots whose capabilities still have work come first.
        targets.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
        let strip_level = authority::level(&self.world.policy, "cannibalize");
        for (_, x) in targets {
            // A robot about to be stripped for parts is not worth recovering.
            if self.strip_live(|d, _| d == x) {
                continue;
            }
            let engaged = self.exec.engaged();
            let running: BTreeSet<usize> = self.exec.all_active().map(|a| a.robot).collect();
            let awaited = self.energy.reserved.values().any(|&k| k == x);
            // A helper on an interruptible job is freed with a cancel, if the policy allows one.
            let can_cancel = !round.blocked.contains("cancel");
            let mut helpers: Vec<usize> = round
                .free
                .iter()
                .chain(round.interruptible.iter().filter(|_| can_cancel))
                .copied()
                .filter(|&h| h != x)
                .collect();
            helpers.sort_unstable();
            helpers.dedup();
            let capacity = cfg.depot_capacity;
            let depot_wait = |d: usize, t: f64| round.depot_wait(d, capacity, t);
            let blocked = &round.blocked;
            let may = |action: &str| !blocked.contains(action);
            let (authority, reg) = (&self.authority, &self.reg);
            let may_strip = |d: usize| {
                strip_level <= 2
                    || authority
                        .authorization("cannibalize", &strip_purpose(reg, d, x), now)
                        .is_some()
            };
            let (plan, ask) = {
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
                    may: &may,
                    may_strip: &may_strip,
                };
                let plan = recovery::plan(&ctx, x, &helpers);
                let ask = (strip_level >= 3)
                    .then(|| recovery::best_cannibalize(&ctx, x, &helpers))
                    .flatten()
                    .filter(|c| plan.as_ref().is_none_or(|p| c.cost_s < p.cost_s));
                (plan, ask)
            };
            // Parts from a donor would beat everything allowed: ask Earth, or
            // wait for its answer, unless the level-3 rule says not to bother.
            if let Some(c) = ask
                && self.weigh_cannibalize(x, plan.as_ref(), &c, cfg, diag)
            {
                continue;
            }
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

    /// Cannibalizing `c` would bring robot `x` back sooner than `alt`, the
    /// best option allowed now, but the policy needs Earth for it. Returns
    /// whether to hold `x` for Earth: just asked, or a request is pending.
    /// The level-3 rule weighs the time saved against waiting and against
    /// the risk of stripping a robot Earth would rather keep.
    fn weigh_cannibalize(
        &mut self,
        x: usize,
        alt: Option<&recovery::Plan>,
        c: &recovery::Plan,
        cfg: &Config,
        diag: &Diag,
    ) -> bool {
        let now = self.world.now;
        let recovery::Step::Cannibalize { donor, .. } = c.step else {
            return false;
        };
        let purpose = strip_purpose(&self.reg, donor, x);
        if self.authority.pending("cannibalize", &purpose, now) {
            return true;
        }
        // Never strip a robot that is itself waiting for parts.
        if self.strip_live(|_, t| t == donor) {
            return false;
        }
        // Answered, or lapsed: what is allowed now stands.
        if self.authority.asked("cannibalize", &purpose) {
            return false;
        }
        let weight = self.capacity.utilization(x).clamp(0.2, 1.0);
        let alt_s = alt.map_or(cfg.no_alternative_s, |p| p.cost_s);
        let stakes = Stakes {
            benefit_s: (alt_s - c.cost_s).max(0.0) * weight,
            wait_cost_per_s: weight,
            risk_s: cfg.cannibalize_risk_s,
        };
        let max_wait = self.world.policy.level3.max_wait_s;
        let reach = self.reach(cfg);
        let ask = match authority::level3_rule(&stakes, reach, now, max_wait, cfg.answer_margin) {
            Some(a) if authority::level(&self.world.policy, "cannibalize") == 3 => a,
            // Level 4: only worth asking if an answer can come, and never on a fallback.
            Some(a) if a.decide_by_t > now => authority::Ask {
                fallback: Fallback::Abort,
                ..a
            },
            _ => return false,
        };
        let summary = format!(
            "cannibalize {} to restore {}: back in about {:.1} h instead of {:.1} h",
            self.reg.robot(donor).id,
            self.reg.robot(x).id,
            c.cost_s / 3600.0,
            alt_s / 3600.0
        );
        diag.info(now, || {
            format!(
                "asking Earth: {summary} (decide by {:.0}, fallback {:?})",
                ask.decide_by_t, ask.fallback
            )
        });
        let policy = self.world.policy.clone();
        self.authority
            .ask(&policy, "cannibalize", &purpose, summary, ask, now);
        self.strips.push((donor, x));
        if ask.decide_by_t <= now {
            self.act_soon = true;
        }
        true
    }

    /// Whether a strip ArcadiaOS asked about, matching `f(donor, target)`, is
    /// still pending or authorized.
    fn strip_live(&self, f: impl Fn(usize, usize) -> bool) -> bool {
        let now = self.world.now;
        self.strips.iter().any(|&(d, t)| {
            let purpose = strip_purpose(&self.reg, d, t);
            f(d, t)
                && (self.authority.pending("cannibalize", &purpose, now)
                    || self
                        .authority
                        .authorization("cannibalize", &purpose, now)
                        .is_some())
        })
    }

    /// How fast Earth can answer now: a round trip at the reported delay plus
    /// mission control's typical response time.
    fn reach(&self, cfg: &Config) -> Reach {
        let d = self.world.one_way_delay_s(cfg.assumed_delay_s);
        let answer_s = 2.0 * d + cfg.earth_response_s;
        if self.world.link_up() {
            Reach::Up { answer_s }
        } else {
            Reach::Down {
                back_t: self.world.comms.as_ref().and_then(|c| c.expected_restore_t),
                answer_s,
            }
        }
    }

    /// Send helper `h` to charge up to `need_wh` (and a little more).
    fn charge_for(&mut self, round: &mut Round, h: usize, need_wh: f64, cfg: &Config, diag: &Diag) {
        if round.blocked.contains("charge") {
            return;
        }
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
                        may: &|_| true,
                        may_strip: &|_| false,
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
                let mut cmd = self.exec.issue(executor, &id, job, action, s);
                // The approval Earth gave (or the fallback that fired) for this very strip.
                cmd.approval_id = self.authority.authorization(
                    "cannibalize",
                    &strip_purpose(&self.reg, donor, x),
                    now,
                );
                round.cmds.push(cmd);
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
                    let Some((_, h, work_s)) = repair.filter(|_| !round.blocked.contains("repair"))
                    else {
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
                    let Some((_, d, est)) = service.filter(|_| !round.blocked.contains("service"))
                    else {
                        continue;
                    };
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

    /// Step 5: station a relay where coverage is wanted (work that would end
    /// out of base-station coverage, or a robot out of contact well past its
    /// expected return) when the robot time it saves beats what the relay
    /// robot gives up; stand it down when nothing needs it any more, or when
    /// its own battery runs low.
    fn relay(&mut self, round: &mut Round, tasks: &[Task], cfg: &Config, diag: &Diag) {
        let now = round.now;
        let cs = self.world.site.cell_size_m;
        let base = Coverage::new(&self.world.base_station, cs, &[]);
        let (w, h) = (self.world.site.width, self.world.site.height);
        let from = self.world.base_station.position;
        // Demand: cells wanting coverage, with the robot time coverage saves there.
        let mut demand: Vec<(GridCoord, f64)> = Vec::new();
        for t in tasks {
            if round.blocked.contains(stage_action(t.stage)) {
                continue;
            }
            let uncovered = match t.stage {
                Stage::Survey | Stage::Verify | Stage::BermCheck | Stage::Recheck => {
                    crate::coverage::safe_scan_area(&t.area, from, &base, w, h, 6).is_none()
                }
                _ => !base.covers(t.cell()),
            };
            let doable = self
                .reg
                .capable(t.capability())
                .iter()
                .any(|&r| self.world.fleet.get(r).operable() && self.in_contact(r));
            if uncovered && doable {
                demand.push((
                    geom::clamp_into(from, &t.area),
                    cfg.lost_link_timeout_s * 0.5,
                ));
            }
        }
        for r in 0..self.reg.len() {
            let e = self.world.fleet.get(r);
            let Some(state) = e.state.as_ref() else {
                continue;
            };
            if e.lost() || state.in_relay_coverage {
                continue;
            }
            let busy_until = self
                .exec
                .active(r)
                .map_or(state.telemetry_t, |a| a.est_end_t.max(state.telemetry_t));
            if now > busy_until + cfg.lost_link_timeout_s + cfg.stranded_margin_s {
                demand.push((state.position, cfg.no_alternative_s / 4.0));
            }
        }
        let active = self.exec.all_active().find_map(|a| match a.job {
            Job::Relay { at } => Some((a.robot, at)),
            _ => None,
        });
        if let Some((rr, at)) = active {
            let range = self.relay_range(rr);
            let serving = demand
                .iter()
                .any(|&(c, _)| crate::coverage::within(c, at, range, cs))
                || self.relay_dependents(rr, &base);
            let low = Energy::low(&self.world, &self.reg, rr, cfg);
            if (!serving || low)
                && !round.blocked.contains("cancel")
                && self.in_contact(rr)
                && let Some((cancel, _)) = self.exec.cancel(rr, &self.reg.robot(rr).id)
            {
                diag.info(now, || {
                    format!(
                        "{}: relay at ({},{}) stood down{}",
                        self.reg.robot(rr).id,
                        at.x,
                        at.y,
                        if low { " to charge" } else { "" }
                    )
                });
                round.cmds.push(cancel);
                round.take(rr);
            }
            return;
        }
        if demand.is_empty() || round.blocked.contains("relay") {
            return;
        }
        // The best relay robot and position: as much demand served as
        // possible from a covered cell, then the shortest drive.
        let covered: Vec<GridCoord> = (0..h)
            .flat_map(|y| (0..w).map(move |x| GridCoord { x, y }))
            .filter(|&c| base.covers(c))
            .collect();
        let mut pick: Option<(f64, f64, usize, GridCoord)> = None;
        for &r in &round.free {
            let range = self.relay_range(r);
            let Some(pos) = self.world.fleet.get(r).position() else {
                continue;
            };
            if range <= 0.0 || Energy::low(&self.world, &self.reg, r, cfg) {
                continue;
            }
            for &c in &covered {
                let served: f64 = demand
                    .iter()
                    .filter(|&&(d, _)| crate::coverage::within(d, c, range, cs))
                    .map(|&(_, b)| b)
                    .sum();
                let travel = self.reg.travel_s(r, pos, c);
                if served > 0.0
                    && pick.is_none_or(|(s, t, _, _)| {
                        served > s + 1e-9 || (served >= s - 1e-9 && travel < t - 1e-9)
                    })
                {
                    pick = Some((served, travel, r, c));
                }
            }
        }
        let Some((served, travel, r, at)) = pick else {
            return;
        };
        // The relay robot gives up its own work while it holds.
        let hold_s = demand.len() as f64 * cfg.lost_link_timeout_s * 0.5;
        let cost = travel + self.capacity.utilization(r) * hold_s;
        let pos = self.world.fleet.get(r).position().unwrap_or(at);
        let hold_wh = self.reg.idle_wh(r, hold_s.max(7200.0))
            + cfg.relay_radio_w * hold_s.max(7200.0) / 3600.0;
        let affordable = Energy::affordable(
            &self.world,
            &self.reg,
            r,
            self.reg.travel_wh(r, pos, at) + hold_wh,
            at,
            cfg,
        );
        if served < cfg.relay_benefit_ratio * cost || !affordable {
            return;
        }
        diag.info(now, || {
            format!(
                "{} relays at ({},{}): saves about {:.0} s against {:.0} s",
                self.reg.robot(r).id,
                at.x,
                at.y,
                served,
                cost
            )
        });
        let planned = drive_use(&self.reg, r, pos, at);
        let s = sent(&self.world, r, planned.drive_s, planned);
        let id = self.reg.robot(r).id.clone();
        round.cmds.push(self.exec.issue(
            r,
            &id,
            Job::Relay { at },
            Action::Relay { position: at },
            s,
        ));
        round.take(r);
    }

    /// Whether anything but relay `rr` depends on coverage beyond the base
    /// station's: a robot whose job ends out there, or one out there now.
    fn relay_dependents(&self, rr: usize, base: &Coverage) -> bool {
        let ends_beyond = self.exec.all_active().any(|a| {
            a.robot != rr
                && match &a.job {
                    Job::Work { plan, .. } => !base.covers(plan.end),
                    Job::Move { to, .. } | Job::Tow { to, .. } => !base.covers(*to),
                    _ => false,
                }
        });
        let out_there = (0..self.reg.len()).any(|r| {
            r != rr
                && self.in_contact(r)
                && self
                    .world
                    .fleet
                    .get(r)
                    .position()
                    .is_some_and(|p| !base.covers(p))
        });
        ends_beyond || out_there
    }

    /// Step 6: robots at their low-water mark charge before anything else.
    /// Returns the robots that cannot reach any charger.
    fn charge_low(&mut self, round: &mut Round, cfg: &Config, diag: &Diag) -> Vec<usize> {
        let mut stranded = Vec::new();
        if round.blocked.contains("charge") {
            return stranded;
        }
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
        if round.blocked.contains("move_to") {
            return false;
        }
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

    /// Step 7: the auction over free robots and interruptible jobs.
    fn auction(
        &mut self,
        round: &mut Round,
        ready: &[Task],
        stranded: &[usize],
        cfg: &Config,
        diag: &Diag,
    ) -> AuctionResult {
        let now = round.now;
        let Some(planner) = self.planner.as_ref() else {
            return AuctionResult::default();
        };
        let busy = self.exec.busy();
        // Work whose command the policy holds back is not offered.
        let tasks: Vec<Task> = ready
            .iter()
            .filter(|t| !round.blocked.contains(stage_action(t.stage)))
            .cloned()
            .collect();
        let can_cancel = !round.blocked.contains("cancel");
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
            .chain(
                round
                    .interruptible
                    .iter()
                    .filter(|_| can_cancel)
                    .map(|&robot| Bidder {
                        robot,
                        roles_only: true,
                    }),
            )
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
                coverage: &self.coverage,
            };
            let (exec, held, authority) = (&self.exec, &self.held, &self.authority);
            // A task held for the robot that had it may go to no other until Earth agrees.
            let withheld = |r: usize, k: TaskKey| {
                held.get(&k).is_some_and(|(owner, purpose)| {
                    *owner != r
                        && authority
                            .authorization("reassign_task", purpose, now)
                            .is_none()
                })
            };
            allocator::auction(&ctx, &bidders, &tasks, &|r, k| {
                exec.cooling(r, k, now) || withheld(r, k)
            })
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
            if (blocked || opportunistic) && !round.blocked.contains("charge") {
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
                if blocked && !rescue_underway && !round.blocked.contains("move_to") {
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
                && !round.blocked.contains("charge")
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
            if idle_work
                && !blocked
                && !round.blocked.contains("charge")
                && self.park(round, r, pos, battery, cfg, diag)
            {
                continue;
            }
            // Wait near the work front if this robot works the pad surface.
            let Some(to) = front else { continue };
            let works_front = FRONT_CAPABILITIES
                .iter()
                .any(|&c| self.capacity.is_role(r, c, cfg.role_min_share));
            if !works_front || geom::dist(pos, to) <= 3.0 || round.blocked.contains("move_to") {
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
        if round.blocked.contains("cancel") {
            return;
        }
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

/// Approval purpose for stripping `donor` to restore `target`.
fn strip_purpose(reg: &Registry, donor: usize, target: usize) -> String {
    format!("strip {} for {}", reg.robot(donor).id, reg.robot(target).id)
}

/// Approval purpose for routine actions under `policy`: a standing approval
/// holds until the policy changes.
fn routine_purpose(policy: &arcadia_protocol::Policy) -> String {
    format!("{ROUTINE} under policy {}", policy.policy_version)
}

/// The robot command a task's stage is carried out with.
fn stage_action(stage: Stage) -> &'static str {
    match stage {
        Stage::Survey | Stage::Verify | Stage::BermCheck | Stage::Recheck => "scan",
        Stage::Dig => "dig",
        Stage::Fill | Stage::Berm => "haul",
        Stage::Grade => "grade",
        Stage::Compact => "compact",
        Stage::Sinter => "sinter",
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
    // Charging from the site's energy reserve waits for Earth's approval.
    if let ChargeSource::Station(s) = plan.source
        && let Some(spare) = round.reserve_spare_w.as_mut()
    {
        let w = reg.stations()[s].slot_power_w;
        if *spare < w {
            round.reserve_wanted = true;
            return;
        }
        *spare -= w;
    }
    match plan.source {
        ChargeSource::Station(s) => {
            round.load.add(s, now + plan.est_s);
            round.charging_w += reg.stations()[s].slot_power_w;
            round.station_charges += 1;
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
    use arcadia_protocol::{CommsCause, Downlink, Link};

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

    // ----- authority, Earth and coverage -----

    fn level(l: u8) -> AuthorityLevel {
        AuthorityLevel::try_from(l).unwrap()
    }

    fn comms(link: Link, cause: CommsCause) -> Event {
        Event {
            t: 0.0,
            kind: EventKind::Comms {
                link,
                cause,
                one_way_delay_s: 720.0,
                expected_restore_t: None,
            },
        }
    }

    fn earth(t: f64, body: Uplink) -> Event {
        Event {
            t,
            kind: EventKind::EarthMessage {
                sent_t: t - 720.0,
                message: EarthToSite {
                    message_id: format!("earth-{t}"),
                    body,
                },
            },
        }
    }

    fn notified(d: &Decision, action: &str) -> Vec<String> {
        d.to_earth
            .iter()
            .filter_map(|m| match &m.body {
                Downlink::Notify {
                    action: a,
                    commands: Some(c),
                    ..
                } if a == action => Some(c.clone()),
                _ => None,
            })
            .flatten()
            .collect()
    }

    /// The crew of `crew_hello` with towing at level 2.
    fn crew_os_with_tow_at(l: u8) -> Coordinator {
        let mut h = crew_hello();
        h.scenario.policy.levels.insert("tow".into(), level(l));
        let mut os = Coordinator::with_config(Config::default(), Diag::new(Level::Off));
        os.start(h);
        os
    }

    #[test]
    fn a_level_2_tow_is_notified_in_the_decision_that_orders_it() {
        let mut os = crew_os_with_tow_at(2);
        let mut worker = state("worker", 12, 12);
        worker.status = RobotStatus::Failed;
        worker.faults = vec![fault("tool", Severity::Disabled, false)];
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal()],
            robots: vec![worker, state("scout", 6, 6), state("hauler", 8, 8)],
        });
        let tow = command_for(&d, "hauler").expect("the hauler tows");
        assert!(matches!(tow.action, Action::Tow { .. }));
        assert_eq!(notified(&d, "tow"), vec![tow.command_id.clone()]);
        // The plan is reported to Earth from the first decision.
        assert!(d
            .to_earth
            .iter()
            .any(|m| matches!(&m.body, Downlink::StatusReport { progress: Some(p), .. } if p[0].goal_id == "pad")));
        assert_eq!(os.authority().unwrap().counts.notifies, 1);
    }

    #[test]
    fn a_policy_update_counts_from_when_it_arrives() {
        let mut os = crew_os_with_tow_at(2);
        let mut worker = state("worker", 12, 12);
        worker.status = RobotStatus::Failed;
        worker.faults = vec![fault("tool", Severity::Disabled, false)];
        let mut loose = os.world().unwrap().policy.clone();
        loose.policy_version = 2;
        loose.levels.insert("tow".into(), level(1));
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal(), earth(0.0, Uplink::PolicyUpdate { policy: loose })],
            robots: vec![worker, state("scout", 6, 6), state("hauler", 8, 8)],
        });
        assert!(command_for(&d, "hauler").is_some_and(|c| matches!(c.action, Action::Tow { .. })));
        assert!(notified(&d, "tow").is_empty(), "towing is level 1 now");
        assert_eq!(os.world().unwrap().policy.policy_version, 2);
        // Reprioritizing the one goal is recorded and changes nothing else.
        let d = os.decide(&Observation {
            seq: 1,
            t: 10.0,
            events: vec![earth(
                10.0,
                Uplink::Reprioritize {
                    priorities: vec![GoalPriority {
                        goal_id: "pad".into(),
                        priority: 5,
                    }],
                },
            )],
            robots: vec![
                state("worker", 12, 12),
                state("scout", 6, 6),
                state("hauler", 8, 8),
            ],
        });
        assert_eq!(d.seq, 1);
        assert_eq!(os.authority().unwrap().priorities.get("pad"), Some(&5));
    }

    /// A sinterer and a spare one, both failed beyond field repair, a third
    /// sinterer working, and a scout that can strip and fit parts. Nothing
    /// can tow and there is no depot, so parts from the spare are the only
    /// way to bring the sinterer back.
    fn strip_os(cannibalize: u8, cfg: Config) -> (Coordinator, Vec<RobotState>) {
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
        sc.site.width = 30;
        sc.site.height = 30;
        sc.site.cell_size_m = 1.0;
        sc.policy
            .levels
            .insert("cannibalize".into(), level(cannibalize));
        sc.policy.level3.max_wait_s = 3600.0;
        let mut h = hello();
        h.scenario = sc;
        let mut os = Coordinator::with_config(cfg, Diag::new(Level::Off));
        os.start(h);
        let mut worker = state("worker", 12, 12);
        worker.status = RobotStatus::Failed;
        worker.faults = vec![fault("tool", Severity::Disabled, false)];
        let mut spare = state("spare", 13, 12);
        spare.status = RobotStatus::Failed;
        spare.faults = vec![fault("wheels", Severity::Disabled, false)];
        let robots = vec![
            worker,
            state("scout", 12, 13),
            spare,
            state("other", 20, 20),
        ];
        (os, robots)
    }

    fn approval_request(d: &Decision) -> Option<(String, Option<f64>, Option<Fallback>)> {
        d.to_earth.iter().find_map(|m| match &m.body {
            Downlink::ApprovalRequest {
                action,
                decide_by_t,
                fallback,
                ..
            } if action == "cannibalize" => Some((m.message_id.clone(), *decide_by_t, *fallback)),
            _ => None,
        })
    }

    fn strip_command(d: &Decision) -> Option<&Command> {
        d.commands
            .iter()
            .find(|c| matches!(c.action, Action::Cannibalize { .. }))
    }

    /// Completion events at `t` for every command in `d`.
    fn completions(d: &Decision, t: f64) -> Vec<Event> {
        d.commands
            .iter()
            .map(|c| Event {
                t,
                kind: EventKind::CommandCompleted {
                    command_id: c.command_id.clone(),
                    robot_id: c.robot_id.clone(),
                    energy_used_wh: 0.0,
                },
            })
            .collect()
    }

    #[test]
    fn cannibalizing_asks_earth_and_follows_its_answer() {
        let (mut os, robots) = strip_os(3, Config::default());
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal(), comms(Link::Up, CommsCause::Nominal)],
            robots: robots.clone(),
        });
        let (id, decide_by_t, _) = approval_request(&d).expect("Earth is asked");
        // A round trip and Earth's thinking time, with margin, within the policy's hour.
        assert_eq!(decide_by_t, Some(3600.0));
        assert!(strip_command(&d).is_none(), "not before Earth answers");
        // Asked once, not again while waiting (the scout finishes its survey meanwhile).
        let d = os.decide(&Observation {
            seq: 1,
            t: 600.0,
            events: completions(&d, 600.0),
            robots: robots.clone(),
        });
        assert!(approval_request(&d).is_none() && strip_command(&d).is_none());
        // Earth approves: the strip goes out citing the request.
        let mut events = completions(&d, 2600.0);
        events.push(earth(
            2600.0,
            Uplink::Approve {
                request_id: id.clone(),
                note: None,
            },
        ));
        let d = os.decide(&Observation {
            seq: 2,
            t: 2600.0,
            events,
            robots: robots.clone(),
        });
        let strip = strip_command(&d).expect("the scout strips the spare");
        assert_eq!(strip.robot_id, "scout");
        assert_eq!(strip.approval_id, Some(id));
        assert_eq!(
            strip.action,
            Action::Cannibalize {
                target_robot: "spare".into(),
                beneficiary_robot: Some("worker".into())
            }
        );
        assert_eq!(os.authority().unwrap().counts.approvals, 1);

        // Vetoed: nothing is stripped.
        let (mut os, robots) = strip_os(3, Config::default());
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal(), comms(Link::Up, CommsCause::Nominal)],
            robots: robots.clone(),
        });
        let (id, _, _) = approval_request(&d).unwrap();
        let mut events = completions(&d, 2600.0);
        events.push(earth(
            2600.0,
            Uplink::Veto {
                request_id: id,
                note: None,
            },
        ));
        let d = os.decide(&Observation {
            seq: 1,
            t: 2600.0,
            events,
            robots,
        });
        assert!(strip_command(&d).is_none());
    }

    #[test]
    fn in_a_blackout_the_fallback_decides_without_waiting() {
        // Low risk: the rule's fallback is to proceed.
        let cfg = Config {
            cannibalize_risk_s: 0.0,
            ..Config::default()
        };
        let (mut os, robots) = strip_os(3, cfg.clone());
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal(), comms(Link::Down, CommsCause::Conjunction)],
            robots: robots.clone(),
        });
        let (id, decide_by_t, fallback) = approval_request(&d).expect("Earth is told");
        assert_eq!(
            (decide_by_t, fallback),
            (Some(0.0), Some(Fallback::Proceed))
        );
        assert_eq!(d.wake_at, Some(1.0), "look again at once");
        let d = os.decide(&Observation {
            seq: 1,
            t: 1.0,
            events: completions(&d, 1.0),
            robots: robots.clone(),
        });
        assert_eq!(
            strip_command(&d).and_then(|c| c.approval_id.clone()),
            Some(id)
        );

        // At level 4 nothing is ever done without Earth: no request it could
        // not answer in time, and no strip.
        let (mut os, robots) = strip_os(4, cfg);
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal(), comms(Link::Down, CommsCause::Conjunction)],
            robots: robots.clone(),
        });
        assert!(approval_request(&d).is_none() && strip_command(&d).is_none());
        let d = os.decide(&Observation {
            seq: 1,
            t: 1.0,
            events: completions(&d, 1.0),
            robots,
        });
        assert!(strip_command(&d).is_none());
    }

    #[test]
    fn robots_out_of_contact_are_never_commanded() {
        let mut os = crew_os();
        // The hauler's snapshot is stale: it is out of coverage.
        let mut hauler = state("hauler", 25, 25);
        hauler.in_relay_coverage = false;
        hauler.telemetry_t = -100.0;
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal()],
            robots: vec![state("worker", 6, 7), state("scout", 6, 6), hauler],
        });
        assert!(command_for(&d, "hauler").is_none());
        assert!(
            command_for(&d, "scout").is_some(),
            "the scout still surveys"
        );
    }

    #[test]
    fn a_relay_is_stationed_when_work_lies_beyond_coverage() {
        let mut h = crew_hello();
        // Only the station's corner is covered: the pad cannot be scanned from inside coverage.
        h.scenario.base_station.range_m = 1.0;
        h.scenario.robots.push(robot(
            "radio",
            0.5,
            vec![CapabilitySpec {
                name: CapabilityName::Relay,
                rate: 0.0,
                energy_wh_per_unit: 0.0,
                capacity: None,
                range_m: Some(30.0),
            }],
        ));
        let mut os = Coordinator::with_config(Config::default(), Diag::new(Level::Off));
        os.start(h);
        let d = os.decide(&Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal()],
            robots: vec![
                state("worker", 1, 1),
                state("scout", 1, 2),
                state("hauler", 2, 1),
                state("radio", 2, 2),
            ],
        });
        let relay = command_for(&d, "radio").expect("the relay robot is sent out");
        let Action::Relay { position } = relay.action else {
            panic!("{:?}", relay.action)
        };
        // It holds inside coverage, reaching the pad from there.
        assert!(geom::dist(position, GridCoord { x: 0, y: 0 }) <= 1.0);
    }
}
