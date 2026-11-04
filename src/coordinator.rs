//! The coordinator: one decision per observation.
//!
//! Each observation is folded into the world model (robot states, then events
//! in order: command outcomes, scan results, environment reports, Earth
//! messages). Then the decision is built:
//!
//! 1. refresh the capacity plan when the work or the fleet has changed;
//! 2. derive the ready tasks from the plan (dependency order per cell);
//! 3. send robots below their energy low-water mark to charge;
//! 4. run the auction over free robots (and robots on interruptible jobs);
//! 5. give leftover robots a charge, an opportunistic top-up or a move
//!    towards upcoming work;
//! 6. ask for a wake-up when something time-based is pending.
//!
//! Extension points: week 5 hooks fault response (repair, tow, service,
//! cannibalize, predictive maintenance) into [`State::on_outcome`] and the
//! `robot_fault` handling, and energy-aware timing into [`State::dispatch`];
//! week 6 adds an authority gate where commands are issued, Earth messages in
//! [`State::on_uplink`], and coverage-aware planning.

use crate::allocator::{self, Bidder, Context};
use crate::capacity::CapacityPlan;
use crate::config::Config;
use crate::diag::Diag;
use crate::energy::{ChargeSource, Energy, StationLoad};
use crate::executor::{Assignment, Executor, Job, Outcome};
use crate::geom;
use crate::planner::{Planner, SpoilKind};
use crate::registry::Registry;
use crate::tasks::{Stage, Task};
use crate::world::WorldModel;
use arcadia_protocol::{
    Action, CapabilityName, Command, Decision, EarthToSite, End, Event, EventKind, FailReason,
    Hello, Objective, Observation, RejectReason, Uplink,
};
use std::collections::BTreeMap;

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
    same_time: (f64, u32),
    decisions: u64,
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
                "run {}: {} robots, {} stations, site {}x{} cells of {} m",
                hello.run_id,
                reg.len(),
                world.stations.len(),
                world.site.width,
                world.site.height,
                world.site.cell_size_m
            )
        });
        self.state = Some(State {
            energy: Energy::new(&reg),
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
                ..
            } => {
                diag.info(e.t, || {
                    format!("{robot_id}: {severity:?} fault in {component}")
                });
            }
            _ => {
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
        match &outcome {
            Outcome::Completed { .. } => {
                diag.debug(t, || format!("{id}: done {}", a.job.label()));
            }
            Outcome::Failed { reason, detail } if *reason != FailReason::Cancelled => {
                diag.info(t, || {
                    format!(
                        "{id}: {} failed: {reason:?} {}",
                        a.job.label(),
                        detail.as_deref().unwrap_or("")
                    )
                });
            }
            Outcome::Rejected { reason, detail } => {
                diag.info(t, || {
                    format!(
                        "{id}: {} rejected: {reason:?} {}",
                        a.job.label(),
                        detail.as_deref().unwrap_or("")
                    )
                });
            }
            Outcome::Failed { .. } => {}
        }
        // Refused again and again, for whatever reason: give the robot a rest.
        if matches!(outcome, Outcome::Rejected { .. })
            && self.exec.rejection_streak(r) >= REJECTION_STREAK_LIMIT
        {
            self.exec.back_off_robot(r, t + cfg.retry_backoff_s / 5.0);
        }
        let done = matches!(outcome, Outcome::Completed { .. });
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
                    let cancelled = matches!(
                        outcome,
                        Outcome::Failed {
                            reason: FailReason::Cancelled,
                            ..
                        }
                    );
                    if !cancelled {
                        self.exec.back_off_robot(r, t + 60.0);
                    }
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
        self.exec.back_off(r, task.key, t + cfg.retry_backoff_s);
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
        }
    }

    fn dispatch(&mut self, cfg: &Config, diag: &Diag) -> (Vec<Command>, Option<f64>) {
        let now = self.world.now;
        let mut cmds = Vec::new();
        let next_cooldown = self.exec.prune_cooldowns(now);
        if self.same_time.1 > cfg.max_same_time_decisions {
            diag.info(now, || "many decisions at one sim time; pausing".into());
            return (cmds, Some(now + 60.0));
        }
        let Self {
            world,
            reg,
            planner,
            capacity,
            capacity_dirty,
            energy,
            exec,
            ..
        } = self;
        let Some(planner) = planner.as_mut() else {
            return (cmds, None);
        };

        // Everything looks done but the run goes on: something is off.
        let progress = planner.progress(world, cfg);
        let recheck_in_flight = exec
            .all_active()
            .any(|a| matches!(&a.job, Job::Work { task, .. } if task.stage == Stage::Recheck));
        if progress.complete() && !planner.recheck.wanted && !recheck_in_flight {
            diag.info(now, || {
                "pad looks complete but the run continues; rechecking".into()
            });
            planner.request_recheck();
        }

        let n = reg.len();
        let available: Vec<bool> = (0..n).map(|r| world.fleet.get(r).operable()).collect();
        let workload = planner.workload(world, reg, cfg);
        if *capacity_dirty || capacity.stale(&workload, &available) {
            *capacity = CapacityPlan::solve(&workload, reg, &available, cfg);
            *capacity_dirty = false;
            diag.info(now, || describe_capacity(capacity, reg));
        }

        // Release mobile chargers whose client no longer needs them.
        let reserved: Vec<(usize, usize)> = energy.reserved.iter().map(|(&c, &k)| (c, k)).collect();
        for (c, client) in reserved {
            let client_ok = world.fleet.get(client).operable();
            let charging = matches!(
                exec.active(client).map(|a| &a.job),
                Some(Job::Charge { plan, .. }) if plan.source == ChargeSource::Mobile(c)
            );
            let charger_moving = exec.active(c).is_some();
            if !client_ok || (!charging && !charger_moving && !Energy::low(world, reg, client, cfg))
            {
                energy.reserved.remove(&c);
            }
        }

        let busy = exec.busy();
        let tasks = planner.tasks(world, &busy, cfg);
        let mut crowd: BTreeMap<(CapabilityName, usize), usize> = BTreeMap::new();
        let mut load = StationLoad::default();
        for a in exec.all_active() {
            match &a.job {
                Job::Work { task, .. } => {
                    if let Some(s) = task.section {
                        *crowd.entry((task.capability(), s)).or_default() += 1;
                    }
                }
                Job::Charge { plan, .. } => {
                    if let ChargeSource::Station(s) = plan.source {
                        load.add(s, a.est_end_t);
                    }
                }
                Job::Move { .. } => {}
            }
        }

        let mut free = Vec::new();
        let mut interruptible = Vec::new();
        for r in 0..n {
            let e = world.fleet.get(r);
            if !e.operable() || exec.robot_cooling(r, now) || energy.reserved.contains_key(&r) {
                continue;
            }
            // A command this coordinator does not know about is still running.
            let unknown_command = e
                .state
                .as_ref()
                .is_some_and(|s| s.current_command.is_some());
            match exec.active(r) {
                None if unknown_command => {}
                None => free.push(r),
                Some(a) if a.interruptible() => interruptible.push(r),
                Some(_) => {}
            }
        }

        // 1. Robots at their low-water mark charge before anything else.
        let mut stranded = Vec::new();
        for r in free.clone() {
            if !Energy::low(world, reg, r, cfg) {
                continue;
            }
            let chargers = chargers_for(r, &free, energy, exec, reg);
            let need = capacity.energy_need_wh(r, &workload, reg, cfg);
            let until = Energy::charge_target(world, reg, r, Some(need), cfg);
            match energy.plan_charge(world, reg, r, until, &load, &chargers) {
                Some(plan) => {
                    issue_charge(
                        world, reg, energy, exec, &mut load, &mut free, r, plan, false, &mut cmds,
                        diag,
                    );
                }
                None => stranded.push(r),
            }
        }

        // 2. The auction.
        let bidders: Vec<Bidder> = free
            .iter()
            .filter(|r| !stranded.contains(r))
            .map(|&robot| Bidder {
                robot,
                roles_only: false,
            })
            .chain(interruptible.iter().map(|&robot| Bidder {
                robot,
                roles_only: true,
            }))
            .collect();
        let result = {
            let ctx = Context {
                world,
                reg,
                planner,
                capacity,
                cfg,
                busy: &busy,
                crowd: &crowd,
            };
            allocator::auction(&ctx, &bidders, &tasks, &|r, k| exec.cooling(r, k, now))
        };
        for award in result.awards {
            let r = award.robot;
            if let Some(a) = exec.active(r) {
                // Cut an opportunistic charge short only when work has piled up.
                let charging = matches!(a.job, Job::Charge { .. });
                if charging
                    && result.role_bids.get(&r).copied().unwrap_or(0) < INTERRUPT_CHARGE_TASKS
                {
                    continue;
                }
                if let Some((cancel, old)) = exec.cancel(r, &reg.robot(r).id) {
                    diag.debug(now, || {
                        format!(
                            "{}: cancel {} for better work",
                            reg.robot(r).id,
                            old.job.label()
                        )
                    });
                    cmds.push(cancel);
                }
            }
            let task = tasks[award.task].clone();
            let action = award.plan.action.clone();
            let est = award.plan.duration_s;
            let job = Job::Work {
                task,
                plan: Box::new(award.plan),
            };
            diag.debug(now, || {
                format!(
                    "{}: {} (score {:.0})",
                    reg.robot(r).id,
                    job.label(),
                    award.score
                )
            });
            let e = world.fleet.get(r);
            cmds.push(exec.issue(
                r,
                &reg.robot(r).id,
                job,
                action,
                now,
                est,
                e.battery_wh(),
                e.payload_kg(),
            ));
            free.retain(|&x| x != r);
        }

        // 3. Leftover robots: charge when work is waiting on energy, top up
        //    while there is nothing to do, or wait near upcoming work.
        let front = planner.front(world, cfg);
        for r in free.clone() {
            if !free.contains(&r) {
                continue;
            }
            let e = world.fleet.get(r);
            let Some(pos) = e.position() else { continue };
            let battery = e.battery_wh();
            let cap_wh = reg.robot(r).battery_wh;
            let need = Some(capacity.energy_need_wh(r, &workload, reg, cfg));
            let blocked = result.energy_blocked.contains(&r) || stranded.contains(&r);
            let idle_work = result.role_bids.get(&r).copied().unwrap_or(0) == 0;
            let opportunistic = !blocked
                && idle_work
                && battery < cfg.opportunistic_charge_frac * cap_wh
                && need.is_some_and(|need| need > battery - Energy::reserve_wh(reg, r, pos, cfg));
            if blocked || opportunistic {
                let chargers = chargers_for(r, &free, energy, exec, reg);
                let until = Energy::charge_target(world, reg, r, need, cfg);
                if let Some(plan) = energy.plan_charge(world, reg, r, until, &load, &chargers) {
                    issue_charge(
                        world,
                        reg,
                        energy,
                        exec,
                        &mut load,
                        &mut free,
                        r,
                        plan,
                        opportunistic,
                        &mut cmds,
                        diag,
                    );
                    continue;
                }
                let rescue_underway = energy.reserved.values().any(|&client| client == r);
                if blocked && !rescue_underway {
                    // Nothing reachable: bring a mobile charger over.
                    let candidates: Vec<usize> = free.iter().copied().filter(|&c| c != r).collect();
                    if let Some(c) = energy.rescuer(world, reg, &candidates, pos, r) {
                        let est =
                            reg.travel_s(c, world.fleet.get(c).position().unwrap_or(pos), pos);
                        let job = Job::Move {
                            to: pos,
                            anticipatory: false,
                            rescue_for: Some(r),
                        };
                        diag.info(now, || {
                            format!("{} rescues {}", reg.robot(c).id, reg.robot(r).id)
                        });
                        let ce = world.fleet.get(c);
                        cmds.push(exec.issue(
                            c,
                            &reg.robot(c).id,
                            job,
                            Action::MoveTo { target: pos },
                            now,
                            est,
                            ce.battery_wh(),
                            ce.payload_kg(),
                        ));
                        energy.reserved.insert(c, r);
                        free.retain(|&x| x != c);
                    }
                    continue;
                }
            }
            // An idle mobile charger with a low tank goes to refill it.
            if idle_work && let Some(plan) = energy.plan_tank_refill(world, reg, r, &load) {
                issue_charge(
                    world, reg, energy, exec, &mut load, &mut free, r, plan, true, &mut cmds, diag,
                );
                continue;
            }
            // Wait near the work front if this robot works the pad surface.
            let Some(to) = front else { continue };
            let works_front = FRONT_CAPABILITIES
                .iter()
                .any(|&c| capacity.is_role(r, c, cfg.role_min_share));
            if !works_front || geom::dist(pos, to) <= 3.0 {
                continue;
            }
            let trip = reg.travel_wh(r, pos, to);
            if !Energy::affordable(world, reg, r, trip, to, cfg) {
                continue;
            }
            let job = Job::Move {
                to,
                anticipatory: true,
                rescue_for: None,
            };
            diag.debug(now, || format!("{}: {}", reg.robot(r).id, job.label()));
            cmds.push(exec.issue(
                r,
                &reg.robot(r).id,
                job,
                Action::MoveTo { target: to },
                now,
                reg.travel_s(r, pos, to),
                battery,
                e.payload_kg(),
            ));
            free.retain(|&x| x != r);
        }

        let mut wake = next_cooldown.filter(|&w| w > now);
        if exec.idle() && !progress.complete() {
            let idle_wake = now + cfg.idle_wake_s;
            wake = Some(wake.map_or(idle_wake, |w| w.min(idle_wake)));
        }
        (cmds, wake)
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
    load: &mut StationLoad,
    free: &mut Vec<usize>,
    r: usize,
    plan: crate::energy::ChargePlan,
    opportunistic: bool,
    cmds: &mut Vec<Command>,
    diag: &Diag,
) {
    let now = world.now;
    match plan.source {
        ChargeSource::Station(s) => load.add(s, now + plan.est_s),
        ChargeSource::Mobile(c) => {
            energy.reserved.insert(c, r);
            free.retain(|&x| x != c);
        }
    }
    let action = Action::Charge {
        source: plan.source_id.clone(),
        until_wh: (!plan.fill_tank).then_some(plan.until_wh),
    };
    let est = plan.est_s;
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
    cmds.push(exec.issue(
        r,
        &reg.robot(r).id,
        job,
        action,
        now,
        est,
        e.battery_wh(),
        e.payload_kg(),
    ));
    free.retain(|&x| x != r);
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
            in_relay_coverage: true,
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
}
