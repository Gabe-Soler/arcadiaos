//! Executor: the command lifecycle.
//!
//! Turns jobs into protocol commands with deterministic ids, tracks each one
//! from issue through accepted to completed, failed or rejected, and hands
//! the outcome back with the job it belonged to. It also owns retry back-off
//! (so a rejected task is not re-sent in a tight loop at one sim time),
//! cancellation of interruptible jobs, and which robots in-flight commands
//! engage (repair, tow and charge targets cannot take commands of their own).

use crate::allocator::WorkPlan;
use crate::energy::ChargePlan;
use crate::planner::{Busy, SpoilKind};
use crate::tasks::{Task, TaskKey};
use arcadia_protocol::{Action, Command, Event, EventKind, FailReason, GridCoord, RejectReason};
use std::collections::{BTreeMap, BTreeSet};

/// What a robot has been asked to do.
#[derive(Debug, Clone, PartialEq)]
pub enum Job {
    /// Planned pad, berm or survey work.
    Work {
        /// The task.
        task: Task,
        /// The robot's plan for it.
        plan: Box<WorkPlan>,
    },
    /// Recharge.
    Charge {
        /// Where and how much.
        plan: ChargePlan,
        /// Taken while idle; may be cut short when work appears.
        opportunistic: bool,
    },
    /// Reposition.
    Move {
        /// Destination.
        to: GridCoord,
        /// Waiting near upcoming work; may be cut short.
        anticipatory: bool,
        /// Mobile charger heading to a stranded client.
        rescue_for: Option<usize>,
    },
    /// Field repair of another robot: clears field-repairable faults, frees
    /// it if stuck and cuts its wear.
    Repair {
        /// The robot being repaired.
        target: usize,
        /// Maintenance of a working robot rather than recovery of a disabled one.
        preventive: bool,
    },
    /// Tow a disabled robot somewhere it can be helped.
    Tow {
        /// The robot being towed.
        target: usize,
        /// Destination cell.
        to: GridCoord,
        /// What the target needs there.
        purpose: TowPurpose,
    },
    /// Depot service: clears every fault, resets wear, frees a stuck robot.
    Service {
        /// Depot index.
        depot: usize,
        /// Maintenance of a working robot rather than recovery of a disabled one.
        preventive: bool,
    },
    /// Strip a disabled robot for parts to restore another, with the
    /// approval the policy requires.
    Cannibalize {
        /// The robot stripped (lost afterwards).
        target: usize,
        /// The robot that receives the parts.
        beneficiary: usize,
    },
    /// Drive to a cell and hold there as a comms relay until cancelled.
    Relay {
        /// Where it holds.
        at: GridCoord,
    },
}

/// Why a robot is being towed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TowPurpose {
    /// To a depot for service.
    Depot(usize),
    /// To a station to recharge an empty battery.
    Station(usize),
    /// Out of soft ground; the tow itself frees it.
    Free,
}

impl Job {
    /// Short label for logs.
    pub fn label(&self) -> String {
        match self {
            Job::Work { task, .. } => {
                format!(
                    "{} ({},{})",
                    task.stage.name(),
                    task.area.min.x,
                    task.area.min.y
                )
            }
            Job::Charge {
                plan,
                opportunistic,
            } => format!(
                "charge at {} to {:.0} Wh{}{}",
                plan.source_id,
                plan.until_wh,
                if plan.fill_tank {
                    " and refill its tank"
                } else {
                    ""
                },
                if *opportunistic {
                    " (opportunistic)"
                } else {
                    ""
                }
            ),
            Job::Move { to, .. } => format!("move to ({},{})", to.x, to.y),
            Job::Repair { target, preventive } => format!(
                "repair robot #{target}{}",
                if *preventive { " (preventive)" } else { "" }
            ),
            Job::Tow {
                target,
                to,
                purpose,
            } => format!("tow robot #{target} to ({},{}) {purpose:?}", to.x, to.y),
            Job::Service { depot, preventive } => format!(
                "service at depot #{depot}{}",
                if *preventive { " (preventive)" } else { "" }
            ),
            Job::Cannibalize {
                target,
                beneficiary,
            } => format!("cannibalize robot #{target} for robot #{beneficiary}"),
            Job::Relay { at } => format!("relay at ({},{})", at.x, at.y),
        }
    }

    /// Robots this job engages besides the one running it: repair, tow and
    /// strip targets, the beneficiary of parts, and a mobile charger in use.
    pub fn engages(&self) -> Vec<usize> {
        match self {
            Job::Repair { target, .. } | Job::Tow { target, .. } => vec![*target],
            Job::Cannibalize {
                target,
                beneficiary,
            } => vec![*target, *beneficiary],
            Job::Charge { plan, .. } => match plan.source {
                crate::energy::ChargeSource::Mobile(c) => vec![c],
                crate::energy::ChargeSource::Station(_) => vec![],
            },
            Job::Work { .. } | Job::Move { .. } | Job::Service { .. } | Job::Relay { .. } => {
                vec![]
            }
        }
    }
}

/// Planned exposure of a command, for the health model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlannedUse {
    /// Driving, m (towing included).
    pub drive_m: f64,
    /// Driving time, s.
    pub drive_s: f64,
    /// Work time other than scanning, s.
    pub work_s: f64,
    /// Scanning time, s.
    pub scan_s: f64,
    /// Calibration factor the energy estimate carried.
    pub energy_factor: f64,
    /// Calibration factor the duration estimate carried.
    pub duration_factor: f64,
}

impl Default for PlannedUse {
    fn default() -> Self {
        Self {
            drive_m: 0.0,
            drive_s: 0.0,
            work_s: 0.0,
            scan_s: 0.0,
            energy_factor: 1.0,
            duration_factor: 1.0,
        }
    }
}

/// What the coordinator knew when it sent a command.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Sent {
    /// Decision time.
    pub now: f64,
    /// Expected duration, s.
    pub est_s: f64,
    /// Battery, Wh.
    pub battery_wh: f64,
    /// Payload, kg.
    pub payload_kg: f64,
    /// Planned exposure.
    pub planned: PlannedUse,
}

/// A command in flight.
#[derive(Debug, Clone, PartialEq)]
pub struct Assignment {
    /// Robot index.
    pub robot: usize,
    /// Command id.
    pub command_id: String,
    /// What it is for.
    pub job: Job,
    /// When it was sent.
    pub issued_t: f64,
    /// Expected completion.
    pub est_end_t: f64,
    /// Battery when sent, Wh.
    pub battery_wh: f64,
    /// Payload when sent, kg.
    pub payload_kg: f64,
    /// Planned exposure.
    pub planned: PlannedUse,
    /// The simulator acknowledged it.
    pub accepted: bool,
    /// The simulator's estimate from `command_accepted`, s.
    pub accepted_est_s: Option<f64>,
}

impl Assignment {
    /// Whether a better job may replace this one.
    pub fn interruptible(&self) -> bool {
        matches!(
            self.job,
            Job::Charge {
                opportunistic: true,
                ..
            } | Job::Move {
                anticipatory: true,
                ..
            }
        )
    }
}

/// How a command ended.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    /// Done.
    Completed {
        /// Energy the robot used, Wh.
        energy_used_wh: f64,
    },
    /// Started but did not finish.
    Failed {
        /// Why.
        reason: FailReason,
        /// Simulator detail.
        detail: Option<String>,
        /// Energy the robot used before it stopped, Wh.
        energy_used_wh: f64,
    },
    /// Refused at submission.
    Rejected {
        /// Why.
        reason: RejectReason,
        /// Simulator detail.
        detail: Option<String>,
    },
}

/// Counters for diagnostics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ExecStats {
    /// Commands sent (cancels included).
    pub issued: u64,
    /// Commands completed.
    pub completed: u64,
    /// Commands failed (cancellations excluded).
    pub failed: u64,
    /// Commands rejected.
    pub rejected: u64,
    /// Interruptible jobs cancelled for better work.
    pub cancelled: u64,
}

/// The command lifecycle tracker.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Executor {
    next_id: u64,
    active: BTreeMap<usize, Assignment>,
    by_command: BTreeMap<String, usize>,
    /// Commands cancelled on purpose; their failure is expected.
    cancelled: BTreeSet<String>,
    /// Cancel commands sent.
    cancels: BTreeSet<String>,
    cooldown: BTreeMap<(usize, TaskKey), f64>,
    robot_cooldown: BTreeMap<usize, f64>,
    /// Consecutive rejections per robot.
    streak: BTreeMap<usize, u32>,
    /// Counters.
    pub stats: ExecStats,
}

impl Executor {
    /// A fresh executor.
    pub fn new() -> Self {
        Self::default()
    }

    fn next_command_id(&mut self) -> String {
        self.next_id += 1;
        format!("os-{:06}", self.next_id)
    }

    /// The job robot `r` is doing, if any.
    pub fn active(&self, r: usize) -> Option<&Assignment> {
        self.active.get(&r)
    }

    /// Every job in flight.
    pub fn all_active(&self) -> impl Iterator<Item = &Assignment> {
        self.active.values()
    }

    /// Whether nothing is in flight.
    pub fn idle(&self) -> bool {
        self.active.is_empty()
    }

    /// Send `job` to robot `r` as `action`.
    pub fn issue(
        &mut self,
        r: usize,
        robot_id: &str,
        job: Job,
        action: Action,
        sent: Sent,
    ) -> Command {
        let id = self.next_command_id();
        self.stats.issued += 1;
        self.by_command.insert(id.clone(), r);
        self.active.insert(
            r,
            Assignment {
                robot: r,
                command_id: id.clone(),
                job,
                issued_t: sent.now,
                est_end_t: sent.now + sent.est_s.max(0.0),
                battery_wh: sent.battery_wh,
                payload_kg: sent.payload_kg,
                planned: sent.planned,
                accepted: false,
                accepted_est_s: None,
            },
        );
        Command {
            command_id: id,
            robot_id: robot_id.into(),
            approval_id: None,
            action,
        }
    }

    /// Cancel robot `r`'s job so it can take another in the same decision.
    pub fn cancel(&mut self, r: usize, robot_id: &str) -> Option<(Command, Assignment)> {
        let a = self.active.remove(&r)?;
        self.by_command.remove(&a.command_id);
        self.cancelled.insert(a.command_id.clone());
        let id = self.next_command_id();
        self.cancels.insert(id.clone());
        self.stats.issued += 1;
        self.stats.cancelled += 1;
        let cmd = Command {
            command_id: id,
            robot_id: robot_id.into(),
            approval_id: None,
            action: Action::Cancel {
                target_command: a.command_id.clone(),
            },
        };
        Some((cmd, a))
    }

    /// The in-flight assignment of command `command_id`.
    pub fn assignment_of(&self, command_id: &str) -> Option<&Assignment> {
        let r = self.by_command.get(command_id)?;
        self.active.get(r)
    }

    /// When the in-flight command `command_id` was issued (scan results
    /// arrive before their command completes).
    pub fn issued_t(&self, command_id: &str) -> Option<f64> {
        let r = self.by_command.get(command_id)?;
        self.active.get(r).map(|a| a.issued_t)
    }

    /// Robots engaged by in-flight commands of other robots.
    pub fn engaged(&self) -> BTreeSet<usize> {
        self.active.values().flat_map(|a| a.job.engages()).collect()
    }

    /// The in-flight command that engages robot `r`, if any.
    pub fn engaging(&self, r: usize) -> Option<&Assignment> {
        self.active.values().find(|a| a.job.engages().contains(&r))
    }

    /// Track a lifecycle event; returns the finished job and its outcome.
    pub fn on_event(&mut self, e: &Event) -> Option<(Assignment, Outcome)> {
        let (id, outcome) = match &e.kind {
            EventKind::CommandAccepted {
                command_id,
                estimated_duration_s,
                ..
            } => {
                if let Some(&r) = self.by_command.get(command_id)
                    && let Some(a) = self.active.get_mut(&r)
                {
                    a.accepted = true;
                    a.accepted_est_s = Some(*estimated_duration_s);
                    self.streak.remove(&r);
                }
                return None;
            }
            EventKind::CommandCompleted {
                command_id,
                energy_used_wh,
                ..
            } => (
                command_id,
                Outcome::Completed {
                    energy_used_wh: *energy_used_wh,
                },
            ),
            EventKind::CommandFailed {
                command_id,
                reason,
                detail,
                energy_used_wh,
                ..
            } => (
                command_id,
                Outcome::Failed {
                    reason: *reason,
                    detail: detail.clone(),
                    energy_used_wh: *energy_used_wh,
                },
            ),
            EventKind::CommandRejected {
                command_id,
                reason,
                detail,
                ..
            } => (
                command_id,
                Outcome::Rejected {
                    reason: *reason,
                    detail: detail.clone(),
                },
            ),
            _ => return None,
        };
        if self.cancels.remove(id) {
            if let Outcome::Rejected { .. } = outcome {
                self.stats.rejected += 1;
            }
            return None;
        }
        if self.cancelled.remove(id) {
            return None;
        }
        let r = self.by_command.remove(id)?;
        let a = self.active.remove(&r)?;
        match outcome {
            Outcome::Completed { .. } => self.stats.completed += 1,
            Outcome::Failed { .. } => self.stats.failed += 1,
            Outcome::Rejected { .. } => {
                self.stats.rejected += 1;
                *self.streak.entry(r).or_default() += 1;
            }
        }
        Some((a, outcome))
    }

    /// Cells and keys that in-flight work covers.
    pub fn busy(&self) -> Busy {
        let mut busy = Busy::default();
        for a in self.active.values() {
            if let Job::Work { task, plan } = &a.job {
                busy.keys.insert(task.key);
                if task.changes_ground() {
                    busy.cells.insert((task.cell().x, task.cell().y));
                }
                if let Some(spoil) = plan.dig.as_ref().and_then(|d| d.spoil)
                    && spoil.kind == SpoilKind::Fill
                {
                    busy.cells.insert((spoil.at.x, spoil.at.y));
                }
            }
        }
        busy
    }

    /// Rejections robot `r` has had in a row.
    pub fn rejection_streak(&self, r: usize) -> u32 {
        self.streak.get(&r).copied().unwrap_or(0)
    }

    /// Back off task `key` on robot `r` until `until`.
    pub fn back_off(&mut self, r: usize, key: TaskKey, until: f64) {
        self.cooldown.insert((r, key), until);
    }

    /// Back off robot `r` entirely until `until`.
    pub fn back_off_robot(&mut self, r: usize, until: f64) {
        self.robot_cooldown.insert(r, until);
    }

    /// Whether task `key` is still backing off on robot `r`.
    pub fn cooling(&self, r: usize, key: TaskKey, now: f64) -> bool {
        self.cooldown.get(&(r, key)).is_some_and(|&t| t > now)
    }

    /// Whether robot `r` is backing off entirely.
    pub fn robot_cooling(&self, r: usize, now: f64) -> bool {
        self.robot_cooldown.get(&r).is_some_and(|&t| t > now)
    }

    /// Forget expired back-offs; returns the earliest one still pending.
    pub fn prune_cooldowns(&mut self, now: f64) -> Option<f64> {
        self.cooldown.retain(|_, t| *t > now);
        self.robot_cooldown.retain(|_, t| *t > now);
        self.cooldown
            .values()
            .chain(self.robot_cooldown.values())
            .copied()
            .fold(None, |m: Option<f64>, t| Some(m.map_or(t, |m| m.min(t))))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tasks::Stage;
    use arcadia_protocol::GridRect;

    fn task(x: u32) -> Task {
        let c = GridCoord { x, y: 0 };
        Task {
            key: TaskKey::Cell(Stage::Grade, x, 0),
            stage: Stage::Grade,
            area: GridRect { min: c, max: c },
            seq: 0.0,
            amount: 1.0,
            max_kg: 0.0,
            elevation_m: 0.0,
            section: Some(0),
            cast_to: None,
        }
    }

    fn work(x: u32) -> Job {
        let c = GridCoord { x, y: 0 };
        Job::Work {
            task: task(x),
            plan: Box::new(WorkPlan {
                action: Action::Grade {
                    area: GridRect { min: c, max: c },
                    target_elevation_m: 0.0,
                },
                end: c,
                travel_s: 0.0,
                duration_s: 20.0,
                energy_wh: 8.0,
                haul: None,
                dig: None,
                strands: false,
            }),
        }
    }

    fn ev(kind: EventKind) -> Event {
        Event { t: 10.0, kind }
    }

    fn sent(est_s: f64) -> Sent {
        Sent {
            now: 0.0,
            est_s,
            battery_wh: 100.0,
            payload_kg: 0.0,
            planned: PlannedUse::default(),
        }
    }

    #[test]
    fn lifecycle_from_issue_to_completion() {
        let mut ex = Executor::new();
        let Job::Work { plan, .. } = work(3) else {
            unreachable!()
        };
        let cmd = ex.issue(0, "r0", work(3), plan.action.clone(), sent(20.0));
        assert_eq!(cmd.command_id, "os-000001");
        assert!(ex.busy().cell(GridCoord { x: 3, y: 0 }));
        assert!(
            ex.on_event(&ev(EventKind::CommandAccepted {
                command_id: cmd.command_id.clone(),
                robot_id: "r0".into(),
                estimated_duration_s: 20.0,
            }))
            .is_none()
        );
        assert!(ex.active(0).unwrap().accepted);
        assert_eq!(ex.active(0).unwrap().accepted_est_s, Some(20.0));
        let (a, out) = ex
            .on_event(&ev(EventKind::CommandCompleted {
                command_id: cmd.command_id,
                robot_id: "r0".into(),
                energy_used_wh: 8.0,
            }))
            .unwrap();
        assert_eq!(a.robot, 0);
        assert_eq!(
            out,
            Outcome::Completed {
                energy_used_wh: 8.0
            }
        );
        assert!(ex.idle());
        assert_eq!(ex.stats.completed, 1);
    }

    #[test]
    fn rejections_failures_and_backoff() {
        let mut ex = Executor::new();
        let Job::Work { plan, .. } = work(1) else {
            unreachable!()
        };
        let cmd = ex.issue(2, "r2", work(1), plan.action, sent(20.0));
        let (_, out) = ex
            .on_event(&ev(EventKind::CommandRejected {
                command_id: cmd.command_id,
                robot_id: "r2".into(),
                reason: RejectReason::InvalidTarget,
                detail: Some("not cleared".into()),
            }))
            .unwrap();
        assert!(matches!(
            out,
            Outcome::Rejected {
                reason: RejectReason::InvalidTarget,
                ..
            }
        ));
        let key = TaskKey::Cell(Stage::Grade, 1, 0);
        ex.back_off(2, key, 300.0);
        assert!(ex.cooling(2, key, 10.0));
        assert!(!ex.cooling(1, key, 10.0));
        ex.back_off_robot(1, 50.0);
        assert!(ex.robot_cooling(1, 10.0));
        assert_eq!(ex.prune_cooldowns(10.0), Some(50.0));
        assert_eq!(ex.prune_cooldowns(400.0), None);
        // Unknown command ids are ignored.
        assert!(
            ex.on_event(&ev(EventKind::CommandFailed {
                command_id: "zz".into(),
                robot_id: "r2".into(),
                reason: FailReason::Blocked,
                detail: None,
                energy_used_wh: 0.0,
            }))
            .is_none()
        );
    }

    #[test]
    fn cancelling_frees_the_robot_and_swallows_the_echo() {
        let mut ex = Executor::new();
        let to = GridCoord { x: 5, y: 5 };
        let first = ex.issue(
            0,
            "r0",
            Job::Move {
                to,
                anticipatory: true,
                rescue_for: None,
            },
            Action::MoveTo { target: to },
            sent(30.0),
        );
        assert!(ex.active(0).unwrap().interruptible());
        let (cancel, old) = ex.cancel(0, "r0").unwrap();
        assert_eq!(old.command_id, first.command_id);
        assert!(matches!(cancel.action, Action::Cancel { .. }));
        assert!(ex.active(0).is_none());
        // The simulator's echo of the cancellation is absorbed.
        for kind in [
            EventKind::CommandAccepted {
                command_id: cancel.command_id.clone(),
                robot_id: "r0".into(),
                estimated_duration_s: 0.0,
            },
            EventKind::CommandFailed {
                command_id: first.command_id.clone(),
                robot_id: "r0".into(),
                reason: FailReason::Cancelled,
                detail: None,
                energy_used_wh: 1.0,
            },
            EventKind::CommandCompleted {
                command_id: cancel.command_id.clone(),
                robot_id: "r0".into(),
                energy_used_wh: 0.0,
            },
        ] {
            assert!(ex.on_event(&ev(kind)).is_none());
        }
        assert_eq!(ex.stats.cancelled, 1);
    }

    #[test]
    fn recovery_jobs_engage_their_targets() {
        let mut ex = Executor::new();
        ex.issue(
            0,
            "r0",
            Job::Repair {
                target: 3,
                preventive: false,
            },
            Action::Repair {
                target_robot: "r3".into(),
            },
            sent(2000.0),
        );
        let depot = GridCoord { x: 1, y: 1 };
        ex.issue(
            1,
            "r1",
            Job::Tow {
                target: 2,
                to: depot,
                purpose: TowPurpose::Depot(0),
            },
            Action::Tow {
                target_robot: "r2".into(),
                to: depot,
            },
            sent(600.0),
        );
        ex.issue(
            4,
            "r4",
            Job::Service {
                depot: 0,
                preventive: true,
            },
            Action::Service { depot: "dp".into() },
            sent(7200.0),
        );
        assert_eq!(ex.engaged(), BTreeSet::from([2, 3]));
        assert_eq!(ex.engaging(3).map(|a| a.robot), Some(0));
        assert!(ex.engaging(4).is_none());
        assert!(!ex.active(0).unwrap().interruptible());
        assert!(ex.active(1).unwrap().job.label().contains("tow robot #2"));
    }
}
