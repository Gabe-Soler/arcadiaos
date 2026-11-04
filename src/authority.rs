//! Authority: the policy in effect, Earth's approvals, and everything
//! ArcadiaOS tells Earth.
//!
//! Every action ArcadiaOS takes has an authority level under the policy in
//! effect (the scenario's, replaced by each `policy_update` when it arrives):
//!
//! - **Level 1:** just do it.
//! - **Level 2:** do it, then `notify` Earth. Notifies go out in the same
//!   decision as the commands they list, so none is ever late, and there is
//!   one per action per decision.
//! - **Level 3:** ask with an `approval_request`, then act on `approve`, or,
//!   once the deadline has passed with no answer, on the fallback. The
//!   deadline and the fallback come from [`level3_rule`], a cost model.
//! - **Level 4:** ask, and act only on `approve`. Never autonomously, even in
//!   a blackout.
//!
//! Approvals are scoped: ArcadiaOS uses an approval only for the purpose it
//! asked for (for example cannibalizing one particular robot), although the
//! simulator would accept it for any command of that action.
//!
//! It also sends a `status_report` about once a sol while the link is up,
//! one after major events, and one to catch up when the link comes back.

use arcadia_protocol::{Downlink, Fallback, GoalProgress, Policy, SiteToEarth};
use std::collections::BTreeMap;

/// The policy level of `action` (1–4).
pub fn level(policy: &Policy, action: &str) -> u8 {
    policy.level_for(action).get()
}

/// Whether Earth can be reached, and how fast an answer would come.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Reach {
    /// The link is up: an answer takes a round trip plus Earth's thinking time.
    Up {
        /// Expected time from asking to the answer arriving, s.
        answer_s: f64,
    },
    /// The link is down until `back_t` (if known), then a round trip.
    Down {
        /// When the link is expected back, if the site knows.
        back_t: Option<f64>,
        /// Expected time from the link coming back to an answer arriving, s.
        answer_s: f64,
    },
}

impl Reach {
    /// Expected wait from `now` for an answer to a request sent now, s.
    pub fn answer_in_s(&self, now: f64) -> f64 {
        match *self {
            Reach::Up { answer_s } => answer_s,
            Reach::Down {
                back_t: Some(back),
                answer_s,
            } => (back - now).max(0.0) + answer_s,
            Reach::Down { back_t: None, .. } => f64::INFINITY,
        }
    }
}

/// What a level-3 action is worth, in seconds of crew time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stakes {
    /// What acting gains over not acting (time saved, weighted by how much
    /// the crew needs the robots involved).
    pub benefit_s: f64,
    /// What each second of waiting for Earth costs.
    pub wait_cost_per_s: f64,
    /// Expected cost of acting when Earth would have said no: how
    /// irreversible and how risky the action is.
    pub risk_s: f64,
}

/// How to ask for a level-3 action.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ask {
    /// Answer needed by.
    pub decide_by_t: f64,
    /// What to do if no answer comes by then.
    pub fallback: Fallback,
}

/// The level-3 fallback rule. Waiting is worth it only if an answer is
/// expected within the policy's `max_wait_s`; otherwise (a blackout, an
/// outage, or a round trip longer than the policy allows) the rule decides
/// at once:
///
/// ```text
/// expected = time until the link is up + 2 × one-way delay + Earth's response time
/// wait     = min(margin × expected, max_wait_s)  if expected ≤ max_wait_s,  else 0
/// net      = benefit − wait × wait_cost_per_s
/// ask      only if net > 0
/// fallback = proceed if net > risk, else abort
/// ```
///
/// A request with no time to wait and fallback abort would change nothing,
/// so it is not sent (`None`).
pub fn level3_rule(
    stakes: &Stakes,
    reach: Reach,
    now: f64,
    max_wait_s: f64,
    margin: f64,
) -> Option<Ask> {
    let answer_s = reach.answer_in_s(now);
    let wait_s = if answer_s <= max_wait_s {
        (answer_s * margin.max(1.0)).min(max_wait_s)
    } else {
        0.0
    };
    let net = stakes.benefit_s - wait_s * stakes.wait_cost_per_s.max(0.0);
    if net.is_nan() || net <= 0.0 {
        return None;
    }
    let fallback = if net > stakes.risk_s {
        Fallback::Proceed
    } else {
        Fallback::Abort
    };
    if wait_s <= 0.0 && fallback == Fallback::Abort {
        return None;
    }
    Some(Ask {
        decide_by_t: now + wait_s,
        fallback,
    })
}

/// An approval request ArcadiaOS sent.
#[derive(Debug, Clone, PartialEq)]
pub struct Request {
    /// Its `message_id`, cited as `approval_id`.
    pub id: String,
    /// Policy action.
    pub action: String,
    /// What it was for, for example `cannibalize sc-1 for cp-1`.
    pub purpose: String,
    /// When it was sent.
    pub sent_t: f64,
    /// The earlier of `decide_by_t` and the send time plus the policy's
    /// `max_wait_s` (semantics §9.2).
    pub deadline_t: f64,
    /// The request's fallback.
    pub fallback: Fallback,
    /// The policy level of the action when it was sent.
    pub level: u8,
    /// Earth's answer and when it arrived: `true` for approve.
    pub answer: Option<(bool, f64)>,
}

/// Traffic to and from Earth.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counts {
    /// Notifies sent.
    pub notifies: u64,
    /// Approval requests sent.
    pub requests: u64,
    /// Status reports sent.
    pub status_reports: u64,
    /// Approvals received.
    pub approvals: u64,
    /// Vetoes received.
    pub vetoes: u64,
    /// Policy updates received.
    pub policy_updates: u64,
}

/// The authority layer's state.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Authority {
    sent: u64,
    requests: Vec<Request>,
    outbox: Vec<SiteToEarth>,
    /// Level-2 coordinator-level actions to report: action → (what, commands).
    notes: BTreeMap<String, (Vec<String>, Vec<String>)>,
    last_status_t: Option<f64>,
    /// A major event wants a status report as soon as the link allows.
    status_wanted: bool,
    /// A periodic report fell due while the link was down.
    status_missed: bool,
    /// Goal priorities from Earth.
    pub priorities: BTreeMap<String, i64>,
    /// Counters.
    pub counts: Counts,
}

impl Authority {
    fn next_id(&mut self) -> String {
        self.sent += 1;
        format!("os-m{:06}", self.sent)
    }

    fn latest(&self, action: &str, purpose: &str) -> Option<&Request> {
        self.requests
            .iter()
            .rev()
            .find(|r| r.action == action && r.purpose == purpose)
    }

    /// The approval to cite for `action` with this `purpose` at `now`, if any:
    /// Earth's approval has arrived, or, at level 3, the deadline has passed
    /// with fallback proceed and no veto.
    pub fn authorization(&self, action: &str, purpose: &str, now: f64) -> Option<String> {
        let r = self.latest(action, purpose)?;
        match r.answer {
            Some((true, _)) => Some(r.id.clone()),
            Some((false, _)) => None,
            None if r.level == 3 && now >= r.deadline_t && r.fallback == Fallback::Proceed => {
                Some(r.id.clone())
            }
            None => None,
        }
    }

    /// Whether a request for `action` with this `purpose` is still worth
    /// waiting for at `now`: unanswered and before its deadline.
    pub fn pending(&self, action: &str, purpose: &str, now: f64) -> bool {
        self.latest(action, purpose)
            .is_some_and(|r| r.answer.is_none() && now < r.deadline_t)
    }

    /// Whether the latest request for `action` with this `purpose` has been
    /// settled without authorizing it: vetoed, or past its deadline with
    /// fallback abort (or unanswered at level 4).
    pub fn refused(&self, action: &str, purpose: &str, now: f64) -> bool {
        self.latest(action, purpose)
            .is_some_and(|r| match r.answer {
                Some((approved, _)) => !approved,
                None => now >= r.deadline_t && (r.level == 4 || r.fallback == Fallback::Abort),
            })
    }

    /// Whether `action` with this `purpose` has ever been asked about.
    pub fn asked(&self, action: &str, purpose: &str) -> bool {
        self.latest(action, purpose).is_some()
    }

    /// The earliest deadline among requests still waiting for an answer.
    pub fn next_deadline(&self, now: f64) -> Option<f64> {
        self.requests
            .iter()
            .filter(|r| r.answer.is_none() && r.deadline_t > now)
            .map(|r| r.deadline_t)
            .fold(None, |m: Option<f64>, t| Some(m.map_or(t, |m| m.min(t))))
    }

    /// Send an approval request for `action`.
    pub fn ask(
        &mut self,
        policy: &Policy,
        action: &str,
        purpose: &str,
        summary: String,
        ask: Ask,
        now: f64,
    ) -> String {
        let id = self.next_id();
        let max_wait = now + policy.level3.max_wait_s.max(0.0);
        self.requests.push(Request {
            id: id.clone(),
            action: action.into(),
            purpose: purpose.into(),
            sent_t: now,
            deadline_t: ask.decide_by_t.min(max_wait),
            fallback: ask.fallback,
            level: level(policy, action),
            answer: None,
        });
        self.outbox.push(SiteToEarth {
            message_id: id.clone(),
            body: Downlink::ApprovalRequest {
                action: action.into(),
                summary,
                decide_by_t: Some(ask.decide_by_t),
                fallback: Some(ask.fallback),
            },
        });
        self.counts.requests += 1;
        id
    }

    /// Earth answered request `request_id`; the answer arrived at `t`.
    pub fn on_answer(&mut self, request_id: &str, approved: bool, t: f64) {
        if approved {
            self.counts.approvals += 1;
        } else {
            self.counts.vetoes += 1;
        }
        if let Some(r) = self
            .requests
            .iter_mut()
            .find(|r| r.id == request_id && r.answer.is_none())
        {
            r.answer = Some((approved, t));
        }
    }

    /// Report level-2 commands of `action` taken in this decision.
    pub fn notify_commands(&mut self, action: &str, commands: Vec<String>) {
        let summary = format!("{action}: {}", commands.join(", "));
        let id = self.next_id();
        self.outbox.push(SiteToEarth {
            message_id: id,
            body: Downlink::Notify {
                action: action.into(),
                summary,
                commands: Some(commands),
            },
        });
        self.counts.notifies += 1;
    }

    /// Note a coordinator-level action (`replan`, `reassign_task`, …) taken
    /// now, with the commands it led to, for the notify at the end of the
    /// decision. The caller has checked that it is level 2.
    pub fn note(&mut self, action: &str, what: String, commands: Vec<String>) {
        let entry = self.notes.entry(action.into()).or_default();
        entry.0.push(what);
        entry.1.extend(commands);
    }

    /// Turn this decision's notes into notifies.
    fn flush_notes(&mut self) {
        for (action, (what, commands)) in std::mem::take(&mut self.notes) {
            let id = self.next_id();
            self.outbox.push(SiteToEarth {
                message_id: id,
                body: Downlink::Notify {
                    action: action.clone(),
                    summary: format!("{action}: {}", what.join("; ")),
                    commands: (!commands.is_empty()).then_some(commands),
                },
            });
            self.counts.notifies += 1;
        }
    }

    /// Something Earth should hear about soon happened.
    pub fn major_event(&mut self) {
        self.status_wanted = true;
    }

    /// Whether a status report is due at `now`: periodically while the link
    /// is up, after a major event, and once to catch up when the link comes
    /// back after a report fell due while it was down.
    pub fn status_due(&mut self, now: f64, link_up: bool, period_s: f64) -> bool {
        let periodic = self.last_status_t.is_none_or(|t| now - t >= period_s);
        if !link_up {
            self.status_missed |= periodic && self.last_status_t.is_some();
            return false;
        }
        periodic || self.status_wanted || self.status_missed
    }

    /// Send a status report.
    pub fn report_status(&mut self, now: f64, summary: String, progress: Vec<GoalProgress>) {
        let id = self.next_id();
        self.outbox.push(SiteToEarth {
            message_id: id,
            body: Downlink::StatusReport {
                summary,
                progress: (!progress.is_empty()).then_some(progress),
            },
        });
        self.last_status_t = Some(now);
        self.status_wanted = false;
        self.status_missed = false;
        self.counts.status_reports += 1;
    }

    /// Messages for this decision's `to_earth`, notes included.
    pub fn take_outbox(&mut self) -> Vec<SiteToEarth> {
        self.flush_notes();
        std::mem::take(&mut self.outbox)
    }

    /// Every request sent so far.
    pub fn requests(&self) -> &[Request] {
        &self.requests
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arcadia_protocol::{AuthorityLevel, Level3Rule};

    fn policy(cannibalize: u8) -> Policy {
        Policy {
            policy_version: 1,
            default_level: AuthorityLevel::NOTIFY,
            levels: [(
                "cannibalize".to_string(),
                AuthorityLevel::try_from(cannibalize).unwrap(),
            )]
            .into_iter()
            .collect(),
            level3: Level3Rule {
                max_wait_s: 3600.0,
                default_fallback: Fallback::Abort,
            },
        }
    }

    const UP: Reach = Reach::Up { answer_s: 2700.0 };

    fn stakes(benefit_s: f64, risk_s: f64) -> Stakes {
        Stakes {
            benefit_s,
            wait_cost_per_s: 1.0,
            risk_s,
        }
    }

    #[test]
    fn the_rule_waits_only_when_earth_can_answer_in_time() {
        // Reachable: wait one expected round trip; proceed when the gain is worth the risk.
        let a = level3_rule(&stakes(20_000.0, 6_000.0), UP, 100.0, 3600.0, 1.0).unwrap();
        assert_eq!(a.decide_by_t, 2800.0);
        assert_eq!(a.fallback, Fallback::Proceed);
        // Worth acting, but not worth the risk without Earth: ask, and abort if no answer.
        let a = level3_rule(&stakes(8_000.0, 6_000.0), UP, 0.0, 3600.0, 1.0).unwrap();
        assert_eq!(a.fallback, Fallback::Abort);
        // Not worth even the wait.
        assert_eq!(
            level3_rule(&stakes(2_000.0, 0.0), UP, 0.0, 3600.0, 1.0),
            None
        );
        // Down with no end in sight: decide at once.
        let down = Reach::Down {
            back_t: None,
            answer_s: 2700.0,
        };
        let a = level3_rule(&stakes(20_000.0, 6_000.0), down, 50.0, 3600.0, 1.0).unwrap();
        assert_eq!((a.decide_by_t, a.fallback), (50.0, Fallback::Proceed));
        // ... and do not ask at all when the rule says abort.
        assert_eq!(
            level3_rule(&stakes(5_000.0, 6_000.0), down, 50.0, 3600.0, 1.0),
            None
        );
        // Down but back soon enough to answer within the policy's wait: wait for it.
        let soon = Reach::Down {
            back_t: Some(500.0),
            answer_s: 2700.0,
        };
        let a = level3_rule(&stakes(20_000.0, 6_000.0), soon, 0.0, 3600.0, 1.0).unwrap();
        assert_eq!(a.decide_by_t, 3200.0);
        // A round trip longer than the policy allows counts as unreachable.
        let slow = Reach::Up { answer_s: 5000.0 };
        assert_eq!(
            level3_rule(&stakes(20_000.0, 6_000.0), slow, 0.0, 3600.0, 1.0).map(|a| a.decide_by_t),
            Some(0.0)
        );
        // The margin stretches the wait for a late answer, up to the policy's limit.
        let a = level3_rule(&stakes(20_000.0, 6_000.0), UP, 0.0, 3600.0, 1.5).unwrap();
        assert_eq!(a.decide_by_t, 3600.0);
        let quick = Reach::Up { answer_s: 1000.0 };
        let a = level3_rule(&stakes(20_000.0, 6_000.0), quick, 0.0, 3600.0, 1.5).unwrap();
        assert_eq!(a.decide_by_t, 1500.0);
    }

    #[test]
    fn approvals_fallbacks_and_vetoes_authorize_as_semantics_9_says() {
        let p = policy(3);
        let mut a = Authority::default();
        let ask = |by, fallback| Ask {
            decide_by_t: by,
            fallback,
        };
        let id = a.ask(
            &p,
            "cannibalize",
            "x",
            "s".into(),
            ask(100.0, Fallback::Proceed),
            0.0,
        );
        assert!(a.pending("cannibalize", "x", 50.0));
        assert_eq!(a.authorization("cannibalize", "x", 50.0), None);
        assert_eq!(a.authorization("cannibalize", "x", 100.0), Some(id.clone()));
        assert_eq!(
            a.authorization("cannibalize", "other", 100.0),
            None,
            "scoped"
        );
        a.on_answer(&id, false, 150.0);
        assert_eq!(a.authorization("cannibalize", "x", 200.0), None);
        assert!(a.refused("cannibalize", "x", 200.0));
        // Abort fallback: refused at the deadline, authorized by an approval.
        let id = a.ask(
            &p,
            "cannibalize",
            "y",
            "s".into(),
            ask(100.0, Fallback::Abort),
            0.0,
        );
        assert!(a.refused("cannibalize", "y", 100.0));
        a.on_answer(&id, true, 120.0);
        assert_eq!(a.authorization("cannibalize", "y", 120.0), Some(id));
        // Level 4: only an approval counts.
        let p4 = policy(4);
        let id = a.ask(
            &p4,
            "cannibalize",
            "z",
            "s".into(),
            ask(0.0, Fallback::Proceed),
            0.0,
        );
        assert_eq!(a.authorization("cannibalize", "z", 10.0), None);
        a.on_answer(&id, true, 20.0);
        assert!(a.authorization("cannibalize", "z", 20.0).is_some());
        // The deadline never exceeds the policy's max_wait_s.
        a.ask(
            &p,
            "cannibalize",
            "w",
            "s".into(),
            ask(1e9, Fallback::Abort),
            10.0,
        );
        assert_eq!(a.requests().last().unwrap().deadline_t, 3610.0);
        assert_eq!(a.counts.requests, 4);
        assert_eq!(a.take_outbox().len(), 4);
    }

    #[test]
    fn notes_become_one_notify_per_action() {
        let mut a = Authority::default();
        a.notify_commands("tow", vec!["os-000003".into()]);
        a.note("reassign_task", "dig (3,4) after a fault".into(), vec![]);
        a.note("reassign_task", "grade (5,4) after a fault".into(), vec![]);
        a.note("replan", "roles without ex-1".into(), vec![]);
        let out = a.take_outbox();
        assert_eq!(out.len(), 3);
        assert!(
            matches!(&out[0].body, Downlink::Notify { commands: Some(c), .. } if c == &["os-000003"])
        );
        assert!(out.iter().any(|m| matches!(&m.body,
            Downlink::Notify { action, summary, commands: None } if action == "reassign_task" && summary.contains("(5,4)"))));
        assert_eq!(a.counts.notifies, 3);
        assert!(a.take_outbox().is_empty());
    }

    #[test]
    fn status_reports_are_periodic_and_catch_up_after_a_blackout() {
        let mut a = Authority::default();
        let sol = 88_775.0;
        assert!(a.status_due(0.0, true, sol), "the first is due at once");
        a.report_status(0.0, "s".into(), vec![]);
        assert!(!a.status_due(1000.0, true, sol));
        a.major_event();
        assert!(a.status_due(1000.0, true, sol));
        a.report_status(1000.0, "s".into(), vec![]);
        // A sol passes in a blackout: nothing sent, one report when it is over.
        assert!(!a.status_due(sol + 2000.0, false, sol));
        assert!(a.status_due(sol + 3000.0, true, sol));
        a.report_status(sol + 3000.0, "s".into(), vec![]);
        assert!(!a.status_due(sol + 3001.0, true, sol));
        assert_eq!(a.counts.status_reports, 3);
    }
}
