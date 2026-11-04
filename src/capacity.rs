//! Capacity planning: how the crew's time should be split across capabilities.
//!
//! The strategic half of the allocation method. Given the remaining work per
//! capability and every robot's rates, it solves a small linear program for
//! the fraction of each capability's work each robot should take:
//!
//! ```text
//! minimise   T + ε · Σ τ[r,k] f[r,k]
//! subject to Σ_r f[r,k] ≥ 1          for every capability k with work
//!            Σ_k τ[r,k] f[r,k] ≤ T   for every robot r
//!            f ≥ 0
//! ```
//!
//! where `τ[r,k]` is the time robot `r` would need to do all of `k` (work,
//! driving between steps and the charging the energy implies). `T` is the
//! makespan of the work if it were perfectly divisible; the small second term
//! picks, among makespan-optimal splits, the one using the least robot time,
//! which keeps each robot on the work it does best. The resulting shares
//! become each robot's *roles*, which the auction in [`crate::allocator`]
//! honours. Precedence is the scheduler's job, not this plan's.

use crate::config::Config;
use crate::lp::{self, Cmp, Problem};
use crate::registry::Registry;
use arcadia_protocol::CapabilityName;
use std::collections::BTreeMap;

/// Remaining work of one capability.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CapWork {
    /// Work units (m², m³ or kg, per the capability).
    pub units: f64,
    /// Separate work steps (cells, loads, scans).
    pub steps: f64,
    /// Driving per step, m.
    pub travel_m_per_step: f64,
    /// Times each unit is handled (2 for hauling: load, then dump).
    pub passes: f64,
}

/// Remaining work per capability.
pub type Workload = BTreeMap<CapabilityName, CapWork>;

/// Total robot-independent size of a workload (sum of steps), for change detection.
pub fn workload_size(w: &Workload) -> f64 {
    w.values().map(|c| c.steps).sum()
}

const EPSILON: f64 = 1e-3;

/// The crew's planned split of work.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CapacityPlan {
    shares: BTreeMap<(usize, CapabilityName), f64>,
    /// Planned busy time per robot, s.
    pub loads_s: Vec<f64>,
    /// Planned makespan of the remaining work, s.
    pub makespan_s: f64,
    /// Planned energy per robot for its share, Wh.
    pub energy_wh: Vec<f64>,
    /// Workload size when solved.
    pub solved_size: f64,
    /// Robots available when solved.
    pub solved_available: Vec<bool>,
    /// Whether the linear program solved (otherwise shares are an even split).
    pub optimal: bool,
}

/// Energy for robot `r` to do all of `w`, Wh.
pub fn robot_energy_wh(
    reg: &Registry,
    r: usize,
    cap: CapabilityName,
    w: &CapWork,
    cfg: &Config,
) -> f64 {
    let (work_s, travel_s) = robot_times(reg, r, cap, w, cfg);
    if !work_s.is_finite() || !travel_s.is_finite() {
        return f64::INFINITY;
    }
    w.units * w.passes.max(1.0) * reg.energy_per_unit(r, cap)
        + w.steps * w.travel_m_per_step * reg.drive_wh_per_m(r)
        + reg.idle_wh(r, work_s + travel_s)
}

fn robot_times(
    reg: &Registry,
    r: usize,
    cap: CapabilityName,
    w: &CapWork,
    cfg: &Config,
) -> (f64, f64) {
    let rate = reg.rate(r, cap);
    let speed = reg.speed_m_s(r);
    if rate <= 0.0 {
        return (f64::INFINITY, f64::INFINITY);
    }
    let passes = w.passes.max(1.0);
    let work = if cap == CapabilityName::Haul {
        passes * w.units / rate
    } else {
        (passes * w.units / rate).max(w.steps * cfg.min_work_s)
    };
    let drive = w.steps * w.travel_m_per_step;
    let travel = if drive <= 0.0 {
        0.0
    } else if speed > 0.0 {
        drive / speed
    } else {
        f64::INFINITY
    };
    (work, travel)
}

/// Time for robot `r` to do all of `w`, including the charging it implies, s.
pub fn robot_time_s(
    reg: &Registry,
    r: usize,
    cap: CapabilityName,
    w: &CapWork,
    cfg: &Config,
) -> f64 {
    let (work, travel) = robot_times(reg, r, cap, w, cfg);
    let energy = robot_energy_wh(reg, r, cap, w, cfg);
    let charge_w = reg.best_slot_power_w();
    let charging = if charge_w > 0.0 {
        energy / charge_w * 3600.0
    } else {
        0.0
    };
    work + travel + charging
}

impl CapacityPlan {
    /// Robot `r`'s planned fraction of `cap` work.
    pub fn share(&self, r: usize, cap: CapabilityName) -> f64 {
        self.shares.get(&(r, cap)).copied().unwrap_or(0.0)
    }

    /// Whether `cap` is one of robot `r`'s roles.
    pub fn is_role(&self, r: usize, cap: CapabilityName, min_share: f64) -> bool {
        self.share(r, cap) >= min_share
    }

    /// Whether robot `r` has any role at all.
    pub fn has_roles(&self, r: usize, min_share: f64) -> bool {
        self.shares
            .iter()
            .any(|(&(rr, _), &s)| rr == r && s >= min_share)
    }

    /// Robot `r`'s planned load as a fraction of the makespan: near 1 for the
    /// bottleneck, near 0 for robots with slack.
    pub fn utilization(&self, r: usize) -> f64 {
        if self.makespan_s <= 0.0 {
            return 0.0;
        }
        (self.loads_s.get(r).copied().unwrap_or(0.0) / self.makespan_s).clamp(0.0, 1.0)
    }

    /// Energy robot `r` needs for its share of `workload` as it stands now, Wh.
    pub fn energy_need_wh(
        &self,
        r: usize,
        workload: &Workload,
        reg: &Registry,
        cfg: &Config,
    ) -> f64 {
        self.shares
            .iter()
            .filter(|((rr, _), _)| *rr == r)
            .filter_map(|(&(_, cap), &share)| {
                workload
                    .get(&cap)
                    .map(|w| share * robot_energy_wh(reg, r, cap, w, cfg))
            })
            .filter(|e| e.is_finite())
            .sum()
    }

    /// Plan the split of `workload` over the `available` robots.
    pub fn solve(workload: &Workload, reg: &Registry, available: &[bool], cfg: &Config) -> Self {
        let n_robots = reg.len();
        let usable = |r: usize| available.get(r).copied().unwrap_or(false);
        // Variables: one per (robot, capability) pair that can carry work, then T.
        let mut vars: Vec<(usize, CapabilityName, f64)> = Vec::new();
        let mut caps: Vec<CapabilityName> = Vec::new();
        for (&cap, w) in workload {
            if w.units <= 0.0 && w.steps <= 0.0 {
                continue;
            }
            let before = vars.len();
            for r in reg.capable(cap) {
                if !usable(r) {
                    continue;
                }
                let tau = robot_time_s(reg, r, cap, w, cfg) / 3600.0;
                if tau.is_finite() {
                    vars.push((r, cap, tau.max(1e-6)));
                }
            }
            if vars.len() > before {
                caps.push(cap);
            }
        }
        let mut plan = Self {
            loads_s: vec![0.0; n_robots],
            energy_wh: vec![0.0; n_robots],
            solved_size: workload_size(workload),
            solved_available: (0..n_robots).map(usable).collect(),
            ..Self::default()
        };
        if vars.is_empty() {
            plan.optimal = true;
            return plan;
        }

        let nv = vars.len() + 1;
        let t_ix = vars.len();
        let mut costs: Vec<f64> = vars.iter().map(|v| EPSILON * v.2).collect();
        costs.push(1.0);
        let mut rows = Vec::new();
        for &cap in &caps {
            let mut a = vec![0.0; nv];
            for (i, v) in vars.iter().enumerate() {
                if v.1 == cap {
                    a[i] = 1.0;
                }
            }
            rows.push((a, Cmp::Ge, 1.0));
        }
        for r in 0..n_robots {
            if !vars.iter().any(|v| v.0 == r) {
                continue;
            }
            let mut a = vec![0.0; nv];
            for (i, v) in vars.iter().enumerate() {
                if v.0 == r {
                    a[i] = v.2;
                }
            }
            a[t_ix] = -1.0;
            rows.push((a, Cmp::Le, 0.0));
        }

        let fractions: Vec<f64> = match lp::minimize(&Problem { costs, rows }) {
            Ok(x) => {
                plan.optimal = true;
                x[..vars.len()].to_vec()
            }
            Err(_) => vars
                .iter()
                .map(|v| 1.0 / vars.iter().filter(|w| w.1 == v.1).count() as f64)
                .collect(),
        };
        for ((r, cap, tau), f) in vars.iter().zip(fractions) {
            let f = f.clamp(0.0, 1.0);
            if f > 1e-9 {
                plan.shares.insert((*r, *cap), f);
                plan.loads_s[*r] += f * tau * 3600.0;
                if let Some(w) = workload.get(cap) {
                    plan.energy_wh[*r] += f * robot_energy_wh(reg, *r, *cap, w, cfg);
                }
            }
        }
        plan.makespan_s = plan.loads_s.iter().copied().fold(0.0, f64::max);
        plan
    }

    /// Whether the plan should be re-solved for this workload and fleet.
    pub fn stale(&self, workload: &Workload, available: &[bool]) -> bool {
        if self.solved_available != available {
            return true;
        }
        let size = workload_size(workload);
        // Re-plan as work drains (every 15%), or when a capability's work ends.
        let drained = self.solved_size > 0.0 && size < 0.85 * self.solved_size;
        let ended = self
            .shares
            .keys()
            .any(|(_, cap)| workload.get(cap).is_none_or(|w| w.steps <= 0.0));
        drained || ended
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::tests::{cap, robot, scenario};
    use CapabilityName::*;

    fn work(units: f64, steps: f64, passes: f64) -> CapWork {
        CapWork {
            units,
            steps,
            travel_m_per_step: 1.0,
            passes,
        }
    }

    #[test]
    fn specialists_keep_their_specialty() {
        // A digger that can also haul (slowly) and a dedicated fast hauler.
        let reg = Registry::new(&scenario(vec![
            robot(
                "digger",
                0.2,
                vec![
                    cap(Dig, 0.004, 100.0, None),
                    cap(Haul, 2.0, 0.01, Some(250.0)),
                ],
            ),
            robot("hauler", 0.3, vec![cap(Haul, 3.0, 0.008, Some(400.0))]),
        ]));
        let mut w = Workload::new();
        w.insert(Dig, work(20.0, 400.0, 1.0));
        w.insert(Haul, work(10_000.0, 30.0, 2.0));
        let plan = CapacityPlan::solve(&w, &reg, &[true, true], &Config::default());
        assert!(plan.optimal);
        assert!((plan.share(0, Dig) - 1.0).abs() < 1e-6);
        assert!(plan.share(0, Haul) < 0.05, "{}", plan.share(0, Haul));
        assert!((plan.share(1, Haul) - 1.0).abs() < 1e-6);
        assert!(plan.is_role(0, Dig, 0.05) && !plan.is_role(0, Haul, 0.05));
        assert!(plan.makespan_s > 0.0);
    }

    #[test]
    fn generalists_share_the_bottleneck() {
        // Two identical generalists and one compactor: grading is split.
        let gen_caps = || vec![cap(Grade, 0.05, 8.0, None), cap(Compact, 0.1, 4.0, None)];
        let reg = Registry::new(&scenario(vec![
            robot("g1", 0.2, gen_caps()),
            robot("g2", 0.2, gen_caps()),
            robot("c", 0.2, vec![cap(Compact, 0.1, 4.0, None)]),
        ]));
        let mut w = Workload::new();
        w.insert(Grade, work(400.0, 400.0, 1.0));
        w.insert(Compact, work(400.0, 400.0, 1.0));
        let plan = CapacityPlan::solve(&w, &reg, &[true, true, true], &Config::default());
        let grade_total = plan.share(0, Grade) + plan.share(1, Grade);
        assert!((grade_total - 1.0).abs() < 1e-6);
        assert!(plan.share(0, Grade) > 0.2 && plan.share(1, Grade) > 0.2);
        assert!(plan.share(2, Compact) > 0.3, "{}", plan.share(2, Compact));
        // Loads are balanced at the optimum.
        let max = plan.loads_s.iter().copied().fold(0.0, f64::max);
        let min = plan.loads_s.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(max - min < 0.02 * max, "{:?}", plan.loads_s);
    }

    #[test]
    fn unavailable_robots_get_nothing_and_staleness_tracks_changes() {
        let reg = Registry::new(&scenario(vec![
            robot("a", 0.2, vec![cap(Scan, 4.0, 0.05, None)]),
            robot("b", 0.2, vec![cap(Scan, 4.0, 0.05, None)]),
        ]));
        let mut w = Workload::new();
        w.insert(Scan, work(400.0, 10.0, 1.0));
        let plan = CapacityPlan::solve(&w, &reg, &[false, true], &Config::default());
        assert_eq!(plan.share(0, Scan), 0.0);
        assert!((plan.share(1, Scan) - 1.0).abs() < 1e-6);
        assert!(!plan.stale(&w, &[false, true]));
        assert!(plan.stale(&w, &[true, true]));
        let mut drained = w.clone();
        drained.get_mut(&Scan).unwrap().steps = 5.0;
        assert!(plan.stale(&drained, &[false, true]));
        // Nothing to do at all is a valid, empty plan.
        let empty = CapacityPlan::solve(&Workload::new(), &reg, &[true, true], &Config::default());
        assert!(empty.optimal && empty.makespan_s == 0.0);
    }
}
