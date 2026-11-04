//! A small dense linear-programming solver: two-phase simplex with Bland's
//! anti-cycling rule.
//!
//! ArcadiaOS only solves tiny problems (the capacity plan has one variable
//! per robot-capability pair), so a dense tableau is simple, exact enough and
//! fully deterministic. Variables are non-negative.

/// How a constraint row compares with its right-hand side.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cmp {
    /// `a·x <= b`
    Le,
    /// `a·x >= b`
    Ge,
    /// `a·x == b`
    Eq,
}

/// Minimise `costs · x` subject to `rows`, with `x >= 0`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Problem {
    /// Objective coefficients, one per variable.
    pub costs: Vec<f64>,
    /// Constraints `(coefficients, comparison, right-hand side)`.
    pub rows: Vec<(Vec<f64>, Cmp, f64)>,
}

/// Why a problem has no optimal solution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LpError {
    /// Rows of the wrong length or non-finite numbers.
    BadInput,
    /// No point satisfies every constraint.
    Infeasible,
    /// The objective decreases without bound.
    Unbounded,
    /// The iteration cap was hit (should not happen with Bland's rule).
    IterationLimit,
}

const EPS: f64 = 1e-9;
const MAX_ITERATIONS: usize = 50_000;

struct Tableau {
    /// `m` constraint rows then the objective row; the last column is the RHS.
    t: Vec<Vec<f64>>,
    basis: Vec<usize>,
    m: usize,
    cols: usize,
}

impl Tableau {
    fn rhs(&self) -> usize {
        self.cols
    }

    fn pivot(&mut self, r: usize, s: usize) {
        let p = self.t[r][s];
        for v in self.t[r].iter_mut() {
            *v /= p;
        }
        let pivot_row = self.t[r].clone();
        for (i, row) in self.t.iter_mut().enumerate() {
            if i == r {
                continue;
            }
            let f = row[s];
            if f != 0.0 {
                for (v, pv) in row.iter_mut().zip(&pivot_row) {
                    *v -= f * pv;
                }
            }
        }
        self.basis[r] = s;
    }

    /// Load reduced costs for `costs` given the current basis. The last
    /// entry of the objective row ends up holding minus the objective value.
    fn set_objective(&mut self, costs: &[f64]) {
        let m = self.m;
        let mut obj = vec![0.0; self.cols + 1];
        obj[..self.cols].copy_from_slice(costs);
        for i in 0..m {
            let cb = costs[self.basis[i]];
            if cb != 0.0 {
                for (o, v) in obj.iter_mut().zip(&self.t[i]) {
                    *o -= cb * v;
                }
            }
        }
        self.t[m] = obj;
    }

    /// Run simplex iterations over the columns `allowed` admits.
    fn optimise(&mut self, allowed: &dyn Fn(usize) -> bool) -> Result<(), LpError> {
        let (m, rhs) = (self.m, self.rhs());
        for _ in 0..MAX_ITERATIONS {
            // Bland: the lowest-index improving column enters.
            let Some(s) = (0..self.cols).find(|&j| allowed(j) && self.t[m][j] < -EPS) else {
                return Ok(());
            };
            // Minimum ratio; ties go to the lowest basic index.
            let mut leave: Option<(usize, f64)> = None;
            for i in 0..m {
                let a = self.t[i][s];
                if a > EPS {
                    let ratio = self.t[i][rhs] / a;
                    let better = match leave {
                        None => true,
                        Some((li, lr)) => {
                            ratio < lr - EPS
                                || (ratio <= lr + EPS && self.basis[i] < self.basis[li])
                        }
                    };
                    if better {
                        leave = Some((i, ratio));
                    }
                }
            }
            let Some((r, _)) = leave else {
                return Err(LpError::Unbounded);
            };
            self.pivot(r, s);
        }
        Err(LpError::IterationLimit)
    }
}

/// Solve `p`, returning an optimal `x`.
pub fn minimize(p: &Problem) -> Result<Vec<f64>, LpError> {
    let n = p.costs.len();
    let bad = p.costs.iter().any(|c| !c.is_finite())
        || p.rows
            .iter()
            .any(|(a, _, b)| a.len() != n || !b.is_finite() || a.iter().any(|v| !v.is_finite()));
    if bad {
        return Err(LpError::BadInput);
    }
    // Normalise every right-hand side to be non-negative.
    let rows: Vec<(Vec<f64>, Cmp, f64)> = p
        .rows
        .iter()
        .map(|(a, cmp, b)| {
            if *b < 0.0 {
                let flipped = match cmp {
                    Cmp::Le => Cmp::Ge,
                    Cmp::Ge => Cmp::Le,
                    Cmp::Eq => Cmp::Eq,
                };
                (a.iter().map(|v| -v).collect(), flipped, -b)
            } else {
                (a.clone(), *cmp, *b)
            }
        })
        .collect();
    let m = rows.len();
    let n_slack = rows.iter().filter(|r| r.1 != Cmp::Eq).count();
    let n_art = rows.iter().filter(|r| r.1 != Cmp::Le).count();
    let cols = n + n_slack + n_art;
    let mut tab = Tableau {
        t: vec![vec![0.0; cols + 1]; m + 1],
        basis: vec![0; m],
        m,
        cols,
    };
    let mut is_art = vec![false; cols];
    let (mut s, mut a) = (n, n + n_slack);
    for (i, (coef, cmp, b)) in rows.iter().enumerate() {
        tab.t[i][..n].copy_from_slice(coef);
        tab.t[i][cols] = *b;
        match cmp {
            Cmp::Le => {
                tab.t[i][s] = 1.0;
                tab.basis[i] = s;
                s += 1;
            }
            Cmp::Ge => {
                tab.t[i][s] = -1.0;
                s += 1;
                tab.t[i][a] = 1.0;
                is_art[a] = true;
                tab.basis[i] = a;
                a += 1;
            }
            Cmp::Eq => {
                tab.t[i][a] = 1.0;
                is_art[a] = true;
                tab.basis[i] = a;
                a += 1;
            }
        }
    }

    if n_art > 0 {
        let phase1: Vec<f64> = is_art
            .iter()
            .map(|&art| if art { 1.0 } else { 0.0 })
            .collect();
        tab.set_objective(&phase1);
        tab.optimise(&|_| true)?;
        let infeasibility = -tab.t[m][cols];
        let scale = 1.0 + rows.iter().map(|r| r.2).sum::<f64>();
        if infeasibility > 1e-7 * scale {
            return Err(LpError::Infeasible);
        }
        // Drive any artificial left in the basis (at zero) out of it.
        for i in 0..m {
            if is_art[tab.basis[i]]
                && let Some(j) = (0..cols).find(|&j| !is_art[j] && tab.t[i][j].abs() > EPS)
            {
                tab.pivot(i, j);
            }
        }
    }

    let mut phase2 = vec![0.0; cols];
    phase2[..n].copy_from_slice(&p.costs);
    tab.set_objective(&phase2);
    tab.optimise(&|j| !is_art[j])?;

    let mut x = vec![0.0; n];
    for i in 0..m {
        if tab.basis[i] < n {
            x[tab.basis[i]] = tab.t[i][cols].max(0.0);
        }
    }
    Ok(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-7
    }

    #[test]
    fn covering_problem() {
        // min x + y  s.t.  x + 2y >= 4,  3x + y >= 6  →  (1.6, 1.2)
        let p = Problem {
            costs: vec![1.0, 1.0],
            rows: vec![
                (vec![1.0, 2.0], Cmp::Ge, 4.0),
                (vec![3.0, 1.0], Cmp::Ge, 6.0),
            ],
        };
        let x = minimize(&p).unwrap();
        assert!(close(x[0], 1.6) && close(x[1], 1.2), "{x:?}");
    }

    #[test]
    fn packing_problem() {
        // max 3x + 2y  s.t.  x + y <= 4,  x + 3y <= 6,  x <= 3  →  (3, 1)
        let p = Problem {
            costs: vec![-3.0, -2.0],
            rows: vec![
                (vec![1.0, 1.0], Cmp::Le, 4.0),
                (vec![1.0, 3.0], Cmp::Le, 6.0),
                (vec![1.0, 0.0], Cmp::Le, 3.0),
            ],
        };
        let x = minimize(&p).unwrap();
        assert!(close(x[0], 3.0) && close(x[1], 1.0), "{x:?}");
    }

    #[test]
    fn equalities_and_negative_rhs() {
        // min x  s.t.  x + y = 5,  -y >= -2 (y <= 2)  →  x = 3
        let p = Problem {
            costs: vec![1.0, 0.0],
            rows: vec![
                (vec![1.0, 1.0], Cmp::Eq, 5.0),
                (vec![0.0, -1.0], Cmp::Ge, -2.0),
            ],
        };
        let x = minimize(&p).unwrap();
        assert!(close(x[0], 3.0) && close(x[1], 2.0), "{x:?}");
    }

    #[test]
    fn redundant_equalities_are_fine() {
        let p = Problem {
            costs: vec![1.0, 2.0],
            rows: vec![
                (vec![1.0, 1.0], Cmp::Eq, 2.0),
                (vec![2.0, 2.0], Cmp::Eq, 4.0),
            ],
        };
        let x = minimize(&p).unwrap();
        assert!(close(x[0], 2.0) && close(x[1], 0.0), "{x:?}");
    }

    #[test]
    fn detects_infeasible_unbounded_and_bad_input() {
        let infeasible = Problem {
            costs: vec![1.0],
            rows: vec![(vec![1.0], Cmp::Le, 1.0), (vec![1.0], Cmp::Ge, 2.0)],
        };
        assert_eq!(minimize(&infeasible), Err(LpError::Infeasible));
        let unbounded = Problem {
            costs: vec![-1.0],
            rows: vec![(vec![1.0], Cmp::Ge, 1.0)],
        };
        assert_eq!(minimize(&unbounded), Err(LpError::Unbounded));
        let bad = Problem {
            costs: vec![1.0, f64::NAN],
            rows: vec![],
        };
        assert_eq!(minimize(&bad), Err(LpError::BadInput));
        let short = Problem {
            costs: vec![1.0, 1.0],
            rows: vec![(vec![1.0], Cmp::Le, 1.0)],
        };
        assert_eq!(minimize(&short), Err(LpError::BadInput));
    }

    #[test]
    fn degenerate_problem_terminates() {
        // A classic cycling example under the largest-coefficient rule.
        let p = Problem {
            costs: vec![-10.0, 57.0, 9.0, 24.0],
            rows: vec![
                (vec![0.5, -5.5, -2.5, 9.0], Cmp::Le, 0.0),
                (vec![0.5, -1.5, -0.5, 1.0], Cmp::Le, 0.0),
                (vec![1.0, 0.0, 0.0, 0.0], Cmp::Le, 1.0),
            ],
        };
        let x = minimize(&p).unwrap();
        let obj: f64 = p.costs.iter().zip(&x).map(|(c, v)| c * v).sum();
        assert!(close(obj, -1.0), "{x:?} {obj}");
    }
}
