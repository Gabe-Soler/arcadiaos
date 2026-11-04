//! Site coverage as ArcadiaOS works it out (semantics §8.4).
//!
//! A cell is covered when it is within the base station's range, or within
//! the relay range of a holding relay that is itself covered, transitively.
//! ArcadiaOS knows the base station from the scenario and its own relays from
//! the commands it sent; each robot's snapshot says whether the site can hear
//! it right now (`in_relay_coverage`).
//!
//! Coverage shapes planning in three ways: work is kept inside coverage where
//! it can be (scans are entered from a covered cell, berm material is cast
//! from covered cells), work that must leave coverage is costed with the
//! lost-link wait it brings, and a relay is stationed when that saves more
//! than it costs (see the coordinator).

use crate::geom;
use arcadia_protocol::{BaseStation, GridCoord, GridRect};

/// Whether two cells are within `range_m` of each other, between cell
/// centres; the boundary counts.
pub fn within(a: GridCoord, b: GridCoord, range_m: f64, cell_m: f64) -> bool {
    let dx = f64::from(a.x) - f64::from(b.x);
    let dy = f64::from(a.y) - f64::from(b.y);
    (dx * dx + dy * dy) * cell_m * cell_m <= range_m * range_m
}

/// The covered part of the site.
#[derive(Debug, Clone, PartialEq)]
pub struct Coverage {
    base: BaseStation,
    cell_m: f64,
    /// Relays that extend coverage: holding, and themselves covered.
    relays: Vec<(GridCoord, f64)>,
}

impl Coverage {
    /// Coverage from the base station and holding relays `(position,
    /// range)`. A relay that is not itself covered, directly or through
    /// another relay, extends nothing.
    pub fn new(base: &BaseStation, cell_m: f64, holding: &[(GridCoord, f64)]) -> Self {
        let mut cov = Self {
            base: base.clone(),
            cell_m: if cell_m > 0.0 { cell_m } else { 1.0 },
            relays: Vec::new(),
        };
        let mut left: Vec<(GridCoord, f64)> = holding.to_vec();
        loop {
            let reached: Vec<usize> = (0..left.len()).filter(|&i| cov.covers(left[i].0)).collect();
            if reached.is_empty() {
                break;
            }
            for &i in reached.iter().rev() {
                cov.relays.push(left.remove(i));
            }
        }
        cov.relays.sort_by(|a, b| {
            (a.0.y, a.0.x)
                .cmp(&(b.0.y, b.0.x))
                .then(a.1.total_cmp(&b.1))
        });
        cov
    }

    /// Whether the base station alone covers `c`.
    pub fn base_covers(&self, c: GridCoord) -> bool {
        within(c, self.base.position, self.base.range_m, self.cell_m)
    }

    /// Whether `c` is covered.
    pub fn covers(&self, c: GridCoord) -> bool {
        self.base_covers(c)
            || self
                .relays
                .iter()
                .any(|&(p, range)| within(c, p, range, self.cell_m))
    }

    /// Relays extending coverage, as `(position, range)`.
    pub fn relays(&self) -> &[(GridCoord, f64)] {
        &self.relays
    }

    /// How far `c` is outside coverage, m (zero or negative inside).
    pub fn gap_m(&self, c: GridCoord) -> f64 {
        let gap = |p: GridCoord, range: f64| geom::dist(c, p) * self.cell_m - range;
        self.relays
            .iter()
            .map(|&(p, range)| gap(p, range))
            .fold(gap(self.base.position, self.base.range_m), f64::min)
    }
}

/// The area to scan from `from` so that the cell a robot enters it by is
/// covered: `area` itself when that cell already is, otherwise `area` grown
/// toward `from` one row or column at a time (at most `max_grow` steps,
/// within a `width` × `height` site), each time on the side that brings the
/// entry cell nearest to coverage. `None` when that does not help: the robot
/// would scan from outside coverage.
pub fn safe_scan_area(
    area: &GridRect,
    from: GridCoord,
    cov: &Coverage,
    width: u32,
    height: u32,
    max_grow: u32,
) -> Option<GridRect> {
    let mut a = *area;
    for _ in 0..=max_grow {
        if cov.covers(geom::clamp_into(from, &a)) {
            return Some(a);
        }
        let mut options: Vec<GridRect> = Vec::new();
        if from.y < a.min.y {
            let mut g = a;
            g.min.y -= 1;
            options.push(g);
        }
        if from.y > a.max.y && a.max.y + 1 < height {
            let mut g = a;
            g.max.y += 1;
            options.push(g);
        }
        if from.x < a.min.x {
            let mut g = a;
            g.min.x -= 1;
            options.push(g);
        }
        if from.x > a.max.x && a.max.x + 1 < width {
            let mut g = a;
            g.max.x += 1;
            options.push(g);
        }
        a = options.into_iter().min_by(|p, q| {
            cov.gap_m(geom::clamp_into(from, p))
                .total_cmp(&cov.gap_m(geom::clamp_into(from, q)))
        })?;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(x: u32, y: u32) -> GridCoord {
        GridCoord { x, y }
    }

    fn base() -> BaseStation {
        BaseStation {
            position: at(30, 8),
            range_m: 33.0,
        }
    }

    #[test]
    fn range_is_between_cell_centres_and_includes_the_edge() {
        assert!(within(at(30, 41), at(30, 8), 33.0, 1.0));
        assert!(!within(at(29, 41), at(30, 8), 33.0, 1.0));
        assert!(within(at(3, 4), at(0, 0), 10.0, 2.0));
    }

    #[test]
    fn relays_extend_coverage_only_from_coverage_and_transitively() {
        let chain = [(at(30, 39), 15.0), (at(30, 52), 15.0), (at(5, 59), 100.0)];
        let cov = Coverage::new(&base(), 1.0, &chain);
        assert!(cov.covers(at(30, 59)), "through both relays");
        assert_eq!(cov.relays().len(), 2, "the far relay is not itself covered");
        assert!(!Coverage::new(&base(), 1.0, &chain[1..]).covers(at(30, 59)));
        assert!(cov.base_covers(at(30, 40)) && !cov.base_covers(at(30, 42)));
    }

    #[test]
    fn scans_are_entered_from_a_covered_cell() {
        let cov = Coverage::new(&base(), 1.0, &[]);
        // A berm line just out of coverage, scanned by a robot on the pad.
        let berm = GridRect {
            min: at(24, 42),
            max: at(35, 42),
        };
        let a = safe_scan_area(&berm, at(25, 30), &cov, 60, 60, 4).unwrap();
        assert_eq!(
            a,
            GridRect {
                min: at(24, 40),
                max: at(35, 42)
            }
        );
        assert!(cov.covers(geom::clamp_into(at(25, 30), &a)));
        // From the west it still grows toward the base station, not away from it.
        let a = safe_scan_area(&berm, at(10, 30), &cov, 60, 60, 4).unwrap();
        assert_eq!(a.min.y, 40);
        // Already covered: unchanged.
        let pad = GridRect {
            min: at(20, 20),
            max: at(39, 39),
        };
        assert_eq!(safe_scan_area(&pad, at(10, 10), &cov, 60, 60, 4), Some(pad));
        // Too far to help within the limit.
        let far = GridRect {
            min: at(0, 59),
            max: at(0, 59),
        };
        assert_eq!(safe_scan_area(&far, at(30, 30), &cov, 60, 60, 4), None);
    }
}
