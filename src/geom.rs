//! Grid geometry: distances, rectangles and sweep orders.
//!
//! Coordinates follow the protocol: x grows east, y grows north, and a
//! [`GridRect`] includes both corners. Distances are Euclidean between cell
//! centres, measured in cells; multiply by the site's cell size for metres.

use arcadia_protocol::{GridCoord, GridRect};

/// Euclidean distance between two cells, in cells.
pub fn dist(a: GridCoord, b: GridCoord) -> f64 {
    let dx = f64::from(a.x) - f64::from(b.x);
    let dy = f64::from(a.y) - f64::from(b.y);
    (dx * dx + dy * dy).sqrt()
}

/// Whether `c` lies inside `r`.
pub fn contains(r: &GridRect, c: GridCoord) -> bool {
    (r.min.x..=r.max.x).contains(&c.x) && (r.min.y..=r.max.y).contains(&c.y)
}

/// Width of `r` in cells.
pub fn width(r: &GridRect) -> u32 {
    r.max.x.saturating_sub(r.min.x) + 1
}

/// Height of `r` in cells.
pub fn height(r: &GridRect) -> u32 {
    r.max.y.saturating_sub(r.min.y) + 1
}

/// Number of cells in `r`.
pub fn cell_count(r: &GridRect) -> u64 {
    u64::from(width(r)) * u64::from(height(r))
}

/// Every cell of `r`, row by row from the south-west corner.
pub fn cells(r: &GridRect) -> impl Iterator<Item = GridCoord> + '_ {
    (r.min.y..=r.max.y).flat_map(move |y| (r.min.x..=r.max.x).map(move |x| GridCoord { x, y }))
}

/// The cell of `r` nearest to `c`.
pub fn clamp_into(c: GridCoord, r: &GridRect) -> GridCoord {
    GridCoord {
        x: c.x.clamp(r.min.x, r.max.x),
        y: c.y.clamp(r.min.y, r.max.y),
    }
}

/// Chebyshev (king-move) distance from `c` to the nearest cell of `r`; 0 inside.
pub fn ring_distance(c: GridCoord, r: &GridRect) -> u32 {
    let n = clamp_into(c, r);
    c.x.abs_diff(n.x).max(c.y.abs_diff(n.y))
}

/// Smallest rectangle covering every cell, or `None` for no cells.
pub fn bounding_rect(cells: impl IntoIterator<Item = GridCoord>) -> Option<GridRect> {
    let mut it = cells.into_iter();
    let first = it.next()?;
    let mut r = GridRect {
        min: first,
        max: first,
    };
    for c in it {
        r.min.x = r.min.x.min(c.x);
        r.min.y = r.min.y.min(c.y);
        r.max.x = r.max.x.max(c.x);
        r.max.y = r.max.y.max(c.y);
    }
    Some(r)
}

/// A single-cell rectangle.
pub fn unit(c: GridCoord) -> GridRect {
    GridRect { min: c, max: c }
}

/// Which corner of a rectangle a sweep starts from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Corner {
    /// Start on the east edge instead of the west.
    pub east: bool,
    /// Start on the north edge instead of the south.
    pub north: bool,
}

impl Corner {
    /// The corner of `r` nearest to `p`; ties prefer south and west.
    pub fn nearest(r: &GridRect, p: (f64, f64)) -> Self {
        let mid_x = (f64::from(r.min.x) + f64::from(r.max.x)) / 2.0;
        let mid_y = (f64::from(r.min.y) + f64::from(r.max.y)) / 2.0;
        Self {
            east: p.0 > mid_x,
            north: p.1 > mid_y,
        }
    }
}

/// Boustrophedon order over `r` starting at `corner`: lanes run along the
/// longer side, alternating direction, so consecutive cells are neighbours.
pub fn serpentine(r: &GridRect, corner: Corner) -> Vec<GridCoord> {
    let along_x = width(r) >= height(r);
    let (lanes, steps) = if along_x {
        (height(r), width(r))
    } else {
        (width(r), height(r))
    };
    let mut out = Vec::with_capacity(cell_count(r) as usize);
    for lane in 0..lanes {
        // Lanes advance away from the start corner; steps alternate direction.
        let lane_pos = if along_x {
            if corner.north {
                r.max.y - lane
            } else {
                r.min.y + lane
            }
        } else if corner.east {
            r.max.x - lane
        } else {
            r.min.x + lane
        };
        let forward = (lane % 2 == 0) != if along_x { corner.east } else { corner.north };
        for step in 0..steps {
            let s = if forward { step } else { steps - 1 - step };
            out.push(if along_x {
                GridCoord {
                    x: r.min.x + s,
                    y: lane_pos,
                }
            } else {
                GridCoord {
                    x: lane_pos,
                    y: r.min.y + s,
                }
            });
        }
    }
    out
}

/// Split `r` into bands of whole lanes (rows when the rectangle is wider than
/// tall, otherwise columns), ordered away from `corner`, each holding at least
/// `min_cells` cells where possible.
pub fn bands(r: &GridRect, corner: Corner, min_cells: u32) -> Vec<GridRect> {
    let along_x = width(r) >= height(r);
    let lane_len = if along_x { width(r) } else { height(r) };
    let lanes = if along_x { height(r) } else { width(r) };
    let per_band = min_cells.div_ceil(lane_len.max(1)).clamp(1, lanes.max(1));
    let mut out = Vec::new();
    let mut lane = 0;
    while lane < lanes {
        let last = (lane + per_band - 1).min(lanes - 1);
        let rect = if along_x {
            let (a, b) = if corner.north {
                (r.max.y - last, r.max.y - lane)
            } else {
                (r.min.y + lane, r.min.y + last)
            };
            GridRect {
                min: GridCoord { x: r.min.x, y: a },
                max: GridCoord { x: r.max.x, y: b },
            }
        } else {
            let (a, b) = if corner.east {
                (r.max.x - last, r.max.x - lane)
            } else {
                (r.min.x + lane, r.min.x + last)
            };
            GridRect {
                min: GridCoord { x: a, y: r.min.y },
                max: GridCoord { x: b, y: r.max.y },
            }
        };
        out.push(rect);
        lane = last + 1;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x0: u32, y0: u32, x1: u32, y1: u32) -> GridRect {
        GridRect {
            min: GridCoord { x: x0, y: y0 },
            max: GridCoord { x: x1, y: y1 },
        }
    }

    #[test]
    fn distances_and_containment() {
        let a = GridCoord { x: 0, y: 0 };
        let b = GridCoord { x: 3, y: 4 };
        assert_eq!(dist(a, b), 5.0);
        let r = rect(2, 2, 4, 5);
        assert!(contains(&r, GridCoord { x: 4, y: 5 }));
        assert!(!contains(&r, GridCoord { x: 5, y: 5 }));
        assert_eq!(cell_count(&r), 12);
        assert_eq!(clamp_into(a, &r), GridCoord { x: 2, y: 2 });
        assert_eq!(ring_distance(GridCoord { x: 0, y: 3 }, &r), 2);
        assert_eq!(ring_distance(GridCoord { x: 3, y: 3 }, &r), 0);
        assert_eq!(cells(&r).count(), 12);
    }

    #[test]
    fn bounding_rect_covers_all() {
        let r = bounding_rect([
            GridCoord { x: 5, y: 1 },
            GridCoord { x: 2, y: 7 },
            GridCoord { x: 3, y: 3 },
        ])
        .unwrap();
        assert_eq!(r, rect(2, 1, 5, 7));
        assert!(bounding_rect(std::iter::empty()).is_none());
    }

    #[test]
    fn serpentine_visits_every_cell_once_with_neighbour_steps() {
        for corner in [
            Corner {
                east: false,
                north: false,
            },
            Corner {
                east: true,
                north: false,
            },
            Corner {
                east: false,
                north: true,
            },
            Corner {
                east: true,
                north: true,
            },
        ] {
            for r in [rect(10, 20, 14, 22), rect(0, 0, 2, 6)] {
                let s = serpentine(&r, corner);
                assert_eq!(s.len() as u64, cell_count(&r));
                let mut sorted = s.clone();
                sorted.sort_by_key(|c| (c.x, c.y));
                sorted.dedup();
                assert_eq!(sorted.len(), s.len());
                assert!(s.windows(2).all(|w| dist(w[0], w[1]) == 1.0), "{corner:?}");
                let start = s[0];
                assert_eq!(start.x, if corner.east { r.max.x } else { r.min.x });
                assert_eq!(start.y, if corner.north { r.max.y } else { r.min.y });
            }
        }
    }

    #[test]
    fn bands_partition_the_rectangle_in_sweep_order() {
        let r = rect(20, 20, 39, 39);
        let corner = Corner {
            east: false,
            north: true,
        };
        let b = bands(&r, corner, 40);
        assert_eq!(b.len(), 10);
        assert_eq!(b[0], rect(20, 38, 39, 39));
        assert_eq!(b.iter().map(cell_count).sum::<u64>(), 400);
        // Odd remainder ends in a thinner band; tall rectangles band by columns.
        let tall = rect(0, 0, 2, 9);
        let tb = bands(
            &tall,
            Corner {
                east: false,
                north: false,
            },
            25,
        );
        assert_eq!(tb, vec![rect(0, 0, 2, 9)]);
        let tb = bands(
            &tall,
            Corner {
                east: true,
                north: false,
            },
            10,
        );
        assert_eq!(
            tb,
            vec![rect(2, 0, 2, 9), rect(1, 0, 1, 9), rect(0, 0, 0, 9)]
        );
    }

    #[test]
    fn nearest_corner() {
        let r = rect(20, 20, 39, 39);
        assert_eq!(
            Corner::nearest(&r, (13.5, 10.0)),
            Corner {
                east: false,
                north: false
            }
        );
        assert_eq!(
            Corner::nearest(&r, (50.0, 50.0)),
            Corner {
                east: true,
                north: true
            }
        );
    }
}
