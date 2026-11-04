//! What ArcadiaOS knows about each cell of the site.
//!
//! Knowledge starts from the orbital prior in `hello`, is replaced by scan
//! results, and is carried forward between scans by modelling the effect of
//! every completed command. Each cell keeps the time of its last known change
//! and the issue time of the latest scan that covered it afterwards, which is
//! what verification needs.

use crate::config::Config;
use arcadia_protocol::{CellObservation, GridCoord, SiteMap, Surface};
use std::f64::consts::PI;

/// Knowledge about one cell.
#[derive(Debug, Clone, PartialEq)]
pub struct CellKnowledge {
    /// Orbital prior elevation, m.
    pub prior_elevation_m: Option<f64>,
    /// Best current estimate of the surface elevation, m (scan or modelled).
    pub elevation_m: Option<f64>,
    /// Completion time of the latest scan of this cell.
    pub scanned_t: Option<f64>,
    /// Estimated surface state.
    pub surface: Surface,
    /// Total boulder volume, m³; `None` until scanned.
    pub boulder_volume_m3: Option<f64>,
    /// Regolith bulk density, kg/m³, from a scan.
    pub density_kg_m3: Option<f64>,
    /// Time of the last change ArcadiaOS knows about.
    pub changed_t: f64,
    /// Issue time of a scan that covered the cell after its last change.
    pub verified_t: Option<f64>,
    /// An operation went wrong here; rescan before planning more work.
    pub stale: bool,
}

impl Default for CellKnowledge {
    fn default() -> Self {
        Self {
            prior_elevation_m: None,
            elevation_m: None,
            scanned_t: None,
            surface: Surface::Natural,
            boulder_volume_m3: None,
            density_kg_m3: None,
            changed_t: f64::NEG_INFINITY,
            verified_t: None,
            stale: false,
        }
    }
}

impl CellKnowledge {
    /// Best elevation estimate: modelled or scanned, else the orbital prior.
    pub fn elevation(&self) -> Option<f64> {
        self.elevation_m.or(self.prior_elevation_m)
    }

    /// Whether a scan has measured this cell and nothing has made it stale since.
    pub fn surveyed(&self) -> bool {
        self.scanned_t.is_some() && !self.stale
    }

    /// Whether the latest change has been covered by a later scan.
    pub fn verified(&self) -> bool {
        self.verified_t.is_some()
    }

    fn changed(&mut self, t: f64) {
        self.changed_t = t;
        self.verified_t = None;
    }
}

/// Knowledge of the whole site grid.
#[derive(Debug, Clone, PartialEq)]
pub struct SiteKnowledge {
    /// Cells west to east.
    pub width: u32,
    /// Cells south to north.
    pub height: u32,
    /// Cell edge length, m.
    pub cell_size_m: f64,
    cells: Vec<CellKnowledge>,
}

impl SiteKnowledge {
    /// Start from the prior map; prior entries outside the grid are ignored.
    pub fn from_prior(map: &SiteMap) -> Self {
        let n = map.width as usize * map.height as usize;
        let mut site = Self {
            width: map.width,
            height: map.height,
            cell_size_m: if map.cell_size_m.is_finite() && map.cell_size_m > 0.0 {
                map.cell_size_m
            } else {
                1.0
            },
            cells: vec![CellKnowledge::default(); n],
        };
        for obs in &map.cells {
            if let Some(k) = site.get_mut(obs.at) {
                k.prior_elevation_m = obs.cell.elevation_m.filter(|e| e.is_finite());
                if let Some(s) = obs.cell.surface {
                    k.surface = s;
                }
            }
        }
        site
    }

    /// Whether `c` is on the site.
    pub fn in_bounds(&self, c: GridCoord) -> bool {
        c.x < self.width && c.y < self.height
    }

    /// Area of one cell, m².
    pub fn cell_area_m2(&self) -> f64 {
        self.cell_size_m * self.cell_size_m
    }

    fn idx(&self, c: GridCoord) -> Option<usize> {
        self.in_bounds(c)
            .then(|| c.y as usize * self.width as usize + c.x as usize)
    }

    /// Knowledge for `c`, if on the site.
    pub fn get(&self, c: GridCoord) -> Option<&CellKnowledge> {
        self.idx(c).map(|i| &self.cells[i])
    }

    /// Mutable knowledge for `c`, if on the site.
    pub fn get_mut(&mut self, c: GridCoord) -> Option<&mut CellKnowledge> {
        self.idx(c).map(|i| &mut self.cells[i])
    }

    /// Density for `c`: scanned, else the configured default.
    pub fn density(&self, c: GridCoord, cfg: &Config) -> f64 {
        self.get(c)
            .and_then(|k| k.density_kg_m3)
            .filter(|d| d.is_finite() && *d > 0.0)
            .unwrap_or(cfg.default_density_kg_m3)
    }

    /// Mass that raises or lowers `c` by `dh` metres, kg.
    pub fn mass_for_height(&self, c: GridCoord, dh: f64, cfg: &Config) -> f64 {
        dh * self.density(c, cfg) * self.cell_area_m2()
    }

    /// Apply a scan result. `issued_t` is when the scan command was sent; a
    /// cell is verified when its last known change was no later than that.
    pub fn apply_scan(&mut self, cells: &[CellObservation], issued_t: f64, t: f64) {
        for obs in cells {
            let Some(k) = self.get_mut(obs.at) else {
                continue;
            };
            let cell = &obs.cell;
            if let Some(e) = cell.elevation_m.filter(|e| e.is_finite()) {
                k.elevation_m = Some(e);
            }
            if let Some(s) = cell.surface {
                k.surface = s;
            }
            if let Some(b) = &cell.boulders {
                k.boulder_volume_m3 = Some(
                    b.iter()
                        .map(|b| PI / 6.0 * b.diameter_m.max(0.0).powi(3))
                        .sum(),
                );
            }
            if let Some(d) = cell
                .regolith
                .as_ref()
                .and_then(|r| r.bulk_density_kg_m3)
                .filter(|d| d.is_finite() && *d > 0.0)
            {
                k.density_kg_m3 = Some(d);
            }
            k.scanned_t = Some(t);
            k.stale = false;
            if k.changed_t <= issued_t {
                k.verified_t = Some(issued_t);
            }
        }
    }

    /// Record a completed dig of `c` to `target_m`; returns the estimated spoil, kg.
    pub fn record_dig(&mut self, c: GridCoord, target_m: f64, t: f64, cfg: &Config) -> f64 {
        let spoil = self.dig_spoil_kg(c, target_m, cfg);
        let Some(k) = self.get_mut(c) else {
            return 0.0;
        };
        if let Some(e) = k.elevation()
            && e > target_m
        {
            k.elevation_m = Some(target_m);
        }
        k.surface = Surface::Cleared;
        k.boulder_volume_m3 = Some(0.0);
        k.changed(t);
        spoil
    }

    /// Estimated spoil from digging `c` to `target_m`: cut plus boulders, kg.
    pub fn dig_spoil_kg(&self, c: GridCoord, target_m: f64, cfg: &Config) -> f64 {
        let Some(k) = self.get(c) else {
            return 0.0;
        };
        let cut = k.elevation().map_or(0.0, |e| (e - target_m).max(0.0));
        cut * self.cell_area_m2() * self.density(c, cfg)
            + k.boulder_volume_m3.unwrap_or(0.0) * cfg.rock_density_kg_m3
    }

    /// Dig work units for `c` to `target_m`: cut plus boulder volume, m³.
    pub fn dig_units_m3(&self, c: GridCoord, target_m: f64) -> f64 {
        let Some(k) = self.get(c) else {
            return 0.0;
        };
        let cut = k.elevation().map_or(0.0, |e| (e - target_m).max(0.0));
        cut * self.cell_area_m2() + k.boulder_volume_m3.unwrap_or(0.0)
    }

    /// Record a completed grade of `c` to `target_m`.
    pub fn record_grade(&mut self, c: GridCoord, target_m: f64, t: f64) {
        if let Some(k) = self.get_mut(c) {
            k.elevation_m = Some(target_m);
            k.surface = Surface::Graded;
            k.changed(t);
        }
    }

    /// Record a completed compaction of `c`, settling it by `settle_m`.
    pub fn record_compact(&mut self, c: GridCoord, settle_m: f64, t: f64) {
        if let Some(k) = self.get_mut(c) {
            k.elevation_m = k.elevation().map(|e| e - settle_m);
            k.surface = Surface::Compacted;
            k.changed(t);
        }
    }

    /// Record a completed sinter of `c`.
    pub fn record_sinter(&mut self, c: GridCoord, t: f64) {
        if let Some(k) = self.get_mut(c) {
            k.surface = Surface::Stabilized;
            k.changed(t);
        }
    }

    /// Record `kg` dumped on `c`. Berm cells become berm; untouched ground or
    /// piles become stockpile; prepared ground reverts to cleared.
    pub fn record_deposit(&mut self, c: GridCoord, kg: f64, berm: bool, t: f64, cfg: &Config) {
        if kg <= 0.0 {
            return;
        }
        let dh = kg / (self.density(c, cfg) * self.cell_area_m2());
        if let Some(k) = self.get_mut(c) {
            k.elevation_m = k.elevation().map(|e| e + dh);
            k.surface = if berm {
                Surface::Berm
            } else if matches!(k.surface, Surface::Natural | Surface::Stockpile) {
                Surface::Stockpile
            } else {
                Surface::Cleared
            };
            k.changed(t);
        }
    }

    /// Record `kg` loaded from `c`.
    pub fn record_removal(&mut self, c: GridCoord, kg: f64, t: f64, cfg: &Config) {
        if kg <= 0.0 {
            return;
        }
        let dh = kg / (self.density(c, cfg) * self.cell_area_m2());
        if let Some(k) = self.get_mut(c) {
            k.elevation_m = k.elevation().map(|e| e - dh);
            k.changed(t);
        }
    }

    /// Something unexpected happened at `c`: plan a rescan before more work.
    pub fn mark_stale(&mut self, c: GridCoord) {
        if let Some(k) = self.get_mut(c) {
            k.stale = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arcadia_protocol::{Boulder, Cell, Regolith};

    fn site() -> SiteKnowledge {
        SiteKnowledge::from_prior(&SiteMap {
            site_id: "s".into(),
            name: None,
            width: 4,
            height: 3,
            cell_size_m: 1.0,
            cells: vec![
                CellObservation {
                    at: GridCoord { x: 1, y: 1 },
                    cell: Cell {
                        elevation_m: Some(-10.0),
                        ..Cell::default()
                    },
                },
                CellObservation {
                    at: GridCoord { x: 9, y: 9 },
                    cell: Cell::default(),
                },
            ],
        })
    }

    fn scan(c: GridCoord, elev: f64, surface: Surface, boulder_d: Option<f64>) -> CellObservation {
        CellObservation {
            at: c,
            cell: Cell {
                elevation_m: Some(elev),
                regolith: Some(Regolith {
                    depth_m: Some(1.0),
                    bulk_density_kg_m3: Some(1500.0),
                    cohesion_kpa: None,
                }),
                boulders: Some(
                    boulder_d
                        .map(|d| {
                            vec![Boulder {
                                diameter_m: d,
                                mass_kg: None,
                            }]
                        })
                        .unwrap_or_default(),
                ),
                surface: Some(surface),
                ice_depth_m: None,
            },
        }
    }

    #[test]
    fn prior_then_scan_then_verification() {
        let mut s = site();
        let c = GridCoord { x: 1, y: 1 };
        assert_eq!(s.get(c).unwrap().elevation(), Some(-10.0));
        assert!(!s.get(c).unwrap().surveyed());
        assert!(s.get(GridCoord { x: 9, y: 9 }).is_none());

        s.apply_scan(&[scan(c, -10.2, Surface::Natural, Some(0.5))], 5.0, 10.0);
        let k = s.get(c).unwrap();
        assert!(k.surveyed());
        assert_eq!(k.elevation(), Some(-10.2));
        assert!((k.boulder_volume_m3.unwrap() - PI / 6.0 * 0.125).abs() < 1e-12);
        assert_eq!(s.density(c, &Config::default()), 1500.0);

        // A change after the scan's issue time is not verified by it.
        s.record_sinter(c, 20.0);
        assert!(!s.get(c).unwrap().verified());
        s.apply_scan(&[scan(c, -10.2, Surface::Stabilized, None)], 15.0, 30.0);
        assert!(!s.get(c).unwrap().verified());
        s.apply_scan(&[scan(c, -10.2, Surface::Stabilized, None)], 20.0, 40.0);
        assert!(s.get(c).unwrap().verified());
    }

    #[test]
    fn modelled_operations_track_elevation_and_surface() {
        let cfg = Config::default();
        let mut s = site();
        let c = GridCoord { x: 1, y: 1 };
        s.apply_scan(&[scan(c, -9.8, Surface::Natural, Some(0.2))], 0.0, 1.0);
        let spoil = s.record_dig(c, -10.0, 2.0, &cfg);
        let boulder = PI / 6.0 * 0.008 * cfg.rock_density_kg_m3;
        assert!((spoil - (0.2 * 1500.0 + boulder)).abs() < 1e-6, "{spoil}");
        let k = s.get(c).unwrap();
        assert_eq!(k.surface, Surface::Cleared);
        assert_eq!(k.elevation(), Some(-10.0));
        assert_eq!(k.boulder_volume_m3, Some(0.0));

        s.record_deposit(c, 150.0, false, 3.0, &cfg);
        assert!((s.get(c).unwrap().elevation().unwrap() + 9.9).abs() < 1e-9);
        assert_eq!(s.get(c).unwrap().surface, Surface::Cleared);
        s.record_grade(c, -10.0, 4.0);
        s.record_compact(c, 0.01, 5.0);
        assert!((s.get(c).unwrap().elevation().unwrap() + 10.01).abs() < 1e-9);
        assert_eq!(s.get(c).unwrap().surface, Surface::Compacted);

        // Untouched ground becomes stockpile; berm cells become berm.
        let n = GridCoord { x: 0, y: 0 };
        s.record_deposit(n, 10.0, false, 6.0, &cfg);
        assert_eq!(s.get(n).unwrap().surface, Surface::Stockpile);
        s.record_deposit(n, 10.0, true, 7.0, &cfg);
        assert_eq!(s.get(n).unwrap().surface, Surface::Berm);
        s.mark_stale(n);
        assert!(s.get(n).unwrap().stale);
    }
}
