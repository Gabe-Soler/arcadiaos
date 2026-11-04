//! Planner: turns the `landing_pad` goal into a task graph over the site.
//!
//! The pad is cut into sections (bands of whole lanes) and swept in a
//! boustrophedon order from the corner nearest the crew. Each pad cell moves
//! through survey → dig → (fill) → grade → compact → sinter → verify on its
//! own, so sections progress independently and in parallel. The plan is
//! *derived*, not stored: every decision recomputes each cell's next step from
//! the world model, which makes replanning after surprises automatic.
//!
//! Alongside the pad cells the planner owns:
//! - the material ledger ([`material`]): stockpile siting, spoil casting, haul
//!   sources and borrow when spoil runs short;
//! - the blast berm: each berm cell's surveyed base, deposits and
//!   confirmation. A berm cell outside the base station's coverage is built
//!   from a borrow cell inside it, by digs that cast their spoil onto the
//!   berm, so no robot has to leave contact to build it;
//! - verification batching, rework of defective cells, and a recheck loop for
//!   the rare case where every cell looks done but the run has not ended.

pub mod material;

use crate::capacity::{CapWork, Workload};
use crate::config::Config;
use crate::coverage::Coverage;
use crate::geom::{self, Corner};
use crate::registry::Registry;
use crate::tasks::{Stage, Task, TaskKey};
use crate::world::WorldModel;
use arcadia_protocol::{
    CapabilityName, CellObservation, GridCoord, GridRect, LandingPadSpec, Surface,
};
use material::Material;
use std::collections::BTreeSet;

/// Cells and task keys that in-flight commands already cover.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Busy {
    /// Cells with a ground-changing command in flight (one at a time per cell).
    pub cells: BTreeSet<(u32, u32)>,
    /// Keys of tasks in flight.
    pub keys: BTreeSet<TaskKey>,
}

impl Busy {
    /// Whether `c` already has a ground-changing command in flight.
    pub fn cell(&self, c: GridCoord) -> bool {
        self.cells.contains(&(c.x, c.y))
    }
}

/// What one pad cell needs next.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CellStatus {
    /// Needs a (re)scan before anything else.
    pub survey: bool,
    /// Needs clearing or cutting.
    pub dig: bool,
    /// Fill still wanted, kg.
    pub fill_kg: f64,
    /// Most that may be dumped here without overshooting, kg.
    pub max_fill_kg: f64,
    /// Ready to grade.
    pub grade: bool,
    /// Ready to compact.
    pub compact: bool,
    /// Ready to sinter.
    pub sinter: bool,
    /// In the required surface state and within tolerance.
    pub finished: bool,
    /// Finished and covered by a scan since its last change.
    pub verified: bool,
}

/// One blast-berm cell.
#[derive(Debug, Clone, PartialEq)]
pub struct BermCell {
    /// Cell.
    pub at: GridCoord,
    /// Ground elevation before any deposit, from the first scan, m.
    pub base_m: Option<f64>,
    /// Material deposited so far (estimated), kg.
    pub deposited_kg: f64,
    /// Extra material requested after a failed check, kg.
    pub extra_kg: f64,
    /// A scan after the last deposit confirmed the height.
    pub confirmed: bool,
}

/// A cell inside the base station's coverage that material for a berm cell
/// outside it is dug from, the spoil cast straight onto the berm, so building
/// the berm never takes a robot out of contact.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Borrow {
    /// The borrow cell.
    pub at: GridCoord,
    /// Deepest it may be dug, m: the first known elevation (orbital prior or
    /// first scan) less the allowed depth. Unknown until then.
    pub floor_m: Option<f64>,
}

/// How a berm cell's next load can be dug from its borrow cell.
enum BorrowDig {
    /// A dig task is ready.
    Ready(Task),
    /// A dig is already in flight there.
    Busy,
    /// No borrow cell, or it is dug out: haul instead.
    Unavailable,
}

/// Where a dig sends spoil that does not fit in the digger's payload.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpoilKind {
    /// A stockpile site.
    Pile(usize),
    /// Straight onto a berm cell within reach.
    Berm(usize),
    /// Straight into a fill cell within reach.
    Fill,
}

/// A spoil destination.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpoilTarget {
    /// Cell.
    pub at: GridCoord,
    /// What kind of destination it is.
    pub kind: SpoilKind,
}

/// Overall progress, as the world model sees it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Progress {
    /// Pad cells.
    pub pad_cells: usize,
    /// Pad cells in spec.
    pub finished: usize,
    /// Pad cells in spec and verified.
    pub verified: usize,
    /// Berm cells.
    pub berm_cells: usize,
    /// Berm cells confirmed by a scan.
    pub berm_confirmed: usize,
}

impl Progress {
    /// Whether everything looks done and verified.
    pub fn complete(&self) -> bool {
        self.verified == self.pad_cells && self.berm_confirmed == self.berm_cells
    }

    /// Pad cells still to finish.
    pub fn unfinished(&self) -> usize {
        self.pad_cells - self.finished
    }
}

/// State of the recheck loop.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Recheck {
    /// A whole-site recheck scan is wanted.
    pub wanted: bool,
    /// Recheck rounds completed.
    pub rounds: u32,
}

/// Height margin a berm check needs above the specified height, m.
const BERM_CHECK_MARGIN_M: f64 = 0.02;

/// The landing-pad plan.
#[derive(Debug, Clone, PartialEq)]
pub struct Planner {
    /// Goal being built.
    pub goal_id: String,
    /// Pad spec.
    pub pad: LandingPadSpec,
    order: Vec<GridCoord>,
    rank: Vec<u32>,
    /// Pad sections in sweep order.
    pub sections: Vec<GridRect>,
    section_of: Vec<u32>,
    /// Berm cells.
    pub berm: Vec<BermCell>,
    /// For each berm cell outside the base station's coverage, its borrow cell.
    pub borrow: Vec<Option<Borrow>>,
    /// Most payload room a digger can have, kg: borrow digs are sized so the
    /// overflow covers the berm's need even when the payload is empty.
    dig_room_kg: f64,
    /// Material ledger.
    pub material: Material,
    rework: BTreeSet<(u32, u32)>,
    /// Cells where grading was refused: brought much closer to target first.
    strict: BTreeSet<(u32, u32)>,
    /// Recheck loop state.
    pub recheck: Recheck,
    /// Whether any robot can haul; without one, spoil is cast straight into
    /// fill and berm cells at any distance.
    haulers: bool,
}

fn surface_level(s: Surface) -> u8 {
    match s {
        Surface::Natural | Surface::Stockpile | Surface::Berm => 0,
        Surface::Cleared => 1,
        Surface::Graded => 2,
        Surface::Compacted => 3,
        Surface::Stabilized => 4,
    }
}

impl Planner {
    /// Plan `pad` for the world as currently known. Pad cells off the site are
    /// clipped; berm cells inside the pad or off the site are ignored.
    pub fn new(
        goal_id: &str,
        pad: &LandingPadSpec,
        world: &WorldModel,
        reg: &Registry,
        cfg: &Config,
    ) -> Self {
        let mut spec = pad.clone();
        let w = world.site.width.max(1);
        let h = world.site.height.max(1);
        let area = GridRect {
            min: GridCoord {
                x: spec.area.min.x.min(spec.area.max.x).min(w - 1),
                y: spec.area.min.y.min(spec.area.max.y).min(h - 1),
            },
            max: GridCoord {
                x: spec.area.max.x.max(spec.area.min.x).min(w - 1),
                y: spec.area.max.y.max(spec.area.min.y).min(h - 1),
            },
        };
        spec.area = area;
        // Sweep from the pad corner nearest the crew (or the stations).
        let anchor = world
            .fleet
            .centroid()
            .or_else(|| {
                let s = &world.stations;
                (!s.is_empty()).then(|| {
                    let n = s.len() as f64;
                    (
                        s.iter().map(|s| f64::from(s.position.x)).sum::<f64>() / n,
                        s.iter().map(|s| f64::from(s.position.y)).sum::<f64>() / n,
                    )
                })
            })
            .unwrap_or((0.0, 0.0));
        let corner = Corner::nearest(&area, anchor);
        let order = geom::serpentine(&area, corner);
        let sections = geom::bands(&area, corner, cfg.section_cells.max(1));
        let n = order.len();
        let mut rank = vec![0; n];
        let mut section_of = vec![0; n];
        let local =
            |c: GridCoord| ((c.y - area.min.y) * geom::width(&area) + (c.x - area.min.x)) as usize;
        for (i, &c) in order.iter().enumerate() {
            rank[local(c)] = i as u32;
        }
        for (s, r) in sections.iter().enumerate() {
            for c in geom::cells(r) {
                section_of[local(c)] = s as u32;
            }
        }
        let mut seen = BTreeSet::new();
        let berm: Vec<BermCell> = spec
            .berm
            .iter()
            .flat_map(|b| b.cells.iter().copied())
            .filter(|&c| world.site.in_bounds(c) && !geom::contains(&area, c))
            .filter(|c| seen.insert((c.x, c.y)))
            .map(|at| BermCell {
                at,
                base_m: None,
                deposited_kg: 0.0,
                extra_kg: 0.0,
                confirmed: false,
            })
            .collect();
        // Borrow digs need a digger; without one the berm is hauled as usual.
        let borrow = if reg.capable(CapabilityName::Dig).is_empty() {
            vec![None; berm.len()]
        } else {
            borrow_cells(&area, &berm, world, cfg)
        };
        let blocked: Vec<GridCoord> = berm
            .iter()
            .map(|b| b.at)
            .chain(borrow.iter().flatten().map(|b| b.at))
            .collect();
        let material = Material::new(&area, &blocked, world, cfg);
        let dig_room_kg = reg
            .capable(CapabilityName::Dig)
            .iter()
            .map(|&r| reg.haul_capacity_kg(r))
            .fold(0.0, f64::max);
        Self {
            goal_id: goal_id.into(),
            pad: spec,
            order,
            rank,
            sections,
            section_of,
            berm,
            borrow,
            dig_room_kg,
            material,
            rework: BTreeSet::new(),
            strict: BTreeSet::new(),
            recheck: Recheck::default(),
            haulers: !reg.capable(CapabilityName::Haul).is_empty(),
        }
    }

    fn local(&self, c: GridCoord) -> Option<usize> {
        let a = &self.pad.area;
        geom::contains(a, c).then(|| ((c.y - a.min.y) * geom::width(a) + (c.x - a.min.x)) as usize)
    }

    /// Whether `c` is a pad cell.
    pub fn in_pad(&self, c: GridCoord) -> bool {
        self.local(c).is_some()
    }

    /// Sweep rank of pad cell `c`.
    pub fn rank_of(&self, c: GridCoord) -> Option<u32> {
        self.local(c).map(|i| self.rank[i])
    }

    /// Section of pad cell `c`.
    pub fn section_of(&self, c: GridCoord) -> Option<usize> {
        self.local(c).map(|i| self.section_of[i] as usize)
    }

    /// Pad cells in sweep order.
    pub fn order(&self) -> &[GridCoord] {
        &self.order
    }

    /// Index of berm cell `c`.
    pub fn berm_index(&self, c: GridCoord) -> Option<usize> {
        self.berm.iter().position(|b| b.at == c)
    }

    fn required_surface(&self) -> Surface {
        if self.pad.stabilize {
            Surface::Stabilized
        } else {
            Surface::Compacted
        }
    }

    /// What pad cell `c` needs next.
    pub fn cell_status(&self, world: &WorldModel, c: GridCoord, cfg: &Config) -> CellStatus {
        let mut st = CellStatus::default();
        let Some(k) = world.site.get(c) else {
            return st;
        };
        let elev = match k.elevation() {
            Some(e) if k.surveyed() => e,
            _ => {
                st.survey = true;
                return st;
            }
        };
        let target = self.pad.target_elevation_m;
        let delta = elev - target;
        let site = &world.site;
        // Where grading was refused, assume a much narrower window.
        let strict = self.strict.contains(&(c.x, c.y));
        let (fill_trigger, cut_trigger, fill_aim, safe) = if strict {
            (0.04, 0.04, 0.0, 0.04)
        } else {
            (
                cfg.fill_trigger_m,
                cfg.cut_trigger_m,
                cfg.fill_aim_m,
                cfg.grade_window_m - 0.03,
            )
        };
        let low = delta < -fill_trigger;
        let fill_kg = site.mass_for_height(c, (target - fill_aim - elev).max(0.0), cfg);
        st.max_fill_kg =
            site.mass_for_height(c, (target + cfg.overfill_limit_m - elev).max(0.0), cfg);
        let boulders = k.boulder_volume_m3.unwrap_or(0.0) > 1e-9;
        let in_tol = delta.abs() <= self.pad.surface_tolerance_m + cfg.rework_margin_m;
        let rework = self.rework.contains(&(c.x, c.y));
        match k.surface {
            Surface::Natural | Surface::Stockpile | Surface::Berm => {
                st.dig = true;
                if low {
                    st.fill_kg = fill_kg;
                }
            }
            Surface::Cleared => {
                st.dig = boulders || delta > cut_trigger;
                if low {
                    st.fill_kg = fill_kg;
                }
                st.grade = !st.dig && !low;
            }
            Surface::Graded | Surface::Compacted | Surface::Stabilized => {
                let wrong_finish = k.surface == Surface::Stabilized && !self.pad.stabilize;
                if !in_tol || rework || boulders || wrong_finish {
                    if boulders || delta > safe {
                        st.dig = true;
                    } else if delta < -safe {
                        st.fill_kg = fill_kg;
                    } else {
                        st.grade = true;
                    }
                } else if k.surface == self.required_surface() {
                    st.finished = true;
                } else if k.surface == Surface::Graded {
                    st.compact = true;
                } else {
                    st.sinter = true;
                }
            }
        }
        st.verified = st.finished && k.verified();
        st
    }

    /// Material still wanted on berm cell `i`, kg.
    pub fn berm_need_kg(&self, world: &WorldModel, i: usize, cfg: &Config) -> f64 {
        let b = &self.berm[i];
        let (Some(base), Some(spec)) = (b.base_m, self.pad.berm.as_ref()) else {
            return 0.0;
        };
        if b.confirmed {
            return 0.0;
        }
        let elev = world
            .site
            .get(b.at)
            .and_then(|k| k.elevation())
            .unwrap_or(base);
        let dh = (base + spec.height_m + cfg.berm_margin_m - elev).max(0.0);
        world.site.mass_for_height(b.at, dh, cfg) * cfg.berm_density_factor + b.extra_kg
    }

    /// Every task that is ready now and not already in flight.
    pub fn tasks(&self, world: &WorldModel, busy: &Busy, cfg: &Config) -> Vec<Task> {
        let mut out = Vec::new();
        let target = self.pad.target_elevation_m;
        let cell_area = world.site.cell_area_m2();
        let mut survey: Vec<Vec<GridCoord>> = vec![Vec::new(); self.sections.len()];
        let mut to_verify: Vec<(u32, GridCoord)> = Vec::new();
        let mut unfinished = 0usize;
        for (rank, &c) in self.order.iter().enumerate() {
            let st = self.cell_status(world, c, cfg);
            if !st.finished {
                unfinished += 1;
            }
            let section = self.section_of(c).unwrap_or(0);
            if st.survey {
                survey[section].push(c);
                continue;
            }
            if busy.cell(c) {
                continue;
            }
            let task = |stage: Stage, amount: f64, max_kg: f64| Task {
                key: TaskKey::Cell(stage, c.x, c.y),
                stage,
                area: geom::unit(c),
                seq: rank as f64,
                amount,
                max_kg,
                elevation_m: target,
                section: Some(section),
                cast_to: None,
            };
            if st.dig {
                out.push(task(Stage::Dig, world.site.dig_units_m3(c, target), 0.0));
            }
            if st.fill_kg >= cfg.min_haul_kg {
                out.push(task(Stage::Fill, st.fill_kg, st.max_fill_kg));
            }
            if st.grade {
                out.push(task(Stage::Grade, cell_area, 0.0));
            }
            if st.compact {
                out.push(task(Stage::Compact, cell_area, 0.0));
            }
            if st.sinter {
                out.push(task(Stage::Sinter, cell_area, 0.0));
            }
            if st.finished && !st.verified {
                to_verify.push((rank as u32, c));
            }
        }
        for (i, cells) in survey.iter().enumerate() {
            let key = TaskKey::Survey(i as u32);
            if busy.keys.contains(&key) {
                continue;
            }
            if let Some(area) = geom::bounding_rect(cells.iter().copied()) {
                out.push(scan_task(
                    key,
                    Stage::Survey,
                    area,
                    -1.0e6 + i as f64,
                    cell_area,
                ));
            }
        }
        let endgame = unfinished <= cfg.endgame_cells;
        if !to_verify.is_empty()
            && !busy.keys.contains(&TaskKey::Verify)
            && (to_verify.len() >= cfg.verify_batch_cells || endgame)
            && let Some(area) = geom::bounding_rect(to_verify.iter().map(|v| v.1))
        {
            let seq = to_verify.iter().map(|v| v.0).min().unwrap_or(0);
            out.push(scan_task(
                TaskKey::Verify,
                Stage::Verify,
                area,
                f64::from(seq),
                cell_area,
            ));
        }
        self.berm_tasks(world, busy, cfg, unfinished, &mut out);
        if self.recheck.wanted
            && !busy.keys.contains(&TaskKey::Recheck)
            && let Some(area) = geom::bounding_rect(
                geom::cells(&self.pad.area).chain(self.berm.iter().map(|b| b.at)),
            )
        {
            out.push(scan_task(
                TaskKey::Recheck,
                Stage::Recheck,
                area,
                -2.0e6,
                cell_area,
            ));
        }
        out
    }

    fn berm_tasks(
        &self,
        world: &WorldModel,
        busy: &Busy,
        cfg: &Config,
        unfinished: usize,
        out: &mut Vec<Task>,
    ) {
        let cell_area = world.site.cell_area_m2();
        let n = self.order.len();
        let unsurveyed: Vec<GridCoord> = self
            .berm
            .iter()
            .filter(|b| b.base_m.is_none())
            .map(|b| b.at)
            .collect();
        if !busy.keys.contains(&TaskKey::BermSurvey)
            && let Some(area) = geom::bounding_rect(unsurveyed)
        {
            out.push(scan_task(
                TaskKey::BermSurvey,
                Stage::Survey,
                area,
                -5.0e5,
                cell_area,
            ));
        }
        // Berm work normally follows the pad's fill; near the end it moves up.
        let late = unfinished * 5 <= n;
        let mut to_check = Vec::new();
        for (i, b) in self.berm.iter().enumerate() {
            if b.base_m.is_none() || busy.cell(b.at) {
                continue;
            }
            let need = self.berm_need_kg(world, i, cfg);
            if need >= cfg.min_haul_kg {
                let seq = if late {
                    (n - unfinished) as f64 + i as f64
                } else {
                    (n + i) as f64
                };
                match self.borrow_dig(world, i, need, seq, busy, cfg) {
                    BorrowDig::Ready(task) => {
                        out.push(task);
                        continue;
                    }
                    BorrowDig::Busy => continue,
                    BorrowDig::Unavailable => {}
                }
                out.push(Task {
                    key: TaskKey::Cell(Stage::Berm, b.at.x, b.at.y),
                    stage: Stage::Berm,
                    area: geom::unit(b.at),
                    seq,
                    amount: need,
                    max_kg: f64::INFINITY,
                    elevation_m: 0.0,
                    section: None,
                    cast_to: None,
                });
            } else if !b.confirmed && b.deposited_kg > 0.0 {
                to_check.push(b.at);
            }
        }
        if !busy.keys.contains(&TaskKey::BermCheck)
            && let Some(area) = geom::bounding_rect(to_check)
        {
            out.push(scan_task(
                TaskKey::BermCheck,
                Stage::BermCheck,
                area,
                (n + 100) as f64,
                cell_area,
            ));
        }
    }

    /// Progress as the world model sees it.
    /// The next dig from berm cell `i`'s borrow cell: deep enough that the
    /// overflow covers `need_kg` even with an empty payload, never below the
    /// borrow cell's floor.
    fn borrow_dig(
        &self,
        world: &WorldModel,
        i: usize,
        need_kg: f64,
        seq: f64,
        busy: &Busy,
        cfg: &Config,
    ) -> BorrowDig {
        let Some(b) = self.borrow.get(i).copied().flatten() else {
            return BorrowDig::Unavailable;
        };
        if busy.cell(b.at) {
            return BorrowDig::Busy;
        }
        let site = &world.site;
        let (Some(elev), Some(floor)) = (site.get(b.at).and_then(|k| k.elevation()), b.floor_m)
        else {
            return BorrowDig::Unavailable;
        };
        let depth = (need_kg + self.dig_room_kg) / (site.density(b.at, cfg) * site.cell_area_m2());
        let target = (elev - depth).max(floor);
        if elev - target < 0.05 {
            return BorrowDig::Unavailable;
        }
        BorrowDig::Ready(Task {
            key: TaskKey::Cell(Stage::Dig, b.at.x, b.at.y),
            stage: Stage::Dig,
            area: geom::unit(b.at),
            seq,
            amount: site.dig_units_m3(b.at, target),
            max_kg: 0.0,
            elevation_m: target,
            section: None,
            cast_to: Some(self.berm[i].at),
        })
    }

    /// Whether berm cell `i` is fed from its borrow cell (which still has material).
    fn fed_by_borrow(&self, world: &WorldModel, i: usize) -> bool {
        self.borrow.get(i).copied().flatten().is_some_and(|b| {
            let elev = world.site.get(b.at).and_then(|k| k.elevation());
            matches!((elev, b.floor_m), (Some(e), Some(f)) if e - f >= 0.05)
        })
    }

    pub fn progress(&self, world: &WorldModel, cfg: &Config) -> Progress {
        let mut p = Progress {
            pad_cells: self.order.len(),
            berm_cells: self.berm.len(),
            berm_confirmed: self.berm.iter().filter(|b| b.confirmed).count(),
            ..Progress::default()
        };
        for &c in &self.order {
            let st = self.cell_status(world, c, cfg);
            p.finished += usize::from(st.finished);
            p.verified += usize::from(st.verified);
        }
        p
    }

    /// Remaining work per capability, for the capacity plan.
    pub fn workload(&self, world: &WorldModel, reg: &Registry, cfg: &Config) -> Workload {
        let cs = world.site.cell_size_m;
        let cell_area = world.site.cell_area_m2();
        let haulers = reg.capable(CapabilityName::Haul);
        let load_kg = if haulers.is_empty() {
            400.0
        } else {
            haulers
                .iter()
                .map(|&r| reg.haul_capacity_kg(r))
                .sum::<f64>()
                / haulers.len() as f64
        };
        let span = f64::from(geom::width(&self.pad.area).max(geom::height(&self.pad.area)));
        let haul_trip_m = (span / 2.0 + 3.0) * cs;
        let target = self.pad.target_elevation_m;
        let mut w = Workload::new();
        let mut add = |cap: CapabilityName, units: f64, steps: f64, travel: f64, passes: f64| {
            let e = w.entry(cap).or_insert(CapWork {
                travel_m_per_step: travel,
                passes,
                ..CapWork::default()
            });
            e.units += units;
            e.steps += steps;
        };
        let scans_per_cell = 1.0 / f64::from(cfg.section_cells.max(1));
        for &c in &self.order {
            let st = self.cell_status(world, c, cfg);
            if st.finished {
                if !st.verified {
                    add(CapabilityName::Scan, cell_area, scans_per_cell, cs, 1.0);
                }
                continue;
            }
            let level = world.site.get(c).map_or(0, |k| surface_level(k.surface));
            let surveying = st.survey;
            if surveying {
                add(CapabilityName::Scan, cell_area, scans_per_cell, cs, 1.0);
            }
            if surveying || st.dig || level < 1 {
                add(
                    CapabilityName::Dig,
                    world.site.dig_units_m3(c, target),
                    1.0,
                    cs,
                    1.0,
                );
            }
            if st.fill_kg > 0.0 {
                add(
                    CapabilityName::Haul,
                    st.fill_kg,
                    (st.fill_kg / load_kg).ceil(),
                    2.0 * haul_trip_m,
                    2.0,
                );
            }
            if surveying || st.grade || level < 2 {
                add(CapabilityName::Grade, cell_area, 1.0, cs, 1.0);
            }
            if surveying || st.compact || level < 3 {
                add(CapabilityName::Compact, cell_area, 1.0, cs, 1.0);
            }
            if self.pad.stabilize && (surveying || st.sinter || level < 4) {
                add(CapabilityName::Sinter, cell_area, 1.0, cs, 1.0);
            }
            add(CapabilityName::Scan, cell_area, scans_per_cell, cs, 1.0);
        }
        let spec_height = self.pad.berm.as_ref().map_or(0.0, |b| b.height_m);
        for (i, b) in self.berm.iter().enumerate() {
            let need = if b.base_m.is_some() {
                self.berm_need_kg(world, i, cfg)
            } else {
                world
                    .site
                    .mass_for_height(b.at, spec_height + cfg.berm_margin_m, cfg)
            };
            if need > 0.0 && self.fed_by_borrow(world, i) {
                let m3 = need / (world.site.density(b.at, cfg) * cell_area);
                add(CapabilityName::Dig, m3, 1.0, cs, 1.0);
            } else if need > 0.0 {
                add(
                    CapabilityName::Haul,
                    need,
                    (need / load_kg).ceil(),
                    2.0 * haul_trip_m,
                    2.0,
                );
            }
        }
        w
    }

    /// Where a dig of `dug` should send `overflow_kg` of spoil.
    pub fn spoil_target(
        &self,
        world: &WorldModel,
        dug: GridCoord,
        overflow_kg: f64,
        busy: &Busy,
        cfg: &Config,
    ) -> Option<SpoilTarget> {
        let cs = world.site.cell_size_m;
        // With haulers, cast only nearby and only where the amount fits the
        // need; without them, casting is the only way to move material.
        let (reach, fill_fraction, berm_fraction) = if self.haulers {
            (cfg.side_cast_reach_m / cs, 0.6, 0.5)
        } else {
            (f64::from(world.site.width.max(world.site.height)), 0.0, 0.0)
        };
        if overflow_kg > cfg.min_haul_kg {
            let berm = self
                .berm
                .iter()
                .enumerate()
                .filter(|(_, b)| geom::dist(dug, b.at) <= reach && !busy.cell(b.at))
                .filter(|(i, _)| {
                    let need = self.berm_need_kg(world, *i, cfg);
                    need >= cfg.min_haul_kg && need >= berm_fraction * overflow_kg
                })
                .min_by(|a, b| {
                    geom::dist(dug, a.1.at)
                        .total_cmp(&geom::dist(dug, b.1.at))
                        .then(a.0.cmp(&b.0))
                });
            if let Some((i, b)) = berm {
                return Some(SpoilTarget {
                    at: b.at,
                    kind: SpoilKind::Berm(i),
                });
            }
            let r = reach.floor() as u32;
            let a = &self.pad.area;
            let window = GridRect {
                min: GridCoord {
                    x: dug.x.saturating_sub(r).max(a.min.x),
                    y: dug.y.saturating_sub(r).max(a.min.y),
                },
                max: GridCoord {
                    x: (dug.x + r).min(a.max.x),
                    y: (dug.y + r).min(a.max.y),
                },
            };
            let fill = geom::cells(&window)
                .filter(|&c| c != dug && geom::dist(dug, c) <= reach && !busy.cell(c))
                .filter(|&c| {
                    let open = world.site.get(c).is_some_and(|k| {
                        matches!(
                            k.surface,
                            Surface::Natural | Surface::Stockpile | Surface::Cleared
                        )
                    });
                    let st = self.cell_status(world, c, cfg);
                    open && st.fill_kg >= cfg.min_haul_kg
                        && st.fill_kg >= fill_fraction * overflow_kg
                        && overflow_kg <= st.max_fill_kg
                })
                .min_by(|&x, &y| {
                    geom::dist(dug, x)
                        .total_cmp(&geom::dist(dug, y))
                        .then((x.y, x.x).cmp(&(y.y, y.x)))
                });
            if let Some(at) = fill {
                return Some(SpoilTarget {
                    at,
                    kind: SpoilKind::Fill,
                });
            }
        }
        let wanted: Vec<GridCoord> = self.berm.iter().map(|b| b.at).collect();
        self.material
            .spoil_site(dug, overflow_kg.max(0.0), &wanted)
            .map(|i| SpoilTarget {
                at: self.material.piles[i].at,
                kind: SpoilKind::Pile(i),
            })
    }

    /// Where idle crew should wait: the first unfinished pad cell in sweep order.
    pub fn front(&self, world: &WorldModel, cfg: &Config) -> Option<GridCoord> {
        self.order
            .iter()
            .copied()
            .find(|&c| !self.cell_status(world, c, cfg).finished)
    }

    /// Update berm bookkeeping after scan results were applied to the site.
    pub fn after_scan(&mut self, world: &WorldModel, cells: &[CellObservation], cfg: &Config) {
        // A borrow cell's floor follows from its first measured elevation.
        for b in self.borrow.iter_mut().flatten() {
            if b.floor_m.is_none()
                && cells.iter().any(|o| o.at == b.at)
                && let Some(e) = world.site.get(b.at).and_then(|k| k.elevation())
            {
                b.floor_m = Some(e - cfg.berm_borrow_max_depth_m);
            }
        }
        let Some(height) = self.pad.berm.as_ref().map(|b| b.height_m) else {
            return;
        };
        for obs in cells {
            let Some(i) = self.berm_index(obs.at) else {
                continue;
            };
            let Some(k) = world.site.get(obs.at) else {
                continue;
            };
            let Some(elev) = k.elevation() else {
                continue;
            };
            let b = &mut self.berm[i];
            match b.base_m {
                None => {
                    let raised = b.deposited_kg
                        / (world.site.density(b.at, cfg) * world.site.cell_area_m2());
                    b.base_m = Some(elev - raised);
                }
                Some(base) if b.deposited_kg > 0.0 => {
                    b.confirmed =
                        k.surface == Surface::Berm && elev >= base + height + BERM_CHECK_MARGIN_M;
                    if b.confirmed {
                        b.extra_kg = 0.0;
                    }
                }
                Some(_) => {}
            }
        }
    }

    /// Record material deposited on a berm cell. A deposit also uses up any
    /// extra material a failed check asked for, so the cell's need falls to
    /// zero and the next check scan follows.
    pub fn record_berm_deposit(&mut self, i: usize, kg: f64) {
        if let Some(b) = self.berm.get_mut(i) {
            b.deposited_kg += kg.max(0.0);
            b.extra_kg = (b.extra_kg - kg.max(0.0)).max(0.0);
            b.confirmed = false;
        }
    }

    /// Grading `c` was refused: insist on a much closer starting elevation there.
    pub fn tighten(&mut self, c: GridCoord) {
        self.strict.insert((c.x, c.y));
    }

    /// A rework grade of `c` has completed.
    pub fn clear_rework(&mut self, c: GridCoord) {
        self.rework.remove(&(c.x, c.y));
    }

    /// Cells currently marked for rework.
    pub fn rework_cells(&self) -> usize {
        self.rework.len()
    }

    /// Everything looks done but the run goes on: ask for a recheck scan.
    pub fn request_recheck(&mut self) {
        self.recheck.wanted = true;
    }

    /// Act on a completed recheck scan: rework the most suspicious pad cells
    /// and top up doubtful berm cells. Always schedules something, so the
    /// loop keeps making progress.
    pub fn after_recheck(&mut self, world: &WorldModel, cfg: &Config) -> usize {
        self.recheck.wanted = false;
        self.recheck.rounds += 1;
        let round = f64::from(self.recheck.rounds.min(4));
        let target = self.pad.target_elevation_m;
        let tol = self.pad.surface_tolerance_m;
        let max_rise = self.pad.max_slope_deg.to_radians().tan() * world.site.cell_size_m;
        let elev = |c: GridCoord| {
            world
                .site
                .get(c)
                .and_then(|k| k.elevation())
                .unwrap_or(target)
        };
        let mut picks: Vec<(f64, GridCoord)> = self
            .order
            .iter()
            .map(|&c| ((elev(c) - target).abs(), c))
            .filter(|&(d, _)| d > tol - 0.01 * round)
            .collect();
        // The steepest step between neighbours, judged against the limit.
        let mut steep: Option<(f64, GridCoord)> = None;
        for &c in &self.order {
            for n in [
                GridCoord { x: c.x + 1, y: c.y },
                GridCoord { x: c.x, y: c.y + 1 },
            ] {
                if !self.in_pad(n) {
                    continue;
                }
                let step = (elev(n) - elev(c)).abs();
                let worse = if (elev(n) - target).abs() > (elev(c) - target).abs() {
                    n
                } else {
                    c
                };
                if steep.is_none_or(|(s, _)| step > s) {
                    steep = Some((step, worse));
                }
            }
        }
        if let Some((step, c)) = steep
            && (picks.is_empty() || step > max_rise * (1.0 - 0.1 * round))
        {
            picks.push((step, c));
        }
        picks.sort_by(|a, b| {
            b.0.total_cmp(&a.0)
                .then((a.1.y, a.1.x).cmp(&(b.1.y, b.1.x)))
        });
        picks.dedup_by_key(|p| p.1);
        let mut actions = 0;
        for (_, c) in picks.into_iter().take(4) {
            self.rework.insert((c.x, c.y));
            actions += 1;
        }
        // Berm cells whose measured height is doubtful get another load's worth.
        let height = self.pad.berm.as_ref().map_or(0.0, |b| b.height_m);
        let mut weakest: Option<(f64, usize)> = None;
        for (i, b) in self.berm.iter().enumerate() {
            let Some(base) = b.base_m else { continue };
            let margin = elev(b.at) - (base + height);
            if weakest.is_none_or(|(m, _)| margin < m) {
                weakest = Some((margin, i));
            }
        }
        let topups: Vec<usize> = self
            .berm
            .iter()
            .enumerate()
            .filter(|(_, b)| {
                b.base_m
                    .is_some_and(|base| elev(b.at) < base + height + 0.03)
            })
            .map(|(i, _)| i)
            .chain(weakest.filter(|_| actions == 0).map(|w| w.1))
            .collect();
        for i in topups {
            let at = self.berm[i].at;
            self.berm[i].confirmed = false;
            self.berm[i].extra_kg += world.site.mass_for_height(at, 0.05, cfg);
            actions += 1;
        }
        actions
    }
}

/// A borrow cell for each berm cell outside the base station's coverage: the
/// nearest cell within side-cast reach that is covered and is not pad, berm,
/// infrastructure or another berm cell's borrow cell.
fn borrow_cells(
    pad: &GridRect,
    berm: &[BermCell],
    world: &WorldModel,
    cfg: &Config,
) -> Vec<Option<Borrow>> {
    let site = &world.site;
    let cov = Coverage::new(&world.base_station, site.cell_size_m, &[]);
    let reach = cfg.side_cast_reach_m / site.cell_size_m;
    let mut taken: BTreeSet<(u32, u32)> = berm.iter().map(|b| (b.at.x, b.at.y)).collect();
    for p in world
        .stations
        .iter()
        .map(|s| s.position)
        .chain(world.depots.iter().map(|d| d.position))
    {
        taken.insert((p.x, p.y));
    }
    let r = reach.floor() as i64;
    berm.iter()
        .map(|b| {
            if cov.base_covers(b.at) {
                return None;
            }
            let mut best: Option<(f64, u32, u32)> = None;
            for dy in -r..=r {
                for dx in -r..=r {
                    let (x, y) = (i64::from(b.at.x) + dx, i64::from(b.at.y) + dy);
                    if x < 0 || y < 0 {
                        continue;
                    }
                    let c = GridCoord {
                        x: x as u32,
                        y: y as u32,
                    };
                    let d = geom::dist(b.at, c);
                    if !site.in_bounds(c)
                        || d > reach
                        || geom::contains(pad, c)
                        || taken.contains(&(c.x, c.y))
                        || !cov.base_covers(c)
                    {
                        continue;
                    }
                    if best.is_none_or(|k| (d, c.y, c.x) < k) {
                        best = Some((d, c.y, c.x));
                    }
                }
            }
            let (_, y, x) = best?;
            let at = GridCoord { x, y };
            taken.insert((x, y));
            Some(Borrow {
                at,
                floor_m: site
                    .get(at)
                    .and_then(|k| k.elevation())
                    .map(|e| e - cfg.berm_borrow_max_depth_m),
            })
        })
        .collect()
}

fn scan_task(key: TaskKey, stage: Stage, area: GridRect, seq: f64, cell_area: f64) -> Task {
    Task {
        key,
        stage,
        area,
        seq,
        amount: geom::cell_count(&area) as f64 * cell_area,
        max_kg: 0.0,
        elevation_m: 0.0,
        section: None,
        cast_to: None,
    }
}

#[cfg(test)]
mod tests;
