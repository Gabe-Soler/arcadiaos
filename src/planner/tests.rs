//! Planner tests on small synthetic sites.

use super::*;
use crate::registry::tests::{cap, robot, scenario};
use arcadia_protocol::*;

const TARGET: f64 = -10.0;

fn pad_spec(berm: Vec<GridCoord>) -> LandingPadSpec {
    LandingPadSpec {
        area: GridRect {
            min: GridCoord { x: 2, y: 2 },
            max: GridCoord { x: 5, y: 5 },
        },
        target_elevation_m: TARGET,
        surface_tolerance_m: 0.05,
        max_slope_deg: 2.0,
        stabilize: true,
        berm: (!berm.is_empty()).then_some(BermSpec {
            cells: berm,
            height_m: 0.5,
        }),
    }
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
        tank_wh: None,
    }
}

/// A 10 × 10 site of 1 m cells with one hauler parked at `(x, y)`.
fn world_with_crew_at(x: u32, y: u32) -> (WorldModel, Registry) {
    let mut sc = scenario(vec![robot(
        "h",
        0.5,
        vec![
            cap(CapabilityName::Haul, 3.0, 0.01, Some(400.0)),
            cap(CapabilityName::Scan, 4.0, 0.05, None),
        ],
    )]);
    sc.site.cell_size_m = 1.0;
    let mut w = WorldModel::new(&sc);
    w.fleet.update(&[state("h", x, y)]);
    (w, Registry::new(&sc))
}

/// The same site with a crew whose sinterer takes `sinter_s` over a 1 m² cell.
fn world_with_sinterer(sinter_s: f64) -> (WorldModel, Registry) {
    let mut sc = scenario(vec![
        robot(
            "h",
            0.5,
            vec![
                cap(CapabilityName::Haul, 3.0, 0.01, Some(400.0)),
                cap(CapabilityName::Scan, 4.0, 0.05, None),
            ],
        ),
        robot(
            "s",
            0.5,
            vec![cap(CapabilityName::Sinter, 1.0 / sinter_s, 60.0, None)],
        ),
    ]);
    sc.site.cell_size_m = 1.0;
    let mut w = WorldModel::new(&sc);
    w.fleet.update(&[state("h", 0, 0), state("s", 0, 1)]);
    (w, Registry::new(&sc))
}

/// A crew for which sintering a cell takes hours, as on ArcadiaBench's
/// catalog 1.1: redoing one is dear, so cells are measured first.
fn careful_world() -> (WorldModel, Registry) {
    world_with_sinterer(25_000.0)
}

fn observe(c: GridCoord, elev: f64, surface: Surface, boulder: bool) -> CellObservation {
    CellObservation {
        at: c,
        cell: Cell {
            elevation_m: Some(elev),
            regolith: Some(Regolith {
                depth_m: Some(1.0),
                bulk_density_kg_m3: Some(1500.0),
                cohesion_kpa: None,
            }),
            boulders: Some(if boulder {
                vec![Boulder {
                    diameter_m: 0.3,
                    mass_kg: None,
                }]
            } else {
                vec![]
            }),
            surface: Some(surface),
            ice_depth_m: None,
        },
    }
}

fn survey_all(w: &mut WorldModel, p: &Planner, elev: f64) {
    let cells: Vec<CellObservation> = p
        .order()
        .iter()
        .map(|&c| observe(c, elev, Surface::Natural, false))
        .collect();
    w.site.apply_scan(&cells, 0.0, 1.0);
}

#[test]
fn sweep_starts_near_the_crew_and_sections_cover_the_pad() {
    let cfg = Config {
        section_cells: 8,
        ..Config::default()
    };
    let (w, reg) = world_with_crew_at(9, 9);
    let p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    assert_eq!(
        p.order()[0],
        GridCoord { x: 5, y: 5 },
        "nearest corner to (9, 9)"
    );
    assert_eq!(p.order().len(), 16);
    assert_eq!(p.sections.len(), 2);
    assert_eq!(p.rank_of(GridCoord { x: 5, y: 5 }), Some(0));
    assert_eq!(p.section_of(GridCoord { x: 2, y: 2 }), Some(1));
    assert!(p.rank_of(GridCoord { x: 0, y: 0 }).is_none());
    assert!(!p.material.piles.is_empty());
}

#[test]
fn cells_move_through_the_pipeline() {
    let cfg = Config::default();
    let (mut w, reg) = careful_world();
    let p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    let c = GridCoord { x: 3, y: 3 };
    assert!(p.cell_status(&w, c, &cfg).survey);
    // Earthworks aim at the grade level: the target plus what compaction
    // will take off again.
    let aim = p.grade_level(&cfg);
    assert_eq!(aim, TARGET + cfg.compact_settle_m);

    // High natural cell with a boulder: dig only.
    w.site.apply_scan(
        &[observe(c, TARGET + 0.2, Surface::Natural, true)],
        0.0,
        1.0,
    );
    let st = p.cell_status(&w, c, &cfg);
    assert!(st.dig && st.fill_kg == 0.0 && !st.grade);

    // Low natural cell: dig and fill, both independent.
    w.site.apply_scan(
        &[observe(c, TARGET - 0.3, Surface::Natural, false)],
        0.0,
        2.0,
    );
    let st = p.cell_status(&w, c, &cfg);
    assert!(st.dig);
    let wanted = (aim - cfg.fill_aim_m - (TARGET - 0.3)) * 1500.0;
    assert!((st.fill_kg - wanted).abs() < 1e-6, "{}", st.fill_kg);
    assert!(st.max_fill_kg > st.fill_kg);

    // Cleared and inside the window: grade.
    w.site.record_dig(c, aim, 3.0, &cfg);
    w.site.record_deposit(c, st.fill_kg, false, 4.0, &cfg);
    let st = p.cell_status(&w, c, &cfg);
    assert!(st.grade && !st.dig && st.fill_kg == 0.0, "{st:?}");

    w.site.record_grade(c, aim, 5.0);
    assert!(p.cell_status(&w, c, &cfg).compact);
    // Compacted, but not measured since: scan it before sintering.
    w.site.record_compact(c, cfg.compact_settle_m, 6.0);
    let st = p.cell_status(&w, c, &cfg);
    assert!(st.measure && !st.sinter && !st.finished, "{st:?}");
    w.site.apply_scan(
        &[observe(c, TARGET + 0.004, Surface::Compacted, false)],
        6.5,
        6.6,
    );
    let st = p.cell_status(&w, c, &cfg);
    assert!(st.sinter && !st.measure, "{st:?}");
    w.site.record_sinter(c, 7.0);
    let st = p.cell_status(&w, c, &cfg);
    assert!(st.finished && !st.verified);
    w.site.apply_scan(
        &[observe(c, TARGET - 0.004, Surface::Stabilized, false)],
        7.0,
        8.0,
    );
    assert!(p.cell_status(&w, c, &cfg).verified);
    // Sintering did not move the surface: both scans count towards its height.
    let k = w.site.get(c).unwrap();
    assert_eq!(k.samples, 2);
    assert!((k.elevation_m.unwrap() - TARGET).abs() < 1e-9);
}

/// A row of compacted pad cells, each scanned `scans` times at its height
/// above the target.
fn compacted(w: &mut WorldModel, cells: &[(GridCoord, f64)], scans: u32) {
    for &(c, _) in cells {
        w.site
            .apply_scan(&[observe(c, TARGET, Surface::Compacted, false)], 0.0, 1.0);
        w.site.record_compact(c, 0.0, 2.0);
    }
    for i in 0..scans {
        let obs: Vec<CellObservation> = cells
            .iter()
            .map(|&(c, above)| observe(c, TARGET + above, Surface::Compacted, false))
            .collect();
        w.site
            .apply_scan(&obs, 3.0 + f64::from(i), 3.5 + f64::from(i));
    }
}

#[test]
fn a_compacted_cell_is_measured_until_its_fit_is_clear() {
    let cfg = Config::default();
    let (mut w, reg) = careful_world();
    let p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    let c = GridCoord { x: 3, y: 3 };
    let status = |w: &WorldModel, c| p.cell_status(w, c, &cfg);
    // One scan with 1 cm of noise cannot place a cell 3 cm off inside a
    // 4.5 cm limit; a few more can.
    compacted(&mut w, &[(c, 0.03)], 1);
    assert!(status(&w, c).measure);
    compacted(&mut w, &[(c, 0.03)], 4);
    assert_eq!(w.site.get(c).unwrap().samples, 4);
    assert!(status(&w, c).sinter);
    // A cell clearly outside is regraded at once, while that is cheap.
    compacted(&mut w, &[(c, 0.08)], 1);
    let st = status(&w, c);
    assert!(st.grade && !st.sinter && !st.measure, "{st:?}");
    // One that stays too close to call is taken at its mean in the end.
    compacted(&mut w, &[(c, 0.044)], cfg.measure_max_samples);
    assert!(status(&w, c).sinter);
    compacted(&mut w, &[(c, 0.046)], cfg.measure_max_samples);
    assert!(status(&w, c).grade);
}

#[test]
fn a_crew_that_sinters_quickly_does_not_wait_for_measurements() {
    // A cell sintered in 100 s costs less to redo than to measure first:
    // it is sintered as soon as it is compacted, unless it is clearly wrong.
    let cfg = Config::default();
    let (mut w, reg) = world_with_sinterer(100.0);
    let p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    let (a, b) = (GridCoord { x: 3, y: 3 }, GridCoord { x: 4, y: 3 });
    let status = |w: &WorldModel, c| p.cell_status(w, c, &cfg);
    w.site
        .apply_scan(&[observe(a, TARGET, Surface::Compacted, false)], 0.0, 1.0);
    w.site.record_compact(a, 0.0, 2.0);
    let st = status(&w, a);
    assert!(st.sinter && !st.measure, "unmeasured: {st:?}");
    compacted(&mut w, &[(a, 0.03)], 1);
    assert!(status(&w, a).sinter, "too close to call");
    compacted(&mut w, &[(a, -0.016), (b, 0.016)], 2);
    assert!(status(&w, a).sinter && status(&w, b).sinter);
    // Clearly outside the tolerance, or a step clearly too high: redone.
    compacted(&mut w, &[(a, 0.08)], 1);
    assert!(status(&w, a).grade);
    compacted(&mut w, &[(a, -0.02), (b, 0.03)], 9);
    assert!(status(&w, b).grade && status(&w, a).sinter);
    // The crew of hours-long sinters would have measured that first cell.
    let (mut w, reg) = careful_world();
    let p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    w.site
        .apply_scan(&[observe(a, TARGET, Surface::Compacted, false)], 0.0, 1.0);
    w.site.record_compact(a, 0.0, 2.0);
    let st = p.cell_status(&w, a, &cfg);
    assert!(st.measure && !st.sinter, "{st:?}");
}

#[test]
fn a_step_between_neighbours_is_settled_before_sintering() {
    let cfg = Config::default();
    let (mut w, reg) = careful_world();
    let p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    let (a, b) = (GridCoord { x: 3, y: 3 }, GridCoord { x: 4, y: 3 });
    let status = |w: &WorldModel, c| p.cell_status(w, c, &cfg);
    // Two degrees over a 1 m cell is a step of 3.5 cm. Both cells are well
    // inside the tolerance, but 5 cm apart: the one further from the target
    // is redone, and the other waits for it.
    compacted(&mut w, &[(a, -0.02), (b, 0.03)], 9);
    let (sa, sb) = (status(&w, a), status(&w, b));
    assert!(sb.grade && !sb.sinter, "{sb:?}");
    assert!(!sa.sinter && !sa.grade && !sa.measure, "{sa:?}");
    // Once it is regraded the pair is no longer judged, and the first cell
    // can be sintered.
    w.site.record_grade(b, p.grade_level(&cfg), 20.0);
    assert!(status(&w, a).sinter);
    assert!(status(&w, b).compact);
    // A step too close to the limit to call is scanned again...
    compacted(&mut w, &[(a, -0.016), (b, 0.016)], 2);
    assert!(status(&w, a).measure && status(&w, b).measure);
    // ...and a comfortable one passes.
    compacted(&mut w, &[(a, -0.01), (b, 0.01)], 4);
    assert!(status(&w, a).sinter && status(&w, b).sinter);
    // Beside a sintered neighbour it is always the unsintered cell that
    // gives way, even when it is the one nearer the target.
    compacted(&mut w, &[(a, -0.035), (b, 0.012)], 9);
    w.site.record_sinter(a, 40.0);
    let (sa, sb) = (status(&w, a), status(&w, b));
    assert!(sa.finished && !sa.grade, "{sa:?}");
    assert!(sb.grade, "{sb:?}");
    // A neighbour that is compacted but not yet measured holds a cell back.
    let c = GridCoord { x: 3, y: 4 };
    w.site
        .apply_scan(&[observe(c, TARGET, Surface::Compacted, false)], 0.0, 1.0);
    w.site.record_compact(c, 0.0, 50.0);
    let d = GridCoord { x: 3, y: 5 };
    compacted(&mut w, &[(d, 0.0)], 4);
    assert!(status(&w, c).measure);
    let sd = status(&w, d);
    assert!(sd.measure && !sd.sinter, "{sd:?}");
}

#[test]
fn defective_finished_cells_are_reworked() {
    let cfg = Config::default();
    let (mut w, reg) = world_with_crew_at(0, 0);
    let p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    let c = GridCoord { x: 4, y: 4 };
    // Slightly out of tolerance: regrade.
    w.site.apply_scan(
        &[observe(c, TARGET + 0.08, Surface::Stabilized, false)],
        0.0,
        1.0,
    );
    let st = p.cell_status(&w, c, &cfg);
    assert!(st.grade && !st.finished);
    // Far too high: dig. Far too low: fill.
    w.site.apply_scan(
        &[observe(c, TARGET + 0.3, Surface::Compacted, false)],
        0.0,
        2.0,
    );
    assert!(p.cell_status(&w, c, &cfg).dig);
    w.site.apply_scan(
        &[observe(c, TARGET - 0.3, Surface::Graded, false)],
        0.0,
        3.0,
    );
    assert!(p.cell_status(&w, c, &cfg).fill_kg > 0.0);
}

#[test]
fn ready_tasks_follow_dependencies_and_skip_busy_cells() {
    let cfg = Config {
        section_cells: 8,
        verify_batch_cells: 4,
        endgame_cells: 0,
        ..Config::default()
    };
    let (mut w, reg) = world_with_crew_at(0, 0);
    let p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    // Nothing surveyed: one survey per section and nothing else.
    let tasks = p.tasks(&w, &Busy::default(), &cfg);
    assert_eq!(tasks.len(), 2);
    assert!(tasks.iter().all(|t| t.stage == Stage::Survey));

    survey_all(&mut w, &p, TARGET + 0.05);
    let tasks = p.tasks(&w, &Busy::default(), &cfg);
    assert_eq!(tasks.iter().filter(|t| t.stage == Stage::Dig).count(), 16);
    assert!(tasks.iter().all(|t| t.stage == Stage::Dig));

    let mut busy = Busy::default();
    busy.cells.insert((2, 2));
    let tasks = p.tasks(&w, &busy, &cfg);
    assert_eq!(tasks.len(), 15);

    // Finish four cells: one verification batch covering them.
    for (i, &c) in p.order().iter().take(4).enumerate() {
        let t = 10.0 + i as f64;
        w.site.record_dig(c, TARGET, t, &cfg);
        w.site.record_grade(c, TARGET, t);
        w.site.record_compact(c, 0.0, t);
        w.site.record_sinter(c, t);
    }
    let tasks = p.tasks(&w, &Busy::default(), &cfg);
    let verify: Vec<&Task> = tasks.iter().filter(|t| t.stage == Stage::Verify).collect();
    assert_eq!(verify.len(), 1);
    for &c in &p.order()[..4] {
        assert!(geom::contains(&verify[0].area, c));
    }
    assert_eq!(p.progress(&w, &cfg).finished, 4);
}

#[test]
fn berm_is_surveyed_built_and_confirmed() {
    let cfg = Config::default();
    let berm = vec![GridCoord { x: 3, y: 8 }, GridCoord { x: 4, y: 8 }];
    let (mut w, reg) = world_with_crew_at(0, 0);
    let mut p = Planner::new("g", &pad_spec(berm.clone()), &w, &reg, &cfg);
    let tasks = p.tasks(&w, &Busy::default(), &cfg);
    assert!(tasks.iter().any(|t| t.key == TaskKey::BermSurvey));

    let scan: Vec<CellObservation> = berm
        .iter()
        .map(|&c| observe(c, -9.5, Surface::Natural, false))
        .collect();
    w.site.apply_scan(&scan, 0.0, 1.0);
    p.after_scan(&w, &scan, &cfg);
    assert_eq!(p.berm[0].base_m, Some(-9.5));
    let need = p.berm_need_kg(&w, 0, &cfg);
    let expected = (0.5 + cfg.berm_margin_m) * 1500.0 * cfg.berm_density_factor;
    assert!((need - expected).abs() < 1e-6, "{need}");
    let tasks = p.tasks(&w, &Busy::default(), &cfg);
    assert_eq!(tasks.iter().filter(|t| t.stage == Stage::Berm).count(), 2);

    // Build the first cell, then a scan confirms it.
    w.site.record_deposit(berm[0], need, true, 2.0, &cfg);
    p.record_berm_deposit(0, need);
    assert!(p.berm_need_kg(&w, 0, &cfg) < cfg.min_haul_kg);
    let tasks = p.tasks(&w, &Busy::default(), &cfg);
    assert!(tasks.iter().any(|t| t.key == TaskKey::BermCheck));
    let check = [observe(berm[0], -9.5 + 0.55, Surface::Berm, false)];
    w.site.apply_scan(&check, 2.0, 3.0);
    p.after_scan(&w, &check, &cfg);
    assert!(p.berm[0].confirmed);
    assert_eq!(p.berm_need_kg(&w, 0, &cfg), 0.0);

    // A recheck asks for one more load; delivering it lets the check follow
    // instead of hauling to the cell for ever.
    p.berm[0].confirmed = false;
    p.berm[0].extra_kg = 75.0;
    assert!(p.berm_need_kg(&w, 0, &cfg) >= 75.0);
    w.site.record_deposit(berm[0], 75.0, true, 4.0, &cfg);
    p.record_berm_deposit(0, 75.0);
    assert_eq!(p.berm[0].extra_kg, 0.0);
    assert!(p.berm_need_kg(&w, 0, &cfg) < cfg.min_haul_kg);
    let tasks = p.tasks(&w, &Busy::default(), &cfg);
    assert!(tasks.iter().any(|t| t.key == TaskKey::BermCheck));
}

#[test]
fn spoil_goes_to_nearby_berm_then_fill_then_piles() {
    let cfg = Config::default();
    let berm = vec![GridCoord { x: 4, y: 7 }];
    let (mut w, reg) = world_with_crew_at(0, 0);
    let mut p = Planner::new("g", &pad_spec(berm.clone()), &w, &reg, &cfg);
    survey_all(&mut w, &p, TARGET);
    let scan = [observe(berm[0], -9.5, Surface::Natural, false)];
    w.site.apply_scan(&scan, 0.0, 1.0);
    p.after_scan(&w, &scan, &cfg);

    // A cell next to the berm casts onto it.
    let near_berm = GridCoord { x: 4, y: 5 };
    let s = p
        .spoil_target(&w, near_berm, 200.0, &Busy::default(), &cfg)
        .unwrap();
    assert_eq!(s.kind, SpoilKind::Berm(0));

    // A low neighbour takes spoil that fits.
    let low = GridCoord { x: 2, y: 3 };
    w.site.apply_scan(
        &[observe(low, TARGET - 0.25, Surface::Natural, false)],
        0.0,
        2.0,
    );
    let s = p
        .spoil_target(&w, GridCoord { x: 2, y: 2 }, 250.0, &Busy::default(), &cfg)
        .unwrap();
    assert_eq!((s.kind, s.at), (SpoilKind::Fill, low));
    // Too much for it, or the cell is busy: a pile instead.
    let s = p
        .spoil_target(&w, GridCoord { x: 2, y: 2 }, 900.0, &Busy::default(), &cfg)
        .unwrap();
    assert!(matches!(s.kind, SpoilKind::Pile(_)));
    let mut busy = Busy::default();
    busy.cells.insert((low.x, low.y));
    let s = p
        .spoil_target(&w, GridCoord { x: 2, y: 2 }, 250.0, &busy, &cfg)
        .unwrap();
    assert!(matches!(s.kind, SpoilKind::Pile(_)));
}

#[test]
fn recheck_always_finds_something_to_fix() {
    let cfg = Config::default();
    let (mut w, reg) = world_with_crew_at(0, 0);
    let mut p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    // Every cell finished and verified, one with a steep step.
    let mut cells = Vec::new();
    for &c in p.order() {
        let elev = if c == (GridCoord { x: 3, y: 3 }) {
            TARGET + 0.045
        } else {
            TARGET
        };
        cells.push(observe(c, elev, Surface::Stabilized, false));
    }
    w.site.apply_scan(&cells, 0.0, 1.0);
    assert!(p.progress(&w, &cfg).complete());
    p.request_recheck();
    assert!(
        p.tasks(&w, &Busy::default(), &cfg)
            .iter()
            .any(|t| t.key == TaskKey::Recheck)
    );
    let actions = p.after_recheck(&w, &cfg);
    assert!(actions >= 1);
    assert!(!p.recheck.wanted);
    assert_eq!(p.rework_cells(), 1);
    let st = p.cell_status(&w, GridCoord { x: 3, y: 3 }, &cfg);
    assert!(st.grade, "the suspicious cell is regraded: {st:?}");
    p.clear_rework(GridCoord { x: 3, y: 3 });
    assert_eq!(p.rework_cells(), 0);
}

/// A careful crew's pad, finished as far as its scans showed when each
/// cell was sintered: every cell read `TARGET` three times while compacted,
/// except `flattered`, whose three readings were `before` above the target.
/// Then `scans` scans of the finished pad, in which `flattered` reads
/// `since` above the target.
fn finished_pad(
    w: &mut WorldModel,
    p: &Planner,
    flattered: GridCoord,
    (before, since): (f64, f64),
    scans: u32,
) {
    let cells: Vec<(GridCoord, f64)> = p
        .order()
        .iter()
        .map(|&c| (c, if c == flattered { before } else { 0.0 }))
        .collect();
    compacted(w, &cells, 3);
    for &(c, _) in &cells {
        w.site.record_sinter(c, 10.0);
    }
    rescan(w, p, flattered, since, scans, 0);
}

/// `scans` more scans of the finished pad, the first of them number `from`.
fn rescan(w: &mut WorldModel, p: &Planner, odd: GridCoord, above: f64, scans: u32, from: u32) {
    for i in from..from + scans {
        let obs: Vec<CellObservation> = p
            .order()
            .iter()
            .map(|&c| {
                let e = TARGET + if c == odd { above } else { 0.0 };
                observe(c, e, Surface::Stabilized, false)
            })
            .collect();
        let t = 20.0 + f64::from(i);
        w.site.apply_scan(&obs, t, t + 0.5);
    }
}

#[test]
fn a_careful_crew_scans_again_before_it_redoes_a_sintered_cell() {
    let cfg = Config::default();
    let (mut w, reg) = careful_world();
    let mut p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    let odd = GridCoord { x: 3, y: 3 };
    // One cell stands 3.6 cm above its neighbours, a millimetre over the
    // 3.49 cm a 2 degree slope allows. The scans it was sintered on showed
    // 2.5 cm, and with them the pad looks finished.
    finished_pad(&mut w, &p, odd, (0.025, 0.036), 1);
    assert!(p.progress(&w, &cfg).complete());
    // One scan since: a millimetre over is far inside its scatter, and no
    // cell has been scanned often enough to be cleared. Scan again.
    assert_eq!(p.after_recheck(&w, &cfg), 0);
    assert!(p.recheck.wanted);
    assert_eq!(
        (p.recheck.rounds, p.recheck.passes, p.rework_cells()),
        (0, 1, 0)
    );
    // After 48 scans the step is still too close to call, and scanning has
    // told what it can: the one cell in doubt is redone, and only that one.
    rescan(&mut w, &p, odd, 0.036, 46, 1);
    assert_eq!(p.after_recheck(&w, &cfg), 0);
    rescan(&mut w, &p, odd, 0.036, 1, 47);
    assert_eq!(p.after_recheck(&w, &cfg), 1);
    assert!(!p.recheck.wanted);
    assert_eq!(
        (p.recheck.rounds, p.recheck.passes, p.rework_cells()),
        (1, 0, 1)
    );
    assert!(p.cell_status(&w, odd, &cfg).grade);
}

#[test]
fn a_cell_clearly_out_is_redone_without_more_scans() {
    let cfg = Config::default();
    let (mut w, reg) = careful_world();
    let mut p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    let odd = GridCoord { x: 4, y: 2 };
    // 4.6 cm above its neighbours in each of four scans since it was
    // sintered: 1.1 cm over the limit, where the step's standard error is
    // 0.7 cm. That is not three standard errors, so the pad is scanned
    // again; after sixteen scans it is, and the cell is redone at once.
    finished_pad(&mut w, &p, odd, (0.02, 0.046), 4);
    assert_eq!(p.after_recheck(&w, &cfg), 0);
    assert!(p.recheck.wanted);
    rescan(&mut w, &p, odd, 0.046, 12, 4);
    assert_eq!(p.after_recheck(&w, &cfg), 1);
    assert_eq!(p.rework_cells(), 1);
    assert!(p.cell_status(&w, odd, &cfg).grade);
}

#[test]
fn a_cell_no_scan_reaches_does_not_keep_the_scout_scanning() {
    let cfg = Config {
        recheck_max_samples: 3,
        ..Config::default()
    };
    let (mut w, reg) = careful_world();
    let mut p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    // A finished pad never scanned since: no cell can be cleared.
    finished_pad(&mut w, &p, GridCoord { x: 3, y: 3 }, (0.0, 0.0), 0);
    for pass in 1..=3 {
        assert_eq!(p.after_recheck(&w, &cfg), 0);
        assert_eq!(p.recheck.passes, pass);
    }
    // The fourth recheck stops asking for scans and ends the round.
    p.after_recheck(&w, &cfg);
    assert!(!p.recheck.wanted);
    assert_eq!((p.recheck.rounds, p.recheck.passes), (1, 0));
}

#[test]
fn workload_counts_remaining_stages() {
    let cfg = Config::default();
    let (mut w, reg) = world_with_crew_at(0, 0);
    let p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    survey_all(&mut w, &p, TARGET - 0.2);
    let wl = p.workload(&w, &reg, &cfg);
    assert_eq!(wl[&CapabilityName::Grade].steps, 16.0);
    assert_eq!(wl[&CapabilityName::Sinter].steps, 16.0);
    assert!(wl[&CapabilityName::Haul].units > 16.0 * 0.1 * 1500.0);
    assert_eq!(wl[&CapabilityName::Haul].passes, 2.0);
}

#[test]
fn refused_grades_tighten_the_window_for_that_cell() {
    let cfg = Config::default();
    let (mut w, reg) = world_with_crew_at(0, 0);
    let mut p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    let c = GridCoord { x: 3, y: 4 };
    w.site.apply_scan(
        &[observe(c, TARGET - 0.07, Surface::Cleared, false)],
        0.0,
        1.0,
    );
    assert!(p.cell_status(&w, c, &cfg).grade, "inside the usual window");
    p.tighten(c);
    let st = p.cell_status(&w, c, &cfg);
    assert!(
        !st.grade && st.fill_kg > 0.0,
        "fill first after a refusal: {st:?}"
    );
    w.site.apply_scan(
        &[observe(c, TARGET + 0.07, Surface::Cleared, false)],
        0.0,
        2.0,
    );
    assert!(p.cell_status(&w, c, &cfg).dig, "cut first after a refusal");
}

#[test]
fn a_berm_out_of_coverage_is_built_from_covered_borrow_cells() {
    let cfg = Config::default();
    let berm = vec![GridCoord { x: 3, y: 9 }, GridCoord { x: 4, y: 9 }];
    let mut sc = scenario(vec![robot(
        "ex",
        0.2,
        vec![
            cap(CapabilityName::Dig, 0.01, 50.0, None),
            cap(CapabilityName::Haul, 2.0, 0.01, Some(250.0)),
            cap(CapabilityName::Scan, 4.0, 0.05, None),
        ],
    )]);
    sc.site.cell_size_m = 1.0;
    // The base station reaches row 7 of the pad's column, not the berm on row 9.
    sc.base_station = BaseStation {
        position: GridCoord { x: 3, y: 0 },
        range_m: 7.5,
    };
    let mut w = WorldModel::new(&sc);
    w.fleet.update(&[state("ex", 0, 0)]);
    let reg = Registry::new(&sc);
    let mut p = Planner::new("g", &pad_spec(berm.clone()), &w, &reg, &cfg);
    let at = |x, y| GridCoord { x, y };
    assert_eq!(p.borrow[0].map(|b| b.at), Some(at(3, 7)));
    assert_eq!(p.borrow[1].map(|b| b.at), Some(at(4, 7)));
    let cov = crate::coverage::Coverage::new(&w.base_station, 1.0, &[]);
    for pile in &p.material.piles {
        assert!(cov.covers(pile.at), "pile at {:?}", pile.at);
        assert!(pile.at != at(3, 7) && pile.at != at(4, 7));
    }

    let scan: Vec<CellObservation> = berm
        .iter()
        .chain([at(3, 7), at(4, 7)].iter())
        .map(|&c| observe(c, -9.5, Surface::Natural, false))
        .collect();
    w.site.apply_scan(&scan, 0.0, 1.0);
    p.after_scan(&w, &scan, &cfg);
    let tasks = p.tasks(&w, &Busy::default(), &cfg);
    assert!(
        !tasks.iter().any(|t| t.stage == Stage::Berm),
        "no hauls out of coverage"
    );
    let dig = tasks
        .iter()
        .find(|t| t.key == TaskKey::Cell(Stage::Dig, 3, 7))
        .expect("a borrow dig");
    assert_eq!(dig.cast_to, Some(at(3, 9)));
    // Deep enough for the berm's need even if the payload takes the first 250 kg.
    let need = p.berm_need_kg(&w, 0, &cfg);
    let spoil = w.site.dig_spoil_kg(at(3, 7), dig.elevation_m, &cfg);
    assert!((spoil - (need + 250.0)).abs() < 1e-6, "{spoil} vs {need}");
    // The dig and its spoil feed the berm; the workload counts it as digging.
    let wl = p.workload(&w, &reg, &cfg);
    assert!(wl.get(&CapabilityName::Dig).is_some_and(|d| d.steps >= 2.0));
    w.site.record_dig(at(3, 7), dig.elevation_m, 2.0, &cfg);
    w.site
        .record_deposit(at(3, 9), spoil - 250.0, true, 2.0, &cfg);
    p.record_berm_deposit(0, spoil - 250.0);
    assert!(p.berm_need_kg(&w, 0, &cfg) < cfg.min_haul_kg);
    // A dug-out borrow cell hands the berm back to hauling.
    w.site.apply_scan(
        &[observe(
            at(4, 7),
            p.borrow[1].unwrap().floor_m.unwrap(),
            Surface::Cleared,
            false,
        )],
        3.0,
        4.0,
    );
    let tasks = p.tasks(&w, &Busy::default(), &cfg);
    assert!(
        tasks
            .iter()
            .any(|t| t.stage == Stage::Berm && t.cell() == at(4, 9))
    );
}
