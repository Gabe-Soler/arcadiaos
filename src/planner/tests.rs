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
    let (mut w, reg) = world_with_crew_at(0, 0);
    let p = Planner::new("g", &pad_spec(vec![]), &w, &reg, &cfg);
    let c = GridCoord { x: 3, y: 3 };
    assert!(p.cell_status(&w, c, &cfg).survey);

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
    assert!(
        (st.fill_kg - (0.3 - cfg.fill_aim_m) * 1500.0).abs() < 1e-6,
        "{}",
        st.fill_kg
    );
    assert!(st.max_fill_kg > st.fill_kg);

    // Cleared and inside the window: grade.
    w.site.record_dig(c, TARGET, 3.0, &cfg);
    w.site.record_deposit(c, st.fill_kg, false, 4.0, &cfg);
    let st = p.cell_status(&w, c, &cfg);
    assert!(st.grade && !st.dig && st.fill_kg == 0.0, "{st:?}");

    w.site.record_grade(c, TARGET, 5.0);
    assert!(p.cell_status(&w, c, &cfg).compact);
    w.site.record_compact(c, 0.01, 6.0);
    assert!(p.cell_status(&w, c, &cfg).sinter);
    w.site.record_sinter(c, 7.0);
    let st = p.cell_status(&w, c, &cfg);
    assert!(st.finished && !st.verified);
    w.site.apply_scan(
        &[observe(c, TARGET - 0.01, Surface::Stabilized, false)],
        7.0,
        8.0,
    );
    assert!(p.cell_status(&w, c, &cfg).verified);
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
