//! ArcadiaOS on synthetic scenarios: crews of 4 to 20 robots with mixed and
//! overlapping capability sets, several stations, small batteries, and no
//! hauler at all. Nothing in ArcadiaOS knows these crews; everything is
//! derived from `hello` and observations.

mod support;

use arcadia_protocol::{CapabilityName::*, ChargeStation, GridCoord, GridRect, SOL_S};
use arcadiaos::Coordinator;
use arcadiaos::diag::{Diag, Level};
use support::minisim::*;

fn pad(x0: u32, y0: u32, side: u32) -> GridRect {
    GridRect {
        min: GridCoord { x: x0, y: y0 },
        max: GridCoord {
            x: x0 + side - 1,
            y: y0 + side - 1,
        },
    }
}

fn berm_row(x0: u32, x1: u32, y: u32) -> Vec<GridCoord> {
    (x0..=x1).map(|x| GridCoord { x, y }).collect()
}

fn coordinator() -> Coordinator {
    Coordinator::with_config(arcadiaos::Config::default(), Diag::new(Level::from_env()))
}

/// Run a setup to completion and return the end time and stats.
fn build(setup: Setup) -> (f64, Stats) {
    let label = format!("seed {} with {} robots", setup.seed, setup.robots.len());
    let mut sim = MiniSim::new(setup);
    let mut os = coordinator();
    let ending = sim.run(&mut os, 5.0 * SOL_S);
    let Ending::Complete(t) = ending else {
        panic!(
            "{label}: did not finish ({ending:?}), stats {:?}",
            (sim.stats.decisions, sim.stats.commands)
        );
    };
    eprintln!(
        "{label}: done in {:.3} sols, {} decisions, {} commands, slowest decision {:.2} ms",
        t / SOL_S,
        sim.stats.decisions,
        sim.stats.commands,
        sim.stats.max_decision_ms
    );
    assert_eq!(sim.stats.offline, 0, "{label}: a robot ran flat");
    assert_eq!(sim.stats.rejected, 0, "{label}: rejected commands");
    (t, sim.stats)
}

fn small_crew() -> Vec<arcadia_protocol::RobotSpec> {
    vec![
        scout("sc-1"),
        excavator("ex-1"),
        hauler("hl-1"),
        compactor("cp-1"),
    ]
}

#[test]
fn small_crew_builds_the_pad() {
    for seed in [1, 2] {
        let (t, stats) = build(Setup {
            seed,
            size: 30,
            pad: pad(10, 10, 8),
            berm: berm_row(11, 16, 20),
            relief_m: 0.25,
            robots: lined_up(small_crew(), 3, 3),
            stations: vec![station("st-1", 2, 2)],
            start_battery: vec![],
        });
        assert!(t < SOL_S, "seed {seed}: {t:.0} s");
        assert!(stats.failed == 0, "seed {seed}: {} failures", stats.failed);
    }
}

#[test]
fn medium_crew_is_faster_than_small() {
    let setup = |robots| Setup {
        seed: 3,
        size: 34,
        pad: pad(10, 10, 12),
        berm: berm_row(11, 20, 24),
        relief_m: 0.25,
        robots,
        stations: vec![station("st-1", 2, 2), station("st-2", 30, 2)],
        start_battery: vec![],
    };
    let (small, _) = build(setup(lined_up(small_crew(), 3, 3)));
    let medium = vec![
        scout("sc-1"),
        scout("sc-2"),
        excavator("ex-1"),
        excavator("ex-2"),
        excavator("ex-3"),
        hauler("hl-1"),
        hauler("hl-2"),
        compactor("cp-1"),
        compactor("cp-2"),
        compactor("cp-3"),
    ];
    let (medium, _) = build(setup(lined_up(medium, 3, 3)));
    assert!(
        medium < 0.6 * small,
        "ten robots should beat four clearly: {medium:.0} s vs {small:.0} s"
    );
}

#[test]
fn large_mixed_crew_with_overlapping_capabilities() {
    // Twenty robots: specialists, generalists and multi-capability support,
    // with three stations around the site.
    let generalist = |id: &str| {
        robot(
            id,
            8000.0,
            0.2,
            0.8,
            30.0,
            vec![
                cap(Scan, 2.0, 0.05, None),
                cap(Dig, 0.003, 100.0, None),
                cap(Grade, 0.04, 8.0, None),
                cap(Haul, 2.0, 0.01, Some(200.0)),
                cap(Compact, 0.08, 4.0, None),
                cap(Sinter, 0.008, 60.0, None),
            ],
        )
    };
    let digger = |id: &str| {
        robot(
            id,
            6000.0,
            0.15,
            0.9,
            35.0,
            vec![cap(Dig, 0.005, 100.0, None)],
        )
    };
    let grader = |id: &str| {
        robot(
            id,
            6000.0,
            0.2,
            0.9,
            30.0,
            vec![cap(Grade, 0.05, 8.0, None), cap(Compact, 0.1, 4.0, None)],
        )
    };
    let sinterer = |id: &str| {
        robot(
            id,
            9000.0,
            0.15,
            1.0,
            30.0,
            vec![cap(Sinter, 0.012, 60.0, None)],
        )
    };
    let charger = |id: &str| {
        robot(
            id,
            10000.0,
            0.3,
            0.7,
            20.0,
            vec![
                cap(Charge, 0.6, 0.0, Some(6000.0)),
                cap(Haul, 3.0, 0.008, Some(300.0)),
            ],
        )
    };
    let crew = vec![
        scout("sc-1"),
        scout("sc-2"),
        digger("dg-1"),
        digger("dg-2"),
        digger("dg-3"),
        digger("dg-4"),
        grader("gr-1"),
        grader("gr-2"),
        grader("gr-3"),
        hauler("hl-1"),
        hauler("hl-2"),
        hauler("hl-3"),
        sinterer("si-1"),
        sinterer("si-2"),
        sinterer("si-3"),
        sinterer("si-4"),
        generalist("gn-1"),
        generalist("gn-2"),
        charger("ch-1"),
        charger("ch-2"),
    ];
    let (t, stats) = build(Setup {
        seed: 5,
        size: 40,
        pad: pad(12, 12, 16),
        berm: berm_row(13, 26, 30),
        relief_m: 0.3,
        robots: lined_up(crew, 4, 4),
        stations: vec![
            station("st-1", 2, 2),
            station("st-2", 36, 2),
            station("st-3", 2, 36),
        ],
        start_battery: vec![],
    });
    assert!(t < SOL_S, "{t:.0} s");
    // Decisions stay fast even with twenty robots bidding on hundreds of tasks.
    let limit_ms = if cfg!(debug_assertions) { 1000.0 } else { 50.0 };
    assert!(
        stats.max_decision_ms < limit_ms,
        "slowest decision took {:.1} ms",
        stats.max_decision_ms
    );
}

#[test]
fn generalist_crew_shares_every_stage() {
    let all = |id: &str| {
        robot(
            id,
            8000.0,
            0.2,
            0.8,
            30.0,
            vec![
                cap(Scan, 2.0, 0.05, None),
                cap(Dig, 0.004, 100.0, None),
                cap(Grade, 0.05, 8.0, None),
                cap(Haul, 2.5, 0.01, Some(300.0)),
                cap(Compact, 0.1, 4.0, None),
                cap(Sinter, 0.01, 60.0, None),
            ],
        )
    };
    let crew: Vec<_> = (1..=5).map(|i| all(&format!("gen-{i}"))).collect();
    let (t, stats) = build(Setup {
        seed: 7,
        size: 30,
        pad: pad(8, 8, 10),
        berm: berm_row(9, 16, 20),
        relief_m: 0.25,
        robots: lined_up(crew, 2, 2),
        stations: vec![station("st-1", 1, 1), station("st-2", 28, 28)],
        start_battery: vec![],
    });
    assert!(t < SOL_S, "{t:.0} s");
    // Every robot did real pad work.
    for i in 1..=5 {
        let id = format!("\"robot_id\":\"gen-{i}\"");
        let worked = stats.command_log.iter().any(|c| {
            c.contains(&id)
                && (c.contains("\"sinter\"") || c.contains("\"compact\"") || c.contains("\"dig\""))
        });
        assert!(worked, "gen-{i} never worked the pad");
    }
}

#[test]
fn small_batteries_force_safe_recharging() {
    let mut crew = small_crew();
    for r in &mut crew {
        r.battery_capacity_wh *= 0.2;
    }
    let (_, stats) = build(Setup {
        seed: 11,
        size: 34,
        pad: pad(10, 10, 12),
        berm: berm_row(11, 20, 24),
        relief_m: 0.2,
        robots: lined_up(crew, 3, 3),
        stations: vec![station("st-1", 2, 2)],
        start_battery: vec![],
    });
    let charges = stats
        .command_log
        .iter()
        .filter(|c| c.contains("\"charge\""))
        .count();
    assert!(
        charges >= 5,
        "only {charges} charges with fifth-size batteries"
    );
    assert_eq!(stats.failed, 0, "no energy failures");
}

#[test]
fn crew_without_a_hauler_casts_spoil_into_fill() {
    let crew = vec![
        scout("sc-1"),
        robot(
            "dg-1",
            6000.0,
            0.15,
            0.9,
            35.0,
            vec![cap(Dig, 0.004, 100.0, None), cap(Grade, 0.05, 8.0, None)],
        ),
        compactor("cp-1"),
    ];
    let (t, _) = build(Setup {
        seed: 13,
        size: 26,
        pad: pad(8, 8, 8),
        berm: vec![],
        relief_m: 0.3,
        robots: lined_up(crew, 2, 2),
        stations: vec![station("st-1", 1, 1)],
        start_battery: vec![],
    });
    assert!(t < SOL_S, "{t:.0} s");
}

#[test]
fn identical_scenarios_give_identical_decisions() {
    let run = || {
        let mut sim = MiniSim::new(Setup {
            seed: 17,
            size: 30,
            pad: pad(10, 10, 8),
            berm: berm_row(11, 16, 20),
            relief_m: 0.25,
            robots: lined_up(small_crew(), 3, 3),
            stations: vec![station("st-1", 2, 2), station("st-2", 27, 27)],
            start_battery: vec![],
        });
        let mut os = coordinator();
        let ending = sim.run(&mut os, 5.0 * SOL_S);
        (ending, sim.stats.command_log)
    };
    let (a_end, a_log) = run();
    let (b_end, b_log) = run();
    assert!(matches!(a_end, Ending::Complete(_)));
    assert_eq!(a_end, b_end);
    assert_eq!(a_log, b_log);
}

/// A slow compactor-sinterer working far from a weak station, and a
/// dedicated mobile charger parked by the pad.
fn far_station_setup(start_battery: Vec<(usize, f64)>, compactor_at: GridCoord) -> Setup {
    let mut cp = compactor("cp-1");
    cp.battery_capacity_wh = 2500.0;
    let charger = robot(
        "ch-1",
        3000.0,
        0.3,
        0.7,
        20.0,
        vec![cap(Charge, 0.5, 0.0, Some(12000.0))],
    );
    let robots = vec![
        (scout("sc-1"), GridCoord { x: 40, y: 38 }),
        (excavator("ex-1"), GridCoord { x: 41, y: 38 }),
        (hauler("hl-1"), GridCoord { x: 42, y: 38 }),
        (cp, compactor_at),
        (charger, GridCoord { x: 30, y: 30 }),
    ];
    Setup {
        seed: 19,
        size: 60,
        pad: pad(44, 44, 8),
        berm: berm_row(45, 50, 55),
        relief_m: 0.2,
        robots,
        stations: vec![ChargeStation {
            station_id: "far".into(),
            position: GridCoord { x: 1, y: 1 },
            max_power_w: 1000.0,
            slots: 1,
        }],
        start_battery,
    }
}

#[test]
fn mobile_charger_beats_a_weak_distant_station() {
    let (_, stats) = build(far_station_setup(vec![], GridCoord { x: 43, y: 38 }));
    let mobile = stats
        .command_log
        .iter()
        .filter(|c| {
            c.contains("\"charge\"")
                && (c.contains("\"source\":\"ch-1\"") || c.contains("\"source\":\"hl-1\""))
        })
        .count();
    assert!(
        mobile >= 1,
        "no mobile charging: {:?}",
        stats
            .command_log
            .iter()
            .filter(|c| c.contains("charge"))
            .collect::<Vec<_>>()
    );
    assert_eq!(stats.failed, 0);
}

#[test]
fn stranded_robot_is_rescued_by_a_mobile_charger() {
    // The compactor starts nearly flat, too far from every charger to drive there.
    let at = GridCoord { x: 47, y: 47 };
    let (_, stats) = build(far_station_setup(vec![(3, 4.0)], at));
    let target = format!("\"target\":{{\"x\":{},\"y\":{}}}", at.x, at.y);
    let rescue = stats
        .command_log
        .iter()
        .position(|c| {
            c.contains("\"move_to\"") && c.contains(&target) && !c.contains("\"robot_id\":\"cp-1\"")
        })
        .expect("a mobile charger drove to the stranded robot");
    let charged = stats.command_log[rescue..].iter().any(|c| {
        c.contains("\"robot_id\":\"cp-1\"")
            && c.contains("\"charge\"")
            && (c.contains("\"source\":\"ch-1\"") || c.contains("\"source\":\"hl-1\""))
    });
    assert!(charged, "the stranded robot charged from the rescuer");
}
