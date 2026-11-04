//! The coordinator stays sane on awkward but valid input: hostile
//! rejections, unknown robots and commands, goals partly off the site, no
//! charging stations, immobile robots, and repeated goals.

use arcadia_protocol::*;
use arcadiaos::Coordinator;
use arcadiaos::diag::{Diag, Level};

fn coordinator() -> Coordinator {
    Coordinator::with_config(arcadiaos::Config::default(), Diag::new(Level::from_env()))
}

fn capability(name: CapabilityName, rate: f64, capacity: Option<f64>) -> CapabilitySpec {
    CapabilitySpec {
        name,
        rate,
        energy_wh_per_unit: 1.0,
        capacity,
        range_m: None,
    }
}

fn spec(id: &str, speed: f64, caps: Vec<CapabilitySpec>) -> RobotSpec {
    RobotSpec {
        robot_id: id.into(),
        model: None,
        mass_kg: 100.0,
        battery_capacity_wh: 5000.0,
        max_speed_m_s: speed,
        drive_energy_wh_per_m: 0.5,
        idle_power_w: 10.0,
        capabilities: caps,
    }
}

fn crew() -> Vec<RobotSpec> {
    use CapabilityName::*;
    vec![
        spec("scan", 0.5, vec![capability(Scan, 4.0, None)]),
        spec(
            "dig",
            0.2,
            vec![capability(Dig, 0.01, None), capability(Grade, 0.05, None)],
        ),
        spec("haul", 0.3, vec![capability(Haul, 3.0, Some(400.0))]),
        spec(
            "finish",
            0.2,
            vec![
                capability(Compact, 0.1, None),
                capability(Sinter, 0.01, None),
            ],
        ),
        // Cannot move at all.
        spec("stuck", 0.0, vec![capability(Dig, 0.01, None)]),
    ]
}

fn hello(robots: Vec<RobotSpec>, stations: Vec<ChargeStation>) -> Box<Hello> {
    Box::new(Hello {
        protocol_version: PROTOCOL_VERSION.into(),
        simulator: SoftwareInfo {
            name: "hostile".into(),
            version: "0".into(),
        },
        run_id: "r".into(),
        seed: 1,
        scenario: Scenario {
            scenario_id: "s".into(),
            start_t: 0.0,
            time_limit_t: SOL_S,
            sol_length_s: SOL_S,
            site: SiteMap {
                site_id: "site".into(),
                name: None,
                width: 12,
                height: 12,
                cell_size_m: 1.0,
                cells: (0..12)
                    .flat_map(|y| (0..12).map(move |x| GridCoord { x, y }))
                    .map(|at| CellObservation {
                        at,
                        cell: Cell {
                            elevation_m: Some(-10.0 + 0.01 * f64::from(at.x)),
                            ..Cell::default()
                        },
                    })
                    .collect(),
            },
            robots,
            stations,
            depots: vec![],
            power: PowerSystem {
                mode: PowerMode::Hybrid,
                fission_w: 10_000.0,
                solar_peak_w: 20_000.0,
                storage_capacity_wh: 100_000.0,
                essential_load_w: 2_000.0,
            },
            base_station: BaseStation {
                position: GridCoord { x: 0, y: 0 },
                range_m: 1000.0,
            },
            policy: Policy {
                policy_version: 1,
                default_level: AuthorityLevel::NOTIFY,
                levels: Default::default(),
                level3: Level3Rule {
                    max_wait_s: 60.0,
                    default_fallback: Fallback::Proceed,
                },
            },
        },
    })
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

fn states() -> Vec<RobotState> {
    let mut s: Vec<RobotState> = ["scan", "dig", "haul", "finish", "stuck"]
        .iter()
        .enumerate()
        .map(|(i, id)| state(id, i as u32, 0))
        .collect();
    // A robot the scenario never mentioned.
    s.push(state("stranger", 3, 3));
    s
}

fn goal(id: &str, area: GridRect, berm: Vec<GridCoord>) -> Event {
    Event {
        t: 0.0,
        kind: EventKind::EarthMessage {
            sent_t: 0.0,
            message: EarthToSite {
                message_id: format!("m-{id}"),
                body: Uplink::Goal {
                    goal_id: id.into(),
                    priority: 1,
                    deadline_t: None,
                    objective: Objective::LandingPad {
                        pad: LandingPadSpec {
                            area,
                            target_elevation_m: -10.0,
                            surface_tolerance_m: 0.05,
                            max_slope_deg: 2.0,
                            stabilize: true,
                            berm: Some(BermSpec {
                                cells: berm,
                                height_m: 0.5,
                            }),
                        },
                    },
                },
            },
        },
    }
}

fn rect(x0: u32, y0: u32, x1: u32, y1: u32) -> GridRect {
    GridRect {
        min: GridCoord { x: x0, y: y0 },
        max: GridCoord { x: x1, y: y1 },
    }
}

/// Every number in a decision is finite and every id is well formed.
fn check(d: &Decision) {
    let ok_id = |s: &str| {
        !s.is_empty()
            && s.len() <= 64
            && s.chars()
                .all(|c| c.is_ascii_alphanumeric() || "_.:-".contains(c))
    };
    for c in &d.commands {
        assert!(ok_id(&c.command_id) && ok_id(&c.robot_id), "{c:?}");
        let finite = match &c.action {
            Action::Dig {
                target_elevation_m, ..
            }
            | Action::Grade {
                target_elevation_m, ..
            } => target_elevation_m.is_finite(),
            Action::Haul { mass_kg, .. } => mass_kg.is_finite() && *mass_kg > 0.0,
            Action::Charge { until_wh, .. } => until_wh.is_none_or(|u| u.is_finite() && u > 0.0),
            Action::Wait { duration_s } => duration_s.is_finite() && *duration_s > 0.0,
            _ => true,
        };
        assert!(finite, "{c:?}");
        assert_ne!(c.robot_id, "stranger");
        assert_ne!(c.robot_id, "stuck", "an immobile robot was sent somewhere");
    }
    assert!(d.wake_at.is_none_or(f64::is_finite));
}

#[test]
fn hostile_rejections_do_not_spin_at_one_time() {
    let mut os = coordinator();
    os.start(hello(crew(), vec![]));
    let mut obs = Observation {
        seq: 0,
        t: 0.0,
        events: vec![goal(
            "pad",
            rect(4, 4, 9, 9),
            vec![GridCoord { x: 5, y: 11 }],
        )],
        robots: states(),
    };
    let mut quiet_at = None;
    for round in 0..60u64 {
        let d = os.decide(&obs);
        check(&d);
        assert_eq!(d.seq, obs.seq);
        if d.commands.is_empty() {
            quiet_at = Some(round);
            assert!(
                d.wake_at.is_some_and(|w| w > obs.t),
                "waits with a wake-up: {d:?}"
            );
            break;
        }
        // Refuse everything, at the same sim time.
        obs = Observation {
            seq: obs.seq + 1,
            t: 0.0,
            events: d
                .commands
                .iter()
                .map(|c| Event {
                    t: 0.0,
                    kind: EventKind::CommandRejected {
                        command_id: c.command_id.clone(),
                        robot_id: c.robot_id.clone(),
                        reason: RejectReason::InvalidTarget,
                        detail: None,
                    },
                })
                .collect(),
            robots: states(),
        };
    }
    let quiet_at = quiet_at.expect("the coordinator kept issuing at one sim time");
    assert!(quiet_at <= 10, "took {quiet_at} rounds to settle");
}

#[test]
fn odd_goals_and_events_are_tolerated() {
    let mut os = coordinator();
    // No stations at all.
    os.start(hello(crew(), vec![]));
    let events = vec![
        // Pad partly off the 12 × 12 site, berm cells inside the pad and off-site.
        goal(
            "pad",
            rect(8, 8, 30, 30),
            vec![
                GridCoord { x: 9, y: 9 },
                GridCoord { x: 50, y: 50 },
                GridCoord { x: 2, y: 2 },
            ],
        ),
        goal("second", rect(0, 0, 1, 1), vec![]),
        Event {
            t: 0.0,
            kind: EventKind::CommandCompleted {
                command_id: "never-sent".into(),
                robot_id: "dig".into(),
                energy_used_wh: 1.0,
            },
        },
        Event {
            t: 0.0,
            kind: EventKind::ScanResult {
                command_id: "never-sent".into(),
                robot_id: "scan".into(),
                cells: vec![CellObservation {
                    at: GridCoord { x: 99, y: 99 },
                    cell: Cell::default(),
                }],
            },
        },
    ];
    let d = os.decide(&Observation {
        seq: 0,
        t: 0.0,
        events,
        robots: states(),
    });
    check(&d);
    let planner = os.planner().expect("the first goal is planned");
    assert_eq!(planner.goal_id, "pad");
    assert_eq!(planner.pad.area, rect(8, 8, 11, 11), "clipped to the site");
    assert_eq!(planner.berm.len(), 1, "only the valid berm cell remains");
    assert!(!d.commands.is_empty(), "work starts even without stations");
}

#[test]
fn decisions_before_a_goal_are_empty_and_harmless() {
    let mut os = coordinator();
    let before_hello = os.decide(&Observation {
        seq: 0,
        t: 0.0,
        events: vec![],
        robots: vec![],
    });
    assert!(before_hello.commands.is_empty());
    os.start(hello(crew(), vec![]));
    let d = os.decide(&Observation {
        seq: 1,
        t: 5.0,
        events: vec![],
        robots: states(),
    });
    assert_eq!(d.seq, 1);
    assert!(d.commands.is_empty());
    assert!(os.planner().is_none());
}
