//! Drives the real `arcadiaos` binary over stdio, playing the simulator's side.

use arcadia_protocol::*;
use std::io::{BufReader, Read};
use std::process::{Command as Proc, Stdio};

fn scanner(id: &str) -> RobotSpec {
    RobotSpec {
        robot_id: id.into(),
        model: None,
        mass_kg: 100.0,
        battery_capacity_wh: 1000.0,
        max_speed_m_s: 0.2,
        drive_energy_wh_per_m: 0.5,
        idle_power_w: 10.0,
        capabilities: vec![CapabilitySpec {
            name: CapabilityName::Scan,
            rate: 1.0,
            energy_wh_per_unit: 0.1,
            capacity: None,
            range_m: None,
        }],
    }
}

fn hello_with(version: &str, size: u32) -> SimToCoordinator {
    SimToCoordinator::Hello(Box::new(Hello {
        protocol_version: version.into(),
        simulator: SoftwareInfo {
            name: "test-sim".into(),
            version: "0".into(),
        },
        run_id: "run-test".into(),
        seed: 7,
        scenario: Scenario {
            scenario_id: "empty".into(),
            start_t: 0.0,
            time_limit_t: SOL_S,
            sol_length_s: SOL_S,
            site: SiteMap {
                site_id: "site".into(),
                name: None,
                width: size,
                height: size,
                cell_size_m: 1.0,
                cells: vec![],
            },
            robots: vec![scanner("r1")],
            stations: vec![ChargeStation {
                station_id: "st-1".into(),
                position: GridCoord { x: 0, y: 0 },
                max_power_w: 2000.0,
                slots: 1,
            }],
            depots: vec![],
            policy: Policy {
                policy_version: 1,
                default_level: AuthorityLevel::AUTONOMOUS,
                levels: Default::default(),
                level3: Level3Rule {
                    max_wait_s: 3600.0,
                    default_fallback: Fallback::Abort,
                },
            },
        },
    }))
}

fn hello(version: &str) -> SimToCoordinator {
    hello_with(version, 4)
}

fn observation(seq: u64) -> SimToCoordinator {
    SimToCoordinator::Observation(Observation {
        seq,
        t: seq as f64 * 60.0,
        events: vec![],
        robots: vec![],
    })
}

fn robot_state(id: &str, x: u32, y: u32) -> RobotState {
    RobotState {
        robot_id: id.into(),
        position: GridCoord { x, y },
        status: RobotStatus::Idle,
        battery_wh: 1000.0,
        payload_kg: 0.0,
        current_command: None,
        health: HealthTelemetry {
            wheel_slip: 0.0,
            motor_current_ratio: 1.0,
            dust_coverage: 0.0,
            temperature_c: -50.0,
        },
        in_relay_coverage: true,
    }
}

fn goal_event() -> Event {
    Event {
        t: 0.0,
        kind: EventKind::EarthMessage {
            sent_t: 0.0,
            message: EarthToSite {
                message_id: "earth-1".into(),
                body: Uplink::Goal {
                    goal_id: "pad".into(),
                    priority: 1,
                    deadline_t: None,
                    objective: Objective::LandingPad {
                        pad: LandingPadSpec {
                            area: GridRect {
                                min: GridCoord { x: 4, y: 4 },
                                max: GridCoord { x: 7, y: 7 },
                            },
                            target_elevation_m: -10.0,
                            surface_tolerance_m: 0.05,
                            max_slope_deg: 2.0,
                            stabilize: true,
                            berm: None,
                        },
                    },
                },
            },
        },
    }
}

/// Send `msgs`, close stdin, and collect every reply, the exit status and stderr.
fn session_with(
    msgs: &[SimToCoordinator],
    log: Option<&str>,
) -> (Vec<CoordinatorToSim>, bool, String) {
    let mut cmd = Proc::new(env!("CARGO_BIN_EXE_arcadiaos"));
    cmd.stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    match log {
        Some(level) => cmd.env("ARCADIAOS_LOG", level),
        None => cmd.env_remove("ARCADIAOS_LOG"),
    };
    let mut child = cmd.spawn().expect("spawn arcadiaos");
    {
        let mut stdin = child.stdin.take().unwrap();
        for m in msgs {
            // The coordinator may exit early (e.g. on version mismatch); ignore broken pipes.
            let _ = wire::send(&mut stdin, m);
        }
    }
    let mut stdout = BufReader::new(child.stdout.take().unwrap());
    let mut replies = Vec::new();
    while let Some(r) = wire::recv::<_, CoordinatorToSim>(&mut stdout).expect("valid reply") {
        replies.push(r);
    }
    let mut stderr = String::new();
    child
        .stderr
        .take()
        .unwrap()
        .read_to_string(&mut stderr)
        .unwrap();
    (replies, child.wait().unwrap().success(), stderr)
}

fn session(msgs: &[SimToCoordinator]) -> (Vec<CoordinatorToSim>, bool) {
    let (replies, ok, _) = session_with(msgs, None);
    (replies, ok)
}

fn decisions(replies: &[CoordinatorToSim]) -> Vec<&Decision> {
    replies
        .iter()
        .filter_map(|r| match r {
            CoordinatorToSim::Decision(d) => Some(d),
            _ => None,
        })
        .collect()
}

#[test]
fn handshake_decisions_and_end() {
    let (replies, ok) = session(&[
        hello(PROTOCOL_VERSION),
        observation(0),
        observation(1),
        SimToCoordinator::End(End {
            t: 120.0,
            reason: EndReason::TimeLimit,
        }),
    ]);
    assert!(ok);
    assert_eq!(replies.len(), 3);
    match &replies[0] {
        CoordinatorToSim::Welcome(w) => {
            assert_eq!(w.protocol_version, PROTOCOL_VERSION);
            assert_eq!(w.coordinator.name, "arcadiaos");
        }
        other => panic!("expected welcome, got {other:?}"),
    }
    for (i, r) in replies[1..].iter().enumerate() {
        match r {
            // No goal yet, so nothing to do.
            CoordinatorToSim::Decision(d) => {
                assert_eq!(d.seq, i as u64);
                assert!(d.commands.is_empty());
            }
            other => panic!("expected decision, got {other:?}"),
        }
    }
}

#[test]
fn rejects_incompatible_version() {
    let (replies, ok) = session(&[hello("9.9.0")]);
    assert!(!ok);
    assert!(matches!(
        replies.as_slice(),
        [CoordinatorToSim::Error(ErrorMessage {
            code: ErrorCode::VersionMismatch,
            ..
        })]
    ));
}

#[test]
fn rejects_observation_before_hello() {
    let (replies, ok) = session(&[observation(0)]);
    assert!(!ok);
    assert!(matches!(
        replies.as_slice(),
        [CoordinatorToSim::Error(ErrorMessage {
            code: ErrorCode::UnexpectedMessage,
            ..
        })]
    ));
}

#[test]
fn surveys_once_the_goal_arrives_and_backs_off_after_a_rejection() {
    let first = SimToCoordinator::Observation(Observation {
        seq: 0,
        t: 0.0,
        events: vec![goal_event()],
        robots: vec![robot_state("r1", 1, 1)],
    });
    let (replies, ok) = session(&[hello_with(PROTOCOL_VERSION, 12), first.clone()]);
    assert!(ok);
    let d = decisions(&replies);
    assert_eq!(d.len(), 1);
    let [cmd] = d[0].commands.as_slice() else {
        panic!("expected one command, got {:?}", d[0].commands)
    };
    assert_eq!(cmd.robot_id, "r1");
    assert_eq!(cmd.command_id, "os-000001");
    let Action::Scan { area } = cmd.action else {
        panic!("expected a survey scan, got {:?}", cmd.action)
    };
    assert!(area.min.x >= 4 && area.max.x <= 7 && area.min.y >= 4 && area.max.y <= 7);

    // The simulator refuses the scan: no immediate identical retry, and a
    // wake-up for when the back-off ends.
    let rejected = SimToCoordinator::Observation(Observation {
        seq: 1,
        t: 0.0,
        events: vec![Event {
            t: 0.0,
            kind: EventKind::CommandRejected {
                command_id: cmd.command_id.clone(),
                robot_id: "r1".into(),
                reason: RejectReason::InvalidTarget,
                detail: Some("test".into()),
            },
        }],
        robots: vec![robot_state("r1", 1, 1)],
    });
    let (replies, ok) = session(&[hello_with(PROTOCOL_VERSION, 12), first, rejected]);
    assert!(ok);
    let d = decisions(&replies);
    assert_eq!(d.len(), 2);
    assert_eq!(d[1].seq, 1);
    assert!(
        !d[1]
            .commands
            .iter()
            .any(|c| matches!(c.action, Action::Scan { .. })),
        "{:?}",
        d[1].commands
    );
    assert!(d[1].wake_at.is_some_and(|w| w > 0.0));
}

#[test]
fn silent_by_default_and_verbose_on_request() {
    let msgs = [
        hello_with(PROTOCOL_VERSION, 12),
        SimToCoordinator::Observation(Observation {
            seq: 0,
            t: 0.0,
            events: vec![goal_event()],
            robots: vec![robot_state("r1", 1, 1)],
        }),
        SimToCoordinator::End(End {
            t: 10.0,
            reason: EndReason::TimeLimit,
        }),
    ];
    let (_, ok, quiet) = session_with(&msgs, None);
    assert!(ok);
    assert!(quiet.is_empty(), "unexpected stderr: {quiet}");
    let (_, ok, loud) = session_with(&msgs, Some("info"));
    assert!(ok);
    assert!(loud.contains("arcadiaos"), "{loud}");
}
