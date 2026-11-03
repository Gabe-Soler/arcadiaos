//! Drives the real `arcadiaos` binary over stdio, playing the simulator's side.

use arcadia_protocol::*;
use std::io::BufReader;
use std::process::{Command as Proc, Stdio};

fn hello(version: &str) -> SimToCoordinator {
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
                width: 4,
                height: 4,
                cell_size_m: 1.0,
                cells: vec![],
            },
            robots: vec![RobotSpec {
                robot_id: "r1".into(),
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
            }],
            stations: vec![],
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

fn observation(seq: u64) -> SimToCoordinator {
    SimToCoordinator::Observation(Observation {
        seq,
        t: seq as f64 * 60.0,
        events: vec![],
        robots: vec![],
    })
}

/// Send `msgs`, close stdin, and collect every reply line and the exit status.
fn session(msgs: &[SimToCoordinator]) -> (Vec<CoordinatorToSim>, bool) {
    let mut child = Proc::new(env!("CARGO_BIN_EXE_arcadiaos"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn arcadiaos");
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
    (replies, child.wait().unwrap().success())
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
            CoordinatorToSim::Decision(d) => assert_eq!(d.seq, i as u64),
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
