//! ArcadiaOS: the coordinator for robot crews building on Mars.
//!
//! Week-1 scaffold: [`run`] implements the protocol session (handshake,
//! observation → decision loop, end) with a [`Coordinator`] that does not plan
//! yet. Task allocation and scheduling arrive in weeks 3–4.

use arcadia_protocol::{
    CoordinatorToSim, Decision, EndReason, ErrorCode, ErrorMessage, Hello, Observation,
    PROTOCOL_VERSION, SimToCoordinator, SoftwareInfo, Welcome, is_compatible, wire,
};
use std::fmt;
use std::io::{BufRead, Write};

/// Decision-making core. The session loop in [`run`] feeds it observations.
#[derive(Debug, Default)]
pub struct Coordinator {
    hello: Option<Box<Hello>>,
}

impl Coordinator {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn start(&mut self, hello: Box<Hello>) {
        self.hello = Some(hello);
    }

    /// Placeholder policy: acknowledge every observation without acting.
    pub fn decide(&mut self, obs: &Observation) -> Decision {
        Decision::empty(obs.seq)
    }
}

pub fn software_info() -> SoftwareInfo {
    SoftwareInfo {
        name: "arcadiaos".into(),
        version: env!("CARGO_PKG_VERSION").into(),
    }
}

#[derive(Debug, Default, PartialEq)]
pub struct RunSummary {
    pub run_id: Option<String>,
    pub decisions: u64,
    pub end_reason: Option<EndReason>,
}

#[derive(Debug)]
pub enum RunError {
    Wire(wire::WireError),
    /// The simulator broke the protocol; an `error` message was sent back.
    Protocol(String),
    /// The simulator reported an error and the session is over.
    Simulator(ErrorMessage),
}

impl fmt::Display for RunError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RunError::Wire(e) => write!(f, "{e}"),
            RunError::Protocol(m) => write!(f, "protocol error: {m}"),
            RunError::Simulator(e) => write!(f, "simulator error ({:?}): {}", e.code, e.message),
        }
    }
}

impl std::error::Error for RunError {}

impl From<wire::WireError> for RunError {
    fn from(e: wire::WireError) -> Self {
        RunError::Wire(e)
    }
}

/// Run one protocol session until the simulator sends `end` or closes the stream.
pub fn run<R: BufRead, W: Write>(mut input: R, mut output: W) -> Result<RunSummary, RunError> {
    let mut coordinator = Coordinator::new();
    let mut summary = RunSummary::default();
    let mut started = false;

    let fail = |output: &mut W, code: ErrorCode, message: String| -> RunError {
        let msg = CoordinatorToSim::Error(ErrorMessage {
            code,
            message: message.clone(),
        });
        // Best effort: the simulator may already be gone.
        let _ = wire::send(output, &msg);
        RunError::Protocol(message)
    };

    loop {
        let msg = match wire::recv::<_, SimToCoordinator>(&mut input) {
            Ok(Some(msg)) => msg,
            Ok(None) => return Ok(summary),
            Err(wire::WireError::Json(e)) => {
                return Err(fail(&mut output, ErrorCode::InvalidMessage, e.to_string()));
            }
            Err(e) => return Err(e.into()),
        };
        match msg {
            SimToCoordinator::Hello(hello) => {
                if started {
                    return Err(fail(
                        &mut output,
                        ErrorCode::UnexpectedMessage,
                        "second hello".into(),
                    ));
                }
                if !is_compatible(&hello.protocol_version, PROTOCOL_VERSION) {
                    return Err(fail(
                        &mut output,
                        ErrorCode::VersionMismatch,
                        format!(
                            "simulator speaks {}, arcadiaos speaks {PROTOCOL_VERSION}",
                            hello.protocol_version
                        ),
                    ));
                }
                summary.run_id = Some(hello.run_id.clone());
                coordinator.start(hello);
                started = true;
                wire::send(
                    &mut output,
                    &CoordinatorToSim::Welcome(Welcome {
                        protocol_version: PROTOCOL_VERSION.into(),
                        coordinator: software_info(),
                    }),
                )?;
            }
            SimToCoordinator::Observation(obs) => {
                if !started {
                    return Err(fail(
                        &mut output,
                        ErrorCode::UnexpectedMessage,
                        "observation before hello".into(),
                    ));
                }
                let decision = coordinator.decide(&obs);
                wire::send(&mut output, &CoordinatorToSim::Decision(decision))?;
                summary.decisions += 1;
            }
            SimToCoordinator::End(end) => {
                summary.end_reason = Some(end.reason);
                return Ok(summary);
            }
            SimToCoordinator::Error(e) => return Err(RunError::Simulator(e)),
        }
    }
}
