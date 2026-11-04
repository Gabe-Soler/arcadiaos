//! ArcadiaOS: the coordinator for robot crews building on Mars.
//!
//! [`run`] implements the protocol session (handshake, observation → decision
//! loop, end) around a [`Coordinator`], which turns the landing-pad goal into
//! work for the whole crew:
//!
//! - [`world`]: the world model, built only from `hello` and observations;
//! - [`registry`]: the capability registry (who can do what, how fast, at what cost);
//! - [`planner`]: goal → per-cell task graph, material balance, berm, verification and rework;
//! - [`capacity`] and [`lp`]: the capacity plan, a small linear program for each robot's roles;
//! - [`allocator`]: a deterministic sequential auction that assigns ready tasks;
//! - [`energy`]: reserves, charging decisions, station contention and the mobile charger;
//! - [`power`]: the site bank forecast and cold nights;
//! - [`health`]: wear estimates from telemetry and usage, performance factors and the
//!   predictive-maintenance rule;
//! - [`recovery`]: getting stuck, failed and offline robots back to work;
//! - [`executor`]: the command lifecycle, retries and cancellation;
//! - [`coordinator`]: one decision per observation, tying the rest together.
//!
//! Decisions are deterministic: the same inputs always give the same commands.

pub mod allocator;
pub mod capacity;
pub mod config;
pub mod coordinator;
pub mod diag;
pub mod energy;
pub mod executor;
pub mod geom;
pub mod health;
pub mod lp;
pub mod planner;
pub mod power;
pub mod recovery;
pub mod registry;
pub mod tasks;
pub mod world;

pub use config::Config;
pub use coordinator::{Coordinator, Summary};

use arcadia_protocol::{
    CoordinatorToSim, EndReason, ErrorCode, ErrorMessage, PROTOCOL_VERSION, SimToCoordinator,
    SoftwareInfo, Welcome, is_compatible, wire,
};
use std::fmt;
use std::io::{BufRead, Write};

/// Name and version reported in `welcome`.
pub fn software_info() -> SoftwareInfo {
    SoftwareInfo {
        name: "arcadiaos".into(),
        version: env!("CARGO_PKG_VERSION").into(),
    }
}

/// What happened in a session.
#[derive(Debug, Default, PartialEq)]
pub struct RunSummary {
    /// Run id from `hello`.
    pub run_id: Option<String>,
    /// Decisions sent.
    pub decisions: u64,
    /// Why the simulator ended the run, if it did.
    pub end_reason: Option<EndReason>,
}

/// Why a session failed.
#[derive(Debug)]
pub enum RunError {
    /// Reading or writing the stream failed.
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
pub fn run<R: BufRead, W: Write>(input: R, output: W) -> Result<RunSummary, RunError> {
    run_with(Coordinator::new(), input, output)
}

/// [`run`] with a preconfigured coordinator.
pub fn run_with<R: BufRead, W: Write>(
    mut coordinator: Coordinator,
    mut input: R,
    mut output: W,
) -> Result<RunSummary, RunError> {
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
                coordinator.finish(&end);
                summary.end_reason = Some(end.reason);
                return Ok(summary);
            }
            SimToCoordinator::Error(e) => return Err(RunError::Simulator(e)),
        }
    }
}
