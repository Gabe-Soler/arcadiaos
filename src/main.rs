//! ArcadiaOS coordinator process: speaks arcadia-protocol on stdin/stdout.
//!
//! Silent on stderr unless `ARCADIAOS_LOG=info|debug` is set; a failed
//! session is always reported.

use arcadiaos::diag::{Diag, Level};
use std::io::{self, BufReader};
use std::process::ExitCode;

fn main() -> ExitCode {
    let stdin = BufReader::new(io::stdin().lock());
    let stdout = io::stdout().lock();
    let diag = Diag::new(Level::from_env());
    let coordinator = arcadiaos::Coordinator::with_config(arcadiaos::Config::default(), diag);
    match arcadiaos::run_with(coordinator, stdin, stdout) {
        Ok(_) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("arcadiaos: {e}");
            ExitCode::FAILURE
        }
    }
}
