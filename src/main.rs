//! ArcadiaOS coordinator process: speaks arcadia-protocol on stdin/stdout.
//! Logs go to stderr.

use std::io::{self, BufReader};
use std::process::ExitCode;

fn main() -> ExitCode {
    let stdin = BufReader::new(io::stdin().lock());
    let stdout = io::stdout().lock();
    match arcadiaos::run(stdin, stdout) {
        Ok(summary) => {
            eprintln!(
                "arcadiaos: run {} ended ({:?}) after {} decisions",
                summary.run_id.as_deref().unwrap_or("?"),
                summary.end_reason,
                summary.decisions
            );
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("arcadiaos: {e}");
            ExitCode::FAILURE
        }
    }
}
