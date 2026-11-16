//! ArcadiaOS coordinator process: speaks arcadia-protocol on stdin/stdout.
//!
//! Silent on stderr unless `ARCADIAOS_LOG=info|debug` is set; a failed
//! session is always reported.
//!
//! `arcadiaos design-crew` is the one other thing the binary does: it
//! prints a crew for a scenario and a mass budget, and runs no session.

use arcadiaos::design;
use arcadiaos::diag::{Diag, Level};
use std::io::{self, BufReader, Read};
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().is_some_and(|a| a == "design-crew") {
        return design_crew(&args[1..]);
    }
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

/// `arcadiaos design-crew`: the crew file on standard output, and with
/// `--explain` the reasons on standard error. Exit code 2 for bad usage, 1
/// when no crew can be designed.
fn design_crew(args: &[String]) -> ExitCode {
    if args.iter().any(|a| a == "-h" || a == "--help") {
        println!("{}", design::USAGE);
        return ExitCode::SUCCESS;
    }
    let request = match design::parse(args) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("arcadiaos design-crew: {e}\n{}", design::USAGE);
            return ExitCode::from(2);
        }
    };
    let text = if request.scenario == "-" {
        let mut text = String::new();
        io::stdin().lock().read_to_string(&mut text).map(|_| text)
    } else {
        std::fs::read_to_string(&request.scenario)
    };
    let text = match text {
        Ok(t) => t,
        Err(e) => {
            eprintln!(
                "arcadiaos design-crew: cannot read the scenario {}: {e}",
                request.scenario
            );
            return ExitCode::FAILURE;
        }
    };
    match design::design(&text, &request.limits, &arcadiaos::Config::default()) {
        Ok((crew, why)) => {
            println!("{crew}");
            if request.explain {
                eprint!("{why}");
            }
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("arcadiaos design-crew: {e}");
            ExitCode::FAILURE
        }
    }
}
