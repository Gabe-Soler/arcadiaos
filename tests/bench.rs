//! End-to-end runs of the real `arcadiaos` binary under the ArcadiaBench
//! simulator.
//!
//! Set `ARCADIABENCH` to the path of an `arcadiabench` binary to run these;
//! without it they print a note and pass. Run them in release so the
//! coordinator under test is the release build:
//!
//! ```sh
//! ARCADIABENCH=/path/to/arcadiabench cargo test --release --test bench
//! ```

use arcadia_protocol::{EndReason, LogEntry, LogRecord, wire};
use std::fs::{self, File};
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::process::Command;

fn bench() -> Option<PathBuf> {
    let path = PathBuf::from(std::env::var_os("ARCADIABENCH")?);
    assert!(
        path.is_file(),
        "ARCADIABENCH points at {}, which is not a file",
        path.display()
    );
    Some(path)
}

fn skip(test: &str) {
    eprintln!("{test}: skipped (set ARCADIABENCH to an arcadiabench binary to run it)");
}

fn log_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join(format!("bench-{name}-{}.jsonl", std::process::id()))
}

/// Run one seed and return the simulator's one-line summary.
fn run(bench: &Path, seed: u64, log: &Path) -> String {
    let out = Command::new(bench)
        .args(["run", "--seed", &seed.to_string(), "--log"])
        .arg(log)
        .arg("--")
        .arg(env!("CARGO_BIN_EXE_arcadiaos"))
        .env_remove("ARCADIAOS_LOG")
        .output()
        .expect("launch arcadiabench");
    let stdout = String::from_utf8_lossy(&out.stdout).trim().to_string();
    assert!(
        out.status.success(),
        "seed {seed}: arcadiabench failed\nstdout: {stdout}\nstderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    stdout
}

/// The reason recorded in the log's `run_end` record.
fn logged_end(log: &Path) -> EndReason {
    let mut reader = BufReader::new(File::open(log).expect("open log"));
    let mut last = None;
    while let Some(record) = wire::recv::<_, LogRecord>(&mut reader).expect("valid log record") {
        last = Some(record);
    }
    match last.map(|r| r.entry) {
        Some(LogEntry::RunEnd { reason }) => reason,
        other => panic!("log does not end with run_end: {other:?}"),
    }
}

/// Sols from a summary like `arcadiaos 0.1.0: GoalsComplete at sol 0.78 | ...`.
fn sols(summary: &str) -> Option<f64> {
    let rest = summary.split(" at sol ").nth(1)?;
    rest.split_whitespace().next()?.parse().ok()
}

#[test]
fn seeds_one_to_three_complete_the_pad() {
    let Some(bench) = bench() else {
        return skip("seeds_one_to_three_complete_the_pad");
    };
    for seed in 1..=3 {
        let log = log_path(&format!("seed{seed}"));
        let summary = run(&bench, seed, &log);
        assert!(summary.contains("GoalsComplete"), "seed {seed}: {summary}");
        assert_eq!(logged_end(&log), EndReason::GoalsComplete, "seed {seed}");
        eprintln!("seed {seed}: {summary} ({:?} sols)", sols(&summary));
        let _ = fs::remove_file(&log);
    }
}

#[test]
fn same_seed_gives_identical_logs() {
    let Some(bench) = bench() else {
        return skip("same_seed_gives_identical_logs");
    };
    let (a, b) = (log_path("det-a"), log_path("det-b"));
    let first = run(&bench, 1, &a);
    let second = run(&bench, 1, &b);
    assert_eq!(first, second);
    let (la, lb) = (fs::read(&a).unwrap(), fs::read(&b).unwrap());
    assert!(!la.is_empty());
    assert!(la == lb, "logs for the same seed differ");
    let _ = fs::remove_file(&a);
    let _ = fs::remove_file(&b);
}

#[test]
fn summary_parsing() {
    assert_eq!(
        sols(
            "arcadiaos 0.1.0: GoalsComplete at sol 0.78 | pad 100% | 1833 decisions, 1837 commands"
        ),
        Some(0.78)
    );
    assert_eq!(sols("nothing here"), None);
}
