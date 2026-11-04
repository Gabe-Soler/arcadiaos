//! End-to-end runs of the real `arcadiaos` binary under the ArcadiaBench
//! simulator.
//!
//! Set `ARCADIABENCH` to the path of an `arcadiabench` binary to run these;
//! without it they print a note and pass. They use the `nominal` variant
//! (hybrid power and the nominal fault model) and the `conjunction` variant
//! (a two-week Earth blackout early in the run). Run them in release so the
//! coordinator under test is the release build:
//!
//! ```sh
//! ARCADIABENCH=/path/to/arcadiabench cargo test --release --test bench
//! ```

use arcadia_protocol::{EndReason, EventKind, LogEntry, LogRecord, RejectReason, RunMetrics, wire};
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

/// The scenario variants the runs here use.
const VARIANTS: [&str; 2] = ["nominal", "conjunction"];

/// Run one seed of `variant` and return the simulator's one-line summary.
fn run(bench: &Path, variant: &str, seed: u64, log: &Path) -> String {
    let out = Command::new(bench)
        .args([
            "run",
            "--seed",
            &seed.to_string(),
            "--variant",
            variant,
            "--log",
        ])
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

/// The reason and metrics recorded in the log's `run_end` record, with the
/// reasons of every rejected command.
fn logged_end(log: &Path) -> (EndReason, RunMetrics, Vec<RejectReason>) {
    let mut reader = BufReader::new(File::open(log).expect("open log"));
    let mut last = None;
    let mut rejections = Vec::new();
    while let Some(record) = wire::recv::<_, LogRecord>(&mut reader).expect("valid log record") {
        if let LogEntry::Observed { event } = &record.entry
            && let EventKind::CommandRejected { reason, .. } = event.kind
        {
            rejections.push(reason);
        }
        last = Some(record);
    }
    match last.map(|r| r.entry) {
        Some(LogEntry::RunEnd { reason, metrics }) => (reason, metrics, rejections),
        other => panic!("log does not end with run_end: {other:?}"),
    }
}

/// Sols from a summary like `arcadiaos 1.0.0: GoalsComplete at sol 0.78 | ...`.
fn sols(summary: &str) -> Option<f64> {
    let rest = summary.split(" at sol ").nth(1)?;
    rest.split_whitespace().next()?.parse().ok()
}

#[test]
fn seeds_one_to_three_complete_the_pad_within_the_policy() {
    let Some(bench) = bench() else {
        return skip("seeds_one_to_three_complete_the_pad_within_the_policy");
    };
    for variant in VARIANTS {
        for seed in 1..=3 {
            let log = log_path(&format!("{variant}-seed{seed}"));
            let summary = run(&bench, variant, seed, &log);
            assert!(
                summary.contains("GoalsComplete"),
                "{variant} seed {seed}: {summary}"
            );
            let (reason, m, rejections) = logged_end(&log);
            assert_eq!(reason, EndReason::GoalsComplete, "{variant} seed {seed}");
            assert!(m.completed, "{variant} seed {seed}");
            assert_eq!(
                m.fleet.robots_lost, 0,
                "{variant} seed {seed}: no robot is given up"
            );
            assert_eq!(
                m.autonomy.policy_violations, 0,
                "{variant} seed {seed}: every level-2 action notified"
            );
            assert!(
                !rejections.iter().any(|r| matches!(
                    r,
                    RejectReason::ApprovalRequired | RejectReason::OutOfCoverage
                )),
                "{variant} seed {seed}: {rejections:?}"
            );
            assert!(
                m.autonomy.status_reports >= 1,
                "{variant} seed {seed}: Earth hears how it goes"
            );
            eprintln!(
                "seed {seed} ({variant}): {:.3} sols, {} faults, {:.1} robot-hours lost, {} rejections, {} messages to Earth ({} notifies, {} status reports) ({:?} sols in the summary)",
                m.sols,
                m.fleet.faults,
                m.fleet.robot_hours_lost,
                m.commands_rejected,
                m.autonomy.downlink_messages,
                m.autonomy.notifications,
                m.autonomy.status_reports,
                sols(&summary)
            );
            let _ = fs::remove_file(&log);
        }
    }
}

#[test]
fn same_seed_gives_identical_logs() {
    let Some(bench) = bench() else {
        return skip("same_seed_gives_identical_logs");
    };
    for variant in VARIANTS {
        let (a, b) = (
            log_path(&format!("det-a-{variant}")),
            log_path(&format!("det-b-{variant}")),
        );
        let first = run(&bench, variant, 1, &a);
        let second = run(&bench, variant, 1, &b);
        assert_eq!(first, second);
        let (la, lb) = (fs::read(&a).unwrap(), fs::read(&b).unwrap());
        assert!(!la.is_empty());
        assert!(la == lb, "{variant}: logs for the same seed differ");
        let _ = fs::remove_file(&a);
        let _ = fs::remove_file(&b);
    }
}

#[test]
fn summary_parsing() {
    assert_eq!(
        sols(
            "arcadiaos 1.0.0: GoalsComplete at sol 0.78 | pad 100% | 1833 decisions, 1837 commands"
        ),
        Some(0.78)
    );
    assert_eq!(sols("nothing here"), None);
}
