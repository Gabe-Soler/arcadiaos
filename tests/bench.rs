//! End-to-end runs of the real `arcadiaos` binary under the ArcadiaBench
//! simulator.
//!
//! Set `ARCADIABENCH` to the path of an `arcadiabench` binary to run these;
//! without it they print a note and pass. The binary must speak this
//! coordinator's protocol version: ArcadiaBench 1.1 or later. Most use
//! ArcadiaBench's scenario catalog 1.0, named explicitly, with the `nominal`
//! variant (hybrid power and the nominal fault model) and the `conjunction`
//! variant (a two-week Earth blackout early in the run). One builds a pad on
//! each outpost of catalog 1.1, where a build takes weeks, and one designs
//! a crew for the fleet design track and builds with it. Run them in
//! release so the coordinator under test is the release build:
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

/// The ArcadiaBench scenario catalog the runs here are on. Without it
/// ArcadiaBench uses its newest catalog.
const CATALOG: &str = "1.0";

/// Run one seed of `variant` and return the simulator's one-line summary.
fn run(bench: &Path, variant: &str, seed: u64, log: &Path) -> String {
    run_on(bench, CATALOG, "small", variant, seed, log)
}

/// Run one seed of a scenario of `catalog` and return the simulator's
/// one-line summary.
fn run_on(
    bench: &Path,
    catalog: &str,
    preset: &str,
    variant: &str,
    seed: u64,
    log: &Path,
) -> String {
    let out = Command::new(bench)
        .args([
            "run",
            "--catalog",
            catalog,
            "--preset",
            preset,
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

/// Sols from a summary like
/// `arcadiaos 1.1.0 [small/nominal]: GoalsComplete after 0.78 sols | ...`.
fn sols(summary: &str) -> Option<f64> {
    let rest = summary.split(" after ").nth(1)?;
    let (number, unit) = rest.split_once(' ')?;
    unit.starts_with("sols").then(|| number.parse().ok())?
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

/// Catalog 1.1: a square metre takes seven hours and 37 kWh to sinter,
/// compaction takes 4 cm off a cell, heights are scanned with a centimetre
/// of error, wear and faults build up over weeks, and Earth is reachable in
/// short passes. One build on each outpost, held to finishing sooner than
/// the mean of ArcadiaBench's greedy baseline over seeds 1 to 50 (54.7,
/// 27.7 and 18.1 sols).
#[test]
fn a_build_of_weeks_completes_on_every_outpost_of_catalog_1_1() {
    let Some(bench) = bench() else {
        return skip("a_build_of_weeks_completes_on_every_outpost_of_catalog_1_1");
    };
    for (preset, within_sols) in [("small", 54.0), ("medium", 27.0), ("large", 18.0)] {
        let log = log_path(&format!("weeks-{preset}"));
        let summary = run_on(&bench, "1.1", preset, "nominal", 1, &log);
        assert!(summary.contains("GoalsComplete"), "{preset}: {summary}");
        let (reason, m, rejections) = logged_end(&log);
        assert_eq!(reason, EndReason::GoalsComplete, "{preset}");
        assert!(m.completed, "{preset}");
        assert!(
            m.sols < within_sols,
            "{preset}: {:.1} sols, the greedy baseline's mean is {within_sols}",
            m.sols
        );
        assert_eq!(m.fleet.robots_lost, 0, "{preset}: no robot is given up");
        assert_eq!(m.autonomy.policy_violations, 0, "{preset}");
        assert!(
            !rejections.iter().any(|r| matches!(
                r,
                RejectReason::ApprovalRequired | RejectReason::OutOfCoverage
            )),
            "{preset}: {rejections:?}"
        );
        // Decisions follow events: a few thousand over weeks, not one every
        // quarter of an hour.
        assert!(m.decisions < 5000, "{preset}: {} decisions", m.decisions);
        eprintln!(
            "catalog 1.1 {preset}: {:.1} sols, {} decisions, {} faults, {} requests to Earth",
            m.sols, m.decisions, m.fleet.faults, m.autonomy.approval_requests
        );
        let _ = fs::remove_file(&log);
    }
}

/// The fleet design track: ArcadiaOS designs a crew for catalog 1.1's
/// outpost from what `arcadiabench scenario` prints, ArcadiaBench accepts
/// it, and ArcadiaOS builds the pad with it.
#[test]
fn a_crew_of_its_own_design_enters_the_fleet_design_track_and_builds() {
    let Some(bench) = bench() else {
        return skip("a_crew_of_its_own_design_enters_the_fleet_design_track_and_builds");
    };
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("fleet-{}", std::process::id()));
    fs::create_dir_all(&dir).expect("create a directory");
    let text = |out: std::process::Output, what: &str| -> String {
        assert!(
            out.status.success(),
            "{what} failed: {}",
            String::from_utf8_lossy(&out.stderr)
        );
        String::from_utf8(out.stdout).expect("text")
    };
    // What a coordinator is told about the outpost every crew works from,
    // and the goal: two lines.
    let scenario = dir.join("scenario.jsonl");
    let told = Command::new(&bench)
        .args([
            "scenario",
            "--catalog",
            "1.1",
            "--preset",
            "medium",
            "--seed",
            "1",
            "--goal",
        ])
        .output()
        .expect("launch arcadiabench");
    fs::write(&scenario, text(told, "arcadiabench scenario")).expect("write the scenario");
    // Catalog 1.1's mass budget is 10 t.
    let designed = Command::new(env!("CARGO_BIN_EXE_arcadiaos"))
        .args([
            "design-crew",
            "--budget-kg",
            "10000",
            "--explain",
            "--scenario",
        ])
        .arg(&scenario)
        .output()
        .expect("launch arcadiaos");
    let why = String::from_utf8_lossy(&designed.stderr).to_string();
    let crew = text(designed, "arcadiaos design-crew");
    assert!(why.contains("<- chosen"), "{why}");
    let count = |design: &str| -> u32 {
        let rest = crew.split(&format!("\"{design}\": ")).nth(1).expect(design);
        let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
        digits.parse().expect("a count")
    };
    let counts = ["scout", "excavator", "hauler", "compactor_sinterer"].map(count);
    eprintln!("designed crew: {} {why}", crew.trim());
    // Something to scan and repair, to dig, to charge a robot in the
    // field, and to sinter with: as many sinterers as the plant can feed.
    assert!(counts.iter().all(|&n| n >= 1), "{crew}");
    assert!(counts[3] >= 8, "{crew}");
    // ArcadiaBench lets it enter, and refuses it with two more sinterers.
    let crew_file = dir.join("crew.json");
    fs::write(&crew_file, &crew).expect("write the crew");
    let checked = Command::new(&bench)
        .arg("crew")
        .arg(&crew_file)
        .output()
        .expect("launch");
    let verdict = text(checked, "arcadiabench crew");
    assert!(
        verdict.contains("of the 10000 kg catalog 1.1 allows"),
        "{verdict}"
    );
    let heavy_file = dir.join("heavy.json");
    let heavy = crew.replace(
        &format!("\"compactor_sinterer\": {}", counts[3]),
        &format!("\"compactor_sinterer\": {}", counts[3] + 2),
    );
    fs::write(&heavy_file, heavy).expect("write the crew");
    let refused = Command::new(&bench)
        .arg("crew")
        .arg(&heavy_file)
        .output()
        .expect("launch");
    assert!(!refused.status.success());
    assert!(String::from_utf8_lossy(&refused.stderr).contains("mass budget is 10000 kg"));
    // And it builds the pad, sooner than the Medium preset's crew does on
    // the same outpost (24.6 sols on average).
    let log = dir.join("run.jsonl");
    let ran = Command::new(&bench)
        .args(["run", "--seed", "1", "--crew"])
        .arg(&crew_file)
        .arg("--log")
        .arg(&log)
        .arg("--")
        .arg(env!("CARGO_BIN_EXE_arcadiaos"))
        .env_remove("ARCADIAOS_LOG")
        .output()
        .expect("launch arcadiabench");
    let summary = text(ran, "arcadiabench run --crew");
    assert!(summary.contains("GoalsComplete"), "{summary}");
    let (reason, m, _) = logged_end(&log);
    assert_eq!(reason, EndReason::GoalsComplete);
    assert!(m.completed && m.sols < 20.0, "{:.1} sols", m.sols);
    assert_eq!((m.fleet.robots_lost, m.autonomy.policy_violations), (0, 0));
    eprintln!("its build: {:.1} sols", m.sols);
    let _ = fs::remove_dir_all(&dir);
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
            "arcadiaos 1.1.0 [small/nominal]: GoalsComplete after 0.78 sols, speed 0.562 | pad 100% | 1833 decisions, 1837 commands"
        ),
        Some(0.78)
    );
    assert_eq!(sols("nothing here"), None);
    assert_eq!(sols("ended after 3 tries"), None);
}
