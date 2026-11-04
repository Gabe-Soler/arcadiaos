# ArcadiaOS

ArcadiaOS: an operating system for robot crews building on Mars.

ArcadiaOS decides which robot does what, when, and with how much power, while Earth supervises across a 4–24 minute delay and multi-week blackouts. It is measured on [ArcadiaBench](https://github.com/Gabe-Soler/ArcadiaBench), and it talks to the world only through [arcadia-protocol](https://github.com/Gabe-Soler/arcadia-protocol). That means the same coordinator can drive the simulator now and real hardware later.

**Version 1.0.0**, speaking arcadia-protocol 1.0.0. It builds the landing pad of every ArcadiaBench 1.0 scenario, with any of the benchmark's crews, and has the best headline score in all 18 ([results](#results)).

## What it does

ArcadiaOS turns Earth's landing-pad goal into work for the whole crew. It uses only what it is told in `hello` and in observations: never ground truth.

- **Plans** the pad as a per-cell task graph: survey, dig, fill, grade, compact, sinter, verify. It also plans the blast berm, keeps a cut-and-fill material ledger with stockpile sites, and borrows native regolith when spoil runs short. Defects found by verification scans go back for rework.
- **Allocates** work with a hybrid method. A small linear program splits each capability's remaining work across the robots that can do it (their roles), and a deterministic auction assigns ready tasks to free robots at every decision.
- **Schedules** in dependency order across pad sections, so every robot works at once. The bottleneck robot minimises its own travel while the rest keep the pipeline ahead of it.
- **Manages energy.**
  - No task is issued unless the battery covers it and the trip to a charger.
  - Robots charge to what their remaining work needs and choose the quickest station or mobile charger (allowing for queues). The mobile charger drives out to a robot when that saves its time.
  - Optional charging stays off the site grid when a night or storm would drain the bank, and takes surplus solar when the bank is full.
  - Idle robots park charged for cold nights.
- **Recovers from faults.**
  - Stuck, failed and offline robots are brought back by the cheapest permitted option: field repair, a tow to a depot for service, a tow to a station, a mobile charger, or parts from a redundant donor once Earth agrees.
  - Degraded robots get adjusted rate and energy estimates.
  - Wear is inferred from noisy health telemetry and usage, and preventive repairs or services are scheduled when they pay off, preferably at natural breaks.
- **Answers to Earth.** Every action goes through the authority policy in effect, updated whenever Earth sends a new one. Level-1 actions are taken, level-2 actions are taken and notified in the same decision, level-3 actions wait for Earth only as long as a cost model says waiting is worth it (and fall back by the same model when Earth cannot answer in time), and level-4 actions are never taken without Earth's approval. A status report goes to Earth about once a sol.
- **Keeps working through blackouts.** Nothing waits on an Earth it cannot reach: during a conjunction or an outage the fallback rule decides at once, notifies are stored and forwarded, and status reports catch up when the link returns.
- **Plans for coverage.** Robots out of range of the comms base station are never commanded, and their late events are folded in when they return. Work is kept inside coverage (scans entered from covered cells, berm material cast from covered borrow cells), and a relay robot is stationed when work beyond coverage makes it pay.
- **Executes** commands through their whole lifecycle (accepted, rejected, completed, failed), with back-off and retries, cancellation of interruptible jobs, and `wake_at`. It recovers from surprises by rescanning and reworking.

Robots are described only by their capabilities, so roles emerge from allocation and any crew works. See [docs/architecture.md](docs/architecture.md) for the design.

## Run it under ArcadiaBench

ArcadiaOS is a coordinator process that speaks newline-delimited JSON on stdin/stdout; a simulator launches it. With Rust (stable) installed and [ArcadiaBench](https://github.com/Gabe-Soler/ArcadiaBench) checked out beside this repository, from this repository's root:

```sh
cargo build --release
(cd ../ArcadiaBench && cargo build --release)

# One run: Small crew, nominal conditions, seed 1, with its event log.
../ArcadiaBench/target/release/arcadiabench run --seed 1 --log run.jsonl -- ./target/release/arcadiaos

# Ten seeds of the Medium crew through a conjunction blackout, scored.
../ArcadiaBench/target/release/arcadiabench bench --preset medium --variant conjunction --seeds 1-10 --out runs/medium-conjunction -- ./target/release/arcadiaos
```

Each run prints a one-line summary, and `bench` prints its scorecard. ArcadiaBench's README covers every preset, variant and command, its replay viewer opens any saved log, and its `scripts/results.sh` reproduces the full benchmark results with this binary (it looks for it at `../arcadiaos/target/release/arcadiaos`, or wherever `ARCADIAOS` says).

ArcadiaOS writes nothing to stderr by default. Set `ARCADIAOS_LOG=info` for plan summaries, the capacity plan, charging, faults, recoveries and maintenance (with each robot's estimated wear at the end), or `ARCADIAOS_LOG=debug` to see every command as well.

## Results

On ArcadiaBench 1.0 ([`results/v1.0/RESULTS.md`](https://github.com/Gabe-Soler/ArcadiaBench/blob/v1.0.0/results/v1.0/RESULTS.md)), every preset and variant over seeds 1–50, ArcadiaOS completes all 900 of its runs, with no policy violation and no robot lost, and has the best headline (success rate × speed) in all 18 scenarios, 10–21% above the greedy baseline's:

| Nominal headline | Small (4 robots) | Medium (10) | Large (20) |
|---|---|---|---|
| ArcadiaOS | **0.559** | **0.753** | **0.710** |
| greedy baseline | 0.501 | 0.681 | 0.585 |
| random baseline | 0.499 | 0.677 | 0.587 |
| first-come-first-served baseline | 0.385 | 0.419 | 0.362 |

- On the nominal build it finishes the pad 15% faster than greedy, averaged over the presets, and neither needs Earth there. Across the whole catalog it asked Earth twice in 900 runs, both to cannibalize a robot in Large `cascading_faults`.
- Its suite resilience is 0.975, against greedy's 0.967.
- Decisions take a fraction of a millisecond, and runs on the same seed are byte-identical.

ArcadiaBench's [demo replays](https://github.com/Gabe-Soler/ArcadiaBench/blob/v1.0.0/docs/demos.md) show ArcadiaOS through a nominal build, a dust storm, a conjunction blackout and cascading faults. [docs/architecture.md](docs/architecture.md#results) discusses where the time goes.

## Develop

```sh
cargo test     # unit tests, synthetic crews of 3–20 robots, and the real binary over stdio
cargo clippy --all-targets --all-features -- -D warnings
cargo fmt --check
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features

# End to end against ArcadiaBench (skipped unless ARCADIABENCH is set):
ARCADIABENCH=../ArcadiaBench/target/release/arcadiabench cargo test --release --test bench
```

CI runs the checks above. A separate job builds ArcadiaBench at the commit pinned in `BENCH_REF` and runs the end-to-end tests (the `nominal` and `conjunction` variants) against it.

[docs/architecture.md](docs/architecture.md) describes the design, and [CHANGELOG.md](CHANGELOG.md) what each release contains.

## License

MIT
