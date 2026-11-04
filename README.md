# ArcadiaOS

ArcadiaOS: an operating system for robot crews building on Mars.

ArcadiaOS decides which robot does what, when, and with how much power, while Earth supervises across a 4–24 minute delay and multi-week blackouts. It is measured on [ArcadiaBench](https://github.com/Gabe-Soler/ArcadiaBench), and it talks to the world only through [arcadia-protocol](https://github.com/Gabe-Soler/arcadia-protocol). That means the same coordinator can drive the simulator now and real hardware later.

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
  - Stuck, failed and offline robots are brought back by the cheapest permitted option: field repair, a tow to a depot for service, a tow to a station, or a mobile charger.
  - Degraded robots get adjusted rate and energy estimates.
  - Wear is inferred from noisy health telemetry and usage, and preventive repairs or services are scheduled when they pay off, preferably at natural breaks.
- **Executes** commands through their whole lifecycle (accepted, rejected, completed, failed), with back-off and retries, cancellation of interruptible jobs, and `wake_at`. It recovers from surprises by rescanning and reworking.

Robots are described only by their capabilities, so roles emerge from allocation and any crew works. See [docs/architecture.md](docs/architecture.md) for the design.

## Status

**Week 5: fault response, predictive maintenance and energy-aware scheduling.** On the ArcadiaBench week-5 simulator (`landing-pad-small`, four robots, faults and wear on), every run tested builds and verifies the pad:

| Variant | Completed | Sols (mean, min–max) | Robot-hours lost (mean) |
|---|---|---|---|
| `nominal`, seeds 1–20 | 20 / 20 | 0.811, 0.791–0.881 | 0.23 |
| `cascading_faults`, seeds 1–20 | 20 / 20 | 0.875, 0.840–0.965 | 0.70 |
| `solar_only`, seeds 1–10 | 10 / 10 | 0.807, 0.792–0.873 | 0.21 |
| `nuclear_heavy`, seeds 1–10 | 10 / 10 | 0.806, 0.791–0.872 | 0.21 |

- No command is rejected and no robot is lost.
- Decisions take about 0.02 ms on average, and runs with the same seed produce byte-identical logs.
- The week 3–4 core managed 18 of 20 `nominal` and 3 of 20 `cascading_faults` seeds on the same simulator.

**Weeks 3–4: core coordinator.** On the week-2 simulator, seeds 1–20 finished in 0.774 sols on average (0.765–0.783), against about 1.6 sols for the naive scripted builder. On synthetic scenarios ArcadiaOS also handles crews of 3 to 20 robots with overlapping capability sets, several stations, small batteries, mobile charging and rescue, and a crew with no hauler.

Not yet: the authority policy, approvals, Earth messages and comms-blackout behaviour (week 6). Until then the policy is recorded but not enforced, except that recovery never uses an action above level 2 (so `cannibalize` waits for the approval flow), and nothing is sent to Earth.

## Run

ArcadiaOS is a coordinator process that speaks newline-delimited JSON on stdin/stdout. A simulator launches it, for example:

```sh
cargo build --release
arcadiabench run --seed 1 --variant nominal --log run.jsonl -- ./target/release/arcadiaos
```

It writes nothing to stderr by default. Set `ARCADIAOS_LOG=info` for plan summaries, the capacity plan, charging, faults, recoveries and maintenance (with each robot's estimated wear at the end), or `ARCADIAOS_LOG=debug` to see every command as well.

## Develop

```sh
cargo test     # unit tests, synthetic crews of 4–20 robots, and the real binary over stdio
cargo clippy --all-targets --all-features -- -D warnings
cargo fmt --check

# End to end against ArcadiaBench (skipped unless ARCADIABENCH is set):
ARCADIABENCH=/path/to/arcadiabench cargo test --release --test bench
```

CI runs the checks above. A separate job builds ArcadiaBench at the commit pinned in `BENCH_REF` and runs the end-to-end test (the `nominal` variant) against it.

## License

MIT
