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
  - Stuck, failed and offline robots are brought back by the cheapest permitted option: field repair, a tow to a depot for service, a tow to a station, a mobile charger, or parts from a redundant donor once Earth agrees.
  - Degraded robots get adjusted rate and energy estimates.
  - Wear is inferred from noisy health telemetry and usage, and preventive repairs or services are scheduled when they pay off, preferably at natural breaks.
- **Answers to Earth.** Every action goes through the authority policy in effect, updated whenever Earth sends a new one. Level-1 actions are taken, level-2 actions are taken and notified in the same decision, level-3 actions wait for Earth only as long as a cost model says waiting is worth it (and fall back by the same model when Earth cannot answer in time), and level-4 actions are never taken without Earth's approval. A status report goes to Earth about once a sol.
- **Keeps working through blackouts.** Nothing waits on an Earth it cannot reach: during a conjunction or an outage the fallback rule decides at once, notifies are stored and forwarded, and status reports catch up when the link returns.
- **Plans for coverage.** Robots out of range of the comms base station are never commanded, and their late events are folded in when they return. Work is kept inside coverage (scans entered from covered cells, berm material cast from covered borrow cells), and a relay robot is stationed when work beyond coverage makes it pay.
- **Executes** commands through their whole lifecycle (accepted, rejected, completed, failed), with back-off and retries, cancellation of interruptible jobs, and `wake_at`. It recovers from surprises by rescanning and reworking.

Robots are described only by their capabilities, so roles emerge from allocation and any crew works. See [docs/architecture.md](docs/architecture.md) for the design.

## Status

**Week 6: authority, Earth in the loop, blackouts and coverage.** On the ArcadiaBench week-6 simulator (`landing-pad-small`, four robots; faults and wear, the Earth link with light-time delay and outages, authority enforcement and site coverage), every run tested builds and verifies the pad within the policy:

| Variant | Completed | Sols (mean, min–max) | Policy violations | Messages to Earth (notifies, status reports) |
|---|---|---|---|---|
| `nominal`, seeds 1–20 | 20 / 20 | 0.806, 0.788–0.880 | 0 | 34 (14, 20) |
| `conjunction`, seeds 1–20 | 20 / 20 | 0.806, 0.788–0.880 | 0 | 34 (14, 20) |
| `cascading_faults`, seeds 1–20 | 20 / 20 | 0.876, 0.856–0.944 | 0 | 91 (71, 20) |
| `solar_only`, seeds 1–10 | 10 / 10 | 0.799, 0.792–0.807 | 0 | 14 (4, 10) |
| `nuclear_heavy`, seeds 1–10 | 10 / 10 | 0.799, 0.788–0.808 | 0 | 14 (4, 10) |

- No command is rejected (no `approval_required`, `vetoed` or `out_of_coverage`), no robot is lost, and runs are byte-identical across repeats.
- No robot leaves the base station's coverage (bar one 3 s clip in 20 `cascading_faults` runs), where week 5's coordinator lost hours of robot time a run to lost-link returns.
- No run needs an approval: the default policy asks for none of what this crew does, and no robot in it is redundant enough to be a donor.

**Week 5: fault response, predictive maintenance and energy-aware scheduling.** On the ArcadiaBench week-5 simulator, every run tested built and verified the pad:

| Variant | Completed | Sols (mean, min–max) | Robot-hours lost (mean) |
|---|---|---|---|
| `nominal`, seeds 1–20 | 20 / 20 | 0.811, 0.791–0.881 | 0.23 |
| `cascading_faults`, seeds 1–20 | 20 / 20 | 0.875, 0.840–0.965 | 0.70 |
| `solar_only`, seeds 1–10 | 10 / 10 | 0.807, 0.792–0.873 | 0.21 |
| `nuclear_heavy`, seeds 1–10 | 10 / 10 | 0.806, 0.791–0.872 | 0.21 |

- No command was rejected and no robot was lost.
- Decisions took about 0.02 ms on average, and runs with the same seed produced byte-identical logs.
- The week 3–4 core managed 18 of 20 `nominal` and 3 of 20 `cascading_faults` seeds on the same simulator.

**Weeks 3–4: core coordinator.** On the week-2 simulator, seeds 1–20 finished in 0.774 sols on average (0.765–0.783), against about 1.6 sols for the naive scripted builder. On synthetic scenarios ArcadiaOS also handles crews of 3 to 20 robots with overlapping capability sets, several stations, small batteries, mobile charging and rescue, and a crew with no hauler.

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

CI runs the checks above. A separate job builds ArcadiaBench at the commit pinned in `BENCH_REF` and runs the end-to-end tests (the `nominal` and `conjunction` variants) against it.

## License

MIT
