# ArcadiaOS

ArcadiaOS: an operating system for robot crews building on Mars.

ArcadiaOS decides which robot does what, when, and with how much power, while Earth supervises across a 4–24 minute delay and multi-week blackouts. It is measured on [ArcadiaBench](https://github.com/Gabe-Soler/ArcadiaBench), and it talks to the world only through [arcadia-protocol](https://github.com/Gabe-Soler/arcadia-protocol). That means the same coordinator can drive the simulator now and real hardware later.

## What it does

ArcadiaOS turns Earth's landing-pad goal into work for the whole crew. It uses only what it is told in `hello` and in observations: never ground truth.

- **Plans** the pad as a per-cell task graph: survey, dig, fill, grade, compact, sinter, verify. It also plans the blast berm, keeps a cut-and-fill material ledger with stockpile sites, and borrows native regolith when spoil runs short. Defects found by verification scans go back for rework.
- **Allocates** work with a hybrid method. A small linear program splits each capability's remaining work across the robots that can do it (their roles), and a deterministic auction assigns ready tasks to free robots at every decision.
- **Schedules** in dependency order across pad sections, so every robot works at once. The bottleneck robot minimises its own travel while the rest keep the pipeline ahead of it.
- **Manages energy.** No task is issued unless the battery covers it and the trip to a charger. Robots charge to what their remaining work needs, choose the quickest station or mobile charger (allowing for queues), and top up while idle.
- **Executes** commands through their whole lifecycle (accepted, rejected, completed, failed), with back-off and retries, cancellation of interruptible jobs, and `wake_at`. It recovers from surprises by rescanning and reworking.

Robots are described only by their capabilities, so roles emerge from allocation and any crew works. See [docs/architecture.md](docs/architecture.md) for the design.

## Status

**Weeks 3–4: core coordinator.** On the ArcadiaBench week-2 simulator (`landing-pad-small`, four robots), ArcadiaOS builds and verifies the pad on every seed tested (1–60):

- Seeds 1–20 finish in 0.774 sols on average (0.765–0.783), against about 1.6 sols for the naive scripted builder.
- No command is rejected and none fails.
- Decisions take well under a millisecond.
- Runs with the same seed produce byte-identical logs.

On synthetic scenarios it also handles crews of 3 to 20 robots with overlapping capability sets, several stations, small batteries, mobile charging and rescue, and a crew with no hauler.

Not yet:

- Fault response, repair, tow, service and predictive maintenance (week 5).
- The authority policy, approvals, Earth messages and comms-blackout behaviour (week 6). Until then the policy is recorded but not enforced, and nothing is sent to Earth.

## Run

ArcadiaOS is a coordinator process that speaks newline-delimited JSON on stdin/stdout. A simulator launches it, for example:

```sh
cargo build --release
arcadiabench run --seed 1 --log run.jsonl -- ./target/release/arcadiaos
```

It writes nothing to stderr by default. Set `ARCADIAOS_LOG=info` for plan summaries, the capacity plan, charging and recoveries, or `ARCADIAOS_LOG=debug` to see every command as well.

## Develop

```sh
cargo test     # unit tests, synthetic crews of 4–20 robots, and the real binary over stdio
cargo clippy --all-targets --all-features -- -D warnings
cargo fmt --check

# End to end against ArcadiaBench (skipped unless ARCADIABENCH is set):
ARCADIABENCH=/path/to/arcadiabench cargo test --release --test bench
```

CI runs the checks above, and a separate job builds ArcadiaBench at `BENCH_REF` (default `main`) and runs the end-to-end test against it.

## License

MIT
