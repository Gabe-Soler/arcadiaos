# Changelog

All notable changes to ArcadiaOS. Versions follow semver.

## [1.0.0] - 2026-11-04

The first release: a coordinator that builds ArcadiaBench's landing pad with any crew, under faults, power limits, a delayed and intermittent Earth link, and Earth's authority policy.

### Added

- **Protocol.** Speaks arcadia-protocol 1.0.0, and decides from `hello` and observations alone, never ground truth.
- **Planning.** The pad as a per-cell task graph (survey, dig, fill, grade, compact, sinter, verify), swept section by section; a cut-and-fill material ledger with stockpiles, spoil casting and borrowing; the blast berm; verification batches, rework and a recheck loop that never stalls.
- **Allocation.** A hybrid method: a linear program, solved by an in-tree simplex, splits each capability's work into roles, and a deterministic sequential auction dispatches ready tasks at every decision. A robot may take the pad step that feeds one of its roles.
- **Scheduling.** Dependency order across sections so every robot works at once, help mode, interruptible jobs and anticipation.
- **Energy.** Reserves and affordability checks before every task, charge targets sized to the remaining work, the quickest station or mobile charger, rescue of stranded robots, a forecast of the site bank that keeps optional charging off a draining grid and takes surplus solar, and parking charged for cold nights.
- **Faults and recovery.** Field repair, a tow to a depot for service, charging in place, a mobile charger brought over, and cannibalizing a redundant donor once Earth agrees, chosen by cost; wear inferred from health telemetry and usage, and predictive maintenance when it pays.
- **Authority and Earth.** Every action under the policy in effect: notifies for level-2 actions, a cost model for the level-3 fallback rule, never a level-4 action without approval, status reports, policy updates, and work that carries on through outages and the conjunction blackout.
- **Coverage.** Work kept inside the base station's coverage, scans entered from covered cells, the berm built from covered borrow cells, relays stationed when they pay, and robots out of contact never commanded.
- **Executor.** The whole command lifecycle with back-off, retries, cancellation and `wake_at`.
- **Diagnostics.** Silent by default; `ARCADIAOS_LOG=info` or `debug` writes plans, charging, faults and commands to stderr.
- **Results.** On ArcadiaBench 1.0 it completes all 900 of its runs over the catalog with no policy violation and no robot lost, and has the best headline in all 18 scenarios.
- **Documentation.** `docs/architecture.md` and the README.
- **Tests and CI.** Unit tests, synthetic crews of 3 to 20 robots on a small simulator written from the protocol's semantics, robustness and stdio session tests, and an end-to-end job against ArcadiaBench.

## [0.1.0] - 2026-11-02

The project scaffold.

- A coordinator process that speaks arcadia-protocol 0.1.0 on stdin and stdout: the version handshake, the observation and decision loop, and `end`. Its planner was a placeholder that returned empty decisions.
- Tests that drive the real binary over stdio, playing the simulator.
- CI.
