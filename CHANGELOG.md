# Changelog

All notable changes to ArcadiaOS. Versions follow semver.

## [Unreleased]

Version 1.1.0, in development. It speaks arcadia-protocol 1.1.0, which does not work with protocol 1.0: it needs ArcadiaBench 1.1 or later. Support for ArcadiaBench 1.1's harder default catalog is being built.

### Added

- **Building on ArcadiaBench's catalog 1.1,** where a square metre takes hours and tens of kilowatt-hours to sinter and compaction takes most of the tolerance off a cell.
  - **Settlement.** Digging, filling and grading aim at the target raised by the compaction settlement the simulator states, so a cell settles onto the target.
  - **Measured before it is sintered.** Every scan of a cell whose surface has not moved is another sample of its height, and the estimate is their mean. A compacted cell is sintered only once its scans place it inside the tolerance and within the slope limit of every compacted neighbour, clear of the scans' noise; one too close to call is scanned again, and one that fails is regraded while that still costs minutes. Of two neighbours a step too far apart, the unsintered one gives way, or else the one further from the target.
  - **Wear from observation.** Wear is read from what a robot shows, not from assumed rates: wheel slip, motor current and dust coverage above the robot's own first readings, and for the tool the energy its work draws over the estimate. How fast it grows is what the robot, or the rest of the crew, has shown per hour since its last maintenance.
  - **Maintenance counted for as long as it pays.** Wear keeps growing, and later maintenance would take the same wear off in its turn, so a repair's benefit is counted over that time and not over all the work left. On a build of weeks the old count made every hour look like an emergency. Of a repair and a service, the one that returns more per second of downtime is chosen, and the fixed overhead charged to a stop falls from 900 s to 300 s.
  - **A decision when something happens.** While some robots work and others have nothing to do, ArcadiaOS no longer wakes every 15 minutes. It wakes when the first idle robot would be half-way to needing to park and charge, or after two hours at most, and otherwise on events. A build of 50 sols takes about 3,700 decisions where it took 7,400.
  - **Measuring only where it pays.** A crew that sinters a cell in a minute or two, as on catalog 1.0, sinters first and reworks afterwards as before.

### Changed

- **Protocol 1.1.0.** ArcadiaOS speaks arcadia-protocol 1.1.0.
  - **Constants from the simulator.** `hello` now carries the constants of the simulator's world (`scenario.constants`), and ArcadiaOS uses them in place of the values it assumed: the grade window, compaction settlement, minimum work time, rock density, the repair, tow, service and cannibalizing numbers, the heater's threshold and draw, the relay radio power, the lost-link timeout and Earth's response time. Without them it assumes protocol 1.0's reference constants, as before.
  - **Mobile chargers' tanks.** A charger's own reading of its tank (`tank_wh`) replaces ArcadiaOS's running estimate whenever the snapshot is current.
  - **Pass windows.** The Earth link's new cause `between_passes` and the end of the window in progress (`window_end_t`) are taken in. Nothing acts on them yet.
  - **Later minor versions.** Values this version does not know (a status, a surface, a reject reason, a fault severity, an uplink, an objective) are handled safely instead of being impossible: an unknown status is not commanded, an unknown surface is treated as unworked ground, and an unknown message is ignored.
- **Cold nights.** When heaters will run is still forecast as a window of the sol, but a temperature seen on the wrong side of the simulator's threshold now moves the window. On ArcadiaBench the assumed window already holds, so nothing changes there.
- **Integration tests.** They name ArcadiaBench's catalog 1.0, and CI runs them against ArcadiaBench's `main` at the commit that added catalogs and the fleet design track.

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
