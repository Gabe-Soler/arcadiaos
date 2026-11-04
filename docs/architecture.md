# ArcadiaOS architecture

ArcadiaOS is the coordinator: it turns Earth's goal into work for a crew of robots and keeps that work on track. This document describes the v1 core delivered in roadmap weeks 3–4 (world model, capability registry, planner, allocation, scheduling, energy policy and executor), the week-5 additions (fault response, predictive maintenance and energy-aware scheduling) and the week-6 additions: the authority layer, Earth messages, blackout behaviour and coverage-aware planning.

## Principles

- **Only the interface.** Every decision is built from `hello` and observations: the orbital prior, scan results, robot states and events. ArcadiaOS never reads ground truth, log files or simulator internals, and it does not depend on ArcadiaBench.
- **Crew-agnostic.** Robots are described only by their capabilities, rates, energy costs and capacities. Nothing is keyed to robot ids, positions or the site layout; roles emerge from allocation.
- **Derived, not stored, plans.** Each cell's next step is recomputed from the world model at every decision, so a surprise (a rejected command, an unexpected scan) replans automatically.
- **Deterministic.** The same inputs give the same commands. Collections are ordered (`BTreeMap`/`BTreeSet`), ties are broken by robot index and task key, and nothing depends on the wall clock.
- **Silent by default.** Diagnostics go to stderr only when `ARCADIAOS_LOG=info|debug` is set, and never influence a decision.

## The decision loop

`run(input, output)` handles the session: version handshake, then one decision per observation, then `end`. For each observation the [`Coordinator`](../src/coordinator.rs) does the following:

1. Takes in the robot states, then the events in order: command outcomes, scan results, faults, power, weather and comms reports, and Earth messages. The landing-pad goal arrives as an Earth message and creates the plan; approvals, vetoes, reprioritisation and policy updates take effect as they arrive. Health telemetry updates the wear estimates, power reports update the bank forecast, and each robot's performance factors are refreshed.
2. Works out the covered part of the site, and refreshes the capacity plan when the remaining work, the set of working robots, or a robot's faults have changed.
3. Derives the ready tasks from the plan.
4. Recovers disabled robots: service or charge in place, field repair, tow, a mobile charger brought over, or parts from a donor (asking Earth first; see [Authority](#authority-and-earth)).
5. Starts maintenance that is pressing enough not to wait for a natural break.
6. Stations a relay, or stands one down, where coverage needs it.
7. Sends robots below their energy low-water mark to charge (or brings a mobile charger to them).
8. Runs the auction over free robots, plus robots whose current job may be interrupted.
9. Gives leftover robots maintenance at this natural break, a charge (when work waits on energy), an opportunistic top-up, a parking charge for the night, or a move towards upcoming work.
10. Reports to Earth: a `notify` for the decision's level-2 commands, approval requests, and a status report when one is due.
11. Requests a `wake_at` while anything waits on time: a retry back-off ending, an idle or disabled robot, an approval deadline, or nothing in flight.

## Modules

| Module | Responsibility |
|---|---|
| `world` | World model: per-cell site knowledge (`world::site`), the crew (`world::fleet`), stations, depots, the base station, power, weather, the Earth link and the policy in force. |
| `registry` | Capability registry: who can do what, at what rate and energy per unit, haul and charger capacities, and the travel and work cost models built on them. |
| `planner` | Goal → task graph: sections and sweep order, per-cell readiness, berm, verification batching, rework and the recheck loop. `planner::material` keeps the cut/fill ledger: stockpile siting, spoil casting, haul sources and borrow. |
| `tasks` | Task and stage types shared by the planner, allocator and executor. |
| `capacity` + `lp` | Capacity plan: a small linear program that splits each capability's work across the robots able to do it, solved by an in-tree two-phase simplex. |
| `allocator` | Deterministic sequential auction: bids, scores, pruning and greedy award with conflict checks. |
| `energy` | Reserves, affordability, charge targets, station queues, mobile chargers and rescue. |
| `power` | The site bank forecast (fission, half-sine solar scaled by measured dust transmittance, essential load, scheduled charging) and the assumed cold-night window. |
| `health` | Smoothed health telemetry, usage since the last maintenance, wear estimates, performance factors and the predictive-maintenance rule. |
| `recovery` | The next recovery step for a stuck, failed or offline robot, costed and gated by the authority layer. |
| `authority` | The level of every action under the policy in effect, approval requests and Earth's answers, the level-3 fallback rule, notifies and status reports. |
| `coverage` | Which cells the base station and holding relays cover; scan areas entered from a covered cell. |
| `executor` | Command lifecycle: deterministic ids, accepted, completed, failed or rejected, back-off, cancellation, and which robots in-flight commands engage. |
| `coordinator` | One decision per observation, tying the rest together. |
| `config` | Every assumption and tuning knob in one place. |
| `diag` | Optional stderr diagnostics. |

## World model

Each cell's knowledge starts from the orbital prior (elevation only, with noise) and is replaced by scan results: elevation, surface state, boulders and density. Between scans it is carried forward by modelling every completed command:

- Dig cuts to the target and clears the cell.
- Grade sets the target elevation.
- Compaction settles the cell slightly.
- Sinter stabilises it.
- Dumps raise a cell by mass ÷ (density × area), and loads lower it.

Each cell also records the time of its last known change and the issue time of the latest scan that covered it afterwards. That is exactly what verification needs: a scan counts only if it started after the cell's last change. A command that fails or is refused for a reason that suggests the model is wrong marks the cell *stale*, and the cell is rescanned before more work is planned on it.

## Capability registry

The registry indexes robots by capability. A capability is usable when its rate is positive, and for hauling and charging also its capacity; a `missing_capability` rejection disables it for that robot. The registry provides the cost models used everywhere else:

- **Travel:** Euclidean distance × cell size ÷ speed. Its energy is distance × drive energy plus idle draw on the way.
- **Work:** units ÷ rate, never less than the per-step floor. Its energy is units × energy per unit plus idle draw.
- **Towing:** the tower's travel, at a reduced speed and with a higher drive energy.

Each robot also carries **performance factors** (speed, drive energy, work rate and energy, scan rate, energy overall). Every cost model applies them, so estimates, reserves, affordability, charge targets and the capacity plan all follow a robot's actual condition. They come from the health model: a degraded fault applies its penalty at once (a wheel fault slows driving, a battery fault raises all energy use, and so on), and completed work teaches a residual calibration. That calibration is the ratio of actual to estimated energy and duration, a geometric moving average that is never below 1. Its first 10% is left to the safety factors estimates already carry; only the excess is applied.

## Planning model

### Task graph

Each pad cell moves through **survey → dig → fill (only when low) → grade → compact → sinter → verify** on its own; dig and fill are independent of each other but both come before grading. The planner reads a cell's needs from its knowledge:

| Cell knowledge | Next step |
|---|---|
| Not scanned, or stale | Survey (its section is rescanned) |
| Natural, stockpile or berm surface | Dig; also fill if lower than target − 0.10 m |
| Cleared, boulders or higher than target + 0.10 m | Dig (re-cut) |
| Cleared, lower than target − 0.10 m | Fill up to target − 0.05 m |
| Cleared, within the window | Grade |
| Graded | Compact |
| Compacted | Sinter (or finished when the pad needs no stabilising) |
| Stabilised and within tolerance | Finished, then verified by a scan |
| Prepared but out of tolerance, or marked for rework | Regrade, re-cut or refill, depending on how far off it is |

The thresholds rest on one interface assumption: grading levels a cell only within 0.15 m of the target. ArcadiaOS keeps a margin inside that window. If a grade is still refused, it tightens that cell's window and brings the cell much closer to the target first, so a narrower simulator window is handled too.

### Sections and sweep order

The pad is swept boustrophedon-style, starting from the corner nearest the crew, along the longer side, so consecutive cells are neighbours. It is cut into sections: bands of whole lanes, about 40 cells each. Sections are the unit of surveying, and of spreading robots that share a capability, so one section can be surveyed while another is compacted. A cell's sweep rank orders the pipeline: upstream work on early cells is needed first.

### Material balance

- **Stockpiles** are cells in a ring just outside the pad, never berm, borrow, station or depot cells, inside the base station's coverage when there are enough such cells, each allowed to rise a limited height.
- **Dig spoil** fills the digger's own payload first. The overflow goes to the destination chosen for that dig:
  - a berm cell within a short reach that still needs material;
  - otherwise a fill cell within reach whose need matches the expected overflow. The cell is rescanned afterwards, because the amount is an estimate;
  - otherwise the nearest stockpile with room, leaning towards the berm.

  A crew without any hauler casts spoil into fill and berm cells at any distance, which is the only way it can move material.
- **Hauls** take material from the cheapest pile: travel there plus travel on, with a small penalty for borrowing. Stockpiled spoil is taken first. When spoil runs short, native regolith is borrowed from pile sites to a limited depth.
- **Ledger.** It records spoil cast, spoil reused and regolith borrowed, so the plan's cut/fill balance is visible at `info` level. Fill aims for just inside the grade window rather than the exact target, because grading finishes the job.

### Berm

Berm cells are surveyed first, to learn their base elevation and density. Hauls and spoil then raise them to the specified height plus a margin, sized with a density safety factor; a berm cell outside the base station's coverage is fed instead by digs at its borrow cell, inside coverage, casting their spoil onto it (see [Coverage](#coverage)). A scan after the last deposit confirms each cell; a short cell gets more material.

### Verification, rework and recheck

- **Verification batches.** Finished cells are verified in batches: a scan of the bounding box of the finished but unverified cells, once enough have accumulated. In the end game (the last few cells) every finished cell is scanned right away, and idle scanners wait at the work front, so the final verification follows the last sinter within seconds.
- **Rework.** A scan showing a finished cell out of tolerance sends it back for rework.
- **Recheck.** If every cell looks finished and verified but the run continues, something is hidden: a slope just over the limit, or a berm cell a little short. The planner then requests a whole-site recheck scan. It reworks the most suspicious cells (furthest from the target, or the steepest step) and tops up the weakest berm cells. Each round always schedules something, so the run never stalls.

## Allocation method

The outline left the allocation method open: auction-based, MILP, or a hybrid. ArcadiaOS uses a **hybrid**: a linear program for strategic capacity planning, and a deterministic auction for tactical dispatch.

### Capacity plan (linear program)

For every capability `k` with remaining work and every working robot `r` able to do it, let `τ[r,k]` be the time `r` would need to do all of `k`: work, driving between steps, and the charging the energy implies. The plan solves:

```text
minimise   T + ε · Σ τ[r,k] f[r,k]
subject to Σ_r f[r,k] ≥ 1          for every capability k
           Σ_k τ[r,k] f[r,k] ≤ T   for every robot r
           f ≥ 0
```

`f[r,k]` is robot `r`'s share of capability `k`, and `T` is the makespan if the work were perfectly divisible. The small second term picks, among makespan-optimal splits, the one using the least robot time, so each robot stays on what it does best. In the default crew, the excavator could also haul, but the dedicated hauler keeps up alone and is faster, so hauling is not one of the excavator's roles. Shares become **roles**. The plan is re-solved when the fleet changes, when a capability's work runs out, and every time the remaining work drains by 15%. It is small (one variable per robot-capability pair), so it is solved exactly by an in-tree dense simplex with Bland's rule. That needs no solver dependency, and the result is deterministic.

### Dispatch (sequential auction)

At every decision each free robot bids on the ready tasks it can do. A bid is the robot's concrete plan for the task: command parameters, haul source, travel, duration and energy. It is scored in seconds:

```text
score = travel time
      + seq_weight × (1 − u) × sweep rank
      + crowd_penalty × same-capability robots already working the section
      + help_penalty if the capability is not one of the robot's roles
```

- `u` is the robot's utilisation in the capacity plan. The bottleneck (u ≈ 1) mostly minimises its own travel, which matters most for slow robots doing long work. Robots with slack follow the sweep so the bottleneck finds contiguous ready cells ahead of it.
- Bids the battery cannot afford are dropped.
- Each robot first ranks its tasks by a cheap lower bound (direct travel plus the same priority terms) and fully plans only the best few, which keeps decisions fast for large crews.
- Bids are then awarded greedily, lowest score first. Ties go to the lower robot index and then the lower task key. Robots, tasks and cells already taken are skipped, and only one ground-changing command runs on a cell at a time.

### Why this method

- **A pure auction has no global view of roles.** Without the plan, a multi-capability robot is tempted by work another robot does better, starving its own speciality. The linear program supplies that view cheaply and explainably.
- **A pure MILP is the wrong tool for dispatch.** The interesting decisions are made one command at a time, as events arrive, with surprises. A MILP over the whole schedule would have to be re-solved at nearly every decision, could not meet a per-decision time budget for 20 robots and hundreds of tasks, and needs a solver. The good MILP solvers need C++ toolchains, and the pure-Rust options pull in a dozen crates.
- **Hungarian assignment** solves one round optimally, but there are usually one or two free robots per decision, so a greedy auction matches it in practice. The auction also handles conflicts (one command per cell) and per-robot plans naturally.
- **The auction is CBAA-style.** Consensus-based bundle auctions are designed to run distributed, so robots or sub-crews out of contact could bid locally with the same scoring.

## Scheduling

Dependencies are enforced by construction: a task is ready only when its predecessors are done. Fill and dig run in either order, grading waits for both, and so on.

- **Priority** comes from the sweep rank, scaled by each robot's slack. Surveys come first, then the pad, then berm work, which moves up once the pad is nearly done.
- **Parallelism** comes from per-cell tasks: robots of different capabilities work in different places at once, and robots of the same capability spread across sections through the crowding penalty.
- **Help mode:** a robot takes work outside its roles only when none of its own is available.
- **Interruptible jobs:**
  - Anticipatory moves are cut short as soon as real work appears.
  - Opportunistic charges are cut short only once enough work is waiting, which avoids flip-flopping between the plug and the pad.
- **Anticipation.** Idle robots whose roles work the pad surface move towards the first unfinished cell in sweep order, so they are on site when the work arrives.

## Energy policy

The simulator draws a step's energy when the step starts, and a step the battery cannot cover drains the robot and leaves it offline. So:

- **Reserve.** A robot keeps enough charge to drive from wherever a task ends to the nearest station (with a safety factor), plus a margin for telemetry noise: 2.5% of capacity, at least 25 Wh.
- **Affordability.** A task is issued only if the battery covers its energy estimate (with a safety factor) and that reserve. A robot below its low-water mark charges before anything else.
- **Charge target.** A robot charges to what its remaining share of the work needs, from the current workload and the capacity plan, plus the drive back and the reserve. It never charges less than a worthwhile top-up, and never more than full, so the last charge of the run is not wasted.
- **Source choice.** Every station is considered, with the expected queue from robots already charging there and the bank's delivery fraction, alongside every free mobile charger with enough in its tank. The quickest option, return trip included, wins.
- **Opportunistic charging.** A robot with nothing to do that will need more energy anyway tops up now.
- **Mobile chargers and rescue.** A mobile charger serving a robot is reserved until the session ends. A stranded robot that can reach no charger gets the nearest free mobile charger driven over to it. An idle charger whose tank has fallen below half goes to a station to fill its battery and then its tank.

## Faults and recovery

Faults arrive as `robot_fault` events and as each snapshot's `faults`. A robot whose status is stuck, failed, offline or being towed is out of the capacity plan, and its task is released for others (a `fault` or `stuck` failure does not back the task off). A lost robot is gone for good: never commanded, never a recovery target. Recovery runs first at every decision, most-needed robot first (robots whose capabilities still have work come before the rest). [`recovery`](../src/recovery.rs) derives the *next step* from the robot's current condition, so a surprise on the way simply replans:

| Condition | Steps considered |
|---|---|
| On a depot cell, stuck or failed | `service` in place (accepted for a disabled robot there) |
| Offline at a station, or beside an idle mobile charger | `charge` in place |
| Stuck | field `repair`; `tow` to a firm cell nearby (a compacted or stabilised one if known), or to a station if the battery is also empty |
| Failed, field repairable | field `repair`; `tow` to a depot, then `service` |
| Failed, not field repairable | `tow` to a depot, then `service`; `cannibalize` a donor only where the policy allows it |
| Offline | bring a mobile charger over; `tow` to a station |

Each candidate is costed in seconds until the robot is back:

- the helper's drive (and a charge first, if its battery cannot cover the step and the reserve);
- the repair work (from the robot's field-repairable faults, being stuck and its estimated wear) or the tow;
- queueing at the depot or station;
- the service or charge that follows.

The cheapest permitted option wins; ties go to the lower helper index. Helpers are free robots or robots on interruptible jobs. A helper that cannot afford the step is sent to charge for it. While a mobile charger is on its way, the robot waits for it. Engaged robots (being repaired or towed, or a charger in use) are never offered other commands.

**Authority.** Every option names the policy action it uses, and the authority layer gates it: levels 1 and 2 are taken (`tow`, level 2 in the default policy, gets its `notify`), higher levels only with an approval in hand. A donor for cannibalizing must be disabled and redundant: every capability it has, another working robot also has. When parts from a donor would beat every allowed option, ArcadiaOS asks Earth and holds the robot for the answer; see [Authority and Earth](#authority-and-earth).

## Authority and Earth

[`authority`](../src/authority.rs) keeps the books; the coordinator routes every action through it.

**The policy in effect** is the scenario's, replaced by each `policy_update` the moment it arrives. Every action has a level under it:

| Level | What ArcadiaOS does |
|---|---|
| 1 | Takes the action. |
| 2 | Takes it, and sends a `notify` listing the command ids in the same decision: one notify per action per decision, so none is ever late and the downlink carries only what policy asks for. |
| 3 | Asks with an `approval_request` carrying a `decide_by_t` and a `fallback` from the level-3 rule below, then acts on `approve`, or, once the deadline has passed with no answer, on the fallback. The command cites the request as its `approval_id`. |
| 4 | Asks, and acts only on `approve`. Never autonomously, even in a blackout. |

Approvals are scoped to their purpose: an approval to strip one donor for one robot is used only for that, although the simulator would accept it for any `cannibalize`.

**Robot commands.** In the default policy every command is level 1 except `tow` (2) and `cannibalize` (3). If a policy puts a routine command (driving, scanning, digging, charging, …) at level 3 or 4, ArcadiaOS stops issuing it and asks Earth for a standing approval for the rest of that policy version, with low stakes (proceed on the fallback at level 3).

**Coordinator-level actions** cannot be enforced by the simulator, but go through the same levels:

- `replan`: re-solving the capacity plan because the set of working robots changed (a robot disabled, back, or lost). Notified at level 2; at levels 3 and 4 the old plan stands until Earth agrees.
- `reassign_task`: a task released by a robot that was disabled during it, offered to the others. Notified at level 2; at levels 3 and 4 the task is held for its robot until Earth agrees.
- `commit_energy_reserve`: station charging that draws on the bottom quarter of the site bank (`bank_floor_frac`) while the bank is that low. At level 2 it is notified once per episode; at levels 3 and 4 such charging waits for Earth's approval (charging the current supply can cover still goes ahead).
- `abandon_section`: ArcadiaOS never abandons a section.

### The level-3 fallback rule

The outline names this the core research problem: weighing the cost of waiting against the cost of being wrong. ArcadiaOS prices both in seconds of crew time:

```text
expected = time until the link is up + 2 × one-way delay + Earth's response time
wait     = min(margin × expected, max_wait_s)  if expected ≤ max_wait_s,  else 0
net      = benefit − wait × wait_cost
ask        only if net > 0
fallback = proceed if net > risk, else abort
```

- **Waiting is worth it only if an answer can come in time.** The expected answer time uses the latest `comms` report: the one-way delay, and when the link is down, `expected_restore_t` if the site knows it (a conjunction), else never (an outage). If Earth cannot answer within the policy's `max_wait_s`, ArcadiaOS does not wait pointlessly: the request goes out with `decide_by_t` now, and the fallback decides at the next decision.
- **A request that changes nothing is not sent**: no time to wait and fallback abort.
- **For cannibalizing**, `benefit` is the time the robot is back sooner than by the best allowed option (towing to a depot and servicing, a field repair, …; a sol when nothing else can bring it back), `wait_cost` is the robot's weight (its capacity-plan utilisation, at least 0.2: a bottleneck's hour costs the crew an hour), and `risk` is `cannibalize_risk_s`, the expected cost of stripping a robot Earth would rather keep. At level 4 only an approval counts, so ArcadiaOS asks only when an answer can come within `max_wait_s`, and otherwise takes the best allowed option.
- **Routine actions** at level 3 have no risk to speak of, so their fallback is proceed.

| Parameter | Value | Meaning |
|---|---|---|
| `earth_response_s` | 1200 s | Mission control's typical response time (interface assumption). |
| `answer_margin` | 1.5 | Waiting stretches to 1.5 × the expected answer time, within `max_wait_s`. |
| `cannibalize_risk_s` | 21,600 s | A robot lost for good: six hours of crew time. |
| `no_alternative_s` | 88,775 s | What cannibalizing gains when nothing else can bring the robot back: a sol. |
| `routine_benefit_s` | 14,400 s | What a standing approval of a routine action is worth. |
| `assumed_delay_s` | 720 s | One-way delay before the first `comms` report. |

At the default delay of 720 s an answer is expected in 2640 s, within the default policy's hour, so ArcadiaOS waits up to the hour. In the `conjunction` variant the delay is near 1440 s, an answer would take over an hour, and the rule decides at once even while the link is up.

### Earth messages, reports and blackouts

- **`goal`** creates the plan; **`policy_update`** replaces the policy in effect when it arrives (the reference simulator's trust script loosens `tow`, `replan` and `reassign_task` to level 1 after two incident-free sols, and tightens `cannibalize` to level 4 after a robot is lost); **`approve`** and **`veto`** settle requests, and after a veto ArcadiaOS takes the best allowed option instead; **`reprioritize`** is recorded (there is one goal, so nothing else changes).
- **Status reports** go out with the plan at the start, about once a sol (`status_period_s`), and after major events (a robot lost, a policy update), with the pad's progress and the state of the crew.
- **Blackouts and outages** do not stall the crew: nothing in the default policy waits on Earth, level-3 requests go straight to the fallback rule, level-2 notifies are sent as usual (the link stores and forwards them), and periodic status reports are held while the link is down and caught up with one report when it returns.

## Coverage

[`coverage`](../src/coverage.rs) works out which cells can hear the site: within the base station's range, or within the range of a relay ArcadiaOS has holding at its position that is itself covered, transitively. Each robot's snapshot says whether the site can hear it right now.

- **Robots out of contact are never commanded.** Their snapshots are stale (`telemetry_t` is when contact was lost), so they are neither free, nor helpers, nor recovery targets; their jobs stay in flight, and their events, delivered late with their original times, settle those jobs when they return. A rejection `out_of_coverage` backs the robot off.
- **Work is kept inside coverage.**
  - A scan whose entry cell (the area's cell nearest the robot) is out of coverage is grown a few rows or columns toward coverage, so the robot scans from a covered cell.
  - Stockpile sites are chosen inside the base station's coverage when there are enough such cells, so loading never takes a hauler out of contact.
  - A berm cell outside coverage is built from a **borrow cell**: the nearest covered cell within side-cast reach (not pad, berm or infrastructure). Digs there cast their spoil straight onto the berm cell, deep enough to cover its need even with an empty payload, down to `berm_borrow_max_depth_m`; a dug-out borrow cell hands the berm back to hauling.
- **Work that must leave coverage is costed.** A bid ending out of coverage carries the lost-link wait (`lost_link_timeout_s`, after which the simulator drives an idle robot back) in its duration and its score, so covered work wins.
- **Relays.** When ready work (or a robot out of contact well past its expected return, `stranded_margin_s`) needs coverage the base station cannot give, ArcadiaOS stations a relay-capable free robot at the covered cell that serves the most of it with the shortest drive, if the robot time saved is at least `relay_benefit_ratio` times what the relay robot gives up (its utilisation times the time it holds). It stands the relay down when nothing needs it, or when the relay robot runs low. The relay draws `relay_radio_w` on top of idle power.

| Parameter | Value | Meaning |
|---|---|---|
| `lost_link_timeout_s` | 1800 s | When the simulator drives an idle robot out of coverage back (interface assumption). |
| `relay_radio_w` | 25 W | A holding relay's radio draw (interface assumption). |
| `berm_borrow_max_depth_m` | 1.5 m | Deepest a borrow cell is dug. |
| `relay_benefit_ratio` | 1.5 | Saving needed per second a relay costs. |
| `stranded_margin_s` | 1800 s | Out of contact this long past its expected return: stranded. |

On ArcadiaBench's `landing-pad-small` the base station covers the pad but not the berm. Week 5's coordinator built the berm by hauling onto it and scanning it from its own row, so the scout, hauler and excavator spent about 18 robot-hours a run out of contact waiting for lost-link returns. Now the berm is built from the row of borrow cells just south of it, and berm scans are entered from that row: no robot leaves coverage, and the relay is never needed.

## Health and predictive maintenance

[`health`](../src/health.rs) infers wear, which is never reported:

- **Telemetry.** Wheel slip, motor current and dust coverage are smoothed per robot. Each has an exponentially weighted level, plus a trend per hour fitted between anchors at least ten minutes apart. Wheel wear follows from slip and drive-motor wear from motor current, net of a known fault's offset. Joint wear follows from dust coverage, net of the sensors' share.
- **Usage.** Tool, sensor and battery wear come from usage since the robot was last as good as new: hours of work, exposed hours and kWh drawn. They are counted at nominal rates scaled by the site's *observed* wear rate, which is the ratio of measured wheel and drive-motor wear to what the same driving gives at nominal rates, pooled over the crew. A harsh site shows within the first kilometres; `cascading_faults` reads about ×3.5.

A field repair cuts the estimate by the repair fraction; a depot service resets it.

**The rule.** Failures follow an assumed hazard of `k · w³` per component per exposed hour, with `k` scaled by the observed wear rate. For a robot with `H` planned hours of work left, maintenance pays when

```text
benefit = Δ(expected faults over the next 4 h) × fault cost × weight
        + degraded-fault slowdown removed × H × weight
        + wear slowdown removed × H × weight
benefit ≥ ratio × (downtime + 900 s overhead)   and   benefit ≥ 600 s
```

- `weight` is the robot's utilisation in the capacity plan, at least 0.2. A bottleneck's lost hour costs the whole crew an hour.
- The wear slowdown is the extra time the estimated wear costs the robot's mix of driving, work and scanning (each effect assumed linear in wear), plus a third of the extra energy for the charging it costs.
- Risk is projected only four hours ahead, because maintenance can always be done later.
- `ratio` is 1 at a **natural break** and 2 when the stop interrupts work. A natural break is a robot with nothing to do, blocked on energy, or about to charge.
- A field repair halves wear and clears degraded faults. A depot service resets wear but costs hours, so it wins only when no repairer exists or wear is extreme.

Late in the run `H` is small, so maintenance stops by itself. The fixed overhead models what the simulator does not charge: stopping work, waiting for the repairer, and its trip back. It keeps preventive repairs to a realistic cadence. In `cascading_faults` that is about five per run, every one to two hours once a bottleneck's summed wear reaches about 0.3–0.5. In `nominal` it is fewer than one per run, always at a natural break.

## Energy: nights and the site grid

- **Bank forecast.** [`power`](../src/power.rs) steps the site bank forward from the latest `power` report. It uses the scenario's power system (fission, plus solar on the half-sine clear-sky curve scaled by the dust transmittance last measured in daylight), the essential load, and the station charging ArcadiaOS has scheduled.
- **Optional charging waits.** Opportunistic top-ups, tank refills and parking charges are held back while the forecast says they would push the bank below a quarter of its capacity before the next sunrise. Charging that work needs always proceeds; an empty bank only slows it, and the energy policy plans for that.
- **Surplus windows.** While the bank is nearly full and the sun is up, idle robots top up: energy that would otherwise be curtailed.
- **Station slots.** Optional charges leave a slot free for working robots. A robot that needs a slot for work cuts short an optional session holding the last one.
- **Nights.** Robot heaters run on cold nights and add to idle drain. The cold window is an assumption, deliberately wider than the reference simulator's. A robot with no work left keeps enough charge to idle for twelve hours with the heater, and still reach a charger; below that it parks charged at a station. Decisions recur at least every 15 minutes while any robot is idle or disabled, so low-water marks are checked through the night.
- **Mobile charger to the field.** When a robot needs a charge and a free mobile charger driving over costs less than the robot's own best trip and return (counting the charger's time at its utilisation), the charger comes to it. Slow robots far from a busy or weak station benefit most.
- **Heavy work.** Charge targets cover each robot's remaining share of the work, so energy-hungry work such as sintering runs between full charges without topping up mid-sequence.

## Executor

Commands get deterministic ids (`os-000001`, …). The executor tracks each one from issue through `command_accepted` to `command_completed`, `command_failed` or `command_rejected`, and hands the outcome back with the job it belonged to.

| Outcome | Response |
|---|---|
| Completed | Apply the modelled effect to the world model and the material ledger. |
| Rejected `invalid_target` / failed `blocked` | Mark the cell stale (rescan before more work); back off that task on that robot. Refused grades tighten the cell's window. A haul that found nothing to load stops counting on that pile. |
| Rejected `missing_capability` | Disable the capability for that robot and re-plan roles. |
| Rejected `busy`, `robot_disabled`, `out_of_coverage` | Back off the robot briefly. |
| Rejected `approval_required`, `vetoed` | Back off the task; the authority layer means this does not happen. |
| Rejected `unknown_robot` | Drop the robot from planning. |
| Failed `insufficient_energy` | Back off; the energy policy prevents this in practice. |
| Failed `cancelled` | Expected after a cancellation; nothing to do. |
| Failed `fault`, `stuck` | Release the task without backing it off; recovery takes over the robot. |
| Failed `target_lost`, `out_of_coverage` | Back off and replan. |
| Repair, tow, service completed | Cut or reset the health estimate; the robot's next recovery step (if any) follows. |
| Repair, tow, service failed or refused | Back off the helper briefly; recovery replans. |
| Three rejections in a row | Rest the robot briefly, whatever the reasons. |

ArcadiaOS never issues a command that would complete instantly, such as a move to the robot's own cell or a charge to its current level. A guard stops issuing and waits after many decisions at one sim time, and `wake_at` covers back-offs and idle periods. Together these mean the run always either makes progress or waits for a wake-up.

## Diagnostics

`ARCADIAOS_LOG=info` writes plan summaries, the capacity plan with each robot's roles, charges, failures, rejections and recheck rounds. `debug` adds every command, completion and cancellation. Lines are stamped with sim time and sol.

## Testing

- **Unit tests next to each module:** geometry, site knowledge, registry and performance factors, simplex, capacity plan, planner (including berm borrow cells), material ledger, energy, allocator (including covered scan entries and costed stranding), executor, health (smoothing, wear, calibration, the maintenance rule), power (forecast, deferral, surplus, cold window), recovery (every step, donors and the authority hooks), authority (the level-3 rule, approvals, fallbacks and vetoes, notifies, status reports) and coverage (transitive relays, scan areas).
- **Coordinator tests:** tow to the depot then service in place, charge in place at a station, a mobile charger for an offline robot, repair of a stuck and failed robot, parking charged before the night, a mobile charger driving to a slow robot, lost robots never commanded, a level-2 tow notified in the same decision, a policy update applied on arrival, cannibalizing asked about and done on approval (and not on a veto), the fallback deciding at once in a blackout (and nothing done at level 4), robots out of contact never commanded, and a relay stationed for work beyond coverage.
- **`tests/crews.rs`:** synthetic scenarios on a small deterministic simulator written from the interface semantics. It covers crews of 3, 4, 5, 10 and 20 robots, generalists, mixed and overlapping capability sets, several stations, fifth-size batteries and a crew with no hauler. A mobile charger beats a weak, distant station, and a robot stranded too far from any charger is rescued by one. It also checks determinism and the per-decision time.
- **`tests/robustness.rs`:** hostile rejections, malformed but valid goals, unknown robots and commands, no stations and immobile robots.
- **`tests/session.rs`:** the real binary over stdio. It covers the handshake, version errors, planning once the goal arrives, back-off after a rejection, and silence by default.
- **`tests/bench.rs`:** the real binary under ArcadiaBench (`nominal` and `conjunction` variants), when `ARCADIABENCH` is set. Seeds 1–3 of each must finish `goals_complete` with no robot lost, no policy violation, no `approval_required` or `out_of_coverage` rejection and at least one status report (metrics are read from `run_end`), and the same seed twice must give byte-identical logs.

## Results (ArcadiaBench week 6, `landing-pad-small`)

Measured against ArcadiaBench `7c41a92` in release, from each log's `run_end.metrics`:

| Variant | Completed | Sols (mean, min–max) | Policy violations | Notifies | Status reports | Approval requests | Robot time out of coverage |
|---|---|---|---|---|---|---|---|
| `nominal`, seeds 1–20 | 20 / 20 | 0.806, 0.788–0.880 | 0 | 14 | 20 | 0 | 0 s |
| `conjunction`, seeds 1–20 | 20 / 20 | 0.806, 0.788–0.880 | 0 | 14 | 20 | 0 | 0 s |
| `cascading_faults`, seeds 1–20 | 20 / 20 | 0.876, 0.856–0.944 | 0 | 71 | 20 | 0 | 3 s |
| `solar_only`, seeds 1–10 | 10 / 10 | 0.799, 0.792–0.807 | 0 | 4 | 10 | 0 | 0 s |
| `nuclear_heavy`, seeds 1–10 | 10 / 10 | 0.799, 0.788–0.808 | 0 | 4 | 10 | 0 | 0 s |

- No command was rejected (no `approval_required`, `vetoed` or `out_of_coverage`), no robot was lost, and runs are byte-identical across repeats.
- Notifies report every `tow`, `replan` and `reassign_task`; each run fits in under a sol, so it sends one status report.
- No run cannibalized: in the four-robot crew no robot is redundant, so there is never a donor. The approval flow is covered by the coordinator tests.
- The 3 s is one excavator drive between two borrow cells that clips a cell just outside coverage.
- The week-5 coordinator, run against the same simulator, finished `nominal` and `conjunction` seeds 1–10 in 0.802 sols on average with one violation each (a `tow` with no notify), its robots out of contact for hours a run. Week 6 takes 0.799 on both.

Decisions take about 0.03 ms on average and about 0.1 ms at most.

## Results (ArcadiaBench week 5, `landing-pad-small`)

Measured against ArcadiaBench `6f4e1b2` in release, from each log's `run_end.metrics`:

| Variant | Completed | Sols (mean, min–max) | Faults per run | Robot-hours lost (mean, max) | Rejections |
|---|---|---|---|---|---|
| `nominal`, seeds 1–20 | 20 / 20 | 0.811, 0.791–0.881 | 0.3 | 0.23, 2.09 | 0 |
| `cascading_faults`, seeds 1–20 | 20 / 20 | 0.875, 0.840–0.965 | 1.4 | 0.70, 2.87 | 0 |
| `solar_only`, seeds 1–10 | 10 / 10 | 0.807, 0.792–0.873 | 0.3 | 0.21, 2.05 | 0 |
| `nuclear_heavy`, seeds 1–10 | 10 / 10 | 0.806, 0.791–0.872 | 0.3 | 0.21, 2.05 | 0 |

No robot was lost, the grid never ran short, and the lowest bank level was 44% of capacity (in `solar_only`). Decisions take about 0.02 ms on average and under 0.15 ms at most. Runs are byte-identical across repeats, faulty seeds included.

The week 3–4 core, run against the same simulator, completed 18 of 20 `nominal` seeds and 3 of 20 `cascading_faults` seeds. In `nominal` seeds 2 and 13, a fault that needed the depot left the compactor-sinterer failed until it drained. In `cascading_faults`, stuck and failed robots were never freed. Now a disabling fault costs the tow (a few minutes) plus a two-hour service, and seeds without faults run as fast as before.

Week 5 also fixed a planner bug that only longer runs expose. After a failed berm check, the extra material requested was never used up by deposits, so the crew hauled to the berm for ever and the confirming scan never came.

## Results (ArcadiaBench week 2, `landing-pad-small`)

| | Seeds 1–20 |
|---|---|
| Completed | 20 / 20 (also 40 / 40 on seeds 21–60) |
| Sols | mean 0.774, min 0.765, max 0.783 (scripted builder ≈ 1.6) |
| Decisions | 1,759–1,859 per run |
| Commands | 1,759–1,860 per run; 0 rejected, 0 failed |
| Decision time | about 0.02 ms on average and under 0.1 ms at most, in release |

The run is bound by the single compactor-sinterer, which is busy essentially all the time: about 11.2 h sintering, 2 h compacting and 6 h charging, trips to the station included. The theoretical floor for this crew is about 0.76 sol.

## Extension points

### After week 6

- **Approvals.** New level-3 actions plug into `State::weigh_cannibalize` (its stakes) or `State::ask_earth` (generic), and are cited from `Authority::authorization`.
- **Coverage.** Relay placement is a heuristic over covered cells; a crew with several relays, or sites mostly out of coverage, could plan relay chains.
- **Distributed allocation.** The auction is CBAA-style, so sub-crews out of contact could bid locally with the same scoring.

## Tuning

All assumptions and knobs live in `Config` (`src/config.rs`). They cover:

- the grade window and fill and cut triggers, side-cast reach, berm margin and stockpile height;
- section size and verification batch;
- energy margins and factors, and the auction weights;
- back-off times;
- tow, service and repair assumptions;
- the health model (telemetry coefficients, nominal wear rates, the hazard, wear and degraded-fault effects);
- maintenance thresholds;
- the cold window, parking horizon and bank thresholds;
- Earth and authority: the response time, the answer margin, the risk and benefit prices of the level-3 rule, and the status period;
- coverage: the lost-link timeout, relay radio power, borrow depth, relay benefit ratio and stranding margin.

The health, repair, Earth response, lost-link and relay numbers are interface assumptions taken from the reference simulator's documentation. ArcadiaOS uses them to rank options, and corrects energy and time estimates from what it measures. None is specific to a seed, a crew or a site.
