//! Allocation: the tactical half of the method, a deterministic sequential
//! auction (CBAA-style greedy assignment, one task per robot per round).
//!
//! Every free robot bids on every ready task it can do. A bid is the robot's
//! concrete plan for the task (command parameters, travel, duration and
//! energy) and a score in seconds:
//!
//! ```text
//! score = travel time
//!       + seq_weight × (1 − u) × sweep rank  (keep the pipeline in order)
//!       + crowd_penalty × same-capability robots already in the section
//!       + help_penalty if the capability is not one of the robot's roles
//!       + lost-link wait if the work leaves the robot out of coverage
//! ```
//!
//! Scans are entered from a covered cell where growing the scan area a
//! little makes that possible.
//!
//! where `u` is the robot's utilisation in the capacity plan: the bottleneck
//! (u ≈ 1) mostly minimises its own travel, while robots with slack follow the
//! sweep so the bottleneck finds contiguous ready cells.
//!
//! Bids the robot cannot afford with its battery (task energy plus the trip
//! to a charger) are dropped. Bids are then awarded greedily, lowest score
//! first, with ties broken by robot index and task key, skipping robots,
//! tasks and cells already taken. The result is fully deterministic.

use crate::capacity::CapacityPlan;
use crate::config::Config;
use crate::coverage::{self, Coverage};
use crate::energy::Energy;
use crate::geom;
use crate::planner::{Busy, Planner, SpoilKind, SpoilTarget};
use crate::registry::Registry;
use crate::tasks::{Stage, Task, TaskKey};
use crate::world::WorldModel;
use arcadia_protocol::{Action, CapabilityName, GridCoord};
use std::collections::{BTreeMap, BTreeSet};

/// Haul details fixed when the haul is planned.
#[derive(Debug, Clone, PartialEq)]
pub struct HaulPlan {
    /// Loading cell.
    pub from: GridCoord,
    /// Dumping cell.
    pub to: GridCoord,
    /// `mass_kg` of the command.
    pub mass_kg: f64,
    /// Pile loaded from, if any.
    pub pile: Option<usize>,
    /// Berm cell index when delivering to the berm.
    pub berm: Option<usize>,
    /// Payload already on board, kg.
    pub payload_kg: f64,
    /// Expected mass loaded, kg.
    pub load_kg: f64,
    /// Expected mass dumped, kg.
    pub dump_kg: f64,
}

/// Dig details fixed when the dig is planned.
#[derive(Debug, Clone, PartialEq)]
pub struct DigPlan {
    /// Where overflow spoil goes.
    pub spoil: Option<SpoilTarget>,
    /// Expected spoil, kg.
    pub spoil_kg: f64,
    /// Free payload capacity before digging, kg.
    pub room_kg: f64,
}

impl DigPlan {
    /// Spoil expected to overflow the payload, kg.
    pub fn overflow_kg(&self) -> f64 {
        (self.spoil_kg - self.room_kg).max(0.0)
    }
}

/// A robot's concrete plan for a task.
#[derive(Debug, Clone, PartialEq)]
pub struct WorkPlan {
    /// The command action.
    pub action: Action,
    /// Where the robot ends up.
    pub end: GridCoord,
    /// Driving time, s.
    pub travel_s: f64,
    /// Expected total duration, s.
    pub duration_s: f64,
    /// Expected energy, Wh.
    pub energy_wh: f64,
    /// Haul details.
    pub haul: Option<HaulPlan>,
    /// Dig details.
    pub dig: Option<DigPlan>,
    /// The robot ends out of coverage: unreachable until its lost-link return.
    pub strands: bool,
}

/// Why a robot cannot bid on a task.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoBid {
    /// It cannot do this task (capability, payload, geometry).
    Unable,
    /// It could, but its battery cannot cover the task and the trip home.
    Unaffordable,
}

/// Everything a bid needs to read.
#[derive(Debug, Clone, Copy)]
pub struct Context<'a> {
    /// World model.
    pub world: &'a WorldModel,
    /// Capability registry.
    pub reg: &'a Registry,
    /// Pad plan.
    pub planner: &'a Planner,
    /// Capacity plan (roles).
    pub capacity: &'a CapacityPlan,
    /// Tuning.
    pub cfg: &'a Config,
    /// Work already in flight.
    pub busy: &'a Busy,
    /// Robots per (capability, section) already working there.
    pub crowd: &'a BTreeMap<(CapabilityName, usize), usize>,
    /// The covered part of the site.
    pub coverage: &'a Coverage,
}

/// Plan task `t` for robot `r`. Dig spoil destinations are chosen later, by
/// [`finish_dig`], once the round's awards are known.
pub fn plan_work(ctx: &Context, r: usize, t: &Task) -> Result<WorkPlan, NoBid> {
    let reg = ctx.reg;
    let cfg = ctx.cfg;
    let cap = t.capability();
    let entry = ctx.world.fleet.get(r);
    let pos = entry.position().ok_or(NoBid::Unable)?;
    if !reg.can(r, cap) {
        return Err(NoBid::Unable);
    }
    let site = &ctx.world.site;
    let mut plan = match t.stage {
        Stage::Survey | Stage::Verify | Stage::BermCheck | Stage::Recheck => {
            // Enter from a covered cell if a slightly larger scan allows it.
            let area = coverage::safe_scan_area(
                &t.area,
                pos,
                ctx.coverage,
                site.width,
                site.height,
                SCAN_GROW_CELLS,
            )
            .unwrap_or(t.area);
            let amount = if area == t.area {
                t.amount
            } else {
                geom::cell_count(&area) as f64 * site.cell_area_m2()
            };
            let at = geom::clamp_into(pos, &area);
            let travel_s = reg.travel_s(r, pos, at);
            let work_s = reg.work_s(r, cap, amount, cfg.min_work_s);
            WorkPlan {
                action: Action::Scan { area },
                end: at,
                travel_s,
                duration_s: travel_s + work_s,
                energy_wh: reg.travel_wh(r, pos, at) + reg.work_wh(r, cap, amount, cfg.min_work_s),
                haul: None,
                dig: None,
                strands: false,
            }
        }
        Stage::Dig | Stage::Grade | Stage::Compact | Stage::Sinter => {
            let c = t.cell();
            let travel_s = reg.travel_s(r, pos, c);
            let work_s = reg.work_s(r, cap, t.amount, cfg.min_work_s);
            let area = geom::unit(c);
            let (action, dig) = match t.stage {
                Stage::Dig => {
                    let room = (reg.haul_capacity_kg(r) - entry.payload_kg()).max(0.0);
                    let spoil = ctx.world.site.dig_spoil_kg(c, t.elevation_m, cfg);
                    (
                        Action::Dig {
                            area,
                            target_elevation_m: t.elevation_m,
                            spoil_to: None,
                        },
                        Some(DigPlan {
                            spoil: None,
                            spoil_kg: spoil,
                            room_kg: room,
                        }),
                    )
                }
                Stage::Grade => (
                    Action::Grade {
                        area,
                        target_elevation_m: t.elevation_m,
                    },
                    None,
                ),
                Stage::Compact => (Action::Compact { area }, None),
                _ => (Action::Sinter { area }, None),
            };
            WorkPlan {
                action,
                end: c,
                travel_s,
                duration_s: travel_s + work_s,
                energy_wh: reg.travel_wh(r, pos, c) + reg.work_wh(r, cap, t.amount, cfg.min_work_s),
                haul: None,
                dig,
                strands: false,
            }
        }
        Stage::Fill | Stage::Berm => plan_haul(ctx, r, t, pos)?,
    };
    if !plan.duration_s.is_finite() {
        return Err(NoBid::Unable);
    }
    // Ending out of coverage leaves the robot unreachable until it drives
    // back on its own after the lost-link timeout.
    if !ctx.coverage.covers(plan.end) {
        plan.strands = true;
        plan.duration_s += cfg.lost_link_timeout_s;
    }
    if !Energy::affordable(ctx.world, reg, r, plan.energy_wh, plan.end, cfg) {
        return Err(NoBid::Unaffordable);
    }
    Ok(plan)
}

fn plan_haul(ctx: &Context, r: usize, t: &Task, pos: GridCoord) -> Result<WorkPlan, NoBid> {
    let reg = ctx.reg;
    let capacity = reg.haul_capacity_kg(r);
    let rate = reg.rate(r, CapabilityName::Haul);
    let payload = ctx.world.fleet.get(r).payload_kg();
    let dest = t.cell();
    if capacity <= 0.0 || rate <= 0.0 || payload > t.max_kg {
        return Err(NoBid::Unable);
    }
    // Everything on board is dumped, so deliver at least the payload.
    let mass = t.amount.min(capacity).max(payload);
    if mass < ctx.cfg.min_haul_kg {
        return Err(NoBid::Unable);
    }
    let load = (mass - payload).max(0.0);
    let (from, pile) = if load <= 1e-6 {
        (pos, None)
    } else {
        let src = ctx
            .planner
            .material
            .source(reg, r, pos, dest, load)
            .ok_or(NoBid::Unable)?;
        (src.at, Some(src.pile))
    };
    let e_unit = reg.energy_per_unit(r, CapabilityName::Haul);
    let handle_s = (load + mass) / rate;
    let travel_s = reg.travel_s(r, pos, from) + reg.travel_s(r, from, dest);
    let energy = reg.travel_wh(r, pos, from)
        + reg.travel_wh(r, from, dest)
        + (load + mass) * e_unit
        + reg.idle_wh(r, handle_s);
    Ok(WorkPlan {
        action: Action::Haul {
            from,
            to: dest,
            mass_kg: mass,
        },
        end: dest,
        travel_s,
        duration_s: travel_s + handle_s,
        energy_wh: energy,
        haul: Some(HaulPlan {
            from,
            to: dest,
            mass_kg: mass,
            pile,
            berm: (t.stage == Stage::Berm)
                .then(|| ctx.planner.berm_index(dest))
                .flatten(),
            payload_kg: payload,
            load_kg: load,
            dump_kg: mass,
        }),
        dig: None,
        strands: false,
    })
}

/// Choose the spoil destination for an awarded dig and write it into the
/// action: the berm cell a borrow dig feeds, or else the planner's choice.
pub fn finish_dig(ctx: &Context, task: &Task, plan: &mut WorkPlan, busy: &Busy) {
    let Some(dig) = plan.dig.as_mut() else {
        return;
    };
    let Action::Dig { area, spoil_to, .. } = &mut plan.action else {
        return;
    };
    let forced = task.cast_to.and_then(|at| {
        ctx.planner.berm_index(at).map(|i| SpoilTarget {
            at,
            kind: SpoilKind::Berm(i),
        })
    });
    let target = forced.or_else(|| {
        ctx.planner
            .spoil_target(ctx.world, area.min, dig.overflow_kg(), busy, ctx.cfg)
    });
    *spoil_to = target.map(|s| s.at);
    dig.spoil = target;
}

/// A won bid.
#[derive(Debug, Clone, PartialEq)]
pub struct Award {
    /// Robot index.
    pub robot: usize,
    /// Index into the task list.
    pub task: usize,
    /// The robot's plan.
    pub plan: WorkPlan,
    /// Winning score.
    pub score: f64,
}

/// A robot taking part in an auction round.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Bidder {
    /// Robot index.
    pub robot: usize,
    /// Only bid on role work (the robot is busy with something interruptible).
    pub roles_only: bool,
}

/// Outcome of one auction round.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AuctionResult {
    /// Awards in the order they were made.
    pub awards: Vec<Award>,
    /// Robots that had role work they could not afford.
    pub energy_blocked: BTreeSet<usize>,
    /// Affordable role tasks each robot could bid on.
    pub role_bids: BTreeMap<usize, usize>,
}

/// Fewest tasks each robot fully plans per round.
const MIN_CANDIDATES: usize = 16;

/// Most rows or columns a scan area is grown to be entered from a covered cell.
const SCAN_GROW_CELLS: u32 = 6;

/// Run one auction round over `tasks`. `cooling(robot, key)` vetoes retries
/// that are still backing off.
pub fn auction(
    ctx: &Context,
    bidders: &[Bidder],
    tasks: &[Task],
    cooling: &dyn Fn(usize, TaskKey) -> bool,
) -> AuctionResult {
    let cfg = ctx.cfg;
    let mut result = AuctionResult::default();
    let mut bids: Vec<(f64, usize, usize, WorkPlan)> = Vec::new();
    // Each robot fully plans only its most promising tasks, ranked by a
    // cheap lower bound (direct travel; hauls via a pile can only be longer).
    let wanted = (2 * bidders.len()).max(MIN_CANDIDATES);
    for b in bidders {
        let r = b.robot;
        let Some(pos) = ctx.world.fleet.get(r).position() else {
            continue;
        };
        // The bottleneck minimises its own travel; robots with slack keep
        // the sweep in order so the bottleneck finds contiguous work.
        let seq_weight = cfg.seq_weight_s * (1.0 - ctx.capacity.utilization(r)).clamp(0.05, 1.0);
        let mut ranked: Vec<(f64, usize, bool, f64)> = Vec::new();
        for (i, t) in tasks.iter().enumerate() {
            let cap = t.capability();
            if !ctx.reg.can(r, cap) || cooling(r, t.key) {
                continue;
            }
            let role = ctx.capacity.is_role(r, cap, cfg.role_min_share);
            if b.roles_only && !role {
                continue;
            }
            let mut extra = seq_weight * t.seq;
            if let Some(section) = t.section {
                let crowd = ctx.crowd.get(&(cap, section)).copied().unwrap_or(0);
                extra += cfg.crowd_penalty_s * crowd as f64;
            }
            if !role {
                extra += cfg.help_penalty_s;
            }
            let bound = ctx.reg.travel_s(r, pos, t.entry(pos)) + extra;
            ranked.push((bound, i, role, extra));
        }
        ranked.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let mut unaffordable = false;
        let mut affordable = 0usize;
        let mut planned = 0usize;
        for (_, i, role, extra) in ranked.into_iter().take(4 * wanted) {
            if planned >= wanted {
                break;
            }
            match plan_work(ctx, r, &tasks[i]) {
                Ok(plan) => {
                    planned += 1;
                    if role {
                        affordable += 1;
                    }
                    let stranding = if plan.strands {
                        cfg.lost_link_timeout_s
                    } else {
                        0.0
                    };
                    bids.push((plan.travel_s + extra + stranding, r, i, plan));
                }
                Err(NoBid::Unaffordable) if role => unaffordable = true,
                Err(_) => {}
            }
        }
        if unaffordable && affordable == 0 {
            result.energy_blocked.insert(r);
        }
        result.role_bids.insert(r, affordable);
    }
    bids.sort_by(|a, b| {
        a.0.total_cmp(&b.0)
            .then(a.1.cmp(&b.1))
            .then(tasks[a.2].key.cmp(&tasks[b.2].key))
    });
    let mut robots = BTreeSet::new();
    let mut taken = BTreeSet::new();
    let mut cells = ctx.busy.clone();
    for (score, r, i, mut plan) in bids {
        let t = &tasks[i];
        if robots.contains(&r) || taken.contains(&i) {
            continue;
        }
        if t.changes_ground() && cells.cell(t.cell()) {
            continue;
        }
        if t.stage == Stage::Dig {
            finish_dig(ctx, t, &mut plan, &cells);
            if let Some(SpoilTarget {
                at,
                kind: SpoilKind::Fill,
            }) = plan.dig.as_ref().and_then(|d| d.spoil)
            {
                cells.cells.insert((at.x, at.y));
            }
        }
        if t.changes_ground() {
            cells.cells.insert((t.cell().x, t.cell().y));
        }
        robots.insert(r);
        taken.insert(i);
        result.awards.push(Award {
            robot: r,
            task: i,
            plan,
            score,
        });
    }
    result
}

#[cfg(test)]
mod tests;
