//! A small deterministic simulator for testing ArcadiaOS on synthetic crews.
//!
//! It is a test double written from the interface semantics, not a copy of
//! any benchmark: per-cell prerequisites (natural → cleared → graded →
//! compacted → stabilized), grading only within ±0.15 m, material moved by
//! dig spoil and hauls, energy drawn at the start of each step (a step the
//! battery cannot cover drains it and leaves the robot offline), station
//! slots with a queue, mobile chargers with a tank, and verification by a
//! scan that starts after a cell's last change. Durations are exact (no
//! noise) and there is no weather, idle drain or fault model.

use arcadia_protocol::*;
use arcadiaos::Coordinator;
use std::collections::{BTreeSet, VecDeque};
use std::f64::consts::PI;

const GRADE_WINDOW: f64 = 0.15;
const COMPACT_SETTLE: f64 = 0.01;
const MIN_WORK_S: f64 = 5.0;
const ROCK_DENSITY: f64 = 2800.0;

/// A deterministic xorshift generator for terrain.
pub struct Rand(u64);

impl Rand {
    pub fn new(seed: u64) -> Self {
        Self(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }
    pub fn next(&mut self) -> f64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        (x >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next()
    }
}

#[derive(Clone)]
struct TCell {
    elev: f64,
    density: f64,
    depth: f64,
    boulders: Vec<f64>,
    surface: Surface,
}

#[derive(Clone, Debug)]
enum Step {
    Travel(GridCoord),
    TravelToRobot(usize),
    Dig(GridCoord, f64, Option<GridCoord>),
    Grade(GridCoord, f64),
    Compact(GridCoord),
    Sinter(GridCoord),
    Scan(GridRect),
    Load(GridCoord, f64),
    Dump(GridCoord),
    Wait(f64),
    StationCharge(usize, f64),
    MobileCharge(usize, f64),
}

struct Job {
    cmd: Command,
    steps: VecDeque<Step>,
    running: Option<(Step, f64, f64)>,
    energy: f64,
}

struct Bot {
    spec: RobotSpec,
    pos: GridCoord,
    battery: f64,
    payload: f64,
    tank: f64,
    status: RobotStatus,
    job: Option<Job>,
    /// Charging: power in W, or waiting in a station queue.
    charging: Option<f64>,
    locked_by: Option<usize>,
}

impl Bot {
    fn cap(&self, name: CapabilityName) -> Option<&CapabilitySpec> {
        self.spec.capabilities.iter().find(|c| c.name == name)
    }
    fn haul_capacity(&self) -> f64 {
        self.cap(CapabilityName::Haul)
            .and_then(|c| c.capacity)
            .unwrap_or(0.0)
    }
}

struct Station {
    spec: ChargeStation,
    occupants: Vec<usize>,
    queue: VecDeque<usize>,
}

/// How a test run ended.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Ending {
    Complete(f64),
    TimeLimit,
}

/// Counters from a run.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Stats {
    pub decisions: u64,
    pub commands: u64,
    pub rejected: u64,
    pub failed: u64,
    pub offline: usize,
    pub max_decision_ms: f64,
    pub command_log: Vec<String>,
}

/// The test simulator.
pub struct MiniSim {
    pub t: f64,
    w: u32,
    h: u32,
    cells: Vec<TCell>,
    pub pad: LandingPadSpec,
    berm_base: Vec<(GridCoord, f64)>,
    last_change: Vec<f64>,
    verified: Vec<bool>,
    bots: Vec<Bot>,
    stations: Vec<Station>,
    pending: Vec<Event>,
    seq: u64,
    wake: Option<f64>,
    ids: BTreeSet<String>,
    decision_due: bool,
    pub stats: Stats,
    hello: Hello,
}

/// A synthetic scenario.
pub struct Setup {
    pub seed: u64,
    pub size: u32,
    pub pad: GridRect,
    pub berm: Vec<GridCoord>,
    pub relief_m: f64,
    pub robots: Vec<(RobotSpec, GridCoord)>,
    pub stations: Vec<ChargeStation>,
    /// Robots that start with less than a full battery: (index, Wh).
    pub start_battery: Vec<(usize, f64)>,
}

fn rect_cells(r: &GridRect) -> impl Iterator<Item = GridCoord> + '_ {
    (r.min.y..=r.max.y).flat_map(move |y| (r.min.x..=r.max.x).map(move |x| GridCoord { x, y }))
}

fn dist(a: GridCoord, b: GridCoord) -> f64 {
    let dx = f64::from(a.x) - f64::from(b.x);
    let dy = f64::from(a.y) - f64::from(b.y);
    (dx * dx + dy * dy).sqrt()
}

fn in_rect(r: &GridRect, c: GridCoord) -> bool {
    (r.min.x..=r.max.x).contains(&c.x) && (r.min.y..=r.max.y).contains(&c.y)
}

impl MiniSim {
    pub fn new(s: Setup) -> Self {
        let mut rng = Rand::new(s.seed);
        let (k1, k2) = (rng.range(0.15, 0.35), rng.range(0.15, 0.35));
        let (p1, p2) = (rng.range(0.0, 6.0), rng.range(0.0, 6.0));
        let mut cells = Vec::new();
        for y in 0..s.size {
            for x in 0..s.size {
                let (fx, fy) = (f64::from(x), f64::from(y));
                let elev = -4100.0
                    + s.relief_m * ((k1 * fx + p1).sin() + (k2 * fy + p2).cos()) / 2.0
                    + rng.range(-0.02, 0.02);
                let boulders = if rng.next() < 0.06 {
                    vec![rng.range(0.1, 0.6)]
                } else {
                    vec![]
                };
                cells.push(TCell {
                    elev,
                    density: rng.range(1400.0, 1700.0),
                    depth: rng.range(0.4, 1.5),
                    boulders,
                    surface: Surface::Natural,
                });
            }
        }
        let idx = |c: GridCoord| (c.y * s.size + c.x) as usize;
        let pad_cells: Vec<GridCoord> = rect_cells(&s.pad).collect();
        let mean =
            pad_cells.iter().map(|&c| cells[idx(c)].elev).sum::<f64>() / pad_cells.len() as f64;
        let pad = LandingPadSpec {
            area: s.pad,
            target_elevation_m: (mean * 100.0).round() / 100.0,
            surface_tolerance_m: 0.05,
            max_slope_deg: 2.0,
            stabilize: true,
            berm: (!s.berm.is_empty()).then(|| BermSpec {
                cells: s.berm.clone(),
                height_m: 0.5,
            }),
        };
        let berm_base = s.berm.iter().map(|&b| (b, cells[idx(b)].elev)).collect();
        let prior = SiteMap {
            site_id: "mini".into(),
            name: None,
            width: s.size,
            height: s.size,
            cell_size_m: 1.0,
            cells: (0..s.size)
                .flat_map(|y| (0..s.size).map(move |x| GridCoord { x, y }))
                .map(|c| CellObservation {
                    at: c,
                    cell: Cell {
                        elevation_m: Some(cells[idx(c)].elev + rng.range(-0.25, 0.25)),
                        ..Cell::default()
                    },
                })
                .collect(),
        };
        let policy = Policy {
            policy_version: 1,
            default_level: AuthorityLevel::AUTONOMOUS,
            levels: Default::default(),
            level3: Level3Rule {
                max_wait_s: 3600.0,
                default_fallback: Fallback::Abort,
            },
        };
        let hello = Hello {
            protocol_version: PROTOCOL_VERSION.into(),
            simulator: SoftwareInfo {
                name: "minisim".into(),
                version: "0".into(),
            },
            run_id: format!("mini-{}", s.seed),
            seed: s.seed,
            scenario: Scenario {
                scenario_id: "mini".into(),
                start_t: 0.0,
                time_limit_t: 10.0 * SOL_S,
                sol_length_s: SOL_S,
                site: prior,
                robots: s.robots.iter().map(|r| r.0.clone()).collect(),
                stations: s.stations.clone(),
                depots: vec![],
                policy,
            },
        };
        let bots: Vec<Bot> = s
            .robots
            .iter()
            .map(|(spec, pos)| Bot {
                tank: spec
                    .capabilities
                    .iter()
                    .find(|c| c.name == CapabilityName::Charge)
                    .and_then(|c| c.capacity)
                    .unwrap_or(0.0),
                battery: spec.battery_capacity_wh,
                spec: spec.clone(),
                pos: *pos,
                payload: 0.0,
                status: RobotStatus::Idle,
                job: None,
                charging: None,
                locked_by: None,
            })
            .collect();
        let mut bots: Vec<Bot> = bots;
        for &(i, wh) in &s.start_battery {
            if let Some(b) = bots.get_mut(i) {
                b.battery = wh;
            }
        }
        let n = pad_cells.len();
        let mut sim = Self {
            t: 0.0,
            w: s.size,
            h: s.size,
            cells,
            pad,
            berm_base,
            last_change: vec![0.0; n],
            verified: vec![false; n],
            bots,
            stations: s
                .stations
                .iter()
                .map(|st| Station {
                    spec: st.clone(),
                    occupants: vec![],
                    queue: VecDeque::new(),
                })
                .collect(),
            pending: vec![],
            seq: 0,
            wake: None,
            ids: BTreeSet::new(),
            decision_due: true,
            stats: Stats::default(),
            hello,
        };
        sim.emit(EventKind::EarthMessage {
            sent_t: 0.0,
            message: EarthToSite {
                message_id: "earth-1".into(),
                body: Uplink::Goal {
                    goal_id: "pad".into(),
                    priority: 1,
                    deadline_t: None,
                    objective: Objective::LandingPad {
                        pad: sim.pad.clone(),
                    },
                },
            },
        });
        sim
    }

    pub fn hello(&self) -> Hello {
        self.hello.clone()
    }

    fn idx(&self, c: GridCoord) -> usize {
        (c.y * self.w + c.x) as usize
    }

    fn pad_local(&self, c: GridCoord) -> Option<usize> {
        let a = &self.pad.area;
        in_rect(a, c)
            .then(|| ((c.y - a.min.y) * (a.max.x - a.min.x + 1) + (c.x - a.min.x)) as usize)
    }

    fn emit(&mut self, kind: EventKind) {
        let decision = !matches!(kind, EventKind::CommandAccepted { .. });
        self.pending.push(Event { t: self.t, kind });
        if decision {
            self.decision_due = true;
        }
    }

    fn changed(&mut self, c: GridCoord) {
        if let Some(i) = self.pad_local(c) {
            self.last_change[i] = self.t;
            self.verified[i] = false;
        }
    }

    /// Whether the pad meets spec, is verified, and the berm is built.
    pub fn complete(&self) -> bool {
        let p = &self.pad;
        let max_rise = p.max_slope_deg.to_radians().tan();
        let cells_ok = rect_cells(&p.area).all(|c| {
            let t = &self.cells[self.idx(c)];
            let neighbours_ok = [
                GridCoord { x: c.x + 1, y: c.y },
                GridCoord { x: c.x, y: c.y + 1 },
            ]
            .into_iter()
            .filter(|&n| in_rect(&p.area, n))
            .all(|n| (self.cells[self.idx(n)].elev - t.elev).abs() <= max_rise);
            t.surface == Surface::Stabilized
                && t.boulders.is_empty()
                && (t.elev - p.target_elevation_m).abs() <= p.surface_tolerance_m
                && neighbours_ok
        });
        let berm_ok = self.berm_base.iter().all(|&(c, base)| {
            let t = &self.cells[self.idx(c)];
            t.surface == Surface::Berm && t.elev >= base + 0.5
        });
        cells_ok && berm_ok && self.verified.iter().all(|&v| v)
    }

    /// Run `coord` until the pad is done or `limit_t` passes.
    pub fn run(&mut self, coord: &mut Coordinator, limit_t: f64) -> Ending {
        coord.start(Box::new(self.hello()));
        loop {
            if self.complete() {
                return Ending::Complete(self.t);
            }
            if self.t > limit_t {
                return Ending::TimeLimit;
            }
            if self.decision_due {
                self.decision_due = false;
                let obs = self.observation();
                let started = std::time::Instant::now();
                let d = coord.decide(&obs);
                let ms = started.elapsed().as_secs_f64() * 1000.0;
                self.stats.max_decision_ms = self.stats.max_decision_ms.max(ms);
                self.stats.decisions += 1;
                assert_eq!(d.seq, obs.seq);
                for c in &d.commands {
                    self.stats.commands += 1;
                    let mut line = Vec::new();
                    wire::send(&mut line, c).expect("commands serialize");
                    let line = String::from_utf8(line).expect("utf-8");
                    self.stats
                        .command_log
                        .push(format!("{:.3} {}", self.t, line.trim_end()));
                    self.submit(c);
                }
                self.wake = d.wake_at.filter(|&w| w > self.t);
                continue;
            }
            // Advance to the next scheduled moment.
            let next_step = self
                .bots
                .iter()
                .filter_map(|b| b.job.as_ref().and_then(|j| j.running.as_ref()).map(|r| r.1))
                .fold(f64::INFINITY, f64::min);
            let next = next_step.min(self.wake.unwrap_or(f64::INFINITY));
            if !next.is_finite() {
                // Nothing will ever happen again.
                self.t = limit_t + 1.0;
                continue;
            }
            self.t = next;
            if self.wake.is_some_and(|w| w <= self.t) {
                self.wake = None;
                self.decision_due = true;
            }
            for r in 0..self.bots.len() {
                let due = self.bots[r]
                    .job
                    .as_ref()
                    .and_then(|j| j.running.as_ref())
                    .is_some_and(|run| run.1 <= self.t);
                if due {
                    self.finish_step(r);
                }
            }
        }
    }

    fn observation(&mut self) -> Observation {
        let seq = self.seq;
        self.seq += 1;
        self.stats.offline = self
            .bots
            .iter()
            .filter(|b| b.status == RobotStatus::Offline)
            .count();
        Observation {
            seq,
            t: self.t,
            events: std::mem::take(&mut self.pending),
            robots: self
                .bots
                .iter()
                .map(|b| RobotState {
                    robot_id: b.spec.robot_id.clone(),
                    position: b.pos,
                    status: b.status,
                    battery_wh: b.battery,
                    payload_kg: b.payload,
                    current_command: b.job.as_ref().map(|j| j.cmd.command_id.clone()),
                    health: HealthTelemetry {
                        wheel_slip: 0.0,
                        motor_current_ratio: 1.0,
                        dust_coverage: 0.0,
                        temperature_c: -50.0,
                    },
                    in_relay_coverage: true,
                })
                .collect(),
        }
    }

    fn reject(&mut self, cmd: &Command, reason: RejectReason, detail: &str) {
        self.stats.rejected += 1;
        self.emit(EventKind::CommandRejected {
            command_id: cmd.command_id.clone(),
            robot_id: cmd.robot_id.clone(),
            reason,
            detail: Some(detail.into()),
        });
    }

    fn prereq(&self, step: &Step) -> Result<(), String> {
        let (c, allowed): (GridCoord, &[Surface]) = match step {
            Step::Grade(c, target) => {
                let t = &self.cells[self.idx(*c)];
                if t.surface == Surface::Natural {
                    return Err("not cleared".into());
                }
                if !t.boulders.is_empty() {
                    return Err("boulders".into());
                }
                if (t.elev - target).abs() > GRADE_WINDOW {
                    return Err("outside the grade window".into());
                }
                return Ok(());
            }
            Step::Compact(c) => (*c, &[Surface::Graded, Surface::Compacted]),
            Step::Sinter(c) => (*c, &[Surface::Compacted, Surface::Stabilized]),
            _ => return Ok(()),
        };
        if allowed.contains(&self.cells[self.idx(c)].surface) {
            Ok(())
        } else {
            Err("wrong surface".into())
        }
    }

    fn in_bounds(&self, c: GridCoord) -> bool {
        c.x < self.w && c.y < self.h
    }

    fn submit(&mut self, cmd: &Command) {
        if !self.ids.insert(cmd.command_id.clone()) {
            return self.reject(cmd, RejectReason::InvalidTarget, "duplicate id");
        }
        let Some(r) = self
            .bots
            .iter()
            .position(|b| b.spec.robot_id == cmd.robot_id)
        else {
            return self.reject(cmd, RejectReason::UnknownRobot, "unknown");
        };
        if let Action::Cancel { target_command } = &cmd.action {
            let running = self.bots[r].job.as_ref().map(|j| j.cmd.command_id.clone());
            if running.as_deref() != Some(target_command.as_str()) {
                return self.reject(cmd, RejectReason::InvalidTarget, "not running");
            }
            self.emit(EventKind::CommandAccepted {
                command_id: cmd.command_id.clone(),
                robot_id: cmd.robot_id.clone(),
                estimated_duration_s: 0.0,
            });
            self.fail(r, FailReason::Cancelled);
            self.emit(EventKind::CommandCompleted {
                command_id: cmd.command_id.clone(),
                robot_id: cmd.robot_id.clone(),
                energy_used_wh: 0.0,
            });
            return;
        }
        let bot = &self.bots[r];
        if bot.status == RobotStatus::Offline {
            return self.reject(cmd, RejectReason::RobotDisabled, "offline");
        }
        if bot.job.is_some() || bot.locked_by.is_some() {
            return self.reject(cmd, RejectReason::Busy, "busy");
        }
        use CapabilityName as C;
        let need = |c: C| bot.cap(c).is_some();
        let steps: VecDeque<Step> = match &cmd.action {
            Action::MoveTo { target } if self.in_bounds(*target) => {
                VecDeque::from([Step::Travel(*target)])
            }
            Action::Scan { area } if need(C::Scan) && self.in_bounds(area.max) => {
                let entry = GridCoord {
                    x: bot.pos.x.clamp(area.min.x, area.max.x),
                    y: bot.pos.y.clamp(area.min.y, area.max.y),
                };
                VecDeque::from([Step::Travel(entry), Step::Scan(*area)])
            }
            Action::Dig {
                area,
                target_elevation_m,
                spoil_to,
            } if need(C::Dig) && self.in_bounds(area.max) => rect_cells(area)
                .flat_map(|c| {
                    [
                        Step::Travel(c),
                        Step::Dig(c, *target_elevation_m, *spoil_to),
                    ]
                })
                .collect(),
            Action::Grade {
                area,
                target_elevation_m,
            } if need(C::Grade) && self.in_bounds(area.max) => {
                if let Some(e) = rect_cells(area)
                    .find_map(|c| self.prereq(&Step::Grade(c, *target_elevation_m)).err())
                {
                    return self.reject(cmd, RejectReason::InvalidTarget, &e);
                }
                rect_cells(area)
                    .flat_map(|c| [Step::Travel(c), Step::Grade(c, *target_elevation_m)])
                    .collect()
            }
            Action::Compact { area } if need(C::Compact) && self.in_bounds(area.max) => {
                if let Some(e) = rect_cells(area).find_map(|c| self.prereq(&Step::Compact(c)).err())
                {
                    return self.reject(cmd, RejectReason::InvalidTarget, &e);
                }
                rect_cells(area)
                    .flat_map(|c| [Step::Travel(c), Step::Compact(c)])
                    .collect()
            }
            Action::Sinter { area } if need(C::Sinter) && self.in_bounds(area.max) => {
                if let Some(e) = rect_cells(area).find_map(|c| self.prereq(&Step::Sinter(c)).err())
                {
                    return self.reject(cmd, RejectReason::InvalidTarget, &e);
                }
                rect_cells(area)
                    .flat_map(|c| [Step::Travel(c), Step::Sinter(c)])
                    .collect()
            }
            Action::Haul { from, to, mass_kg }
                if need(C::Haul) && self.in_bounds(*from) && self.in_bounds(*to) =>
            {
                VecDeque::from([
                    Step::Travel(*from),
                    Step::Load(*from, *mass_kg),
                    Step::Travel(*to),
                    Step::Dump(*to),
                ])
            }
            Action::Charge { source, until_wh } => {
                let until = until_wh.unwrap_or(f64::INFINITY);
                if let Some(s) = self
                    .stations
                    .iter()
                    .position(|s| &s.spec.station_id == source)
                {
                    VecDeque::from([
                        Step::Travel(self.stations[s].spec.position),
                        Step::StationCharge(s, until),
                    ])
                } else if let Some(c) = self.bots.iter().position(|b| &b.spec.robot_id == source)
                    && c != r
                    && self.bots[c].cap(C::Charge).is_some()
                {
                    VecDeque::from([Step::TravelToRobot(c), Step::MobileCharge(c, until)])
                } else {
                    return self.reject(cmd, RejectReason::InvalidTarget, "bad charge source");
                }
            }
            Action::Wait { duration_s } => VecDeque::from([Step::Wait(*duration_s)]),
            _ => return self.reject(cmd, RejectReason::MissingCapability, "cannot"),
        };
        self.emit(EventKind::CommandAccepted {
            command_id: cmd.command_id.clone(),
            robot_id: cmd.robot_id.clone(),
            estimated_duration_s: 0.0,
        });
        self.bots[r].job = Some(Job {
            cmd: cmd.clone(),
            steps,
            running: None,
            energy: 0.0,
        });
        self.next_step(r);
    }

    fn work(&self, r: usize, cap: CapabilityName, units: f64) -> (f64, f64) {
        let b = &self.bots[r];
        let c = b.cap(cap).expect("validated capability");
        let d = (units / c.rate).max(MIN_WORK_S);
        (
            d,
            units * c.energy_wh_per_unit + d * b.spec.idle_power_w / 3600.0,
        )
    }

    fn next_step(&mut self, r: usize) {
        loop {
            let Some(step) = self.bots[r].job.as_mut().and_then(|j| j.steps.pop_front()) else {
                if self.bots[r].job.is_some() {
                    self.complete_job(r);
                }
                return;
            };
            let step = match step {
                Step::TravelToRobot(c) => Step::Travel(self.bots[c].pos),
                s => s,
            };
            let b = &self.bots[r];
            let idle = b.spec.idle_power_w;
            let (duration, energy, status) = match &step {
                Step::Travel(to) => {
                    if *to == b.pos {
                        continue;
                    }
                    let d = dist(b.pos, *to) / b.spec.max_speed_m_s;
                    (
                        d,
                        dist(b.pos, *to) * b.spec.drive_energy_wh_per_m + d * idle / 3600.0,
                        RobotStatus::Moving,
                    )
                }
                Step::Scan(area) => {
                    let units =
                        ((area.max.x - area.min.x + 1) * (area.max.y - area.min.y + 1)) as f64;
                    let (d, e) = self.work(r, CapabilityName::Scan, units);
                    (d, e, RobotStatus::Working)
                }
                Step::Dig(c, target, _) => {
                    let t = &self.cells[self.idx(*c)];
                    let units = (t.elev - target).max(0.0)
                        + t.boulders.iter().map(|d| PI / 6.0 * d * d * d).sum::<f64>();
                    let (d, e) = self.work(r, CapabilityName::Dig, units);
                    (d, e, RobotStatus::Working)
                }
                Step::Grade(..) | Step::Compact(_) | Step::Sinter(_) => {
                    if let Err(e) = self.prereq(&step) {
                        return self.fail_with(r, FailReason::Blocked, &e);
                    }
                    let cap = match step {
                        Step::Grade(..) => CapabilityName::Grade,
                        Step::Compact(_) => CapabilityName::Compact,
                        _ => CapabilityName::Sinter,
                    };
                    let (d, e) = self.work(r, cap, 1.0);
                    (d, e, RobotStatus::Working)
                }
                Step::Load(at, kg) => {
                    let room = b.haul_capacity() - b.payload;
                    let t = &self.cells[self.idx(*at)];
                    let amount = (kg - b.payload).min(room).min(t.depth * t.density).max(0.0);
                    if amount <= 0.0 {
                        if b.payload > 0.0 {
                            continue;
                        }
                        return self.fail_with(r, FailReason::Blocked, "nothing to load");
                    }
                    let c = b.cap(CapabilityName::Haul).unwrap();
                    let d = amount / c.rate;
                    (
                        d,
                        amount * c.energy_wh_per_unit + d * idle / 3600.0,
                        RobotStatus::Working,
                    )
                }
                Step::Dump(_) => {
                    if b.payload <= 0.0 {
                        continue;
                    }
                    let c = b.cap(CapabilityName::Haul).unwrap();
                    let d = b.payload / c.rate;
                    (
                        d,
                        b.payload * c.energy_wh_per_unit + d * idle / 3600.0,
                        RobotStatus::Working,
                    )
                }
                Step::Wait(d) => (*d, d * idle / 3600.0, RobotStatus::Idle),
                Step::StationCharge(s, until) => {
                    let (s, until) = (*s, *until);
                    self.bots[r].status = RobotStatus::Charging;
                    let st = &mut self.stations[s];
                    if st.occupants.len() < st.spec.slots as usize {
                        st.occupants.push(r);
                        self.start_charge(r, s, until);
                    } else {
                        st.queue.push_back(r);
                        if let Some(j) = self.bots[r].job.as_mut() {
                            j.running =
                                Some((Step::StationCharge(s, until), f64::INFINITY, self.t));
                        }
                    }
                    return;
                }
                Step::MobileCharge(c, until) => {
                    let (c, until) = (*c, *until);
                    let ch = &self.bots[c];
                    if ch.job.is_some()
                        || ch.locked_by.is_some()
                        || ch.status == RobotStatus::Offline
                        || dist(ch.pos, b.pos) > 1.5
                    {
                        return self.fail_with(r, FailReason::Blocked, "charger unavailable");
                    }
                    let power = ch.cap(CapabilityName::Charge).unwrap().rate * 3600.0;
                    let need = (until.min(b.spec.battery_capacity_wh) - b.battery)
                        .max(0.0)
                        .min(ch.tank);
                    self.bots[c].locked_by = Some(r);
                    self.bots[r].status = RobotStatus::Charging;
                    self.bots[r].charging = Some(power);
                    let end = self.t + need / power * 3600.0;
                    if let Some(j) = self.bots[r].job.as_mut() {
                        j.running = Some((
                            Step::MobileCharge(c, until),
                            end.max(self.t + 0.001),
                            self.t,
                        ));
                    }
                    return;
                }
                Step::TravelToRobot(_) => unreachable!(),
            };
            let bot = &mut self.bots[r];
            if bot.battery < energy {
                bot.battery = 0.0;
                return self.fail_with(r, FailReason::InsufficientEnergy, "flat");
            }
            bot.battery -= energy;
            bot.status = status;
            let end = self.t + duration.max(0.001);
            if let Some(j) = bot.job.as_mut() {
                j.energy += energy;
                j.running = Some((step, end, self.t));
            }
            return;
        }
    }

    fn start_charge(&mut self, r: usize, s: usize, until: f64) {
        let st = &self.stations[s].spec;
        let power = st.max_power_w / f64::from(st.slots.max(1));
        let b = &mut self.bots[r];
        let mut need = (until.min(b.spec.battery_capacity_wh) - b.battery).max(0.0);
        // Without a target, a mobile charger also refills its tank.
        if until.is_infinite() {
            let tank_cap = b
                .cap(CapabilityName::Charge)
                .and_then(|c| c.capacity)
                .unwrap_or(0.0);
            need += (tank_cap - b.tank).max(0.0);
        }
        b.charging = Some(power);
        let end = self.t + need / power * 3600.0;
        if let Some(j) = b.job.as_mut() {
            j.running = Some((
                Step::StationCharge(s, until),
                end.max(self.t + 0.001),
                self.t,
            ));
        }
    }

    fn leave_station(&mut self, r: usize) {
        for s in 0..self.stations.len() {
            let st = &mut self.stations[s];
            st.queue.retain(|&q| q != r);
            if let Some(i) = st.occupants.iter().position(|&o| o == r) {
                st.occupants.remove(i);
                if let Some(next) = st.queue.pop_front() {
                    st.occupants.push(next);
                    let until = match self.bots[next]
                        .job
                        .as_ref()
                        .and_then(|j| j.running.as_ref())
                    {
                        Some((Step::StationCharge(_, u), ..)) => *u,
                        _ => f64::INFINITY,
                    };
                    self.start_charge(next, s, until);
                }
            }
        }
    }

    /// Credit the energy delivered by `step` (running since `start`, due at `end`).
    fn settle_charge(&mut self, r: usize, step: &Step, start: f64, end: f64) {
        let Some(power) = self.bots[r].charging.take() else {
            return;
        };
        let elapsed = (self.t.min(end) - start).max(0.0);
        let mut gained = power * elapsed / 3600.0;
        if let Step::MobileCharge(c, _) = step {
            gained = gained.min(self.bots[*c].tank);
            self.bots[*c].tank -= gained;
            self.bots[*c].locked_by = None;
        }
        let b = &mut self.bots[r];
        let to_battery = gained.min((b.spec.battery_capacity_wh - b.battery).max(0.0));
        b.battery += to_battery;
        if let Step::StationCharge(_, until) = step
            && until.is_infinite()
        {
            let tank_cap = b
                .cap(CapabilityName::Charge)
                .and_then(|c| c.capacity)
                .unwrap_or(0.0);
            b.tank = (b.tank + gained - to_battery).min(tank_cap);
        }
    }

    fn finish_step(&mut self, r: usize) {
        let Some((step, end, start)) = self.bots[r].job.as_mut().and_then(|j| j.running.take())
        else {
            return;
        };
        match step {
            Step::Travel(to) => self.bots[r].pos = to,
            Step::Dig(c, target, spoil_to) => {
                let i = self.idx(c);
                let t = &mut self.cells[i];
                let boulder_kg: f64 = t
                    .boulders
                    .iter()
                    .map(|d| PI / 6.0 * d * d * d * ROCK_DENSITY)
                    .sum();
                let cut = (t.elev - target).max(0.0);
                t.elev -= cut;
                t.depth = (t.depth - cut).max(0.0);
                let spoil = cut * t.density + boulder_kg;
                t.boulders.clear();
                t.surface = Surface::Cleared;
                self.changed(c);
                let b = &mut self.bots[r];
                let to_payload = spoil.min((b.haul_capacity() - b.payload).max(0.0));
                b.payload += to_payload;
                let overflow = spoil - to_payload;
                if overflow > 0.0 {
                    let dest = spoil_to.unwrap_or(GridCoord {
                        x: self.pad.area.max.x + 1,
                        y: c.y,
                    });
                    self.dump(dest, overflow);
                }
            }
            Step::Grade(c, target) => {
                let i = self.idx(c);
                self.cells[i].elev = target;
                self.cells[i].surface = Surface::Graded;
                self.changed(c);
            }
            Step::Compact(c) => {
                let i = self.idx(c);
                self.cells[i].elev -= COMPACT_SETTLE;
                self.cells[i].surface = Surface::Compacted;
                self.changed(c);
            }
            Step::Sinter(c) => {
                let i = self.idx(c);
                self.cells[i].surface = Surface::Stabilized;
                self.changed(c);
            }
            Step::Scan(area) => {
                let cells: Vec<CellObservation> = rect_cells(&area)
                    .map(|c| {
                        let t = &self.cells[self.idx(c)];
                        CellObservation {
                            at: c,
                            cell: Cell {
                                elevation_m: Some(t.elev),
                                regolith: Some(Regolith {
                                    depth_m: Some(t.depth),
                                    bulk_density_kg_m3: Some(t.density),
                                    cohesion_kpa: None,
                                }),
                                boulders: Some(
                                    t.boulders
                                        .iter()
                                        .map(|&d| Boulder {
                                            diameter_m: d,
                                            mass_kg: None,
                                        })
                                        .collect(),
                                ),
                                surface: Some(t.surface),
                                ice_depth_m: None,
                            },
                        }
                    })
                    .collect();
                for c in rect_cells(&area) {
                    if let Some(i) = self.pad_local(c)
                        && self.last_change[i] <= start
                    {
                        self.verified[i] = true;
                    }
                }
                let j = self.bots[r].job.as_ref().unwrap();
                let (command_id, robot_id) = (j.cmd.command_id.clone(), j.cmd.robot_id.clone());
                self.emit(EventKind::ScanResult {
                    command_id,
                    robot_id,
                    cells,
                });
            }
            Step::Load(at, kg) => {
                let b = &self.bots[r];
                let room = b.haul_capacity() - b.payload;
                let i = self.idx(at);
                let t = &mut self.cells[i];
                let amount = (kg - b.payload).min(room).min(t.depth * t.density).max(0.0);
                let dh = amount / t.density;
                t.elev -= dh;
                t.depth -= dh;
                self.bots[r].payload += amount;
                self.changed(at);
            }
            Step::Dump(at) => {
                let kg = std::mem::take(&mut self.bots[r].payload);
                self.dump(at, kg);
            }
            Step::StationCharge(..) | Step::MobileCharge(..) => {
                self.settle_charge(r, &step, start, end);
                self.leave_station(r);
            }
            Step::Wait(_) | Step::TravelToRobot(_) => {}
        }
        self.next_step(r);
    }

    fn dump(&mut self, c: GridCoord, kg: f64) {
        if kg <= 0.0 || !self.in_bounds(c) {
            return;
        }
        let berm = self.berm_base.iter().any(|b| b.0 == c);
        let i = self.idx(c);
        let t = &mut self.cells[i];
        let dh = kg / t.density;
        t.elev += dh;
        t.depth += dh;
        t.surface = if berm {
            Surface::Berm
        } else if matches!(t.surface, Surface::Natural | Surface::Stockpile) {
            Surface::Stockpile
        } else {
            Surface::Cleared
        };
        self.changed(c);
    }

    fn complete_job(&mut self, r: usize) {
        let b = &mut self.bots[r];
        let job = b.job.take().unwrap();
        b.status = RobotStatus::Idle;
        self.emit(EventKind::CommandCompleted {
            command_id: job.cmd.command_id,
            robot_id: job.cmd.robot_id,
            energy_used_wh: job.energy,
        });
    }

    fn fail(&mut self, r: usize, reason: FailReason) {
        self.fail_with(r, reason, "")
    }

    fn fail_with(&mut self, r: usize, reason: FailReason, detail: &str) {
        if let Some((step, end, start)) = self.bots[r].job.as_ref().and_then(|j| j.running.clone())
        {
            self.settle_charge(r, &step, start, end);
        }
        self.leave_station(r);
        let b = &mut self.bots[r];
        let Some(job) = b.job.take() else { return };
        b.status = if b.battery <= 0.0 {
            RobotStatus::Offline
        } else {
            RobotStatus::Idle
        };
        if reason != FailReason::Cancelled {
            self.stats.failed += 1;
        }
        self.emit(EventKind::CommandFailed {
            command_id: job.cmd.command_id,
            robot_id: job.cmd.robot_id,
            reason,
            detail: Some(detail.into()),
            energy_used_wh: job.energy,
        });
    }
}

// ----- crew builders -----

pub fn cap(name: CapabilityName, rate: f64, e: f64, capacity: Option<f64>) -> CapabilitySpec {
    CapabilitySpec {
        name,
        rate,
        energy_wh_per_unit: e,
        capacity,
        range_m: None,
    }
}

pub fn robot(
    id: &str,
    battery: f64,
    speed: f64,
    drive: f64,
    idle: f64,
    caps: Vec<CapabilitySpec>,
) -> RobotSpec {
    RobotSpec {
        robot_id: id.into(),
        model: None,
        mass_kg: 300.0,
        battery_capacity_wh: battery,
        max_speed_m_s: speed,
        drive_energy_wh_per_m: drive,
        idle_power_w: idle,
        capabilities: caps,
    }
}

pub fn scout(id: &str) -> RobotSpec {
    use CapabilityName::*;
    robot(
        id,
        1500.0,
        0.5,
        0.2,
        8.0,
        vec![cap(Scan, 4.0, 0.05, None), cap(Repair, 0.0005, 400.0, None)],
    )
}

pub fn excavator(id: &str) -> RobotSpec {
    use CapabilityName::*;
    robot(
        id,
        6000.0,
        0.15,
        0.9,
        35.0,
        vec![
            cap(Dig, 0.004, 100.0, None),
            cap(Grade, 0.05, 8.0, None),
            cap(Haul, 2.0, 0.01, Some(250.0)),
        ],
    )
}

pub fn hauler(id: &str) -> RobotSpec {
    use CapabilityName::*;
    robot(
        id,
        12000.0,
        0.3,
        0.7,
        20.0,
        vec![
            cap(Haul, 3.0, 0.008, Some(400.0)),
            cap(Charge, 0.5, 0.0, Some(8000.0)),
            cap(Relay, 0.0, 0.0, None),
        ],
    )
}

pub fn compactor(id: &str) -> RobotSpec {
    use CapabilityName::*;
    robot(
        id,
        10000.0,
        0.15,
        1.0,
        30.0,
        vec![cap(Compact, 0.1, 4.0, None), cap(Sinter, 0.01, 60.0, None)],
    )
}

pub fn station(id: &str, x: u32, y: u32) -> ChargeStation {
    ChargeStation {
        station_id: id.into(),
        position: GridCoord { x, y },
        max_power_w: 6000.0,
        slots: 2,
    }
}

/// Place robots in a row starting at `(x, y)`.
pub fn lined_up(specs: Vec<RobotSpec>, x: u32, y: u32) -> Vec<(RobotSpec, GridCoord)> {
    specs
        .into_iter()
        .enumerate()
        .map(|(i, s)| (s, GridCoord { x: x + i as u32, y }))
        .collect()
}
