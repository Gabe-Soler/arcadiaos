//! Crew design: how many robots of each design to send.
//!
//! A benchmark track such as ArcadiaBench's fleet design track lets a
//! coordinator bring its own crew: a count for each of a scenario's robot
//! designs, within a number of robots and a launch mass. `arcadiaos
//! design-crew` chooses one with the model ArcadiaOS plans a build with.
//!
//! For every crew within the limits it plans the pad from what `hello` and
//! the goal say (the site as seen from orbit, the designs, the stations and
//! the power plant), splits the work with the capacity plan, and holds the
//! build time to the energy the site can deliver. A crew that leaves out a
//! capability the pad needs is not a candidate. The best crews by that time
//! are then charged for outages: each robot is taken to be out for a small
//! fraction of the build, and what its absence costs is the build time of
//! the crew without it. A crew whose only scout is out stands still; one
//! with a second scout does not.
//!
//! The pad's work is not all a crew is for. A robot that fails has to be
//! repaired, and one that runs flat in the field has to be charged where it
//! stands. So a crew also keeps a robot for each of those, when the
//! scenario's designs offer one: measured on ArcadiaBench, the crew this
//! model liked best without that rule had no hauler, was the quickest when
//! it finished, and finished 27 runs of 60.
//!
//! Nothing here is specific to a benchmark: the designs are whatever robots
//! the scenario lists, grouped by model.

use crate::capacity::CapacityPlan;
use crate::config::Config;
use crate::planner::Planner;
use crate::power::PowerModel;
use crate::registry::Registry;
use crate::world::WorldModel;
use arcadia_protocol::{
    CapabilityName, EarthToSite, EventKind, Hello, LandingPadSpec, Objective, RobotSpec, Scenario,
    SimToCoordinator, Uplink, wire,
};
use std::collections::BTreeMap;
use std::fmt::Write as _;

/// One of the robot designs a scenario's crew is made of.
#[derive(Debug, Clone, PartialEq)]
pub struct Design {
    /// Its model name, or for robots that report none, their id up to its
    /// last `-`.
    pub name: String,
    /// A robot of this design, as the scenario lists it.
    pub spec: RobotSpec,
}

fn id_prefix(id: &str) -> &str {
    id.rsplit_once('-').map_or(id, |p| p.0)
}

/// The designs in `scenario`'s crew: its robots grouped by model, in order
/// of first appearance.
pub fn designs(scenario: &Scenario) -> Vec<Design> {
    let mut out: Vec<Design> = Vec::new();
    for r in &scenario.robots {
        let name = r
            .model
            .clone()
            .filter(|m| !m.is_empty())
            .unwrap_or_else(|| id_prefix(&r.robot_id).to_string());
        if !out.iter().any(|d| d.name == name) {
            out.push(Design {
                name,
                spec: r.clone(),
            });
        }
    }
    out
}

/// Capabilities the pad's work does not list and a crew still needs: to
/// repair a robot that fails, and to charge one that runs flat in the field.
const SUPPORT: [CapabilityName; 2] = [CapabilityName::Repair, CapabilityName::Charge];

/// What a crew may be.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    /// The most the crew may weigh, kg.
    pub budget_kg: f64,
    /// The fewest and the most robots it may have.
    pub robots: (u32, u32),
}

/// A crew within the limits, and what the model expects of it.
#[derive(Debug, Clone, PartialEq)]
pub struct Candidate {
    /// Robots of each design, in the order of [`designs`].
    pub counts: Vec<u32>,
    /// Its mass, kg.
    pub mass_kg: f64,
    /// The planned build time with every robot working, s.
    pub build_s: f64,
    /// The time the site needs to deliver the build's energy, s: equal to
    /// `build_s` where the plant sets the pace.
    pub power_s: f64,
    /// The build time with outages charged, s.
    pub expected_s: f64,
}

impl Candidate {
    /// Robots in the crew.
    pub fn robots(&self) -> u32 {
        self.counts.iter().sum()
    }

    /// The counts as `2.2.2.9`.
    pub fn label(&self) -> String {
        let parts: Vec<String> = self.counts.iter().map(u32::to_string).collect();
        parts.join(".")
    }
}

/// Chooses a crew for one scenario and pad.
pub struct Designer {
    scenario: Scenario,
    pad: LandingPadSpec,
    designs: Vec<Design>,
    cfg: Config,
    /// (build time, power time) of crews already planned; `None` for a crew
    /// that cannot build the pad.
    planned: BTreeMap<Vec<u32>, Option<(f64, f64)>>,
}

impl Designer {
    /// A designer for the pad `pad` in `scenario`, with the constants the
    /// scenario sends laid over `cfg`.
    pub fn new(scenario: &Scenario, pad: &LandingPadSpec, cfg: &Config) -> Self {
        let cfg = match &scenario.constants {
            Some(c) => cfg.clone().with_constants(c),
            None => cfg.clone(),
        };
        Self {
            designs: designs(scenario),
            scenario: scenario.clone(),
            pad: pad.clone(),
            cfg,
            planned: BTreeMap::new(),
        }
    }

    /// The designs a crew is made of.
    pub fn designs(&self) -> &[Design] {
        &self.designs
    }

    /// The power the site can put into a crew over a sol, W: what its
    /// station slots carry, or what the plant has to spare.
    pub fn deliverable_w(&self) -> f64 {
        let slots: f64 = self
            .scenario
            .stations
            .iter()
            .map(|s| s.max_power_w.max(0.0))
            .sum();
        let plant = PowerModel::new(self.scenario.power.clone(), self.scenario.sol_length_s);
        slots.min(plant.mean_spare_w())
    }

    /// The mass of a crew, kg.
    pub fn mass_kg(&self, counts: &[u32]) -> f64 {
        self.designs
            .iter()
            .zip(counts)
            .map(|(d, &n)| f64::from(n) * d.spec.mass_kg.max(0.0))
            .sum()
    }

    fn robots(&self, counts: &[u32]) -> Vec<RobotSpec> {
        let mut out = Vec::new();
        for (d, &n) in self.designs.iter().zip(counts) {
            let prefix = id_prefix(&d.spec.robot_id);
            for k in 1..=n {
                let mut spec = d.spec.clone();
                spec.robot_id = format!("{prefix}-{k}");
                out.push(spec);
            }
        }
        out
    }

    /// The planned build time of a crew and the time the site needs to
    /// deliver its energy, s: `None` for a crew that lacks a capability the
    /// pad needs or a support capability the designs offer, or has no
    /// robots.
    pub fn plan(&mut self, counts: &[u32]) -> Option<(f64, f64)> {
        if let Some(known) = self.planned.get(counts) {
            return *known;
        }
        let result = self.plan_afresh(counts);
        self.planned.insert(counts.to_vec(), result);
        result
    }

    fn plan_afresh(&mut self, counts: &[u32]) -> Option<(f64, f64)> {
        let robots = self.robots(counts);
        if robots.is_empty() {
            return None;
        }
        self.scenario.robots = robots;
        let world = WorldModel::new(&self.scenario);
        let reg = Registry::new(&self.scenario);
        let planner = Planner::new("design", &self.pad, &world, &reg, &self.cfg);
        let workload = planner.workload(&world, &reg, &self.cfg);
        let undoable = workload
            .iter()
            .any(|(&cap, w)| w.steps > 0.0 && reg.capable(cap).is_empty());
        if undoable {
            return None;
        }
        // The crew keeps a robot for each support capability on offer.
        let unsupported = SUPPORT.iter().any(|&cap| {
            self.designs
                .iter()
                .any(|d| d.spec.capability(cap).is_some())
                && reg.capable(cap).is_empty()
        });
        if unsupported {
            return None;
        }
        let everyone = vec![true; reg.len()];
        let mut plan = CapacityPlan::solve(&workload, &reg, &everyone, &self.cfg);
        plan.bound_by_power(self.deliverable_w());
        // Where the crew sets the pace a build runs over its plan; where
        // the plant does, the energy takes as long as it takes.
        let crew_s = plan.loads_s.iter().copied().fold(0.0, f64::max)
            * (1.0 + self.cfg.design_crew_overhead.max(0.0));
        let build_s = crew_s.max(plan.power_s);
        (build_s.is_finite() && build_s > 0.0).then_some((build_s, plan.power_s))
    }

    /// What the build is expected to take with outages charged, s: each
    /// robot is out for `design_outage_frac` of the build, and what that
    /// costs is the build time of the crew without it, at most twice the
    /// crew's own (a build that stands still while its only robot of a kind
    /// is out).
    fn expected_s(&mut self, counts: &[u32], build_s: f64) -> f64 {
        let mut extra = 0.0;
        for i in 0..counts.len() {
            if counts[i] == 0 {
                continue;
            }
            let mut fewer = counts.to_vec();
            fewer[i] -= 1;
            let without = self
                .plan(&fewer)
                .map_or(2.0 * build_s, |p| p.0.clamp(build_s, 2.0 * build_s));
            extra += f64::from(counts[i]) * (without - build_s);
        }
        build_s + self.cfg.design_outage_frac.max(0.0) * extra
    }

    /// Every crew within `limits` that can build the pad, as far as the
    /// shortlist goes, best first. The first is the crew to send: among
    /// those whose expected time is within `design_tie_frac` of the least,
    /// the lightest.
    pub fn rank(&mut self, limits: &Limits) -> Vec<Candidate> {
        let mut crews = Vec::new();
        let mut counts = vec![0u32; self.designs.len()];
        self.enumerate(0, 0.0, 0, limits, &mut counts, &mut crews);
        let mut planned: Vec<Candidate> = crews
            .into_iter()
            .filter_map(|counts| {
                let (build_s, power_s) = self.plan(&counts)?;
                Some(Candidate {
                    mass_kg: self.mass_kg(&counts),
                    counts,
                    build_s,
                    power_s,
                    expected_s: build_s,
                })
            })
            .collect();
        let by_counts = |a: &Candidate, b: &Candidate| a.counts.cmp(&b.counts);
        planned.sort_by(|a, b| a.build_s.total_cmp(&b.build_s).then(by_counts(a, b)));
        planned.truncate(self.cfg.design_shortlist.max(1));
        for c in &mut planned {
            c.expected_s = self.expected_s(&c.counts, c.build_s);
        }
        planned.sort_by(|a, b| a.expected_s.total_cmp(&b.expected_s).then(by_counts(a, b)));
        let Some(least) = planned.first().map(|c| c.expected_s) else {
            return planned;
        };
        let near = least * (1.0 + self.cfg.design_tie_frac.max(0.0));
        let pick = planned
            .iter()
            .enumerate()
            .filter(|(_, c)| c.expected_s <= near)
            .min_by(|a, b| {
                (a.1.mass_kg.total_cmp(&b.1.mass_kg))
                    .then(a.1.robots().cmp(&b.1.robots()))
                    .then(a.0.cmp(&b.0))
            })
            .map_or(0, |(i, _)| i);
        let chosen = planned.remove(pick);
        planned.insert(0, chosen);
        planned
    }

    fn enumerate(
        &self,
        design: usize,
        mass_kg: f64,
        robots: u32,
        limits: &Limits,
        counts: &mut Vec<u32>,
        out: &mut Vec<Vec<u32>>,
    ) {
        if design == self.designs.len() {
            if robots >= limits.robots.0 {
                out.push(counts.clone());
            }
            return;
        }
        let each = self.designs[design].spec.mass_kg.max(0.0);
        for n in 0..=limits.robots.1.saturating_sub(robots) {
            let mass = mass_kg + f64::from(n) * each;
            if mass > limits.budget_kg + 1e-9 {
                break;
            }
            counts[design] = n;
            self.enumerate(design + 1, mass, robots + n, limits, counts, out);
        }
        counts[design] = 0;
    }
}

fn goal_pad(message: &EarthToSite) -> Option<LandingPadSpec> {
    match &message.body {
        Uplink::Goal {
            objective: Objective::LandingPad { pad },
            ..
        } => Some(pad.clone()),
        _ => None,
    }
}

/// Read a scenario file: the `hello` a simulator sends, and on a later line
/// the goal, either as Earth's message or inside an observation, as in the
/// first lines of a session. `arcadiabench scenario --goal` prints such a
/// file.
pub fn read_scenario(text: &str) -> Result<(Hello, LandingPadSpec), String> {
    let (mut hello, mut pad) = (None, None);
    for (n, line) in text.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        if let Ok(Some(msg)) = wire::recv::<_, SimToCoordinator>(&mut line.as_bytes()) {
            match msg {
                SimToCoordinator::Hello(h) => hello = hello.or(Some(*h)),
                SimToCoordinator::Observation(o) => {
                    for e in &o.events {
                        if let EventKind::EarthMessage { message, .. } = &e.kind {
                            pad = pad.or_else(|| goal_pad(message));
                        }
                    }
                }
                _ => {}
            }
        } else if let Ok(Some(message)) = wire::recv::<_, EarthToSite>(&mut line.as_bytes()) {
            pad = pad.or_else(|| goal_pad(&message));
        } else {
            return Err(format!(
                "line {}: not a hello, an observation or a message from Earth",
                n + 1
            ));
        }
    }
    let hello = hello.ok_or("no hello: the scenario to design a crew for")?;
    let pad = pad.ok_or("no goal: the pad to design a crew for")?;
    Ok((hello, pad))
}

/// The crew file for `counts`: a JSON object with a count for each design,
/// named by its model with `-` written `_`, as ArcadiaBench's crew files
/// are.
pub fn crew_file(designs: &[Design], counts: &[u32]) -> String {
    let fields: Vec<String> = designs
        .iter()
        .zip(counts)
        .map(|(d, n)| {
            let key: String = d
                .name
                .chars()
                .map(|c| match c {
                    '-' | ' ' => '_',
                    '"' | '\\' => '_',
                    c if c.is_control() => '_',
                    c => c,
                })
                .collect();
            format!("\"{key}\": {n}")
        })
        .collect();
    format!("{{{}}}", fields.join(", "))
}

/// How to use `arcadiaos design-crew`.
pub const USAGE: &str = "\
usage: arcadiaos design-crew --scenario FILE --budget-kg KG [--robots MIN-MAX] [--explain]

  Prints a crew file: how many robots of each of the scenario's designs to
  send, within the mass budget.

  scenario  the hello message of the outpost to design for and, on a later
            line, the goal with the pad: what `arcadiabench scenario --goal`
            prints. `-` reads standard input.
  budget-kg the most the crew may weigh
  robots    the fewest and the most robots (default 4-20)
  explain   also say why on standard error: the best crews and their
            expected build times";

/// What `design-crew` was asked.
#[derive(Debug, Clone, PartialEq)]
pub struct Request {
    /// The scenario file, or `-` for standard input.
    pub scenario: String,
    /// The limits a crew has to keep.
    pub limits: Limits,
    /// Whether to explain the choice.
    pub explain: bool,
}

/// Parse the arguments after `design-crew`.
pub fn parse(args: &[String]) -> Result<Request, String> {
    let (mut scenario, mut budget, mut robots, mut explain) = (None, None, (4, 20), false);
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        let mut value = || it.next().ok_or_else(|| format!("{arg} needs a value"));
        match arg.as_str() {
            "--scenario" => scenario = Some(value()?.clone()),
            "--budget-kg" => {
                let v = value()?;
                let kg: f64 = v.parse().map_err(|_| format!("bad budget '{v}'"))?;
                if !(kg.is_finite() && kg > 0.0) {
                    return Err(format!("bad budget '{v}'"));
                }
                budget = Some(kg);
            }
            "--robots" => {
                let v = value()?;
                let range = v
                    .split_once('-')
                    .and_then(|(a, b)| Some((a.parse::<u32>().ok()?, b.parse::<u32>().ok()?)))
                    .filter(|&(a, b)| a >= 1 && a <= b && b <= 1000);
                robots = range.ok_or_else(|| format!("bad robot range '{v}'"))?;
            }
            "--explain" => explain = true,
            other => return Err(format!("unknown option '{other}'")),
        }
    }
    Ok(Request {
        scenario: scenario.ok_or("--scenario is needed")?,
        limits: Limits {
            budget_kg: budget.ok_or("--budget-kg is needed")?,
            robots,
        },
        explain,
    })
}

/// Design a crew for the scenario in `text` within `limits`. Returns the
/// crew file and an explanation of the choice.
pub fn design(text: &str, limits: &Limits, cfg: &Config) -> Result<(String, String), String> {
    let (hello, pad) = read_scenario(text)?;
    let mut designer = Designer::new(&hello.scenario, &pad, cfg);
    let ranked = designer.rank(limits);
    let designs = designer.designs().to_vec();
    let Some(best) = ranked.first() else {
        return Err(format!(
            "no crew of {} to {} robots within {:.0} kg can build this pad",
            limits.robots.0, limits.robots.1, limits.budget_kg
        ));
    };
    let sol = hello.scenario.sol_length_s.max(1.0);
    let mut why = String::new();
    let listed: Vec<String> = designs
        .iter()
        .map(|d| format!("{} {:.0} kg", d.name, d.spec.mass_kg))
        .collect();
    let _ = writeln!(
        why,
        "designs: {}; budget {:.0} kg, {} to {} robots; the site delivers {:.1} kW over a sol",
        listed.join(", "),
        limits.budget_kg,
        limits.robots.0,
        limits.robots.1,
        designer.deliverable_w() / 1000.0
    );
    for (i, c) in ranked.iter().take(8).enumerate() {
        let _ = writeln!(
            why,
            "  crew {}: {:.0} kg, {} robots, build {:.1} sols{}, {:.1} with outages{}",
            c.label(),
            c.mass_kg,
            c.robots(),
            c.build_s / sol,
            if c.power_s >= c.build_s {
                " (the plant's limit)"
            } else {
                ""
            },
            c.expected_s / sol,
            if i == 0 { "  <- chosen" } else { "" }
        );
    }
    Ok((crew_file(&designs, &best.counts), why))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::tests::{cap, robot, scenario};
    use arcadia_protocol::CapabilityName::*;
    use arcadia_protocol::{ChargeStation, GridCoord, GridRect, PowerMode, PowerSystem};

    /// Four designs like ArcadiaBench's, at catalog 1.1's scale, on a small
    /// site: 16 cells to sinter at 37 kWh and seven hours each.
    fn outpost(fission_w: f64) -> Scenario {
        let named = |id: &str, model: &str, mass: f64, speed: f64, caps| {
            let mut r = robot(id, speed, caps);
            r.model = Some(model.into());
            r.mass_kg = mass;
            r
        };
        let mut sc = scenario(vec![
            named(
                "sc-1",
                "scout",
                90.0,
                0.5,
                vec![cap(Scan, 4.0, 0.05, None), cap(Repair, 0.0005, 400.0, None)],
            ),
            named(
                "ex-1",
                "excavator",
                420.0,
                0.2,
                vec![
                    cap(Dig, 0.004, 100.0, None),
                    cap(Grade, 0.05, 20.0, None),
                    cap(Haul, 2.0, 0.01, Some(250.0)),
                ],
            ),
            named(
                "hl-1",
                "hauler",
                300.0,
                0.3,
                vec![cap(Haul, 3.0, 0.008, Some(400.0))],
            ),
            named(
                "cp-1",
                "compactor-sinterer",
                900.0,
                0.1,
                vec![
                    cap(Compact, 0.05, 4.0, None),
                    cap(Sinter, 4e-5, 37_000.0, None),
                ],
            ),
            named(
                "cp-2",
                "compactor-sinterer",
                900.0,
                0.1,
                vec![
                    cap(Compact, 0.05, 4.0, None),
                    cap(Sinter, 4e-5, 37_000.0, None),
                ],
            ),
        ]);
        sc.site.width = 20;
        sc.site.height = 20;
        sc.site.cell_size_m = 1.0;
        sc.stations = vec![ChargeStation {
            station_id: "st-1".into(),
            position: GridCoord { x: 1, y: 1 },
            max_power_w: 80_000.0,
            slots: 4,
        }];
        sc.power = PowerSystem {
            mode: PowerMode::Hybrid,
            fission_w,
            solar_peak_w: 0.0,
            storage_capacity_wh: 200_000.0,
            essential_load_w: 8_000.0,
        };
        sc
    }

    fn pad() -> LandingPadSpec {
        LandingPadSpec {
            area: GridRect {
                min: GridCoord { x: 6, y: 6 },
                max: GridCoord { x: 9, y: 9 },
            },
            target_elevation_m: -10.0,
            surface_tolerance_m: 0.05,
            max_slope_deg: 2.0,
            stabilize: true,
            berm: None,
        }
    }

    fn limits(budget_kg: f64) -> Limits {
        Limits {
            budget_kg,
            robots: (4, 20),
        }
    }

    #[test]
    fn designs_are_the_models_in_the_crew() {
        let sc = outpost(40_000.0);
        let d = designs(&sc);
        let names: Vec<&str> = d.iter().map(|d| d.name.as_str()).collect();
        assert_eq!(
            names,
            ["scout", "excavator", "hauler", "compactor-sinterer"]
        );
        assert_eq!(d[3].spec.robot_id, "cp-1");
        // Robots that report no model are grouped by what their ids share.
        let mut plain = sc.clone();
        for r in &mut plain.robots {
            r.model = None;
        }
        let names: Vec<String> = designs(&plain).into_iter().map(|d| d.name).collect();
        assert_eq!(names, ["sc", "ex", "hl", "cp"]);
        assert_eq!(
            crew_file(&d, &[2, 1, 0, 9]),
            r#"{"scout": 2, "excavator": 1, "hauler": 0, "compactor_sinterer": 9}"#
        );
    }

    #[test]
    fn a_crew_keeps_the_limits_and_can_build_the_pad() {
        let sc = outpost(400_000.0);
        let mut d = Designer::new(&sc, &pad(), &Config::default());
        let ranked = d.rank(&limits(5_000.0));
        assert!(!ranked.is_empty());
        for c in &ranked {
            assert!(c.mass_kg <= 5_000.0, "{c:?}");
            assert!((4..=20).contains(&c.robots()), "{c:?}");
            // A pad needs scanning, digging and grading, and sintering.
            assert!(
                c.counts[0] >= 1 && c.counts[1] >= 1 && c.counts[3] >= 1,
                "{c:?}"
            );
            assert!(c.expected_s >= c.build_s && c.build_s > 0.0, "{c:?}");
        }
        // A crew without a sinterer is no plan at all.
        assert_eq!(d.plan(&[1, 1, 1, 0]), None);
        assert_eq!(d.plan(&[0, 0, 0, 0]), None);
        assert!(d.plan(&[1, 1, 0, 1]).is_some());
        // Nothing fits in 1.2 t: the lightest crew that can build weighs
        // 90 + 420 + 900 kg, and a fourth robot at least 90 more.
        assert!(d.rank(&limits(1_400.0)).is_empty());
        assert_eq!(d.rank(&limits(1_500.0))[0].counts, [2, 1, 0, 1]);
    }

    #[test]
    fn mass_goes_to_sinterers_while_the_plant_can_feed_them() {
        // A plant with power to spare: the sinterers set the pace, so the
        // budget buys as many as it can beside the robots the pad needs.
        let sc = outpost(400_000.0);
        let mut d = Designer::new(&sc, &pad(), &Config::default());
        let rich = d.rank(&limits(6_000.0))[0].clone();
        assert!(rich.power_s < rich.build_s, "{rich:?}");
        // Two scouts, an excavator and six sinterers weigh the 6 t exactly.
        assert_eq!(rich.counts, [2, 1, 0, 6], "{rich:?}");
        let more = d.plan(&[1, 1, 0, 5]).unwrap().0;
        let fewer = d.plan(&[1, 1, 0, 4]).unwrap().0;
        assert!(more < 0.85 * fewer, "{more} against {fewer}");
        // A plant with 12 kW to spare feeds about three: more sinterers
        // build no faster, and the crew chosen does not carry them.
        let sc = outpost(20_000.0);
        let mut d = Designer::new(&sc, &pad(), &Config::default());
        assert!((d.deliverable_w() - 12_000.0).abs() < 1e-6);
        let two = d.plan(&[1, 1, 0, 2]).unwrap();
        let four = d.plan(&[1, 1, 0, 4]).unwrap();
        let six = d.plan(&[1, 1, 0, 6]).unwrap();
        assert!(two.0 > two.1, "two sinterers set the pace: {two:?}");
        assert_eq!(four.0, four.1, "the plant sets the pace: {four:?}");
        assert!(
            (six.0 - four.0).abs() < 1e-6 * four.0,
            "{six:?} against {four:?}"
        );
        let poor = d.rank(&limits(6_000.0))[0].clone();
        assert!((3..=4).contains(&poor.counts[3]), "{poor:?}");
        assert!(poor.mass_kg < rich.mass_kg);
    }

    #[test]
    fn a_spare_of_a_robot_the_build_stands_still_without_is_worth_its_mass() {
        let sc = outpost(400_000.0);
        let mut d = Designer::new(&sc, &pad(), &Config::default());
        // One scout and one excavator against two of each, with the same
        // sinterers: the plan's build time is the same, and the outages
        // tell them apart.
        let (lone, pair) = ([1, 1, 0, 3], [2, 2, 0, 3]);
        let build = d.plan(&lone).unwrap().0;
        assert!((d.plan(&pair).unwrap().0 - build).abs() < 0.02 * build);
        assert!(d.expected_s(&pair, build) < d.expected_s(&lone, build));
        // With nothing charged for outages they tie, and the lighter wins.
        let cfg = Config {
            design_outage_frac: 0.0,
            ..Config::default()
        };
        let mut d = Designer::new(&sc, &pad(), &cfg);
        let best = d.rank(&limits(4_000.0))[0].clone();
        assert_eq!((best.counts[0], best.counts[1]), (1, 1), "{best:?}");
    }

    #[test]
    fn the_scenario_file_is_a_hello_and_a_goal() {
        let hello = SimToCoordinator::Hello(Box::new(Hello {
            protocol_version: arcadia_protocol::PROTOCOL_VERSION.into(),
            simulator: crate::software_info(),
            run_id: "r".into(),
            seed: 1,
            scenario: outpost(400_000.0),
        }));
        let goal = EarthToSite {
            message_id: "earth-1".into(),
            body: Uplink::Goal {
                goal_id: "pad".into(),
                priority: 1,
                deadline_t: None,
                objective: Objective::LandingPad { pad: pad() },
            },
        };
        let mut file = Vec::new();
        wire::send(&mut file, &hello).unwrap();
        wire::send(&mut file, &goal).unwrap();
        let text = String::from_utf8(file).unwrap();
        let (h, p) = read_scenario(&text).unwrap();
        assert_eq!((h.seed, p), (1, pad()));
        // The whole command: a crew file, and why.
        let (crew, why) = design(&text, &limits(6_000.0), &Config::default()).unwrap();
        assert!(
            crew.starts_with(r#"{"scout": "#) && crew.ends_with('}'),
            "{crew}"
        );
        assert!(crew.contains(r#""compactor_sinterer": 6"#), "{crew}");
        assert!(
            why.contains("<- chosen") && why.contains("budget 6000 kg"),
            "{why}"
        );
        // What is missing is said.
        let first = text.lines().next().unwrap();
        assert!(read_scenario(first).unwrap_err().contains("no goal"));
        assert!(read_scenario("").unwrap_err().contains("no hello"));
        assert!(read_scenario("{}").unwrap_err().contains("line 1"));
        let e = design(&text, &limits(1_000.0), &Config::default()).unwrap_err();
        assert!(
            e.contains("no crew of 4 to 20 robots within 1000 kg"),
            "{e}"
        );
    }

    #[test]
    fn arguments_are_checked() {
        let args = |s: &str| -> Vec<String> { s.split_whitespace().map(String::from).collect() };
        let r = parse(&args("--scenario s.jsonl --budget-kg 10000")).unwrap();
        assert_eq!(r.scenario, "s.jsonl");
        assert_eq!(r.limits, limits(10_000.0));
        assert!(!r.explain);
        let r = parse(&args(
            "--budget-kg 6e3 --robots 6-12 --explain --scenario -",
        ))
        .unwrap();
        assert_eq!((r.limits.budget_kg, r.limits.robots), (6000.0, (6, 12)));
        assert!(r.explain && r.scenario == "-");
        for bad in [
            "",
            "--scenario s",
            "--budget-kg 100",
            "--scenario s --budget-kg heavy",
            "--scenario s --budget-kg -5",
            "--scenario s --budget-kg 100 --robots 8",
            "--scenario s --budget-kg 100 --robots 9-3",
            "--scenario s --budget-kg 100 --robots 0-3",
            "--scenario s --budget-kg 100 --fast",
            "--scenario",
        ] {
            assert!(parse(&args(bad)).is_err(), "{bad}");
        }
    }
}
