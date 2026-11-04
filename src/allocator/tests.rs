//! Allocator tests: roles, tie-breaks, energy, conflicts and spreading.

use super::*;
use crate::capacity::CapacityPlan;
use crate::registry::tests::{cap, robot, scenario};
use arcadia_protocol::*;

const TARGET: f64 = -10.0;

fn state(id: &str, x: u32, y: u32, battery: f64, payload: f64) -> RobotState {
    RobotState {
        robot_id: id.into(),
        position: GridCoord { x, y },
        status: RobotStatus::Idle,
        battery_wh: battery,
        payload_kg: payload,
        current_command: None,
        health: HealthTelemetry {
            wheel_slip: 0.0,
            motor_current_ratio: 1.0,
            dust_coverage: 0.0,
            temperature_c: -50.0,
        },
        faults: vec![],
        in_relay_coverage: true,
        telemetry_t: 0.0,
    }
}

struct Fixture {
    world: WorldModel,
    reg: Registry,
    planner: Planner,
    capacity: CapacityPlan,
    cfg: Config,
}

impl Fixture {
    fn new(robots: Vec<RobotSpec>, states: Vec<RobotState>, cfg: Config) -> Self {
        let mut sc = scenario(robots);
        sc.site.cell_size_m = 1.0;
        sc.site.width = 20;
        sc.site.height = 20;
        let mut world = WorldModel::new(&sc);
        world.fleet.update(&states);
        let reg = Registry::new(&sc);
        let pad = LandingPadSpec {
            area: GridRect {
                min: GridCoord { x: 4, y: 4 },
                max: GridCoord { x: 11, y: 11 },
            },
            target_elevation_m: TARGET,
            surface_tolerance_m: 0.05,
            max_slope_deg: 2.0,
            stabilize: true,
            berm: None,
        };
        let planner = Planner::new("g", &pad, &world, &reg, &cfg);
        Self {
            world,
            reg,
            planner,
            capacity: CapacityPlan::default(),
            cfg,
        }
    }

    /// Survey every pad cell at `delta` from the target as natural ground.
    fn survey(&mut self, delta: f64) {
        let cells: Vec<CellObservation> = self
            .planner
            .order()
            .iter()
            .map(|&at| CellObservation {
                at,
                cell: Cell {
                    elevation_m: Some(TARGET + delta),
                    regolith: Some(Regolith {
                        depth_m: Some(1.0),
                        bulk_density_kg_m3: Some(1500.0),
                        cohesion_kpa: None,
                    }),
                    boulders: Some(vec![]),
                    surface: Some(Surface::Natural),
                    ice_depth_m: None,
                },
            })
            .collect();
        self.world.site.apply_scan(&cells, 0.0, 1.0);
    }

    fn plan_capacity(&mut self) {
        let available = vec![true; self.reg.len()];
        let wl = self.planner.workload(&self.world, &self.reg, &self.cfg);
        self.capacity = CapacityPlan::solve(&wl, &self.reg, &available, &self.cfg);
    }

    fn auction(
        &self,
        robots: &[usize],
        tasks: &[Task],
        busy: &Busy,
        crowd: &BTreeMap<(CapabilityName, usize), usize>,
    ) -> AuctionResult {
        let ctx = Context {
            world: &self.world,
            reg: &self.reg,
            planner: &self.planner,
            capacity: &self.capacity,
            cfg: &self.cfg,
            busy,
            crowd,
        };
        let bidders: Vec<Bidder> = robots
            .iter()
            .map(|&robot| Bidder {
                robot,
                roles_only: false,
            })
            .collect();
        auction(&ctx, &bidders, tasks, &|_, _| false)
    }
}

fn digger(id: &str) -> RobotSpec {
    robot(
        id,
        0.15,
        vec![
            cap(CapabilityName::Dig, 0.004, 100.0, None),
            cap(CapabilityName::Haul, 2.0, 0.01, Some(250.0)),
        ],
    )
}

fn hauler(id: &str) -> RobotSpec {
    robot(
        id,
        0.3,
        vec![cap(CapabilityName::Haul, 3.0, 0.008, Some(400.0))],
    )
}

#[test]
fn specialists_win_their_work_and_helpers_only_help() {
    let mut f = Fixture::new(
        vec![digger("dg"), hauler("hl")],
        vec![
            state("dg", 4, 3, 5000.0, 0.0),
            state("hl", 5, 3, 5000.0, 0.0),
        ],
        Config::default(),
    );
    // Digging dominates; a few low cells need fill on the side.
    f.survey(0.3);
    let low: Vec<CellObservation> = f.planner.order()[..4]
        .iter()
        .map(|&at| CellObservation {
            at,
            cell: Cell {
                elevation_m: Some(TARGET - 0.3),
                surface: Some(Surface::Natural),
                boulders: Some(vec![]),
                ..Cell::default()
            },
        })
        .collect();
    f.world.site.apply_scan(&low, 0.0, 2.0);
    f.plan_capacity();
    assert!(f.capacity.is_role(0, CapabilityName::Dig, 0.05));
    assert!(
        !f.capacity.is_role(0, CapabilityName::Haul, 0.05),
        "the hauler alone can keep up"
    );
    assert!(f.capacity.is_role(1, CapabilityName::Haul, 0.05));
    let tasks = f.planner.tasks(&f.world, &Busy::default(), &f.cfg);
    let result = f.auction(&[0, 1], &tasks, &Busy::default(), &BTreeMap::new());
    assert_eq!(result.awards.len(), 2);
    for a in &result.awards {
        let stage = tasks[a.task].stage;
        match a.robot {
            0 => assert_eq!(stage, Stage::Dig),
            _ => assert_eq!(stage, Stage::Fill),
        }
    }
    // Only fill work left: the digger helps.
    let fills: Vec<Task> = tasks
        .iter()
        .filter(|t| t.stage == Stage::Fill)
        .cloned()
        .collect();
    let result = f.auction(&[0], &fills, &Busy::default(), &BTreeMap::new());
    assert_eq!(result.awards.len(), 1);
    assert!(result.awards[0].score >= f.cfg.help_penalty_s);
}

#[test]
fn awards_are_deterministic_with_stable_tie_breaks() {
    let mut f = Fixture::new(
        vec![hauler("a"), hauler("b")],
        vec![state("a", 4, 3, 5000.0, 0.0), state("b", 4, 3, 5000.0, 0.0)],
        Config::default(),
    );
    f.survey(-0.3);
    f.plan_capacity();
    let tasks = f.planner.tasks(&f.world, &Busy::default(), &f.cfg);
    let first = f.auction(&[0, 1], &tasks, &Busy::default(), &BTreeMap::new());
    let second = f.auction(&[1, 0], &tasks, &Busy::default(), &BTreeMap::new());
    assert_eq!(first, second, "bidder order does not matter");
    // Identical robots at the same spot: the lower index takes the best task.
    let best = first
        .awards
        .iter()
        .min_by(|x, y| x.score.total_cmp(&y.score))
        .unwrap();
    assert_eq!(best.robot, 0);
}

#[test]
fn unaffordable_work_marks_the_robot_energy_blocked() {
    let mut f = Fixture::new(
        vec![digger("dg")],
        vec![state("dg", 4, 3, 150.0, 0.0)],
        Config::default(),
    );
    f.survey(0.3);
    f.plan_capacity();
    let tasks = f.planner.tasks(&f.world, &Busy::default(), &f.cfg);
    let result = f.auction(&[0], &tasks, &Busy::default(), &BTreeMap::new());
    assert!(result.awards.is_empty());
    assert!(result.energy_blocked.contains(&0));
}

#[test]
fn one_ground_change_per_cell_and_spoil_destinations() {
    let mut f = Fixture::new(
        vec![digger("dg"), hauler("hl")],
        vec![
            state("dg", 4, 3, 5000.0, 0.0),
            state("hl", 4, 3, 5000.0, 0.0),
        ],
        Config::default(),
    );
    // A single low natural cell needs both dig and fill.
    f.survey(0.0);
    let c = f.planner.order()[0];
    f.world.site.apply_scan(
        &[CellObservation {
            at: c,
            cell: Cell {
                elevation_m: Some(TARGET - 0.3),
                surface: Some(Surface::Natural),
                boulders: Some(vec![]),
                ..Cell::default()
            },
        }],
        0.0,
        2.0,
    );
    f.plan_capacity();
    let tasks: Vec<Task> = f
        .planner
        .tasks(&f.world, &Busy::default(), &f.cfg)
        .into_iter()
        .filter(|t| t.cell() == c)
        .collect();
    assert_eq!(tasks.len(), 2, "{tasks:?}");
    let result = f.auction(&[0, 1], &tasks, &Busy::default(), &BTreeMap::new());
    assert_eq!(
        result.awards.len(),
        1,
        "only one robot may change a cell at a time"
    );

    // Digging a high cell sends overflow spoil somewhere explicit.
    f.survey(0.3);
    let tasks = f.planner.tasks(&f.world, &Busy::default(), &f.cfg);
    let result = f.auction(&[0], &tasks, &Busy::default(), &BTreeMap::new());
    let plan = &result.awards[0].plan;
    let dig = plan.dig.as_ref().unwrap();
    assert!(dig.overflow_kg() > 0.0);
    let Action::Dig { spoil_to, .. } = &plan.action else {
        panic!("{:?}", plan.action)
    };
    assert_eq!(*spoil_to, dig.spoil.map(|s| s.at));
    assert!(spoil_to.is_some());
}

#[test]
fn hauls_use_payload_on_board_and_respect_overfill() {
    let mut f = Fixture::new(
        vec![hauler("hl")],
        vec![state("hl", 4, 3, 5000.0, 380.0)],
        Config::default(),
    );
    f.survey(-0.3);
    f.plan_capacity();
    let tasks = f.planner.tasks(&f.world, &Busy::default(), &f.cfg);
    let fill = tasks.iter().find(|t| t.stage == Stage::Fill).unwrap();
    let ctx = Context {
        world: &f.world,
        reg: &f.reg,
        planner: &f.planner,
        capacity: &f.capacity,
        cfg: &f.cfg,
        busy: &Busy::default(),
        crowd: &BTreeMap::new(),
    };
    // 380 kg on board covers the 375 kg the cell wants: dump without loading.
    let plan = plan_work(&ctx, 0, fill).unwrap();
    let h = plan.haul.unwrap();
    assert_eq!(h.load_kg, 0.0);
    assert_eq!(h.mass_kg, 380.0);
    assert_eq!(h.pile, None);
    // A cell that cannot take that much is refused.
    let mut small = fill.clone();
    small.max_kg = 300.0;
    assert_eq!(plan_work(&ctx, 0, &small), Err(NoBid::Unable));
}

#[test]
fn crowding_spreads_robots_across_sections() {
    let cfg = Config {
        section_cells: 16,
        seq_weight_s: 0.0,
        ..Config::default()
    };
    let mut f = Fixture::new(vec![digger("a")], vec![state("a", 4, 3, 5000.0, 0.0)], cfg);
    f.survey(0.05);
    f.plan_capacity();
    let tasks = f.planner.tasks(&f.world, &Busy::default(), &f.cfg);
    let first_section = tasks[0].section;
    let mut crowd = BTreeMap::new();
    let plain = f.auction(&[0], &tasks, &Busy::default(), &crowd);
    assert_eq!(tasks[plain.awards[0].task].section, first_section);
    // Two diggers already work the nearest section: go elsewhere.
    crowd.insert((CapabilityName::Dig, first_section.unwrap()), 2);
    let spread = f.auction(&[0], &tasks, &Busy::default(), &crowd);
    assert_ne!(tasks[spread.awards[0].task].section, first_section);
}
