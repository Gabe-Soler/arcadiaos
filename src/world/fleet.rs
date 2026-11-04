//! The crew: each robot's static spec and its latest observed state.

use arcadia_protocol::{ActiveFault, GridCoord, RobotSpec, RobotState, RobotStatus, Severity};
use std::collections::BTreeMap;

/// One robot as ArcadiaOS knows it.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotEntry {
    /// Static spec from `hello`.
    pub spec: RobotSpec,
    /// Latest observed state; `None` before the first observation.
    pub state: Option<RobotState>,
    /// The simulator does not know this robot (rejected as unknown).
    pub gone: bool,
}

impl RobotEntry {
    /// Latest observed position.
    pub fn position(&self) -> Option<GridCoord> {
        self.state.as_ref().map(|s| s.position)
    }

    /// Latest observed status (idle before the first observation).
    pub fn status(&self) -> RobotStatus {
        self.state.as_ref().map_or(RobotStatus::Idle, |s| s.status)
    }

    /// Latest observed battery charge, Wh (full before the first observation).
    pub fn battery_wh(&self) -> f64 {
        self.state
            .as_ref()
            .map_or(self.spec.battery_capacity_wh, |s| s.battery_wh)
            .max(0.0)
    }

    /// Latest observed payload, kg.
    pub fn payload_kg(&self) -> f64 {
        self.state.as_ref().map_or(0.0, |s| s.payload_kg).max(0.0)
    }

    /// Whether the robot can take commands at all right now.
    pub fn operable(&self) -> bool {
        !self.gone
            && self.state.is_some()
            && !matches!(
                self.status(),
                RobotStatus::Offline
                    | RobotStatus::Failed
                    | RobotStatus::Stuck
                    | RobotStatus::BeingTowed
                    | RobotStatus::Lost
            )
    }

    /// Whether the robot is permanently gone: cannibalized, or unknown to the
    /// simulator.
    pub fn lost(&self) -> bool {
        self.gone || self.status() == RobotStatus::Lost
    }

    /// Active faults from the latest snapshot.
    pub fn faults(&self) -> &[ActiveFault] {
        self.state.as_ref().map_or(&[], |s| s.faults.as_slice())
    }

    /// Whether any active fault is disabling.
    pub fn has_disabled_fault(&self) -> bool {
        self.faults()
            .iter()
            .any(|f| f.severity == Severity::Disabled)
    }

    /// Whether every disabling fault can be cleared by a field repair.
    pub fn disabled_faults_field_repairable(&self) -> bool {
        self.faults()
            .iter()
            .filter(|f| f.severity == Severity::Disabled)
            .all(|f| f.field_repairable)
    }

    /// Degraded faults, all of which a field repair clears.
    pub fn degraded_faults(&self) -> usize {
        self.faults()
            .iter()
            .filter(|f| f.severity == Severity::Degraded)
            .count()
    }

    /// Whether `component` has an active fault.
    pub fn has_fault(&self, component: &str) -> bool {
        self.faults().iter().any(|f| f.component == component)
    }

    /// Whether the battery is (or reads) empty: the robot is `offline`, or
    /// another disabling status hides an empty battery.
    pub fn battery_empty(&self, cfg_margin_wh: f64) -> bool {
        match self.status() {
            RobotStatus::Offline => true,
            RobotStatus::Stuck | RobotStatus::Failed | RobotStatus::BeingTowed => {
                self.battery_wh() <= cfg_margin_wh
            }
            _ => false,
        }
    }

    /// Whether the robot is disabled but could be brought back: failed,
    /// stuck or offline, and not lost.
    pub fn recoverable(&self) -> bool {
        !self.lost()
            && self.state.is_some()
            && matches!(
                self.status(),
                RobotStatus::Offline | RobotStatus::Failed | RobotStatus::Stuck
            )
    }
}

/// Every robot in the scenario, in `hello` order.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Fleet {
    robots: Vec<RobotEntry>,
    index: BTreeMap<String, usize>,
}

impl Fleet {
    /// Build from the scenario's specs. A repeated id keeps its first entry.
    pub fn new(specs: &[RobotSpec]) -> Self {
        let mut fleet = Self::default();
        for spec in specs {
            if fleet.index.contains_key(&spec.robot_id) {
                continue;
            }
            fleet
                .index
                .insert(spec.robot_id.clone(), fleet.robots.len());
            fleet.robots.push(RobotEntry {
                spec: spec.clone(),
                state: None,
                gone: false,
            });
        }
        fleet
    }

    /// Number of robots.
    pub fn len(&self) -> usize {
        self.robots.len()
    }

    /// Whether the crew is empty.
    pub fn is_empty(&self) -> bool {
        self.robots.is_empty()
    }

    /// Index of the robot with `id`.
    pub fn index_of(&self, id: &str) -> Option<usize> {
        self.index.get(id).copied()
    }

    /// Robot `r`.
    pub fn get(&self, r: usize) -> &RobotEntry {
        &self.robots[r]
    }

    /// Mutable robot `r`.
    pub fn get_mut(&mut self, r: usize) -> &mut RobotEntry {
        &mut self.robots[r]
    }

    /// Every robot with its index.
    pub fn iter(&self) -> impl Iterator<Item = (usize, &RobotEntry)> {
        self.robots.iter().enumerate()
    }

    /// Take in the robot states of an observation; unknown ids are ignored.
    pub fn update(&mut self, states: &[RobotState]) {
        for s in states {
            if let Some(r) = self.index_of(&s.robot_id) {
                self.robots[r].state = Some(s.clone());
            }
        }
    }

    /// Mean position of operable robots, if any are placed.
    pub fn centroid(&self) -> Option<(f64, f64)> {
        let placed: Vec<GridCoord> = self
            .robots
            .iter()
            .filter(|r| r.operable())
            .filter_map(RobotEntry::position)
            .collect();
        if placed.is_empty() {
            return None;
        }
        let n = placed.len() as f64;
        Some((
            placed.iter().map(|c| f64::from(c.x)).sum::<f64>() / n,
            placed.iter().map(|c| f64::from(c.y)).sum::<f64>() / n,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arcadia_protocol::{CapabilityName, CapabilitySpec, HealthTelemetry};

    fn spec(id: &str) -> RobotSpec {
        RobotSpec {
            robot_id: id.into(),
            model: None,
            mass_kg: 100.0,
            battery_capacity_wh: 1000.0,
            max_speed_m_s: 0.5,
            drive_energy_wh_per_m: 0.2,
            idle_power_w: 5.0,
            capabilities: vec![CapabilitySpec {
                name: CapabilityName::Scan,
                rate: 1.0,
                energy_wh_per_unit: 0.1,
                capacity: None,
                range_m: None,
            }],
        }
    }

    fn state(id: &str, x: u32, status: RobotStatus) -> RobotState {
        RobotState {
            robot_id: id.into(),
            position: GridCoord { x, y: 0 },
            status,
            battery_wh: 400.0,
            payload_kg: 0.0,
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

    #[test]
    fn tracks_states_and_operability() {
        let mut f = Fleet::new(&[spec("a"), spec("b"), spec("a")]);
        assert_eq!(f.len(), 2);
        assert!(!f.get(0).operable(), "unplaced robots are not operable yet");
        assert_eq!(f.get(0).battery_wh(), 1000.0);
        f.update(&[
            state("a", 2, RobotStatus::Idle),
            state("b", 4, RobotStatus::Offline),
            state("zz", 0, RobotStatus::Idle),
        ]);
        assert!(f.get(0).operable());
        assert!(!f.get(1).operable());
        assert!(f.get(1).recoverable() && f.get(1).battery_empty(1.0));
        assert!(!f.get(0).recoverable() && !f.get(0).battery_empty(1.0));
        assert_eq!(f.get(0).battery_wh(), 400.0);
        assert_eq!(f.centroid(), Some((2.0, 0.0)));
        assert_eq!(f.index_of("b"), Some(1));
        assert_eq!(f.index_of("zz"), None);
    }

    #[test]
    fn lost_robots_are_never_operable_and_faults_are_read_from_the_snapshot() {
        use arcadia_protocol::FaultCause;
        let mut f = Fleet::new(&[spec("a"), spec("b")]);
        let mut a = state("a", 1, RobotStatus::Lost);
        a.faults = vec![ActiveFault {
            component: "tool".into(),
            severity: Severity::Disabled,
            cause: FaultCause::Random,
            since_t: 3.0,
            field_repairable: false,
        }];
        let mut b = state("b", 2, RobotStatus::Failed);
        b.faults = vec![
            ActiveFault {
                component: "joints".into(),
                severity: Severity::Degraded,
                cause: FaultCause::Wear,
                since_t: 1.0,
                field_repairable: true,
            },
            ActiveFault {
                component: "wheels".into(),
                severity: Severity::Disabled,
                cause: FaultCause::Wear,
                since_t: 2.0,
                field_repairable: true,
            },
        ];
        f.update(&[a, b]);
        assert!(f.get(0).lost() && !f.get(0).operable() && !f.get(0).recoverable());
        assert!(!f.get(0).disabled_faults_field_repairable());
        let b = f.get(1);
        assert!(b.recoverable() && b.has_disabled_fault());
        assert!(b.disabled_faults_field_repairable());
        assert_eq!(b.degraded_faults(), 1);
        assert!(b.has_fault("wheels") && !b.has_fault("tool"));
        assert!(
            !b.battery_empty(1.0),
            "a failed robot with charge is not empty"
        );
    }
}
