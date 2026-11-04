//! Units of work: what the planner offers and the allocator assigns.
//!
//! The landing-pad task graph is per cell: survey → dig (clear and cut) →
//! fill when low → grade → compact → sinter → verify, plus berm hauling and
//! berm checks. The planner derives the *ready* tasks from the world model at
//! every decision, so replanning after any surprise is automatic.

use crate::geom;
use arcadia_protocol::{CapabilityName, GridCoord, GridRect};

/// Pipeline stage of a task.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Stage {
    /// Measure cells before work (and again after surprises).
    Survey,
    /// Clear boulders and cut high ground to the target.
    Dig,
    /// Haul material into a low cell.
    Fill,
    /// Level a cleared cell to the target.
    Grade,
    /// Densify a graded cell.
    Compact,
    /// Stabilize a compacted cell.
    Sinter,
    /// Scan finished cells so they count.
    Verify,
    /// Haul material onto a berm cell.
    Berm,
    /// Scan berm cells to confirm their height.
    BermCheck,
    /// Rescan everything when the run should already have ended.
    Recheck,
}

impl Stage {
    /// The capability that does this stage.
    pub fn capability(self) -> CapabilityName {
        match self {
            Stage::Survey | Stage::Verify | Stage::BermCheck | Stage::Recheck => {
                CapabilityName::Scan
            }
            Stage::Dig => CapabilityName::Dig,
            Stage::Fill | Stage::Berm => CapabilityName::Haul,
            Stage::Grade => CapabilityName::Grade,
            Stage::Compact => CapabilityName::Compact,
            Stage::Sinter => CapabilityName::Sinter,
        }
    }

    /// Short name for logs.
    pub fn name(self) -> &'static str {
        match self {
            Stage::Survey => "survey",
            Stage::Dig => "dig",
            Stage::Fill => "fill",
            Stage::Grade => "grade",
            Stage::Compact => "compact",
            Stage::Sinter => "sinter",
            Stage::Verify => "verify",
            Stage::Berm => "berm",
            Stage::BermCheck => "berm-check",
            Stage::Recheck => "recheck",
        }
    }
}

/// Stable identity of a task, used for reservations, retries and tie-breaks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum TaskKey {
    /// Per-cell work.
    Cell(Stage, u32, u32),
    /// Survey of pad section `i`.
    Survey(u32),
    /// Survey of the berm line.
    BermSurvey,
    /// Verification scan of finished pad cells.
    Verify,
    /// Height check of built berm cells.
    BermCheck,
    /// Whole-site recheck.
    Recheck,
}

/// One unit of ready work.
#[derive(Debug, Clone, PartialEq)]
pub struct Task {
    /// Identity.
    pub key: TaskKey,
    /// Pipeline stage.
    pub stage: Stage,
    /// Cells the work covers (a single cell for per-cell work).
    pub area: GridRect,
    /// Sweep priority: lower is needed sooner.
    pub seq: f64,
    /// Work units for the step (m², m³), or kg to deliver for hauls.
    pub amount: f64,
    /// Hauls: the most the target may receive, kg.
    pub max_kg: f64,
    /// Dig and grade: target elevation, m.
    pub elevation_m: f64,
    /// Pad section, for spreading robots of one capability.
    pub section: Option<usize>,
}

impl Task {
    /// The capability needed.
    pub fn capability(&self) -> CapabilityName {
        self.stage.capability()
    }

    /// The cell of per-cell work (the area's corner otherwise).
    pub fn cell(&self) -> GridCoord {
        self.area.min
    }

    /// Where a robot at `from` starts the work: the nearest cell of the area.
    pub fn entry(&self, from: GridCoord) -> GridCoord {
        geom::clamp_into(from, &self.area)
    }

    /// Whether the task changes the ground (as opposed to scanning it).
    pub fn changes_ground(&self) -> bool {
        self.capability() != CapabilityName::Scan
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stages_map_to_capabilities() {
        assert_eq!(Stage::Verify.capability(), CapabilityName::Scan);
        assert_eq!(Stage::Berm.capability(), CapabilityName::Haul);
        assert_eq!(Stage::Sinter.capability(), CapabilityName::Sinter);
        assert_eq!(Stage::Fill.name(), "fill");
    }

    #[test]
    fn entry_is_the_nearest_cell() {
        let t = Task {
            key: TaskKey::Verify,
            stage: Stage::Verify,
            area: GridRect {
                min: GridCoord { x: 5, y: 5 },
                max: GridCoord { x: 9, y: 9 },
            },
            seq: 0.0,
            amount: 25.0,
            max_kg: 0.0,
            elevation_m: 0.0,
            section: None,
        };
        assert_eq!(t.entry(GridCoord { x: 0, y: 7 }), GridCoord { x: 5, y: 7 });
        assert!(!t.changes_ground());
    }
}
