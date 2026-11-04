//! Tunable assumptions and thresholds, in one place.
//!
//! Two kinds of numbers live here. *Interface assumptions* describe work
//! semantics that the protocol does not carry in `hello` (for example how far
//! grading can level a cell); they come from the interface semantics and are
//! deliberately conservative. *Policy knobs* tune how ArcadiaOS plans. Nothing
//! here is specific to a seed, a crew or a site.

/// Every assumption and tuning knob ArcadiaOS uses.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    // ----- interface assumptions -----
    /// Grading only levels a cell within this distance of the target, m.
    pub grade_window_m: f64,
    /// Floor on any per-cell work step, s.
    pub min_work_s: f64,
    /// Regolith bulk density assumed before a scan, kg/m³.
    pub default_density_kg_m3: f64,
    /// Rock density for boulder spoil, kg/m³.
    pub rock_density_kg_m3: f64,
    /// Expected settlement from compaction, m (refined by verification scans).
    pub compact_settle_m: f64,

    // ----- earthworks plan -----
    /// Cells lower than target minus this need fill before grading, m.
    pub fill_trigger_m: f64,
    /// Fill aims for target minus this, m.
    pub fill_aim_m: f64,
    /// Cleared cells higher than target plus this are re-cut before grading, m.
    pub cut_trigger_m: f64,
    /// Spoil cast into a fill cell may leave it at most this far above target, m.
    pub overfill_limit_m: f64,
    /// Farthest a digger casts spoil straight into a fill or berm cell, m.
    pub side_cast_reach_m: f64,
    /// Berm built this far above the specified height, m.
    pub berm_margin_m: f64,
    /// Safety factor on scanned berm density when sizing berm loads.
    pub berm_density_factor: f64,
    /// Most a stockpile cell is allowed to rise, m.
    pub stockpile_max_rise_m: f64,
    /// Deepest a borrow cell is lowered, m.
    pub borrow_max_depth_m: f64,
    /// Smallest load worth hauling, kg.
    pub min_haul_kg: f64,

    // ----- survey and verification -----
    /// Target cells per pad section (survey and verification band).
    pub section_cells: u32,
    /// Verify once this many finished cells await a scan.
    pub verify_batch_cells: usize,
    /// A verified cell further than tolerance plus this from target is reworked, m.
    pub rework_margin_m: f64,

    // ----- energy -----
    /// Battery margin kept beyond the trip home, as a fraction of capacity.
    pub energy_margin_frac: f64,
    /// Lower bound on that margin, Wh.
    pub energy_margin_min_wh: f64,
    /// Safety factor on the energy of the trip to the nearest charger.
    pub reserve_factor: f64,
    /// Safety factor on task energy estimates.
    pub estimate_factor: f64,
    /// An idle robot with no work charges below this battery fraction.
    pub opportunistic_charge_frac: f64,
    /// Smallest charge worth a trip, as a fraction of capacity.
    pub min_charge_frac: f64,
    /// Extra energy added to the remaining-work estimate when charging.
    pub charge_headroom_frac: f64,

    // ----- allocation -----
    /// Seconds of travel a robot accepts to keep work in sweep order, per step.
    pub seq_weight_s: f64,
    /// Penalty for work outside a robot's planned roles, s.
    pub help_penalty_s: f64,
    /// Smallest capacity-plan share that makes a capability a robot's role.
    pub role_min_share: f64,
    /// Penalty per same-capability robot already working in a section, s.
    pub crowd_penalty_s: f64,

    // ----- execution -----
    /// Back-off before retrying a rejected or failed task on the same robot, s.
    pub retry_backoff_s: f64,
    /// Decisions at one sim time before ArcadiaOS stops issuing and waits.
    pub max_same_time_decisions: u32,
    /// Wake-up interval while waiting with nothing in flight, s.
    pub idle_wake_s: f64,
    /// Remaining unfinished pad cells at which the end game starts.
    pub endgame_cells: usize,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            grade_window_m: 0.15,
            min_work_s: 5.0,
            default_density_kg_m3: 1550.0,
            rock_density_kg_m3: 2800.0,
            compact_settle_m: 0.01,

            fill_trigger_m: 0.10,
            fill_aim_m: 0.05,
            cut_trigger_m: 0.10,
            overfill_limit_m: 0.08,
            side_cast_reach_m: 3.0,
            berm_margin_m: 0.05,
            berm_density_factor: 1.04,
            stockpile_max_rise_m: 1.2,
            borrow_max_depth_m: 0.3,
            min_haul_kg: 20.0,

            section_cells: 40,
            verify_batch_cells: 20,
            rework_margin_m: 0.0,

            energy_margin_frac: 0.025,
            energy_margin_min_wh: 25.0,
            reserve_factor: 1.25,
            estimate_factor: 1.1,
            opportunistic_charge_frac: 0.7,
            min_charge_frac: 0.15,
            charge_headroom_frac: 0.05,

            seq_weight_s: 1.0,
            help_penalty_s: 7200.0,
            role_min_share: 0.05,
            crowd_penalty_s: 60.0,

            retry_backoff_s: 300.0,
            max_same_time_decisions: 50,
            idle_wake_s: 900.0,
            endgame_cells: 8,
        }
    }
}
