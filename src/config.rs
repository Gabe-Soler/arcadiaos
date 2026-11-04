//! Tunable assumptions and thresholds, in one place.
//!
//! Two kinds of numbers live here. *Interface assumptions* describe work
//! semantics that the protocol does not carry in `hello` (for example how far
//! grading can level a cell, how long a depot service takes, or how health
//! telemetry relates to wear); they come from the interface semantics and the
//! reference simulator's documentation and are deliberately conservative.
//! *Policy knobs* tune how ArcadiaOS plans. Nothing here is specific to a
//! seed, a crew or a site.

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

    // ----- repair, tow and service (interface assumptions) -----
    /// Tow speed as a fraction of the tower's speed.
    pub tow_speed_factor: f64,
    /// Tow drive energy as a multiple of the tower's own.
    pub tow_energy_factor: f64,
    /// Depot service time until a service acceptance reveals it, s.
    pub service_s: f64,
    /// Robots a depot services at once.
    pub depot_capacity: u32,
    /// Repair work per disabled field-repairable fault, units.
    pub repair_units_disabled: f64,
    /// Repair work per degraded fault, units.
    pub repair_units_degraded: f64,
    /// Repair work to free a stuck robot, units.
    pub repair_units_stuck: f64,
    /// Repair work per unit of summed component wear, units.
    pub repair_units_per_wear: f64,
    /// Fraction of every component's wear a field repair removes.
    pub repair_wear_cut: f64,
    /// Most of a repairer's battery one repair may use.
    pub repair_max_battery_frac: f64,

    // ----- health model (interface assumptions) -----
    /// Per-observation smoothing weight of health telemetry.
    pub health_alpha: f64,
    /// Per-observation smoothing weight of health telemetry trends.
    pub health_beta: f64,
    /// Wheel slip of a healthy robot.
    pub slip_base: f64,
    /// Wheel slip per unit of wheel wear.
    pub slip_per_wear: f64,
    /// Extra wheel slip while the wheels have a fault.
    pub slip_fault: f64,
    /// Motor current ratio per unit of drive-motor wear.
    pub current_per_wear: f64,
    /// Extra motor current ratio while the drive motor has a fault.
    pub current_fault: f64,
    /// Dust coverage per unit of joint wear.
    pub dust_per_joint_wear: f64,
    /// Dust coverage per unit of sensor wear.
    pub dust_per_sensor_wear: f64,
    /// Nominal wheel wear per km driven (scaled by the observed site rate).
    pub wear_wheels_per_km: f64,
    /// Nominal drive-motor wear per km driven.
    pub wear_drive_per_km: f64,
    /// Nominal tool wear per hour of work (scans excluded).
    pub wear_tool_per_work_h: f64,
    /// Nominal joint wear per hour of work.
    pub wear_joints_per_work_h: f64,
    /// Nominal sensor wear per exposed hour outdoors.
    pub wear_sensors_per_exposed_h: f64,
    /// Nominal battery wear per kWh drawn.
    pub wear_battery_per_kwh: f64,
    /// Fault hazard per exposed hour of one component at wear `w`, as `k · w³`.
    pub wear_hazard_per_h: f64,
    /// Driving speed lost per unit of wheel wear (fraction).
    pub wear_speed_loss: f64,
    /// Drive energy gained per unit of drive-motor wear (fraction).
    pub wear_drive_energy_gain: f64,
    /// Work rate lost per unit of joint or tool wear (fraction, each).
    pub wear_work_rate_loss: f64,
    /// Work energy gained per unit of tool wear (fraction).
    pub wear_work_energy_gain: f64,
    /// Scan rate lost per unit of sensor wear (fraction).
    pub wear_scan_rate_loss: f64,
    /// Energy gained on everything per unit of battery wear (fraction).
    pub wear_energy_gain: f64,
    /// Slip penalty of a degraded wheel fault on speed (factor).
    pub degraded_speed_factor: f64,
    /// Drive energy factor of a degraded drive-motor fault.
    pub degraded_drive_energy_factor: f64,
    /// Work rate factor of each degraded joints or tool fault.
    pub degraded_work_rate_factor: f64,
    /// Work energy factor of a degraded tool fault.
    pub degraded_work_energy_factor: f64,
    /// Scan rate factor of a degraded sensor fault.
    pub degraded_scan_rate_factor: f64,
    /// Energy factor of a degraded battery fault, on everything.
    pub degraded_energy_factor: f64,

    // ----- predictive maintenance -----
    /// Expected cost of one fault in the faulty robot's time, s.
    pub fault_cost_s: f64,
    /// Maintenance at a natural break when benefit ≥ this × cost.
    pub maint_break_ratio: f64,
    /// Maintenance that interrupts work when benefit ≥ this × cost.
    pub maint_urgent_ratio: f64,
    /// Smallest expected benefit worth a maintenance stop, s.
    pub maint_min_benefit_s: f64,
    /// Fixed cost of any maintenance stop (broken work flow, waiting for
    /// the helper, its trip back), s.
    pub maint_overhead_s: f64,
    /// Fault risk is projected this far ahead: maintenance can always be done
    /// later, so risk further out does not argue for acting now, h.
    pub maint_horizon_h: f64,

    // ----- nights and the site grid -----
    /// Heater draw on cold nights, as a multiple of idle power.
    pub heater_idle_factor: f64,
    /// Cold window assumed to start at this sol phase …
    pub cold_from_phase: f64,
    /// … and to end at this sol phase the next morning.
    pub cold_to_phase: f64,
    /// A robot with no work left keeps enough charge to idle this long, s.
    pub park_horizon_s: f64,
    /// Non-urgent station charging waits while the site bank is projected
    /// below this fraction before the next sunrise.
    pub bank_floor_frac: f64,
    /// Above this bank fraction in daylight surplus, free robots top up.
    pub bank_surplus_frac: f64,
    /// Battery fraction below which a free robot tops up in a surplus window.
    pub surplus_charge_frac: f64,

    // ----- Earth, authority and coverage -----
    /// One-way light time assumed before the first `comms` report, s.
    pub assumed_delay_s: f64,
    /// Mission control's typical response time to an approval request, s
    /// (interface assumption).
    pub earth_response_s: f64,
    /// An approval is waited for this multiple of the expected round trip.
    pub answer_margin: f64,
    /// Expected cost of a cannibalization Earth would not have approved: a
    /// robot lost for good, in seconds of crew time.
    pub cannibalize_risk_s: f64,
    /// A status report goes to Earth this often while the link is up, s.
    pub status_period_s: f64,
    /// An idle robot out of coverage this long drives back on its own, s
    /// (interface assumption).
    pub lost_link_timeout_s: f64,
    /// Radio power a holding relay draws on top of idle, W (interface assumption).
    pub relay_radio_w: f64,
    /// Deepest a berm borrow cell is dug below its first known elevation, m.
    pub berm_borrow_max_depth_m: f64,
    /// A relay is stationed when the robot time it saves is at least this
    /// multiple of what it costs the relay robot.
    pub relay_benefit_ratio: f64,
    /// A robot out of contact this long past its expected return is presumed
    /// stranded, s.
    pub stranded_margin_s: f64,
    /// What cannibalizing gains when nothing else can bring the robot back:
    /// its work for the rest of the mission, taken as a sol, s.
    pub no_alternative_s: f64,
    /// What a standing approval of a routine action is worth, for the
    /// level-3 rule, s.
    pub routine_benefit_s: f64,
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

            tow_speed_factor: 0.5,
            tow_energy_factor: 2.0,
            service_s: 7200.0,
            depot_capacity: 1,
            repair_units_disabled: 1.0,
            repair_units_degraded: 0.25,
            repair_units_stuck: 0.5,
            repair_units_per_wear: 0.15,
            repair_wear_cut: 0.5,
            repair_max_battery_frac: 0.8,

            health_alpha: 0.05,
            health_beta: 0.01,
            slip_base: 0.02,
            slip_per_wear: 0.3,
            slip_fault: 0.15,
            current_per_wear: 0.6,
            current_fault: 0.3,
            dust_per_joint_wear: 0.6,
            dust_per_sensor_wear: 0.4,
            wear_wheels_per_km: 0.08,
            wear_drive_per_km: 0.06,
            wear_tool_per_work_h: 0.02,
            wear_joints_per_work_h: 0.012,
            wear_sensors_per_exposed_h: 0.006,
            wear_battery_per_kwh: 0.006,
            wear_hazard_per_h: 0.05,
            wear_speed_loss: 0.3,
            wear_drive_energy_gain: 0.3,
            wear_work_rate_loss: 0.2,
            wear_work_energy_gain: 0.2,
            wear_scan_rate_loss: 0.3,
            wear_energy_gain: 0.15,
            degraded_speed_factor: 0.7,
            degraded_drive_energy_factor: 1.4,
            degraded_work_rate_factor: 0.75,
            degraded_work_energy_factor: 1.25,
            degraded_scan_rate_factor: 0.5,
            degraded_energy_factor: 1.25,

            fault_cost_s: 3600.0,
            maint_break_ratio: 1.0,
            maint_urgent_ratio: 2.0,
            maint_min_benefit_s: 600.0,
            maint_overhead_s: 900.0,
            maint_horizon_h: 4.0,

            heater_idle_factor: 1.0,
            cold_from_phase: 0.9,
            cold_to_phase: 0.35,
            park_horizon_s: 43_200.0,
            bank_floor_frac: 0.25,
            bank_surplus_frac: 0.9,
            surplus_charge_frac: 0.9,

            assumed_delay_s: 720.0,
            earth_response_s: 1200.0,
            answer_margin: 1.5,
            cannibalize_risk_s: 21_600.0,
            status_period_s: 88_775.0,
            lost_link_timeout_s: 1800.0,
            relay_radio_w: 25.0,
            berm_borrow_max_depth_m: 1.5,
            relay_benefit_ratio: 1.5,
            stranded_margin_s: 1800.0,
            no_alternative_s: 88_775.0,
            routine_benefit_s: 14_400.0,
        }
    }
}
