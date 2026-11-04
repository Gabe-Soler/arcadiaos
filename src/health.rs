//! Robot health: smoothed telemetry, usage since the last maintenance, wear
//! estimates, performance factors for estimates, and the predictive
//! maintenance rule.
//!
//! # Wear estimate
//!
//! Wear is never reported, so it is inferred:
//!
//! - **Telemetry.** Health readings are noisy, so each channel (wheel slip,
//!   motor current, dust coverage) is smoothed per robot: an exponentially
//!   weighted level, plus a trend per hour fitted between anchors at least
//!   ten minutes apart. Wheel wear follows from slip and drive-motor wear
//!   from motor current, net of the offset a known fault adds. Joint wear
//!   follows from dust coverage, net of the sensors' share.
//! - **Usage.** Tool, sensor and battery wear have no channel of their own.
//!   They are estimated from usage since the robot was last as good as new
//!   (hours of work, exposed hours, kWh drawn) at assumed nominal rates,
//!   scaled by the site's *observed* wear rate: the ratio of the directly
//!   measured wheel and drive-motor wear to what the same driving would give
//!   at nominal rates, pooled over the crew. A harsh site shows up there
//!   within the first kilometres driven.
//!
//! A field repair cuts the estimate (and the usage behind it) by the repair
//! fraction; a depot service resets it.
//!
//! # Performance factors
//!
//! Each degraded fault applies its assumed penalty to the robot's
//! [`Perf`] at once. On top of that, completed work teaches a residual
//! calibration: the ratio of actual to estimated energy and duration,
//! smoothed and never below 1. The first 10% of it is left to the safety
//! factors estimates already carry; the excess is applied. Every cost model
//! in the registry uses the result, so reserves, affordability and charge
//! targets follow a worn or faulty robot's real consumption.
//!
//! # Maintenance rule
//!
//! Failures follow a hazard per exposed hour that rises steeply with wear,
//! assumed `k · w³` per component (with `k` scaled by the observed site wear
//! rate). For a robot with `H` hours of planned work left, the expected
//! number of faults is the integral of that hazard along its projected wear,
//! over the next few hours only (maintenance can always be done later, so
//! risk further out is no reason to stop now).
//! A maintenance option is worth it when
//!
//! ```text
//! benefit = (faults expected without − faults expected with) × fault cost × weight
//!         + degraded penalties removed × H × weight
//!         + wear slowdown removed × H × weight
//! benefit ≥ ratio × cost   and   benefit ≥ minimum benefit
//! ```
//!
//! The wear slowdown is the extra time the estimated wear costs the robot's
//! mix of driving, work and scanning (each effect assumed linear in wear),
//! plus a third of the extra energy for the charging it costs.
//!
//! where `weight` is the robot's utilisation in the capacity plan (at least
//! 0.2: a bottleneck's lost hour costs the whole crew an hour), `cost` is the
//! downtime it causes (the robot's and its repairer's, weighted the same
//! way) plus a fixed overhead per stop, and `ratio` is 1 at a natural break
//! (the robot has no work it can do right now) and 2 when the stop
//! interrupts work. A field repair halves
//! every component's wear and clears degraded faults; a depot service resets
//! wear but costs hours, so it wins only when no repairer exists or wear is
//! extreme. Late in the run `H` is small, so maintenance stops by itself.

use crate::config::Config;
use crate::executor::PlannedUse;
use crate::registry::{Perf, Registry};
use crate::world::WorldModel;
use arcadia_protocol::{COMPONENTS, CapabilityName, Severity};

/// Index of each component in a [`Wear`] array (the order of [`COMPONENTS`]).
pub mod component {
    /// Wheels.
    pub const WHEELS: usize = 0;
    /// Drive motor.
    pub const DRIVE_MOTOR: usize = 1;
    /// Joints.
    pub const JOINTS: usize = 2;
    /// Tool.
    pub const TOOL: usize = 3;
    /// Sensors.
    pub const SENSORS: usize = 4;
    /// Battery.
    pub const BATTERY: usize = 5;
}

/// Estimated wear per component, in [`COMPONENTS`] order, each in `[0, 1]`.
pub type Wear = [f64; 6];

/// Shortest gap between two trend anchors, s.
const TREND_ANCHOR_S: f64 = 600.0;

/// Weight of each new anchor-to-anchor slope in the trend.
const TREND_WEIGHT: f64 = 0.3;

/// Anchors before a telemetry trend is trusted.
const TREND_MIN_POINTS: u32 = 6;

/// Readings before a smoothed telemetry level is trusted over usage.
const LEVEL_MIN_SAMPLES: u32 = 10;

/// Prior weight (in wear units) pulling the observed site wear rate to 1.
const SCALE_PRIOR: f64 = 0.05;

/// Telemetry-based wear above this is ignored for the site rate (saturation).
const SCALE_SATURATION: f64 = 0.9;

/// Smallest planned energy that teaches the calibration, Wh.
const CALIB_MIN_WH: f64 = 5.0;

/// Smallest planned duration that teaches the calibration, s.
const CALIB_MIN_S: f64 = 30.0;

/// Weight of each new observation in the calibration (geometric moving average).
const CALIB_WEIGHT: f64 = 0.1;

/// Highest residual calibration accepted.
const CALIB_MAX: f64 = 2.0;

/// Share of a heavy robot's time spent charging: how much extra energy use
/// costs in time.
const ENERGY_TIME_SHARE: f64 = 0.3;

/// Calibration within this fraction of 1 is left to the existing safety
/// factors on estimates; only the excess is applied.
const CALIB_DEAD_BAND: f64 = 0.1;

/// One smoothed telemetry channel: a level and a trend per hour.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Smoothed {
    level: f64,
    samples: u32,
    anchor: Option<(f64, f64)>,
    trend_per_h: f64,
    trend_points: u32,
}

impl Smoothed {
    /// Take in reading `x` at time `t`.
    pub fn update(&mut self, x: f64, t: f64, alpha: f64) {
        if !x.is_finite() {
            return;
        }
        if self.samples == 0 {
            self.level = x;
            self.samples = 1;
            self.anchor = Some((t, x));
            return;
        }
        self.level += alpha.clamp(0.0, 1.0) * (x - self.level);
        self.samples += 1;
        if let Some((t0, l0)) = self.anchor
            && t - t0 >= TREND_ANCHOR_S
        {
            let slope = (self.level - l0) / ((t - t0) / 3600.0);
            self.trend_per_h = if self.trend_points == 0 {
                slope
            } else {
                self.trend_per_h + TREND_WEIGHT * (slope - self.trend_per_h)
            };
            self.trend_points += 1;
            self.anchor = Some((t, self.level));
        }
    }

    /// The smoothed level, once there is a reading.
    pub fn value(&self) -> Option<f64> {
        (self.samples > 0).then_some(self.level)
    }

    /// The smoothed trend per hour, once two anchors exist.
    pub fn trend_per_h(&self) -> Option<f64> {
        (self.trend_points > 0).then_some(self.trend_per_h)
    }

    /// Readings taken in.
    pub fn samples(&self) -> u32 {
        self.samples
    }
}

/// Usage since a robot was last as good as new.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Usage {
    /// Distance driven, km (towing included).
    pub drive_km: f64,
    /// Hours driving.
    pub drive_h: f64,
    /// Hours of work other than scanning.
    pub work_h: f64,
    /// Hours of scanning.
    pub scan_h: f64,
    /// Hours driving or working.
    pub exposed_h: f64,
    /// Energy drawn by commands, kWh.
    pub kwh: f64,
}

impl Usage {
    fn scale(&mut self, f: f64) {
        self.drive_km *= f;
        self.drive_h *= f;
        self.work_h *= f;
        self.scan_h *= f;
        self.exposed_h *= f;
        self.kwh *= f;
    }
}

/// What ArcadiaOS tracks about one robot's health.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotHealth {
    /// Wheel slip.
    pub slip: Smoothed,
    /// Motor current ratio.
    pub current: Smoothed,
    /// Dust coverage.
    pub dust: Smoothed,
    /// Usage since the robot was last as good as new, cut by field repairs.
    pub usage: Usage,
    /// Usage over the whole run (for the split between driving and work).
    pub lifetime: Usage,
    /// Residual factor on energy estimates (at least 1).
    pub energy_calib: f64,
    /// Residual factor on durations (at least 1).
    pub duration_calib: f64,
    /// Faults seen so far.
    pub faults_seen: u32,
}

impl Default for RobotHealth {
    fn default() -> Self {
        Self {
            slip: Smoothed::default(),
            current: Smoothed::default(),
            dust: Smoothed::default(),
            usage: Usage::default(),
            lifetime: Usage::default(),
            energy_calib: 1.0,
            duration_calib: 1.0,
            faults_seen: 0,
        }
    }
}

/// Which maintenance a robot should get.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Maintenance {
    /// A field repair by another robot.
    Repair,
    /// A depot service.
    Service,
}

/// A maintenance option with its expected benefit and cost.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaintenanceCase {
    /// The option.
    pub kind: Maintenance,
    /// Expected benefit, s.
    pub benefit_s: f64,
    /// Expected cost, s.
    pub cost_s: f64,
}

impl MaintenanceCase {
    /// Whether the rule says go, at a natural break or not.
    pub fn worth_it(&self, natural_break: bool, cfg: &Config) -> bool {
        let ratio = if natural_break {
            cfg.maint_break_ratio
        } else {
            cfg.maint_urgent_ratio
        };
        self.benefit_s >= cfg.maint_min_benefit_s && self.benefit_s >= ratio * self.cost_s
    }
}

/// Health of the whole crew.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Health {
    robots: Vec<RobotHealth>,
}

impl Health {
    /// Fresh health records for `n` robots.
    pub fn new(n: usize) -> Self {
        Self {
            robots: vec![RobotHealth::default(); n],
        }
    }

    /// Robot `r`'s record.
    pub fn robot(&self, r: usize) -> &RobotHealth {
        &self.robots[r]
    }

    /// Take in the latest telemetry of every placed robot that is not lost.
    pub fn observe(&mut self, world: &WorldModel, cfg: &Config) {
        for (r, e) in world.fleet.iter() {
            let (Some(s), Some(h)) = (e.state.as_ref(), self.robots.get_mut(r)) else {
                continue;
            };
            if e.lost() {
                continue;
            }
            // Out-of-coverage robots report stale telemetry; only fresh readings count.
            if s.telemetry_t + 1e-6 < world.now {
                continue;
            }
            let t = s.telemetry_t;
            h.slip.update(s.health.wheel_slip, t, cfg.health_alpha);
            h.current
                .update(s.health.motor_current_ratio, t, cfg.health_alpha);
            h.dust.update(s.health.dust_coverage, t, cfg.health_alpha);
        }
    }

    /// Record the exposure of a command robot `r` ran, scaled by the fraction
    /// of it that ran, and the energy it drew.
    pub fn record_use(&mut self, r: usize, planned: &PlannedUse, fraction: f64, energy_wh: f64) {
        let Some(h) = self.robots.get_mut(r) else {
            return;
        };
        let f = fraction.clamp(0.0, 1.0);
        let pos = |v: f64| if v.is_finite() { v.max(0.0) } else { 0.0 };
        let drive_h = pos(planned.drive_s) * f / 3600.0;
        let work_h = pos(planned.work_s) * f / 3600.0;
        let scan_h = pos(planned.scan_s) * f / 3600.0;
        for u in [&mut h.usage, &mut h.lifetime] {
            u.drive_km += pos(planned.drive_m) * f / 1000.0;
            u.drive_h += drive_h;
            u.work_h += work_h;
            u.scan_h += scan_h;
            u.exposed_h += drive_h + work_h + scan_h;
            u.kwh += pos(energy_wh) / 1000.0;
        }
    }

    /// Learn from a completed command: the ratio of actual energy and
    /// duration to the estimate *without* calibration (degraded-fault
    /// penalties included), so the residual converges to the true factor.
    pub fn calibrate(
        &mut self,
        r: usize,
        planned_wh: f64,
        actual_wh: f64,
        planned_s: f64,
        actual_s: f64,
    ) {
        let Some(h) = self.robots.get_mut(r) else {
            return;
        };
        // A geometric moving average of the ratio, never below 1.
        let learn = |calib: &mut f64, planned: f64, actual: f64, min: f64| {
            if planned >= min && actual.is_finite() && actual > 0.0 {
                let ratio = (actual / planned).clamp(0.5, 2.0);
                let log = (1.0 - CALIB_WEIGHT) * calib.ln() + CALIB_WEIGHT * ratio.ln();
                *calib = log.exp().clamp(1.0, CALIB_MAX);
            }
        };
        learn(&mut h.energy_calib, planned_wh, actual_wh, CALIB_MIN_WH);
        learn(&mut h.duration_calib, planned_s, actual_s, CALIB_MIN_S);
    }

    /// Note a new fault on robot `r`.
    pub fn fault(&mut self, r: usize) {
        if let Some(h) = self.robots.get_mut(r) {
            h.faults_seen += 1;
        }
    }

    /// A field repair of robot `r` completed: wear (and the usage behind the
    /// estimate) is cut, and the telemetry smoothing starts afresh.
    pub fn repaired(&mut self, r: usize, cfg: &Config) {
        if let Some(h) = self.robots.get_mut(r) {
            h.usage.scale(1.0 - cfg.repair_wear_cut.clamp(0.0, 1.0));
            h.slip = Smoothed::default();
            h.current = Smoothed::default();
            h.dust = Smoothed::default();
            h.energy_calib = 1.0 + (h.energy_calib - 1.0) * 0.5;
            h.duration_calib = 1.0 + (h.duration_calib - 1.0) * 0.5;
        }
    }

    /// A depot service of robot `r` completed: as good as new.
    pub fn serviced(&mut self, r: usize) {
        if let Some(h) = self.robots.get_mut(r) {
            h.usage = Usage::default();
            h.slip = Smoothed::default();
            h.current = Smoothed::default();
            h.dust = Smoothed::default();
            h.energy_calib = 1.0;
            h.duration_calib = 1.0;
        }
    }

    fn telemetry_wear(
        &self,
        world: &WorldModel,
        r: usize,
        cfg: &Config,
    ) -> (Option<f64>, Option<f64>) {
        let h = &self.robots[r];
        let e = world.fleet.get(r);
        let level = |s: &Smoothed| s.value().filter(|_| s.samples >= LEVEL_MIN_SAMPLES);
        let wheels = level(&h.slip).map(|v| {
            let fault = if e.has_fault(COMPONENTS[component::WHEELS]) {
                cfg.slip_fault
            } else {
                0.0
            };
            ((v - cfg.slip_base - fault) / cfg.slip_per_wear.max(1e-9)).clamp(0.0, 1.0)
        });
        let drive = level(&h.current).map(|v| {
            let fault = if e.has_fault(COMPONENTS[component::DRIVE_MOTOR]) {
                cfg.current_fault
            } else {
                0.0
            };
            ((v - 1.0 - fault) / cfg.current_per_wear.max(1e-9)).clamp(0.0, 1.0)
        });
        (wheels, drive)
    }

    /// The site's observed wear rate relative to the assumed nominal rates,
    /// pooled over the crew: 1 until driving has shown otherwise.
    pub fn wear_scale(&self, world: &WorldModel, cfg: &Config) -> f64 {
        let mut measured = 0.0;
        let mut predicted = 0.0;
        for r in 0..self.robots.len().min(world.fleet.len()) {
            let (wheels, drive) = self.telemetry_wear(world, r, cfg);
            let km = self.robots[r].usage.drive_km;
            for (tele, rate) in [
                (wheels, cfg.wear_wheels_per_km),
                (drive, cfg.wear_drive_per_km),
            ] {
                if let Some(w) = tele
                    && w < SCALE_SATURATION
                {
                    measured += w;
                    predicted += rate * km;
                }
            }
        }
        ((measured + SCALE_PRIOR) / (predicted + SCALE_PRIOR)).clamp(0.5, 8.0)
    }

    /// Estimated wear of robot `r`.
    pub fn wear(&self, world: &WorldModel, r: usize, cfg: &Config) -> Wear {
        use component::*;
        let scale = self.wear_scale(world, cfg);
        let h = &self.robots[r];
        let u = &h.usage;
        let (tele_wheels, tele_drive) = self.telemetry_wear(world, r, cfg);
        let mut w = [0.0; 6];
        w[WHEELS] = tele_wheels.unwrap_or(scale * cfg.wear_wheels_per_km * u.drive_km);
        w[DRIVE_MOTOR] = tele_drive.unwrap_or(scale * cfg.wear_drive_per_km * u.drive_km);
        w[SENSORS] = scale * cfg.wear_sensors_per_exposed_h * u.exposed_h;
        let usage_joints = scale * cfg.wear_joints_per_work_h * u.work_h;
        let tele_joints = h
            .dust
            .value()
            .filter(|_| h.dust.samples >= LEVEL_MIN_SAMPLES)
            .map(|v| {
                (v - cfg.dust_per_sensor_wear * w[SENSORS]) / cfg.dust_per_joint_wear.max(1e-9)
            });
        w[JOINTS] = tele_joints.map_or(usage_joints, |t| t.max(usage_joints));
        w[TOOL] = scale * cfg.wear_tool_per_work_h * u.work_h;
        w[BATTERY] = scale * cfg.wear_battery_per_kwh * u.kwh;
        w.map(|x| {
            if x.is_finite() {
                x.clamp(0.0, 1.0)
            } else {
                0.0
            }
        })
    }

    /// Estimated wear growth of robot `r` per exposed hour of the work it
    /// does: the nominal rates for its mix of driving and work, scaled by the
    /// observed site wear rate. Where a telemetry trend is well established
    /// and steeper, it wins (capped at three times the model).
    pub fn wear_growth(&self, world: &WorldModel, reg: &Registry, r: usize, cfg: &Config) -> Wear {
        use component::*;
        let scale = self.wear_scale(world, cfg);
        let h = &self.robots[r];
        let life = &h.lifetime;
        let (drive, work, _) = self.duty_split(r);
        let speed_km_h = if life.drive_h > 0.1 {
            life.drive_km / life.drive_h
        } else {
            reg.robot(r).speed_m_s * 3.6
        };
        let kwh_per_h = if life.exposed_h > 0.5 {
            life.kwh / life.exposed_h
        } else {
            0.0
        };
        let mut g = [0.0; 6];
        g[WHEELS] = scale * cfg.wear_wheels_per_km * speed_km_h * drive;
        g[DRIVE_MOTOR] = scale * cfg.wear_drive_per_km * speed_km_h * drive;
        g[JOINTS] = scale * cfg.wear_joints_per_work_h * work;
        g[TOOL] = scale * cfg.wear_tool_per_work_h * work;
        g[SENSORS] = scale * cfg.wear_sensors_per_exposed_h;
        g[BATTERY] = scale * cfg.wear_battery_per_kwh * kwh_per_h;
        // Telemetry trends are per hour of sim time; convert to exposed hours.
        let elapsed_h = ((world.now - world.start_t) / 3600.0).max(1.0);
        let duty = (life.exposed_h / elapsed_h).clamp(0.1, 1.0);
        let trusted = |s: &Smoothed| s.trend_points >= TREND_MIN_POINTS;
        if trusted(&h.slip)
            && let Some(tr) = h.slip.trend_per_h()
        {
            let t = tr / cfg.slip_per_wear.max(1e-9) / duty;
            g[WHEELS] = g[WHEELS].max(t.min(3.0 * g[WHEELS]));
        }
        if trusted(&h.current)
            && let Some(tr) = h.current.trend_per_h()
        {
            let t = tr / cfg.current_per_wear.max(1e-9) / duty;
            g[DRIVE_MOTOR] = g[DRIVE_MOTOR].max(t.min(3.0 * g[DRIVE_MOTOR]));
        }
        g.map(|x| if x.is_finite() { x.max(0.0) } else { 0.0 })
    }

    /// The calibration factors applied to robot `r`'s estimates now (energy,
    /// duration): the learned residual beyond the dead band.
    pub fn applied_calibration(&self, r: usize) -> (f64, f64) {
        let h = &self.robots[r];
        let applied = |calib: f64| 1.0 + (calib - 1.0 - CALIB_DEAD_BAND).max(0.0);
        (applied(h.energy_calib), applied(h.duration_calib))
    }

    /// Performance factors for robot `r`: degraded-fault penalties and the
    /// learned calibration.
    pub fn perf(&self, world: &WorldModel, r: usize, cfg: &Config) -> Perf {
        let (energy, duration) = self.applied_calibration(r);
        let mut p = Perf {
            speed: 1.0 / duration,
            work_rate: 1.0 / duration,
            scan_rate: 1.0 / duration,
            energy,
            ..Perf::default()
        };
        for f in world.fleet.get(r).faults() {
            if f.severity != Severity::Degraded {
                continue;
            }
            match f.component.as_str() {
                "wheels" => p.speed *= cfg.degraded_speed_factor,
                "drive_motor" => p.drive_energy *= cfg.degraded_drive_energy_factor,
                "joints" => p.work_rate *= cfg.degraded_work_rate_factor,
                "tool" => {
                    p.work_rate *= cfg.degraded_work_rate_factor;
                    p.work_energy *= cfg.degraded_work_energy_factor;
                }
                "sensors" => p.scan_rate *= cfg.degraded_scan_rate_factor,
                "battery" => p.energy *= cfg.degraded_energy_factor,
                _ => {}
            }
        }
        p.sanitized()
    }

    /// Fraction of robot `r`'s exposed time spent driving, working and scanning.
    fn duty_split(&self, r: usize) -> (f64, f64, f64) {
        let u = &self.robots[r].lifetime;
        let total = u.drive_h + u.work_h + u.scan_h;
        if total <= 1e-9 {
            return (0.4, 0.4, 0.2);
        }
        (u.drive_h / total, u.work_h / total, u.scan_h / total)
    }

    /// Fractional slowdown of robot `r`'s remaining work from its degraded
    /// faults (0 when healthy).
    pub fn degraded_penalty(&self, world: &WorldModel, r: usize, cfg: &Config) -> f64 {
        let (drive, work, scan) = self.duty_split(r);
        let inv = |f: f64| if f > 0.0 { 1.0 / f - 1.0 } else { 0.0 };
        let mut penalty = 0.0;
        for f in world.fleet.get(r).faults() {
            if f.severity != Severity::Degraded {
                continue;
            }
            // Extra energy costs charging time at roughly half its weight.
            penalty += match f.component.as_str() {
                "wheels" => inv(cfg.degraded_speed_factor) * drive,
                "drive_motor" => (cfg.degraded_drive_energy_factor - 1.0) * 0.5 * drive,
                "joints" => inv(cfg.degraded_work_rate_factor) * work,
                "tool" => {
                    (inv(cfg.degraded_work_rate_factor)
                        + (cfg.degraded_work_energy_factor - 1.0) * 0.5)
                        * work
                }
                "sensors" => inv(cfg.degraded_scan_rate_factor) * scan,
                "battery" => (cfg.degraded_energy_factor - 1.0) * 0.5,
                _ => 0.0,
            };
        }
        penalty
    }

    /// Fractional slowdown of robot `r`'s work from wear `w`: extra time per
    /// unit of time, for its mix of driving, work and scanning, plus a share
    /// of the extra energy for the charging it costs. Each effect is assumed
    /// linear in wear.
    pub fn wear_slowdown(&self, r: usize, w: &Wear, cfg: &Config) -> f64 {
        use component::*;
        let (drive, work, scan) = self.duty_split(r);
        let inv = |f: f64| if f > 0.0 { 1.0 / f - 1.0 } else { 0.0 };
        let speed = 1.0 - cfg.wear_speed_loss * w[WHEELS];
        let rate =
            (1.0 - cfg.wear_work_rate_loss * w[JOINTS]) * (1.0 - cfg.wear_work_rate_loss * w[TOOL]);
        let scan_rate = 1.0 - cfg.wear_scan_rate_loss * w[SENSORS];
        let time = drive * inv(speed) + work * inv(rate) + scan * inv(scan_rate);
        let energy = cfg.wear_energy_gain * w[BATTERY]
            + drive * cfg.wear_drive_energy_gain * w[DRIVE_MOTOR]
            + work * cfg.wear_work_energy_gain * w[TOOL];
        time + ENERGY_TIME_SHARE * energy
    }

    /// Expected faults of robot `r` over `hours` of exposure from wear `w`
    /// growing at `g` per hour.
    pub fn expected_faults(
        &self,
        world: &WorldModel,
        w: &Wear,
        g: &Wear,
        hours: f64,
        cfg: &Config,
    ) -> f64 {
        let k = cfg.wear_hazard_per_h * self.wear_scale(world, cfg);
        expected_faults(w, g, hours, k)
    }

    /// Repair work a field repair of robot `r` would take, units: its
    /// field-repairable faults, being stuck, and its estimated wear.
    pub fn repair_units(&self, world: &WorldModel, r: usize, cfg: &Config) -> f64 {
        let e = world.fleet.get(r);
        let mut units = 0.0;
        for f in e.faults() {
            if !f.field_repairable {
                continue;
            }
            units += match f.severity {
                Severity::Degraded => cfg.repair_units_degraded,
                Severity::Disabled => cfg.repair_units_disabled,
            };
        }
        if e.status() == arcadia_protocol::RobotStatus::Stuck {
            units += cfg.repair_units_stuck;
        }
        units + cfg.repair_units_per_wear * self.wear(world, r, cfg).iter().sum::<f64>()
    }

    /// Weigh maintenance for robot `r` with `hours` of planned exposure left
    /// and utilisation `util`. `repair_s` is the downtime of a field repair
    /// (with the repairer's weighted time), when a repairer is available;
    /// `service_s` the downtime of a depot service, when a depot is.
    #[allow(clippy::too_many_arguments)]
    pub fn maintenance(
        &self,
        world: &WorldModel,
        reg: &Registry,
        r: usize,
        hours: f64,
        util: f64,
        repair_s: Option<f64>,
        service_s: Option<f64>,
        cfg: &Config,
    ) -> Option<MaintenanceCase> {
        if hours <= 0.0 {
            return None;
        }
        let weight = util.clamp(0.2, 1.0);
        let w = self.wear(world, r, cfg);
        let g = self.wear_growth(world, reg, r, cfg);
        let horizon = hours.min(cfg.maint_horizon_h.max(0.0));
        let without = self.expected_faults(world, &w, &g, horizon, cfg);
        let degraded = self.degraded_penalty(world, r, cfg) * hours * 3600.0 * weight;
        // Wear also slows the robot down; a repair cuts that, a service removes it.
        let wear_cut = cfg.repair_wear_cut.clamp(0.0, 1.0);
        let cut = 1.0 - wear_cut;
        let slow_now = self.wear_slowdown(r, &w, cfg);
        let slow_repaired = self.wear_slowdown(r, &w.map(|x| x * cut), cfg);
        let time = |slowdown: f64| slowdown.max(0.0) * hours * 3600.0 * weight;
        let mut best: Option<MaintenanceCase> = None;
        let mut consider = |case: MaintenanceCase| {
            let net = case.benefit_s - case.cost_s;
            if best.is_none_or(|b| net > b.benefit_s - b.cost_s) {
                best = Some(case);
            }
        };
        if let Some(cost) = repair_s {
            let after = self.expected_faults(world, &w.map(|x| x * cut), &g, horizon, cfg);
            consider(MaintenanceCase {
                kind: Maintenance::Repair,
                benefit_s: (without - after) * cfg.fault_cost_s * weight
                    + degraded
                    + time(slow_now - slow_repaired),
                cost_s: cost + cfg.maint_overhead_s * weight,
            });
        }
        if let Some(cost) = service_s {
            let after = self.expected_faults(world, &[0.0; 6], &g, horizon, cfg);
            consider(MaintenanceCase {
                kind: Maintenance::Service,
                benefit_s: (without - after) * cfg.fault_cost_s * weight
                    + degraded
                    + time(slow_now),
                cost_s: (cost + cfg.maint_overhead_s) * weight,
            });
        }
        best
    }

    /// One line summarising robot `r`'s estimated health, for diagnostics.
    pub fn describe(&self, world: &WorldModel, reg: &Registry, r: usize, cfg: &Config) -> String {
        let w = self.wear(world, r, cfg);
        let h = &self.robots[r];
        let parts: Vec<String> = COMPONENTS
            .iter()
            .zip(w)
            .filter(|(_, x)| *x >= 0.05)
            .map(|(c, x)| format!("{c} {x:.2}"))
            .collect();
        let trend = |s: &Smoothed| {
            s.trend_per_h()
                .map_or("-".to_string(), |t| format!("{t:+.3}/h"))
        };
        format!(
            "{}: wear [{}], slip trend {}, current trend {}, site wear rate x{:.2}, energy x{:.2}, duration x{:.2}, {} faults so far",
            reg.robot(r).id,
            parts.join(", "),
            trend(&h.slip),
            trend(&h.current),
            self.wear_scale(world, cfg),
            h.energy_calib,
            h.duration_calib,
            h.faults_seen
        )
    }
}

/// Expected failures over `hours` for wear `w` growing at `g` per hour under
/// a hazard of `k · w³` per component (wear capped at 1), by Simpson's rule.
pub fn expected_faults(w: &Wear, g: &Wear, hours: f64, k: f64) -> f64 {
    if hours <= 0.0 || k <= 0.0 {
        return 0.0;
    }
    let mut total = 0.0;
    for c in 0..w.len() {
        let at = |t: f64| (w[c] + g[c] * t).clamp(0.0, 1.0).powi(3);
        let steps = 8;
        let dt = hours / f64::from(steps);
        let mut sum = at(0.0) + at(hours);
        for i in 1..steps {
            let t = f64::from(i) * dt;
            sum += if i % 2 == 1 { 4.0 } else { 2.0 } * at(t);
        }
        total += k * sum * dt / 3.0;
    }
    total
}

/// Whether capability `cap`'s work counts as tool work for wear (scans do not).
pub fn is_tool_work(cap: CapabilityName) -> bool {
    !matches!(
        cap,
        CapabilityName::Scan | CapabilityName::Charge | CapabilityName::Relay
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::tests::{cap, robot, scenario};
    use arcadia_protocol::*;

    fn state(id: &str, slip: f64, current: f64, dust: f64, t: f64) -> RobotState {
        RobotState {
            robot_id: id.into(),
            position: GridCoord { x: 1, y: 1 },
            status: RobotStatus::Idle,
            battery_wh: 4000.0,
            payload_kg: 0.0,
            current_command: None,
            health: HealthTelemetry {
                wheel_slip: slip,
                motor_current_ratio: current,
                dust_coverage: dust,
                temperature_c: -50.0,
            },
            faults: vec![],
            in_relay_coverage: true,
            telemetry_t: t,
        }
    }

    fn world() -> (WorldModel, Registry) {
        let sc = scenario(vec![robot(
            "a",
            0.15,
            vec![cap(CapabilityName::Sinter, 0.01, 60.0, None)],
        )]);
        (WorldModel::new(&sc), Registry::new(&sc))
    }

    #[test]
    fn smoothing_tracks_level_and_trend_through_noise() {
        let mut s = Smoothed::default();
        assert_eq!(s.value(), None);
        // A level rising 0.1 per hour with alternating noise, sampled every minute.
        for i in 0..600 {
            let t = f64::from(i) * 60.0;
            let noise = if i % 2 == 0 { 0.01 } else { -0.01 };
            s.update(0.02 + 0.1 * t / 3600.0 + noise, t, 0.05);
        }
        let level = s.value().unwrap();
        assert!(
            (level - (0.02 + 0.1 * 599.0 / 60.0)).abs() < 0.05,
            "{level}"
        );
        let trend = s.trend_per_h().unwrap();
        assert!((trend - 0.1).abs() < 0.02, "{trend}");
        assert_eq!(s.samples(), 600);
    }

    #[test]
    fn wear_comes_from_telemetry_and_scaled_usage() {
        let (mut w, _) = world();
        let cfg = Config::default();
        let mut h = Health::new(1);
        // Nominal driving: 2 km with slip and current matching nominal wear.
        h.record_use(
            0,
            &PlannedUse {
                drive_m: 2000.0,
                drive_s: 4000.0,
                work_s: 3600.0 * 5.0,
                ..PlannedUse::default()
            },
            1.0,
            10_000.0,
        );
        for i in 0..200 {
            let t = f64::from(i) * 30.0;
            w.now = t;
            w.fleet
                .update(&[state("a", 0.02 + 0.3 * 0.16, 1.0 + 0.6 * 0.12, 0.05, t)]);
            h.observe(&w, &cfg);
        }
        let scale = h.wear_scale(&w, &cfg);
        assert!((scale - 1.0).abs() < 0.05, "{scale}");
        let wear = h.wear(&w, 0, &cfg);
        assert!((wear[component::WHEELS] - 0.16).abs() < 0.01, "{wear:?}");
        assert!((wear[component::DRIVE_MOTOR] - 0.12).abs() < 0.01);
        assert!((wear[component::TOOL] - 0.1).abs() < 0.01);
        assert!((wear[component::BATTERY] - 0.06).abs() < 0.01);

        // The same usage on a site that wears four times faster.
        let mut fast = Health::new(1);
        fast.record_use(
            0,
            &PlannedUse {
                drive_m: 2000.0,
                drive_s: 4000.0,
                work_s: 3600.0 * 5.0,
                ..PlannedUse::default()
            },
            1.0,
            10_000.0,
        );
        for i in 0..200 {
            let t = 6000.0 + f64::from(i) * 30.0;
            w.now = t;
            w.fleet
                .update(&[state("a", 0.02 + 0.3 * 0.64, 1.0 + 0.6 * 0.48, 0.2, t)]);
            fast.observe(&w, &cfg);
        }
        let scale = fast.wear_scale(&w, &cfg);
        assert!(scale > 3.5 && scale < 4.2, "{scale}");
        let wear = fast.wear(&w, 0, &cfg);
        assert!(wear[component::TOOL] > 0.35, "{wear:?}");

        // A repair halves the estimate; a service resets it.
        fast.repaired(0, &cfg);
        assert!((fast.robot(0).usage.work_h - 2.5).abs() < 1e-9);
        fast.serviced(0);
        assert_eq!(fast.robot(0).usage, Usage::default());
    }

    #[test]
    fn degraded_faults_and_calibration_shape_performance() {
        let (mut w, _) = world();
        let cfg = Config::default();
        let mut h = Health::new(1);
        let mut s = state("a", 0.02, 1.0, 0.0, 0.0);
        s.faults = vec![
            ActiveFault {
                component: "tool".into(),
                severity: Severity::Degraded,
                cause: FaultCause::Wear,
                since_t: 0.0,
                field_repairable: true,
            },
            ActiveFault {
                component: "battery".into(),
                severity: Severity::Degraded,
                cause: FaultCause::Random,
                since_t: 0.0,
                field_repairable: true,
            },
        ];
        w.fleet.update(&[s]);
        let p = h.perf(&w, 0, &cfg);
        assert_eq!(p.work_rate, 0.75);
        assert_eq!(p.work_energy, 1.25);
        assert_eq!(p.energy, 1.25);
        assert_eq!(p.speed, 1.0);
        assert!(h.degraded_penalty(&w, 0, &cfg) > 0.1);

        // Commands that keep using 30% more energy than planned teach it;
        // the part the safety factors already cover is not applied twice.
        for _ in 0..40 {
            h.calibrate(0, 100.0, 130.0, 100.0, 100.0);
        }
        assert!(
            h.robot(0).energy_calib > 1.25,
            "{}",
            h.robot(0).energy_calib
        );
        assert_eq!(h.robot(0).duration_calib, 1.0);
        let applied = h.perf(&w, 0, &cfg).energy / 1.25;
        assert!((applied - (h.robot(0).energy_calib - 0.1)).abs() < 1e-9);
        // Commands cheaper than planned never push estimates below spec.
        for _ in 0..200 {
            h.calibrate(0, 100.0, 50.0, 100.0, 50.0);
        }
        assert_eq!(h.robot(0).energy_calib, 1.0);
    }

    #[test]
    fn expected_faults_rise_steeply_with_wear() {
        let none = [0.0; 6];
        assert_eq!(expected_faults(&none, &none, 10.0, 0.05), 0.0);
        let mut w = [0.0; 6];
        w[component::TOOL] = 0.3;
        let low = expected_faults(&w, &none, 10.0, 0.05);
        assert!((low - 0.05 * 0.027 * 10.0).abs() < 1e-9, "{low}");
        w[component::TOOL] = 0.9;
        let high = expected_faults(&w, &none, 10.0, 0.05);
        assert!(high > 20.0 * low);
        // Growth matters: from 0 at 0.1 per hour over 10 h, ∫ (0.1 t)³ dt = 2.5.
        let mut g = [0.0; 6];
        g[component::TOOL] = 0.1;
        let grown = expected_faults(&none, &g, 10.0, 1.0);
        assert!((grown - 2.5).abs() < 1e-6, "{grown}");
    }

    #[test]
    fn the_maintenance_rule_weighs_risk_against_downtime() {
        let (mut w, reg) = world();
        let cfg = Config::default();
        w.fleet.update(&[state("a", 0.02, 1.0, 0.0, 0.0)]);
        let mut h = Health::new(1);
        // A barely used robot: nothing to gain.
        let case = h
            .maintenance(&w, &reg, 0, 10.0, 1.0, Some(600.0), Some(9000.0), &cfg)
            .unwrap();
        assert!(!case.worth_it(true, &cfg), "{case:?}");
        // Heavy tool wear on the bottleneck with ten hours to go: repair.
        h.record_use(
            0,
            &PlannedUse {
                work_s: 3600.0 * 30.0,
                ..PlannedUse::default()
            },
            1.0,
            0.0,
        );
        let case = h
            .maintenance(&w, &reg, 0, 10.0, 1.0, Some(600.0), Some(9000.0), &cfg)
            .unwrap();
        assert_eq!(case.kind, Maintenance::Repair);
        assert!(case.worth_it(true, &cfg), "{case:?}");
        // Wear also slows a robot down: halving heavy tool wear wins time back.
        let worn = h.wear(&w, 0, &cfg);
        let halved = worn.map(|x| x * 0.5);
        assert!(h.wear_slowdown(0, &worn, &cfg) > h.wear_slowdown(0, &halved, &cfg) + 0.02);
        assert_eq!(h.wear_slowdown(0, &[0.0; 6], &cfg), 0.0);
        // The same robot with nothing left to do: no maintenance.
        assert!(
            h.maintenance(&w, &reg, 0, 0.0, 1.0, Some(600.0), None, &cfg)
                .is_none()
        );
        // Without a repairer only a service is possible, and it costs hours.
        let case = h
            .maintenance(&w, &reg, 0, 10.0, 1.0, None, Some(9000.0), &cfg)
            .unwrap();
        assert_eq!(case.kind, Maintenance::Service);
        assert!(h.repair_units(&w, 0, &cfg) > 0.0);
    }
}
