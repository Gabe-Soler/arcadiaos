//! Site power and the night: a forecast of the battery bank, and the cold
//! nights that drain idle robots.
//!
//! The bank forecast starts from the latest `power` report and steps forward
//! with the scenario's power system: fission, plus solar on the half-sine
//! clear-sky curve scaled by the dust transmittance last seen in daylight,
//! minus the essential load and the station charging ArcadiaOS itself has
//! scheduled. ArcadiaOS uses it to keep optional charging (top-ups and
//! parking charges) out of nights and storms that would drain the bank, and
//! to top robots up while the bank is full and solar would be curtailed.
//! Charging a robot needs to work is never held back; when the bank is empty
//! the delivered fraction slows it, and the energy policy plans for that.
//!
//! Robot heaters run on cold nights and add to idle drain. The cold window is
//! an assumption ([`Config::cold_from_phase`] to [`Config::cold_to_phase`]),
//! deliberately wider than the reference simulator's.

use crate::config::Config;
use crate::world::WorldModel;
use arcadia_protocol::{PowerSystem, WeatherCondition};

/// Forecast step, s.
const STEP_S: f64 = 900.0;

/// Transmittance below this clear-sky fraction of the peak is not estimated.
const MIN_SUN_FRAC: f64 = 0.1;

/// The site's power system and day length.
#[derive(Debug, Clone, PartialEq)]
pub struct PowerModel {
    /// From the scenario.
    pub system: PowerSystem,
    /// Sol length, s.
    pub sol_s: f64,
    /// Dust transmittance last measured in daylight.
    transmittance: f64,
}

/// What the forecast expects of the bank.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BankForecast {
    /// Lowest storage over the window, Wh.
    pub min_wh: f64,
    /// Storage at the end of the window, Wh.
    pub end_wh: f64,
    /// Whether demand outruns supply and storage somewhere in the window.
    pub deficit: bool,
}

impl PowerModel {
    /// A model of `system` on a site with sols of `sol_s` seconds.
    pub fn new(system: PowerSystem, sol_s: f64) -> Self {
        Self {
            system,
            sol_s: if sol_s > 0.0 { sol_s } else { 88_775.0 },
            transmittance: 1.0,
        }
    }

    /// Sol phase of time `t`, in `[0, 1)`.
    pub fn phase(&self, t: f64) -> f64 {
        (t / self.sol_s).rem_euclid(1.0)
    }

    /// Whether the sun is up at `t`.
    pub fn daylight(&self, t: f64) -> bool {
        let p = self.phase(t);
        (0.25..0.75).contains(&p)
    }

    /// Clear-sky solar output at `t`, W.
    pub fn clear_sky_w(&self, t: f64) -> f64 {
        let p = self.phase(t);
        if !(0.25..0.75).contains(&p) {
            return 0.0;
        }
        self.system.solar_peak_w.max(0.0) * (std::f64::consts::PI * (p - 0.25) / 0.5).sin()
    }

    /// The next sunrise strictly after `t`.
    pub fn next_sunrise(&self, t: f64) -> f64 {
        let sol = (t / self.sol_s).floor();
        let today = (sol + 0.25) * self.sol_s;
        if today > t { today } else { today + self.sol_s }
    }

    /// Take in the latest reports: daylight `power` events measure the dust
    /// transmittance, and a `weather` report of clear sky resets it.
    pub fn observe(&mut self, world: &WorldModel) {
        if let Some(p) = &world.power {
            let clear = self.clear_sky_w(p.t);
            if p.daylight && clear > MIN_SUN_FRAC * self.system.solar_peak_w.max(1.0) {
                self.transmittance = (p.solar_w / clear).clamp(0.0, 1.0);
            }
        }
        if let Some(w) = &world.weather
            && w.condition == WeatherCondition::Clear
            && world.power.as_ref().is_none_or(|p| p.t <= w.t)
        {
            self.transmittance = 1.0;
        }
    }

    /// Expected supply at `t`, W.
    pub fn supply_w(&self, t: f64) -> f64 {
        self.system.fission_w.max(0.0) + self.clear_sky_w(t) * self.transmittance
    }

    /// Forecast the bank from the latest report until `until`, with
    /// `charging_w` of station charging throughout (`extra_w` more from now on).
    pub fn forecast(
        &self,
        world: &WorldModel,
        charging_w: f64,
        extra_w: f64,
        until: f64,
    ) -> Option<BankForecast> {
        let p = world.power.as_ref()?;
        let cap = p.storage_capacity_wh.max(0.0);
        let mut storage = p.storage_wh.clamp(0.0, cap);
        let mut t = p.t;
        let mut min_wh = storage;
        let mut deficit = false;
        let essential = self.system.essential_load_w.max(0.0);
        while t < until {
            let dt = STEP_S.min(until - t);
            let load = essential + charging_w + if t >= world.now { extra_w } else { 0.0 };
            let net_wh = (self.supply_w(t + dt / 2.0) - load) * dt / 3600.0;
            storage += net_wh;
            if storage < 0.0 {
                storage = 0.0;
                deficit = true;
            }
            storage = storage.min(cap);
            if t >= world.now {
                min_wh = min_wh.min(storage);
            }
            t += dt;
        }
        Some(BankForecast {
            min_wh,
            end_wh: storage,
            deficit,
        })
    }

    /// Whether `extra_w` of optional station charging can start now without
    /// pushing the bank below the floor before the next sunrise (or within
    /// `horizon_s`, whichever is later). Without power reports, yes.
    pub fn optional_charging_ok(
        &self,
        world: &WorldModel,
        charging_w: f64,
        extra_w: f64,
        horizon_s: f64,
        cfg: &Config,
    ) -> bool {
        let Some(p) = world.power.as_ref() else {
            return true;
        };
        let until = self.next_sunrise(world.now).max(world.now + horizon_s);
        let Some(f) = self.forecast(world, charging_w, extra_w, until) else {
            return true;
        };
        !f.deficit && f.min_wh >= cfg.bank_floor_frac * p.storage_capacity_wh
    }

    /// Whether the bank is full and the sun is up: energy that would
    /// otherwise be curtailed is free to take.
    pub fn surplus(&self, world: &WorldModel, charging_w: f64, cfg: &Config) -> bool {
        let Some(p) = world.power.as_ref() else {
            return false;
        };
        let full = p.storage_wh >= cfg.bank_surplus_frac * p.storage_capacity_wh;
        let load = self.system.essential_load_w.max(0.0) + charging_w;
        full && self.daylight(world.now) && self.supply_w(world.now) > load
    }

    /// Expected fraction of rated station power delivered over the next
    /// `duration_s`, with `charging_w` already drawn and `extra_w` added.
    pub fn delivered_fraction(
        &self,
        world: &WorldModel,
        charging_w: f64,
        extra_w: f64,
        duration_s: f64,
    ) -> f64 {
        let now_fraction = world.charge_fraction();
        let Some(f) = self.forecast(world, charging_w, extra_w, world.now + duration_s.max(0.0))
        else {
            return now_fraction;
        };
        if !f.deficit {
            return now_fraction;
        }
        // Part of the session runs from an empty bank: what supply covers.
        let essential = self.system.essential_load_w.max(0.0);
        let load = (charging_w + extra_w).max(1.0);
        let night_supply = (self.system.fission_w.max(0.0) - essential).max(0.0);
        (0.5 + 0.5 * (night_supply / load))
            .clamp(0.1, 1.0)
            .min(now_fraction)
    }

    /// Seconds of the assumed cold window within `[from, to]`.
    pub fn cold_s(&self, from: f64, to: f64, cfg: &Config) -> f64 {
        if to <= from {
            return 0.0;
        }
        let mut total = 0.0;
        let mut sol = (from / self.sol_s).floor() - 1.0;
        while sol * self.sol_s <= to {
            // The window starting this sol runs into the next one.
            let start = (sol + cfg.cold_from_phase) * self.sol_s;
            let end = (sol + 1.0 + cfg.cold_to_phase) * self.sol_s;
            let a = start.max(from);
            let b = end.min(to);
            if b > a {
                total += b - a;
            }
            sol += 1.0;
        }
        total
    }

    /// Whether the assumed cold window covers `t`.
    pub fn cold(&self, t: f64, cfg: &Config) -> bool {
        let p = self.phase(t);
        p >= cfg.cold_from_phase || p < cfg.cold_to_phase
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::tests::{cap, robot, scenario};
    use arcadia_protocol::{CapabilityName, Event, EventKind, PowerMode, SOL_S};

    fn model(mode: PowerMode, fission: f64, solar: f64, cap: f64) -> PowerModel {
        PowerModel::new(
            PowerSystem {
                mode,
                fission_w: fission,
                solar_peak_w: solar,
                storage_capacity_wh: cap,
                essential_load_w: 2000.0,
            },
            SOL_S,
        )
    }

    fn world_with_power(t: f64, storage: f64, cap: f64, solar_w: f64) -> WorldModel {
        let sc = scenario(vec![robot("a", 0.5, vec![cap_spec()])]);
        let mut w = WorldModel::new(&sc);
        w.now = t;
        w.observe(&Event {
            t,
            kind: EventKind::Power {
                solar_w,
                fission_w: 0.0,
                demand_w: 2000.0,
                storage_wh: storage,
                storage_capacity_wh: cap,
                daylight: (0.25..0.75).contains(&((t / SOL_S).rem_euclid(1.0))),
            },
        });
        w
    }

    fn cap_spec() -> arcadia_protocol::CapabilitySpec {
        cap(CapabilityName::Scan, 1.0, 0.1, None)
    }

    #[test]
    fn solar_follows_the_half_sine_and_sunrise_is_found() {
        let m = model(PowerMode::SolarOnly, 0.0, 30_000.0, 150_000.0);
        assert_eq!(m.clear_sky_w(0.1 * SOL_S), 0.0);
        assert!((m.clear_sky_w(0.5 * SOL_S) - 30_000.0).abs() < 1e-6);
        assert!(m.daylight(0.3 * SOL_S) && !m.daylight(0.8 * SOL_S));
        assert!((m.next_sunrise(0.1 * SOL_S) - 0.25 * SOL_S).abs() < 1e-6);
        assert!((m.next_sunrise(0.5 * SOL_S) - 1.25 * SOL_S).abs() < 1e-6);
    }

    #[test]
    fn a_solar_night_with_heavy_charging_is_held_back() {
        let cfg = Config::default();
        let m = model(PowerMode::SolarOnly, 0.0, 30_000.0, 150_000.0);
        // Sunset with a third of the bank: 12 h of night at 2 kW essential.
        let w = world_with_power(0.75 * SOL_S, 50_000.0, 150_000.0, 0.0);
        let f = m.forecast(&w, 0.0, 0.0, m.next_sunrise(w.now)).unwrap();
        assert!(f.min_wh < 50_000.0 - 20_000.0 && !f.deficit, "{f:?}");
        assert!(m.optional_charging_ok(&w, 0.0, 0.0, 0.0, &cfg) || f.min_wh < 37_500.0);
        // Adding 6 kW of charging all night drains it.
        assert!(!m.optional_charging_ok(&w, 0.0, 6000.0, 0.0, &cfg));
        let f = m.forecast(&w, 0.0, 6000.0, m.next_sunrise(w.now)).unwrap();
        assert!(f.deficit);
        assert!(m.delivered_fraction(&w, 0.0, 6000.0, 12.0 * 3600.0) < 1.0);
        assert!(!m.surplus(&w, 0.0, &cfg));
    }

    #[test]
    fn fission_keeps_the_bank_safe_and_full_banks_at_noon_are_surplus() {
        let cfg = Config::default();
        let m = model(PowerMode::Hybrid, 10_000.0, 20_000.0, 100_000.0);
        let w = world_with_power(0.8 * SOL_S, 30_000.0, 100_000.0, 0.0);
        assert!(m.optional_charging_ok(&w, 3000.0, 3000.0, 0.0, &cfg));
        let noon = world_with_power(0.5 * SOL_S, 99_000.0, 100_000.0, 20_000.0);
        let mut m2 = m.clone();
        m2.observe(&noon);
        assert!(m2.surplus(&noon, 0.0, &cfg));
        assert_eq!(m2.delivered_fraction(&noon, 0.0, 6000.0, 3600.0), 1.0);
    }

    #[test]
    fn dust_transmittance_is_measured_in_daylight() {
        let mut m = model(PowerMode::SolarOnly, 0.0, 30_000.0, 150_000.0);
        let w = world_with_power(0.5 * SOL_S, 80_000.0, 150_000.0, 9_000.0);
        m.observe(&w);
        assert!((m.supply_w(0.5 * SOL_S) - 9_000.0).abs() < 1.0);
    }

    #[test]
    fn the_cold_window_spans_midnight() {
        let cfg = Config::default();
        let m = model(PowerMode::Hybrid, 10_000.0, 20_000.0, 100_000.0);
        assert!(m.cold(0.0, &cfg) && m.cold(0.95 * SOL_S, &cfg) && !m.cold(0.5 * SOL_S, &cfg));
        // One full sol contains one whole window: 0.45 sol.
        let s = m.cold_s(0.5 * SOL_S, 1.5 * SOL_S, &cfg);
        assert!((s - 0.45 * SOL_S).abs() < 1e-6, "{s}");
        assert_eq!(m.cold_s(0.4 * SOL_S, 0.6 * SOL_S, &cfg), 0.0);
        assert!((m.cold_s(0.0, 0.1 * SOL_S, &cfg) - 0.1 * SOL_S).abs() < 1e-6);
    }
}
