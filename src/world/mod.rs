//! The world model: everything ArcadiaOS believes about the site, built only
//! from `hello` and observations.
//!
//! - [`site`]: per-cell knowledge (prior, scans with timestamps, modelled
//!   surface state and elevation).
//! - [`fleet`]: robot specs and latest states.
//! - Infrastructure (stations, depots, the comms base station), the site
//!   power and weather reports, the Earth link, and the authority policy in
//!   force.

pub mod fleet;
pub mod site;

use arcadia_protocol::{
    BaseStation, ChargeStation, CommsCause, Depot, Event, EventKind, Link, Policy, Scenario,
    WeatherCondition,
};
use fleet::Fleet;
use site::SiteKnowledge;

/// Latest site power report.
#[derive(Debug, Clone, PartialEq)]
pub struct PowerStatus {
    /// When it was reported.
    pub t: f64,
    /// Solar output, W.
    pub solar_w: f64,
    /// Fission output, W.
    pub fission_w: f64,
    /// Total demand including station charging, W.
    pub demand_w: f64,
    /// Energy in the site battery bank, Wh.
    pub storage_wh: f64,
    /// Bank capacity, Wh.
    pub storage_capacity_wh: f64,
    /// Whether the sun is up.
    pub daylight: bool,
}

/// Latest weather report.
#[derive(Debug, Clone, PartialEq)]
pub struct WeatherStatus {
    /// When it was reported.
    pub t: f64,
    /// Clear or dust storm.
    pub condition: WeatherCondition,
    /// Dust optical depth.
    pub optical_depth: f64,
    /// Air temperature, °C.
    pub temperature_c: f64,
}

/// Latest Earth link report.
#[derive(Debug, Clone, PartialEq)]
pub struct CommsStatus {
    /// When it was reported.
    pub t: f64,
    /// Up or down.
    pub link: Link,
    /// Why the link is in this state.
    pub cause: CommsCause,
    /// One-way light time, s.
    pub one_way_delay_s: f64,
    /// When the link is expected back, if it is down and the site knows.
    pub expected_restore_t: Option<f64>,
}

/// Everything ArcadiaOS believes about the world.
#[derive(Debug, Clone, PartialEq)]
pub struct WorldModel {
    /// Mission start time.
    pub start_t: f64,
    /// When the run is cut off.
    pub time_limit_t: f64,
    /// Length of a sol, s.
    pub sol_length_s: f64,
    /// Time of the latest observation.
    pub now: f64,
    /// Per-cell knowledge.
    pub site: SiteKnowledge,
    /// The crew.
    pub fleet: Fleet,
    /// Charging stations.
    pub stations: Vec<ChargeStation>,
    /// Maintenance depots.
    pub depots: Vec<Depot>,
    /// The site comms hub: robots in its range (or a relay's) can be commanded.
    pub base_station: BaseStation,
    /// Authority policy in force: the scenario's, replaced by each policy update.
    pub policy: Policy,
    /// Latest power report.
    pub power: Option<PowerStatus>,
    /// Latest weather report.
    pub weather: Option<WeatherStatus>,
    /// Latest comms report.
    pub comms: Option<CommsStatus>,
}

impl WorldModel {
    /// Build the initial model from the scenario in `hello`.
    pub fn new(scenario: &Scenario) -> Self {
        Self {
            start_t: scenario.start_t,
            time_limit_t: scenario.time_limit_t,
            sol_length_s: scenario.sol_length_s,
            now: scenario.start_t,
            site: SiteKnowledge::from_prior(&scenario.site),
            fleet: Fleet::new(&scenario.robots),
            stations: scenario.stations.clone(),
            depots: scenario.depots.clone(),
            base_station: scenario.base_station.clone(),
            policy: scenario.policy.clone(),
            power: None,
            weather: None,
            comms: None,
        }
    }

    /// Take in an environment report; command events are handled elsewhere.
    pub fn observe(&mut self, e: &Event) {
        match &e.kind {
            EventKind::Power {
                solar_w,
                fission_w,
                demand_w,
                storage_wh,
                storage_capacity_wh,
                daylight,
            } => {
                self.power = Some(PowerStatus {
                    t: e.t,
                    solar_w: *solar_w,
                    fission_w: *fission_w,
                    demand_w: *demand_w,
                    storage_wh: *storage_wh,
                    storage_capacity_wh: *storage_capacity_wh,
                    daylight: *daylight,
                });
            }
            EventKind::Weather {
                condition,
                optical_depth,
                temperature_c,
            } => {
                self.weather = Some(WeatherStatus {
                    t: e.t,
                    condition: *condition,
                    optical_depth: *optical_depth,
                    temperature_c: *temperature_c,
                });
            }
            EventKind::Comms {
                link,
                cause,
                one_way_delay_s,
                expected_restore_t,
            } => {
                self.comms = Some(CommsStatus {
                    t: e.t,
                    link: *link,
                    cause: *cause,
                    one_way_delay_s: *one_way_delay_s,
                    expected_restore_t: *expected_restore_t,
                });
            }
            _ => {}
        }
    }

    /// Whether the Earth link is up, as last reported (up before any report).
    pub fn link_up(&self) -> bool {
        self.comms.as_ref().is_none_or(|c| c.link == Link::Up)
    }

    /// One-way light time to Earth, as last reported, s.
    pub fn one_way_delay_s(&self, assumed_s: f64) -> f64 {
        self.comms.as_ref().map_or(assumed_s, |c| c.one_way_delay_s)
    }

    /// Expected fraction of rated station power actually delivered: 1 unless
    /// the bank is empty and demand outruns supply.
    pub fn charge_fraction(&self) -> f64 {
        let Some(p) = &self.power else {
            return 1.0;
        };
        let empty = p.storage_wh <= 0.01 * p.storage_capacity_wh.max(1.0);
        let supply = p.solar_w + p.fission_w;
        if empty && p.demand_w > supply && p.demand_w > 0.0 {
            (supply / p.demand_w).clamp(0.1, 1.0)
        } else {
            1.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::tests::{cap, robot, scenario};
    use arcadia_protocol::CapabilityName;

    fn event(kind: EventKind) -> Event {
        Event { t: 5.0, kind }
    }

    #[test]
    fn keeps_the_latest_environment_reports() {
        let sc = scenario(vec![robot(
            "a",
            0.5,
            vec![cap(CapabilityName::Scan, 1.0, 0.1, None)],
        )]);
        let mut w = WorldModel::new(&sc);
        assert_eq!(w.fleet.len(), 1);
        assert_eq!(w.charge_fraction(), 1.0);
        w.observe(&event(EventKind::Weather {
            condition: WeatherCondition::DustStorm,
            optical_depth: 3.0,
            temperature_c: -60.0,
        }));
        w.observe(&event(EventKind::Comms {
            link: Link::Down,
            cause: CommsCause::Outage,
            one_way_delay_s: 700.0,
            expected_restore_t: None,
        }));
        assert_eq!(
            w.weather.as_ref().unwrap().condition,
            WeatherCondition::DustStorm
        );
        assert_eq!(w.comms.as_ref().unwrap().link, Link::Down);

        // A full bank delivers everything; an empty one under heavy demand throttles.
        let power = |storage_wh: f64, demand_w: f64| {
            event(EventKind::Power {
                solar_w: 0.0,
                fission_w: 8000.0,
                demand_w,
                storage_wh,
                storage_capacity_wh: 100_000.0,
                daylight: false,
            })
        };
        w.observe(&power(50_000.0, 16_000.0));
        assert_eq!(w.charge_fraction(), 1.0);
        w.observe(&power(0.0, 16_000.0));
        assert!((w.charge_fraction() - 0.5).abs() < 1e-12);
        assert_eq!(w.power.as_ref().unwrap().t, 5.0);
    }
}
