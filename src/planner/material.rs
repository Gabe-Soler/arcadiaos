//! Material balance: where spoil goes and where fill comes from.
//!
//! Stockpile sites are cells in a ring just outside the pad (never berm,
//! station, depot or berm borrow cells), inside the base station's coverage
//! when there are enough such cells, so loading never strands a hauler out
//! of contact. Dig spoil that does not fit in the digger's own
//! payload is cast onto the nearest site with room; hauls for fill and berm
//! load from the cheapest site, taking stockpiled spoil first and borrowing
//! native regolith (to a limited depth) when spoil runs short. The ledger
//! tracks cut, reuse and borrow so the plan's cut/fill balance is visible.

use crate::config::Config;
use crate::coverage::Coverage;
use crate::geom;
use crate::registry::Registry;
use crate::world::WorldModel;
use arcadia_protocol::{GridCoord, GridRect};
use std::collections::BTreeSet;

/// A stockpile (and borrow) site.
#[derive(Debug, Clone, PartialEq)]
pub struct Pile {
    /// Cell.
    pub at: GridCoord,
    /// Most spoil it should hold, kg.
    pub capacity_kg: f64,
    /// Spoil currently on it (estimated), kg.
    pub spoil_kg: f64,
    /// Native regolith borrowed from it so far, kg.
    pub borrowed_kg: f64,
    /// Most native regolith it may give, kg.
    pub max_borrow_kg: f64,
}

/// Where a haul loads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Source {
    /// Index of the pile.
    pub pile: usize,
    /// Cell to load at.
    pub at: GridCoord,
}

/// The material ledger.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Material {
    /// Stockpile and borrow sites, nearest the pad first.
    pub piles: Vec<Pile>,
    /// Spoil cast onto piles so far, kg.
    pub spoil_kg: f64,
    /// Spoil cast straight into fill or berm cells, kg.
    pub side_cast_kg: f64,
    /// Spoil reloaded from piles, kg.
    pub reused_kg: f64,
    /// Native regolith borrowed, kg.
    pub borrowed_kg: f64,
}

/// Penalty for borrowing native regolith instead of reusing spoil, s per load.
const BORROW_PENALTY_S: f64 = 45.0;

impl Material {
    /// Choose pile sites around `pad`, avoiding `blocked` cells (the berm and
    /// its borrow cells) and infrastructure, inside base-station coverage
    /// when at least eight such sites exist within the search.
    pub fn new(pad: &GridRect, blocked: &[GridCoord], world: &WorldModel, cfg: &Config) -> Self {
        let mut blocked: BTreeSet<(u32, u32)> = blocked.iter().map(|c| (c.x, c.y)).collect();
        for s in &world.stations {
            blocked.insert((s.position.x, s.position.y));
        }
        for d in &world.depots {
            blocked.insert((d.position.x, d.position.y));
        }
        let site = &world.site;
        let cov = Coverage::new(&world.base_station, site.cell_size_m, &[]);
        let mut picks: Vec<(u32, GridCoord)> = Vec::new();
        // Widen the search ring until there are enough covered sites.
        for max_ring in [2u32, 4, 8, 16] {
            picks.clear();
            let lo_x = pad.min.x.saturating_sub(max_ring);
            let lo_y = pad.min.y.saturating_sub(max_ring);
            let hi_x = (pad.max.x + max_ring).min(site.width.saturating_sub(1));
            let hi_y = (pad.max.y + max_ring).min(site.height.saturating_sub(1));
            for y in lo_y..=hi_y {
                for x in lo_x..=hi_x {
                    let c = GridCoord { x, y };
                    let ring = geom::ring_distance(c, pad);
                    if ring >= 1 && ring <= max_ring && !blocked.contains(&(x, y)) {
                        picks.push((ring, c));
                    }
                }
            }
            if picks.iter().filter(|(_, c)| cov.covers(*c)).count() >= 8 {
                break;
            }
        }
        if picks.iter().filter(|(_, c)| cov.covers(*c)).count() >= 8 {
            picks.retain(|(_, c)| cov.covers(*c));
        }
        picks.sort_by_key(|&(ring, c)| (ring, c.y, c.x));
        let area = site.cell_area_m2();
        let piles = picks
            .into_iter()
            .map(|(_, at)| {
                let density = site.density(at, cfg);
                Pile {
                    at,
                    capacity_kg: cfg.stockpile_max_rise_m * density * area,
                    spoil_kg: 0.0,
                    borrowed_kg: 0.0,
                    max_borrow_kg: cfg.borrow_max_depth_m * density * area,
                }
            })
            .collect();
        Self {
            piles,
            ..Self::default()
        }
    }

    /// Pile for `kg` of spoil dug at `near`: the nearest with room, leaning
    /// slightly towards sites close to where material is wanted (`wanted`).
    pub fn spoil_site(&self, near: GridCoord, kg: f64, wanted: &[GridCoord]) -> Option<usize> {
        let pull = |at: GridCoord| {
            wanted
                .iter()
                .map(|&w| geom::dist(at, w))
                .fold(f64::INFINITY, f64::min)
        };
        self.piles
            .iter()
            .enumerate()
            .filter(|(_, p)| p.capacity_kg - p.spoil_kg >= kg.min(p.capacity_kg))
            .map(|(i, p)| {
                let pull = pull(p.at);
                let pull = if pull.is_finite() { 0.3 * pull } else { 0.0 };
                (geom::dist(near, p.at) + pull, i)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)))
            .map(|(_, i)| i)
    }

    /// Record spoil cast onto pile `i`.
    pub fn record_spoil(&mut self, i: usize, kg: f64) {
        if let Some(p) = self.piles.get_mut(i) {
            p.spoil_kg += kg.max(0.0);
            self.spoil_kg += kg.max(0.0);
        }
    }

    /// Cheapest pile for robot `r` at `from` to load `kg` and deliver to `dest`.
    pub fn source(
        &self,
        reg: &Registry,
        r: usize,
        from: GridCoord,
        dest: GridCoord,
        kg: f64,
    ) -> Option<Source> {
        self.piles
            .iter()
            .enumerate()
            .filter(|(_, p)| p.spoil_kg + (p.max_borrow_kg - p.borrowed_kg).max(0.0) >= kg * 0.99)
            .map(|(i, p)| {
                let borrow = if p.spoil_kg >= kg { 0.0 } else { 1.0 };
                let cost = reg.travel_s(r, from, p.at)
                    + reg.travel_s(r, p.at, dest)
                    + BORROW_PENALTY_S * borrow;
                (cost, i)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)))
            .map(|(_, i)| Source {
                pile: i,
                at: self.piles[i].at,
            })
    }

    /// Record `kg` loaded from pile `i`: spoil first, then borrowed regolith.
    pub fn record_load(&mut self, i: usize, kg: f64) {
        let Some(p) = self.piles.get_mut(i) else {
            return;
        };
        let kg = kg.max(0.0);
        let from_spoil = kg.min(p.spoil_kg);
        p.spoil_kg -= from_spoil;
        p.borrowed_kg += kg - from_spoil;
        self.reused_kg += from_spoil;
        self.borrowed_kg += kg - from_spoil;
    }

    /// Index of the pile at `c`, if any.
    pub fn pile_at(&self, c: GridCoord) -> Option<usize> {
        self.piles.iter().position(|p| p.at == c)
    }

    /// Spoil available on piles, kg.
    pub fn stock_kg(&self) -> f64 {
        self.piles.iter().map(|p| p.spoil_kg).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::tests::{cap, robot, scenario};
    use arcadia_protocol::CapabilityName;

    fn pad() -> GridRect {
        GridRect {
            min: GridCoord { x: 3, y: 3 },
            max: GridCoord { x: 5, y: 5 },
        }
    }

    #[test]
    fn piles_ring_the_pad_and_avoid_berm_and_infrastructure() {
        let sc = scenario(vec![robot(
            "h",
            0.5,
            vec![cap(CapabilityName::Haul, 3.0, 0.01, Some(400.0))],
        )]);
        let world = WorldModel::new(&sc);
        let berm = [GridCoord { x: 4, y: 7 }];
        let m = Material::new(&pad(), &berm, &world, &Config::default());
        assert!(!m.piles.is_empty());
        for p in &m.piles {
            let ring = geom::ring_distance(p.at, &pad());
            assert!((1..=2).contains(&ring), "{:?}", p.at);
            assert_ne!(p.at, berm[0]);
            assert_ne!(p.at, GridCoord { x: 0, y: 0 }, "station cell");
        }
        // Nearest ring first.
        assert_eq!(geom::ring_distance(m.piles[0].at, &pad()), 1);
    }

    #[test]
    fn spoil_and_loads_balance_with_borrow_fallback() {
        let sc = scenario(vec![robot(
            "h",
            0.5,
            vec![cap(CapabilityName::Haul, 3.0, 0.01, Some(400.0))],
        )]);
        let world = WorldModel::new(&sc);
        let reg = Registry::new(&sc);
        let mut m = Material::new(&pad(), &[], &world, &Config::default());
        let near = GridCoord { x: 3, y: 3 };
        let i = m.spoil_site(near, 300.0, &[]).unwrap();
        assert_eq!(geom::ring_distance(m.piles[i].at, &pad()), 1);
        assert!(geom::dist(near, m.piles[i].at) <= 1.5);
        m.record_spoil(i, 300.0);
        assert_eq!(m.stock_kg(), 300.0);

        // A haul prefers the pile holding spoil when distances are similar.
        let dest = GridCoord { x: 4, y: 4 };
        let src = m.source(&reg, 0, m.piles[i].at, dest, 250.0).unwrap();
        assert_eq!(src.pile, i);
        m.record_load(i, 400.0);
        assert_eq!(m.reused_kg, 300.0);
        assert_eq!(m.borrowed_kg, 100.0);
        assert_eq!(m.pile_at(m.piles[i].at), Some(i));
    }
}
