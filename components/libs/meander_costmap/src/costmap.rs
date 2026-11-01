//! Row-major costs and a separable squared Euclidean distance transform.
use crate::CuCostmapParams;
use cu29::prelude::CuError;
use cu29::prelude::CuResult;
use cu29::units::si::length::meter;
use meander_core::CuPoint2;
use meander_grid::CuCell2;
use meander_grid::CuGridGeometry2;
use meander_grid::CuOccupancy;
use meander_grid::CuOccupancyGrid;

fn square(value: f64) -> f64 {
    value * value
}

/// Traversable cell without inflation.
pub const FREE_SPACE: u8 = 0;
/// Cell inside the inscribed robot radius of an obstacle.
pub const INSCRIBED: u8 = 253;
/// Obstacle cell.
pub const LETHAL: u8 = 254;
/// Unobserved cell under the default unknown policy.
pub const NO_INFORMATION: u8 = 255;

/// Fixed-size costmap with inline distance-transform scratch storage.
#[derive(Debug, Clone)]
pub struct CuCostmap2<const W: usize, const H: usize> {
    geometry: CuGridGeometry2,
    params: CuCostmapParams,
    costs: [[u8; W]; H],
    distances: [[f64; W]; H],
    sites: [usize; H],
    starts: [f64; H],
}

impl<const W: usize, const H: usize> CuCostmap2<W, H> {
    /// Creates an all-unknown costmap, validating geometry and inflation parameters.
    pub fn from_params(geometry: CuGridGeometry2, params: CuCostmapParams) -> CuResult<Self> {
        geometry.validate()?;
        params.validate()?;
        Ok(Self {
            geometry,
            params,
            costs: [[NO_INFORMATION; W]; H],
            distances: [[f64::INFINITY; W]; H],
            sites: [0; H],
            starts: [0.0; H],
        })
    }

    /// Row-major costs: `costs()[y][x]`, matching the occupancy grid layout.
    pub fn costs(&self) -> &[[u8; W]; H] {
        &self.costs
    }

    /// Placement in the world frame.
    pub fn geometry(&self) -> &CuGridGeometry2 {
        &self.geometry
    }

    /// Validated inflation parameters.
    pub fn params(&self) -> &CuCostmapParams {
        &self.params
    }

    /// Cost of a cell, or `None` outside the grid.
    pub fn cost(&self, cell: CuCell2) -> Option<u8> {
        self.costs
            .get(cell.y)
            .and_then(|row| row.get(cell.x))
            .copied()
    }

    /// Cost at a world point, or `None` outside the grid or for nonfinite points.
    pub fn cost_at(&self, point: CuPoint2) -> Option<u8> {
        let r = self.geometry.resolution.get::<meter>();
        let x = libm::floor((point.x - self.geometry.origin.x).get::<meter>() / r);
        let y = libm::floor((point.y - self.geometry.origin.y).get::<meter>() / r);
        if !x.is_finite() || !y.is_finite() || x < 0.0 || y < 0.0 || x >= W as f64 || y >= H as f64
        {
            return None;
        }
        self.cost(CuCell2::new(x as usize, y as usize))
    }

    /// Rebuilds all costs from occupancy, measuring distance between cell centers.
    /// Unknown cells retain `NO_INFORMATION` unless configured as lethal seeds.
    /// A geometry mismatch returns an error before changing the costmap.
    pub fn update_from_occupancy(&mut self, grid: &CuOccupancyGrid<W, H>) -> CuResult<()> {
        if self.geometry != *grid.geometry() {
            return Err(CuError::from(
                "meander-costmap: occupancy and costmap geometries differ",
            ));
        }
        // Horizontal nearest-seed distances, scanned in both directions.
        for (y, row) in grid.cells().iter().enumerate() {
            let mut nearest = None;
            for (x, &cell) in row.iter().enumerate() {
                let seed = cell == CuOccupancy::Occupied
                    || (cell == CuOccupancy::Unknown && self.params.unknown_is_lethal);
                if seed {
                    nearest = Some(x);
                }
                self.distances[y][x] = nearest.map_or(f64::INFINITY, |n| square((x - n) as f64));
            }
            nearest = None;
            for (x, &cell) in row.iter().enumerate().rev() {
                if cell == CuOccupancy::Occupied
                    || (cell == CuOccupancy::Unknown && self.params.unknown_is_lethal)
                {
                    nearest = Some(x);
                }
                if let Some(n) = nearest {
                    self.distances[y][x] = self.distances[y][x].min(square((n - x) as f64));
                }
            }
        }
        for x in 0..W {
            // Lower envelope of parabolas f(q) + (y-q)^2. Each site enters
            // and leaves the envelope once; skip rows with no horizontal seed.
            let mut count = 0;
            for q in 0..H {
                if !self.distances[q][x].is_finite() {
                    continue;
                }
                let mut start = f64::NEG_INFINITY;
                while count > 0 {
                    let p = self.sites[count - 1];
                    start = ((self.distances[q][x] + square(q as f64))
                        - (self.distances[p][x] + square(p as f64)))
                        / (2.0 * (q - p) as f64);
                    if start > self.starts[count - 1] {
                        break;
                    }
                    count -= 1;
                }
                self.sites[count] = q;
                self.starts[count] = if count == 0 { f64::NEG_INFINITY } else { start };
                count += 1;
            }
            let mut active = 0;
            for y in 0..H {
                while active + 1 < count && self.starts[active + 1] < y as f64 {
                    active += 1;
                }
                let distance = if count == 0 {
                    f64::INFINITY
                } else {
                    let q = self.sites[active];
                    libm::sqrt(self.distances[q][x] + square(y.abs_diff(q) as f64))
                        * self.geometry.resolution.get::<meter>()
                };
                self.costs[y][x] = if grid.cells()[y][x] == CuOccupancy::Unknown
                    && !self.params.unknown_is_lethal
                {
                    NO_INFORMATION
                } else if distance == 0.0 {
                    LETHAL
                } else if distance <= self.params.robot_radius.get::<meter>() {
                    INSCRIBED
                } else if distance <= self.params.inflation_radius.get::<meter>() {
                    ((INSCRIBED - 1) as f64
                        * libm::exp(
                            -self.params.cost_scaling_factor
                                * (distance - self.params.robot_radius.get::<meter>()),
                        )) as u8
                } else {
                    FREE_SPACE
                };
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::units::si::f64::Length;
    use meander_grid::CuOccupancyUpdateParams;

    fn point(x: f64, y: f64) -> CuPoint2 {
        CuPoint2::new(Length::new::<meter>(x), Length::new::<meter>(y))
    }

    fn grid() -> CuOccupancyGrid<17, 13> {
        let geometry = CuGridGeometry2::new(Length::new::<meter>(0.1), point(-0.5, -0.3)).unwrap();
        let mut grid =
            CuOccupancyGrid::from_params(geometry, CuOccupancyUpdateParams::default()).unwrap();
        grid.fill(CuOccupancy::Free);
        grid
    }

    #[test]
    fn test_single_obstacle_and_accessors() {
        let mut grid = grid();
        grid.set(CuCell2::new(6, 6), CuOccupancy::Occupied).unwrap();
        let mut map =
            CuCostmap2::from_params(*grid.geometry(), CuCostmapParams::default()).unwrap();
        map.update_from_occupancy(&grid).unwrap();
        assert_eq!(map.cost(CuCell2::new(6, 6)), Some(LETHAL));
        assert_eq!(map.costs()[6][8], INSCRIBED);
        assert!(map.costs()[6][9] > map.costs()[6][10]);
        assert!(map.costs()[6][10] > map.costs()[6][11]);
        assert_eq!(map.costs()[6][12], FREE_SPACE);
        assert_eq!(map.cost_at(point(0.15, 0.35)), Some(LETHAL));
        for p in [
            point(-0.51, 0.0),
            point(1.2, 0.0),
            point(f64::NAN, 0.0),
            point(f64::INFINITY, 0.0),
        ] {
            assert_eq!(map.cost_at(p), None);
        }
        assert_eq!(map.cost(CuCell2::new(17, 0)), None);
        assert_eq!(map.cost(CuCell2::new(0, 13)), None);
        assert_eq!(map.geometry(), grid.geometry());
    }

    #[test]
    fn test_exact_distances_against_exhaustive_reference_and_reset() {
        let mut grid = grid();
        let obstacles = [(0, 0), (7, 3), (2, 11), (16, 12), (12, 8)];
        for (x, y) in obstacles {
            grid.set(CuCell2::new(x, y), CuOccupancy::Occupied).unwrap();
        }
        let params = CuCostmapParams::default();
        let mut map = CuCostmap2::from_params(*grid.geometry(), params.clone()).unwrap();
        map.update_from_occupancy(&grid).unwrap();
        for y in 0usize..13 {
            for x in 0usize..17 {
                let d = obstacles
                    .iter()
                    .map(|&(ox, oy)| {
                        libm::sqrt((x.abs_diff(ox).pow(2) + y.abs_diff(oy).pow(2)) as f64) * 0.1
                    })
                    .fold(f64::INFINITY, f64::min);
                let expected = if d == 0.0 {
                    LETHAL
                } else if d <= 0.2 {
                    INSCRIBED
                } else if d <= 0.55 {
                    (252.0 * libm::exp(-3.0 * (d - 0.2))) as u8
                } else {
                    FREE_SPACE
                };
                assert_eq!(map.costs()[y][x], expected, "cell ({x}, {y})");
            }
        }
        grid.fill(CuOccupancy::Free);
        map.update_from_occupancy(&grid).unwrap();
        assert!(map.costs().iter().flatten().all(|&c| c == FREE_SPACE));
    }

    #[test]
    fn test_unknown_policy() {
        let mut grid = grid();
        grid.set(CuCell2::new(5, 5), CuOccupancy::Unknown).unwrap();
        grid.set(CuCell2::new(6, 5), CuOccupancy::Occupied).unwrap();
        for lethal in [false, true] {
            let params = CuCostmapParams {
                unknown_is_lethal: lethal,
                ..Default::default()
            };
            let mut map = CuCostmap2::from_params(*grid.geometry(), params).unwrap();
            map.update_from_occupancy(&grid).unwrap();
            assert_eq!(
                map.costs()[5][5],
                if lethal { LETHAL } else { NO_INFORMATION }
            );
            assert_eq!(
                map.costs()[5][3],
                if lethal {
                    INSCRIBED
                } else {
                    (252.0 * libm::exp(-3.0 * (0.3 - 0.2))) as u8
                }
            );
        }
    }

    #[test]
    fn test_geometry_mismatch_preserves_costs_and_empty_dimensions() {
        let grid = grid();
        for geometry in [
            CuGridGeometry2::new(Length::new::<meter>(0.2), grid.geometry().origin).unwrap(),
            CuGridGeometry2::new(grid.geometry().resolution, point(0.0, 0.0)).unwrap(),
        ] {
            let mut map = CuCostmap2::from_params(geometry, CuCostmapParams::default()).unwrap();
            assert!(map.update_from_occupancy(&grid).is_err());
            assert!(map.costs().iter().flatten().all(|&c| c == NO_INFORMATION));
        }
        let mut map =
            CuCostmap2::<0, 0>::from_params(*grid.geometry(), CuCostmapParams::default()).unwrap();
        let empty =
            CuOccupancyGrid::from_params(*grid.geometry(), CuOccupancyUpdateParams::default())
                .unwrap();
        map.update_from_occupancy(&empty).unwrap();
        assert_eq!(map.cost_at(point(0.0, 0.0)), None);
    }
}
