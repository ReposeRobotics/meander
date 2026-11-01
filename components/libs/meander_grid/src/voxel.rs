//! Fixed-capacity classified and probabilistic occupancy storage.

use crate::geometry::{CuCell2, CuCell3, CuGridGeometry3};
use crate::occupancy::{CuOccupancy, CuOccupancyGrid};
use crate::raycast::{cell_of, traverse};
use crate::update::{CuOccupancyUpdateParams, UpdateModel};
use cu29::prelude::*;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use meander_core::CuPoint3;

/// A fixed-size 3D voxel grid of `W × H × D` voxels (`x`, `y`, `z`).
///
/// Every voxel starts at zero log-odds (probability 0.5), classified Unknown.
/// Log-odds use f32 storage; thresholds are compared in the same precision.
#[derive(Debug, Clone, PartialEq)]
pub struct CuVoxelGrid<const W: usize, const H: usize, const D: usize> {
    geometry: CuGridGeometry3,
    params: CuOccupancyUpdateParams,
    model: UpdateModel,
    log_odds: [[[f32; W]; H]; D],
    cells: [[[CuOccupancy; W]; H]; D],
}

impl<const W: usize, const H: usize, const D: usize> CuVoxelGrid<W, H, D> {
    /// Creates an all-unknown grid, validating the update parameters.
    pub fn from_params(
        geometry: CuGridGeometry3,
        params: CuOccupancyUpdateParams,
    ) -> CuResult<Self> {
        params.validate()?;
        let model = UpdateModel::new(&params);
        Ok(Self {
            geometry,
            params,
            model,
            log_odds: [[[0.0; W]; H]; D],
            cells: [[[CuOccupancy::Unknown; W]; H]; D],
        })
    }

    /// Validated update parameters.
    pub fn params(&self) -> &CuOccupancyUpdateParams {
        &self.params
    }

    /// Dimensionless log-odds, or None outside. Zero represents the 0.5 prior.
    pub fn log_odds(&self, cell: CuCell3) -> Option<f64> {
        self.contains(cell)
            .then(|| self.log_odds[cell.z][cell.y][cell.x] as f64)
    }

    /// Dimensionless probability, or None outside; unobserved cells have probability 0.5.
    pub fn probability(&self, cell: CuCell3) -> Option<f64> {
        self.log_odds(cell).map(crate::update::sigmoid)
    }

    /// Borrowed raw dimensionless f32 log-odds, in the same order as cells().
    pub fn log_odds_rows(&self) -> &[[[f32; W]; H]; D] {
        &self.log_odds
    }

    /// The grid's placement in the world.
    pub fn geometry(&self) -> &CuGridGeometry3 {
        &self.geometry
    }

    /// The voxel containing a world point, or `None` outside the grid.
    pub fn cell_of(&self, point: CuPoint3) -> Option<CuCell3> {
        cell_of(self.geometry.grid_coords(point), [W, H, D]).map(|[x, y, z]| CuCell3::new(x, y, z))
    }

    /// World position of a voxel's center, or `None` outside the grid.
    pub fn cell_center(&self, cell: CuCell3) -> Option<CuPoint3> {
        self.contains(cell).then(|| self.geometry.center(cell))
    }

    /// Whether `cell` lies inside the grid.
    pub fn contains(&self, cell: CuCell3) -> bool {
        cell.x < W && cell.y < H && cell.z < D
    }

    /// The state of `cell`, or `None` outside the grid.
    pub fn get(&self, cell: CuCell3) -> Option<CuOccupancy> {
        self.cells
            .get(cell.z)
            .and_then(|layer| layer.get(cell.y))
            .and_then(|row| row.get(cell.x))
            .copied()
    }

    /// The state of the voxel containing a world point, or `None` outside.
    pub fn occupancy_at(&self, point: CuPoint3) -> Option<CuOccupancy> {
        self.cell_of(point).and_then(|cell| self.get(cell))
    }

    /// Sets log-odds to the clamp extreme (Free/Occupied) or zero (Unknown).
    /// Sets the state of `cell`; errors outside the grid.
    pub fn set(&mut self, cell: CuCell3, state: CuOccupancy) -> CuResult<()> {
        let slot = self
            .cells
            .get_mut(cell.z)
            .and_then(|layer| layer.get_mut(cell.y))
            .and_then(|row| row.get_mut(cell.x))
            .ok_or_else(|| CuError::from("meander-grid: voxel outside the voxel grid"))?;
        *slot = state;
        self.log_odds[cell.z][cell.y][cell.x] = self.model.value(state);
        Ok(())
    }

    /// Uses clamp extremes for Free/Occupied and zero for Unknown.
    /// Sets every voxel to `state`.
    pub fn fill(&mut self, state: CuOccupancy) {
        for row in self.log_odds.iter_mut().flatten() {
            row.fill(self.model.value(state));
        }
        for row in self.cells.iter_mut().flatten() {
            row.fill(state);
        }
    }

    /// Layered voxel storage: `cells()[z][y][x]`.
    pub fn cells(&self) -> &[[[CuOccupancy; W]; H]; D] {
        &self.cells
    }

    /// Inserts a sensor return: voxels from `origin` up to (not including)
    /// the voxel of `hit` receive miss evidence; the hit voxel receives hit evidence if it
    /// is inside the grid. Errors if `origin` is outside the grid.
    pub fn insert_ray(&mut self, origin: CuPoint3, hit: CuPoint3) -> CuResult<()> {
        let hit_cell = self.cell_of(hit);
        self.trace_free(origin, hit, hit_cell)?;
        if let Some(cell) = hit_cell {
            let state = self
                .model
                .add(&mut self.log_odds[cell.z][cell.y][cell.x], self.model.hit);
            self.cells[cell.z][cell.y][cell.x] = state;
        }
        Ok(())
    }

    /// Clears a ray without a return (max-range reading): every voxel from
    /// `origin` to `end` receives miss evidence. Errors if `origin` is outside the grid.
    pub fn clear_ray(&mut self, origin: CuPoint3, end: CuPoint3) -> CuResult<()> {
        self.trace_free(origin, end, None)
    }

    /// Flattens the voxels whose centers lie in `[z_min, z_max]` into `out`:
    /// a column is occupied if any voxel in the band is occupied, otherwise
    /// free if any is free, otherwise unknown. Each voxel uses this grid's
    /// thresholds; the target receives classifications at its clamp extremes.
    ///
    /// `out` must share this grid's resolution and `x`/`y` origin; a band
    /// that contains no voxel center makes every column unknown.
    pub fn project_into(
        &self,
        z_min: Length,
        z_max: Length,
        out: &mut CuOccupancyGrid<W, H>,
    ) -> CuResult<()> {
        if *out.geometry() != self.geometry.xy() {
            return Err(CuError::from(
                "meander-grid: projection target must share resolution and x/y origin",
            ));
        }
        let (lo, hi) = (z_min.get::<meter>(), z_max.get::<meter>());
        if !lo.is_finite() || !hi.is_finite() || lo > hi {
            return Err(CuError::from(
                "meander-grid: projection band needs finite 'z_min' <= 'z_max'",
            ));
        }
        // Voxel z has its center inside the band iff lo <= center <= hi.
        let r = self.geometry.resolution.get::<meter>();
        let oz = self.geometry.origin.z.get::<meter>();
        let first = libm::ceil((lo - oz) / r - 0.5).max(0.0);
        let last = libm::floor((hi - oz) / r - 0.5).min(D as f64 - 1.0);
        let band = (first <= last).then_some((first as usize, last as usize));
        for y in 0..H {
            for x in 0..W {
                let mut state = CuOccupancy::Unknown;
                if let Some((z0, z1)) = band {
                    for z in z0..=z1 {
                        match self.cells[z][y][x] {
                            CuOccupancy::Occupied => {
                                state = CuOccupancy::Occupied;
                                break;
                            }
                            CuOccupancy::Free => state = CuOccupancy::Free,
                            CuOccupancy::Unknown => {}
                        }
                    }
                }
                out.set(CuCell2::new(x, y), state)?;
            }
        }
        Ok(())
    }

    fn trace_free(
        &mut self,
        origin: CuPoint3,
        end: CuPoint3,
        keep: Option<CuCell3>,
    ) -> CuResult<()> {
        let cells = &mut self.cells;
        let log_odds = &mut self.log_odds;
        let model = &self.model;
        let inside = traverse(
            self.geometry.grid_coords(origin),
            self.geometry.grid_coords(end),
            [W, H, D],
            |[x, y, z]| {
                if keep != Some(CuCell3::new(x, y, z)) {
                    cells[z][y][x] = model.add(&mut log_odds[z][y][x], model.miss);
                }
            },
        );
        if inside {
            Ok(())
        } else {
            Err(CuError::from(
                "meander-grid: ray origin outside the voxel grid",
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::CuGridGeometry2;
    use meander_core::CuPoint2;

    fn m(v: f64) -> Length {
        Length::new::<meter>(v)
    }
    fn p(x: f64, y: f64, z: f64) -> CuPoint3 {
        CuPoint3::new(m(x), m(y), m(z))
    }

    fn geometry() -> CuGridGeometry3 {
        // 0.1 m voxels from the origin: 0.8 × 0.8 × 0.4 m.
        CuGridGeometry3::new(m(0.1), p(0.0, 0.0, 0.0)).unwrap()
    }

    #[test]
    fn test_world_voxel_round_trip() {
        let g = CuVoxelGrid::<8, 8, 4>::from_params(geometry(), CuOccupancyUpdateParams::default())
            .unwrap();
        assert_eq!(g.cell_of(p(0.55, 0.15, 0.35)), Some(CuCell3::new(5, 1, 3)));
        assert_eq!(g.cell_of(p(0.55, 0.15, 0.45)), None);
        let c = g.cell_center(CuCell3::new(5, 1, 3)).unwrap();
        assert!((c.z.get::<meter>() - 0.35).abs() < 1e-12);
    }

    #[test]
    fn test_insert_ray_3d() {
        let mut g =
            CuVoxelGrid::<8, 8, 4>::from_params(geometry(), CuOccupancyUpdateParams::default())
                .unwrap();
        for _ in 0..4 {
            g.insert_ray(p(0.05, 0.05, 0.05), p(0.75, 0.05, 0.35))
                .unwrap();
        }
        assert_eq!(g.get(CuCell3::new(0, 0, 0)), Some(CuOccupancy::Free));
        assert_eq!(g.get(CuCell3::new(7, 0, 3)), Some(CuOccupancy::Occupied));
        let occupied = g.cells().iter().flatten().flatten();
        assert_eq!(occupied.filter(|c| **c == CuOccupancy::Occupied).count(), 1);
        assert!(g.insert_ray(p(0.0, 0.0, -0.1), p(0.1, 0.1, 0.1)).is_err());
    }

    #[test]
    fn test_project_band() {
        let mut g =
            CuVoxelGrid::<8, 8, 4>::from_params(geometry(), CuOccupancyUpdateParams::default())
                .unwrap();
        g.set(CuCell3::new(1, 1, 0), CuOccupancy::Occupied).unwrap(); // floor clutter
        g.set(CuCell3::new(2, 2, 2), CuOccupancy::Occupied).unwrap(); // table edge
        g.set(CuCell3::new(3, 3, 1), CuOccupancy::Free).unwrap();
        g.set(CuCell3::new(3, 3, 2), CuOccupancy::Free).unwrap();
        let mut out = CuOccupancyGrid::<8, 8>::from_params(
            geometry().xy(),
            CuOccupancyUpdateParams::default(),
        )
        .unwrap();
        // Band 0.1–0.3 m: voxel centers 0.15 and 0.25 (z = 1, 2).
        g.project_into(m(0.1), m(0.3), &mut out).unwrap();
        assert_eq!(out.get(CuCell2::new(1, 1)), Some(CuOccupancy::Unknown));
        assert_eq!(out.get(CuCell2::new(2, 2)), Some(CuOccupancy::Occupied));
        assert_eq!(out.get(CuCell2::new(3, 3)), Some(CuOccupancy::Free));
        // A band between voxel centers selects nothing.
        g.project_into(m(0.16), m(0.24), &mut out).unwrap();
        assert!(
            out.cells()
                .iter()
                .flatten()
                .all(|c| *c == CuOccupancy::Unknown)
        );
    }

    #[test]
    fn test_project_rejects_mismatch() {
        let g = CuVoxelGrid::<8, 8, 4>::from_params(geometry(), CuOccupancyUpdateParams::default())
            .unwrap();
        let shifted = CuGridGeometry2::new(m(0.1), CuPoint2::new(m(0.05), m(0.0))).unwrap();
        let mut out =
            CuOccupancyGrid::<8, 8>::from_params(shifted, CuOccupancyUpdateParams::default())
                .unwrap();
        assert!(g.project_into(m(0.0), m(0.4), &mut out).is_err());
        let mut out = CuOccupancyGrid::<8, 8>::from_params(
            geometry().xy(),
            CuOccupancyUpdateParams::default(),
        )
        .unwrap();
        assert!(g.project_into(m(0.3), m(0.1), &mut out).is_err());
    }
}
