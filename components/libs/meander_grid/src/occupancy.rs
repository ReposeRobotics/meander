use crate::geometry::{CuCell2, CuGridGeometry2};
use crate::raycast::{cell_of, traverse};
use bincode::Decode;
use bincode::Encode;
use cu29::prelude::*;
use meander_core::CuPoint2;
use serde::Deserialize;
use serde::Serialize;

/// State of one map cell.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuOccupancy {
    /// Never observed.
    #[default]
    Unknown,
    /// Observed empty.
    Free,
    /// Observed occupied.
    Occupied,
}

/// A fixed-size 2D occupancy grid of `W × H` cells.
///
/// Cell `(x, y)` covers `origin + [x, x+1) × [y, y+1)` cells of `resolution`.
/// Every cell starts [`CuOccupancy::Unknown`].
#[derive(Debug, Clone, PartialEq)]
pub struct CuOccupancyGrid<const W: usize, const H: usize> {
    geometry: CuGridGeometry2,
    cells: [[CuOccupancy; W]; H],
}

impl<const W: usize, const H: usize> CuOccupancyGrid<W, H> {
    /// Creates an all-unknown grid placed by `geometry`.
    pub fn new(geometry: CuGridGeometry2) -> Self {
        Self {
            geometry,
            cells: [[CuOccupancy::Unknown; W]; H],
        }
    }

    /// The grid's placement in the world.
    pub fn geometry(&self) -> &CuGridGeometry2 {
        &self.geometry
    }

    /// The cell containing a world point, or `None` outside the grid.
    pub fn cell_of(&self, point: CuPoint2) -> Option<CuCell2> {
        cell_of(self.geometry.grid_coords(point), [W, H]).map(|[x, y]| CuCell2::new(x, y))
    }

    /// World position of a cell's center, or `None` outside the grid.
    pub fn cell_center(&self, cell: CuCell2) -> Option<CuPoint2> {
        self.contains(cell).then(|| self.geometry.center(cell))
    }

    /// Whether `cell` lies inside the grid.
    pub fn contains(&self, cell: CuCell2) -> bool {
        cell.x < W && cell.y < H
    }

    /// The state of `cell`, or `None` outside the grid.
    pub fn get(&self, cell: CuCell2) -> Option<CuOccupancy> {
        self.cells
            .get(cell.y)
            .and_then(|row| row.get(cell.x))
            .copied()
    }

    /// The state of the cell containing a world point, or `None` outside.
    pub fn occupancy_at(&self, point: CuPoint2) -> Option<CuOccupancy> {
        self.cell_of(point).and_then(|cell| self.get(cell))
    }

    /// Sets the state of `cell`; errors outside the grid.
    pub fn set(&mut self, cell: CuCell2, state: CuOccupancy) -> CuResult<()> {
        let slot = self
            .cells
            .get_mut(cell.y)
            .and_then(|row| row.get_mut(cell.x))
            .ok_or_else(|| CuError::from("meander-grid: cell outside the occupancy grid"))?;
        *slot = state;
        Ok(())
    }

    /// Sets every cell to `state`.
    pub fn fill(&mut self, state: CuOccupancy) {
        for row in &mut self.cells {
            row.fill(state);
        }
    }

    /// Row-major cell storage: `cells()[y][x]`.
    pub fn cells(&self) -> &[[CuOccupancy; W]; H] {
        &self.cells
    }

    /// Inserts a sensor return: cells from `origin` up to (not including)
    /// the cell of `hit` become free; the hit cell becomes occupied if it is
    /// inside the grid. Errors if `origin` is outside the grid.
    pub fn insert_ray(&mut self, origin: CuPoint2, hit: CuPoint2) -> CuResult<()> {
        let hit_cell = self.cell_of(hit);
        self.trace_free(origin, hit, hit_cell)?;
        if let Some(cell) = hit_cell {
            self.set(cell, CuOccupancy::Occupied)?;
        }
        Ok(())
    }

    /// Clears a ray without a return (max-range reading): every cell from
    /// `origin` to `end` becomes free. Errors if `origin` is outside the grid.
    pub fn clear_ray(&mut self, origin: CuPoint2, end: CuPoint2) -> CuResult<()> {
        self.trace_free(origin, end, None)
    }

    fn trace_free(
        &mut self,
        origin: CuPoint2,
        end: CuPoint2,
        keep: Option<CuCell2>,
    ) -> CuResult<()> {
        let cells = &mut self.cells;
        let inside = traverse(
            self.geometry.grid_coords(origin),
            self.geometry.grid_coords(end),
            [W, H],
            |[x, y]| {
                if keep != Some(CuCell2::new(x, y)) {
                    cells[y][x] = CuOccupancy::Free;
                }
            },
        );
        if inside {
            Ok(())
        } else {
            Err(CuError::from(
                "meander-grid: ray origin outside the occupancy grid",
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::units::si::f64::Length;
    use cu29::units::si::length::meter;

    fn p(x: f64, y: f64) -> CuPoint2 {
        CuPoint2::new(Length::new::<meter>(x), Length::new::<meter>(y))
    }

    fn grid() -> CuOccupancyGrid<10, 8> {
        // 0.5 m cells, origin at (-1, -1): covers x ∈ [-1, 4), y ∈ [-1, 3).
        CuOccupancyGrid::new(
            CuGridGeometry2::new(Length::new::<meter>(0.5), p(-1.0, -1.0)).unwrap(),
        )
    }

    #[test]
    fn test_geometry_rejects_bad_resolution() {
        for r in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(CuGridGeometry2::new(Length::new::<meter>(r), p(0.0, 0.0)).is_err());
        }
        assert!(CuGridGeometry2::new(Length::new::<meter>(0.1), p(f64::NAN, 0.0)).is_err());
    }

    #[test]
    fn test_world_cell_round_trip() {
        let g = grid();
        assert_eq!(g.cell_of(p(-1.0, -1.0)), Some(CuCell2::new(0, 0)));
        assert_eq!(g.cell_of(p(0.3, 0.6)), Some(CuCell2::new(2, 3)));
        assert_eq!(g.cell_of(p(4.0, 0.0)), None);
        assert_eq!(g.cell_of(p(-1.01, 0.0)), None);
        assert_eq!(g.cell_center(CuCell2::new(2, 3)), Some(p(0.25, 0.75)));
        assert_eq!(g.cell_center(CuCell2::new(10, 0)), None);
    }

    #[test]
    fn test_set_get_and_bounds() {
        let mut g = grid();
        assert_eq!(g.get(CuCell2::new(3, 3)), Some(CuOccupancy::Unknown));
        g.set(CuCell2::new(3, 3), CuOccupancy::Occupied).unwrap();
        assert_eq!(g.cells()[3][3], CuOccupancy::Occupied);
        assert!(g.set(CuCell2::new(0, 8), CuOccupancy::Free).is_err());
        assert_eq!(g.get(CuCell2::new(10, 0)), None);
        g.fill(CuOccupancy::Free);
        assert!(g.cells().iter().flatten().all(|c| *c == CuOccupancy::Free));
    }

    #[test]
    fn test_insert_ray_frees_path_and_marks_hit() {
        let mut g = grid();
        g.insert_ray(p(-0.75, 0.25), p(2.25, 0.25)).unwrap();
        for x in 0..6 {
            assert_eq!(g.get(CuCell2::new(x, 2)), Some(CuOccupancy::Free), "x={x}");
        }
        assert_eq!(g.get(CuCell2::new(6, 2)), Some(CuOccupancy::Occupied));
        assert_eq!(g.get(CuCell2::new(7, 2)), Some(CuOccupancy::Unknown));
        assert_eq!(g.get(CuCell2::new(3, 3)), Some(CuOccupancy::Unknown));
    }

    #[test]
    fn test_insert_ray_hit_outside_frees_to_border() {
        let mut g = grid();
        g.insert_ray(p(0.25, 0.25), p(10.0, 0.25)).unwrap();
        assert_eq!(g.get(CuCell2::new(9, 2)), Some(CuOccupancy::Free));
        assert!(
            g.cells()
                .iter()
                .flatten()
                .all(|c| *c != CuOccupancy::Occupied)
        );
    }

    #[test]
    fn test_clear_ray_and_outside_origin() {
        let mut g = grid();
        g.set(CuCell2::new(4, 2), CuOccupancy::Occupied).unwrap();
        g.clear_ray(p(0.25, 0.25), p(1.25, 0.25)).unwrap();
        assert_eq!(g.get(CuCell2::new(4, 2)), Some(CuOccupancy::Free));
        assert!(g.insert_ray(p(-5.0, 0.0), p(0.0, 0.0)).is_err());
    }
}
