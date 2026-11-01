//! Fixed-size occupancy maps for mobile robot navigation.
//!
//! - [`CuOccupancyGrid<W, H>`]: a 2D grid of [`CuOccupancy`] cells.
//! - [`CuVoxelGrid<W, H, D>`]: the same in 3D, plus
//!   [`CuVoxelGrid::project_into`] to flatten a height band into a 2D grid
//!   (how a voxel layer feeds a planar costmap).
//!
//! Both grids store their cells inline (`W·H` or `W·H·D` bytes) and never
//! allocate. Large maps belong in a `Box` or a `static`, not on the stack.
//! World coordinates map to cells through [`CuGridGeometry2`] /
//! [`CuGridGeometry3`]: the origin is the outer corner of cell `(0, 0[, 0])`
//! and every cell is a square (cube) of side `resolution`.
//!
//! Sensor rays are inserted with `insert_ray` (cells along the ray become
//! [`CuOccupancy::Free`], the hit cell [`CuOccupancy::Occupied`]) and
//! `clear_ray` (max-range readings: free only).
//!
//! ```
//! use cu29::units::si::f64::Length;
//! use cu29::units::si::length::meter;
//! use meander_core::CuPoint2;
//! use meander_grid::{CuGridGeometry2, CuOccupancy, CuOccupancyGrid};
//!
//! let geometry = CuGridGeometry2::new(
//!     Length::new::<meter>(0.05),
//!     CuPoint2::new(Length::new::<meter>(0.0), Length::new::<meter>(0.0)),
//! )?;
//! let mut grid = CuOccupancyGrid::<64, 64>::new(geometry);
//! let sensor = CuPoint2::new(Length::new::<meter>(0.5), Length::new::<meter>(0.5));
//! let hit = CuPoint2::new(Length::new::<meter>(2.0), Length::new::<meter>(1.0));
//! grid.insert_ray(sensor, hit)?;
//! assert_eq!(grid.occupancy_at(hit), Some(CuOccupancy::Occupied));
//! assert_eq!(grid.occupancy_at(sensor), Some(CuOccupancy::Free));
//! # Ok::<(), cu29::prelude::CuError>(())
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

mod geometry;
mod occupancy;
mod raycast;
mod voxel;

pub use geometry::CuCell2;
pub use geometry::CuCell3;
pub use geometry::CuGridGeometry2;
pub use geometry::CuGridGeometry3;
pub use occupancy::CuOccupancy;
pub use occupancy::CuOccupancyGrid;
pub use voxel::CuVoxelGrid;
