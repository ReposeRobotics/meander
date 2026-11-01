//! Fixed-size occupancy maps for mobile robot navigation.
//!
//! - [`CuOccupancyGrid<W, H>`]: a 2D grid of [`CuOccupancy`] cells.
//! - [`CuVoxelGrid<W, H, D>`]: the same in 3D, plus
//!   [`CuVoxelGrid::project_into`] to flatten a height band into a 2D grid
//!   (how a voxel layer feeds a planar costmap).
//!
//! Both grids store f32 log-odds and a cached classified view inline and never
//! allocate. Large maps belong in a `Box` or a `static`, not on the stack.
//! World coordinates map to cells through [`CuGridGeometry2`] /
//! [`CuGridGeometry3`]: the origin is the outer corner of cell `(0, 0[, 0])`
//! and every cell is a square (cube) of side `resolution`.
//!
//! Sensor rays add clamped hit/miss log-odds evidence using
//! [`CuOccupancyUpdateParams`]. Unknown means zero evidence or probability
//! between the inclusive free/occupied thresholds. A single default miss
//! remains unknown; repeated misses establish free space. Logarithms are
//! cached at construction, so ray updates use only addition and comparisons.
//! `cells()` borrows classified storage; `log_odds_rows()` borrows raw f32
//! storage. Scalar accessors expose dimensionless values as f64.
//! Voxel projection classifies each voxel, giving occupied precedence over
//! free and free over unknown, then sets the target to its clamp extremes.
//!
//! ```
//! use cu29::units::si::f64::Length;
//! use cu29::units::si::length::meter;
//! use meander_core::CuPoint2;
//! use meander_grid::{CuGridGeometry2, CuOccupancy, CuOccupancyGrid, CuOccupancyUpdateParams};
//!
//! let geometry = CuGridGeometry2::new(
//!     Length::new::<meter>(0.05),
//!     CuPoint2::new(Length::new::<meter>(0.0), Length::new::<meter>(0.0)),
//! )?;
//! let mut grid = CuOccupancyGrid::<64, 64>::from_params(geometry, CuOccupancyUpdateParams::default())?;
//! let sensor = CuPoint2::new(Length::new::<meter>(0.5), Length::new::<meter>(0.5));
//! let hit = CuPoint2::new(Length::new::<meter>(2.0), Length::new::<meter>(1.0));
//! for _ in 0..4 { grid.insert_ray(sensor, hit)?; }
//! assert_eq!(grid.occupancy_at(hit), Some(CuOccupancy::Occupied));
//! assert_eq!(grid.occupancy_at(sensor), Some(CuOccupancy::Free));
//! # Ok::<(), cu29::prelude::CuError>(())
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

mod geometry;
mod occupancy;
mod raycast;
mod update;
mod voxel;

pub use geometry::CuCell2;
pub use geometry::CuCell3;
pub use geometry::CuGridGeometry2;
pub use geometry::CuGridGeometry3;
pub use occupancy::CuOccupancy;
pub use occupancy::CuOccupancyGrid;
pub use update::CuOccupancyUpdateParams;
pub use voxel::CuVoxelGrid;
