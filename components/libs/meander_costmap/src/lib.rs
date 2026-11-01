//! Fixed-capacity Nav2 costmaps with exact Euclidean obstacle inflation.
//!
//! Storage and distance-transform scratch space are inline. Large maps should
//! live in a `Box` or static storage. Updates run in O(W × H) time without heap
//! allocation; distances are measured between cell centers.
//!
//! ```
//! use cu29::units::si::f64::Length;
//! use cu29::units::si::length::meter;
//! use meander_core::CuPoint2;
//! use meander_grid::{CuCell2, CuGridGeometry2, CuOccupancy, CuOccupancyGrid, CuOccupancyUpdateParams};
//! use meander_costmap::{CuCostmap2, CuCostmapParams, LETHAL};
//!
//! let geometry = CuGridGeometry2::new(Length::new::<meter>(0.05), CuPoint2::default())?;
//! let mut occupancy = CuOccupancyGrid::<16, 16>::from_params(geometry, CuOccupancyUpdateParams::default())?;
//! occupancy.fill(CuOccupancy::Free);
//! occupancy.set(CuCell2::new(8, 8), CuOccupancy::Occupied)?;
//! let mut costs = CuCostmap2::from_params(geometry, CuCostmapParams::default())?;
//! costs.update_from_occupancy(&occupancy)?;
//! assert_eq!(costs.costs()[8][8], LETHAL);
//! # Ok::<(), cu29::prelude::CuError>(())
//! ```
#![cfg_attr(not(feature = "std"), no_std)]

mod costmap;
mod params;

pub use costmap::CuCostmap2;
pub use costmap::FREE_SPACE;
pub use costmap::INSCRIBED;
pub use costmap::LETHAL;
pub use costmap::NO_INFORMATION;
pub use params::CuCostmapParams;
