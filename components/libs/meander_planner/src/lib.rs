//! Bidirectional Dijkstra path planner over Nav2-style cost grids.
//!
//! [`CuBidirectionalDijkstra<W, H>`] plans from a start to a goal pose over
//! costs in row-major `[y][x]` order with a [`meander_grid::CuGridGeometry2`] placement: 0
//! is free, 1..=252 are inflated, 253 is inscribed, 254 is lethal and 255 is
//! unknown. Planning is allocation-free: every distance, parent, queue and
//! path buffer is fixed-capacity storage owned by the planner.
//!
//! ```
//! use cu29::units::si::angle::radian;
//! use cu29::units::si::f64::{Angle, Length};
//! use cu29::units::si::length::meter;
//! use meander_core::{CuPoint2, CuPose2};
//! use meander_grid::CuGridGeometry2;
//! use meander_planner::{CuBidirectionalDijkstra, CuPlanStatus, CuPlannerParams};
//!
//! let geometry = CuGridGeometry2::new(
//!     Length::new::<meter>(1.0),
//!     CuPoint2::new(Length::new::<meter>(0.0), Length::new::<meter>(0.0)),
//! )?;
//! let mut planner = CuBidirectionalDijkstra::<8, 8>::from_params(CuPlannerParams::default())?;
//! let costs = [[0u8; 8]; 8];
//! let start = CuPose2::new(
//!     Length::new::<meter>(0.5),
//!     Length::new::<meter>(0.5),
//!     Angle::new::<radian>(0.0),
//! );
//! let goal = CuPose2::new(
//!     Length::new::<meter>(7.5),
//!     Length::new::<meter>(7.5),
//!     Angle::new::<radian>(0.0),
//! );
//! let mut out = [CuPose2::default(); 64];
//! let report = planner.plan(&costs, &geometry, &start, &goal, &mut out)?;
//! assert_eq!(report.status, CuPlanStatus::Found);
//! assert_eq!(report.len, 8);
//! # Ok::<(), cu29::prelude::CuError>(())
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

mod params;
mod planner;
mod report;

pub use params::CuPlannerParams;
pub use planner::CuBidirectionalDijkstra;
pub use report::CuPlanReport;
pub use report::CuPlanStatus;
