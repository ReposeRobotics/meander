//! Path following controllers for mobile robot navigation.
//!
//! - [`CuPurePursuit`]: regulated pure pursuit algorithm for differential-drive
//!   robots, supporting lookahead interpolation, curvature-preserving velocity
//!   clamping, rotate-in-place at sharp angles, and goal heading alignment.
//! - [`CuPurePursuitParams`]: unified parameters for pure pursuit.
//! - [`CuFollowStatus`]: controller follow status outcome.
//!
//! All calculations are allocation-free and compatible with `no_std`.
//!
//! ```
//! use cu29::units::si::angle::radian;
//! use cu29::units::si::f64::{Angle, Length};
//! use cu29::units::si::length::meter;
//! use meander_controller::{CuFollowStatus, CuPurePursuit, CuPurePursuitParams};
//! use meander_core::{CuPose2, CuTwist2};
//!
//! let params = CuPurePursuitParams::default();
//! let mut controller = CuPurePursuit::from_params(params)?;
//!
//! let pose = CuPose2::default();
//! let path = [
//!     CuPose2::new(Length::new::<meter>(0.0), Length::new::<meter>(0.0), Angle::new::<radian>(0.0)),
//!     CuPose2::new(Length::new::<meter>(1.0), Length::new::<meter>(0.0), Angle::new::<radian>(0.0)),
//! ];
//! let mut twist = CuTwist2::default();
//! let status = controller.compute(&pose, &path, &mut twist)?;
//! assert_eq!(status, CuFollowStatus::Following);
//! # Ok::<(), cu29::prelude::CuError>(())
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

mod params;
mod pure_pursuit;

pub use params::CuPurePursuitParams;
pub use pure_pursuit::CuFollowStatus;
pub use pure_pursuit::CuPurePursuit;
