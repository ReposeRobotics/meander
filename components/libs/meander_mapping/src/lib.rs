//! Occupancy mapping from robot poses and range sensors, as Copper tasks.
//!
//! The pose comes from whatever localizes the robot (a SLAM front end,
//! AMCL-style localization, or odometry); this crate only folds observations
//! into log-odds maps from `meander-grid`:
//!
//! - [`CuScanMapper2`] / [`CuScanMappingTask`]: `CuPose2` + `CuLaserScan<N>`
//!   into a 2D [`meander_grid::CuOccupancyGrid`].
//! - [`CuCloudMapper3`] / [`CuCloudMappingTask`]: `CuPose3` + `CuPointCloud3<N>`
//!   into a 3D [`meander_grid::CuVoxelGrid`], flattened to 2D on save.
//!
//! Every ray clears the cells it crosses and marks its end cell occupied.
//! Laser beams without a return clear up to the scan's `range_max`; returns
//! beyond [`CuMappingParams::max_range`] only clear. Both tasks publish a
//! [`CuMapUpdateReport`] per observation and can save a Nav2 map (PGM + YAML)
//! on `stop`. Updates allocate nothing.
//!
//! ```
//! use cu29::units::si::angle::radian;
//! use cu29::units::si::f64::{Angle, Length};
//! use cu29::units::si::length::meter;
//! use meander_core::{CuLaserScan, CuPoint3, CuPose2};
//! use meander_grid::CuOccupancy;
//! use meander_mapping::{CuMapUpdateReport, CuMappingParams, CuScanMapper2};
//!
//! let params = CuMappingParams {
//!     resolution: Length::new::<meter>(0.1),
//!     origin: CuPoint3::new(Length::new::<meter>(-2.0), Length::new::<meter>(-2.0), Length::default()),
//!     ..CuMappingParams::default()
//! };
//! let mut mapper = CuScanMapper2::<40, 40>::from_params(params)?;
//!
//! // One beam straight ahead, hitting a wall 1.5 m away.
//! let mut scan = CuLaserScan::<1>::default();
//! scan.range_max = Length::new::<meter>(10.0);
//! scan.ranges[0] = Length::new::<meter>(1.5);
//! let mut report = CuMapUpdateReport::default();
//! for _ in 0..3 {
//!     mapper.insert_scan(&CuPose2::default(), &scan, &mut report)?;
//! }
//! let wall = mapper.grid().cell_of(meander_core::CuPoint2::new(
//!     Length::new::<meter>(1.55),
//!     Length::default(),
//! ));
//! assert_eq!(wall.and_then(|c| mapper.grid().get(c)), Some(CuOccupancy::Occupied));
//! # let _ = Angle::new::<radian>(0.0);
//! # Ok::<(), cu29::prelude::CuError>(())
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

mod mapper;
mod params;
mod task;

pub use mapper::CuCloudMapper3;
pub use mapper::CuMapUpdateReport;
pub use mapper::CuMapUpdateStatus;
pub use mapper::CuScanMapper2;
pub use params::CuHeightBand;
pub use params::CuMappingParams;
pub use task::CuCloudMappingTask;
pub use task::CuScanMappingTask;
