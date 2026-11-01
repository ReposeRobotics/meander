//! Mapping configuration: one params struct, identical in Rust and RON.

use alloc::string::String;
use cu29::prelude::*;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use meander_core::CuPoint3;
use meander_core::CuPose3;
use meander_grid::CuOccupancyUpdateParams;
use serde::Deserialize;
use serde::Serialize;

/// A height band `[z_min, z_max]` in the map frame, flattened onto the 2D map.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(deny_unknown_fields)]
pub struct CuHeightBand {
    pub z_min: Length,
    pub z_max: Length,
}

/// Parameters of the mapping tasks and mappers.
///
/// ```ron
/// config: {
///     "resolution": 0.05,
///     "origin": (x: -5.0, y: -5.0, z: 0.0),
///     "sensor_pose": (position: (x: 0.1, y: 0.0, z: 0.2), orientation: (w: 1.0, x: 0.0, y: 0.0, z: 0.0)),
///     "max_range": Some(8.0),
///     "save_path": Some("maps/warehouse"),
/// }
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuMappingParams {
    /// Cell (voxel) side length.
    pub resolution: Length,
    /// Map-frame position of the outer corner of cell `(0, 0[, 0])`. The 2D
    /// mapper uses `x`/`y` only.
    pub origin: CuPoint3,
    /// Log-odds inverse sensor model and classification thresholds.
    pub update: CuOccupancyUpdateParams,
    /// Sensor pose in the robot base frame (`base ← sensor`).
    pub sensor_pose: CuPose3,
    /// Returns farther than this from the sensor are not trusted as hits:
    /// the ray is cleared up to `max_range` instead. `None` trusts every
    /// valid return.
    pub max_range: Option<Length>,
    /// Clear laser beams that have no valid return up to the scan's
    /// `range_max` (capped by `max_range`), as Nav2's `raytrace` does.
    pub clear_no_return: bool,
    /// Height band flattened onto the 2D map by the 3D task when saving.
    pub projection: CuHeightBand,
    /// File stem the task saves `<stem>.pgm` + `<stem>.yaml` to on `stop`
    /// (`std` only). `None` disables saving.
    pub save_path: Option<String>,
}

impl Default for CuMappingParams {
    fn default() -> Self {
        Self {
            resolution: Length::new::<meter>(0.05),
            origin: CuPoint3::default(),
            update: CuOccupancyUpdateParams::default(),
            sensor_pose: CuPose3::default(),
            max_range: None,
            clear_no_return: true,
            projection: CuHeightBand {
                z_min: Length::new::<meter>(0.05),
                z_max: Length::new::<meter>(2.0),
            },
            save_path: None,
        }
    }
}

impl CuMappingParams {
    /// Rejects invalid geometry, update parameters, ranges and bands.
    pub fn validate(&self) -> CuResult<()> {
        let r = self.resolution.get::<meter>();
        if !r.is_finite() || r <= 0.0 {
            return Err(CuError::from(
                "meander-mapping: resolution must be finite and positive",
            ));
        }
        let o = &self.origin;
        let p = &self.sensor_pose.position;
        if ![o.x, o.y, o.z, p.x, p.y, p.z]
            .iter()
            .all(|v| v.get::<meter>().is_finite())
        {
            return Err(CuError::from(
                "meander-mapping: origin and sensor_pose must be finite",
            ));
        }
        if let Some(max) = self.max_range {
            let max = max.get::<meter>();
            if !max.is_finite() || max <= 0.0 {
                return Err(CuError::from(
                    "meander-mapping: max_range must be finite and positive",
                ));
            }
        }
        let (lo, hi) = (
            self.projection.z_min.get::<meter>(),
            self.projection.z_max.get::<meter>(),
        );
        if !lo.is_finite() || !hi.is_finite() || lo > hi {
            return Err(CuError::from(
                "meander-mapping: projection needs finite z_min <= z_max",
            ));
        }
        if self.save_path.as_deref() == Some("") {
            return Err(CuError::from(
                "meander-mapping: save_path must not be empty",
            ));
        }
        self.update.validate()
    }
}
