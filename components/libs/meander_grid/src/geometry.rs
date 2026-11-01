use bincode::Decode;
use bincode::Encode;
use cu29::prelude::*;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use meander_core::{CuPoint2, CuPoint3};
use serde::Deserialize;
use serde::Serialize;

/// Integer coordinates of a 2D grid cell.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuCell2 {
    pub x: usize,
    pub y: usize,
}

impl CuCell2 {
    /// Creates a cell index.
    pub fn new(x: usize, y: usize) -> Self {
        Self { x, y }
    }
}

/// Integer coordinates of a 3D grid voxel.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuCell3 {
    pub x: usize,
    pub y: usize,
    pub z: usize,
}

impl CuCell3 {
    /// Creates a voxel index.
    pub fn new(x: usize, y: usize, z: usize) -> Self {
        Self { x, y, z }
    }
}

fn check_resolution(resolution: Length) -> CuResult<f64> {
    let r = resolution.get::<meter>();
    if !r.is_finite() || r <= 0.0 {
        return Err(CuError::from(
            "meander-grid: 'resolution' must be finite and > 0",
        ));
    }
    Ok(r)
}

fn check_finite(values: &[Length]) -> CuResult<()> {
    if values.iter().any(|v| !v.get::<meter>().is_finite()) {
        return Err(CuError::from("meander-grid: 'origin' must be finite"));
    }
    Ok(())
}

/// Placement of a 2D grid in its world frame (axis-aligned).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(deny_unknown_fields)]
pub struct CuGridGeometry2 {
    /// Side length of one square cell.
    pub resolution: Length,
    /// World position of the outer corner of cell `(0, 0)`.
    pub origin: CuPoint2,
}

impl CuGridGeometry2 {
    /// Creates a geometry; `resolution` must be positive and finite.
    pub fn new(resolution: Length, origin: CuPoint2) -> CuResult<Self> {
        let geometry = Self { resolution, origin };
        geometry.validate()?;
        Ok(geometry)
    }

    /// Checks `resolution > 0` and that every value is finite.
    pub fn validate(&self) -> CuResult<()> {
        check_resolution(self.resolution)?;
        check_finite(&[self.origin.x, self.origin.y])
    }

    /// Continuous grid coordinates (in cells) of a world point.
    pub(crate) fn grid_coords(&self, p: CuPoint2) -> [f64; 2] {
        let r = self.resolution.get::<meter>();
        [
            (p.x.get::<meter>() - self.origin.x.get::<meter>()) / r,
            (p.y.get::<meter>() - self.origin.y.get::<meter>()) / r,
        ]
    }

    /// World position of a cell's center.
    pub(crate) fn center(&self, cell: CuCell2) -> CuPoint2 {
        let r = self.resolution.get::<meter>();
        CuPoint2::new(
            Length::new::<meter>(self.origin.x.get::<meter>() + (cell.x as f64 + 0.5) * r),
            Length::new::<meter>(self.origin.y.get::<meter>() + (cell.y as f64 + 0.5) * r),
        )
    }
}

/// Placement of a 3D grid in its world frame (axis-aligned).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(deny_unknown_fields)]
pub struct CuGridGeometry3 {
    /// Side length of one cubic voxel.
    pub resolution: Length,
    /// World position of the outer corner of voxel `(0, 0, 0)`.
    pub origin: CuPoint3,
}

impl CuGridGeometry3 {
    /// Creates a geometry; `resolution` must be positive and finite.
    pub fn new(resolution: Length, origin: CuPoint3) -> CuResult<Self> {
        let geometry = Self { resolution, origin };
        geometry.validate()?;
        Ok(geometry)
    }

    /// Checks `resolution > 0` and that every value is finite.
    pub fn validate(&self) -> CuResult<()> {
        check_resolution(self.resolution)?;
        check_finite(&[self.origin.x, self.origin.y, self.origin.z])
    }

    /// The planar geometry of this grid's footprint (drops `z`).
    pub fn xy(&self) -> CuGridGeometry2 {
        CuGridGeometry2 {
            resolution: self.resolution,
            origin: self.origin.xy(),
        }
    }

    /// Continuous grid coordinates (in voxels) of a world point.
    pub(crate) fn grid_coords(&self, p: CuPoint3) -> [f64; 3] {
        let r = self.resolution.get::<meter>();
        [
            (p.x.get::<meter>() - self.origin.x.get::<meter>()) / r,
            (p.y.get::<meter>() - self.origin.y.get::<meter>()) / r,
            (p.z.get::<meter>() - self.origin.z.get::<meter>()) / r,
        ]
    }

    /// World position of a voxel's center.
    pub(crate) fn center(&self, cell: CuCell3) -> CuPoint3 {
        let r = self.resolution.get::<meter>();
        let c = |o: Length, i: usize| Length::new::<meter>(o.get::<meter>() + (i as f64 + 0.5) * r);
        CuPoint3::new(
            c(self.origin.x, cell.x),
            c(self.origin.y, cell.y),
            c(self.origin.z, cell.z),
        )
    }
}
