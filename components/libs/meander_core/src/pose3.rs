//! Spatial rotations and child-to-parent rigid transforms.

use crate::CuPoint3;
use crate::CuPose2;
use bincode::Decode;
use bincode::Encode;
use cu29::prelude::CuError;
use cu29::prelude::CuResult;
use cu29::prelude::Reflect;
// The real Reflect derive references this alias when cu29/reflect is enabled.
#[allow(unused_imports)]
use cu29::prelude::bevy_reflect;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

/// A unit quaternion, with dimensionless scalar `w` and vector `x/y/z` components.
///
/// Rotations are right-handed and map child coordinates into parent coordinates.
/// Serialized components must also describe a unit quaternion.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(deny_unknown_fields)]
pub struct CuQuaternion {
    w: f64,
    x: f64,
    y: f64,
    z: f64,
}

impl Default for CuQuaternion {
    fn default() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

impl CuQuaternion {
    /// Normalizes finite, nonzero dimensionless components into a unit quaternion.
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> CuResult<Self> {
        let scale = w.abs().max(x.abs()).max(y.abs()).max(z.abs());
        if ![w, x, y, z].iter().all(|v| v.is_finite()) || scale == 0.0 {
            return Err(CuError::from(
                "Quaternion components must be finite and nonzero",
            ));
        }
        let (w, x, y, z) = (w / scale, x / scale, y / scale, z / scale);
        let norm = libm::sqrt(w * w + x * x + y * y + z * z);
        Ok(Self {
            w: w / norm,
            x: x / norm,
            y: y / norm,
            z: z / norm,
        })
    }

    /// Writes dimensionless components in `w, x, y, z` order.
    pub fn components(&self, out: &mut [f64; 4]) {
        *out = [self.w, self.x, self.y, self.z];
    }

    /// Constructs the rotation `Rz(yaw) Ry(pitch) Rx(roll)` from finite angles.
    ///
    /// ```
    /// use meander_core::{CuPoint3, CuQuaternion};
    /// use cu29::units::si::f64::Angle;
    /// let rotation = CuQuaternion::from_rpy(
    ///     Angle::default(), Angle::default(), Angle::default())?;
    /// let mut out = CuPoint3::default();
    /// rotation.rotate(&CuPoint3::default(), &mut out);
    /// # Ok::<(), cu29::prelude::CuError>(())
    /// ```
    pub fn from_rpy(roll: Angle, pitch: Angle, yaw: Angle) -> CuResult<Self> {
        let (sr, cr) = libm::sincos(roll.get::<radian>() * 0.5);
        let (sp, cp) = libm::sincos(pitch.get::<radian>() * 0.5);
        let (sy, cy) = libm::sincos(yaw.get::<radian>() * 0.5);
        Self::new(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    }

    /// Constructs a rotation around a finite, nonzero dimensionless axis.
    pub fn from_axis_angle(axis: &[f64; 3], angle: Angle) -> CuResult<Self> {
        let axis = Self::new(0.0, axis[0], axis[1], axis[2])?;
        let (s, c) = libm::sincos(angle.get::<radian>() * 0.5);
        Self::new(c, s * axis.x, s * axis.y, s * axis.z)
    }

    /// Writes roll, pitch, yaw in that order; pitch is in `[-π/2, π/2]`.
    /// At gimbal lock roll is zero and yaw carries the remaining rotation.
    pub fn to_rpy(&self, out: &mut [Angle; 3]) {
        let sin_pitch = (2.0 * (self.w * self.y - self.z * self.x)).clamp(-1.0, 1.0);
        if sin_pitch.abs() >= 1.0 - 4.0 * f64::EPSILON {
            *out = [
                Angle::default(),
                Angle::new::<radian>(sin_pitch.signum() * core::f64::consts::FRAC_PI_2),
                crate::normalize_angle(Angle::new::<radian>(2.0 * libm::atan2(self.z, self.w))),
            ];
            return;
        }
        out[0] = Angle::new::<radian>(libm::atan2(
            2.0 * (self.w * self.x + self.y * self.z),
            1.0 - 2.0 * (self.x * self.x + self.y * self.y),
        ));
        out[1] = Angle::new::<radian>(libm::asin(
            (2.0 * (self.w * self.y - self.z * self.x)).clamp(-1.0, 1.0),
        ));
        out[2] = Angle::new::<radian>(libm::atan2(
            2.0 * (self.w * self.z + self.x * self.y),
            1.0 - 2.0 * (self.y * self.y + self.z * self.z),
        ));
    }

    /// Writes `self ∘ other`, applying `other` first.
    pub fn compose(&self, other: &Self, out: &mut Self) {
        let a = self;
        let b = other;
        let mut q = Self {
            w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        };
        let n = libm::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        q.w /= n;
        q.x /= n;
        q.y /= n;
        q.z /= n;
        *out = q;
    }

    /// Writes the inverse rotation.
    pub fn inverse(&self, out: &mut Self) {
        *out = Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        };
    }

    /// Rotates a point about the origin into `out`.
    pub fn rotate(&self, point: &CuPoint3, out: &mut CuPoint3) {
        let (x, y, z) = (
            point.x.get::<meter>(),
            point.y.get::<meter>(),
            point.z.get::<meter>(),
        );
        let (tx, ty, tz) = (
            2.0 * (self.y * z - self.z * y),
            2.0 * (self.z * x - self.x * z),
            2.0 * (self.x * y - self.y * x),
        );
        *out = CuPoint3::new(
            Length::new::<meter>(x + self.w * tx + self.y * tz - self.z * ty),
            Length::new::<meter>(y + self.w * ty + self.z * tx - self.x * tz),
            Length::new::<meter>(z + self.w * tz + self.x * ty - self.y * tx),
        );
    }
}

/// A rigid transform from a child frame to its parent frame.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(deny_unknown_fields)]
pub struct CuPose3 {
    pub position: CuPoint3,
    pub orientation: CuQuaternion,
}

impl CuPose3 {
    /// Creates a pose from its position and unit orientation.
    pub fn new(position: CuPoint3, orientation: CuQuaternion) -> Self {
        Self {
            position,
            orientation,
        }
    }

    /// Maps a child-frame point into the parent frame.
    pub fn transform_point(&self, point: &CuPoint3, out: &mut CuPoint3) {
        self.orientation.rotate(point, out);
        out.x += self.position.x;
        out.y += self.position.y;
        out.z += self.position.z;
    }

    /// Writes `self ∘ other`, applying `other` first.
    pub fn compose(&self, other: &Self, out: &mut Self) {
        self.transform_point(&other.position, &mut out.position);
        self.orientation
            .compose(&other.orientation, &mut out.orientation);
    }

    /// Writes the inverse child-to-parent transform.
    pub fn inverse(&self, out: &mut Self) {
        self.orientation.inverse(&mut out.orientation);
        let negative = CuPoint3::new(-self.position.x, -self.position.y, -self.position.z);
        out.orientation.rotate(&negative, &mut out.position);
    }

    /// Embeds a planar pose at zero height with rotation about `z`.
    pub fn from_pose2(pose: &CuPose2) -> Self {
        let (s, c) = libm::sincos(pose.theta.get::<radian>() * 0.5);
        Self::new(
            CuPoint3::new(pose.x, pose.y, Length::default()),
            CuQuaternion {
                w: c,
                x: 0.0,
                y: 0.0,
                z: s,
            },
        )
    }

    /// Projects position onto `xy` and orientation onto yaw.
    pub fn to_pose2(&self, out: &mut CuPose2) {
        let mut rpy = [Angle::default(); 3];
        self.orientation.to_rpy(&mut rpy);
        *out = CuPose2::new(self.position.x, self.position.y, rpy[2]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::FRAC_PI_2;

    fn m(v: f64) -> Length {
        Length::new::<meter>(v)
    }
    fn rad(v: f64) -> Angle {
        Angle::new::<radian>(v)
    }
    fn point_close(a: &CuPoint3, b: &CuPoint3) {
        assert!(a.distance(b).get::<meter>() < 1e-9, "{a:?} != {b:?}");
    }

    #[test]
    fn test_rpy_round_trip() {
        for angles in [[0.3, -0.7, 2.1], [-1.2, 0.9, -2.4], [0.0; 3]] {
            let q = CuQuaternion::from_rpy(rad(angles[0]), rad(angles[1]), rad(angles[2])).unwrap();
            let mut out = [Angle::default(); 3];
            q.to_rpy(&mut out);
            for i in 0..3 {
                assert!((out[i].get::<radian>() - angles[i]).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn test_gimbal_lock_preserves_rotation() {
        let point = CuPoint3::new(m(0.3), m(-0.8), m(1.7));
        for pitch in [-FRAC_PI_2, FRAC_PI_2] {
            let q = CuQuaternion::from_rpy(rad(0.7), rad(pitch), rad(-1.1)).unwrap();
            let mut angles = [Angle::default(); 3];
            q.to_rpy(&mut angles);
            let reconstructed = CuQuaternion::from_rpy(angles[0], angles[1], angles[2]).unwrap();
            let mut expected = CuPoint3::default();
            let mut actual = CuPoint3::default();
            q.rotate(&point, &mut expected);
            reconstructed.rotate(&point, &mut actual);
            point_close(&expected, &actual);
        }
    }

    #[test]
    fn test_normalization_and_axis_angle() {
        for scale in [1e-300, 1e300] {
            let q = CuQuaternion::new(scale, 0.0, 0.0, scale).unwrap();
            let mut components = [0.0; 4];
            q.components(&mut components);
            assert!((components.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() < 1e-12);
        }
        assert!(CuQuaternion::new(0.0, 0.0, 0.0, 0.0).is_err());
        assert!(CuQuaternion::new(f64::NAN, 0.0, 0.0, 1.0).is_err());
        assert!(CuQuaternion::new(f64::INFINITY, 0.0, 0.0, 1.0).is_err());
        assert!(CuQuaternion::from_axis_angle(&[0.0; 3], rad(0.5)).is_err());
        assert!(CuQuaternion::from_rpy(rad(f64::NAN), rad(0.0), rad(0.0)).is_err());
        let q = CuQuaternion::from_axis_angle(&[0.0, 0.0, 8.0], rad(FRAC_PI_2)).unwrap();
        let mut rotated = CuPoint3::default();
        q.rotate(&CuPoint3::new(m(1.0), m(0.0), m(0.0)), &mut rotated);
        point_close(&rotated, &CuPoint3::new(m(0.0), m(1.0), m(0.0)));
    }

    #[test]
    fn test_compose_inverse_and_transform() {
        let pose = CuPose3::new(
            CuPoint3::new(m(1.5), m(-0.7), m(2.0)),
            CuQuaternion::from_rpy(rad(0.3), rad(-0.4), rad(2.1)).unwrap(),
        );
        let mut inverse = CuPose3::default();
        pose.inverse(&mut inverse);
        let point = CuPoint3::new(m(0.4), m(-2.0), m(3.0));
        for (a, b) in [(pose, inverse), (inverse, pose)] {
            let mut identity = CuPose3::default();
            a.compose(&b, &mut identity);
            point_close(&identity.position, &CuPoint3::default());
            let mut result = CuPoint3::default();
            identity.transform_point(&point, &mut result);
            point_close(&result, &point);
        }
        let other = CuPose3::new(
            CuPoint3::new(m(0.0), m(1.0), m(-3.0)),
            CuQuaternion::from_rpy(rad(-0.7), rad(0.8), rad(0.4)).unwrap(),
        );
        let mut combined = CuPose3::default();
        pose.compose(&other, &mut combined);
        let mut first = CuPoint3::default();
        let mut sequential = CuPoint3::default();
        let mut direct = CuPoint3::default();
        other.transform_point(&point, &mut first);
        pose.transform_point(&first, &mut sequential);
        combined.transform_point(&point, &mut direct);
        point_close(&direct, &sequential);
    }

    #[test]
    fn test_planar_consistency() {
        let pose2 = CuPose2::new(m(1.0), m(-2.0), rad(0.8));
        let pose3 = CuPose3::from_pose2(&pose2);
        let point = CuPoint3::new(m(0.5), m(-0.7), m(0.0));
        let planar = pose2.transform_point(point.xy());
        let mut spatial = CuPoint3::default();
        pose3.transform_point(&point, &mut spatial);
        point_close(&spatial, &CuPoint3::new(planar.x, planar.y, m(0.0)));
        let mut projected = CuPose2::default();
        pose3.to_pose2(&mut projected);
        assert!((projected.theta - pose2.theta).get::<radian>().abs() < 1e-9);
        assert_eq!(projected.position(), pose2.position());
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_pose3_ron_round_trip() {
        let pose = CuPose3::new(
            CuPoint3::new(m(1.0), m(2.0), m(3.0)),
            CuQuaternion::from_rpy(rad(0.2), rad(0.4), rad(-0.8)).unwrap(),
        );
        let text = ron::to_string(&pose).unwrap();
        assert_eq!(ron::from_str::<CuPose3>(&text).unwrap(), pose);
    }
}
