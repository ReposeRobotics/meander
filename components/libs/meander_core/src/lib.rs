//! Units-typed geometry for mobile robot navigation.
//!
//! - [`CuPoint2`] / [`CuPoint3`]: positions in a planar or 3D frame.
//! - [`CuPose2`]: a planar pose (`x`, `y`, heading `theta`) with composition
//!   ([`CuPose2::compose`]), inversion ([`CuPose2::inverse`]) and point
//!   transforms ([`CuPose2::transform_point`]).
//! - [`CuTwist2`]: a body-frame planar velocity; [`CuPose2::integrate`]
//!   advances a pose by a constant twist exactly (the odometry step).
//!
//! All types are plain `Copy` payloads; nothing here allocates.

#![cfg_attr(not(feature = "std"), no_std)]

use bincode::Decode;
use bincode::Encode;
use core::f64::consts::PI;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::{Angle, AngularVelocity, Length, Time, Velocity};
use cu29::units::si::length::meter;
use cu29::units::si::time::second;
use cu29::units::si::velocity::meter_per_second;
use serde::Deserialize;
use serde::Serialize;

/// Wraps an angle into `(-π, π]`.
pub fn normalize_angle(angle: Angle) -> Angle {
    let a = angle.get::<radian>();
    let mut wrapped = libm::fmod(a + PI, 2.0 * PI);
    if wrapped <= 0.0 {
        wrapped += 2.0 * PI;
    }
    Angle::new::<radian>(wrapped - PI)
}

/// A position in a planar frame.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(deny_unknown_fields)]
pub struct CuPoint2 {
    pub x: Length,
    pub y: Length,
}

impl CuPoint2 {
    /// Creates a point from its coordinates.
    pub fn new(x: Length, y: Length) -> Self {
        Self { x, y }
    }

    /// Euclidean distance to `other`.
    pub fn distance(&self, other: &Self) -> Length {
        let dx = self.x.get::<meter>() - other.x.get::<meter>();
        let dy = self.y.get::<meter>() - other.y.get::<meter>();
        Length::new::<meter>(libm::hypot(dx, dy))
    }
}

/// A position in a 3D frame.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(deny_unknown_fields)]
pub struct CuPoint3 {
    pub x: Length,
    pub y: Length,
    pub z: Length,
}

impl CuPoint3 {
    /// Creates a point from its coordinates.
    pub fn new(x: Length, y: Length, z: Length) -> Self {
        Self { x, y, z }
    }

    /// Euclidean distance to `other`.
    pub fn distance(&self, other: &Self) -> Length {
        let dx = self.x.get::<meter>() - other.x.get::<meter>();
        let dy = self.y.get::<meter>() - other.y.get::<meter>();
        let dz = self.z.get::<meter>() - other.z.get::<meter>();
        Length::new::<meter>(libm::sqrt(dx * dx + dy * dy + dz * dz))
    }

    /// Drops `z`, projecting onto the planar frame.
    pub fn xy(&self) -> CuPoint2 {
        CuPoint2::new(self.x, self.y)
    }
}

/// A planar pose: position plus heading, `theta` measured counter-clockwise
/// from the frame's `x` axis.
///
/// Read as the transform from the pose's own (child) frame to its parent
/// frame: [`CuPose2::transform_point`] maps child coordinates to parent
/// coordinates.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(deny_unknown_fields)]
pub struct CuPose2 {
    pub x: Length,
    pub y: Length,
    pub theta: Angle,
}

impl CuPose2 {
    /// Creates a pose; `theta` is wrapped into `(-π, π]`.
    pub fn new(x: Length, y: Length, theta: Angle) -> Self {
        Self {
            x,
            y,
            theta: normalize_angle(theta),
        }
    }

    /// The pose's position.
    pub fn position(&self) -> CuPoint2 {
        CuPoint2::new(self.x, self.y)
    }

    /// Maps a point given in this pose's frame into the parent frame.
    pub fn transform_point(&self, point: CuPoint2) -> CuPoint2 {
        let (s, c) = libm::sincos(self.theta.get::<radian>());
        let px = point.x.get::<meter>();
        let py = point.y.get::<meter>();
        CuPoint2::new(
            Length::new::<meter>(self.x.get::<meter>() + c * px - s * py),
            Length::new::<meter>(self.y.get::<meter>() + s * px + c * py),
        )
    }

    /// `self ∘ other`: `other` is expressed in this pose's frame; the result is
    /// expressed in this pose's parent frame.
    pub fn compose(&self, other: &Self) -> Self {
        let p = self.transform_point(other.position());
        Self::new(p.x, p.y, self.theta + other.theta)
    }

    /// The inverse transform (parent expressed in this pose's frame).
    pub fn inverse(&self) -> Self {
        let (s, c) = libm::sincos(self.theta.get::<radian>());
        let x = self.x.get::<meter>();
        let y = self.y.get::<meter>();
        Self::new(
            Length::new::<meter>(-c * x - s * y),
            Length::new::<meter>(s * x - c * y),
            Angle::new::<radian>(-self.theta.get::<radian>()),
        )
    }

    /// Advances the pose by `twist` (body frame) held constant for `dt`.
    ///
    /// Integration is exact on the resulting arc, so it is the odometry step
    /// for any planar base; for `|ω·dt|` near zero it falls back to the
    /// straight-line limit.
    pub fn integrate(&self, twist: &CuTwist2, dt: Time) -> Self {
        let dt = dt.get::<second>();
        let vx = twist.linear_x.get::<meter_per_second>();
        let vy = twist.linear_y.get::<meter_per_second>();
        let dtheta = twist.angular.get::<radian_per_second>() * dt;
        // Body-frame displacement over the arc: V(dθ) · v · dt.
        let (a, b) = if libm::fabs(dtheta) < 1e-9 {
            (1.0 - dtheta * dtheta / 6.0, dtheta / 2.0)
        } else {
            let (s, c) = libm::sincos(dtheta);
            (s / dtheta, (1.0 - c) / dtheta)
        };
        let local = CuPoint2::new(
            Length::new::<meter>((a * vx - b * vy) * dt),
            Length::new::<meter>((b * vx + a * vy) * dt),
        );
        let p = self.transform_point(local);
        Self::new(p.x, p.y, self.theta + Angle::new::<radian>(dtheta))
    }
}

/// A planar velocity in the body frame: forward (`linear_x`), leftward
/// (`linear_y`, zero for non-holonomic bases) and yaw rate (`angular`).
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(deny_unknown_fields)]
pub struct CuTwist2 {
    pub linear_x: Velocity,
    pub linear_y: Velocity,
    pub angular: AngularVelocity,
}

impl CuTwist2 {
    /// Creates a twist from its components.
    pub fn new(linear_x: Velocity, linear_y: Velocity, angular: AngularVelocity) -> Self {
        Self {
            linear_x,
            linear_y,
            angular,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn m(v: f64) -> Length {
        Length::new::<meter>(v)
    }
    fn rad(v: f64) -> Angle {
        Angle::new::<radian>(v)
    }
    fn close(a: f64, b: f64) -> bool {
        libm::fabs(a - b) < 1e-9
    }
    fn pose_close(a: &CuPose2, b: &CuPose2) -> bool {
        close(a.x.get::<meter>(), b.x.get::<meter>())
            && close(a.y.get::<meter>(), b.y.get::<meter>())
            && close(normalize_angle(a.theta - b.theta).get::<radian>(), 0.0)
    }

    #[test]
    fn test_normalize_angle_wraps() {
        assert!(close(normalize_angle(rad(3.0 * PI)).get::<radian>(), PI));
        assert!(close(normalize_angle(rad(-PI)).get::<radian>(), PI));
        assert!(close(normalize_angle(rad(-0.5)).get::<radian>(), -0.5));
        assert!(close(
            normalize_angle(rad(2.0 * PI + 0.25)).get::<radian>(),
            0.25
        ));
    }

    #[test]
    fn test_transform_point_rotates_then_translates() {
        let pose = CuPose2::new(m(1.0), m(2.0), rad(PI / 2.0));
        let p = pose.transform_point(CuPoint2::new(m(1.0), m(0.0)));
        assert!(close(p.x.get::<meter>(), 1.0));
        assert!(close(p.y.get::<meter>(), 3.0));
    }

    #[test]
    fn test_compose_with_inverse_is_identity() {
        let pose = CuPose2::new(m(1.5), m(-0.7), rad(2.1));
        let id = pose.compose(&pose.inverse());
        assert!(pose_close(&id, &CuPose2::default()));
        let id = pose.inverse().compose(&pose);
        assert!(pose_close(&id, &CuPose2::default()));
    }

    #[test]
    fn test_integrate_straight_line() {
        let twist = CuTwist2::new(
            Velocity::new::<meter_per_second>(0.5),
            Velocity::default(),
            AngularVelocity::default(),
        );
        let start = CuPose2::new(m(0.0), m(0.0), rad(PI / 2.0));
        let end = start.integrate(&twist, Time::new::<second>(2.0));
        assert!(pose_close(
            &end,
            &CuPose2::new(m(0.0), m(1.0), rad(PI / 2.0))
        ));
    }

    #[test]
    fn test_integrate_quarter_circle() {
        // v = 1 m/s, ω = π/2 rad/s for 1 s: quarter circle of radius 2/π.
        let twist = CuTwist2::new(
            Velocity::new::<meter_per_second>(1.0),
            Velocity::default(),
            AngularVelocity::new::<radian_per_second>(PI / 2.0),
        );
        let end = CuPose2::default().integrate(&twist, Time::new::<second>(1.0));
        let r = 2.0 / PI;
        assert!(pose_close(&end, &CuPose2::new(m(r), m(r), rad(PI / 2.0))));
    }

    #[test]
    fn test_integrate_matches_many_small_steps() {
        let twist = CuTwist2::new(
            Velocity::new::<meter_per_second>(0.8),
            Velocity::new::<meter_per_second>(0.2),
            AngularVelocity::new::<radian_per_second>(-0.9),
        );
        let one = CuPose2::default().integrate(&twist, Time::new::<second>(1.0));
        let mut many = CuPose2::default();
        for _ in 0..1000 {
            many = many.integrate(&twist, Time::new::<second>(0.001));
        }
        assert!(pose_close(&one, &many));
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_pose_ron_round_trip() {
        let pose: CuPose2 = ron::from_str("(x: 1.0, y: -2.0, theta: 0.5)").unwrap();
        assert!(pose_close(&pose, &CuPose2::new(m(1.0), m(-2.0), rad(0.5))));
        let text = ron::to_string(&pose).unwrap();
        assert_eq!(ron::from_str::<CuPose2>(&text).unwrap(), pose);
    }
}
