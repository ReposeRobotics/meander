//! Example: a minimal differential-drive robot model (two wheels on a common
//! axle) built on `meander-core` poses and twists.
//!
//! `urdf/meander_diff_drive.urdf` describes a primitive-geometry robot matching
//! the default wheel radius and separation. Its ground frame is `base_footprint`,
//! with x forward, y left, and z up; all URDF dimensions are in SI units.

#![cfg_attr(not(feature = "std"), no_std)]

use bincode::Decode;
use bincode::Encode;
use cu29::prelude::*;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::{AngularVelocity, Length, Time, Velocity};
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;
use meander_core::{CuPose2, CuTwist2};
use serde::Deserialize;
use serde::Serialize;

/// Geometry of a differential-drive base.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuDiffDriveParams {
    /// Radius of each drive wheel.
    pub wheel_radius: Length,
    /// Distance between the two wheel contact points along the axle.
    pub wheel_separation: Length,
}

impl Default for CuDiffDriveParams {
    fn default() -> Self {
        Self {
            wheel_radius: Length::new::<meter>(0.05),
            wheel_separation: Length::new::<meter>(0.3),
        }
    }
}

impl CuDiffDriveParams {
    /// Rejects non-positive or non-finite dimensions.
    pub fn validate(&self) -> CuResult<()> {
        let r = self.wheel_radius.get::<meter>();
        let s = self.wheel_separation.get::<meter>();
        if !(r.is_finite() && r > 0.0) {
            return Err(CuError::from("CuDiffDriveParams: wheel_radius must be > 0"));
        }
        if !(s.is_finite() && s > 0.0) {
            return Err(CuError::from(
                "CuDiffDriveParams: wheel_separation must be > 0",
            ));
        }
        Ok(())
    }
}

/// Wheel angular velocities of a differential-drive base; positive spins
/// drive the robot forward.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(deny_unknown_fields)]
pub struct CuWheelSpeeds2 {
    pub left: AngularVelocity,
    pub right: AngularVelocity,
}

impl CuWheelSpeeds2 {
    /// Creates wheel speeds from the left and right wheel rates.
    pub fn new(left: AngularVelocity, right: AngularVelocity) -> Self {
        Self { left, right }
    }
}

/// A differential-drive kinematic model (ideal rolling, no slip).
#[derive(Debug, Clone, PartialEq, Reflect)]
pub struct CuDiffDrive {
    params: CuDiffDriveParams,
}

impl CuDiffDrive {
    /// Builds the model from validated params.
    pub fn from_params(params: CuDiffDriveParams) -> CuResult<Self> {
        params.validate()?;
        Ok(Self { params })
    }

    /// The model's geometry.
    pub fn params(&self) -> &CuDiffDriveParams {
        &self.params
    }

    /// Forward kinematics: wheel speeds to body twist (`linear_y` is zero).
    pub fn twist(&self, wheels: &CuWheelSpeeds2, out: &mut CuTwist2) {
        let r = self.params.wheel_radius.get::<meter>();
        let s = self.params.wheel_separation.get::<meter>();
        let vl = wheels.left.get::<radian_per_second>() * r;
        let vr = wheels.right.get::<radian_per_second>() * r;
        *out = CuTwist2::new(
            Velocity::new::<meter_per_second>((vr + vl) / 2.0),
            Velocity::default(),
            AngularVelocity::new::<radian_per_second>((vr - vl) / s),
        );
    }

    /// Inverse kinematics: body twist to wheel speeds. `linear_y` is ignored,
    /// since a differential drive cannot move sideways.
    pub fn wheel_speeds(&self, twist: &CuTwist2, out: &mut CuWheelSpeeds2) {
        let r = self.params.wheel_radius.get::<meter>();
        let half = self.params.wheel_separation.get::<meter>() / 2.0;
        let v = twist.linear_x.get::<meter_per_second>();
        let w = twist.angular.get::<radian_per_second>();
        *out = CuWheelSpeeds2::new(
            AngularVelocity::new::<radian_per_second>((v - w * half) / r),
            AngularVelocity::new::<radian_per_second>((v + w * half) / r),
        );
    }

    /// Odometry step: advances `pose` in place by `wheels` held for `dt`.
    pub fn step(&self, pose: &mut CuPose2, wheels: &CuWheelSpeeds2, dt: Time) {
        let mut twist = CuTwist2::default();
        self.twist(wheels, &mut twist);
        *pose = pose.integrate(&twist, dt);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;
    use cu29::units::si::angle::radian;
    use cu29::units::si::f64::Angle;
    use cu29::units::si::time::second;

    fn close(a: f64, b: f64) -> bool {
        libm::fabs(a - b) < 1e-9
    }
    fn rps(v: f64) -> AngularVelocity {
        AngularVelocity::new::<radian_per_second>(v)
    }
    fn model() -> CuDiffDrive {
        // r = 0.1 m, separation = 0.5 m.
        CuDiffDrive::from_params(CuDiffDriveParams {
            wheel_radius: Length::new::<meter>(0.1),
            wheel_separation: Length::new::<meter>(0.5),
        })
        .unwrap()
    }

    #[test]
    fn test_rejects_bad_params() {
        let mut p = CuDiffDriveParams::default();
        assert!(CuDiffDrive::from_params(p.clone()).is_ok());
        p.wheel_radius = Length::new::<meter>(0.0);
        assert!(CuDiffDrive::from_params(p.clone()).is_err());
        p = CuDiffDriveParams::default();
        p.wheel_separation = Length::new::<meter>(f64::NAN);
        assert!(CuDiffDrive::from_params(p).is_err());
    }

    #[test]
    fn test_equal_wheels_drive_straight() {
        let mut t = CuTwist2::default();
        model().twist(&CuWheelSpeeds2::new(rps(10.0), rps(10.0)), &mut t);
        assert!(close(t.linear_x.get::<meter_per_second>(), 1.0));
        assert!(close(t.angular.get::<radian_per_second>(), 0.0));
    }

    #[test]
    fn test_opposite_wheels_spin_in_place() {
        let mut t = CuTwist2::default();
        model().twist(&CuWheelSpeeds2::new(rps(-5.0), rps(5.0)), &mut t);
        assert!(close(t.linear_x.get::<meter_per_second>(), 0.0));
        // (0.5 - (-0.5)) / 0.5 = 2 rad/s, counter-clockwise.
        assert!(close(t.angular.get::<radian_per_second>(), 2.0));
    }

    #[test]
    fn test_inverse_round_trips_forward() {
        let m = model();
        let twist = CuTwist2::new(
            Velocity::new::<meter_per_second>(0.7),
            Velocity::default(),
            rps(-1.3),
        );
        let mut wheels = CuWheelSpeeds2::default();
        m.wheel_speeds(&twist, &mut wheels);
        let mut back = CuTwist2::default();
        m.twist(&wheels, &mut back);
        assert!(close(back.linear_x.get::<meter_per_second>(), 0.7));
        assert!(close(back.angular.get::<radian_per_second>(), -1.3));
    }

    #[test]
    fn test_full_circle_returns_home() {
        // Right wheel faster: drive a closed circle in many small steps.
        let m = model();
        let wheels = CuWheelSpeeds2::new(rps(5.0), rps(15.0));
        let mut t = CuTwist2::default();
        m.twist(&wheels, &mut t);
        let period = 2.0 * PI / t.angular.get::<radian_per_second>();
        let n = 1000;
        let dt = Time::new::<second>(period / n as f64);
        let mut pose = CuPose2::default();
        for _ in 0..n {
            m.step(&mut pose, &wheels, dt);
        }
        assert!(libm::fabs(pose.x.get::<meter>()) < 1e-9);
        assert!(libm::fabs(pose.y.get::<meter>()) < 1e-9);
        assert!(close(
            meander_core::normalize_angle(pose.theta - Angle::default()).get::<radian>(),
            0.0
        ));
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_urdf_matches_default_params() {
        let document =
            roxmltree::Document::parse(include_str!("../urdf/meander_diff_drive.urdf")).unwrap();
        let robot = document.root_element();
        let params = CuDiffDriveParams::default();
        let mut origins = [[0.0_f64; 3]; 2];

        for (index, side) in ["left", "right"].iter().enumerate() {
            let joint_name = std::format!("{side}_wheel_joint");
            let joint = robot
                .children()
                .find(|node| {
                    node.has_tag_name("joint")
                        && node.attribute("name") == Some(joint_name.as_str())
                })
                .unwrap();
            assert_eq!(joint.attribute("type"), Some("continuous"));
            let origin = joint
                .children()
                .find(|node| node.has_tag_name("origin"))
                .unwrap();
            let coordinates: std::vec::Vec<f64> = origin
                .attribute("xyz")
                .unwrap()
                .split_whitespace()
                .map(|value| value.parse().unwrap())
                .collect();
            origins[index].copy_from_slice(&coordinates);

            let link_name = std::format!("{side}_wheel_link");
            let link = robot
                .children()
                .find(|node| {
                    node.has_tag_name("link") && node.attribute("name") == Some(link_name.as_str())
                })
                .unwrap();
            for element in ["visual", "collision"] {
                let geometry = link
                    .children()
                    .find(|node| node.has_tag_name(element))
                    .unwrap();
                let cylinder = geometry
                    .descendants()
                    .find(|node| node.has_tag_name("cylinder"))
                    .unwrap();
                let radius: f64 = cylinder.attribute("radius").unwrap().parse().unwrap();
                assert!(close(radius, params.wheel_radius.get::<meter>()));
            }
        }

        let distance = origins[0]
            .iter()
            .zip(origins[1])
            .map(|(left, right)| (left - right).powi(2))
            .sum::<f64>()
            .sqrt();
        assert!(close(distance, params.wheel_separation.get::<meter>()));
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_params_ron() {
        let p: CuDiffDriveParams = ron::from_str("(wheel_separation: 0.4)").unwrap();
        assert!(close(p.wheel_separation.get::<meter>(), 0.4));
        assert!(close(p.wheel_radius.get::<meter>(), 0.05));
        assert!(ron::from_str::<CuDiffDriveParams>("(wheel_base: 0.4)").is_err());
    }
}
