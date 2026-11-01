//! Parameter configuration for path tracking controllers.

use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::{Angle, AngularVelocity, Length, Velocity};
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;
use serde::Deserialize;
use serde::Serialize;

/// Parameters for regulated pure pursuit path following.
///
/// ```ron
/// config: {
///     "lookahead": 0.4,
///     "max_linear": 0.5,
///     "max_angular": 1.5,
///     "goal_tolerance": 0.1,
///     "yaw_tolerance": 0.15,
///     "rotate_in_place_angle": 0.8,
///     "slowdown_radius": 0.5,
/// }
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuPurePursuitParams {
    /// Distance ahead of the robot along the path to track.
    pub lookahead: Length,
    /// Maximum linear velocity commanded along the arc.
    pub max_linear: Velocity,
    /// Maximum angular velocity commanded.
    pub max_angular: AngularVelocity,
    /// Distance threshold from the final waypoint considered at the goal.
    pub goal_tolerance: Length,
    /// Orientation threshold from the final waypoint considered oriented.
    pub yaw_tolerance: Angle,
    /// Angle threshold beyond which the robot rotates in place before driving.
    pub rotate_in_place_angle: Angle,
    /// Distance to the goal within which linear velocity scales down smoothly.
    pub slowdown_radius: Length,
}

impl Default for CuPurePursuitParams {
    fn default() -> Self {
        Self {
            lookahead: Length::new::<meter>(0.4),
            max_linear: Velocity::new::<meter_per_second>(0.5),
            max_angular: AngularVelocity::new::<radian_per_second>(1.5),
            goal_tolerance: Length::new::<meter>(0.1),
            yaw_tolerance: Angle::new::<radian>(0.15),
            rotate_in_place_angle: Angle::new::<radian>(0.8),
            slowdown_radius: Length::new::<meter>(0.5),
        }
    }
}

impl CuPurePursuitParams {
    /// Rejects invalid, non-finite, zero or negative limits and tolerances.
    pub fn validate(&self) -> CuResult<()> {
        let l = self.lookahead.get::<meter>();
        if !l.is_finite() || l <= 0.0 {
            return Err(CuError::from(
                "meander-controller: lookahead must be finite and positive",
            ));
        }
        let v = self.max_linear.get::<meter_per_second>();
        if !v.is_finite() || v <= 0.0 {
            return Err(CuError::from(
                "meander-controller: max_linear must be finite and positive",
            ));
        }
        let w = self.max_angular.get::<radian_per_second>();
        if !w.is_finite() || w <= 0.0 {
            return Err(CuError::from(
                "meander-controller: max_angular must be finite and positive",
            ));
        }
        let gt = self.goal_tolerance.get::<meter>();
        if !gt.is_finite() || gt <= 0.0 {
            return Err(CuError::from(
                "meander-controller: goal_tolerance must be finite and positive",
            ));
        }
        let yt = self.yaw_tolerance.get::<radian>();
        if !yt.is_finite() || yt <= 0.0 {
            return Err(CuError::from(
                "meander-controller: yaw_tolerance must be finite and positive",
            ));
        }
        let rip = self.rotate_in_place_angle.get::<radian>();
        if !rip.is_finite() || rip <= 0.0 {
            return Err(CuError::from(
                "meander-controller: rotate_in_place_angle must be finite and positive",
            ));
        }
        let sr = self.slowdown_radius.get::<meter>();
        if !sr.is_finite() || sr < 0.0 {
            return Err(CuError::from(
                "meander-controller: slowdown_radius must be finite and non-negative",
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_params_validate() {
        let params = CuPurePursuitParams::default();
        assert!(params.validate().is_ok());
    }

    #[test]
    fn test_invalid_params_rejected() {
        let p = CuPurePursuitParams {
            lookahead: Length::new::<meter>(-0.1),
            ..Default::default()
        };
        assert!(p.validate().is_err());

        let p = CuPurePursuitParams {
            max_linear: Velocity::new::<meter_per_second>(0.0),
            ..Default::default()
        };
        assert!(p.validate().is_err());

        let p = CuPurePursuitParams {
            max_angular: AngularVelocity::new::<radian_per_second>(-1.0),
            ..Default::default()
        };
        assert!(p.validate().is_err());

        let p = CuPurePursuitParams {
            goal_tolerance: Length::new::<meter>(0.0),
            ..Default::default()
        };
        assert!(p.validate().is_err());

        let p = CuPurePursuitParams {
            yaw_tolerance: Angle::new::<radian>(-0.1),
            ..Default::default()
        };
        assert!(p.validate().is_err());

        let p = CuPurePursuitParams {
            rotate_in_place_angle: Angle::new::<radian>(0.0),
            ..Default::default()
        };
        assert!(p.validate().is_err());

        let p = CuPurePursuitParams {
            slowdown_radius: Length::new::<meter>(-0.5),
            ..Default::default()
        };
        assert!(p.validate().is_err());
    }
}
