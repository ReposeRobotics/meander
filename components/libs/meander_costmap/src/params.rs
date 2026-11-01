//! Inflation parameters shared by Rust and RON configuration.
use cu29::prelude::CuError;
use cu29::prelude::CuResult;
use cu29::prelude::Reflect;
// The real Reflect derive references this alias when cu29/reflect is enabled.
#[allow(unused_imports)]
use cu29::prelude::bevy_reflect;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

/// Circular robot footprint and exponential inflation policy.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuCostmapParams {
    /// Inscribed radius of the robot footprint.
    pub robot_radius: Length,
    /// Maximum distance from an obstacle receiving inflation costs.
    pub inflation_radius: Length,
    /// Exponential decay coefficient in inverse meters (default 3.0 m⁻¹).
    /// Multiplication by the clearance in meters produces a dimensionless exponent.
    pub cost_scaling_factor: f64,
    /// Treat unknown cells as lethal obstacles and inflate around them.
    /// Otherwise unknown cells retain `NO_INFORMATION`, even near obstacles.
    pub unknown_is_lethal: bool,
}

impl Default for CuCostmapParams {
    fn default() -> Self {
        Self {
            robot_radius: Length::new::<meter>(0.2),
            inflation_radius: Length::new::<meter>(0.55),
            cost_scaling_factor: 3.0,
            unknown_is_lethal: false,
        }
    }
}

impl CuCostmapParams {
    /// Requires finite nonnegative radii, inflation at least the robot radius,
    /// and a finite nonnegative decay coefficient. Zero gives constant decay costs.
    pub fn validate(&self) -> CuResult<()> {
        let robot = self.robot_radius.get::<meter>();
        let inflation = self.inflation_radius.get::<meter>();
        if !robot.is_finite() || robot < 0.0 || !inflation.is_finite() || inflation < robot {
            return Err(CuError::from(
                "meander-costmap: radii must be finite with 0 <= robot_radius <= inflation_radius",
            ));
        }
        if !self.cost_scaling_factor.is_finite() || self.cost_scaling_factor < 0.0 {
            return Err(CuError::from(
                "meander-costmap: cost_scaling_factor must be finite and nonnegative",
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validation_and_ron() {
        let defaults = CuCostmapParams::default();
        defaults.validate().unwrap();
        assert_eq!(ron::from_str::<CuCostmapParams>("()").unwrap(), defaults);
        let text = ron::to_string(&defaults).unwrap();
        assert_eq!(ron::from_str::<CuCostmapParams>(&text).unwrap(), defaults);
        let params: CuCostmapParams = ron::from_str("(robot_radius:0.1,inflation_radius:0.8,cost_scaling_factor:2.0,unknown_is_lethal:true)").unwrap();
        params.validate().unwrap();
        assert_eq!(params.robot_radius.get::<meter>(), 0.1);
        assert!(params.unknown_is_lethal);
        assert!(ron::from_str::<CuCostmapParams>("(robot_raduis:0.1)").is_err());
        for bad in [-1.0, f64::NAN, f64::INFINITY] {
            for params in [
                CuCostmapParams {
                    robot_radius: Length::new::<meter>(bad),
                    ..defaults.clone()
                },
                CuCostmapParams {
                    inflation_radius: Length::new::<meter>(bad),
                    ..defaults.clone()
                },
                CuCostmapParams {
                    cost_scaling_factor: bad,
                    ..defaults.clone()
                },
            ] {
                assert!(params.validate().is_err());
            }
        }
        assert!(
            CuCostmapParams {
                inflation_radius: Length::new::<meter>(0.1),
                ..defaults
            }
            .validate()
            .is_err()
        );
    }
}
