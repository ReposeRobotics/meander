//! Planner configuration: one params struct, identical in Rust and RON.

use alloc::string::String;
use cu29::prelude::*;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use meander_core::CuPose2;
use meander_costmap::CuCostmapParams;
use serde::Deserialize;
use serde::Serialize;

/// Parameters of [`CuBidirectionalDijkstra`](crate::CuBidirectionalDijkstra).
///
/// ```ron
/// config: {
///     "cost_weight": 1.0,
///     "lethal_cost": 253,
///     "allow_unknown": false,
///     "goal_tolerance": 0.0,
/// }
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuPlannerParams {
    /// Dimensionless weight of the costmap value in the edge cost
    /// `step * (1 + cost_weight * cell_cost / 252)`. Zero plans the
    /// shortest path; larger values steer around inflated cells.
    pub cost_weight: f64,
    /// Cells with a cost at or above this are blocked. Unknown (255) is
    /// governed by `allow_unknown` instead of this threshold.
    pub lethal_cost: u8,
    /// Whether unknown cells (255) are traversable. Allowed unknown cells
    /// contribute zero cost, like free cells.
    pub allow_unknown: bool,
    /// If the goal cell is blocked, the nearest traversable cell within this
    /// distance of the goal is used instead.
    pub goal_tolerance: Length,
}

impl Default for CuPlannerParams {
    fn default() -> Self {
        Self {
            cost_weight: 1.0,
            lethal_cost: 253,
            allow_unknown: false,
            goal_tolerance: Length::new::<meter>(0.0),
        }
    }
}

impl CuPlannerParams {
    /// Rejects a negative or nonfinite `cost_weight` or `goal_tolerance`,
    /// and a `lethal_cost` of zero, which would block free cells.
    pub fn validate(&self) -> CuResult<()> {
        if !self.cost_weight.is_finite() || self.cost_weight < 0.0 {
            return Err(CuError::from(
                "meander-planner: 'cost_weight' must be finite and >= 0",
            ));
        }
        if self.lethal_cost == 0 {
            return Err(CuError::from(
                "meander-planner: 'lethal_cost' must be >= 1 so free cells stay traversable",
            ));
        }
        let tolerance = self.goal_tolerance.get::<meter>();
        if !tolerance.is_finite() || tolerance < 0.0 {
            return Err(CuError::from(
                "meander-planner: 'goal_tolerance' must be finite and >= 0",
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_defaults_match_nav2_cost_scale() {
        let params = CuPlannerParams::default();
        assert_eq!(params.cost_weight, 1.0);
        assert_eq!(params.lethal_cost, 253);
        assert!(!params.allow_unknown);
        assert_eq!(params.goal_tolerance.get::<meter>(), 0.0);
        assert!(params.validate().is_ok());
    }

    #[test]
    fn test_validate_rejects_bad_values() {
        for weight in [f64::NAN, f64::INFINITY, -1.0] {
            let params = CuPlannerParams {
                cost_weight: weight,
                ..CuPlannerParams::default()
            };
            assert!(params.validate().is_err(), "weight={weight}");
        }
        let params = CuPlannerParams {
            lethal_cost: 0,
            ..CuPlannerParams::default()
        };
        assert!(params.validate().is_err());
        for tolerance in [f64::NAN, f64::INFINITY, -0.5] {
            let params = CuPlannerParams {
                goal_tolerance: Length::new::<meter>(tolerance),
                ..CuPlannerParams::default()
            };
            assert!(params.validate().is_err(), "tolerance={tolerance}");
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_params_ron_round_trip() {
        let params: CuPlannerParams = ron::from_str(
            "(cost_weight: 2.0, lethal_cost: 254, allow_unknown: true, goal_tolerance: 1.5)",
        )
        .unwrap();
        assert_eq!(params.cost_weight, 2.0);
        assert_eq!(params.lethal_cost, 254);
        assert!(params.allow_unknown);
        assert_eq!(params.goal_tolerance.get::<meter>(), 1.5);
        let text = ron::to_string(&params).unwrap();
        assert_eq!(ron::from_str::<CuPlannerParams>(&text).unwrap(), params);
        let partial: CuPlannerParams = ron::from_str("(cost_weight: 0.0)").unwrap();
        assert_eq!(partial.lethal_cost, CuPlannerParams::default().lethal_cost);
        assert!(ron::from_str::<CuPlannerParams>("(cost_weight: 1.0, bogus: 1)").is_err());
    }
}

/// Parameters of [`CuPlannerTask`](crate::CuPlannerTask): planner and costmap
/// settings, the goal, and the Nav2 map to plan on.
///
/// ```ron
/// config: {
///     "map_path": "target/waypoint-mapping/map.yaml",
///     "goal": (x: 2.8, y: 2.8, theta: 0.0),
/// }
/// ```
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuPlannerTaskParams {
    /// Bidirectional Dijkstra settings.
    pub planner: CuPlannerParams,
    /// Obstacle inflation applied to the loaded map.
    pub costmap: CuCostmapParams,
    /// Goal pose in the map frame.
    pub goal: CuPose2,
    /// Nav2 map YAML (`std` only) loaded and inflated in `CuTask::new`.
    /// `None` is only usable programmatically via
    /// [`CuPlannerTask::from_costmap`](crate::CuPlannerTask::from_costmap).
    pub map_path: Option<String>,
}

impl CuPlannerTaskParams {
    /// Rejects invalid planner/costmap settings, a non-finite goal, and an
    /// empty `map_path`.
    pub fn validate(&self) -> CuResult<()> {
        self.planner.validate()?;
        self.costmap.validate()?;
        let goal = self.goal;
        if !goal.x.get::<meter>().is_finite()
            || !goal.y.get::<meter>().is_finite()
            || !goal
                .theta
                .get::<cu29::units::si::angle::radian>()
                .is_finite()
        {
            return Err(CuError::from("meander-planner: goal must be finite"));
        }
        if self.map_path.as_deref() == Some("") {
            return Err(CuError::from("meander-planner: map_path must not be empty"));
        }
        Ok(())
    }
}
