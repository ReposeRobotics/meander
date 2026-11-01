//! Planning outcome: one status enum plus waypoint count and total cost.

use bincode::Decode;
use bincode::Encode;
use cu29::prelude::*;
use cu29::units::si::f64::Length;
use serde::Deserialize;
use serde::Serialize;

/// Outcome of one [`plan`](crate::CuBidirectionalDijkstra::plan) call.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuPlanStatus {
    /// No path connects start and goal on traversable cells.
    #[default]
    NoPath,
    /// `out` holds the whole path.
    Found,
    /// The start cell is blocked; `out` is untouched.
    StartBlocked,
    /// The goal cell is blocked and no traversable cell lies within
    /// `goal_tolerance`; `out` is untouched.
    GoalBlocked,
    /// Start or goal lies outside the map; `out` is untouched.
    OutsideMap,
    /// The path is longer than `out`: its first `out.len()` waypoints were
    /// written and `len` is the full waypoint count.
    PathTooLong,
}

/// Waypoint count, outcome and total edge cost of one planned path.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(deny_unknown_fields)]
pub struct CuPlanReport {
    pub status: CuPlanStatus,
    /// Waypoint count of the full path: what `out` holds on [`Found`](CuPlanStatus::Found),
    /// what it would need on [`PathTooLong`](CuPlanStatus::PathTooLong), zero otherwise.
    pub len: usize,
    /// Total edge cost of the full path; zero unless waypoints were written.
    pub cost: Length,
}
