//! Regulated pure pursuit path following for differential-drive robots.

use crate::params::CuPurePursuitParams;
use bincode::Decode;
use bincode::Encode;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::{AngularVelocity, Length, Velocity};
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;
use meander_core::CuPoint2;
use meander_core::CuPose2;
use meander_core::CuTwist2;
use meander_core::normalize_angle;
use serde::Deserialize;
use serde::Serialize;

/// Status outcome of one controller compute step.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuFollowStatus {
    /// The path was empty; zero twist was commanded.
    #[default]
    EmptyPath,
    /// Actively tracking the path segments.
    Following,
    /// At the goal position; rotating in place to align with the goal heading.
    RotatingToGoal,
    /// Reached both the goal position and heading within tolerances; stopped.
    GoalReached,
}

/// Regulated pure pursuit controller for differential-drive mobile robots.
#[derive(Debug, Clone, PartialEq)]
pub struct CuPurePursuit {
    params: CuPurePursuitParams,
    progress_index: usize,
}

impl CuPurePursuit {
    /// Creates a pure pursuit controller, validating `params`.
    pub fn from_params(params: CuPurePursuitParams) -> CuResult<Self> {
        params.validate()?;
        Ok(Self {
            params,
            progress_index: 0,
        })
    }

    /// The validated parameters.
    pub fn params(&self) -> &CuPurePursuitParams {
        &self.params
    }

    /// The current path progress index.
    pub fn progress_index(&self) -> usize {
        self.progress_index
    }

    /// Resets progress tracking for a new path.
    pub fn reset(&mut self) {
        self.progress_index = 0;
    }

    /// Restores a progress index saved by a `Freezable` task wrapper.
    pub(crate) fn restore_progress_index(&mut self, index: usize) {
        self.progress_index = index;
    }

    /// Computes the twist command to track `path` from `pose`.
    ///
    /// Writes the body-frame twist into `out` and returns the follow status.
    pub fn compute(
        &mut self,
        pose: &CuPose2,
        path: &[CuPose2],
        out: &mut CuTwist2,
    ) -> CuResult<CuFollowStatus> {
        if path.is_empty() {
            *out = CuTwist2::default();
            return Ok(CuFollowStatus::EmptyPath);
        }

        let goal = &path[path.len() - 1];
        let dist_to_goal = pose.position().distance(&goal.position());

        // At the goal position: align heading or stop.
        if dist_to_goal <= self.params.goal_tolerance {
            self.progress_index = path.len() - 1;
            let yaw_err = normalize_angle(goal.theta - pose.theta).get::<radian>();
            if libm::fabs(yaw_err) <= self.params.yaw_tolerance.get::<radian>() {
                *out = CuTwist2::default();
                return Ok(CuFollowStatus::GoalReached);
            }

            let max_omega = self.params.max_angular.get::<radian_per_second>();
            let omega = if yaw_err > 0.0 { max_omega } else { -max_omega };
            *out = CuTwist2::new(
                Velocity::default(),
                Velocity::default(),
                AngularVelocity::new::<radian_per_second>(omega),
            );
            return Ok(CuFollowStatus::RotatingToGoal);
        }

        // Advance progress to the closest waypoint at or after current progress index.
        let mut closest_idx = self.progress_index.min(path.len() - 1);
        let mut min_dist_sq = {
            let d = pose
                .position()
                .distance(&path[closest_idx].position())
                .get::<meter>();
            d * d
        };
        for (i, pt) in path.iter().enumerate().skip(closest_idx + 1) {
            let d = pose.position().distance(&pt.position()).get::<meter>();
            let d_sq = d * d;
            if d_sq < min_dist_sq {
                min_dist_sq = d_sq;
                closest_idx = i;
            }
        }
        self.progress_index = closest_idx;

        // Find lookahead point along segments.
        let target = self.find_lookahead_point(pose, path, closest_idx);

        // Transform target into the robot's body frame.
        let local = pose.inverse().transform_point(target);
        let x_local = local.x.get::<meter>();
        let y_local = local.y.get::<meter>();

        let alpha = libm::atan2(y_local, x_local);

        // Rotate in place when target is at a large angle.
        if libm::fabs(alpha) > self.params.rotate_in_place_angle.get::<radian>() {
            let max_omega = self.params.max_angular.get::<radian_per_second>();
            let omega = if alpha > 0.0 { max_omega } else { -max_omega };
            *out = CuTwist2::new(
                Velocity::default(),
                Velocity::default(),
                AngularVelocity::new::<radian_per_second>(omega),
            );
            return Ok(CuFollowStatus::Following);
        }

        // Curvature κ = 2 · y_local / L²
        let l = self.params.lookahead.get::<meter>();
        let kappa = 2.0 * y_local / (l * l);

        // Linear velocity scaled near goal.
        let v_max = self.params.max_linear.get::<meter_per_second>();
        let d_goal = dist_to_goal.get::<meter>();
        let r_slow = self.params.slowdown_radius.get::<meter>();
        let scale = if r_slow > 0.0 && d_goal < r_slow {
            d_goal / r_slow
        } else {
            1.0
        };
        let v = v_max * scale;

        // Clamped angular velocity; scale v down if clamped to preserve arc curvature.
        let max_omega = self.params.max_angular.get::<radian_per_second>();
        let omega_raw = v * kappa;
        let (v_final, omega_final) = if libm::fabs(omega_raw) > max_omega {
            let ratio = max_omega / libm::fabs(omega_raw);
            (v * ratio, omega_raw * ratio)
        } else {
            (v, omega_raw)
        };

        *out = CuTwist2::new(
            Velocity::new::<meter_per_second>(v_final),
            Velocity::default(),
            AngularVelocity::new::<radian_per_second>(omega_final),
        );
        Ok(CuFollowStatus::Following)
    }

    /// Finds the lookahead target point by interpolating along path segments.
    fn find_lookahead_point(&self, pose: &CuPose2, path: &[CuPose2], start_idx: usize) -> CuPoint2 {
        let l = self.params.lookahead.get::<meter>();
        let c = pose.position();
        let cx = c.x.get::<meter>();
        let cy = c.y.get::<meter>();

        // Pass 1: first segment at or after start_idx that exits the lookahead circle.
        for i in start_idx..(path.len().saturating_sub(1)) {
            let a = path[i].position();
            let b = path[i + 1].position();

            let ax = a.x.get::<meter>();
            let ay = a.y.get::<meter>();
            let bx = b.x.get::<meter>();
            let by = b.y.get::<meter>();

            let dx = bx - ax;
            let dy = by - ay;
            let seg_len_sq = dx * dx + dy * dy;
            if seg_len_sq < 1e-12 {
                continue;
            }

            let fx = ax - cx;
            let fy = ay - cy;

            let a_q = seg_len_sq;
            let b_q = 2.0 * (fx * dx + fy * dy);
            let c_q = (fx * fx + fy * fy) - l * l;

            let discriminant = b_q * b_q - 4.0 * a_q * c_q;
            if discriminant >= 0.0 {
                let sqrt_disc = libm::sqrt(discriminant);
                let t2 = (-b_q + sqrt_disc) / (2.0 * a_q);

                if (0.0..=1.0).contains(&t2) {
                    return CuPoint2::new(
                        Length::new::<meter>(ax + t2 * dx),
                        Length::new::<meter>(ay + t2 * dy),
                    );
                }
            }
        }

        // Pass 2: if no segment exited the circle and the goal is within L, target the goal.
        let goal_pos = path[path.len() - 1].position();
        if pose.position().distance(&goal_pos).get::<meter>() <= l {
            return goal_pos;
        }

        // Pass 3: segment entering the circle from outside.
        for i in start_idx..(path.len().saturating_sub(1)) {
            let a = path[i].position();
            let b = path[i + 1].position();

            let ax = a.x.get::<meter>();
            let ay = a.y.get::<meter>();
            let bx = b.x.get::<meter>();
            let by = b.y.get::<meter>();

            let dx = bx - ax;
            let dy = by - ay;
            let seg_len_sq = dx * dx + dy * dy;
            if seg_len_sq < 1e-12 {
                continue;
            }

            let fx = ax - cx;
            let fy = ay - cy;

            let a_q = seg_len_sq;
            let b_q = 2.0 * (fx * dx + fy * dy);
            let c_q = (fx * fx + fy * fy) - l * l;

            let discriminant = b_q * b_q - 4.0 * a_q * c_q;
            if discriminant >= 0.0 {
                let sqrt_disc = libm::sqrt(discriminant);
                let t1 = (-b_q - sqrt_disc) / (2.0 * a_q);

                if (0.0..=1.0).contains(&t1) {
                    return CuPoint2::new(
                        Length::new::<meter>(ax + t1 * dx),
                        Length::new::<meter>(ay + t1 * dy),
                    );
                }
            }
        }

        // Pass 4: fallback to start_idx waypoint.
        path[start_idx].position()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;
    use cu29::units::si::f64::Angle;
    use cu29::units::si::f64::Time;
    use cu29::units::si::time::second;

    fn m(v: f64) -> Length {
        Length::new::<meter>(v)
    }

    fn rad(v: f64) -> Angle {
        Angle::new::<radian>(v)
    }

    #[test]
    fn test_straight_path_drives_straight() {
        let mut controller = CuPurePursuit::from_params(CuPurePursuitParams::default()).unwrap();
        let pose = CuPose2::default();
        let path = [
            CuPose2::new(m(0.0), m(0.0), rad(0.0)),
            CuPose2::new(m(1.0), m(0.0), rad(0.0)),
            CuPose2::new(m(2.0), m(0.0), rad(0.0)),
        ];
        let mut twist = CuTwist2::default();
        let status = controller.compute(&pose, &path, &mut twist).unwrap();

        assert_eq!(status, CuFollowStatus::Following);
        assert!(twist.linear_x.get::<meter_per_second>() > 0.0);
        assert_eq!(twist.linear_y.get::<meter_per_second>(), 0.0);
        assert!(libm::fabs(twist.angular.get::<radian_per_second>()) < 1e-6);
    }

    #[test]
    fn test_path_to_the_left_gives_positive_omega() {
        let mut controller = CuPurePursuit::from_params(CuPurePursuitParams::default()).unwrap();
        let pose = CuPose2::default();
        let path = [
            CuPose2::new(m(0.0), m(0.0), rad(0.0)),
            CuPose2::new(m(1.0), m(0.5), rad(0.0)),
        ];
        let mut twist = CuTwist2::default();
        let status = controller.compute(&pose, &path, &mut twist).unwrap();

        assert_eq!(status, CuFollowStatus::Following);
        assert!(twist.angular.get::<radian_per_second>() > 0.0);
    }

    #[test]
    fn test_target_behind_triggers_rotate_in_place() {
        let mut controller = CuPurePursuit::from_params(CuPurePursuitParams::default()).unwrap();
        let pose = CuPose2::default();
        let path = [
            CuPose2::new(m(-1.0), m(0.0), rad(0.0)),
            CuPose2::new(m(-2.0), m(0.0), rad(0.0)),
        ];
        let mut twist = CuTwist2::default();
        let status = controller.compute(&pose, &path, &mut twist).unwrap();

        assert_eq!(status, CuFollowStatus::Following);
        assert_eq!(twist.linear_x.get::<meter_per_second>(), 0.0);
        assert!(libm::fabs(twist.angular.get::<radian_per_second>()) > 0.0);
    }

    #[test]
    fn test_goal_reached_stops_and_status() {
        let mut controller = CuPurePursuit::from_params(CuPurePursuitParams::default()).unwrap();
        let pose = CuPose2::new(m(1.0), m(0.0), rad(0.0));
        let path = [
            CuPose2::new(m(0.0), m(0.0), rad(0.0)),
            CuPose2::new(m(1.0), m(0.0), rad(0.0)),
        ];
        let mut twist = CuTwist2::default();
        let status = controller.compute(&pose, &path, &mut twist).unwrap();

        assert_eq!(status, CuFollowStatus::GoalReached);
        assert_eq!(twist.linear_x.get::<meter_per_second>(), 0.0);
        assert_eq!(twist.linear_y.get::<meter_per_second>(), 0.0);
        assert_eq!(twist.angular.get::<radian_per_second>(), 0.0);
    }

    #[test]
    fn test_rotating_to_goal_at_goal_position() {
        let mut controller = CuPurePursuit::from_params(CuPurePursuitParams::default()).unwrap();
        let pose = CuPose2::new(m(1.0), m(0.0), rad(0.0));
        let path = [
            CuPose2::new(m(0.0), m(0.0), rad(0.0)),
            CuPose2::new(m(1.0), m(0.0), rad(PI / 2.0)),
        ];
        let mut twist = CuTwist2::default();
        let status = controller.compute(&pose, &path, &mut twist).unwrap();

        assert_eq!(status, CuFollowStatus::RotatingToGoal);
        assert_eq!(twist.linear_x.get::<meter_per_second>(), 0.0);
        assert_eq!(twist.linear_y.get::<meter_per_second>(), 0.0);
        assert!(twist.angular.get::<radian_per_second>() > 0.0);
    }

    #[test]
    fn test_slowdown_near_goal() {
        let mut controller = CuPurePursuit::from_params(CuPurePursuitParams::default()).unwrap();
        let path = [
            CuPose2::new(m(0.0), m(0.0), rad(0.0)),
            CuPose2::new(m(1.0), m(0.0), rad(0.0)),
        ];

        let mut twist_far = CuTwist2::default();
        controller
            .compute(
                &CuPose2::new(m(0.0), m(0.0), rad(0.0)),
                &path,
                &mut twist_far,
            )
            .unwrap();

        let mut twist_near = CuTwist2::default();
        controller
            .compute(
                &CuPose2::new(m(0.8), m(0.0), rad(0.0)),
                &path,
                &mut twist_near,
            )
            .unwrap();

        assert!(
            twist_near.linear_x.get::<meter_per_second>()
                < twist_far.linear_x.get::<meter_per_second>()
        );
    }

    #[test]
    fn test_progress_index_never_decreases() {
        let mut controller = CuPurePursuit::from_params(CuPurePursuitParams::default()).unwrap();
        let path = [
            CuPose2::new(m(0.0), m(0.0), rad(0.0)),
            CuPose2::new(m(1.0), m(0.0), rad(0.0)),
            CuPose2::new(m(2.0), m(0.0), rad(0.0)),
            CuPose2::new(m(3.0), m(0.0), rad(0.0)),
        ];
        let mut twist = CuTwist2::default();

        controller
            .compute(&CuPose2::new(m(0.0), m(0.0), rad(0.0)), &path, &mut twist)
            .unwrap();
        let p0 = controller.progress_index();

        controller
            .compute(&CuPose2::new(m(1.1), m(0.0), rad(0.0)), &path, &mut twist)
            .unwrap();
        let p1 = controller.progress_index();
        assert!(p1 >= p0);

        // Even if robot is placed back near the start, progress does not regress
        controller
            .compute(&CuPose2::new(m(0.0), m(0.0), rad(0.0)), &path, &mut twist)
            .unwrap();
        let p2 = controller.progress_index();
        assert!(p2 >= p1);

        controller.reset();
        assert_eq!(controller.progress_index(), 0);
    }

    #[test]
    fn test_closed_loop_simulation_l_shaped_path() {
        let mut controller = CuPurePursuit::from_params(CuPurePursuitParams::default()).unwrap();
        let path = [
            CuPose2::new(m(0.0), m(0.0), rad(0.0)),
            CuPose2::new(m(1.5), m(0.0), rad(0.0)),
            CuPose2::new(m(1.5), m(1.5), rad(PI / 2.0)),
        ];

        let mut pose = CuPose2::default();
        let dt = Time::new::<second>(0.05);
        let mut twist = CuTwist2::default();
        let mut reached = false;

        for _ in 0..1000 {
            let status = controller.compute(&pose, &path, &mut twist).unwrap();
            if status == CuFollowStatus::GoalReached {
                reached = true;
                break;
            }
            pose = pose.integrate(&twist, dt);
        }

        assert!(reached, "Closed loop simulation should reach the goal");
        let goal = path[path.len() - 1];
        let pos_err = pose.position().distance(&goal.position()).get::<meter>();
        let yaw_err = libm::fabs(normalize_angle(goal.theta - pose.theta).get::<radian>());

        assert!(
            pos_err <= 0.1,
            "Position error {} should be <= 0.1 m",
            pos_err
        );
        assert!(
            yaw_err <= 0.15,
            "Yaw error {} should be <= 0.15 rad",
            yaw_err
        );
    }
}
