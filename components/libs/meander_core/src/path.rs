//! A fixed-capacity planar path: the message between a planner and a controller.

use crate::CuPose2;
use bincode::Decode;
use bincode::Encode;
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
use serde_big_array::BigArray;

/// Up to `N` poses from start to goal, in one frame (usually `map`).
///
/// Only `poses[..len]` belong to the path; [`Self::poses`] returns exactly
/// that prefix. Producers keep `len <= N`; readers clamp defensively.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(deny_unknown_fields)]
pub struct CuPath2<const N: usize> {
    #[serde(with = "BigArray")]
    pub poses: [CuPose2; N],
    pub len: usize,
}

impl<const N: usize> Default for CuPath2<N> {
    fn default() -> Self {
        Self {
            poses: [CuPose2::default(); N],
            len: 0,
        }
    }
}

impl<const N: usize> CuPath2<N> {
    /// The populated poses, start first.
    pub fn poses(&self) -> &[CuPose2] {
        &self.poses[..self.len.min(N)]
    }

    /// Whether the path has no poses.
    pub fn is_empty(&self) -> bool {
        self.poses().is_empty()
    }

    /// Replaces the path with `poses`; errors if it holds more than `N`.
    pub fn set(&mut self, poses: &[CuPose2]) -> CuResult<()> {
        let target = self
            .poses
            .get_mut(..poses.len())
            .ok_or_else(|| CuError::from("meander-core: path exceeds CuPath2 capacity"))?;
        target.copy_from_slice(poses);
        self.len = poses.len();
        Ok(())
    }

    /// Sum of the straight segments between consecutive poses.
    pub fn length(&self) -> Length {
        let total = self
            .poses()
            .windows(2)
            .map(|w| w[0].position().distance(&w[1].position()).get::<meter>())
            .sum();
        Length::new::<meter>(total)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::prelude::CuMsgPayload;
    use cu29::units::si::f64::Angle;

    fn pose(x: f64, y: f64) -> CuPose2 {
        CuPose2::new(
            Length::new::<meter>(x),
            Length::new::<meter>(y),
            Angle::default(),
        )
    }

    #[test]
    fn test_set_poses_and_length() {
        fn payload<T: CuMsgPayload>() {}
        payload::<CuPath2<64>>();

        let mut path = CuPath2::<4>::default();
        assert!(path.is_empty());
        path.set(&[pose(0.0, 0.0), pose(3.0, 0.0), pose(3.0, 4.0)])
            .unwrap();
        assert_eq!(path.poses().len(), 3);
        assert!((path.length().get::<meter>() - 7.0).abs() < 1e-12);
        assert!(path.set(&[pose(0.0, 0.0); 5]).is_err());
        assert_eq!(path.len, 3, "a rejected set leaves the path unchanged");

        path.len = 99;
        assert_eq!(path.poses().len(), 4, "oversized len is clamped");
    }

    #[test]
    fn test_bincode_round_trip() {
        let mut path = CuPath2::<40>::default();
        path.set(&[pose(1.0, 2.0), pose(3.0, 4.0)]).unwrap();
        let config = bincode::config::standard();
        let mut buf = [0u8; 2048];
        let n = bincode::encode_into_slice(path, &mut buf, config).unwrap();
        let (back, _): (CuPath2<40>, usize) =
            bincode::decode_from_slice(&buf[..n], config).unwrap();
        assert_eq!(back, path);
    }
}
