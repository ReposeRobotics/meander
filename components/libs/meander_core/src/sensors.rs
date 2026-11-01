//! Fixed-capacity sensor observations, without envelope timestamps.

use crate::CuPoint3;
use crate::CuPose2;
use crate::CuPose3;
use bincode::Decode;
use bincode::Encode;
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
use serde_big_array::BigArray;

/// A planar scan with `N` angularly ordered beams, in the sensor's `xy` plane.
///
/// A beam is valid only when its range is finite, positive, and within the
/// inclusive `[range_min, range_max]` bounds. Zero is the default invalid value;
/// NaN and infinity also represent invalid/no-return readings. Invalid beams
/// retain their angular slot. [`Self::valid_count`] counts usable returns.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(deny_unknown_fields)]
pub struct CuLaserScan<const N: usize> {
    pub angle_min: Angle,
    pub angle_increment: Angle,
    pub range_min: Length,
    pub range_max: Length,
    #[serde(with = "BigArray")]
    pub ranges: [Length; N],
}

impl<const N: usize> Default for CuLaserScan<N> {
    fn default() -> Self {
        Self {
            angle_min: Angle::default(),
            angle_increment: Angle::default(),
            range_min: Length::default(),
            range_max: Length::default(),
            ranges: [Length::default(); N],
        }
    }
}

impl<const N: usize> CuLaserScan<N> {
    /// Writes a beam endpoint in sensor coordinates. Returns false for an
    /// out-of-bounds index, invalid range or nonfinite angle; leaves `out` unchanged.
    pub fn beam_endpoint(&self, index: usize, out: &mut CuPoint3) -> bool {
        let Some(range) = self.ranges.get(index) else {
            return false;
        };
        let range = range.get::<meter>();
        let min = self.range_min.get::<meter>();
        let max = self.range_max.get::<meter>();
        let angle =
            self.angle_min.get::<radian>() + index as f64 * self.angle_increment.get::<radian>();
        if !range.is_finite()
            || range <= 0.0
            || !min.is_finite()
            || !max.is_finite()
            || min < 0.0
            || range < min
            || range > max
            || !angle.is_finite()
        {
            return false;
        }
        let (s, c) = libm::sincos(angle);
        *out = CuPoint3::new(
            Length::new::<meter>(range * c),
            Length::new::<meter>(range * s),
            Length::default(),
        );
        true
    }

    /// Number of valid endpoints; invalid angular metadata yields no endpoints.
    pub fn valid_count(&self) -> usize {
        let mut point = CuPoint3::default();
        (0..N)
            .filter(|&i| self.beam_endpoint(i, &mut point))
            .count()
    }

    /// Writes valid endpoints compactly into `out` in beam order and returns
    /// the written length. The remaining output slots are unchanged.
    pub fn transform_into(&self, pose: &CuPose3, out: &mut [CuPoint3; N]) -> usize {
        let mut len = 0;
        let mut point = CuPoint3::default();
        for i in 0..N {
            if self.beam_endpoint(i, &mut point) {
                pose.transform_point(&point, &mut out[len]);
                len += 1;
            }
        }
        len
    }

    /// Transforms valid endpoints using a planar sensor-to-parent pose.
    pub fn transform_planar_into(&self, pose: &CuPose2, out: &mut [CuPoint3; N]) -> usize {
        self.transform_into(&CuPose3::from_pose2(pose), out)
    }
}

/// A fixed-capacity point cloud in one sensor frame.
///
/// Only `points[..len]` are observations. Producers must keep `len <= N`;
/// transform helpers defensively clamp an oversized length to capacity.
/// Point coordinates are preserved, including nonfinite sensor readings.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(deny_unknown_fields)]
pub struct CuPointCloud3<const N: usize> {
    #[serde(with = "BigArray")]
    pub points: [CuPoint3; N],
    pub len: usize,
}

impl<const N: usize> Default for CuPointCloud3<N> {
    fn default() -> Self {
        Self {
            points: [CuPoint3::default(); N],
            len: 0,
        }
    }
}

impl<const N: usize> CuPointCloud3<N> {
    /// Writes the populated prefix in parent coordinates, returning its length.
    /// The remaining output slots are unchanged.
    pub fn transform_into(&self, pose: &CuPose3, out: &mut [CuPoint3; N]) -> usize {
        let len = self.len.min(N);
        for (point, target) in self.points[..len].iter().zip(out.iter_mut()) {
            pose.transform_point(point, target);
        }
        len
    }

    /// Transforms the populated prefix using a planar sensor-to-parent pose.
    pub fn transform_planar_into(&self, pose: &CuPose2, out: &mut [CuPoint3; N]) -> usize {
        self.transform_into(&CuPose3::from_pose2(pose), out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::FRAC_PI_2;
    use cu29::prelude::CuMsgPayload;

    fn m(v: f64) -> Length {
        Length::new::<meter>(v)
    }

    #[test]
    fn test_scan_filters_and_compacts() {
        let scan = CuLaserScan::<7> {
            angle_min: Angle::default(),
            angle_increment: Angle::new::<radian>(FRAC_PI_2),
            range_min: m(1.0),
            range_max: m(2.0),
            ranges: [
                m(1.0),
                m(2.0),
                m(0.0),
                m(f64::NAN),
                m(f64::INFINITY),
                m(0.5),
                m(3.0),
            ],
        };
        assert_eq!(scan.valid_count(), 2);
        let sentinel = CuPoint3::new(m(9.0), m(9.0), m(9.0));
        let mut point = sentinel;
        assert!(!scan.beam_endpoint(7, &mut point));
        assert_eq!(point, sentinel);
        let pose = CuPose2::new(m(3.0), m(4.0), Angle::new::<radian>(FRAC_PI_2));
        let mut out = [sentinel; 7];
        assert_eq!(scan.transform_planar_into(&pose, &mut out), 2);
        assert!(
            out[0]
                .distance(&CuPoint3::new(m(3.0), m(5.0), m(0.0)))
                .get::<meter>()
                < 1e-9
        );
        assert!(
            out[1]
                .distance(&CuPoint3::new(m(1.0), m(4.0), m(0.0)))
                .get::<meter>()
                < 1e-9
        );
        assert_eq!(out[2], sentinel);
        let mut invalid = scan;
        invalid.angle_min = Angle::new::<radian>(f64::NAN);
        assert_eq!(invalid.valid_count(), 0);
        assert_eq!(
            CuLaserScan::<0>::default().transform_into(&CuPose3::default(), &mut []),
            0
        );
    }

    #[test]
    fn test_spatial_sensor_transform() {
        let pose = CuPose3::new(
            CuPoint3::new(m(1.0), m(2.0), m(3.0)),
            crate::CuQuaternion::from_axis_angle(&[0.0, 1.0, 0.0], Angle::new::<radian>(FRAC_PI_2))
                .unwrap(),
        );
        let scan = CuLaserScan::<1> {
            range_max: m(5.0),
            ranges: [m(2.0)],
            ..Default::default()
        };
        let cloud = CuPointCloud3::<1> {
            points: [CuPoint3::new(m(2.0), m(0.0), m(0.0))],
            len: 1,
        };
        let mut scan_out = [CuPoint3::default(); 1];
        let mut cloud_out = [CuPoint3::default(); 1];
        assert_eq!(scan.transform_into(&pose, &mut scan_out), 1);
        assert_eq!(cloud.transform_into(&pose, &mut cloud_out), 1);
        assert_eq!(scan_out, cloud_out);
        assert!(
            scan_out[0]
                .distance(&CuPoint3::new(m(1.0), m(2.0), m(1.0)))
                .get::<meter>()
                < 1e-9
        );
    }

    #[test]
    fn test_cloud_prefix_and_capacity() {
        let mut cloud = CuPointCloud3::<40>::default();
        cloud.points[0] = CuPoint3::new(m(1.0), m(2.0), m(3.0));
        cloud.len = 1;
        let sentinel = CuPoint3::new(m(9.0), m(9.0), m(9.0));
        let mut out = [sentinel; 40];
        let pose = CuPose3::new(CuPoint3::new(m(2.0), m(-1.0), m(4.0)), Default::default());
        assert_eq!(cloud.transform_into(&pose, &mut out), 1);
        assert_eq!(out[0], CuPoint3::new(m(3.0), m(1.0), m(7.0)));
        assert_eq!(out[1], sentinel);
        cloud.len = usize::MAX;
        assert_eq!(cloud.transform_into(&pose, &mut out), 40);
        assert_eq!(
            CuPointCloud3::<0>::default().transform_into(&pose, &mut []),
            0
        );
    }

    #[test]
    fn test_large_array_payload_contract_and_bincode() {
        fn payload<T: CuMsgPayload>() {}
        payload::<CuLaserScan<40>>();
        payload::<CuPointCloud3<40>>();
        payload::<CuPose3>();
        let mut scan = CuLaserScan::<40> {
            range_max: m(10.0),
            ..Default::default()
        };
        scan.ranges[39] = m(3.0);
        let mut bytes = [0u8; 4096];
        let len =
            bincode::encode_into_slice(scan, &mut bytes, bincode::config::standard()).unwrap();
        let (decoded, used): (CuLaserScan<40>, usize) =
            bincode::decode_from_slice(&bytes[..len], bincode::config::standard()).unwrap();
        assert_eq!(decoded, scan);
        assert_eq!(used, len);
        let mut cloud = CuPointCloud3::<40> {
            len: 40,
            ..Default::default()
        };
        cloud.points[39] = CuPoint3::new(m(1.0), m(2.0), m(3.0));
        let len =
            bincode::encode_into_slice(cloud, &mut bytes, bincode::config::standard()).unwrap();
        let (decoded, _): (CuPointCloud3<40>, usize) =
            bincode::decode_from_slice(&bytes[..len], bincode::config::standard()).unwrap();
        assert_eq!(decoded, cloud);
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_large_array_ron_round_trip() {
        let scan = CuLaserScan::<40>::default();
        let text = ron::to_string(&scan).unwrap();
        assert_eq!(ron::from_str::<CuLaserScan<40>>(&text).unwrap(), scan);
        let cloud = CuPointCloud3::<40>::default();
        let text = ron::to_string(&cloud).unwrap();
        assert_eq!(ron::from_str::<CuPointCloud3<40>>(&text).unwrap(), cloud);
    }
}
