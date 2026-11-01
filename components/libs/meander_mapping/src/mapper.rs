//! Allocation-free mappers: fold one pose + one observation into a grid.

use crate::params::CuMappingParams;
use bincode::Decode;
use bincode::Encode;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use meander_core::CuLaserScan;
use meander_core::CuPoint3;
use meander_core::CuPointCloud3;
use meander_core::CuPose2;
use meander_core::CuPose3;
use meander_grid::CuGridGeometry2;
use meander_grid::CuGridGeometry3;
use meander_grid::CuOccupancyGrid;
use meander_grid::CuVoxelGrid;
use serde::Deserialize;
use serde::Serialize;

/// Outcome of folding one observation into a map.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuMapUpdateStatus {
    /// The pose or the observation was missing; the map is unchanged.
    #[default]
    NoInput,
    /// The observation was folded into the map.
    Updated,
    /// The sensor lies outside the map, so no ray can be traced; the map is unchanged.
    SensorOutsideMap,
}

/// Per-observation mapping report, published by the mapping tasks.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(deny_unknown_fields)]
pub struct CuMapUpdateReport {
    pub status: CuMapUpdateStatus,
    /// Rays that ended in a trusted return (free cells along, occupied at the end).
    pub hits: u32,
    /// Rays cleared without a hit (no return, or a return beyond `max_range`).
    pub clears: u32,
    /// Beams or points that could not be used (invalid angle or nonfinite point).
    pub skipped: u32,
}

/// Composes `map ← base` with `base ← sensor`.
fn sensor_in_map(map_from_base: &CuPose3, base_from_sensor: &CuPose3) -> CuPose3 {
    let mut out = CuPose3::default();
    map_from_base.compose(base_from_sensor, &mut out);
    out
}

/// The point `distance` from `from` towards `to`.
fn along(from: &CuPoint3, to: &CuPoint3, distance: f64) -> CuPoint3 {
    let d = [
        to.x.get::<meter>() - from.x.get::<meter>(),
        to.y.get::<meter>() - from.y.get::<meter>(),
        to.z.get::<meter>() - from.z.get::<meter>(),
    ];
    let norm = libm::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    let s = if norm > 0.0 { distance / norm } else { 0.0 };
    CuPoint3::new(
        Length::new::<meter>(from.x.get::<meter>() + d[0] * s),
        Length::new::<meter>(from.y.get::<meter>() + d[1] * s),
        Length::new::<meter>(from.z.get::<meter>() + d[2] * s),
    )
}

/// Builds a 2D log-odds occupancy map from planar laser scans.
#[derive(Debug, Clone, PartialEq)]
pub struct CuScanMapper2<const W: usize, const H: usize> {
    params: CuMappingParams,
    grid: CuOccupancyGrid<W, H>,
}

impl<const W: usize, const H: usize> CuScanMapper2<W, H> {
    /// Creates an all-unknown `W × H` map, validating `params`.
    pub fn from_params(params: CuMappingParams) -> CuResult<Self> {
        params.validate()?;
        let geometry = CuGridGeometry2::new(params.resolution, params.origin.xy())?;
        let grid = CuOccupancyGrid::from_params(geometry, params.update.clone())?;
        Ok(Self { params, grid })
    }

    /// The validated parameters.
    pub fn params(&self) -> &CuMappingParams {
        &self.params
    }

    /// The map built so far.
    pub fn grid(&self) -> &CuOccupancyGrid<W, H> {
        &self.grid
    }

    /// Mutable map access, e.g. to restore a frozen state or load a saved map.
    pub fn grid_mut(&mut self) -> &mut CuOccupancyGrid<W, H> {
        &mut self.grid
    }

    /// Folds `scan`, taken with the robot at `map_from_base`, into the map.
    ///
    /// Rays are traced in the map's `xy` plane from the sensor position. A
    /// sensor outside the map leaves it unchanged and reports
    /// [`CuMapUpdateStatus::SensorOutsideMap`].
    pub fn insert_scan<const N: usize>(
        &mut self,
        map_from_base: &CuPose2,
        scan: &CuLaserScan<N>,
        report: &mut CuMapUpdateReport,
    ) -> CuResult<()> {
        *report = CuMapUpdateReport::default();
        let sensor = sensor_in_map(
            &CuPose3::from_pose2(map_from_base),
            &self.params.sensor_pose,
        );
        let origin = sensor.position;
        if self.grid.cell_of(origin.xy()).is_none() {
            report.status = CuMapUpdateStatus::SensorOutsideMap;
            return Ok(());
        }
        let max_range = self.params.max_range.map(|r| r.get::<meter>());
        let clear_range = {
            let range_max = scan.range_max.get::<meter>();
            let cap = max_range.unwrap_or(f64::INFINITY);
            let r = range_max.min(cap);
            (r.is_finite() && r > 0.0).then_some(r)
        };
        let mut local = CuPoint3::default();
        let mut world = CuPoint3::default();
        for i in 0..N {
            let angle =
                scan.angle_min.get::<radian>() + i as f64 * scan.angle_increment.get::<radian>();
            if !angle.is_finite() {
                report.skipped += 1;
                continue;
            }
            let valid = scan.beam_endpoint(i, &mut local);
            let range = scan.ranges[i].get::<meter>();
            if valid && max_range.is_none_or(|max| range <= max) {
                sensor.transform_point(&local, &mut world);
                self.grid.insert_ray(origin.xy(), world.xy())?;
                report.hits += 1;
                continue;
            }
            // No trusted return: clear along the beam if configured.
            let end = if valid {
                max_range
            } else if self.params.clear_no_return {
                clear_range
            } else {
                None
            };
            let Some(end) = end else {
                report.skipped += 1;
                continue;
            };
            let (s, c) = libm::sincos(angle);
            local = CuPoint3::new(
                Length::new::<meter>(end * c),
                Length::new::<meter>(end * s),
                Length::default(),
            );
            sensor.transform_point(&local, &mut world);
            self.grid.clear_ray(origin.xy(), world.xy())?;
            report.clears += 1;
        }
        report.status = CuMapUpdateStatus::Updated;
        Ok(())
    }
}

/// Builds a 3D log-odds voxel map from point clouds.
#[derive(Debug, Clone, PartialEq)]
pub struct CuCloudMapper3<const W: usize, const H: usize, const D: usize> {
    params: CuMappingParams,
    grid: CuVoxelGrid<W, H, D>,
}

impl<const W: usize, const H: usize, const D: usize> CuCloudMapper3<W, H, D> {
    /// Creates an all-unknown `W × H × D` map, validating `params`.
    pub fn from_params(params: CuMappingParams) -> CuResult<Self> {
        params.validate()?;
        let geometry = CuGridGeometry3::new(params.resolution, params.origin)?;
        let grid = CuVoxelGrid::from_params(geometry, params.update.clone())?;
        Ok(Self { params, grid })
    }

    /// The validated parameters.
    pub fn params(&self) -> &CuMappingParams {
        &self.params
    }

    /// The voxel map built so far.
    pub fn grid(&self) -> &CuVoxelGrid<W, H, D> {
        &self.grid
    }

    /// Mutable map access, e.g. to restore a frozen state.
    pub fn grid_mut(&mut self) -> &mut CuVoxelGrid<W, H, D> {
        &mut self.grid
    }

    /// Folds `cloud` (sensor frame), taken with the robot at `map_from_base`,
    /// into the voxel map. Points beyond `max_range` clear their ray up to
    /// `max_range` without marking a hit.
    pub fn insert_cloud<const N: usize>(
        &mut self,
        map_from_base: &CuPose3,
        cloud: &CuPointCloud3<N>,
        report: &mut CuMapUpdateReport,
    ) -> CuResult<()> {
        *report = CuMapUpdateReport::default();
        let sensor = sensor_in_map(map_from_base, &self.params.sensor_pose);
        let origin = sensor.position;
        if self.grid.cell_of(origin).is_none() {
            report.status = CuMapUpdateStatus::SensorOutsideMap;
            return Ok(());
        }
        let max_range = self.params.max_range.map(|r| r.get::<meter>());
        let mut world = CuPoint3::default();
        for point in &cloud.points[..cloud.len.min(N)] {
            sensor.transform_point(point, &mut world);
            let distance = origin.distance(&world).get::<meter>();
            if !distance.is_finite() {
                report.skipped += 1;
                continue;
            }
            match max_range {
                Some(max) if distance > max => {
                    self.grid.clear_ray(origin, along(&origin, &world, max))?;
                    report.clears += 1;
                }
                _ => {
                    self.grid.insert_ray(origin, world)?;
                    report.hits += 1;
                }
            }
        }
        report.status = CuMapUpdateStatus::Updated;
        Ok(())
    }

    /// Flattens the configured height band onto `out`, a 2D map sharing this
    /// map's resolution and `x`/`y` origin.
    pub fn project_into(&self, out: &mut CuOccupancyGrid<W, H>) -> CuResult<()> {
        let band = &self.params.projection;
        self.grid.project_into(band.z_min, band.z_max, out)
    }

    /// A 2D map with this map's footprint, for [`Self::project_into`].
    pub fn empty_projection(&self) -> CuResult<CuOccupancyGrid<W, H>> {
        CuOccupancyGrid::from_params(self.grid.geometry().xy(), self.params.update.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;
    use cu29::units::si::f64::Angle;
    use meander_core::CuPoint2;
    use meander_core::CuQuaternion;
    use meander_grid::CuCell2;
    use meander_grid::CuOccupancy;

    /// Default model: 4 misses reach Free, 2 hits reach Occupied.
    const REPEAT: usize = 5;

    fn m(v: f64) -> Length {
        Length::new::<meter>(v)
    }
    fn params() -> CuMappingParams {
        // 0.1 m cells covering [-2, 2) in x and y, and [0, 1) in z.
        CuMappingParams {
            resolution: m(0.1),
            origin: CuPoint3::new(m(-2.0), m(-2.0), m(0.0)),
            ..CuMappingParams::default()
        }
    }
    fn scan<const N: usize>(ranges: [f64; N], angle_min: f64, increment: f64) -> CuLaserScan<N> {
        CuLaserScan {
            angle_min: Angle::new::<radian>(angle_min),
            angle_increment: Angle::new::<radian>(increment),
            range_min: m(0.05),
            range_max: m(1.0),
            ranges: ranges.map(m),
        }
    }
    fn state2(mapper: &CuScanMapper2<40, 40>, x: f64, y: f64) -> Option<CuOccupancy> {
        let cell = mapper.grid().cell_of(CuPoint2::new(m(x), m(y)))?;
        mapper.grid().get(cell)
    }

    #[test]
    fn test_scan_marks_hit_and_frees_path() {
        let mut mapper = CuScanMapper2::<40, 40>::from_params(params()).unwrap();
        let mut report = CuMapUpdateReport::default();
        for _ in 0..REPEAT {
            mapper
                .insert_scan(&CuPose2::default(), &scan([0.75], 0.0, 0.0), &mut report)
                .unwrap();
        }
        assert_eq!(
            report,
            CuMapUpdateReport {
                status: CuMapUpdateStatus::Updated,
                hits: 1,
                clears: 0,
                skipped: 0
            }
        );
        assert_eq!(state2(&mapper, 0.75, 0.05), Some(CuOccupancy::Occupied));
        assert_eq!(state2(&mapper, 0.35, 0.05), Some(CuOccupancy::Free));
        assert_eq!(state2(&mapper, 0.05, 0.55), Some(CuOccupancy::Unknown));
    }

    #[test]
    fn test_scan_uses_robot_pose_and_sensor_mount() {
        // Robot at (1, 0) facing +y; sensor mounted 0.2 m ahead of the base.
        let mut p = params();
        p.sensor_pose = CuPose3::new(
            CuPoint3::new(m(0.2), m(0.0), m(0.0)),
            CuQuaternion::default(),
        );
        let mut mapper = CuScanMapper2::<40, 40>::from_params(p).unwrap();
        let pose = CuPose2::new(m(1.0), m(0.0), Angle::new::<radian>(PI / 2.0));
        let mut report = CuMapUpdateReport::default();
        for _ in 0..REPEAT {
            mapper
                .insert_scan(&pose, &scan([0.5], 0.0, 0.0), &mut report)
                .unwrap();
        }
        // Sensor at (1, 0.2); the hit lands 0.5 m further along +y.
        assert_eq!(state2(&mapper, 1.05, 0.75), Some(CuOccupancy::Occupied));
        assert_eq!(state2(&mapper, 1.05, 0.45), Some(CuOccupancy::Free));
    }

    #[test]
    fn test_no_return_and_far_returns_only_clear() {
        let mut p = params();
        p.max_range = Some(m(0.5));
        let mut mapper = CuScanMapper2::<40, 40>::from_params(p).unwrap();
        let mut report = CuMapUpdateReport::default();
        // Beam 0 (+x): no return (0 is invalid). Beam 1 (+y): valid 0.9 m, beyond max_range.
        for _ in 0..REPEAT {
            mapper
                .insert_scan(
                    &CuPose2::default(),
                    &scan([0.0, 0.9], 0.0, PI / 2.0),
                    &mut report,
                )
                .unwrap();
        }
        assert_eq!((report.hits, report.clears, report.skipped), (0, 2, 0));
        // Both clear to 0.5 m (no-return capped by max_range) and mark nothing occupied.
        assert_eq!(state2(&mapper, 0.35, 0.05), Some(CuOccupancy::Free));
        assert_eq!(state2(&mapper, 0.05, 0.35), Some(CuOccupancy::Free));
        assert_eq!(state2(&mapper, 0.85, 0.05), Some(CuOccupancy::Unknown));
        assert_eq!(state2(&mapper, 0.05, 0.85), Some(CuOccupancy::Unknown));
        assert!(
            mapper
                .grid()
                .cells()
                .iter()
                .flatten()
                .all(|c| *c != CuOccupancy::Occupied)
        );
    }

    #[test]
    fn test_no_return_skipped_when_clearing_disabled() {
        let mut p = params();
        p.clear_no_return = false;
        let mut mapper = CuScanMapper2::<40, 40>::from_params(p).unwrap();
        let mut report = CuMapUpdateReport::default();
        mapper
            .insert_scan(&CuPose2::default(), &scan([0.0], 0.0, 0.0), &mut report)
            .unwrap();
        assert_eq!((report.hits, report.clears, report.skipped), (0, 0, 1));
        assert!(
            mapper
                .grid()
                .cells()
                .iter()
                .flatten()
                .all(|c| *c == CuOccupancy::Unknown)
        );
    }

    #[test]
    fn test_sensor_outside_map_leaves_it_unchanged() {
        let mut mapper = CuScanMapper2::<40, 40>::from_params(params()).unwrap();
        let before = mapper.clone();
        let mut report = CuMapUpdateReport::default();
        let far = CuPose2::new(m(10.0), m(0.0), Angle::default());
        mapper
            .insert_scan(&far, &scan([0.5], 0.0, 0.0), &mut report)
            .unwrap();
        assert_eq!(report.status, CuMapUpdateStatus::SensorOutsideMap);
        assert_eq!(mapper, before);
    }

    #[test]
    fn test_log_odds_ignore_a_single_spurious_return() {
        let mut mapper = CuScanMapper2::<40, 40>::from_params(params()).unwrap();
        let mut report = CuMapUpdateReport::default();
        let open = scan([0.95], 0.0, 0.0);
        for _ in 0..10 {
            mapper
                .insert_scan(&CuPose2::default(), &open, &mut report)
                .unwrap();
        }
        // A person walks through once at 0.45 m.
        mapper
            .insert_scan(&CuPose2::default(), &scan([0.45], 0.0, 0.0), &mut report)
            .unwrap();
        assert_eq!(state2(&mapper, 0.45, 0.05), Some(CuOccupancy::Free));
        assert_eq!(state2(&mapper, 0.95, 0.05), Some(CuOccupancy::Occupied));
    }

    #[test]
    fn test_cloud_marks_voxels_and_projects() {
        let mut mapper = CuCloudMapper3::<40, 40, 10>::from_params(params()).unwrap();
        let mut cloud = CuPointCloud3::<2>::default();
        cloud.points[0] = CuPoint3::new(m(0.75), m(0.0), m(0.3));
        cloud.points[1] = CuPoint3::new(m(f64::NAN), m(0.0), m(0.0));
        cloud.len = 2;
        let pose = CuPose3::new(
            CuPoint3::new(m(0.0), m(0.0), m(0.3)),
            CuQuaternion::default(),
        );
        let mut report = CuMapUpdateReport::default();
        for _ in 0..REPEAT {
            mapper.insert_cloud(&pose, &cloud, &mut report).unwrap();
        }
        assert_eq!((report.hits, report.clears, report.skipped), (1, 0, 1));
        // Sensor at z=0.3 (base at 0.3, identity mount): the hit is at (0.75, 0, 0.6).
        let hit = mapper
            .grid()
            .cell_of(CuPoint3::new(m(0.75), m(0.0), m(0.6)))
            .unwrap();
        assert_eq!(mapper.grid().get(hit), Some(CuOccupancy::Occupied));
        // 40% along the slanted ray from (0, 0, 0.3) to (0.75, 0, 0.6).
        let on_ray = mapper
            .grid()
            .cell_of(CuPoint3::new(m(0.3), m(0.0), m(0.42)))
            .unwrap();
        assert_eq!(mapper.grid().get(on_ray), Some(CuOccupancy::Free));

        let mut flat = mapper.empty_projection().unwrap();
        mapper.project_into(&mut flat).unwrap();
        assert_eq!(
            flat.get(CuCell2::new(hit.x, hit.y)),
            Some(CuOccupancy::Occupied)
        );
    }

    #[test]
    fn test_cloud_beyond_max_range_only_clears() {
        let mut p = params();
        p.max_range = Some(m(0.5));
        let mut mapper = CuCloudMapper3::<40, 40, 10>::from_params(p).unwrap();
        let mut cloud = CuPointCloud3::<1>::default();
        cloud.points[0] = CuPoint3::new(m(1.5), m(0.0), m(0.0));
        cloud.len = 1;
        let pose = CuPose3::new(
            CuPoint3::new(m(0.0), m(0.0), m(0.55)),
            CuQuaternion::default(),
        );
        let mut report = CuMapUpdateReport::default();
        for _ in 0..REPEAT {
            mapper.insert_cloud(&pose, &cloud, &mut report).unwrap();
        }
        assert_eq!((report.hits, report.clears), (0, 1));
        let near = mapper
            .grid()
            .cell_of(CuPoint3::new(m(0.35), m(0.0), m(0.55)))
            .unwrap();
        let far = mapper
            .grid()
            .cell_of(CuPoint3::new(m(1.45), m(0.0), m(0.55)))
            .unwrap();
        assert_eq!(mapper.grid().get(near), Some(CuOccupancy::Free));
        assert_eq!(mapper.grid().get(far), Some(CuOccupancy::Unknown));
    }

    #[test]
    fn test_params_validation() {
        let mut p = params();
        p.max_range = Some(m(0.0));
        assert!(CuScanMapper2::<4, 4>::from_params(p).is_err());
        let mut p = params();
        p.projection.z_min = m(3.0);
        assert!(CuCloudMapper3::<4, 4, 4>::from_params(p).is_err());
        let mut p = params();
        p.update.p_hit = 0.4;
        assert!(CuScanMapper2::<4, 4>::from_params(p).is_err());
        let mut p = params();
        p.resolution = m(f64::NAN);
        assert!(CuScanMapper2::<4, 4>::from_params(p).is_err());
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_params_ron() {
        let p: CuMappingParams = ron::from_str(
            "(resolution: 0.1, origin: (x: -2.0, y: -2.0, z: 0.0), max_range: Some(5.0), save_path: Some(\"maps/lab\"))",
        )
        .unwrap();
        assert_eq!(p.max_range, Some(m(5.0)));
        assert_eq!(p.save_path.as_deref(), Some("maps/lab"));
        assert!(p.clear_no_return);
        assert!(ron::from_str::<CuMappingParams>("(resolutoin: 0.1)").is_err());
    }
}
