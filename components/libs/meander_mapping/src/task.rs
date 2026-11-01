//! Copper `CuTask` wrappers around the mappers.

use crate::mapper::CuCloudMapper3;
use crate::mapper::CuMapUpdateReport;
use crate::mapper::CuScanMapper2;
use crate::params::CuMappingParams;
use bincode::Decode;
use bincode::Encode;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu29::prelude::*;
use meander_core::CuLaserScan;
use meander_core::CuPointCloud3;
use meander_core::CuPose2;
use meander_core::CuPose3;

fn read_params(cfg: Option<&ComponentConfig>) -> CuResult<CuMappingParams> {
    let params = cfg
        .map(ComponentConfig::deserialize_into::<CuMappingParams>)
        .transpose()?
        .unwrap_or_default();
    params.validate()?;
    Ok(params)
}

fn invalid_state(_: CuError) -> DecodeError {
    DecodeError::Other("meander-mapping: frozen map state is invalid")
}

/// Builds a 2D log-odds occupancy map from robot poses and planar laser scans.
///
/// - Input 0: `CuPose2`, the robot base in the map frame (`map ← base`), e.g.
///   from SLAM or localization, time-aligned with the scan.
/// - Input 1: `CuLaserScan<N>` in the sensor frame given by
///   [`CuMappingParams::sensor_pose`].
/// - Output: one [`CuMapUpdateReport`] per scan, stamped with the scan's `tov`.
///
/// Node config: [`CuMappingParams`]. With `save_path` set (and `std`), the
/// map is written as a Nav2 PGM + YAML pair on `stop`.
///
/// The map is task state: `freeze`/`thaw` round-trip every cell's log-odds,
/// so replay resumes the exact map. The whole grid lives in the task, so pick
/// `W × H` with the stack and keyframe size in mind.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuScanMappingTask<const W: usize, const H: usize, const N: usize> {
    #[reflect(ignore)]
    mapper: CuScanMapper2<W, H>,
}

impl<const W: usize, const H: usize, const N: usize> CuScanMappingTask<W, H, N> {
    /// Creates the task from validated params.
    pub fn from_params(params: CuMappingParams) -> CuResult<Self> {
        Ok(Self {
            mapper: CuScanMapper2::from_params(params)?,
        })
    }

    /// The mapper and the map built so far.
    pub fn mapper(&self) -> &CuScanMapper2<W, H> {
        &self.mapper
    }
}

impl<const W: usize, const H: usize, const N: usize> Freezable for CuScanMappingTask<W, H, N> {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        for row in self.mapper.grid().log_odds_rows() {
            for value in row {
                Encode::encode(value, encoder)?;
            }
        }
        Ok(())
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        let mut rows = [[0.0f32; W]; H];
        for row in &mut rows {
            for value in row {
                *value = <f32 as Decode<D::Context>>::decode(decoder)?;
            }
        }
        self.mapper
            .grid_mut()
            .load_log_odds_rows(&rows)
            .map_err(invalid_state)
    }
}

impl<const W: usize, const H: usize, const N: usize> CuTask for CuScanMappingTask<W, H, N> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuPose2, CuLaserScan<N>);
    type Output<'m> = output_msg!(CuMapUpdateReport);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Self::from_params(read_params(cfg)?)
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (pose_msg, scan_msg) = *input;
        output.tov = scan_msg.tov;
        let mut report = CuMapUpdateReport::default();
        if let (Some(pose), Some(scan)) = (pose_msg.payload(), scan_msg.payload()) {
            self.mapper.insert_scan(pose, scan, &mut report)?;
        }
        output.set_payload(report);
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        #[cfg(feature = "std")]
        if let Some(path) = &self.mapper.params().save_path {
            meander_grid::save_map(self.mapper.grid(), path)?;
        }
        Ok(())
    }
}

/// Builds a 3D log-odds voxel map from robot poses and point clouds.
///
/// - Input 0: `CuPose3`, the robot base in the map frame (`map ← base`).
/// - Input 1: `CuPointCloud3<N>` in the sensor frame given by
///   [`CuMappingParams::sensor_pose`].
/// - Output: one [`CuMapUpdateReport`] per cloud, stamped with the cloud's `tov`.
///
/// Node config: [`CuMappingParams`]. With `save_path` set (and `std`), the
/// [`CuMappingParams::projection`] height band is flattened into a 2D map and
/// written as a Nav2 PGM + YAML pair on `stop`.
///
/// `freeze`/`thaw` round-trip every voxel's log-odds.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuCloudMappingTask<const W: usize, const H: usize, const D: usize, const N: usize> {
    #[reflect(ignore)]
    mapper: CuCloudMapper3<W, H, D>,
}

impl<const W: usize, const H: usize, const D: usize, const N: usize>
    CuCloudMappingTask<W, H, D, N>
{
    /// Creates the task from validated params.
    pub fn from_params(params: CuMappingParams) -> CuResult<Self> {
        Ok(Self {
            mapper: CuCloudMapper3::from_params(params)?,
        })
    }

    /// The mapper and the voxel map built so far.
    pub fn mapper(&self) -> &CuCloudMapper3<W, H, D> {
        &self.mapper
    }
}

impl<const W: usize, const H: usize, const D: usize, const N: usize> Freezable
    for CuCloudMappingTask<W, H, D, N>
{
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        for value in self
            .mapper
            .grid()
            .log_odds_rows()
            .iter()
            .flatten()
            .flatten()
        {
            Encode::encode(value, encoder)?;
        }
        Ok(())
    }

    fn thaw<Dec: Decoder>(&mut self, decoder: &mut Dec) -> Result<(), DecodeError> {
        let mut rows = [[[0.0f32; W]; H]; D];
        for value in rows.iter_mut().flatten().flatten() {
            *value = <f32 as Decode<Dec::Context>>::decode(decoder)?;
        }
        self.mapper
            .grid_mut()
            .load_log_odds_rows(&rows)
            .map_err(invalid_state)
    }
}

impl<const W: usize, const H: usize, const D: usize, const N: usize> CuTask
    for CuCloudMappingTask<W, H, D, N>
{
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuPose3, CuPointCloud3<N>);
    type Output<'m> = output_msg!(CuMapUpdateReport);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Self::from_params(read_params(cfg)?)
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (pose_msg, cloud_msg) = *input;
        output.tov = cloud_msg.tov;
        let mut report = CuMapUpdateReport::default();
        if let (Some(pose), Some(cloud)) = (pose_msg.payload(), cloud_msg.payload()) {
            self.mapper.insert_cloud(pose, cloud, &mut report)?;
        }
        output.set_payload(report);
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        #[cfg(feature = "std")]
        if let Some(path) = &self.mapper.params().save_path {
            let mut flat = self.mapper.empty_projection()?;
            self.mapper.project_into(&mut flat)?;
            meander_grid::save_map(&flat, path)?;
        }
        Ok(())
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use crate::mapper::CuMapUpdateStatus;
    use cu29::units::si::f64::Length;
    use cu29::units::si::length::meter;
    use meander_core::CuPoint3;
    use meander_core::CuQuaternion;
    use meander_grid::CuOccupancy;

    fn m(v: f64) -> Length {
        Length::new::<meter>(v)
    }
    fn params() -> CuMappingParams {
        CuMappingParams {
            resolution: m(0.1),
            origin: CuPoint3::new(m(-2.0), m(-2.0), m(0.0)),
            ..CuMappingParams::default()
        }
    }
    fn scan() -> CuLaserScan<1> {
        CuLaserScan {
            range_min: m(0.05),
            range_max: m(5.0),
            ranges: [m(0.95)],
            ..CuLaserScan::default()
        }
    }

    /// Freezes `task` to bytes through its `Freezable` impl.
    struct Frozen<'a, T: Freezable>(&'a T);
    impl<T: Freezable> Encode for Frozen<'_, T> {
        fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
            self.0.freeze(encoder)
        }
    }
    fn freeze<T: Freezable>(task: &T) -> Vec<u8> {
        bincode::encode_to_vec(Frozen(task), bincode::config::standard()).unwrap()
    }
    fn thaw<T: Freezable>(task: &mut T, bytes: &[u8]) -> Result<(), DecodeError> {
        let reader = bincode::de::read::SliceReader::new(bytes);
        let mut decoder = bincode::de::DecoderImpl::new(reader, bincode::config::standard(), ());
        task.thaw(&mut decoder)
    }

    fn run_scans(task: &mut CuScanMappingTask<40, 40, 1>, n: usize) -> CuMsg<CuMapUpdateReport> {
        let (ctx, _clock) = CuContext::new_mock_clock();
        let pose = CuMsg::new(Some(CuPose2::default()));
        let mut scan = CuMsg::new(Some(scan()));
        scan.tov = Tov::Time(CuTime::from_nanos(42_000_000));
        let mut out = CuMsg::<CuMapUpdateReport>::new(None);
        for _ in 0..n {
            task.process(&ctx, &(&pose, &scan), &mut out).unwrap();
        }
        out
    }

    #[test]
    fn test_scan_task_reports_and_propagates_tov() {
        let mut task = CuScanMappingTask::<40, 40, 1>::from_params(params()).unwrap();
        let out = run_scans(&mut task, 5);
        assert_eq!(out.tov, Tov::Time(CuTime::from_nanos(42_000_000)));
        let report = out.payload().unwrap();
        assert_eq!(
            (report.status, report.hits),
            (CuMapUpdateStatus::Updated, 1)
        );

        // A missing pose maps nothing but still reports.
        let (ctx, _clock) = CuContext::new_mock_clock();
        let pose = CuMsg::<CuPose2>::new(None);
        let scan = CuMsg::new(Some(scan()));
        let mut out = CuMsg::<CuMapUpdateReport>::new(None);
        task.process(&ctx, &(&pose, &scan), &mut out).unwrap();
        assert_eq!(out.payload().unwrap().status, CuMapUpdateStatus::NoInput);
    }

    #[test]
    fn test_scan_task_freeze_thaw_restores_the_map() {
        let mut task = CuScanMappingTask::<40, 40, 1>::from_params(params()).unwrap();
        run_scans(&mut task, 5);
        let bytes = freeze(&task);

        let mut replayed = CuScanMappingTask::<40, 40, 1>::from_params(params()).unwrap();
        thaw(&mut replayed, &bytes).unwrap();
        assert_eq!(replayed.mapper(), task.mapper());

        // Same input after the keyframe gives the same map.
        run_scans(&mut task, 1);
        run_scans(&mut replayed, 1);
        assert_eq!(replayed.mapper(), task.mapper());

        // Truncated or corrupt state is rejected, not silently loaded.
        assert!(thaw(&mut replayed, &bytes[..bytes.len() / 2]).is_err());
        let corrupt =
            bincode::encode_to_vec([f32::NAN; 40 * 40], bincode::config::standard()).unwrap();
        assert!(thaw(&mut replayed, &corrupt).is_err());
    }

    #[test]
    fn test_scan_task_saves_a_nav2_map_on_stop() {
        let dir = tempfile::tempdir().unwrap();
        let stem = dir.path().join("lab");
        let mut p = params();
        p.save_path = Some(stem.to_str().unwrap().into());
        let mut task = CuScanMappingTask::<40, 40, 1>::from_params(p).unwrap();
        run_scans(&mut task, 5);
        let (ctx, _clock) = CuContext::new_mock_clock();
        task.stop(&ctx).unwrap();

        let loaded = meander_grid::load_map::<40, 40>(dir.path().join("lab.yaml")).unwrap();
        let wall = loaded
            .cell_of(meander_core::CuPoint2::new(m(0.95), m(0.05)))
            .unwrap();
        assert_eq!(loaded.get(wall), Some(CuOccupancy::Occupied));
        assert_eq!(loaded.cells(), task.mapper().grid().cells());
    }

    #[test]
    fn test_cloud_task_maps_freezes_and_saves_projection() {
        let dir = tempfile::tempdir().unwrap();
        let mut p = params();
        p.save_path = Some(dir.path().join("cloud").to_str().unwrap().into());
        let mut task = CuCloudMappingTask::<40, 40, 10, 1>::from_params(p.clone()).unwrap();

        let (ctx, _clock) = CuContext::new_mock_clock();
        let pose = CuMsg::new(Some(CuPose3::new(
            CuPoint3::new(m(0.0), m(0.0), m(0.35)),
            CuQuaternion::default(),
        )));
        let mut cloud = CuPointCloud3::<1>::default();
        cloud.points[0] = CuPoint3::new(m(0.95), m(0.0), m(0.0));
        cloud.len = 1;
        let cloud = CuMsg::new(Some(cloud));
        let mut out = CuMsg::<CuMapUpdateReport>::new(None);
        for _ in 0..5 {
            task.process(&ctx, &(&pose, &cloud), &mut out).unwrap();
        }
        assert_eq!(out.payload().unwrap().hits, 1);

        let mut replayed = CuCloudMappingTask::<40, 40, 10, 1>::from_params(p).unwrap();
        thaw(&mut replayed, &freeze(&task)).unwrap();
        assert_eq!(replayed.mapper(), task.mapper());

        task.stop(&ctx).unwrap();
        let flat = meander_grid::load_map::<40, 40>(dir.path().join("cloud.yaml")).unwrap();
        let wall = flat
            .cell_of(meander_core::CuPoint2::new(m(0.95), m(0.05)))
            .unwrap();
        assert_eq!(flat.get(wall), Some(CuOccupancy::Occupied));
    }

    #[test]
    fn test_task_reads_node_config() {
        let mut cfg = ComponentConfig::new();
        cfg.set("resolution", 0.1f64);
        cfg.set("max_range", 4.0f64);
        let task = CuScanMappingTask::<8, 8, 4>::new(Some(&cfg), ()).unwrap();
        assert_eq!(task.mapper().params().resolution, m(0.1));
        assert_eq!(task.mapper().params().max_range, Some(m(4.0)));

        let mut typo = ComponentConfig::new();
        typo.set("resolutoin", 0.1f64);
        assert!(CuScanMappingTask::<8, 8, 4>::new(Some(&typo), ()).is_err());
        let mut invalid = ComponentConfig::new();
        invalid.set("resolution", -1.0f64);
        assert!(CuScanMappingTask::<8, 8, 4>::new(Some(&invalid), ()).is_err());
    }
}
