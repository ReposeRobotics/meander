//! Copper `CuTask` wrapper around the pure pursuit controller.

use crate::params::CuPurePursuitParams;
use crate::pure_pursuit::CuPurePursuit;
use bincode::Decode;
use bincode::Encode;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu29::prelude::*;
use meander_core::CuPath2;
use meander_core::CuPose2;
use meander_core::CuTwist2;

/// Follows a planned path with regulated pure pursuit.
///
/// - Input 0: `CuPose2`, the robot base in the map frame.
/// - Input 1: `CuPath2<N>`, the path to follow, start first.
/// - Output: `CuTwist2`, the body-frame velocity command, stamped with the
///   pose's `tov`. Zero twist once the goal pose is reached.
///
/// Node config: [`CuPurePursuitParams`]. The path progress index is task
/// state: `freeze`/`thaw` round-trips it, so replay resumes mid-path.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuPurePursuitTask<const N: usize> {
    #[reflect(ignore)]
    controller: CuPurePursuit,
}

impl<const N: usize> CuPurePursuitTask<N> {
    /// Creates the task from validated params.
    pub fn from_params(params: CuPurePursuitParams) -> CuResult<Self> {
        Ok(Self {
            controller: CuPurePursuit::from_params(params)?,
        })
    }

    /// The wrapped controller.
    pub fn controller(&self) -> &CuPurePursuit {
        &self.controller
    }
}

impl<const N: usize> Freezable for CuPurePursuitTask<N> {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.controller.progress_index(), encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        let index = <usize as Decode<D::Context>>::decode(decoder)?;
        self.controller.restore_progress_index(index);
        Ok(())
    }
}

impl<const N: usize> CuTask for CuPurePursuitTask<N> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuPose2, CuPath2<N>);
    type Output<'m> = output_msg!(CuTwist2);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let params = cfg
            .map(ComponentConfig::deserialize_into::<CuPurePursuitParams>)
            .transpose()?
            .unwrap_or_default();
        params.validate()?;
        Self::from_params(params)
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (pose_msg, path_msg) = *input;
        output.tov = pose_msg.tov;
        let (Some(pose), Some(path)) = (pose_msg.payload(), path_msg.payload()) else {
            output.clear_payload();
            return Ok(());
        };
        let mut twist = CuTwist2::default();
        self.controller.compute(pose, path.poses(), &mut twist)?;
        output.set_payload(twist);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pure_pursuit::CuFollowStatus;
    use cu29::units::si::angle::radian;
    use cu29::units::si::f64::Angle;
    use cu29::units::si::f64::Length;
    use cu29::units::si::length::meter;
    use cu29::units::si::velocity::meter_per_second;

    fn m(v: f64) -> Length {
        Length::new::<meter>(v)
    }

    fn path<const N: usize>() -> CuPath2<N> {
        let mut path = CuPath2::default();
        path.set(&[
            CuPose2::new(m(0.0), m(0.0), Angle::default()),
            CuPose2::new(m(2.0), m(0.0), Angle::default()),
        ])
        .unwrap();
        path
    }

    #[test]
    fn test_task_drives_towards_path_and_propagates_tov() {
        let mut task = CuPurePursuitTask::<8>::from_params(CuPurePursuitParams::default()).unwrap();
        let (ctx, _clock) = CuContext::new_mock_clock();
        let mut pose = CuMsg::new(Some(CuPose2::default()));
        pose.tov = Tov::Time(CuTime::from_nanos(7_000_000));
        let path_msg = CuMsg::new(Some(path::<8>()));
        let mut out = CuMsg::<CuTwist2>::new(None);
        task.process(&ctx, &(&pose, &path_msg), &mut out).unwrap();
        assert_eq!(out.tov, Tov::Time(CuTime::from_nanos(7_000_000)));
        assert!(out.payload().unwrap().linear_x.get::<meter_per_second>() > 0.0);

        // Missing inputs publish nothing.
        let empty = CuMsg::<CuPose2>::new(None);
        task.process(&ctx, &(&empty, &path_msg), &mut out).unwrap();
        assert!(out.payload().is_none());
    }

    #[test]
    fn test_task_freeze_thaw_restores_progress() {
        let mut task = CuPurePursuitTask::<8>::from_params(CuPurePursuitParams::default()).unwrap();
        let (ctx, _clock) = CuContext::new_mock_clock();
        let pose = CuMsg::new(Some(CuPose2::new(m(1.1), m(0.0), Angle::default())));
        let path_msg = CuMsg::new(Some(path::<8>()));
        let mut out = CuMsg::<CuTwist2>::new(None);
        task.process(&ctx, &(&pose, &path_msg), &mut out).unwrap();
        assert!(task.controller().progress_index() > 0);

        struct Frozen<'a, T: Freezable>(&'a T);
        impl<T: Freezable> Encode for Frozen<'_, T> {
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                self.0.freeze(encoder)
            }
        }
        let bytes = bincode::encode_to_vec(Frozen(&task), bincode::config::standard()).unwrap();
        let mut replayed =
            CuPurePursuitTask::<8>::from_params(CuPurePursuitParams::default()).unwrap();
        let reader = bincode::de::read::SliceReader::new(&bytes);
        let mut decoder = bincode::de::DecoderImpl::new(reader, bincode::config::standard(), ());
        replayed.thaw(&mut decoder).unwrap();
        assert_eq!(
            replayed.controller().progress_index(),
            task.controller().progress_index()
        );
        let _ = CuFollowStatus::Following;
        let _ = Angle::new::<radian>(0.0);
    }

    #[test]
    fn test_task_reads_node_config() {
        let mut cfg = ComponentConfig::new();
        cfg.set("lookahead", 0.3f64);
        let task = CuPurePursuitTask::<8>::new(Some(&cfg), ()).unwrap();
        assert_eq!(task.controller().params().lookahead, m(0.3));
        let mut typo = ComponentConfig::new();
        typo.set("lookahed", 0.3f64);
        assert!(CuPurePursuitTask::<8>::new(Some(&typo), ()).is_err());
    }
}
