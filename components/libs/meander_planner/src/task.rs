//! Copper `CuTask` wrapper around the bidirectional Dijkstra planner.

use crate::params::CuPlannerParams;
use crate::params::CuPlannerTaskParams;
use crate::planner::CuBidirectionalDijkstra;
use crate::report::CuPlanStatus;
use bincode::Decode;
use bincode::Encode;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu29::prelude::*;
use meander_core::CuPath2;
use meander_core::CuPose2;
use meander_costmap::CuCostmap2;

/// Plans once from the first pose to the configured goal, then republishes
/// the cached path every cycle.
///
/// - Input: `CuPose2`, the robot base in the map frame.
/// - Output: `CuPath2<N>`, start first, stamped with the pose's `tov`.
///
/// Node config: [`CuPlannerTaskParams`]. With `std`, `CuTask::new` loads the
/// Nav2 map at `map_path` and inflates it into the costmap; without a map the
/// task is built programmatically with
/// [`from_costmap`](CuPlannerTask::from_costmap). The cached path is task
/// state: `freeze`/`thaw` round-trips it, so replay does not replan.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuPlannerTask<const W: usize, const H: usize, const N: usize> {
    #[reflect(ignore)]
    planner: CuBidirectionalDijkstra<W, H>,
    #[reflect(ignore)]
    costmap: CuCostmap2<W, H>,
    #[reflect(ignore)]
    goal: CuPose2,
    #[reflect(ignore)]
    planned: Option<CuPath2<N>>,
    #[reflect(ignore)]
    scratch: [CuPose2; N],
}

impl<const W: usize, const H: usize, const N: usize> CuPlannerTask<W, H, N> {
    /// Creates the task over an already-inflated costmap.
    pub fn from_costmap(
        costmap: CuCostmap2<W, H>,
        goal: CuPose2,
        planner: CuPlannerParams,
    ) -> CuResult<Self> {
        Ok(Self {
            planner: CuBidirectionalDijkstra::from_params(planner)?,
            costmap,
            goal,
            planned: None,
            scratch: [CuPose2::default(); N],
        })
    }

    /// Creates the task from validated task params and an inflated costmap.
    pub fn from_params(params: CuPlannerTaskParams, costmap: CuCostmap2<W, H>) -> CuResult<Self> {
        params.validate()?;
        Self::from_costmap(costmap, params.goal, params.planner)
    }

    /// The goal pose in the map frame.
    pub fn goal(&self) -> CuPose2 {
        self.goal
    }

    /// The cached path, once the first plan has succeeded.
    pub fn planned(&self) -> Option<&CuPath2<N>> {
        self.planned.as_ref()
    }
}

impl<const W: usize, const H: usize, const N: usize> Freezable for CuPlannerTask<W, H, N> {
    // The frozen size is constant (all `N` pose slots, planned or not): the
    // runtime sizes its keyframe buffer from the first freeze, before any
    // plan exists, so a growing snapshot would overflow it.
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        match &self.planned {
            None => {
                Encode::encode(&false, encoder)?;
                Encode::encode(&0usize, encoder)?;
                for pose in &[CuPose2::default(); N] {
                    Encode::encode(pose, encoder)?;
                }
                Ok(())
            }
            Some(path) => {
                Encode::encode(&true, encoder)?;
                Encode::encode(&path.len, encoder)?;
                for pose in &path.poses {
                    Encode::encode(pose, encoder)?;
                }
                Ok(())
            }
        }
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        let planned = <bool as Decode<D::Context>>::decode(decoder)?;
        let len = <usize as Decode<D::Context>>::decode(decoder)?;
        if len > N {
            return Err(DecodeError::Other(
                "meander-planner: frozen path is too long",
            ));
        }
        let mut path = CuPath2::default();
        for pose in &mut path.poses {
            *pose = <CuPose2 as Decode<D::Context>>::decode(decoder)?;
        }
        path.len = len;
        self.planned = planned.then_some(path);
        Ok(())
    }
}

impl<const W: usize, const H: usize, const N: usize> CuTask for CuPlannerTask<W, H, N> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuPose2);
    type Output<'m> = output_msg!(CuPath2<N>);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let params = cfg
            .map(ComponentConfig::deserialize_into::<CuPlannerTaskParams>)
            .transpose()?
            .unwrap_or_default();
        params.validate()?;
        #[cfg(feature = "std")]
        {
            let path = params
                .map_path
                .as_deref()
                .ok_or_else(|| CuError::from("meander-planner: task config needs a map_path"))?;
            let map = meander_grid::load_map::<W, H>(path)?;
            let mut costmap =
                CuCostmap2::<W, H>::from_params(*map.geometry(), params.costmap.clone())?;
            costmap.update_from_occupancy(&map)?;
            Self::from_params(params, costmap)
        }
        #[cfg(not(feature = "std"))]
        {
            let _ = params;
            Err(CuError::from(
                "meander-planner: CuPlannerTask::new needs std to load a map; use from_costmap",
            ))
        }
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        let Some(start) = input.payload() else {
            output.clear_payload();
            return Ok(());
        };
        if self.planned.is_none() {
            let report = self.planner.plan(
                self.costmap.costs(),
                self.costmap.geometry(),
                start,
                &self.goal,
                &mut self.scratch,
            )?;
            if report.status != CuPlanStatus::Found {
                return Err(CuError::from(format!(
                    "meander-planner: no plan to the goal: {:?}",
                    report.status
                )));
            }
            let mut path = CuPath2::default();
            path.set(&self.scratch[..report.len])?;
            self.planned = Some(path);
        }
        if let Some(path) = &self.planned {
            output.set_payload(*path);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::units::si::f64::Length;
    use cu29::units::si::length::meter;
    use meander_core::CuPoint2;
    use meander_grid::CuGridGeometry2;

    fn m(v: f64) -> Length {
        Length::new::<meter>(v)
    }

    fn task() -> CuPlannerTask<8, 8, 64> {
        let geometry = CuGridGeometry2::new(m(1.0), CuPoint2::default()).unwrap();
        let costmap =
            CuCostmap2::<8, 8>::from_params(geometry, meander_costmap::CuCostmapParams::default())
                .unwrap();
        // An untouched costmap is all NO_INFORMATION, which the default
        // planner treats as blocked; rebuild it as free space.
        let mut costmap = costmap;
        let mut grid =
            meander_grid::CuOccupancyGrid::<8, 8>::from_params(geometry, Default::default())
                .unwrap();
        grid.fill(meander_grid::CuOccupancy::Free);
        costmap.update_from_occupancy(&grid).unwrap();
        CuPlannerTask::from_costmap(
            costmap,
            CuPose2::new(m(7.5), m(7.5), Default::default()),
            CuPlannerParams::default(),
        )
        .unwrap()
    }

    fn run(task: &mut CuPlannerTask<8, 8, 64>) -> CuMsg<CuPath2<64>> {
        let (ctx, _clock) = CuContext::new_mock_clock();
        let mut pose = CuMsg::new(Some(CuPose2::new(m(0.5), m(0.5), Default::default())));
        pose.tov = Tov::Time(CuTime::from_nanos(3_000_000));
        let mut out = CuMsg::<CuPath2<64>>::new(None);
        task.process(&ctx, &pose, &mut out).unwrap();
        out
    }

    #[test]
    fn test_task_plans_once_and_republishes() {
        let mut task = task();
        let out = run(&mut task);
        assert_eq!(out.tov, Tov::Time(CuTime::from_nanos(3_000_000)));
        let path = out.payload().unwrap();
        assert_eq!(path.poses().first().unwrap().x, m(0.5));
        assert_eq!(path.poses().last().unwrap().x, m(7.5));
        // A later, moved pose still gets the path planned from the first pose.
        let again = run(&mut task);
        assert_eq!(again.payload().unwrap().poses(), path.poses());
    }

    #[test]
    fn test_task_freeze_thaw_restores_cached_path() {
        let mut task = task();
        let out = run(&mut task);
        struct Frozen<'a, T: Freezable>(&'a T);
        impl<T: Freezable> Encode for Frozen<'_, T> {
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                self.0.freeze(encoder)
            }
        }
        let bytes = bincode::encode_to_vec(Frozen(&task), bincode::config::standard()).unwrap();
        let mut replayed = self::task();
        let reader = bincode::de::read::SliceReader::new(&bytes);
        let mut decoder = bincode::de::DecoderImpl::new(reader, bincode::config::standard(), ());
        replayed.thaw(&mut decoder).unwrap();
        assert_eq!(
            replayed.planned().unwrap().poses(),
            out.payload().unwrap().poses()
        );
    }
}
