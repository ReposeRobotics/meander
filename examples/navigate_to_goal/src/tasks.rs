//! App-local sink: records the driven path and writes the picture on stop.

use cu29::prelude::*;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use meander_core::CuPath2;
use meander_core::CuPose2;
use meander_core::CuTwist2;
use meander_costmap::CuCostmap2;
use meander_costmap::CuCostmapParams;
use meander_costmap::INSCRIBED;
use meander_costmap::LETHAL;
use meander_costmap::NO_INFORMATION;
use serde::Deserialize;
use serde::Serialize;

pub const W: usize = 200;
pub const H: usize = 200;
pub const MAX_PATH: usize = 2048;

/// Recorder configuration, identical in Rust and RON.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuNavigationRecorderParams {
    /// Nav2 map YAML whose inflated costmap backs the picture.
    pub map_path: String,
    /// PNG written on `stop`.
    pub output_path: String,
}

impl Default for CuNavigationRecorderParams {
    fn default() -> Self {
        Self {
            map_path: "target/waypoint-mapping/map.yaml".into(),
            output_path: "target/navigate-to-goal/navigation.png".into(),
        }
    }
}

/// Records every pose, keeps the first planned path, and on `stop` writes
/// `navigation.png`: costmap, planned path (green), driven path (red),
/// start (blue) and goal (orange).
///
/// - Input 0: `CuPose2` from the simulated base.
/// - Input 1: `CuPath2<MAX_PATH>` from the planner.
/// - Input 2: `CuTwist2` from the controller (logged; not drawn).
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuNavigationRecorder {
    #[reflect(ignore)]
    params: CuNavigationRecorderParams,
    #[reflect(ignore)]
    costs: [[u8; W]; H],
    #[reflect(ignore)]
    origin: (f64, f64),
    #[reflect(ignore)]
    resolution: f64,
    #[reflect(ignore)]
    planned: Vec<(f64, f64)>,
    #[reflect(ignore)]
    driven: Vec<(f64, f64)>,
}

impl Freezable for CuNavigationRecorder {}

impl CuSinkTask for CuNavigationRecorder {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuPose2, CuPath2<MAX_PATH>, CuTwist2);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let params = cfg
            .map(ComponentConfig::deserialize_into::<CuNavigationRecorderParams>)
            .transpose()?
            .unwrap_or_default();
        if params.map_path.is_empty() || params.output_path.is_empty() {
            return Err(CuError::from(
                "navigate recorder: map_path and output_path must not be empty",
            ));
        }
        let map = meander_grid::load_map::<W, H>(&params.map_path).inspect_err(|_| {
            eprintln!(
                "no map at {}: run `cargo run --release -p meander-example-waypoint-mapping` first",
                params.map_path
            );
        })?;
        let mut costmap =
            CuCostmap2::<W, H>::from_params(*map.geometry(), CuCostmapParams::default())?;
        costmap.update_from_occupancy(&map)?;
        let origin = map.geometry().origin;
        Ok(Self {
            params,
            costs: *costmap.costs(),
            origin: (origin.x.get::<meter>(), origin.y.get::<meter>()),
            resolution: map.geometry().resolution.get::<meter>(),
            planned: Vec::new(),
            driven: Vec::new(),
        })
    }

    fn process<'i>(&mut self, _ctx: &CuContext, input: &Self::Input<'i>) -> CuResult<()> {
        let (pose_msg, path_msg, _twist_msg) = *input;
        if self.planned.is_empty()
            && let Some(path) = path_msg.payload()
            && !path.is_empty()
        {
            self.planned = path
                .poses()
                .iter()
                .map(|p| (p.x.get::<meter>(), p.y.get::<meter>()))
                .collect();
        }
        if let Some(pose) = pose_msg.payload() {
            let point = (pose.x.get::<meter>(), pose.y.get::<meter>());
            if self.driven.last() != Some(&point) {
                self.driven.push(point);
            }
        }
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        let (Some(&start), Some(&goal)) = (self.planned.first(), self.planned.last()) else {
            return Err(CuError::from("navigate recorder: no path was planned"));
        };
        let Some(&end) = self.driven.last() else {
            return Err(CuError::from("navigate recorder: no pose was recorded"));
        };
        let miss = Length::new::<meter>((end.0 - goal.0).hypot(end.1 - goal.1));
        if let Some(dir) = std::path::Path::new(&self.params.output_path).parent() {
            std::fs::create_dir_all(dir)
                .map_err(|e| CuError::new_with_cause("cannot create output dir", e))?;
        }
        write_png(
            &self.costs,
            self.origin,
            self.resolution,
            &self.planned,
            &self.driven,
            start,
            goal,
            &self.params.output_path,
        )?;
        println!(
            "planned {} waypoints; robot ended {:.2} m from the goal point",
            self.planned.len(),
            miss.get::<meter>()
        );
        println!("saved {}", self.params.output_path);
        Ok(())
    }
}

/// Writes the costmap as an RGB PNG (3 px per cell) with paths and markers.
#[allow(clippy::too_many_arguments)]
fn write_png(
    costs: &[[u8; W]; H],
    origin: (f64, f64),
    resolution: f64,
    planned: &[(f64, f64)],
    driven: &[(f64, f64)],
    start: (f64, f64),
    goal: (f64, f64),
    file: &str,
) -> CuResult<()> {
    const SCALE: usize = 3;
    let (w, h) = (W * SCALE, H * SCALE);
    let mut rgb = vec![0u8; w * h * 3];
    let mut paint = |px: usize, py: usize, c: [u8; 3]| {
        if px < w && py < h {
            rgb[(py * w + px) * 3..][..3].copy_from_slice(&c);
        }
    };
    for (cy, row) in costs.iter().enumerate() {
        for (cx, &cost) in row.iter().enumerate() {
            let color = match cost {
                LETHAL => [0, 0, 0],
                INSCRIBED => [110, 60, 140],
                NO_INFORMATION => [205, 205, 205],
                c => {
                    let t = c as f64 / INSCRIBED as f64;
                    [
                        (254.0 - 90.0 * t) as u8,
                        (254.0 - 170.0 * t) as u8,
                        (254.0 - 60.0 * t) as u8,
                    ]
                }
            };
            for d in 0..SCALE * SCALE {
                paint(
                    cx * SCALE + d % SCALE,
                    (H - 1 - cy) * SCALE + d / SCALE,
                    color,
                );
            }
        }
    }
    let to_px = |(x, y): (f64, f64)| {
        let px = (x - origin.0) / resolution * SCALE as f64;
        let py = h as f64 - (y - origin.1) / resolution * SCALE as f64;
        (px as usize, py as usize)
    };
    let mut line = |points: &[(f64, f64)], color: [u8; 3], thick: usize| {
        for pair in points.windows(2) {
            let ((x0, y0), (x1, y1)) = (to_px(pair[0]), to_px(pair[1]));
            let steps = x0.abs_diff(x1).max(y0.abs_diff(y1)).max(1);
            for i in 0..=steps {
                let t = i as f64 / steps as f64;
                let x = (x0 as f64 + (x1 as f64 - x0 as f64) * t) as usize;
                let y = (y0 as f64 + (y1 as f64 - y0 as f64) * t) as usize;
                for d in 0..thick * thick {
                    paint(x + d % thick, y + d / thick, color);
                }
            }
        }
    };
    line(planned, [30, 170, 60], 3);
    line(driven, [220, 40, 40], 1);
    for (point, color) in [(start, [40, 90, 230]), (goal, [245, 140, 20])] {
        let (px, py) = to_px(point);
        for d in 0..81 {
            paint(
                (px + d % 9).saturating_sub(4),
                (py + d / 9).saturating_sub(4),
                color,
            );
        }
    }

    let out =
        std::fs::File::create(file).map_err(|e| CuError::new_with_cause("cannot create PNG", e))?;
    let mut encoder = png::Encoder::new(std::io::BufWriter::new(out), w as u32, h as u32);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    encoder
        .write_header()
        .and_then(|mut writer| writer.write_image_data(&rgb))
        .map_err(|e| CuError::new_with_cause("cannot write PNG", e))
}
