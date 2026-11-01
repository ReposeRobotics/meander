//! Drive to a goal on a saved map, like Nav2's "2D Goal Pose":
//! load map → inflate it into a costmap → plan with bidirectional Dijkstra →
//! follow the path with pure pursuit → save a picture.
//!
//! First build a map with the waypoint example, then navigate on it:
//!
//! ```sh
//! cargo run --release -p meander-example-waypoint-mapping
//! cargo run --release -p meander-example-navigate-to-goal            # default goal (2.8, 2.8)
//! cargo run --release -p meander-example-navigate-to-goal -- 0.5 -3.2 # goal x y in metres
//! ```
//!
//! Writes `target/navigate-to-goal/navigation.png`: the costmap, the planned
//! path (green), the driven path (red), start (blue) and goal (orange).

use cu29::prelude::*;
use cu29::units::si::f64::{Angle, Length, Time};
use cu29::units::si::length::meter;
use cu29::units::si::time::second;
use meander_controller::{CuFollowStatus, CuPurePursuit, CuPurePursuitParams};
use meander_core::{CuPath2, CuPose2, CuTwist2};
use meander_costmap::{CuCostmap2, CuCostmapParams, INSCRIBED, LETHAL, NO_INFORMATION};
use meander_example_diff_drive::{CuDiffDrive, CuDiffDriveParams, CuWheelSpeeds2};
use meander_planner::{CuBidirectionalDijkstra, CuPlanStatus, CuPlannerParams};

/// Must match the map saved by the waypoint example (200 × 200 cells of 5 cm).
const W: usize = 200;
const H: usize = 200;
const MAP: &str = "target/waypoint-mapping/map.yaml";
const OUT: &str = "target/navigate-to-goal/navigation.png";

const START: (f64, f64) = (-3.2, -3.2);
const DEFAULT_GOAL: (f64, f64) = (2.8, 2.8);
const DT: f64 = 0.02; // control period, s
const MAX_PATH: usize = 2048;

fn m(v: f64) -> Length {
    Length::new::<meter>(v)
}

fn main() -> CuResult<()> {
    // The planner and costmap keep whole-grid scratch arrays (a few MB):
    // give them a roomy stack instead of boxing everything.
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(run)
        .map_err(|e| CuError::new_with_cause("cannot start navigation thread", e))?
        .join()
        .map_err(|_| CuError::from("navigation thread panicked"))?
}

fn run() -> CuResult<()> {
    let args: Vec<f64> = std::env::args()
        .skip(1)
        .filter_map(|a| a.parse().ok())
        .collect();
    let goal_xy = match args[..] {
        [x, y] => (x, y),
        _ => DEFAULT_GOAL,
    };

    // 1. Load the map (Nav2 PGM + YAML) and inflate it by the robot's size.
    let map = meander_grid::load_map::<W, H>(MAP).inspect_err(|_| {
        eprintln!(
            "no map at {MAP}: run `cargo run --release -p meander-example-waypoint-mapping` first"
        );
    })?;
    let mut costmap = CuCostmap2::<W, H>::from_params(*map.geometry(), CuCostmapParams::default())?;
    costmap.update_from_occupancy(&map)?;

    // 2. Plan from start to goal with bidirectional Dijkstra.
    let start = CuPose2::new(m(START.0), m(START.1), Angle::default());
    let goal = CuPose2::new(m(goal_xy.0), m(goal_xy.1), Angle::default());
    let mut planner = CuBidirectionalDijkstra::<W, H>::from_params(CuPlannerParams::default())?;
    let mut waypoints = [CuPose2::default(); MAX_PATH];
    let report = planner.plan(
        costmap.costs(),
        costmap.geometry(),
        &start,
        &goal,
        &mut waypoints,
    )?;
    if report.status != CuPlanStatus::Found {
        return Err(CuError::from(format!(
            "no plan to ({:.1}, {:.1}): {:?}",
            goal_xy.0, goal_xy.1, report.status
        )));
    }
    let mut path = CuPath2::<MAX_PATH>::default(); // the planner → controller message
    path.set(&waypoints[..report.len])?;
    println!(
        "planned {:.2} m to ({:.1}, {:.1}) through {} waypoints",
        path.length().get::<meter>(),
        goal_xy.0,
        goal_xy.1,
        report.len
    );

    // 3. Follow the path with pure pursuit, driving the diff-drive model.
    let robot = CuDiffDrive::from_params(CuDiffDriveParams::default())?;
    let mut controller = CuPurePursuit::from_params(CuPurePursuitParams::default())?;
    let (mut pose, mut twist, mut wheels) = (start, CuTwist2::default(), CuWheelSpeeds2::default());
    let mut driven = vec![(START.0, START.1)];
    let mut status = CuFollowStatus::Following;
    let mut seconds = 0.0;
    while status != CuFollowStatus::GoalReached {
        if seconds > 300.0 {
            return Err(CuError::from("timed out before reaching the goal"));
        }
        status = controller.compute(&pose, path.poses(), &mut twist)?;
        robot.wheel_speeds(&twist, &mut wheels);
        robot.step(&mut pose, &wheels, Time::new::<second>(DT));
        driven.push((pose.x.get::<meter>(), pose.y.get::<meter>()));
        seconds += DT;
    }
    let miss = pose.position().distance(&goal.position()).get::<meter>();
    println!(
        "reached the goal in {seconds:.1} s, {:.2} m from the goal point",
        miss
    );

    // 4. Picture.
    if let Some(dir) = std::path::Path::new(OUT).parent() {
        std::fs::create_dir_all(dir)
            .map_err(|e| CuError::new_with_cause("cannot create output dir", e))?;
    }
    let planned: Vec<(f64, f64)> = path
        .poses()
        .iter()
        .map(|p| (p.x.get::<meter>(), p.y.get::<meter>()))
        .collect();
    let origin = map.geometry().origin;
    let picture = Picture {
        costs: costmap.costs(),
        origin: (origin.x.get::<meter>(), origin.y.get::<meter>()),
        resolution: map.geometry().resolution.get::<meter>(),
    };
    picture.write(&planned, &driven, START, goal_xy, OUT)?;
    println!("saved {OUT}");
    Ok(())
}

struct Picture<'a> {
    costs: &'a [[u8; W]; H],
    origin: (f64, f64),
    resolution: f64,
}

impl Picture<'_> {
    /// Writes the costmap as an RGB PNG (3 px per cell) with paths and markers.
    fn write(
        &self,
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
        for (cy, row) in self.costs.iter().enumerate() {
            for (cx, &cost) in row.iter().enumerate() {
                let color = match cost {
                    LETHAL => [0, 0, 0],
                    INSCRIBED => [110, 60, 140],
                    NO_INFORMATION => [205, 205, 205],
                    // Inflation: white fading to purple as cost rises.
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
            let px = (x - self.origin.0) / self.resolution * SCALE as f64;
            let py = h as f64 - (y - self.origin.1) / self.resolution * SCALE as f64;
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

        let out = std::fs::File::create(file)
            .map_err(|e| CuError::new_with_cause("cannot create PNG", e))?;
        let mut encoder = png::Encoder::new(std::io::BufWriter::new(out), w as u32, h as u32);
        encoder.set_color(png::ColorType::Rgb);
        encoder.set_depth(png::BitDepth::Eight);
        encoder
            .write_header()
            .and_then(|mut writer| writer.write_image_data(&rgb))
            .map_err(|e| CuError::new_with_cause("cannot write PNG", e))
    }
}
