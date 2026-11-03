//! Drive a diff-drive robot through waypoints in a room full of obstacles,
//! map it with a simulated 360° lidar, and save the map.
//!
//! ```sh
//! cargo run -p meander-example-waypoint-mapping            # writes target/waypoint-mapping/map.{pgm,yaml,png}
//! cargo run -p meander-example-waypoint-mapping -- out/lab # writes out/lab.{pgm,yaml,png}
//! ```
//!
//! `map.pgm` + `map.yaml` are a standard Nav2 map; `map.png` is the same map,
//! enlarged, with the driven path (red) and the waypoints (blue) drawn on top.

use core::f64::consts::PI;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::{Angle, AngularVelocity, Length, Time, Velocity};
use cu29::units::si::length::meter;
use cu29::units::si::time::second;
use cu29::units::si::velocity::meter_per_second;
use meander_core::{CuLaserScan, CuPoint3, CuPose2, CuTwist2, normalize_angle};
use meander_example_diff_drive::{CuDiffDrive, CuDiffDriveParams, CuWheelSpeeds2};
use meander_grid::CuOccupancy;
use meander_mapping::{CuMapUpdateReport, CuMappingParams, CuScanMapper2};

// ---- The world ---------------------------------------------------------------

/// An 8 m × 8 m room, centred on the origin.
const ROOM: f64 = 4.0;
/// Axis-aligned boxes: (centre x, centre y, half width, half height), metres.
const BOXES: [(f64, f64, f64, f64); 4] = [
    (-1.5, -1.5, 0.5, 0.5),
    (1.5, 1.5, 0.6, 0.4),
    (2.5, -1.5, 0.3, 0.8),
    (-1.8, 2.0, 0.6, 0.3),
];
/// Round pillars: (centre x, centre y, radius), metres.
const PILLARS: [(f64, f64, f64); 3] = [(0.0, -2.5, 0.4), (1.2, -0.8, 0.35), (-0.5, 1.2, 0.3)];

/// The robot drives through these, in order.
const WAYPOINTS: [(f64, f64); 6] = [
    (3.2, -3.2),
    (3.2, 3.2),
    (-3.2, 3.2),
    (-3.2, -1.0),
    (0.0, 0.0),
    (0.6, -1.8),
];
const START: (f64, f64) = (-3.2, -3.2);

// ---- Robot, sensor and map -----------------------------------------------------

const DT: f64 = 0.02; // simulation step, s
const SCAN_EVERY: usize = 5; // lidar at 10 Hz
const BEAMS: usize = 180;
const LIDAR_RANGE: f64 = 6.0;
const MAX_SPEED: f64 = 0.5; // m/s
const MAX_TURN: f64 = 1.5; // rad/s
const ARRIVED: f64 = 0.1; // m

/// 200 × 200 cells of 5 cm: a 10 m × 10 m map around the room.
const MAP_W: usize = 200;
const MAP_H: usize = 200;
const MAP_ORIGIN: f64 = -5.0;
const RESOLUTION: f64 = 0.05;

fn m(v: f64) -> Length {
    Length::new::<meter>(v)
}

fn main() -> CuResult<()> {
    let stem = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "target/waypoint-mapping/map".into());
    if let Some(dir) = std::path::Path::new(&stem).parent() {
        std::fs::create_dir_all(dir)
            .map_err(|e| CuError::new_with_cause("cannot create output dir", e))?;
    }

    let robot = CuDiffDrive::from_params(CuDiffDriveParams::default())?;
    let mut mapper = CuScanMapper2::<MAP_W, MAP_H>::from_params(CuMappingParams {
        resolution: m(RESOLUTION),
        origin: CuPoint3::new(m(MAP_ORIGIN), m(MAP_ORIGIN), m(0.0)),
        ..CuMappingParams::default()
    })?;

    let mut pose = CuPose2::new(m(START.0), m(START.1), Angle::default());
    let mut scan = CuLaserScan::<BEAMS> {
        angle_min: Angle::new::<radian>(-PI),
        angle_increment: Angle::new::<radian>(2.0 * PI / BEAMS as f64),
        range_min: m(0.05),
        range_max: m(LIDAR_RANGE),
        ..CuLaserScan::default()
    };
    let mut report = CuMapUpdateReport::default();
    let mut wheels = CuWheelSpeeds2::default();
    let mut path = vec![(START.0, START.1)];
    let (mut scans, mut distance) = (0, 0.0);

    for (i, &(gx, gy)) in WAYPOINTS.iter().enumerate() {
        for step in 0.. {
            let (x, y) = (pose.x.get::<meter>(), pose.y.get::<meter>());
            let to_goal = (gx - x).hypot(gy - y);
            if to_goal < ARRIVED {
                break;
            }
            if step > 60_000 {
                return Err(CuError::from("robot got stuck on the way to a waypoint"));
            }

            // Lidar + mapping at 10 Hz.
            if step % SCAN_EVERY == 0 {
                simulate_lidar(&pose, &mut scan);
                mapper.insert_scan(&pose, &scan, &mut report)?;
                scans += 1;
            }

            // Turn towards the waypoint; drive forward only when roughly facing it.
            let heading = Angle::new::<radian>((gy - y).atan2(gx - x));
            let error = normalize_angle(heading - pose.theta).get::<radian>();
            let twist = CuTwist2::new(
                Velocity::new::<meter_per_second>(MAX_SPEED * error.cos().max(0.0)),
                Velocity::default(),
                AngularVelocity::new::<radian_per_second>((2.0 * error).clamp(-MAX_TURN, MAX_TURN)),
            );
            robot.wheel_speeds(&twist, &mut wheels);
            robot.step(&mut pose, &wheels, Time::new::<second>(DT));

            let now = (pose.x.get::<meter>(), pose.y.get::<meter>());
            distance += (now.0 - x).hypot(now.1 - y);
            path.push(now);
        }
        println!("reached waypoint {} at ({gx:.1}, {gy:.1})", i + 1);
    }

    meander_grid::save_map(mapper.grid(), &stem)?;
    write_png(mapper.grid().cells(), &path, &format!("{stem}.png"))?;

    let known = mapper
        .grid()
        .cells()
        .iter()
        .flatten()
        .filter(|c| **c != CuOccupancy::Unknown)
        .count();
    println!(
        "drove {distance:.1} m, {scans} scans, mapped {:.0}% of the map",
        100.0 * known as f64 / (MAP_W * MAP_H) as f64
    );
    println!("saved {stem}.pgm + {stem}.yaml (Nav2 map) and {stem}.png (picture)");
    Ok(())
}

// ---- Simulated lidar -----------------------------------------------------------

/// Fills `scan.ranges` with the distance to the nearest obstacle along each
/// beam, or 0 (no return) beyond the lidar's range.
fn simulate_lidar(pose: &CuPose2, scan: &mut CuLaserScan<BEAMS>) {
    let (x, y) = (pose.x.get::<meter>(), pose.y.get::<meter>());
    for (i, range) in scan.ranges.iter_mut().enumerate() {
        let angle = pose.theta.get::<radian>() - PI + i as f64 * 2.0 * PI / BEAMS as f64;
        let d = cast_ray(x, y, angle.cos(), angle.sin());
        *range = m(if d <= LIDAR_RANGE { d } else { 0.0 });
    }
}

/// Distance from `(x, y)` along the unit direction `(dx, dy)` to the first obstacle.
fn cast_ray(x: f64, y: f64, dx: f64, dy: f64) -> f64 {
    let mut nearest = f64::INFINITY;
    // Room walls and boxes are axis-aligned rectangles: intersect their edges.
    let mut rect = |cx: f64, cy: f64, hw: f64, hh: f64| {
        for (wall, lo, hi, vertical) in [
            (cx - hw, cy - hh, cy + hh, true),
            (cx + hw, cy - hh, cy + hh, true),
            (cy - hh, cx - hw, cx + hw, false),
            (cy + hh, cx - hw, cx + hw, false),
        ] {
            let (p, d, q, dq) = if vertical {
                (x, dx, y, dy)
            } else {
                (y, dy, x, dx)
            };
            if d.abs() > 1e-12 {
                let t = (wall - p) / d;
                let along = q + t * dq;
                if t > 1e-9 && along >= lo && along <= hi {
                    nearest = nearest.min(t);
                }
            }
        }
    };
    rect(0.0, 0.0, ROOM, ROOM);
    for (cx, cy, hw, hh) in BOXES {
        rect(cx, cy, hw, hh);
    }
    // Pillars: solve |p + t·d - c| = r for the nearest positive t.
    for (cx, cy, r) in PILLARS {
        let (ox, oy) = (x - cx, y - cy);
        let b = ox * dx + oy * dy;
        let disc = b * b - (ox * ox + oy * oy - r * r);
        if disc >= 0.0 {
            let t = -b - disc.sqrt();
            if t > 1e-9 {
                nearest = nearest.min(t);
            }
        }
    }
    nearest
}

// ---- Picture -------------------------------------------------------------------

/// Writes the map as an RGB PNG, 3 pixels per cell, with the path and waypoints.
fn write_png(
    cells: &[[CuOccupancy; MAP_W]; MAP_H],
    path: &[(f64, f64)],
    file: &str,
) -> CuResult<()> {
    const SCALE: usize = 3;
    let (w, h) = (MAP_W * SCALE, MAP_H * SCALE);
    let mut rgb = vec![0u8; w * h * 3];
    let mut paint = |px: usize, py: usize, color: [u8; 3]| {
        if px < w && py < h {
            rgb[(py * w + px) * 3..][..3].copy_from_slice(&color);
        }
    };
    for (cy, row) in cells.iter().enumerate() {
        for (cx, cell) in row.iter().enumerate() {
            let shade = match cell {
                CuOccupancy::Occupied => 0,
                CuOccupancy::Free => 254,
                CuOccupancy::Unknown => 205,
            };
            for dy in 0..SCALE {
                for dx in 0..SCALE {
                    // Image rows run top to bottom; map y runs bottom to top.
                    paint(cx * SCALE + dx, (MAP_H - 1 - cy) * SCALE + dy, [shade; 3]);
                }
            }
        }
    }
    let to_px = |x: f64, y: f64| {
        let px = (x - MAP_ORIGIN) / RESOLUTION * SCALE as f64;
        let py = h as f64 - (y - MAP_ORIGIN) / RESOLUTION * SCALE as f64;
        (px as usize, py as usize)
    };
    for &(x, y) in path {
        let (px, py) = to_px(x, y);
        paint(px, py, [220, 40, 40]);
    }
    for (x, y) in WAYPOINTS.iter().copied().chain([START]) {
        let (px, py) = to_px(x, y);
        for d in 0..25 {
            paint(px + d % 5 - 2, py + d / 5 - 2, [40, 90, 230]);
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
