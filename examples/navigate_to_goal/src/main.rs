//! Drive to a goal on a saved map, like Nav2's "2D Goal Pose", as a real
//! Copper application: a compile-time task graph in `copperconfig.ron`,
//! unified `.copper` logging, and deterministic replay.
//!
//! ```text
//! sim/pose ──► planner ──► controller ──► sim/twist
//!      │            │             │
//!      └────────────┴──► recorder (writes navigation.png on stop)
//! ```
//!
//! The simulated robot is a Copper bridge (`CuDiffDriveSimBridge`): it
//! receives twists and publishes poses, like a motor driver plus odometry.
//!
//! First build a map with the waypoint example, then navigate on it:
//!
//! ```sh
//! cargo run --release -p meander-example-waypoint-mapping
//! cargo run --release -p meander-example-navigate-to-goal
//! ```
//!
//! The goal, start pose and all task parameters live in `copperconfig.ron`
//! next to this file. Writes `target/navigate-to-goal/navigation.png` and
//! the unified log `target/navigate-to-goal/log.copper`.

mod tasks;
mod telemetry;

use cu29::curuntime::LoopRateLimiter;
use cu29::prelude::*;

/// CopperLists to run: 60 s at the graph rate. The robot reaches the goal
/// in ~21 s and then holds position on zero twists.
const ITERATIONS: usize = 3000;
/// Loop rate; must match `runtime.rate_target_hz` in `copperconfig.ron`.
const RATE_HZ: u64 = 50;

#[copper_runtime(config = "copperconfig.ron")]
struct NavigateApp {}

fn main() -> CuResult<()> {
    // The planner and costmap keep whole-grid scratch arrays (a few MB):
    // give the runtime a roomy stack instead of boxing everything.
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(run)
        .map_err(|e| CuError::new_with_cause("cannot start navigation thread", e))?
        .join()
        .map_err(|_| CuError::from("navigation thread panicked"))?
}

fn run() -> CuResult<()> {
    // The unified logger cannot create its slab in a missing directory.
    std::fs::create_dir_all("target/navigate-to-goal")
        .map_err(|e| CuError::new_with_cause("cannot create output dir", e))?;
    let app = NavigateApp::builder()
        .with_log_path(
            std::path::Path::new("target/navigate-to-goal/log.copper"),
            None,
        )?
        .build()?;
    let mut running = app.start()?;
    // Pace the loop with the runtime's own limiter on the robot clock, the
    // same policy `run_until_shutdown` uses: 50 Hz of wall time, so the
    // simulated robot moves in real time and Studio can follow it live.
    // (`run_one_iteration` alone would fast-forward as fast as the CPU can.)
    let clock = running.clock();
    let mut limiter = LoopRateLimiter::from_rate_target_hz(RATE_HZ, &clock)?;
    let result = (0..ITERATIONS).try_for_each(|_| {
        limiter.limit(&clock);
        running.run_one_iteration()
    });
    let stopped = running.stop().map(|_| ()).map_err(|fault| fault.error);
    result.and(stopped)?;
    println!("wrote target/navigate-to-goal/log.copper");
    Ok(())
}
