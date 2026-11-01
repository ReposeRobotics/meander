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

use cu29::prelude::*;

/// CopperLists to run: 60 s at the 50 Hz graph rate. The robot reaches the
/// goal in ~21 s and then holds position on zero twists.
const ITERATIONS: usize = 3000;

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
    let app = NavigateApp::builder()
        .with_log_path(
            std::path::Path::new("target/navigate-to-goal/log.copper"),
            None,
        )?
        .build()?;
    let mut running = app.start()?;
    let result = (0..ITERATIONS).try_for_each(|_| running.run_one_iteration());
    let stopped = running.stop().map(|_| ()).map_err(|fault| fault.error);
    result.and(stopped)?;
    println!("wrote target/navigate-to-goal/log.copper");
    Ok(())
}
