# Navigate-to-goal example

Nav2's "2D Goal Pose" as a real Copper application: a compile-time task
graph in `copperconfig.ron`, unified `.copper` logging, and a logreader for
the recorded run.

```text
sim/pose ──► planner ──► controller ──► sim/twist
     │            │             │
     └────────────┴──► recorder (writes navigation.png on stop)
```

- **Simulated robot:** `CuDiffDriveSimBridge` (in `examples/diff_drive`) is a
  Copper bridge — the external-world side of the graph, like a motor driver
  plus odometry on hardware. It receives `CuTwist2` commands, advances the
  diff-drive model, and publishes `CuPose2`.
- **Planner:** `meander_planner::CuPlannerTask` loads the saved Nav2 map,
  inflates it with `meander-costmap` (0.2 m robot radius, decaying cost out
  to 0.55 m), and plans once with bidirectional Dijkstra. It then republishes
  the cached `CuPath2` every cycle.
- **Controller:** `meander_controller::CuPurePursuitTask` turns pose + path
  into twists with regulated pure pursuit until the goal pose is reached.
- **Recorder:** an app-local sink that writes `navigation.png` on stop.

The goal, the start pose, and every task's parameters live in
`copperconfig.ron` — edit the planner's `goal` config there to drive
somewhere else. A goal inside an obstacle's inflation stops the planner task
with `GoalBlocked`; a goal off the map with `OutsideMap`.

```sh
cargo run --release -p meander-example-waypoint-mapping   # build and save the map first
cargo run --release -p meander-example-navigate-to-goal   # goal (2.8, 2.8) from copperconfig.ron
```

The loop is paced with the runtime's `LoopRateLimiter` at the
`copperconfig.ron` rate (50 Hz of wall time), and the sim bridge integrates
each twist over the measured time since the previous command's `tov` — so
the robot drives in real time, the same timing a hardware bridge would see.

## Watch it drive in Studio

Studio is now a generic Copper viewer: this app describes itself and
Studio renders that description — no per-app Studio code. The telemetry
sink serves, on one port (18765 by default):

- `GET /studio/describe` — the self-description (`studio: 1`): frames
  (`map`, `base_link`), the Copper graph from `copperconfig.ron`, and
  three streams — `costmap` (`occupancy_grid`), `pose` (`pose`), and
  `path` (`path`). No controls are declared: the app takes no uplink.
- `GET /events` — Server-Sent Events: the same description as the first
  `describe` event, one `map` event (inflated costmap), then `state`
  events (pose, twist, planned path) as the robot moves.

Studio is a separate repository — it is not part of this checkout.
Clone [ReposeRobotics/studio](https://github.com/ReposeRobotics/studio)
alongside Meander, then install and start the viewer **before** starting
this demo (the robot reaches its goal in ~21 s and the demo stops after
60 s, so a first-time `npm ci` started afterwards finishes too late to
watch it drive):

```sh
# in the studio checkout, once
cd studio/app && npm ci
npm run dev
# then, in this repository
cargo run --release -p meander-example-navigate-to-goal
```

Leave the viewer's Copper endpoint at `http://<host>:18765` and click
**Connect**. Displays for the costmap, robot pose, and planned path are
created from the description; use **+ Display**, the Graph panel, and
the raw inspector to explore the rest.

Over Tailscale, the sink binds `0.0.0.0` (see `copperconfig.ron`), so
serve Studio on the same machine and open it by its tailnet name. The
viewer derives the endpoint host from the hostname it was served from,
so Connect just works; the traffic stays inside the tailnet. Set the
sink's `host` back to `127.0.0.1` if the robot should not be visible on
any network.

Outputs under `target/navigate-to-goal/`:

- `navigation.png`: the costmap (black obstacles, purple inflation, white
  free space, grey unknown), the planned path in green, the driven path in
  red, the start in blue and the goal in orange.
- `log.copper`: the unified Copper log of all 3000 cycles. Inspect it with
  the bundled logreader (pass the base path, not a `_N` slab):

```sh
cargo run -p meander-example-navigate-to-goal --bin logreader -- \
    target/navigate-to-goal/log.copper extract-copperlists --export-format json
cargo run -p meander-example-navigate-to-goal --bin logreader -- \
    target/navigate-to-goal/log.copper fsck
```
