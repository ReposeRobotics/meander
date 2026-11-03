# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/) as described in `docs/versioning.md`.

## [Unreleased]

### Added

- Navigate-to-goal streams its saved occupancy map for 3D display in Studio.

### Changed

- CI/review cost controls: CodeRabbit no longer auto-reviews (request one
  with `@coderabbitai review` on a PR; see `.coderabbit.yaml`), and the
  Actions gate skips draft PRs and cancels superseded runs on the same
  branch.

### Breaking

- Occupancy and voxel grids accumulate clamped log-odds evidence instead of
  overwriting cells on sensor rays. Unknown includes probabilities between
  classification thresholds. Construct grids with
  `from_params(geometry, CuOccupancyUpdateParams::default())?` instead of
  `new(geometry)`. Classified `get`, `set`, `fill`, and `cells` remain
  available; probability and raw log-odds accessors expose accumulated evidence.
  See `docs/log-odds-migration.md` for migration details.

### Added

- `examples/navigate_to_goal`: the telemetry sink now serves the Studio
  Copper viewer contract — `GET /studio/describe` (self-description:
  `costmap`/`pose`/`path` semantic-kind streams, `map`/`base_link`
  frames, Copper graph, no controls) and the same document as the first
  `describe` event on `/events`, before the existing `map`/`state`
  events. The legacy Drive-tab Live mode instructions are replaced by
  the generic viewer's Connect flow.
- `meander-planner`: `CuPlannerTask<W, H, N>` Copper task (with
  `CuPlannerTaskParams`) that loads and inflates a Nav2 map, plans once to
  the configured goal, and republishes the cached `CuPath2<N>`.
- `meander-controller`: `CuPurePursuitTask<N>` Copper task wrapping regulated
  pure pursuit (pose + `CuPath2<N>` in, `CuTwist2` out).
- `examples/diff_drive`: `CuDiffDriveSimBridge`, a Copper bridge simulating
  the diff-drive base (twist in, pose out) with `CuDiffDriveSimParams`.
- `examples/navigate_to_goal` now runs in real time: the loop is paced by
  the runtime's `LoopRateLimiter` at the RON rate, and the sim bridge
  integrates twists over measured `tov` elapsed time (clamped to 100 ms)
  instead of a fixed sample period. `CuDiffDriveSimParams.sample_period`
  is removed. A telemetry sink streams the costmap, pose, twist and
  planned path as Server-Sent Events on `127.0.0.1:18765` for Studio's
  Live mode. The sink's bind host is configurable (`host` in RON); the
  example sets `0.0.0.0` so Studio on another tailnet machine can watch.
- `examples/navigate_to_goal` is now a Copper application
  (`copperconfig.ron` + `#[copper_runtime]`) with unified `.copper` logging
  and a `logreader` binary, instead of a standalone simulation loop.

- `examples/navigate_to_goal`: load a saved Nav2 map, inflate it with
  `meander-costmap`, plan to a goal with `meander-planner`'s bidirectional
  Dijkstra, follow the `CuPath2` with `meander-controller`'s pure pursuit, and
  save a picture of the costmap, planned path and driven path.

- `meander-core`: `CuPath2<N>`, a fixed-capacity planar path payload (the
  planner → controller message) with `poses()`, `set()` and `length()`.

- `meander-costmap`: fixed-size Nav2 costs with allocation-free exact Euclidean
  inflation, typed robot/inflation radii, and configurable unknown handling.

- `meander-controller`: `CuPurePursuit` regulated pure pursuit path following for
  differential-drive robots with `CuPurePursuitParams`, lookahead segment
  interpolation, curvature-preserving velocity clamping, rotate-in-place at
  sharp angles, and goal heading alignment.

- `meander-planner`: allocation-free `CuBidirectionalDijkstra<W, H>` over
  Nav2-style cost grids (`&[[u8; W]; H]` + `CuGridGeometry2`), with 8-connected
  steps, no corner cutting past blocked cells, correct bidirectional
  termination, and goal tolerance for blocked goals. `plan` writes cell-center
  waypoints with along-path headings into a caller buffer and reports a
  `CuPlanStatus` (`Found`, `NoPath`, `StartBlocked`, `GoalBlocked`,
  `OutsideMap`, `PathTooLong`) with waypoint count and total cost.
- `examples/waypoint_mapping`: a diff-drive robot drives through waypoints in a
  room with obstacles, maps it with a simulated lidar through
  `CuScanMapper2`, and saves a Nav2 map plus a PNG picture with its path.

- `meander-mapping`: `CuScanMappingTask` (`CuPose2` + `CuLaserScan<N>` → 2D
  log-odds map) and `CuCloudMappingTask` (`CuPose3` + `CuPointCloud3<N>` → 3D
  voxel map), built on the allocation-free `CuScanMapper2` / `CuCloudMapper3`.
  Sensor mount pose, `max_range`, no-return clearing and the 3D → 2D height
  band are configured through `CuMappingParams`; each observation publishes a
  `CuMapUpdateReport`; `freeze`/`thaw` round-trip the whole map for replay;
  `save_path` writes a Nav2 PGM + YAML map on `stop`.
- `meander-grid`: `load_log_odds_rows` on both grids restores a saved
  log-odds state, validating every value against the clamp range.

- `meander-core`: unit quaternions, spatial poses and fixed-capacity laser scan /
  point cloud payloads with allocation-free transforms into caller-owned buffers.
- `meander-grid`: allocation-free trinary P5 map encoding/decoding, typed Nav2
  YAML metadata, and std-only PGM/YAML save/load helpers.

- Primitive-geometry diff-drive URDF with chassis, wheels and rear caster, plus
  a consistency test against the example model’s default wheel dimensions.

- `examples/diff_drive`: a minimal differential-drive robot model
  (`CuDiffDrive`, `CuDiffDriveParams`, `CuWheelSpeeds2`) with forward/inverse
  kinematics, an odometry step and tests.
- Branch, worktree and PR workflow (`AGENTS.md`, `CONTRIBUTING.md`) with
  `just wt`, `just wt-clean` and `just tag`; releases land through a PR.
- `meander-core`: `CuPoint2`, `CuPoint3`, `CuPose2` and `CuTwist2` payloads
  with pose composition, inversion, point transforms and exact constant-twist
  integration.
- `meander-grid`: `CuOccupancyGrid<W, H>` and `CuVoxelGrid<W, H, D>` with
  world/cell geometry, ray insertion of sensor hits, and projection of a voxel
  height band onto a 2D occupancy grid.
