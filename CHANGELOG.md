# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/) as described in `docs/versioning.md`.

## [Unreleased]

### Breaking

- Occupancy and voxel grids accumulate clamped log-odds evidence instead of
  overwriting cells on sensor rays. Unknown includes probabilities between
  classification thresholds. Construct grids with
  `from_params(geometry, CuOccupancyUpdateParams::default())?` instead of
  `new(geometry)`. Classified `get`, `set`, `fill`, and `cells` remain
  available; probability and raw log-odds accessors expose accumulated evidence.
  See `docs/log-odds-migration.md` for migration details.

### Added

- `meander-core`: `CuPath2<N>`, a fixed-capacity planar path payload (the
  planner → controller message) with `poses()`, `set()` and `length()`.

- `meander-costmap`: fixed-size Nav2 costs with allocation-free exact Euclidean
  inflation, typed robot/inflation radii, and configurable unknown handling.

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
