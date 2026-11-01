# Roadmap

Meander grows slowly: each release adds one layer and keeps it solid.

## 0.1 — Core and maps

- `meander-core`: units-typed `CuPoint2`, `CuPoint3`, `CuPose2`, `CuTwist2`;
  pose composition/inversion and exact constant-twist integration (odometry).
- `meander-grid`: fixed-size 2D occupancy grid and 3D voxel grid with
  world↔cell geometry, clamped probabilistic log-odds ray updates, and voxel → 2D
  projection.

## Later (unordered, not committed)

- Costmap layers: static, obstacle, inflation.
- Drive kinematics: differential drive, then omnidirectional.
- Global planner contract and an A* grid planner.
- Path-following controller (Pure Pursuit, then a DWB-style local planner).
- Localization (AMCL-style), behaviors and recovery, simulation bridge.
- Mobile manipulation by composing with Repose (`repose-kinematics`,
  `repose-collision`).
