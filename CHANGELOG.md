# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/) as described in `docs/versioning.md`.

## Unreleased

### Added

- `meander-core`: `CuPoint2`, `CuPoint3`, `CuPose2` and `CuTwist2` payloads
  with pose composition, inversion, point transforms and exact constant-twist
  integration.
- `meander-grid`: `CuOccupancyGrid<W, H>` and `CuVoxelGrid<W, H, D>` with
  world/cell geometry, ray insertion of sensor hits, and projection of a voxel
  height band onto a 2D occupancy grid.
