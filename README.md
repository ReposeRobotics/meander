# Meander

Deterministic, allocation-free mobile robot navigation for
[copper-rs](https://github.com/copper-project/copper-rs) (`cu29`), in the
spirit of ROS 2 Nav2.

Meander is early. Version 0.2 covers the foundations only:

| Crate | What it provides |
|---|---|
| `meander-core` | Units-typed 2D/3D points, 2D poses and twists, odometry integration |
| `meander-grid` | Fixed-size 2D occupancy grids and 3D voxel grids, ray casting, voxel → 2D projection |
| `meander-mapping` | Copper tasks building log-odds 2D/3D maps from poses + laser scans or point clouds, saving Nav2 maps |

All crates are `no_std` and never allocate on the hot path. Grid sizes are
const generics, so a map's memory is known at compile time.

```rust
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use meander_core::CuPoint2;
use meander_grid::{CuGridGeometry2, CuOccupancy, CuOccupancyGrid, CuOccupancyUpdateParams};

let geometry = CuGridGeometry2::new(
    Length::new::<meter>(0.05),
    CuPoint2::new(Length::new::<meter>(0.0), Length::new::<meter>(0.0)),
)?;
let mut grid = CuOccupancyGrid::<64, 64>::from_params(geometry, CuOccupancyUpdateParams::default())?;
let sensor = CuPoint2::new(Length::new::<meter>(0.5), Length::new::<meter>(0.5));
let hit = CuPoint2::new(Length::new::<meter>(2.0), Length::new::<meter>(1.0));
grid.insert_ray(sensor, hit)?;
assert_eq!(grid.occupancy_at(hit), Some(CuOccupancy::Occupied));
# Ok::<(), cu29::prelude::CuError>(())
```

See [ROADMAP.md](ROADMAP.md) for what comes next.

## Compatibility

| Meander | cu29 | MSRV |
|---|---|---|
| 0.2 | 1.3 | 1.95 |
| 0.1 | 1.3 | 1.95 |

## Development

`just pr-check` runs formatting, clippy (std and `no_std`) and the tests.
Versioning rules are in [docs/versioning.md](docs/versioning.md); API rules in
[docs/api-conventions.md](docs/api-conventions.md).

## License

Apache-2.0.
