# meander-grid

Fixed-size occupancy and voxel maps for Copper navigation.

## ROS / Nav2 map files

`encode_pgm` writes a binary P5 image into caller-provided storage;
`decode_pgm` loads image bytes into an existing grid. Both work without `std`
and allocate no memory on success. The existing grid must have the YAML
resolution and origin, and the image dimensions must equal its const `W`/`H`.
The top PGM row corresponds to grid row `H - 1`. Saved pixel values are
occupied = 0, free = 254, unknown = 205.

`CuMapMetadata` uses typed lengths and a `CuPose2` in Rust; its serde form uses
Nav2's scalar `resolution` and `origin: [x, y, yaw]` in meters/radians.
`from_yaml` / `to_yaml` allocate and belong outside the real-time path.
Loading applies `negate` and the occupancy thresholds: at or above
`occupied_thresh` is occupied, at or below `free_thresh` is free, and
all other values are unknown. Only trinary mode is supported. Nonzero yaw
is rejected when decoding because grid geometry is axis-aligned.

With the default `std` feature, `save_map` and `load_map` handle filesystem I/O:

```rust
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use meander_core::CuPoint2;
use meander_grid::{
    CuCell2, CuGridGeometry2, CuOccupancy, CuOccupancyGrid, CuOccupancyUpdateParams, load_map,
    save_map,
};

# fn main() -> cu29::prelude::CuResult<()> {
let geometry = CuGridGeometry2::new(Length::new::<meter>(0.05), CuPoint2::default())?;
let mut grid = CuOccupancyGrid::<64, 64>::from_params(geometry, CuOccupancyUpdateParams::default())?;
grid.set(CuCell2::new(10, 20), CuOccupancy::Occupied)?;
save_map(&grid, "warehouse")?; // warehouse.pgm + warehouse.yaml
let restored = load_map::<64, 64>("warehouse.yaml")?;
assert_eq!(restored.get(CuCell2::new(10, 20)), Some(CuOccupancy::Occupied));
# Ok(())
# }
```

Image paths are resolved relative to the YAML file, including when the current
working directory differs. `save_map` appends the two extensions to the supplied
stem, overwrites existing files, and uses free/occupied thresholds 0.196/0.65,
preserving the unknown pixel 205 on reload.
Writes are not atomic. `load_map` creates geometry from YAML, validates its
resolution, and rejects a PGM size that differs from `W`/`H`.

The hand-written YAML reader deliberately accepts the flat subset emitted by
[Nav2 map_saver](https://api.nav2.org/nav2-jazzy/html/map__io_8cpp_source.html):
scalar fields, inline origin arrays, optional mode (defaults to trinary), comments,
and plain, single-quoted or double-quoted paths. Double-quoted paths support
escaped quotes, backslashes and slashes. Duplicate keys, unknown fields, invalid
thresholds, anchors, tags, block scalars and other YAML structures are rejected.
This avoids a general YAML dependency and keeps metadata text conversion
available in `no_std + alloc`; it is not a general YAML parser.
The test fixture is hand-written to match Nav2's emitted PGM/YAML format.
