# Probabilistic grid migration

Replace `CuOccupancyGrid::new(geometry)` and `CuVoxelGrid::new(geometry)` with
`from_params(geometry, CuOccupancyUpdateParams::default())?`.
The same parameter struct deserializes from RON; call `validate()` after
deserialization, or let the constructor validate it.

Rays now add hit or miss evidence to a 0.5 prior. With defaults, one hit
classifies occupied, while four misses establish free space. The minimum
clamp is 0.09 so one spurious 0.7 hit on saturated free evidence stays
below the 0.2 free threshold; the maximum clamp is 0.97. Both thresholds
are inclusive; probabilities between them classify unknown. Unknown also
covers never observed cells and cells reset with `set(Unknown)`.

`set` and `fill` use the configured clamp minimum for Free, maximum for
Occupied, and zero log-odds for Unknown. `get`, `occupancy_at` and borrowed
`cells` retain their classified interface. `probability` and `log_odds`
return dimensionless f64 values for in-bounds cells (including the 0.5 prior),
and None outside. `log_odds_rows` borrows the underlying f32 storage.

Voxel projection classifies each voxel using the source parameters, selects
occupied before free before unknown, and writes that classification into the
target using its own clamp extremes. It does not transfer probability evidence.
