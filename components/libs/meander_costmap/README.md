# meander-costmap

`CuCostmap2<W, H>` converts a `meander-grid` occupancy map into Nav2 costs.
It measures exact Euclidean distance between cell centers and updates in
O(W × H) time using inline scratch storage, without heap allocation.

Construct it with `CuCostmap2::from_params(geometry, params)` and call
`update_from_occupancy(&grid)` after changing the occupancy map. The geometry
must match exactly; a mismatch leaves the existing costs unchanged.
`costs()[y][x]` has the same row-major layout as the occupancy grid.
`cost(cell)` and `cost_at(point)` return `None` outside the map.

Occupied cells cost `LETHAL` (254). Cells within `robot_radius` cost
`INSCRIBED` (253). Beyond that radius, through `inflation_radius` inclusive,
costs are `252 × exp(-cost_scaling_factor × clearance_in_meters)`, truncated
to an integer. More distant cells cost `FREE_SPACE` (0).
Unknown cells retain `NO_INFORMATION` (255) unless `unknown_is_lethal` is
true, in which case they become lethal cells and seed inflation.

`CuCostmapParams` is shared by Rust and RON. Defaults are:

```ron
(
    robot_radius: 0.2,
    inflation_radius: 0.55,
    cost_scaling_factor: 3.0,
    unknown_is_lethal: false,
)
```

Radii use `cu29::units::si::f64::Length` in Rust and meters in RON. The decay
coefficient is in inverse meters. Radii must be finite and nonnegative,
`inflation_radius` must be at least `robot_radius`, and the coefficient must
be finite and nonnegative. A zero coefficient gives constant cost 252 outside
the inscribed radius through the inflation radius. Unknown RON fields are rejected.

The crate supports `no_std` with default features disabled. Map storage and
scratch arrays are inline; use a `Box` or static storage for large maps.
