# Waypoint mapping example

A diff-drive robot drives through six waypoints in an 8 m × 8 m room with
boxes and pillars. A simulated 360° lidar (180 beams, 6 m, 10 Hz) feeds
`meander-mapping`'s `CuScanMapper2`, which builds a 5 cm log-odds occupancy
map. At the end the map is saved.

```sh
cargo run --release -p meander-example-waypoint-mapping
```

Output in `target/waypoint-mapping/` (pass a file stem to change it):

- `map.pgm` + `map.yaml`: a standard Nav2 / ROS `map_server` map.
- `map.png`: the same map enlarged, with the driven path in red and the
  waypoints in blue. Black is occupied, white is free, grey is unknown.

To change the run, edit the constants at the top of `src/main.rs`:
`BOXES`, `PILLARS`, `WAYPOINTS`, the lidar settings, and the map size.

The robot uses the `examples/diff_drive` model: a simple go-to-waypoint
controller computes a twist, `CuDiffDrive::wheel_speeds` turns it into wheel
speeds, and `CuDiffDrive::step` integrates the pose. The world is only used to
simulate lidar returns; the robot does not collide with obstacles.
