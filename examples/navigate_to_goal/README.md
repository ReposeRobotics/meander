# Navigate-to-goal example

Nav2's "2D Goal Pose" in one file: drive a diff-drive robot to a goal on a
saved map.

1. **Map:** load the Nav2 map (`map.pgm` + `map.yaml`) saved by the waypoint
   mapping example.
2. **Costmap:** `meander-costmap` inflates obstacles by the robot radius
   (0.2 m) and adds a decaying cost out to 0.55 m, so paths keep their distance.
3. **Plan:** `meander-planner`'s bidirectional Dijkstra finds the cheapest
   8-connected path from the start to the goal on that costmap, written into a
   `CuPath2` (the planner → controller message).
4. **Drive:** `meander-controller`'s pure pursuit turns the path into twists;
   the `examples/diff_drive` model turns those into wheel speeds and moves the
   robot until it reaches the goal.

```sh
cargo run --release -p meander-example-waypoint-mapping      # build and save the map first
cargo run --release -p meander-example-navigate-to-goal       # default goal (2.8, 2.8)
cargo run --release -p meander-example-navigate-to-goal -- 0.5 -3.2   # goal x y in metres
```

It writes `target/navigate-to-goal/navigation.png`: the costmap (black
obstacles, purple inflation, white free space, grey unknown), the planned path
in green, the driven path in red, the start in blue and the goal in orange.

A goal inside an obstacle's inflation stops with `GoalBlocked`; a goal off the
map with `OutsideMap`.
