# A navigation that starts inside inflation escapes first, or is refused at once

Owner: claude
Branch: claude/escape-inflation
Date: 2026-10-04

## What changed

- `flyto_robotics/inflation_escape.py` (new, no ROS import): costmap parameter
  interpretation (`read_costmap_geometry`: each costmap's `plugins`, the
  `*InflationLayer` plugin's `enabled` / `inflation_radius`, `footprint` as its
  circumscribed radius else `robot_radius`; largest of each across costmaps;
  Nav2 defaults 0.55 / 0.1 m when unreadable, `source` = parameters / partial /
  fallback), sweep geometry (`sweep_beams`, `sector_minimum` with four 90-degree
  sectors and "under half the bins readable = unreadable", `obstacle_cluster`,
  `segment_clearance`), and the decision (`backoff_distance`,
  `allowed_backoff`, `choose_sides`, `lateral_offset`, `plan_escape`,
  `waypoint_pose`).
- Pinned = nearest return in the front sector (+/-45 deg) < inflation + robot
  radius. Escape = back-off `clamp(threshold - front, 0, max)` limited so
  `rear - backoff >= floor` (0 if rear unreadable or the back-off segment would
  pass within the floor of any return); then the side with the larger sector
  minimum first, lateral offset = outermost `range*sin(bearing)` of the obstacle
  cluster + robot radius + margin, accepted only if the side is readable, the
  offset <= max lateral, and the lateral segment keeps the floor from every
  return; else the other side; else `no_escape_room`.
- `generic_ros2_adapter.py`: `GenericROS2Adapter.invoke` decides once per call
  id for `motion.navigate` (lidar basis, sweep and `map_pose` present).
  `no_escape_room` -> REFUSED before any motion, `evidence.reason_code`,
  `evidence.navigation_escape` (clearances, costmap, sides tried), detail led
  by the reason. Escape -> `motion.retreat` leg (`<call>:escape-backoff`, 0.05
  m/s) when `/backup` is declared, then `NavigateThroughPoses` [waypoint, goal]
  under the call's own id when `/navigate_through_poses` is on the graph, else
  `<call>:escape-waypoint` NavigateToPose then the goal under the call id.
  `cancel(call_id)` reaches the running leg and stops later legs. Backends gain
  `get_parameter(node, name)` (standard `<node>/get_parameters`,
  `rcl_interfaces/srv/GetParameters`; rosbridge sends the service type) and
  `action_available(capability_id)`; the through-poses action is internal
  (`ESCAPE_INTERFACES`), never declared, and not added to `discover()` so
  runtime snapshots are unchanged.
- Env: `FLYTO_ROS2_INFLATION_ESCAPE` (on), `FLYTO_ROS2_COSTMAP_NODES`,
  `FLYTO_ROS2_ESCAPE_MAX_BACKOFF_M` (0.30), `FLYTO_ROS2_ESCAPE_MARGIN_M`
  (0.10), `FLYTO_ROS2_ESCAPE_MAX_LATERAL_M` (1.0),
  `FLYTO_ROS2_NAVIGATE_THROUGH_ACTION`. Floor unchanged (0.35 m).
- 0.4.0 (setup.py, package.xml, `__version__`); CHANGELOG, README, DECISIONS,
  STATE. Tests: `tests/test_inflation_escape.py` (176 cases).

## Why

Twin, stock TurtleBot3 Nav2 burger params: an advance stopped at the floor
about 0.40 m in front of a 0.30 x 0.40 m box; Cloud's recovery navigate to
(1.20, 0) never moved the robot (controller "Failed to make progress",
spin/wait/backup cycles, abort after ~170 s). The start is inside the box's
inflation (0.40 < 0.50 + 0.10). The robot runs the same stock config, so Nav2
tuning was rejected; lowering the floor and retrying the same goal were also
rejected. Side sectors are not part of the pinned test: they would call every
corridor narrower than ~1.2 m pinned.

## Verified

- Live twin parameters, read-only (`docker exec flyto-turtlebot3-twin ... ros2
  param get` / `ros2 service call .../get_parameters`): local and global
  `inflation_layer.inflation_radius` 0.5, `robot_radius` 0.1, `footprint` "[]",
  inflation plugin present in both `plugins` lists; `/navigate_through_poses`
  and `/backup` listed; Jazzy `NavigateThroughPoses` goal is
  `geometry_msgs/PoseStamped[] poses` + `behavior_tree`. A GetParameters call
  naming one unknown parameter returns an empty `values` for the whole request,
  which is why the adapter asks one name per call.
- For the observed scene (ray-cast), the decision is: pinned, front 0.40,
  threshold 0.60, back-off 0.20, left side, lateral offset 0.395 m, waypoint
  (-0.20, 0.395) in the start's robot frame, path clearance 0.60 m.
- `make verify PYTHON=<repo .venv>`: exit 0, ruff "All checks passed!",
  1400 passed / 1 skipped, asset and dry-run targets pass.
- `flyto-index verify --strict` (in the worktree): 20 pass, 0 warn, 0 fail.
- `flyto-index task validate` (indexer interpreter, worktree on PYTHONPATH):
  ruff pass, pytest pass, overall pass. `pr-risk`: medium, no breaking change.

## Not verified

- Not run on the Gazebo twin or the physical robot (the owner was using the
  twin). Whether Nav2 reaches (1.20, 0) after the back-off and waypoint is
  unproven; so is the rosbridge `NavigateThroughPoses` goal and the
  `get_parameters` call through rosbridge (shape tested with fakes only).
- The rclpy backend's `get_parameter` / `action_available` / through-poses goal
  are untested (no rclpy in CI).
- The LiDAR is assumed at the robot centre facing forward (TurtleBot3
  base_scan is 0.032 m behind centre; ignored).
- Kilted+ changed `NavigateThroughPoses.poses` to `nav_msgs/Goals`; only the
  Jazzy shape is sent.

## Follow-ups

- Run the observed scenario on the twin: blocked advance, then navigate to the
  original end; confirm `navigation_escape.legs` and arrival.
- If the waypoint itself sits in inflation for wider obstacles, consider a
  margin derived from `inflation_radius` rather than the fixed 0.10 m.
