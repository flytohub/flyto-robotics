# Map-frame pose beside odometry

Owner: claude
Branch: claude/map-frame-pose
Date: 2026-10-04

## What changed

- `flyto_robotics/map_frame.py` (new, pure): reads a map->odom transform as
  (x, y, yaw) and composes it with the odometry pose.
- `generic_ros2_adapter.py`: both backends keep the map->odom transform's
  values, not only when it was seen. The snapshot gains `map_pose`
  (`{frame: "map", x, y, yaw}`) while the transform and odometry are both
  fresh; motion evidence gains `map_pose`; a motion track records its start
  map pose.
- `motion_outcome.py`: `start_map_pose` / `final_map_pose` in the summary, and
  the operator line ends with `stopped at map x=.. y=.. yaw=..`.
- `provider_evidence.recovery_context` carries both map poses.
- `ros2_observation_bundle.py` + `contracts/ros2-observation-bundle-v1.schema.json`:
  optional `map_pose`. A bundle without one has no key, so its content address
  is unchanged; one with a non-map frame, or without `map_tf_available`, is refused.

Every odometry field is unchanged; motion is still judged on odometry.

## Why

A blocked straight move was reported only in `odom`, so Cloud's recovery
planner could never offer `motion.navigate` (a map coordinate) and fell back
to a blind rotate/advance detour computed from odometry, which kept failing.

## Verified

- `make verify` (1156 passed, 1 skipped), `flyto-index verify . --full-scan --strict` exit 0.
- `tests/test_map_frame_pose.py` (10 tests; fails to collect without the change).
- Live, Gazebo twin `flyto-turtlebot3-twin` over rosbridge ws://127.0.0.1:19090
  only, from the worktree code: advance 1.2 m -> `obstacle_blocked` after
  0.364 m (nearest 0.40-0.43 m, floor 0.35 m), result carries
  `start_map_pose` ~(0.00, -0.01) and `final_map_pose` ~(0.33, 0.00), detail
  ends `stopped at map x=0.331 y=0.004`.

## Not verified

- Navigate to the original 1.2 m end (map ~(1.19, -0.01)) from the blocked
  pose did NOT arrive in the twin: the adapter timed out at 180 s with 14 Nav2
  recoveries ("Failed to make progress"); the same happened for a clear point
  at 1.7 m. From open space Nav2 did reach (0.2, -0.8) and (1.7, -0.7). The
  1.2 m end lies ~0.14 m behind the box's far face, inside the 0.35 m floor
  and the 0.5 m inflation. Nav2 tuning was not changed.
- Physical robot: not run.
