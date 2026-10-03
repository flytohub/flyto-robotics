# A motion result says why it ended; twin Nav2 timing slack

Owner: claude
Branch: claude/abort-reason (twin part: claude/twin-timing, PR #44)
Date: 2026-10-04

## What changed

- `flyto_robotics/motion_outcome.py` (new, ROS-free). `MotionTrack` records
  what one goal saw: start pose, LiDAR minimum during the motion, action
  feedback, and the collision monitor's actions. `summarize()` turns that
  into the `motion_outcome` record. `describe()` turns it into one line for an
  operator. Reason codes, most specific first: `completed`, `cancelled`,
  `obstacle_blocked`, `sensor_stale`, `timeout`, `localization_error`,
  `no_path`, `no_progress`, `aborted_by_server`, `unknown`.
  - `obstacle_blocked` is assigned for any of these: a Nav2 COLLISION_AHEAD
    code (Spin 703, BackUp 714, DriveOnHeading 723); a collision monitor
    STOP or APPROACH for a polygon; or, at the stop, the nearest return in the
    travel wedge (±20° ahead for advance, behind for retreat, the whole sweep
    otherwise) inside the clearance floor.
  - The collision monitor's `"invalid source"` stop (late or missing sensor
    data) is `sensor_stale`, not an obstacle.
- `flyto_robotics/generic_ros2_adapter.py`:
  - Both backends track each goal from send to result. Every motion result now
    carries `evidence.motion_outcome`: `reason`, `action_status`(+name),
    `error_code`/`error_msg`, `start_pose`, `final_pose`,
    `distance_travelled_m` vs `requested_distance_m` (or `yaw_turned_rad` vs
    `requested_yaw_rad`, or `requested_goal`), `minimum_range_at_stop_m`,
    `travel_direction_range_m`, `minimum_range_during_m`,
    `clearance_floor_m`, `collision_monitor` events, `feedback`,
    `elapsed_seconds`.
  - A failed or timed-out motion leads its `detail` with the line from
    `describe()`, e.g. `ROS 2 action status 6 (aborted): obstacle_blocked;
    error 723 'Collision Ahead'; travelled 0.115 of 0.300 m; nearest LiDAR
    return at stop 0.220 m (floor 0.350 m)`. Completed and cancelled motions
    keep their old detail.
  - Rosbridge subscribes to `/collision_monitor_state`
    (`FLYTO_ROS2_COLLISION_STATE_TOPIC`, empty disables) and reads
    `action_feedback`. rclpy subscribes to it when `nav2_msgs` imports, and
    passes a feedback callback.
  - `_clearance_floor()` is the existing 0.35 m floor computation, factored
    out unchanged.
- `tests/test_motion_outcome.py` (new): classification, travel wedge, and
  rosbridge end-to-end for abort, success, cancel, deadline and safe stop.
- Twin (PR #44): see its description. Nav2 runs on the robot's `burger.yaml`
  with only `collision_monitor.scan.source_timeout` 0.2→0.5 s and
  `bt_navigator.default_server_timeout` 20→200 ms changed. The generator
  refuses any non-timing key.

Cloud needs no change. `external_capability_dispatch.py` already copies
`result.evidence` into `adapter_evidence` and prefixes `detail` to its own
`cancel=…, safe_stop=…` suffix, so the reason reaches Cloud through the
existing generic path.

## Why

A planner saw only `ROS 2 action status 6; cancel=refused, safe_stop=completed`.
That reads the same whether a box was in the way or the robot is broken.
`cancel=refused` is expected: the goal had already ended, so nothing was left
to cancel.

On `main`, these were already present and needed no change: the observation
waits up to 3 s for fresh odometry and LiDAR before the motion preflight
(`_observation_wait_seconds`, `_motion_preflight`); `vision.observe` (JPEG
from the compressed camera topic) and `sensing.map` (OccupancyGrid) are
declared and captured over rosbridge.

## Verified

- `make verify` (both branches): ruff clean. 1078 passed (abort-reason) and
  1065 passed (twin-timing).
- `flyto-index verify --strict` (both): no FAIL or WARN.
- `flyto-index task validate`: ruff passes. Its pytest step fails collecting
  with `No module named 'flyto_robotics'` because it runs under the indexer's
  own interpreter. The untouched main checkout fails the same way, so this is
  environmental.
- Live twin (Docker, `/clock` present, `FLYTO_ROS2_DEPLOYMENT_MODE=simulation`,
  deployment check `None`, start clearance 0.649 m). Nav2 was restarted inside
  the running container with the generated params. `ros2 param get` returned
  0.5 / 200 / `PolygonStop.radius` 0.1. Then one advance of 0.5 m at
  0.10 m/s through this branch's adapter: `completed`, travelled 0.5006 m,
  nearest return at stop 0.276 m ahead (the demo-room box), and the collision
  monitor event `APPROACH FootprintApproach` was recorded. 0 "invalid source"
  lines after the restart, against 299 in the previous Nav2 log. Scan lag during
  that run was 0.050 s median, so the 0.2 s load was not reproduced. Safe stop
  completed, and the robot was stationary within 0.28 s.

## Not verified

- An `obstacle_blocked`, `timeout` or `sensor_stale` result on a live graph.
  These are covered by unit tests only, because the one live advance
  completed.
- The rclpy backend's feedback and collision-state wiring. rclpy is not
  installed here.
- NavigateToPose through follow_path after the timing change.
- The physical robot: not touched.

## Follow-ups

- **Safety gap, not fixed here.** Preflight checks only that the clearance
  *now* is at least 0.35 m. It does not check that the requested distance
  keeps the floor. The live advance started with 0.794 m ahead, was asked for
  0.5 m, and ended 0.276 m from the box. DriveOnHeading's collision check did
  not stop it. Refuse-never-clamp suggests refusing an advance or retreat
  whose `distance_m` exceeds the travel-wedge clearance minus the floor. That
  needs an owner decision, because planners currently rely on it.
- The twin is now parked 0.276 m from the box, so a forward motion there is
  refused by preflight. Retreat first, or restart the twin.
- The running container's Nav2 was restarted by hand with the new params.
  Rebuild from `main` after #44 so a container restart keeps them.
- The demo-room world mounted by the running twin
  (`.claude/worktrees/sim-twin/sim/twin/worlds/flyto_demo_room.world`,
  uncommitted) includes Gazebo Fuel URIs (online download). AGENTS.md forbids
  that for committed assets, so it was not committed.
