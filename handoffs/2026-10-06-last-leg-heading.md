# The escape waypoint's heading keeps the robot facing past the obstacle

Owner: claude
Branch: claude/last-leg
Date: 2026-10-06

## What changed

- `flyto_robotics/inflation_escape.py`: `read_heading_tolerance()` reads the
  largest `yaw_goal_tolerance` of the controller's goal checkers (same walk as
  `read_arrival_tolerance`, now a shared helper; Nav2's 0.25 rad fallback; a
  quarter turn or more is not believed). `plan_escape()` takes
  `heading_tolerance_rad` and stores `clear_heading_rad` on the decision:
  from the waypoint, the bound heading that passes every obstacle return by
  robot radius + margin (tangents), and never back across the start's line of
  travel (the LiDAR sees only the obstacle's near face). `waypoint_heading()`
  turns the goal's bearing toward the open side until heading +/- tolerance is
  all on the open side of that bound; `waypoint_pose()` uses it.
- `flyto_robotics/generic_ros2_adapter.py`: reads the tolerance next to the
  arrival tolerance and adds `waypoint_heading` to `navigation_escape`.
- 0.6.2, CHANGELOG. No Nav2 parameter changed on the twin or the robot.

## Why (measured)

Desktop log + `flyto-cloud/evidence/*/evidence.jsonl` + twin
`~/.ros/log/{controller_server,bt_navigator,behavior_server,planner_server}_*_1791281724*`
(read-only; container clock is UTC, Desktop log is UTC+8).

- Slow: t-c3e495596129a8d4, navigate execution 816b4418 (20:04:21-20:06:42
  local). Backoff 4.8 s; waypoint leg 16.6 s, sent at map yaw -0.278 (bearing
  to the goal), ended at (-0.158, 0.371) yaw **-0.523** (inside the 0.25 rad
  `yaw_goal_tolerance`), facing the box. Goal leg: "Failed to make progress"
  at +11.5, 23.1, 35.5, 47.1, 60.5, 72.1, 89.0, 100.3 s (every ~11.5 s =
  `movement_time_allowance` 10 s + loop), recoveries clear local/global
  costmap, spin 1.57, wait 5 s, backup; it succeeded 16 s after the backup.
  Goal leg 118.9 s, `number_of_recoveries` 9.
- Fast: t-6699fe46d1b42d6d, execution 5a8b5d31 (19:57:35-19:58:08). Same box,
  same lateral offset (0.401 vs 0.397 m), waypoint sent at -0.284, ended at
  yaw **-0.065**; goal leg 17.4 s, 0 recoveries, no progress failure.
- All 18 escape navigations in the evidence store on 2026-10-06: 16 goal legs
  started within 0.07 rad of the line of travel and took 8.9-18.6 s with 0
  recoveries. The two slow ones: this run (start yaw -0.523) and the 0.6.0
  run 4 at 18:38 (start yaw +0.024 but y = 0.270, the narrowed waypoint fixed
  in 0.6.1).
- Controller is DWB (critics RotateToGoal, Oscillation, BaseObstacle,
  GoalAlign, PathAlign, PathDist, GoalDist), `min_vel_x` 0 (no reversing), no
  RotationShim; progress checker 0.1 m / 10 s; goal checker 0.25 m / 0.25 rad;
  inflation 0.5, robot_radius 0.1. Read from `/etc/ros/twin-burger.yaml` in
  the container, which is TurtleBot3's `burger.yaml` with only the timing keys
  `twin_nav2_params.py` allows; `deploy/native_ros2/nav2.service` runs the
  robot on the same `burger.yaml`. So the twin config is not the defect and
  changing it would only hide this from the robot.

Hypotheses: (a)/(c) as stated (RotationShim spin) are rejected because there
is no RotationShim, but their substance holds: the arrival heading the
waypoint goal permits is what separates the slow run from the 16 fast ones.
(b) the goal leg's path runs through the box's inflation in every run (0.5 m
inflation vs ~0.2 m gap) and does not separate them. (d) rejected as a cause:
a legitimate turn of <= 0.8 rad at 1 rad/s takes ~1 s, far inside 10 s; the
progress failures are the symptom of DWB not leaving the pose.

Not established: DWB's per-trajectory scores (no critic logging); why DWB
turned clockwise (odom w = -1.0 per the caller) rather than toward the path
is inferred, not measured.

## Verified

- `make PYTHON=<repo .venv>/bin/python verify`: ruff clean, 1566 passed, 1
  skipped, remaining verify targets passed. Plain `make verify` in this
  worktree falls back to system `python3` (no `.venv` in the worktree) and
  7 graph/presence tests fail there; they pass with the repo venv and on a
  direct pytest run, unrelated to this change.
- `flyto-index scan . && flyto-index verify --strict`: every check PASS, exit 0.
  The MCP `task(action='validate')` was not run: this session's MCP server is
  pinned to flyto-indexer's own index, not this repo.
- New tests: every heading in the accepted band drives past the box by robot
  radius (and the old band reached into it); right side mirrors; a goal on
  the open side is faced directly; tolerance and start yaw are honoured;
  `read_heading_tolerance` takes the largest, falls back, refuses >= pi/2;
  the adapter sends the waypoint at the live tolerance and reports it.
- Replay of the recorded twin sweep (`tests/fixtures/twin_box_sweep_2026-10-06.json`):
  toward-goal -0.288 rad, clear bound 0.0, sent heading +0.25 rad, so the
  accepted band is [0, 0.5] rad, never toward the box.

## Not verified

- Not run on the twin (another loop was driving it; no motion was sent) and
  not on the physical robot (`flyto-robot.local` did not resolve; its Nav2
  config was not read live, only the repo's unit file).
- Expected effect, not measured: the goal leg always starts within the band
  on the open side, as in the 16 fast runs (~15-19 s goal leg); the arrival
  turn at the waypoint grows by up to ~0.5 rad (~0.5 s).

## Follow-ups

- Run the twin box series on 0.6.2 and record the goal-leg start yaw and
  `number_of_recoveries` per run.
