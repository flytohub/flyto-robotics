# Simulator closed loops run on hosted runners, non-blocking

Owner: claude
Branch: claude/sim-ci-20261012
Date: 2026-10-12

## What changed

- `.github/workflows/simulator.yml`: workflow `simulator`, one job
  **`simulator-closed-loop`** on `ubuntu-24.04` (GitHub-hosted). Triggers:
  push to `main`, nightly 18:17 UTC (02:17 Asia/Taipei), `workflow_dispatch`,
  and `pull_request` limited to simulator paths. `timeout-minutes: 90`.
  Uploads `results/nav2-closed-loop/<run>` and `results/twin-scenario/<run>`
  (evidence, verdict, twin service logs) as artifact
  `simulator-closed-loop-<run>`; writes a job summary with every check.
- Step 1 runs the existing `scripts/run_nav2_closed_loop.sh` (Gazebo + Nav2 +
  safety supervisor in `docker/Dockerfile.jazzy`; `success`, `cancel`,
  `emergency_stop`, each judged by `verify-ros2-execution-evidence`).
- Step 2 runs `scripts/run_twin_scenario.sh
  scenarios/twin/demo-room-obstacle-stop.json`: the existing TurtleBot3 twin
  (`sim/twin`, compose) in `flyto_demo_room.world` at (-1.8, 0), box 0.75 m
  ahead; `sim/twin/tools/run_scenario.py` sends `motion.advance` 1.2 m at
  0.1 m/s through the Generic ROS 2 Adapter over rosbridge, recording every
  outgoing goal and velocity command; `sim/twin/tools/scenario_check.py`
  judges it. The scenario is declarative (`flyto.robotics.twin-scenario.v1`).
- What step 2 asserts (all must hold): outcome `failed`;
  `motion_outcome.reason == obstacle_blocked`; the adapter applied the
  0.35 m floor (`clearance_floor_m` and `stop_clearance.floor_m`);
  `floor_held` and `travel_floor_held` true; travelled 0.10-0.45 m; the only
  goal sent was one `nav2_msgs/action/DriveOnHeading` (no NavigateToPose,
  NavigateThroughPoses, BackUp or Spin: no automatic detour, no recovery);
  no goal and no non-zero velocity command after the result; odometry within
  0.02 m for 10 s after it.
- The checker refuses a scenario whose floor is not exactly 0.35 m or that
  allows a detour/recovery action; the runner removes
  `FLYTO_ROS2_MIN_CLEARANCE_M` and forces `FLYTO_ROS2_DEPLOYMENT_MODE=simulation`.
- `launch/nav2_closed_loop.launch.py`: `lifecycle_manager_navigation`
  `autostart` true -> false; `tests/test_nav2_launch_contracts.py` pins it.
- `tests/test_twin_scenarios.py` (20 tests).

## Why

The owner wants simulator tests as a required check, but only after the job
is green twice in a row on `main`. Until then it runs and reports, and is
**not** in branch protection (main had no protection and no rulesets at the
time). The owner flips it to required.

autostart: first hosted run, the manager activated controller_server while
the lab (started at 15 s) read the nodes; the lab, which configures nodes
itself and then sends RESUME, failed closed on "partial activation:
/controller_server". Two owners raced over one lifecycle. With autostart off
the second run showed RESUME activating all five nodes with bonds.

## Verified

- PR run 1 (38209683669) and run 2 (38210463914) on hosted runners: the
  twin step passed every check both times. Run 1 numbers: `failed`,
  `obstacle_blocked`, travelled 0.385 of 1.2 m, nearest return 0.411 m
  ahead, floor 0.350 held, one DriveOnHeading goal, nothing after.
- `make verify` (single process, repo venv): ruff clean, 1675 passed,
  1 skipped. actionlint 1.7.7 clean. `flyto-index verify . --full-scan
  --strict` pass; `flyto-index task validate` pass.
- No simulator or Docker was run on the Mac.

## Not verified

- **The Nav2 closed-loop step is red.** After the autostart fix, the lab
  fails with "ROS 2 pairing is not ready:
  adapter.ros2.nav2.navigate_to_pose.v1.ready, ...lifecycle_active". The
  canonical manifest `examples/ros2-adapters/flyto2-standard.json` (since
  9cda230, 2026-08-27, locked by af6f86b) requires `/velocity_smoother` and
  `/waypoint_follower` to be active; `nav2_closed_loop.launch.py` has never
  started either. So the loop could not have passed on main since
  2026-08-27. Not fixed here: putting velocity_smoother into the command
  chain (controller -> smoother -> supervisor) changes the safety path (my
  reading, not tested: with `velocity_timeout: 1.0` it may keep publishing
  after a controller failure, which the `nav2_lifecycle_failure` stress fault
  expects the supervisor to catch as `command_stale` within 750 ms); launching it off
  the chain would only fake the pairing. That is an owner decision.
- Status on main at merge time: the job therefore starts red on main
  because of the Nav2 step; the twin obstacle-stop step is expected green.
  The required-check clock (two consecutive green runs on main) cannot start
  until the Nav2 step is fixed.
- The twin demo world includes Ground Plane and Sun from Gazebo Fuel (online)
  at runtime; hosted runners have network, so it works, but it is not
  self-contained as AGENTS.md asks.
- `run_nav2_stress.sh` / resilience series use the same launch; they were
  not run with autostart off.

## Follow-ups

- Decide how the Nav2 lab satisfies the standard profile's eight managed
  nodes (velocity_smoother in the chain with a timeout no longer than the
  supervisor's, plus waypoint_follower), then watch for two green runs on
  main and add `simulator-closed-loop` to branch protection.
- Optionally vendor Ground Plane and Sun into `sim/twin/models`.
