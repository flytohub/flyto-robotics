# Nav2 lab: velocity smoother in the chain, waypoint follower started

Owner: claude
Branch: claude/nav2-chain-20261012 (PR #73)
Date: 2026-10-12

## What changed

- `launch/nav2_closed_loop.launch.py` starts `velocity_smoother` and
  `waypoint_follower`. `lifecycle_manager_navigation` manages seven nodes in
  this order: controller_server, smoother_server, planner_server,
  behavior_server, velocity_smoother, bt_navigator, waypoint_follower.
  `autostart` stays false; the lab configures all seven, then sends RESUME.
  `NAVIGATION_LIFECYCLE_NODES` in `flyto_robotics/ros2_closed_loop_lab.py`
  is the same list.
- Velocity chain, one producer per stage:
  - controller_server and behavior_server publish `cmd_vel` -> `/nav2/cmd_vel_nav`
  - velocity_smoother reads `/nav2/cmd_vel_nav` and publishes `cmd_vel_smoothed` -> `/nav2/cmd_vel`
  - the safety supervisor reads `/nav2/cmd_vel` and writes `/cmd_vel`. It is
    still the last stage and the only writer to the base. Its parameters,
    stop behaviour and the 0.35 m floor are unchanged.
- `config/nav2_params.yaml` sets `velocity_smoother.velocity_timeout` to
  0.15 s (was 1.0). If the controller goes silent, the smoother keeps
  sending its last command until the timeout. It checks once per 20 Hz
  tick, then ramps down to zero at max_decel 1.0 m/s^2 from the
  supervisor's 0.10 m/s cap. That is 0.15 + 0.05 + 0.10 = 0.30 s, no longer
  than the supervisor's own `command_timeout_seconds` 0.30. After that the
  smoother goes silent, and the supervisor's `command_stale` lands at about
  0.5 s, inside the evidence verdict's 750 ms `bounded_stop_latency`.
- `scripts/run_nav2_closed_loop.sh`, `scripts/run_nav2_stress.sh` and
  `scripts/run_ros2_resilience_series.py` now give the run directory back
  to the host uid/gid after each scenario container (`chown` through the
  same image). They do this before the host reads anything. The closed-loop
  runner also does it from an EXIT trap.
  Why: the containers run as root, and `write_json_atomic` writes mode 0600.
  On the Linux hosted runner the verifier got `Permission denied` on
  `success.json`, and the artifact upload could not zip the directory.
  Docker Desktop on macOS hides this problem.
- Tests: `tests/test_nav2_launch_contracts.py` checks:
  - the node set equals the standard manifest's managed nodes
  - the manager order
  - the chain remappings: only the smoother publishes into the supervisor's
    input, and no Nav2 node writes `/cmd_vel`
  - the deadman/timeout relation
  - that the 750 ms constant matches the verdict

  `tests/test_nav2_runner_ownership.py` checks that the ownership restore
  happens before the first host read in all three runners. Both files were
  committed failing first (207e95e, 21bd8f3).

## Why

The canonical manifest `examples/ros2-adapters/flyto2-standard.json`
(9cda230, 2026-08-27) pairs against `/velocity_smoother` and
`/waypoint_follower`. The lab launch never started them, so the Nav2 step
had failed pairing on main since then. The owner chose to put the smoother
inside the chain. Starting it unconnected was rejected because it would
only fake the pairing check.

## Verified

- PR run 38223996991 (commit 26cd826): pairing passed for the first time.
  All seven nodes were activated with bonds. `success` passed (displacement
  1.26 m). The step then failed on the evidence permission problem
  described above, and the twin step passed.
- PR run 38225219669 (commit bdc5fcb): `simulator-closed-loop` green, both steps:
  - success: `succeeded`, 1.260 m
  - cancel: `canceled`, `operator_cancel`, 0.28 m
  - emergency_stop: `safety_stopped`, `emergency_stop`, 0.25 m
  - twin: 10/10 checks; `failed` / `obstacle_blocked`, travelled 0.379 m,
    nearest 0.42 m ahead, floor 0.35 held, one DriveOnHeading goal, nothing
    after the result, drift 0.007 m
- Locally (single process, repo venv), on the final code:
  - `make verify`: ruff clean, 1724 passed, 1 skipped
  - `flyto-index verify . --full-scan --strict`: exit 0
  - `task validate`: ruff and pytest pass. This needed the project on
    PYTHONPATH, because the CLI shim sets PYTHONSAFEPATH and runs pytest in
    the indexer venv.
- No simulator, Gazebo or Docker was run on the Mac.
- Main runs after the merge: see the PR #73 conversation and the
  `simulator` workflow on main. The first main run is green run #1 of the
  two needed before the owner makes `simulator-closed-loop` required. The
  PR runs do not count toward that.

## Not verified

- `nav2_lifecycle_failure`, `lidar_dropout` and `odometry_freeze` (the
  stress and resilience series) were not run. These are the runs that would
  measure the smoother's effect on `command_stale` latency. The ~0.5 s
  figure is an analysis, not a measurement.
- `run_nav2_stress.sh` and `run_ros2_resilience_series.py` were not run
  with the ownership change. Only their order is pinned by tests.
- `velocity_timeout` 0.15 s is three controller periods. Under heavy CI
  load a late controller cycle makes the smoother ramp toward zero briefly.
  That is safe, but it can slow a run.
- The task gates in `flyto-index task gate` need a task contract the CLI
  does not provide, so they were not run. Exploration used `context`,
  `impact` and `task plan`.

## Follow-ups

- Watch for a second consecutive green `simulator-closed-loop` on main,
  then the owner adds it to branch protection.
- Run the stress series once on a Linux host to measure `command_stale`
  latency with the smoother in the chain.
