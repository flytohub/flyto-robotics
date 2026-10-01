Owner: claude
Branch: claude/sim-twin
Date: 2026-10-02

# TurtleBot3 twin presents the robot's own ROS interface

## What changed

`sim/twin/` started on this branch as upstream `turtlebot3_gazebo` plus the
robot's SLAM/Nav2/rosbridge arguments (`01bc8c9`, by an earlier Claude session
that handed the branch over). Two commits on top:

- `572269b` — the twin now presents the robot's driver interface:
  - `models/flyto_burger/model.sdf`: upstream burger with the robot's sensor
    rates and ranges, a camera (12.5 cm above the floor, centred — measured by
    the owner; forward position and 60° FOV estimated), Gazebo topics under
    `twin/`, and the upstream `odom_publisher_frequency` typo fixed
    (`odom_publish_frequency`).
  - `scripts/twin_hardware.py`: `turtlebot3_node`, `diff_drive_controller`,
    `lidar_node`, `/camera/v4l2_camera` with the robot's names, topics, QoS,
    message fields and parameters (`config/real/*.params.yaml`, dumped from the
    robot). Reads Gazebo through `gz.transport13`; no `ros_gz_bridge`.
  - `launch/twin.launch.py`: Gazebo, spawn, and `robot_state_publisher` with
    the robot's own `robot_description` and `frame_prefix ''`.
  - `tools/fingerprint.py` / `tools/compare_fingerprint.py` and the robot's
    fingerprint in `config/real/interface-fingerprint.json`.
  - Entrypoint clears a stale Xvfb lock on restart; the Gazebo window exiting
    no longer shuts the simulation down.
  - `bounded_advance.py` now calls `reconnect()` (it could never discover).
  - `pyproject.toml`: ruff excludes the stabilizer, which is a byte-for-byte
    copy of the robot's file.
- `3c4722f` — Generic ROS 2 Adapter refuses motion when
  `FLYTO_ROS2_DEPLOYMENT_MODE` disagrees with the graph (`/clock` present means
  simulator). `motion.halt` is exempt. `FLYTO_ROS2_SIM_MARKER_TOPIC=""`
  disables it.

## Why

The owner wants Flyto2 Cloud work to run against the twin while the robot is
at home, with the twin indistinguishable from the robot at the ROS interface.
Robot and twin share `ws://127.0.0.1:19090`, so the deployment-mode label alone
could let a twin configuration drive the real robot.

Rejected: keeping `ros_gz_bridge` and renaming topics (adds bridge nodes and
relay topics the robot does not have); running the twin on wall time without
`/clock` (Gazebo drops below real time under CPU rendering, and `/clock` is the
only reliable simulator marker).

## Verified

- Interface: `compare_fingerprint.py config/real/interface-fingerprint.json <twin>`
  → `0 unexpected difference(s)`; expected ones are `/clock` and the empty
  `theora`/`zstd` camera transports. Same 138 topics (+`/clock`), node names,
  34 actions, services, TF frame names.
- Unloaded rates (robot → twin): odom 20.8 → 18.0, imu 20.1 → 18.0,
  joint_states/battery/sensor_state/magnetic 20 → 20, scan 10.1 → 9.1,
  camera 29.8 → 30.0 Hz. Gazebo real-time factor 0.82–0.99 steady.
- Same adapter scripts on both (robot numbers from
  `2026-10-02-first-physical-motion.md`): advance 0.10 m → robot 0.119 m /
  3.25 s, twin 0.104 m / 2.52 s; 0.30 m advance cancelled at 1.5 s → robot
  0.159 m, cancel confirmed ~2.8 s; twin 0.066 m, 0.1 s.
- Deployment-mode check live on the twin: hardware mode refused,
  `execution_count=0`, 0.000 m moved; simulation mode completed (0.059 m).
- `make verify` (lint, 961 tests, assets, dry-runs, contracts): pass.
- `flyto-index verify --strict`: all pass except `env_contract`
  (`FLYTO_ROBOTICS_PARAMETER_BRIDGE` read in `bridge_guard.py`, absent from
  `.env.example`) — identical on the main checkout, not from this branch.
- `flyto-index task validate`: ruff pass; its pytest step could not import
  `flyto_robotics` from the worktree (validator environment), while the same
  tests pass under `make verify` and directly (19 in the adapter file).

## Not verified

- The simulation-mode-on-real-robot refusal was not run live (robot powered
  down); it is covered by a unit test.
- The twin does not reproduce the robot's latency and dynamics: +19 % advance
  overshoot, ~2.8 s cancel confirmation, IMU heading drift (~0.6° over minutes).
  Anything that must hold physically still needs the robot's numbers.
- Camera image content: forward position and FOV are estimates; `theora` and
  `zstd` transports publish nothing.
- nav2_bringup is 1.3.13 in the twin, 1.3.12 on the robot (apt serves only the
  newest).
- Flyto2 Cloud / War Room against the twin was not run.

## Follow-ups

1. Run the Cloud loop against the twin (local AI Space host with
   `FLYTO_ROS2_TRANSPORT=rosbridge`, `FLYTO_ROSBRIDGE_URL=ws://127.0.0.1:19090`,
   `FLYTO_ROS2_DEPLOYMENT_MODE=simulation`, a twin-specific
   `FLYTO_ROS2_RESOURCE_ID`).
2. The flyto-cloud local `.venv` has `flyto-robotics` installed editable from
   the main checkout, which does not yet contain `3c4722f`. Reinstall from this
   branch (or merge it) before relying on the deployment-mode check from Cloud.
3. If demos need the twin to behave like the robot physically, add latency and
   overshoot deliberately, calibrated against the numbers above.
4. Build the demo world (`TWIN_WORLD`) for the 2026-10-14 review.
