# TurtleBot3 digital twin in Docker

Owner: claude
Branch: claude/sim-twin
Date: 2026-10-02

## What changed

- `sim/twin/`: a Docker container that presents the physical robot's ROS 2
  interface with Gazebo in place of the hardware bringup.
  - `Dockerfile`, `compose.yaml`, `scripts/entrypoint.sh`.
  - `config/slam_toolbox.yaml` and `scripts/ros2-laserscan-stabilizer.py`,
    copied verbatim from `flyto-robot.local`.
  - `tools/bounded_advance.py`, the same adapter script used on the robot.
  - `README.md` with the parity table and the known differences.
- Every service starts with the robot's own systemd `ExecStart` arguments, plus
  `use_sim_time`: the stabilizer, `slam_toolbox`, Nav2 with
  `turtlebot3_navigation2` `burger.yaml`, and rosbridge on 9090. Also
  `ROS_DOMAIN_ID=30`, CycloneDDS, `burger` and `LDS-03`.
- rosbridge is published on the host's `127.0.0.1:19090`, the same local
  endpoint the robot's SSH tunnel uses. Gazebo's window is served at
  `http://localhost:6080/vnc.html`.

## Why

Chester works away from the robot during the day. The Generic ROS 2 Adapter
and the Cloud path above it need a graph to talk to that behaves like the
robot's, rather than the older Gazebo lab, which assumed the retired on-robot
runtime and a static map-to-odom transform.

## Verified

- Image builds on Apple Silicon (arm64). The container starts all services, in
  the robot's order.
- Inside the container:
  - topics `/scan`, `/scan_stable`, `/odom`, `/map`, `/tf`, `/tf_static`,
    `/cmd_vel` and `/clock` are present;
  - actions `/navigate_to_pose`, `/drive_on_heading`, `/backup` and `/spin` are
    present;
  - `bt_navigator` and `behavior_server` are active.
- `tools/bounded_advance.py`, unchanged from the robot run, with
  `FLYTO_ROS2_DEPLOYMENT_MODE=simulation`:
  - discovers the same 5 capabilities;
  - preflight clearance is 0.489 m;
  - `motion.advance` of 0.15 m at 0.05 m/s completed with odometry +0.154 m;
  - the safe stop held.
- Gazebo's view renders in the browser through noVNC.

## Not verified

- The Cloud / AI Space path against the twin. Production was down at the time
  (flyto-cloud #412).
- Behaviour after long runs. Software rendering on the Mac's Docker VM is slow
  (real-time factor about 0.9).

## Follow-ups

- Add the robot's USB camera to the simulated model, so `/camera/image_raw`
  exists in the twin too.
- Build a Gazebo world from the real robot's SLAM map, so the twin's room
  matches the real one.
