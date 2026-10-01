# TurtleBot3 digital twin

A Docker container that presents the same ROS 2 interface as the physical
TurtleBot3 (`flyto-robot.local`), so the Generic ROS 2 Adapter and anything
above it (AI Space, War Room) can be developed without the robot.

The rule for this directory: **copy from the robot, do not invent.** Service
arguments, the SLAM parameters and the scan stabilizer are taken verbatim from
the robot's systemd units and files. Only the hardware bringup is replaced by
Gazebo.

## Run

```bash
cd sim/twin
docker compose up --build
```

The first build downloads ROS 2 Jazzy, Gazebo Harmonic and Nav2, which takes
several minutes. Later starts take about a minute.

| What | Where |
| --- | --- |
| Gazebo window (3D view) | <http://localhost:6080/vnc.html> |
| rosbridge (adapter, Foxglove) | `ws://127.0.0.1:19090` |
| Logs per service | `docker exec flyto-turtlebot3-twin ls /var/log/twin` |

Stop with `docker compose down`.

## Connecting

The robot is reached through `ssh -L 19090:127.0.0.1:9090 ubuntu@flyto-robot.local`.
The twin publishes the same endpoint on the host, so a client written for the
robot connects unchanged. Set `FLYTO_ROS2_DEPLOYMENT_MODE=simulation` so that
evidence is labelled as simulation and never as hardware.

Foxglove: open a Rosbridge connection to `ws://127.0.0.1:19090` to see the
scan, the SLAM map, TF and the Nav2 plans. The same connection works against
the real robot through its tunnel.

## Same as the robot

| Robot | Twin |
| --- | --- |
| ROS 2 Jazzy, `TURTLEBOT3_MODEL=burger`, `LDS_MODEL=LDS-03` | same |
| `ROS_DOMAIN_ID=30`, `rmw_cyclonedds_cpp` | same |
| `laser-scan-stabilizer` publishing `/scan_stable` (400 bins) | same script |
| `slam_toolbox online_async_launch.py` with `/etc/ros/slam_toolbox.yaml` | same file and arguments, plus `use_sim_time` |
| `nav2_bringup bringup_launch.py` with `turtlebot3_navigation2` `burger.yaml` | same arguments, plus `use_sim_time` |
| `rosbridge_websocket_launch.xml`, port 9090, local-only | same, published on host `127.0.0.1` |
| Five capabilities through Nav2 (`navigate`, `advance`, `retreat`, `rotate`, `halt`) | same actions |

## Different from the robot

- **No camera.** The Gazebo burger model has none; the robot has a USB camera on
  `/camera/image_raw`. Adapter motion preflight does not need it.
- **LiDAR model.** Gazebo simulates an ideal scanner (TurtleBot3 LDS model:
  360 samples, 0.12–3.5 m). The real LDS-03 reports `range_max` 100 m and has
  noise and dropouts.
- **No hardware faults.** No OpenCR resets, wheel slip, odometry drift, boot
  races or the mounting error found on 2026-10-02 (LiDAR rotated 90°).
- **World.** `turtlebot3_world` (pillars) by default; choose another with
  `TWIN_WORLD_LAUNCH=<launch file in turtlebot3_gazebo>`.

The twin proves the software path. Physical acceptance still happens on the
robot.
