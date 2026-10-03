# TurtleBot3 digital twin

A Docker container that presents the same ROS 2 interface as the physical
TurtleBot3 (`flyto-robot.local`), so the Generic ROS 2 Adapter and anything
above it (AI Space, War Room) can be developed without the robot.

The rule for this directory: **copy from the robot, do not invent.** Service
arguments, the SLAM parameters, the scan stabilizer, the robot description and
every driver node's parameters are taken verbatim from the robot. Only the
hardware is replaced: Gazebo simulates it, and `scripts/twin_hardware.py`
stands in for the robot's driver nodes. Anything that is an estimate rather
than a measurement says so below.

## Run

```bash
cd sim/twin
docker compose up -d --build
```

The first build downloads ROS 2 Jazzy, Gazebo Harmonic and Nav2, which takes
several minutes. Later starts take about a minute.

| What | Where |
| --- | --- |
| Gazebo window (3D view) | <http://localhost:6080/vnc.html> |
| rosbridge (adapter, Foxglove) | `ws://127.0.0.1:19090` |
| Logs per service | `docker exec flyto-turtlebot3-twin ls /var/log/twin` |

Stop with `docker compose down`. Choose another world or start pose with
`TWIN_WORLD` (a world file inside the container), `TWIN_X`, `TWIN_Y`,
`TWIN_YAW`.

## Connecting, and never confusing the twin with the robot

The robot is reached through `ssh -L 19090:127.0.0.1:9090 ubuntu@flyto-robot.local`.
The twin publishes the same endpoint on the host, so a client written for the
robot connects unchanged. Only one of them can hold port 19090 at a time.

Because the endpoint is the same, the adapter checks what it is talking to
before every motion (`motion.halt` is never blocked):

| `FLYTO_ROS2_DEPLOYMENT_MODE` | Graph publishes `/clock` | Result |
| --- | --- | --- |
| `simulation` | yes (twin) | allowed |
| `hardware` (default) | no (robot) | allowed |
| `hardware` | yes | refused: "the ROS graph publishes /clock (a simulator)…" |
| `simulation` | no | refused: "…it may be physical hardware" |

So set `FLYTO_ROS2_DEPLOYMENT_MODE=simulation` for the twin. Evidence is then
labelled simulation, and a twin configuration pointed at the real robot by
mistake cannot move it. `FLYTO_ROS2_SIM_MARKER_TOPIC=` (empty) disables the
check for a robot that legitimately publishes `/clock`.

Foxglove: open a Rosbridge connection to `ws://127.0.0.1:19090` to see the
scan, the SLAM map, TF, the camera and the Nav2 plans. The same connection
works against the real robot through its tunnel.

## Same as the robot

`tools/compare_fingerprint.py` checks this against `config/real/interface-fingerprint.json`
(captured on the robot on 2026-10-02); on 2026-10-02 it reported no
differences beyond those listed in the next section.

| | Robot | Twin |
| --- | --- | --- |
| ROS 2 | Jazzy, `ROS_DOMAIN_ID=30`, `rmw_cyclonedds_cpp`, `TURTLEBOT3_MODEL=burger`, `LDS_MODEL=LDS-03` | same |
| Packages | turtlebot3 2.3.6, turtlebot3_msgs 2.4.0, slam_toolbox 2.8.5, rosbridge_server 2.7.1 | same |
| Nodes | `turtlebot3_node`, `diff_drive_controller`, `lidar_node`, `/camera/v4l2_camera`, `robot_state_publisher`, `laser_scan_stabilizer`, `slam_toolbox`, Nav2, rosbridge | same names, same parameters (`config/real/*.params.yaml`) |
| Topics, types, QoS | 138 topics | same set, plus `/clock` |
| Actions / services | 34 actions; `/motor_power`, `/reset`, `/reset_odometry`, `/sound`, `set_camera_info` | same |
| TF | `odom -> base_footprint -> base_link -> {base_scan, imu_link, wheels, caster}` | same frame names |
| `robot_description` | xacro of `turtlebot3_burger.urdf` | the robot's own text (`config/real/robot_description.urdf`) |
| `/cmd_vel` | `TwistStamped`, consumed by `turtlebot3_node`; limits 0.22 m/s, 2.84 rad/s; wheels stop 0.5 s after the last command | same; `/motor_power false` stops and ignores commands |
| `/odom`, `/imu`, `/joint_states`, `/battery_state`, `/sensor_state`, `/magnetic_field` | ~20 Hz, zero covariances, `imu_link` / `base_link` frames | same (18–20 Hz, see below) |
| `/scan` (LDS-03, coin_d4) | BEST_EFFORT, ~10 Hz, 399–401 readings over 0..2π, `angle_increment = 2π/(n+1)`, range 0.1–100 m, no-return = `0.0` (~15 per turn, intensity 66–97), ~0.9 NaN per turn, valid intensity 219–247, static noise median 3.4 mm / p90 17 mm | emulated to the same figures |
| Camera | `/camera/image_raw` 640×480 `yuv422_yuy2` 30 Hz, `camera_info` all-zero K/R/P (uncalibrated), `compressed` transport | same |
| Downstream | `laser-scan-stabilizer` → `/scan_stable` (400 bins), SLAM Toolbox with `/etc/ros/slam_toolbox.yaml`, Nav2 with `turtlebot3_navigation2` `burger.yaml`, rosbridge on 9090 | same files and arguments, plus `use_sim_time` |

## Different from the robot

Interface:

- **`/clock`** exists only on the twin (simulation time; the adapter's marker).
- **Camera transports `theora` and `zstd`** are advertised but publish nothing.
- **nav2_bringup** is 1.3.13 in the twin (1.3.12 on the robot): the apt
  repository serves only the newest build.
- **Camera placement and lens are partly estimated.** Height 12.5 cm and
  lateral centre were measured by the owner; the forward position (front edge
  of the chassis) and the 60° horizontal field of view are estimates. The
  camera renders at 10 Hz and is republished at 30 Hz to keep Gazebo near real
  time on a CPU-only renderer.
- **Battery** holds 12.0 V (83 %); the robot drains.
- **`/reset_odometry`** re-zeroes odometry in the twin; the IMU is not reset.
- **Nav2 timing slack.** Nav2 reads `/etc/ros/twin-burger.yaml`, written at
  start by `scripts/twin_nav2_params.py` from the robot's `burger.yaml` with
  two timing keys changed: the collision monitor's scan `source_timeout`
  0.2 s -> 0.5 s, and bt_navigator's `default_server_timeout` 20 ms -> 200 ms.
  Gazebo renders the LiDAR on the CPU at a real-time factor near 0.7, so under
  load a scan reached the collision monitor 0.20-0.21 s old in simulation time
  and it stopped the base for an "invalid source"; bt_navigator also aborted
  navigation waiting 20 ms for follow_path to acknowledge. The script refuses
  any other key (no polygon, speed, footprint or clearance) and caps each
  value. The adapter's 0.35 m clearance floor is not a Nav2 parameter.

Behaviour, same adapter scripts on both (2026-10-02):

| | Robot | Twin |
| --- | --- | --- |
| `motion.advance` 0.10 m at 0.05 m/s | 0.119 m (+19 %), 3.25 s | 0.104 m (+4 %), 2.52 s |
| 0.30 m advance cancelled after 1.5 s | stopped at 0.159 m; cancel confirmed after ~2.8 s; ~4 cm after the cancel | stopped at 0.066 m; confirmed after 0.1 s; 0.2 cm |
| Heading drift while still | ~0.6° over a few minutes | none |
| Rates | odom 20.8, imu 20.1, scan 10.1 Hz | 18, 18, 9.1 Hz (real-time factor 0.82–0.99) |

The robot's extra latency comes from the Raspberry Pi (rosbridge and Nav2 on
four cores) and its motors; the twin does not reproduce it. Budget for the
robot's numbers in anything that must hold physically. Also absent: OpenCR
resets, wheel slip, boot races, and mounting errors such as the LiDAR found
rotated 90° on 2026-10-02.

**World.** `turtlebot3_world` (pillars) by default, not the robot's room.

The twin proves the software path. Physical acceptance still happens on the
robot.

## Files

| Path | What |
| --- | --- |
| `models/flyto_burger/` | Upstream burger model with the robot's sensor rates, ranges and camera; Gazebo topics under `twin/` |
| `launch/twin.launch.py` | Gazebo, spawn, `robot_state_publisher` (frame_prefix `''`) |
| `scripts/twin_hardware.py` | The four driver stand-ins |
| `scripts/twin_nav2_params.py` | The robot's Nav2 `burger.yaml` plus the twin's timing slack, nothing else |
| `config/real/` | Captured from the robot: parameters, `robot_description`, interface fingerprint (the comparison baseline) |
| `tools/fingerprint.py`, `tools/compare_fingerprint.py` | Capture an interface fingerprint (read-only) and compare |
| `tools/bounded_advance.py` | One bounded advance through the adapter, robot or twin |
