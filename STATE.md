# Flyto2 Robotics State

## Current — external adapter / standard ROS 2 robot (2026-09-21)

Production topology:

```text
Flyto2 Cloud / War Room
  -> selected AI Space computer
  -> approved Generic ROS 2 Adapter
  -> standard ROS 2 / DDS
  -> robot resource
  -> observations / execution receipt
  -> Cloud independent verification
```

The robot is commanded equipment, not a Flyto2 worker. A clean TurtleBot3 can
be reinstalled from upstream ROS 2 / ROBOTIS documentation without cloning this
repository or provisioning Flyto2 credentials.

### Source boundary

The current source tree no longer contains the retired Pi appliance runtime:

- no Flyto2 job runner;
- no robot-local delivery gateway;
- no lifecycle installer/profile registry;
- no robot doctor or recovery portal;
- no device-credential provisioning path;
- no Flyto2-specific bringup watchdog;
- no Flyto2-specific robot systemd units.

Historical behavior and evidence remain in dated `handoffs/` and Git history,
not executable current source.

The remaining `deploy/native_ros2/` files are optional standard ROS 2 site
configuration examples only. They invoke upstream TurtleBot3, V4L2 camera,
SLAM Toolbox, and Nav2 packages. They contain no Flyto2 code, credentials,
scheduler, gateway, task state, or evidence database.

### Simulation / physical parity

Simulation and hardware use the same semantic capability and evidence boundary.
`flyto.robotics.ros2-observation-bundle.v1` carries the same fields for both:

- pose;
- range/LiDAR observation;
- camera observation;
- map/TF availability;
- execution binding;
- content-addressed provenance.

Only `deployment_mode` and provider provenance differ. Raw camera observations
are valid evidence without calibration; metric/geometric vision is not ready
unless a calibration snapshot is explicitly bound.

ROS/Nav2 action success remains execution evidence only. It never completes a
Cloud task by itself.

### Physical TurtleBot3 acceptance

The physical lab TurtleBot3 currently has zero Flyto2 runtime. No Flyto2
systemd unit, package, credential/runtime tree, container, scheduler, gateway,
or task/evidence database is installed on the Pi.

Current native stack:

- ROS 2 Jazzy;
- upstream TurtleBot3 bringup;
- LDS-03 LiDAR;
- standard V4L2 USB camera;
- SLAM Toolbox;
- Nav2;
- CycloneDDS (`rmw_cyclonedds_cpp`) on ROS domain 30.

CycloneDDS replaced the previous Fast DDS runtime after a real long-running
failure where graph discovery remained visible while new local subscribers could
not receive odom, LiDAR, or camera data. Restarting the upstream ROS services
restored all three; after switching to CycloneDDS, direct native `rclpy`
subscribers again received all three simultaneously.

Nav2 runs non-composed with respawn enabled. SLAM and navigation are separate
upstream services so one lifecycle failure does not collapse the entire stack.
The standard action surface has been observed live:

- `NavigateToPose`;
- `DriveOnHeading`;
- `BackUp`;
- `Spin`.

A 30-second direct `rclpy` soak observed continuously fresh odometry, LiDAR,
camera frames, and `map -> odom` TF with no new Nav2 heartbeat failure. The
OpenCR has emitted an intermittent `There is no status packet!` read error in
the past; the latest observed occurrence recovered without losing fresh odom.
This remains a hardware/driver reliability fact, not a reason to reinstall a
Flyto2 robot daemon.

The USB camera publishes its native `yuv422_yuy2` image encoding to avoid
unnecessary RGB conversion load on the Pi. The camera currently has no trusted
calibration file, so raw visual evidence is usable but metric vision is not
claimed.

### Still not claimed

No new physical movement is claimed by this source migration. The final bounded
motion / cancel / safe-stop / recovery / Cloud-verification acceptance requires
a physically clear area and must not be replaced by a source, simulation, or
topic-presence check.

The external execution host still needs a ROS-2-capable path to the physical
robot graph for the final real adapter acceptance. Flyto2 code must not be
moved back onto the Pi to satisfy that requirement.
