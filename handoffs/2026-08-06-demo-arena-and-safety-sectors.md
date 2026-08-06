# Demo arena, directional safety, and what the robot still cannot do

Date: 2026-08-06
Related: `flyto-cloud/handoffs/2026-08-05-space-task-closed-loop.md`

## What changed

**The obstacle guard judges the direction of travel, with a floor under it.**
A single omnidirectional threshold cannot enter a corridor narrower than twice
its distance — the side walls trip it forever. Measured through the real
`MissionController`: at `obstacle_stop_distance: 0.25`, aisles of 25, 30 and
40 cm all returned zero linear velocity, and 50 cm was the first that moved.
The demo arena specifies 25–30 cm.

The guard now reads sectors by intent — forward when driving, behind when
reversing, every side while rotating, because a rotation sweeps the whole
footprint — with an omnidirectional `emergency_stop_distance` (0.08 m default,
a contract field) underneath, so relaxing the sides for a narrow aisle never
becomes permission to graze one.

`RangeField` defaults every sector to infinity: absent means nothing was
measured there, never that nothing is there. A caller passing one number keeps
the old omnidirectional rule.

**`worlds/mission-arena.sdf`** — 48 cm zones, 28 cm aisles, 124 cm frame.
Solid blocks, not floor markings: the lidar has to see them or the aisles are
not aisles.

**`deploy/flyto_job_runner.py`** — claims Flyto2 jobs for this robot in the
standard library alone. The cloud's own `connected_runner.py` reaches 379
modules and 21 third-party packages because it is the whole backend minus the
web server.

**Boot recovery fixed.** `turtlebot3-bringup.service` waited only on the
network, so at boot it opened serial ports that had not enumerated and exited —
cleanly, so `Restart=on-failure` never fired, leaving a unit systemd called
active with no processes and no log. It now waits for both devices and restarts
always. Verified by rebooting: four nodes came back unattended.

## Not done

### The corridor answer is from synthetic sectors

Hand-computed values were fed into the real guard. **A robot has never driven
through the arena.** An aisle intersection has four corners, and a robot
crossing the middle sees them diagonally at distances shorter than the side
walls — which the synthetic test does not model. The world loads headless
(Gazebo 8.11.0, 200 iterations, exit 0); the rover has not been spawned in it.

### No shipped job sets a lateral distance

The guard supports `lateral_stop_distance`, and every job in `examples/jobs/`
omits it, so every one still gets the omnidirectional rule. The arena job needs
`0.10`. Deliberately not applied to the delivery jobs, whose corridors are real
ones.

### The job runner has never run on the Pi

Logic is covered by 15 tests against a fake device API and a fake gateway, both
real loopback servers. Pairing needs a one-time code from the operations room,
and the Mac was on a different subnet. Nothing about it is proven on hardware.

### This robot has no camera

`ros2 node list` on the TurtleBot3 Burger returns four nodes —
`turtlebot3_node`, `lidar_node`, `diff_drive_controller`,
`robot_state_publisher`. There is no camera. So the arena's close-up image,
QR/AprilTag reading and `vision.read_code` evidence cannot be produced by this
robot as it stands. A USB UVC camera and a capture path are needed; neither
exists yet.

What it *can* report today, verified over eight live runs: pose, full lidar
sweep and nearest range, battery, and per-step events with timestamps and
attribution. Blocked passage is therefore already answerable, and by
measurement rather than by picture.

### One unexplained observation

On the first run after a cold start the yaw drifted about 48° during a straight
reverse, then held within 1° over four further moves. That run was the only one
that hit the provisional `cmd_vel` binding window, which is a plausible cause
and is not established. If it recurs, capture `/odom` and `/cmd_vel` together
from the first command.

## Measurements worth keeping

TurtleBot3 Burger, LiPo 11.9–12.2 V, ROS 2 Jazzy, `ROS_DOMAIN_ID=30`:

| Direction | Target | Measured |
|---|---|---|
| backward | -0.400 m | -0.372 / -0.371 / -0.371 |
| forward | +0.400 m | +0.371 / +0.370 / +0.372 |

Consistently 28–30 mm short: the controller converges inside the job's 0.05 m
`pose_tolerance` and stops. Zero safety stops across every run. Tighten
`pose_tolerance` to 0.02 if the arena needs the accuracy.
