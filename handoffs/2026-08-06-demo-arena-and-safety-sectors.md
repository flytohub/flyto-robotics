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

### The arena run now completes, and the aisle is passable

A TurtleBot3-sized robot drives a 28 cm aisle in the arena, in a fresh
simulation per job:

| Job | Result | Moved | Safety stops | Reason |
|---|---|---|---|---|
| `tb3-lab-shortcut` (omnidirectional 0.25) | failed | 0.000 m | 1 | obstacle alongside |
| `mission-arena` (forward 0.25 / side 0.10 / emergency 0.08) | succeeded | 0.372 m | 0 | — |

Two earlier attempts at this were wrong and are worth remembering. The first
compared two jobs run back to back in one simulation, where the second starts
where the first stopped — not a comparison at all. The second used a 42 cm
rover in a 28 cm aisle, where success means only that the readings cleared the
gate.

The third attempt worked because it stopped guessing: park the robot in the
aisle, run no mission, and read the sectors. That took one run and gave the
answer.

### The forward cone was wider than the aisle

Parked mid-aisle, the sides read 0.120 and 0.125 m — the aisle half-width, as
expected — but forward read 0.452 m where the aisle ahead was clear for 0.92 m.
0.452 m is the distance to the corner of the zone diagonally beyond the
intersection. A ±30° cone spans ±0.26 m at a quarter of a metre out, which is
wider than the aisle, so the robot was reading corners it would pass and
stopping for them.

The cone now covers the width the robot actually sweeps and no more: ±15°,
which is ±0.067 m at the 0.25 m stop distance, a Burger's half-width. Narrower
would let something graze the corner of the chassis unseen.

This is the second thing simulation caught that the unit tests could not, the
first being that no node ever built a `RangeField`. Both were geometry and
wiring rather than logic, which is exactly what a synthetic test cannot reach.

What the runs also established:

* The guard was unreachable. Both nodes passed a scalar, so every tick took
  the omnidirectional branch. The stop reason read "range below configured stop
  distance" — the omnidirectional wording. Fixed; `sector_field` now feeds both.
* Robot width matters and was wrong. `flyto_rover` is 42 cm and cannot fit a
  28 cm aisle, so its "successful" run proved nothing. `flyto_tb3_burger` is
  13.8 cm, with the LDS-03's 0.12 m minimum range and gaussian noise.
* Two jobs run sequentially in one simulation are not comparable — the second
  starts where the first stopped. An earlier pass-versus-fail comparison was an
  artefact of exactly that and was retracted.

An aisle intersection also has four corners, seen diagonally at shorter
distances than the side walls: measured at 0.120 m against 0.174 m to the walls,
with the 42 cm rover. That gap is real but the number needs remeasuring with
the Burger.

### Simulation is not the robot

Same `MissionController`, same guard, same `ros2_node.py`, same contracts — so a
logic fault found in one is a fault in the other, which is how the unreachable
guard surfaced. Not the same machine:

| | Simulation | Robot |
|---|---|---|
| `cmd_vel` type | `Twist` | `TwistStamped` |
| Lidar | 360 samples, modelled noise | LDS-03, 399 reported |
| Camera | present on the model | **none on this Burger** |
| Traction, wheel slip, battery sag | absent | the source of the 28–30 mm shortfall |

The `TwistStamped` difference matters most: the silent binding bug that cost
hours on hardware cannot reproduce in simulation, because `Twist` is correct
there.

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
