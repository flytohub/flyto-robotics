# The robot turns but will not drive

Owner: claude
Branch: main
Date: 2026-08-08

## The symptom, and the thing that made it readable

Six consecutive operator runs, all against a healthy robot:

| Command | `first odometry pose` | Result |
|---|---|---|
| `left` ×4 | +0.004s | completed, every time |
| `forward` | never arrived | failed |
| `backward` | +9.1s | failed |

Every ROS topic and node was up the whole time — `/cmd_vel`, `/scan`, `/odom`,
`/turtlebot3_node`, `/diff_drive_controller`, `/lidar_node` — with
`ROS_DOMAIN_ID=30` exported, which the move script already did.

So the failure was never "the robot is not running". It was **odometry
discovery latency**, and the mission's sensor gate correctly refusing to drive
without it. The turn commands won that race; the drive commands did not.

The one structural difference between them: `forward`/`backward` run a lidar
clearance pre-check first, and `left`/`right` do not.

## What was fixed

### 1. A safety gate that failed open (`168ec99`)

The pre-check parsed the sweep with a bare `except: print('99')`. An unreadable
scan therefore reported **99 m of clearance** and the check waved the robot
through. A measured reading at the time was **0.36 m front, 0.36 m rear** —
well inside the 0.70 m the check exists to require.

"Nothing within 12 m" and "I could not read the sensor" both arrive as an empty
list of usable beams. That is why they were easy to collapse, and why
`flyto_robotics/scan_clearance.py` now returns a distance **or** `None` and
never a number standing in for ignorance. `is_clear()` refuses on `None`.

`scripts/move-robot.sh` replaces the robot-local script and is version
controlled. A test asserts the substitution cannot come back; the guard was
confirmed to fire against the old pattern before being kept.

**The mission controller was checked and does not share this defect.**
`ros2_node._control_tick` already fails with `required_sensor_not_ready` when
`/scan` is absent. The fail-open was only ever on the operator path.

### 2. A type guess that could never be revised (`f1717c4`)

`CmdVelChannel` waits 3s for DDS to discover the driver's subscription, then
falls back to `Twist`. That fallback was returned as **decided**, so it was
cached for the life of the process.

Discovery here was measured at 2.6s–9.1s against that 3s window, so the driver
routinely appears just after the deadline — and a final guess meant its arrival
was never read. Observed directly in one run:

```
+0.1s  cmd_vel bound: type=Twist (auto, provisional)
+2.7s  cmd_vel bound: type=TwistStamped (auto)      ← self-corrected, inside the window
```

A binding made *inside* the window stayed provisional and corrected itself.
Only the one made *outside* it was permanent. Exactly backwards. Publishing
`Twist` at a `TwistStamped` topic matches zero subscribers and **DDS reports no
error at all**, so the whole run looks healthy and the robot ignores every
command.

`/cmd_vel` on this robot is confirmed `geometry_msgs/msg/TwistStamped`,
subscriber `turtlebot3_node`.

### 3. Logic that could not be tested at all

Both fixes are splits, not patches. `ros2_cmd_vel.py` imports `rclpy` and
`geometry_msgs` at module scope, so it could not be imported on any machine
without a ROS install — which is every development machine here. The one piece
of logic whose failure mode is *the robot silently ignores every command* had
**zero tests**.

The decisions now live in `cmd_vel_policy.py` and `scan_clearance.py`, which
import no ROS, matching `mission.evaluate_sensor_gate`. `validated_topic` moved
with them and is covered now too.

Both regression tests were confirmed to **fail against the previous behaviour**
before being kept. A test that has never been red is not evidence.

## The probe hypothesis was wrong

An earlier revision of this handoff suspected the clearance probe of delaying
discovery: the correlation was perfect across the first six runs and the
mechanism was plausible. **Twelve alternating runs on the robot refuted it**,
with a fixed 5s quiet gap before each mission so the previous mission's
teardown was not the variable:

| arm | n | median | max | over 1s |
|---|---|---|---|---|
| no probe | 6 | 2.108s | 2.598s | 4 |
| probe first | 6 | 2.067s | 2.471s | 5 |

No difference. The 2s settle pause has been removed from
`scripts/move-robot.sh`; the comment there now records why, so nobody adds it
back on the same reasoning.

## What is actually going on, with the arithmetic

**Odometry discovery latency is intrinsic and wildly variable** — 0.007s to
2.598s across those twelve runs, and 9.1s in the original failing `backward`.
Nothing in front of it changes the number.

The budget it runs against is tighter than it looks. `ros2_node` declares
`sensor_startup_grace_seconds = 10.0` and `sensor_stabilization_seconds = 1.0`,
and `evaluate_sensor_gate` requires the sensors to stay fresh for the
stabilization window *before* the grace expires. So the real budget for
discovery is **9.0s, not 10s**.

Odometry arrived at **9.1s** in the `backward` run. It missed by 0.1 second.
The `forward` run never saw odometry at all inside the window. Both then failed
`required_sensor_not_ready` — correctly. The gate was doing its job on a robot
whose discovery was late.

That failure said only `failed  x=None y=None yaw=None`, which is why this took
a day to read. It now names the sensor and how late it was
(`mission.unready_sensors`, tested). The result contract keeps the same reason
code; only the operator-facing log gained the detail.

**Not yet decided: whether 10.0s is the right grace.** Discovery has been
observed at 9.1s, so the current default fails roughly whenever the tail is
hit. Raising it is safe in the sense that the node publishes a stop and refuses
to move for the whole waiting period — it only extends patience, never
permission. It has been left alone because it is a safety parameter and the
choice is the owner's, not something to change while diagnosing something else.

## Verified on the robot

Both refusal paths were run on the real machine after the fix was deployed:

```
clearance front: 0.21 m
REFUSED: needs 0.70 m to move 0.40 m safely.                      exit 3

clearance front: unreadable (no usable lidar return)
REFUSED: cannot see front. No usable return on /no_such_scan in 20s,
         so there is no clearance to check.                        exit 4
```

The second is the case that previously reported 99 m and drove. The robot did
not move in either.

`/cmd_vel` binding was observed settling on `TwistStamped` in every one of the
twelve runs, with no `no subscriber` warning — the type resolution is behaving.

## Where the robot is

Measured with the new module, all four sectors:

| front | left | rear | right |
|---|---|---|---|
| 0.21 m | 0.54 m | 0.41 m | 1.32 m |

**It is in a corner.** 0.21 m is inside the controller's own stop distance, so
even a turn in place trips a safety stop — every one of the twelve runs above
ended `failed` with `safety_stops=1`, which is the safety system working, not
a defect. Nothing will drive until it is physically moved; right is the only
open side.

## What the next person must do

1. **Move the robot into open space.** Every motion test is meaningless until
   then, and a `failed` result there means "correctly refused", not "broken".
2. **Decide on `sensor_startup_grace_seconds`.** See the arithmetic above: the
   effective budget is 9.0s and discovery has been seen at 9.1s.
3. `make verify` passes — ruff clean, 435 passed, 3 skipped.

## Two notes on the tooling

`impact` reported **0 references** for `ros2_cmd_vel.py`. Three modules import
it. File-level symbols carry no references in the index; use grep to confirm a
blast radius before trusting a zero.

`make verify` had never run past its first target on this machine: `ruff`'s
wrapper was installed without its binary, `lint` is first in
`verify: lint test assets ...`, and a chained target list stops there. Nine
checks had been silently skipped. Fixed by linking the binary where the finder
looks. **A green-looking gate that exits early is worse than a red one.**
