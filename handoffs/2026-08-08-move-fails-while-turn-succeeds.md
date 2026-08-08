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

## What is suspected and NOT proven

**That the clearance probe is what delays discovery.** The correlation is
perfect across six runs, and the mechanism is plausible — the probe joins and
leaves the DDS graph immediately before the mission node comes up. But the
decisive experiment (`left` *with* a probe in front of it, which should then
fail) was never run: **the robot went off the network mid-diagnosis** and did
not come back.

`scripts/move-robot.sh` pauses 2s between probe and mission
(`DISCOVERY_SETTLE_SECONDS`), and its comment says the link is unproven. If the
pause turns out to be unnecessary, delete it — do not leave it as folklore.

## What the next person must do

1. **Run the experiment above.** It is one command and it either confirms or
   kills the hypothesis.
2. **The robot still has the old `~/move.sh`** with the fail-open in it.
   Replace it with `scripts/move-robot.sh`. The checkout is a real git clone
   now, so a pull is enough.
3. **Neither fix has run on hardware.** Everything here is verified against
   tests and code; nothing in this handoff was demonstrated on the robot.
   `make verify` passes — ruff clean, 428 passed, 3 skipped.
4. **The robot was boxed in** at 0.36 m front and rear when last measured. Move
   it before expecting `forward` to do anything except refuse — which, after
   these changes, is what it will correctly do.

## Two notes on the tooling

`impact` reported **0 references** for `ros2_cmd_vel.py`. Three modules import
it. File-level symbols carry no references in the index; use grep to confirm a
blast radius before trusting a zero.

`make verify` had never run past its first target on this machine: `ruff`'s
wrapper was installed without its binary, `lint` is first in
`verify: lint test assets ...`, and a chained target list stops there. Nine
checks had been silently skipped. Fixed by linking the binary where the finder
looks. **A green-looking gate that exits early is worse than a red one.**
