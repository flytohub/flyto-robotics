# Bringup: the boot-time race is fixed; a second, worse failure mode is not

Date: 2026-08-07 (updated 2026-08-10: live start-limiter evidence, then first
real motion on hardware)
Spans: `flyto-robotics` (merged), live testing on the lab TurtleBot3 host

## What this is

Two different ways `turtlebot3-bringup.service` leaves the robot with no
`/odom` and no working `cmd_vel` path, found on the same robot on the same
day. The first is fixed and merged. The second was found *while verifying
the first fix was holding* and is not fixed — there is no code for it yet,
only a diagnosis.

---

## Fixed and merged: the OpenCR handshake race (PR #1, `main`)

**Symptom**: three `move.sh` calls in a row failed at exactly the
`sensor_startup_grace_seconds` timeout (10.0s), `x=None y=None yaw=None`
throughout — the mission node never received a single odometry message.

**Cause**: `turtlebot3_ros` (the OpenCR bridge — publishes `/odom`, relays
`/cmd_vel` to the wheels) lost its very first handshake with the OpenCR board
on cold boot:

```
[ERROR] [turtlebot3_node]: Failed connection with Devices
terminate called after throwing 'rclcpp::exceptions::RCLError'
[ERROR]: process has died [pid 1044, exit code -6, ...]
```

`ros2 launch`'s default behaviour when one of the three bringup processes
dies is to log it and keep the other two running. Systemd sees the wrapping
launch process as continuously active, so `Restart=always` — already
configured for exactly this case — never fires.

**Fix**: `launch/turtlebot3_bringup_supervised.launch.py` includes ROBOTIS's
own `robot.launch.py` unchanged and adds an `OnProcessExit` handler
(`target_action=None`) that shuts the whole group down the moment any of the
three processes exits, for any reason. `turtlebot3-bringup.service` entered
version control for the first time in the same change.

**Verified against the real robot**, not a mock: killed the exact
cgroup-managed `turtlebot3_ros` PID under the real systemd unit; recorded
`NRestarts=1`; the whole group came back with fresh PIDs; `/odom` reached
20 Hz again — zero manual intervention. Full detail in
[2026-08-05-turtlebot3-bringup.md](2026-08-05-turtlebot3-bringup.md) is the
prior state; this fix is the commit on `main` titled *"recover automatically
when the OpenCR handshake fails at boot."*

---

## Not fixed, no code yet: a silent hang after a real motor command

Found a few hours after the fix above shipped, during ordinary use — not a
contrived test.

```
chester@ChesterdeMacBook-Pro ~ % ssh ubuntu@flyto-robot.local '~/move.sh left'
[INFO] ...: first odometry pose: x=-0.000, y=-0.000
[INFO] ...: mission tb3-lab-shortcut-001 finished with state completed
succeeded  x=0.0001 y=0.0002 yaw=1.5501  safety_stops=0
  turn.left turned 1.541rad toward 1.571rad

chester@ChesterdeMacBook-Pro ~ % ssh ubuntu@flyto-robot.local '~/move.sh left'
failed  x=None y=None yaw=None  safety_stops=0

chester@ChesterdeMacBook-Pro ~ % ssh ubuntu@flyto-robot.local '~/move.sh forward'
clearance front: 99 m
failed  x=None y=None yaw=None  safety_stops=0
```

The first call **actually drove the robot** — a real 90° turn, real odometry
throughout. The next two calls, seconds later, show the exact same symptom as
the bug above (`x=None`, the 99 m clearance sentinel meaning `/scan` is also
dead) — but this time it is a different failure, confirmed live:

```
$ systemctl show turtlebot3-bringup.service -p ActiveState,NRestarts
ActiveState=active
NRestarts=0
```

`turtlebot3_ros` never exited. `ps` showed it still running (`Sl`, sleeping/
interruptible) at the same PID the whole time. `NRestarts=0` — the fix above
never had anything to catch, because nothing crashed. The process is alive
and systemd believes the service is healthy.

**But every topic that process publishes went dead at once**, not just
`/odom`:

```
$ ros2 topic hz /odom            → does not appear to be published yet
$ ros2 topic hz /battery_state   → does not appear to be published yet
```

Its own log is silent from the moment `diff_drive_controller: Run!` printed
at startup — no further line, ever, not even after the turn completed. No
error, no warning, nothing. `dmesg -T | grep ttyACM` shows the USB-serial
device enumerated once at boot and never disconnected or reset — this is not
the OS losing the device. Whatever hung, hung above the OS's serial layer,
inside `turtlebot3_ros`'s own communication with the OpenCR firmware — most
likely a read/ack loop that blocked forever with no timeout, right after the
first real motor command completed.

### Why the existing fix cannot catch this

`OnProcessExit` fires on exit. This process does not exit — it hangs while
alive. The whole mechanism PR #1 added is scoped to "a process died"; this is
"a process is running and producing nothing," which is a disjoint failure
mode that needs a different kind of check: not *is it running* but *is it
still doing anything*.

### The fix that was agreed but not started

A watchdog based on message freshness, not process liveness — the same
pattern `mission.py`'s `evaluate_sensor_gate` already uses to decide whether
a *mission* can trust its sensors, applied instead to whether the *bringup
service itself* is worth keeping around:

- A small node (bundled into the same launch file, so it shares the
  service's cgroup and `$NOTIFY_SOCKET`) subscribes to `/odom`.
- While messages keep arriving within some window, it calls
  `sd_notify(WATCHDOG=1)`.
- The unit changes `Type=simple` → `Type=notify` (`NotifyAccess=all`, since
  the ping comes from a sibling process, not the direct child) and gets a
  `WatchdogSec=`.
- If `/odom` goes stale for longer than that window, the watchdog simply
  stops pinging. Systemd's own watchdog timer kills the whole cgroup and
  `Restart=always` — already there — brings it back. No new privilege
  needed: this is systemd's built-in mechanism, not a script calling
  `systemctl restart` on itself.
- `READY=1` should be deferred until the first `/odom` message (or a
  generous fixed timeout matching `sensor_startup_grace_seconds`, 10s,
  already established in `mission.py`) so the normal ~8s cold-start window
  before odometry begins does not itself trip the watchdog.

None of this is written. The only work done so far on it was reading
`evaluate_sensor_gate` for the pattern to reuse — no launch-file changes, no
systemd unit changes, no tests.

> **Update (same day, later session): implemented AND verified on the robot.**
> The watchdog is built exactly as designed — `flyto_robotics/
> bringup_watchdog.py` (pure decision function `evaluate_bringup_watchdog` +
> clock-free `WatchdogTicker` + a thin rclpy runner), added to
> `turtlebot3_bringup_supervised.launch.py` as a fourth tracked process, with
> the unit switched to `Type=notify`, `NotifyAccess=all`, `WatchdogSec=15`,
> `TimeoutStartSec=120`. Unit tests in `tests/test_bringup_watchdog.py` cover
> wait/arm/ping/starve, the full hang-shaped lifecycle, and the sd_notify
> datagram.
>
> Steps 1–3 below were then all executed live:
>
> 1. The robot was found still in this exact hang (`active`, `NRestarts=0`,
>    no `/odom`). Deploying the unit and restarting cleared it.
> 2. Two deployment realities surfaced and are fixed in the same change:
>    `flyto_robotics` is **not** pip-installed on the robot (the delivery
>    unit finds it via `WorkingDirectory`), so the watchdog `ExecuteProcess`
>    sets `cwd` from the launch file's own path — without it the module
>    exits 1 with ModuleNotFoundError and the supervisor cycles the group.
>    And rclpy surfaces SIGTERM as `ExternalShutdownException`, which must be
>    caught or every clean stop logs a crash traceback.
> 3. Induced the real failure shape at 22:04:58: `sudo kill -STOP` on the
>    live `turtlebot3_ros` PID — process alive, every topic silent.
>    Observed: `+5s` "withholding watchdog pings", `+20s` systemd `Watchdog
>    timeout (limit 15s)!`, stop escalation to SIGKILL (a stopped process
>    ignores SIGINT), restart at `+50s`, `READY=1` on first `/odom` at
>    `+56s`, `/odom` back at 20.01 Hz. `NRestarts` 0→1, `Result=watchdog`,
>    zero manual intervention.
>
> A note for future `ros2 topic hz` checks over SSH: export
> `ROS_DOMAIN_ID=30` first — a bare sourced shell sits on domain 0 and
> reports a healthy topic as "not published yet", the same class of false
> negative as the short-window mistake warned about below.

---

## Third failure: recovery that never stops, and the lidar that got blamed

The two fixes above both do the same thing — turn a stuck bringup into a
restart. Neither of them asks how many times that should be allowed to happen.
Production answered: the unit was found at

```
$ systemctl show turtlebot3-bringup.service -p NRestarts
NRestarts=193
```

**What it looked like**: the LDS lidar flapping. `/scan` appearing, then gone,
then back a minute later. Every instinct points at the lidar — a loose USB
plug, a failing LDS-03, a `/dev/tb3_lidar` udev rule misfiring.

**What it actually was**: the lidar was healthy the entire time and was being
killed as collateral. `turtlebot3_node` was failing its OpenCR/Dynamixel
motor-bus handshake — the same class of failure as the boot race at the top of
this document, except now persistent rather than a cold-boot race. The
supervised launch group did exactly what it was built to do and brought the
whole group down for that one dead process, taking the working lidar with it.
`Restart=always` brought it straight back to fail the same way. 193 times.

That is the lesson worth carrying: **whole-group supervision converts a
single-device hardware fault into a whole-robot flap, and the flapping device
you observe is not the device that is broken.** If a sensor looks intermittent
on this robot, check `NRestarts` on the bringup unit before touching the
sensor.

### Why the rate limit that was already there did not fire

The unit had carried `StartLimitIntervalSec=300` and `StartLimitBurst=20` since
it entered version control. Both keys were in `[Unit]`, which is correct.
Neither did anything, and no larger burst would have either.

The limiter counts starts inside a sliding window. One failure cycle here is
the `ExecStartPre` device wait, plus `ros2 launch` coming up, plus the OpenCR
handshake attempt and the group teardown that follows it, plus `RestartSec=5`.
Twenty of those do not fit inside 300 seconds — so the counter kept aging out
before it ever reached 20, and `Restart=always` was, in practice, unbounded.
A burst of 3 does fit inside the same window.

### The change

`[Unit] StartLimitBurst=20` → `3`. `StartLimitIntervalSec` stays at `300`.
Nothing in `[Service]` changed: `Type=notify`, `NotifyAccess=all`,
`WatchdogSec=15`, `Restart=always`, `RestartSec=5`, the two `ExecStartPre`
serial waits, `KillSignal=SIGINT`, `TimeoutStopSec=20`, and the supervised
whole-group `ExecStart` are all retained exactly as verified above.

Three failed starts within 300 seconds now park the unit in `failed`. It stays
there until a human inspects the board and clears it deliberately:

```
$ systemctl status turtlebot3-bringup.service      # expect: failed, start request repeated too quickly
$ systemctl reset-failed turtlebot3-bringup.service
$ systemctl start turtlebot3-bringup.service
```

Note both keys must stay in `[Unit]`. systemd reads `StartLimit*` from `[Unit]`
only; moved to `[Service]` they are accepted, ignored, and the unbounded retry
loop comes back silently. The contract tests added to
`tests/test_bringup_watchdog.py` parse the unit into sections and assert
section + key + value for exactly that reason — a substring match on
`StartLimitBurst=3` would pass on a unit where the directive does nothing.

### What this does not do — read this before trusting it

- **It does not fix the hardware.** The OpenCR/motor-bus handshake failure is
  untouched. This is containment: it stops a hardware fault from being retried
  forever and presenting as an intermittent sensor. The board still has to be
  inspected, reseated, or replaced.
- **It trades availability for legibility, on purpose.** Before, a transient
  fault self-healed and a permanent one flapped forever. Now a permanent fault
  stops the robot until someone looks. A robot parked in `failed` is a robot
  that will not come back on its own after a reboot loop — that is the intent,
  but it is a real behavioural change for anyone expecting the old unit.
- ~~**The live limiter behaviour is unverified.**~~ **Superseded 2026-08-10 —
  now verified live; see the section below.** As written at the time: the
  placement and values were covered by the parsing tests, but that systemd
  actually parks *this* unit on the third failed start within 300s had not been
  observed on the robot, and the unit had not been deployed. Unlike the
  watchdog above — verified by inducing the real hang with `kill -STOP` — no
  one had yet induced three consecutive OpenCR handshake failures and watched
  the unit stop retrying. That observation has since happened, unforced, on a
  cold boot.
- **The interaction with the watchdog path is also unverified.** A
  `Result=watchdog` restart counts against the same start limiter as a
  handshake failure. In principle three watchdog recoveries inside five minutes
  will now park a robot that the previous unit would have kept recovering.
  That is believed to be the right trade — three silent hangs in five minutes
  is not a healthy robot — but it has not been exercised.

---

## Update 2026-08-10: the limiter is verified live; the OpenCR bus is not fixed

A read-only inspection of the robot over SSH after a cold boot. Nothing was
deployed, restarted, or commanded during it — no `systemctl start`, no
`reset-failed`, and **no motion command of any kind**. The robot had reached
this state on its own.

**What was observed**

```
$ systemctl show turtlebot3-bringup.service \
    -p ActiveState,SubState,Result,NRestarts
ActiveState=failed
SubState=failed
Result=protocol
NRestarts=3
```

That is the limiter doing exactly what the change above intended: three failed
starts inside `StartLimitIntervalSec=300` parked the unit in `failed` instead
of letting `Restart=always` cycle it indefinitely. The caveat struck through
above — that this had only ever been proven by parsing tests — no longer holds
for this robot. **The live limiter behaviour is verified.** It was verified by
the real fault occurring, not by an induced proxy.

**Where the fault actually sits now**

The journal for each of the three attempts shows the serial layer succeeding
before the failure:

- the port `/dev/ttyACM0` was opened successfully;
- the baudrate was changed successfully;
- *then* `turtlebot3_node` logged `Failed connection with Devices`.

So the surviving fault is **below the USB serial layer**: the host can talk to
the OpenCR's USB CDC endpoint fine, and the failure is on the
OpenCR/Dynamixel device bus behind it. This is consistent with the earlier
`dmesg` finding that the OS never lost or reset `ttyACM0` — the OS side has
been healthy in every one of these incidents, and remains so here.

**The lidar was healthy again, and again looked guilty**

The LDS-03 on `/dev/tb3_lidar` activated successfully on every one of the three
attempts. It was stopped only as collateral, when whole-group supervision tore
the group down for the dead `turtlebot3_node`. Same shape as the 193-restart
incident above, now bounded to three cycles instead of running unattended.
This is the second independent confirmation of that lesson: **check
`NRestarts` and the bringup unit before suspecting a flapping sensor.**

**The observation layer reported it correctly**

`flyto-robot-doctor` continued emitting privacy-bounded `system.diagnostics`
throughout, with `quality=degraded`, `primary_reason_code=
robot_service_unhealthy`, and `turtlebot3-bringup.service` named as the
unhealthy unit. A parked robot is still legible from the diagnostic portal
without exposing anything beyond the doctor's bounded fields.

**What was not available, and what was not claimed**

No `/odom`, no `/scan`, and no working `cmd_vel` path existed during the
inspection — a unit in `failed` publishes nothing. No physical motion was
commanded or observed, so this update is **not** evidence of movement on real
hardware.

### Verified vs. still unresolved — do not blur these

| Verified live 2026-08-10 | Still unresolved |
| --- | --- |
| The start limiter parks the unit on the third failed start in 300s (`Result=protocol`, `NRestarts=3`). | The OpenCR/Dynamixel bus failure that causes those starts to fail. |
| Whole-group supervision stops the lidar as collateral, not as a lidar fault. | Whether the cause is the OpenCR board, its power rail, the Dynamixel cabling, or a servo. |
| `flyto-robot-doctor` reports the parked unit as `robot_service_unhealthy` at `quality=degraded`. | Any claim of real robot movement — none was attempted. |

Containment works. Repair has not happened.

> **Superseded later the same day.** The paragraph that stood here said the
> board, its power, and the Dynamixel cabling had to be physically inspected,
> reseated, or replaced *before this robot could be said to move*. That gate no
> longer holds: a later cold reboot brought the bus up and the robot moved
> under a real command. See
> [Update 2026-08-10 (later): the robot moved](#update-2026-08-10-later-the-robot-moved-cold-start-and-command-timing-did-not-hold-up).
> The observations above stand exactly as recorded — they are what a parked
> unit looked like — but the conclusion drawn from them was wrong about what
> had to happen next.

### Next step from here (as written during the read-only inspection)

1. ~~Physically inspect the OpenCR: power rail and switch, the 12V/battery
   feed, and every Dynamixel cable and connector on the bus. Reseat them.~~
   **Overtaken by events** — a cold reboot cleared the handshake failure with
   no physical intervention performed or recorded. This is still worth doing
   as preventive maintenance, but it is no longer a blocker, and the fact that
   it was never done is itself the finding: see the intermittency note below.
2. Clear the parked unit deliberately (`systemctl reset-failed` then
   `systemctl start`) and confirm `/odom` at 20 Hz with a generous
   `ros2 topic hz --window`, remembering to export `ROS_DOMAIN_ID=30` first.
3. The watchdog/limiter interaction noted above is still unexercised: three
   `Result=watchdog` recoveries inside 300s would now also park the unit.
   This incident was `Result=protocol`, so it says nothing about that path.

---

## Update 2026-08-10 (later): the robot moved; cold start and command timing did not hold up

Later the same day, after a cold reboot, the OpenCR/Dynamixel handshake
succeeded and the robot was driven under a real command. Nothing was repaired
between the two observations — no inspection, reseat, or replacement was
performed or recorded. **The bus fault is therefore intermittent, not fixed.**

### Cold start: one launch failed the watchdog before one held

- The **first launch reached `READY`**, then `/odom` went stale **74 s** later.
  The freshness watchdog did its job and restarted the unit **once**.
- The **second launch stayed active** and healthy:

```
/odom            19.95 Hz
/scan            10.05 Hz
battery          12.37 V
throttling       none
USB disconnect   none
```

So the recovery path works end to end on real hardware for a second distinct
`Result` class. But note what this also says: **a cold start is not reliably
one-shot on this robot.** One of two launches this boot needed the watchdog to
rescue it. That is not the same as a healthy boot, and it should not be
reported as one.

### First real motion — but driven by hand, outside the supported path

This first run was **ad hoc**: raw `ros2` CLI publishers, not the product's own
command path. Read it as a warning about how it was driven, not as a statement
about the shipped code.

A single safety-gated `TwistStamped` command was issued with the sensor gate
satisfied:

- front clearance before: **1.328 m**;
- exactly **one matched subscriber** on the command topic;
- odometry `x` moved **0.000209 m → 0.171863 m**.

**The robot physically moved under its own power.** This is the first real
motion evidence on this hardware, and it retires the "no proof of movement on
the real robot" caveat that has stood in this repo since the simulation work.

After the command:

```
linear velocity   0.0
publisher count   0
front clearance   1.167 m
service           active
NRestarts         1
```

The stop path did end at zero velocity with no publisher left attached, and
the service survived the whole exercise.

### The distance overshot — a warning about ad-hoc CLI publishers

The move was **intended to be about 4 cm. It travelled 17.2 cm** — more than
four times the target. This is not a controller or calibration finding,
because the run did not use the controller's command path at all.

The zero/stop command came from a **second `ros2` CLI publisher**, started
after the one that issued the motion. The most likely explanation, and one
**consistent with the observed timing**, is that the new publisher had to
complete **DDS discovery** before its zero could be delivered, and the robot
kept moving across that gap.

Stated honestly: **that cause is an inference, not a proven root cause.** No
discovery-timing trace was captured, and the overshoot was not reproduced or
bisected. What *is* solid is the operational lesson, and it is worth carrying
on its own:

> **Do not drive this robot with two separate `ros2` CLI publishers.** A stop
> that has to discover its subscriber before it can be delivered is not a
> stop. Whatever the exact mechanism, an unmatched publisher in the stop path
> has unbounded and unmeasured latency.

**This does not describe a missing feature.** The single-process, already-matched
command path this run should have used **already exists in this repo**:
`CmdVelChannel` in `flyto_robotics/ros2_cmd_vel.py` holds one publisher for the
life of the run, resolves the message type against the live subscriber with its
own discovery grace deadline, and `scripts/move-robot.sh` drives it through
`flyto_robotics.ros2_node.run`. An earlier draft of this handoff called for
building that path before the stop could be trusted; that was wrong, and the
next section is the run that used it.

---

## Update 2026-08-10 (later still): the supported path, end to end

The same hardware, driven through the product's own path rather than by hand.

**Before moving** — `scripts/move-robot.sh` required a forward clearance and
refused to proceed without one; it measured **1.17 m** front clearance and
continued.

**During** — `CmdVelChannel` resolved its binding against the live subscriber,
moving from a **provisional `Twist`** to a **live `TwistStamped`** publisher.
That is the mismatch the channel exists to catch, resolving correctly against
real hardware rather than against the Gazebo bridge's `Twist`.

**Result** — the mission **succeeded**: it moved **0.371 m toward a 0.400 m
target**, ran `safe_stop`, and recorded **`safety_stops=0`**.

**Post-run check**

```
linear velocity        0.0
/cmd_vel publishers    0
front clearance        0.869 m
turtlebot3-bringup     active (running)
NRestarts              1
```

The publisher count returning to zero is the meaningful part: the channel tore
itself down cleanly, so nothing was left holding `/cmd_vel` after `safe_stop`.

### What is proven now, and what is explicitly not

**Proven on this hardware, 2026-08-10:** supported **single-process,
odometry-closed-loop motion and stop**. The robot moves under the shipped
command path, closes the loop on real odometry, stops on its own `safe_stop`,
and releases the topic — with the sensor gate enforced before motion and no
safety stop needed during it. The freshness watchdog also recovered a
stale-`/odom` launch automatically on the same boot (`NRestarts=1`), and
sensors and power were healthy while up (`/odom` 19.95 Hz, `/scan` 10.05 Hz,
12.37 V, no throttling, no USB disconnect).

**These remain separate, unmet gates — none is implied by the run above:**

| Gate | Status |
| --- | --- |
| Repeated cold-boot stability | Not established. One of two launches this boot needed a watchdog rescue; one boot is not a rate. |
| Distance calibration tolerance | Not established. 0.371 m against a 0.400 m target is one sample with no declared tolerance. |
| Hardware E-stop | Not exercised. |
| Network-loss and sensor-loss acceptance | Not exercised on hardware. |
| Whether the OpenCR bus fault is gone | No. Nothing was repaired; it is intermittent and unexplained. |

### Next step from here

1. Declare a distance tolerance and measure against it over repeated runs.
   One 0.371 m sample against 0.400 m is not a calibration result.
2. Repeat cold boots and count how many launches reach `/odom` without a
   watchdog restart. One-in-two is a number worth tracking, not a footnote.
3. Exercise the hardware E-stop, and network/sensor loss, on the robot.
   These are listed in `STATE.md` as field-demo gates and remain unmet.
4. Still unexercised: three `Result=watchdog` recoveries inside 300 s would
   park the unit under the same limiter. This boot produced one.
5. Preventive OpenCR/Dynamixel inspection remains worthwhile — the earlier
   `Result=protocol` failure was never explained, only outlived.

---

## State of the robot as of 2026-08-07 (historical; superseded above)

As of this write-up, `turtlebot3-bringup.service` is still `active` with
`NRestarts=0` and `/odom` still not publishing — the hang has not cleared on
its own. A manual `sudo systemctl restart turtlebot3-bringup.service` clears
it (confirmed working earlier in the day for the *other* failure mode; not
yet re-confirmed against *this* one specifically, since the two are
different failures and the manual restart was only exercised against the
first).

## Next step, in order (as written 2026-08-07; steps 1–3 completed that day)

1. Manually restart the service to get a working robot back for further
   testing (`sudo systemctl restart turtlebot3-bringup.service`, then confirm
   `/odom` with a generous `ros2 topic hz --window` — short windows gave
   false "not published" readings earlier the same day purely from the CLI's
   own DDS discovery warm-up, not a real problem; do not repeat that mistake).
2. Build the freshness watchdog described above. Write the freshness/ready
   decision as a pure function first (mirroring `evaluate_sensor_gate`'s own
   separation of decision logic from ROS I/O) so it can be unit tested
   without a robot, the same way this repo already tests `mission.py`.
3. Verify against the real robot the same way PR #1 was verified: induce the
   actual failure (not a proxy for it) and confirm the specific mechanism —
   here, that letting `/odom` go stale on purpose causes systemd to restart
   the unit on its own, via the watchdog timeout, not via a crash.
