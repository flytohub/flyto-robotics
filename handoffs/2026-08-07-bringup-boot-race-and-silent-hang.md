# Bringup: the boot-time race is fixed; a second, worse failure mode is not

Date: 2026-08-07
Spans: `flyto-robotics` (merged), live testing on `flyto-robot.local`

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

## State of the robot right now

As of this write-up, `turtlebot3-bringup.service` is still `active` with
`NRestarts=0` and `/odom` still not publishing — the hang has not cleared on
its own. A manual `sudo systemctl restart turtlebot3-bringup.service` clears
it (confirmed working earlier in the day for the *other* failure mode; not
yet re-confirmed against *this* one specifically, since the two are
different failures and the manual restart was only exercised against the
first).

## Next step, in order

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
