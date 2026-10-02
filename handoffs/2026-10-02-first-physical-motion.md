Owner: claude
Branch: claude/physical-acceptance-20261002
Date: 2026-10-02

# First accepted physical motion: bounded advance, mid-motion cancel, safe stop

## What changed

No code changed. This records the first physical motion of the TurtleBot3
Burger through the external Generic ROS 2 Adapter, run against the runbook in
`flyto-cloud/handoffs/2026-09-21-robotics-software-hardware-final-closure.md`
(Phases 1, 2 and the first half of 4).

Evidence stays untracked, as `AGENTS.md` requires for generated evidence. It is
on the owner's Mac under `results/physical-2026-10-02/` (ignored by
`results/*`), and the numbers that matter are copied into this file:

- `advance-0.10m-post-stop.json` — three post-stop observation bundles after
  the 0.10 m advance.
- `advance-0.30m-cancel.json` — preflight, mid-motion, and three post-stop
  bundles for the cancel run, plus the invoke/cancel/halt outcomes.
- `cancel_test.py` — the exact script used for the cancel run (has a `DRY=1`
  mode that observes without invoking anything).

Path used, unchanged from the runbook:

```text
Mac -> flyto_robotics.generic_ros2_adapter.build() (FLYTO_ROS2_TRANSPORT=rosbridge)
    -> ssh -L 19090:127.0.0.1:9090 ubuntu@192.168.0.34
    -> Pi loopback rosbridge -> Nav2 behavior_server drive_on_heading
```

The 0.35 m clearance floor was not changed.

## Why

`STATE.md` and the 09-21 runbook both said physical motion had never been
accepted: every earlier preflight saw 0.14–0.20 m and was refused. The robot was
moved to an open area and the LiDAR re-seated by the owner, so the gate could be
exercised in its passing direction for the first time.

## Verified

Cold boot (boot id `e064e440…`, before the LiDAR was re-seated):

- `turtlebot3-bringup`, `camera-v4l2`, `laser-scan-stabilizer`,
  `slam-toolbox`, `nav2`, `rosbridge` reached `active` on their own in ~110 s,
  every `NRestarts=0`. No manual restart.
- No Flyto2 unit or process on the Pi (the only `flyto` match is avahi
  announcing the hostname).
- `rosbridge` listens on `127.0.0.1:9090` only.

After the LiDAR was re-seated (boot id `7446021b…`, booted 01:39 +08:00):

- `/scan` 10.2 Hz, 400 bins, `base_link -> base_scan` yaw 0.0°. The owner
  confirmed physically that 0° faces the front.
- Minimum clearance by the adapter's rule (finite and within
  `[range_min, range_max]`): 0.539 m global, at 97°.
- `/odom` ~20 Hz, camera 640x480 `yuv422_yuy2`, `/map` 133x52 at 0.05 m,
  `map -> odom` live, Nav2 lifecycle nodes `active`, all four Nav2 actions
  present.
- Adapter rediscovery over the tunnel: 5 capabilities
  (`advance`, `halt`, `navigate`, `retreat`, `rotate`), `deployment_mode=hardware`,
  `calibrated=false`.

Motion 1 — `motion.advance distance_m=0.10 speed_mps=0.05`, owner present and
confirmed the area clear:

- Preflight clearance 0.541 m; outcome `completed` in 3.25 s,
  `execution_count=1`.
- odom (0.2176, 0.0034) -> (0.3360, 0.0188): 0.119 m along heading
  (displacement bearing 0.129 rad, robot yaw 0.129 rad).
- `motion.halt` `completed`; x/y identical across three samples over 3 s.

Motion 2 — `motion.advance distance_m=0.30 speed_mps=0.05`, invoked with a
1.5 s deadline, then `cancel`:

- `invoke` returned `timeout` / "ROS 2 action still running" as designed.
- `cancel` returned `cancelled` (Nav2 status 5); `motion.halt` `completed`.
- Total travel 0.159 m of the commanded 0.30 m; post-stop x/y identical across
  three samples.

## Not verified

- **Mid-motion timing.** The "mid" sample (0.117 m at "1.5 s") implies
  ~0.08 m/s against a commanded 0.05 m/s. Most likely the sample was taken
  later than 1.5 s, because `observe()` first calls two rosapi services. The
  goal does carry `speed: 0.05`. Not confirmed with message timestamps.
- **Cancel stopping distance.** The robot travelled ≤ 0.040 m between the mid
  sample and the stop, and the cancel took ~2.8 s to be confirmed. Measure this
  with timestamps before relying on cancel in a tight space.
- **Overshoot.** The 0.10 m advance travelled 0.119 m (+19 %). Budget for it.
- **The preflight bundle of motion 1 was printed but not saved**: the script
  used an unsupported phase name (`post_execution`; valid phases are
  `preflight`, `before`, `after`, `post_stop`) and raised after `halt`. Halt was
  still sent from a `finally` block. Its values above come from stdout.
- **LiDAR blind readings.** ~12–15 readings per revolution are `0` (no return)
  and are excluded by the adapter's clearance rule, so an object nearer than
  0.10 m or below the scan plane is not seen. Owner sight confirmation stays a
  required step before motion.
- **Yaw drifts** ~0.0003 rad per 1.5 s while stationary (IMU), ~0.6° over a
  few minutes.
- **Battery** read 11.41 V / ~50 % before motion, down from 12.13 V / 91 %
  about 45 min earlier. Charge before repeated runs.
- **Two earlier `drive_on_heading` runs** at 01:46:22 and 01:48:42 (+08:00)
  came from another agent session on the same Mac with the same key. The owner
  says they were authorised. They are not part of this evidence.
- Nothing here went through Cloud: production `api.flyto2.com` was down
  (Cloudflare container stuck in `provisioning`; fix in flyto-cloud#412), so
  War Room / independent verification was not exercised.

## Follow-ups

1. Phase 5 of the runbook: once flyto-cloud#412 is deployed and the API is
   healthy, run one Space Task through the local AI Space host to the robot and
   let the Cloud verifier mark it. The local host already has this package
   installed editable and loads `ros2.generic`; it is deliberately *not*
   connected (no tunnel, no `FLYTO_ROS2_*` env) until someone is at the robot.
2. Before connecting the local host, check the legacy `flyto-modules-robotics`
   plugin (`robotics.move/turn/stop`) that the Cloud backend still loads: it is
   a second motion path and has not been checked against the 0.35 m gate.
3. Re-measure cancel latency and stopping distance with odom header stamps.
4. Camera calibration is still unclaimed (`K` all zeros).
