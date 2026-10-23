# 2026-08-05 — TurtleBot3 commissioning and the keyboard-shortcut loop

Written while the robot was unreachable (the laptop had moved to another
subnet). Everything below is committed and pushed; nothing is left uncommitted.

## Robot as it stands

| Item | Value |
|---|---|
| Host | `flyto-robot.local` (mDNS). Last seen at `192.168.0.34` on `TP-Link_9001` |
| Login | key only — `ssh ubuntu@flyto-robot.local`. Password auth is disabled and the old password hash was removed |
| OS | Ubuntu 24.04.4, kernel 6.8.0-1060-raspi, Pi 4 Model B Rev 1.5 / 4 GB |
| ROS | Jazzy, 335 packages, `ROS_DOMAIN_ID=30`, `TURTLEBOT3_MODEL=burger` |
| Lidar | **LDS-03**, driven by `coin_d4_driver`, needs the `/dev/tb3_lidar` udev symlink |
| OpenCR | `/dev/ttyACM0`, battery reads ~11.7 V |
| Services | `turtlebot3-bringup` and `flyto-delivery`, both enabled; cold boot to fully ready measured at 40 s |

Verified on hardware: odometry streaming, 398-point scans, battery telemetry,
and the wheels turning (30.7° in-place rotation, 0.0 cm translation).

## Three traps that cost real time — do not rediscover them

1. **The Ubuntu image ships without `noble-updates`.** Only `noble` and
   `noble-security` are enabled, so any `-dev` package whose runtime came from
   updates can never resolve. ROS installation fails with "held broken
   packages". Fixed by adding `/etc/apt/sources.list.d/ubuntu-updates.sources`.

2. **`ld08_driver` reports `FOUND LDS-02` for any CP2102 device.** All three
   TurtleBot3 lidars use the same USB-UART chip, so that detection is a false
   positive and it sent the diagnosis down the wrong path. The honest signal
   came from the LDS-03 driver, which said `Failed to open lidar port` and
   named the port it wanted. A driver that reports failure precisely is worth
   more than one that guesses optimistically.

3. **`/cmd_vel` is `TwistStamped` on Jazzy, not `Twist`.** A type mismatch
   matches zero subscribers and DDS reports no error at all: the logs look
   healthy, sensors stream, and the robot ignores every command. This is the
   single hardest failure to diagnose here, and it cannot be reproduced in a
   container whose fake robot uses the same type on both ends. Both the
   delivery runner and the shortcut controller now resolve the type from the
   topic's live subscription info via `flyto_robotics/ros2_cmd_vel.py`.

## Landed today

- `12acf28` goal resolution: free-form goals become validated workflows, or a
  structured rejection with reason, stage, candidates and an operator action.
- `a261b82` cmd_vel type resolution and configurable topics for delivery.
- `2d1f826` `turn_relative` capability, the four arrow-key cards as reviewed
  JSON, and a fail-closed input ack.
- `ffe7ed1` one shortcut controller driving four cards, sharing `CmdVelChannel`,
  plus `deploy/systemd/flyto-shortcut.service` with a reciprocal
  `Conflicts=flyto-delivery.service`.

## Next, in dependency order

1. **Finish the robot-side install.** `pip install -e .` was interrupted by the
   network change, so `ros2 run flyto_robotics shortcut_controller` may not
   resolve yet. Re-run it, then `sudo systemctl start flyto-shortcut` (which
   stops `flyto-delivery` by design) and verify the four cards load.
2. **Cloud side of the shortcut loop.** The relay and the browser runtime
   already exist. What is missing: a `robot_workflow_id` field on the binding
   (the UI currently sends a Firestore document id where the robot expects a
   plan id), `FLYTO_ROBOTICS_INPUT_TOKEN` in the desktop environment, and an
   allowlist on the ack the relay forwards to the browser.
3. **The first real delivery.** Needs the robot on the floor with LiPo power.
   It travels about 4 m.
4. **Register the Pi as a device resource.** It runs the gateway and ROS but
   not a Flyto2 runner, so it never appears in the resource registry and a
   Space cannot scope itself to it.

## Standing risks

- **Addressing follows the network, not the robot.** The Pi knows two SSIDs.
  Move the laptop to a third and they cannot see each other, which is exactly
  what will happen at a venue. Add the phone hotspot as a third SSID before
  travelling.
- **The lidar's `minimum_range` is omnidirectional.** Anything within 25 cm in
  any direction latches an obstacle stop, including a wall the robot is turning
  away from. Commission on open floor first.
- **Static friction against the 0.02 m/s speed floor** is the most likely
  first-run surprise on a real floor: if the robot cannot break stiction, the
  last centimetres never close and the step times out. Fail-closed, but it
  would make every forward card fail at the finish line.
