# Handoff registry

Newest first. Each handoff is `YYYY-MM-DD-topic.md` and records state that the
code and git history do not carry on their own: what was tried, what misled
us, and what the next person should not rediscover.

| Date | Topic | Summary |
|---|---|---|
| 2026-08-08 | [The robot turns but will not drive](2026-08-08-move-fails-while-turn-succeeds.md) | `left` succeeded 4/4 while `forward`/`backward` failed 2/2, on a robot with every topic healthy. Cause was odometry discovery latency and the sensor gate correctly refusing. Fixed a clearance check that reported 99 m when it could not read the lidar (0.36 m of real room at the time), and a cmd_vel type guess that was cached permanently so a driver discovered after the 3s deadline was never adopted. Both split into ROS-free modules that could be tested at all. The probe-delays-discovery link is **suspected, not proven** — the robot went offline before the decisive run |
| 2026-08-08 | [One-time recovery and persistent diagnostics](2026-08-08-one-time-recovery-and-diagnostics.md) | Idempotent Pi USB gadget recovery at 10.77.0.1, key-only SSH, read-only reason portal, and generic current/last-failure telemetry remove routine card pulls; physical Pi acceptance remains. |
| 2026-08-08 | The robot loses the network, and how to get back in | Router swap left the robot with no known SSID and no SSH. Board is a Pi 4B (dual band, so band was never it); the cloud half dials out and is unaffected; card edits need an `instance-id` bump; wlan0 ends up DORMANT with a silent supplicant. |
| 2026-08-05 | [TurtleBot3 commissioning and the keyboard-shortcut loop](2026-08-05-turtlebot3-bringup.md) | Robot fully provisioned on Jazzy with LDS-03; goal resolution, `turn_relative` and the four arrow cards landed; three non-obvious traps documented |
| 2026-08-06 | Demo arena, directional safety | `2026-08-06-demo-arena-and-safety-sectors.md` | Active |
| 2026-08-07 | [Bringup: boot race fixed, silent hang is not](2026-08-07-bringup-boot-race-and-silent-hang.md) | OpenCR handshake race merged (`main`) and verified via forced kill + systemd auto-recovery; a second failure — the same node hangs alive with every topic dead after a real motor command — found live the same day. The `/odom`-freshness watchdog (`Type=notify` + `WatchdogSec`) is now written, unit-tested, deployed, and verified on the robot: induced silent hang → automatic restart in ~56s |
| 2026-08-08 | [First real cloud-to-robot mission](2026-08-08-cloud-to-robot-first-real-run.md) | Seven defects between "written" and "proven" — completion contract, terminal state, pose field, sim identity on hardware, unrecorded range, stale runner copy, wrong lease header. All fixed; a cloud task now turns the robot and its evidence is assessed. Camera step still blocked |
