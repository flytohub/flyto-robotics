# Handoff registry

Newest first. Each handoff is `YYYY-MM-DD-topic.md` and records state that the
code and git history do not carry on their own: what was tried, what misled
us, and what the next person should not rediscover.

| Date | Topic | Summary |
|---|---|---|
| 2026-08-05 | [TurtleBot3 commissioning and the keyboard-shortcut loop](2026-08-05-turtlebot3-bringup.md) | Robot fully provisioned on Jazzy with LDS-03; goal resolution, `turn_relative` and the four arrow cards landed; three non-obvious traps documented |
| 2026-08-06 | Demo arena, directional safety | `2026-08-06-demo-arena-and-safety-sectors.md` | Active |
| 2026-08-07 | [Bringup: boot race fixed, silent hang is not](2026-08-07-bringup-boot-race-and-silent-hang.md) | OpenCR handshake race merged (`main`) and verified via forced kill + systemd auto-recovery; a second failure — the same node hangs alive with every topic dead after a real motor command — found live the same day. The `/odom`-freshness watchdog (`Type=notify` + `WatchdogSec`) is now written, unit-tested, deployed, and verified on the robot: induced silent hang → automatic restart in ~56s |
| 2026-08-08 | [First real cloud-to-robot mission](2026-08-08-cloud-to-robot-first-real-run.md) | Seven defects between "written" and "proven" — completion contract, terminal state, pose field, sim identity on hardware, unrecorded range, stale runner copy, wrong lease header. All fixed; a cloud task now turns the robot and its evidence is assessed. Camera step still blocked |
