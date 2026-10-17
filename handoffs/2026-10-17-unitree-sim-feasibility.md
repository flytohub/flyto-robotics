# Unitree simulation feasibility

Owner: claude
Branch: claude/unitree-sim-feasibility
Date: 2026-10-17

## What changed

- `docs/UNITREE_SIM.md`: why Unitree, what was measured, and two
  integration options.
- `scripts/unitree_mujoco_check.py`: a headless probe of the G1 policy in
  MuJoCo. It walks under a velocity command, then stops, and reports
  displacement and stance. It is not an adapter.

## Why

The owner chose Unitree in simulation as the second robot for the UCC
cross-brand acceptance target, and is buying an ESP32 device for the
cross-technology half.

## Verified

- The probe ran on the development Mac: G1 moved +2.224 m in 5 s at 0.5 m/s,
  then drifted 0.224 m during a 3 s stop and stayed upright.

## Not verified

- **Flyto2 path.** No Flyto2 path exists: nothing is registered, dispatched
  or judged.
- **Go2 and the SDK2 bridge.** Neither Go2 nor unitree_mujoco's SDK2 bridge
  was tried.
- **Repository checks.** This change adds a script and docs only, and the
  repository's `make verify` targets do not cover them.

## Follow-ups

- Choose option 1 (standard ROS 2 bridge) or option 2 (Core external
  adapter) from `docs/UNITREE_SIM.md`.
- Declare `establishes`/`occupies` on the motion contracts once
  flyto-core 2.41.0 lands.
