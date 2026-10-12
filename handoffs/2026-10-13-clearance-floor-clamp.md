# The configured clearance can raise but never lower the physical floor

| Field | Value |
|---|---|
| Date | 2026-10-13 |
| Owner | Codex |
| Scope | Generic ROS2 physical adapter and its declared scan live channel |
| Release candidate | 0.8.3 |
| Physical robot | Not run |

## Why this changed

The 0.8.2 documentation said the 0.35 m safety floor was unchanged and could
not be lowered, but `_clearance_floor()` used `max(0.1, configured)`. Setting
`FLYTO_ROS2_MIN_CLEARANCE_M=0.1` therefore made every caller enforce and
advertise 0.1 m. A read-only reproduction on the pinned 0.8.2 commit returned
`0.1`.

## What changed

| Contract | Result |
|---|---|
| No variable | 0.35 m |
| Variable below 0.35 m | 0.35 m |
| Variable above 0.35 m | The configured, stricter value |
| Scan live channel | Advertises the same clamped floor the adapter enforces |

The rule remains in the adapter's single `_clearance_floor()` source. No robot
name, deployment name, prompt, or special-case task wording was introduced.

## Verification

Unit tests cover the default, a 0.1 m attempted downgrade, a 0.5 m stricter
setting, and the live-channel declaration. `make verify` passed with 1,793
tests, lint, assets, deterministic dry runs, lab contract, ROS2 pairing and
execution-grant checks. `flyto-index verify . --full-scan --strict` passed all
20 checks with no warnings or failures.

## Still required

- Do not merge or update Flyto2 Cloud's pinned adapter commit until the real
  closed loop is exercised: stop, capture, blocked conclusion, and hospital
  system handoff.
- On the robot, explicitly set 0.1 m once and confirm both the declared live
  channel and motion evidence report 0.35 m, then restore the deployment
  configuration.
