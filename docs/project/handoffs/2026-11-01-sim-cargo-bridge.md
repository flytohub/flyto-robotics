# The live twin publishes the simulated tray's cargo state

> Superseded (2026-11-02): `cargo_tray.py`, `rosbridge_link.py` and `arm_place_signal.py` are deleted; the bridge is now the gated implementation described in [2026-11-02-fetch-live-halt-speed](2026-11-02-fetch-live-halt-speed.md).

Owner: claude
Branch: claude/fetch-live-sim
Date: 2026-11-01

## What changed

SIMULATION ONLY. `sim/fetch_return/cargo_bridge.py` publishes the symbolic
cargo state of the simulated tray as `std_msgs/msg/Bool` on
`$FLYTO_ROS2_CARGO_TOPIC` (default `/flyto/cargo_present`), advertised over
rosbridge with `latch: true` and re-published at 2 Hz, so the adapter can read
`sensing.cargo_present` (flyto-cloud `services/space_tasks/cargo_vocabulary.py`,
field `cargo_present`, admits only literal True) from a topic.

- `cargo_tray.py`: the one decision. Starts empty; a place loads the tray only
  with a fresh (<= 1 s) `/odom` pose within the station tolerance and the chassis
  still; unknown pose -> `robot_pose_unknown`, away -> `robot_not_at_station`,
  moving -> `robot_not_parked`. Cargo stays with the robot; only unload clears.
- `arm_place_signal.py`: rising edge of `inside` from the e2e host's
  `GET /sim/truth` on `E2E_SIM_CONTROL_PORT` (default 7102). Already-inside at
  start never counts; unreachable or non-boolean readings are no signal.
- `rosbridge_link.py`: minimal advertise/publish/subscribe link that restores
  both after reconnect. Never commands motion.
- Loopback HTTP control on `FLYTO_SIM_CARGO_BRIDGE_PORT` (default 7110):
  `GET /sim/cargo`, `POST /sim/cargo/place`, `POST /sim/cargo/unload`.

## Why

The twin has no tray sensor, and the arm world (MuJoCo) and the moving base
(Gazebo) are separate simulations. `/sim/truth` already exists in Cloud's e2e
harness, so the arm signal needs no Cloud change; the HTTP place endpoint is
for glue without a truth port (`--arm-source http-only`).

## Verified

- `make verify PYTHON=<flyto-robotics/.venv/bin/python>`: `1969 passed, 19 skipped`, ruff `All checks passed!`.
- `tests/test_cargo_bridge_sim.py`: 22 tests, including a real websocket server speaking rosbridge v2 (no Docker).
- `flyto-index verify --strict`: no FAIL or WARN.

## Not verified

- Not run against the live twin (19090 was down) or a real rosbridge_server;
  that rosbridge Jazzy maps `latch` to transient-local is from its source,
  not observed here.
- Not run against a live `/sim/truth` (did not poll it: each read steps the
  MuJoCo world of someone else's running harness).
- Plain `make verify` with the global `python3` fails 9 tests (8 pre-existing)
  because it has no `websockets`; the new real-socket test now skips there.

## Follow-ups

- The adapter-side `sensing.cargo_present` read (freshness window, typed
  no-reading error) belongs to another lane.
- The station is given in the odometry frame; read it from `/odom` at the dock.
