# The robot's own cargo sensor: sensing.cargo_present

Owner: claude
Branch: claude/fetch-live-cargo-sensor
Date: 2026-11-01

## What changed

- `flyto_robotics/cargo_sensor.py` (new, pure): `CargoSensor` keeps the newest
  literal-boolean `data` of a `std_msgs/msg/Bool` message with its receipt time;
  `verdict()` answers a value only inside the freshness window, else
  `cargo_reading_missing` / `cargo_reading_stale`; `call_result()` builds the
  `CallResult`.
- `flyto_robotics/generic_ros2_adapter.py`: `ARGUMENTS["sensing.cargo_present"] = ()`;
  `_discovered_capabilities` declares it only when `FLYTO_ROS2_CARGO_TOPIC` is set,
  the backend has `cargo_reading` (rosbridge only) and rosapi lists the topic as
  `std_msgs/msg/Bool`; `_invoke` routes it to `_read_cargo`, which applies
  `_deployment_mismatch` (the motion graph check) before reading.
  `RosbridgeROS2Backend` subscribes the topic (reliable, volatile, unthrottled),
  records it in `_handle_publish`, clears it on reconnect, and `cargo_reading()`
  waits at most the window (never past the deadline) for a fresh message.
- `flyto_robotics/adapter_contract.py`: metadata row (read_only, no permission,
  no safe stop, not cancellable).
- `.env.example`: `FLYTO_ROS2_CARGO_TOPIC`, `FLYTO_ROS2_CARGO_MAX_AGE_S` (commented).

Result shape. Completed: `evidence = {"cargo_present": <bool>, "cargo_reading":
{topic, message_type, max_age_s, age_s, deployment_mode}}`. Failed:
`evidence = {"cargo_reading": {..., "reason"}}`, no `cargo_present`; detail starts
with the reason. Cloud's `capability_results.call_result` turns a completed
call's evidence into `data`, which `result_expectation` judges against
`cargo_vocabulary` (`cargo_present` equals `True`).

## Why

Cloud's `fetch_red_block_return` template (flyto-cloud c0bda2ddf) needs
`sensing.cargo_present` on the mobile resource; no provider declared it, so the
plan could not be bound. The pack side (`robotics.cargo_present`) is in
flyto-modules-robotics, branch `claude/fetch-live-sim`.

Rejected: declaring it whenever the env is set even if the graph lacks the topic
(would let planning bind a capability nothing serves); reading one-shot like
`vision.observe` (cannot tell stale from missing, and a latched topic would
replay an old value); clamping a bad window (configuration mistakes fall back to
the 2 s default instead).

## Verified

- New tests `tests/test_cargo_sensor.py` (fake rosbridge): failed on import before
  the change; 25 passed after.
- `make verify PYTHON=<main checkout>/.venv/bin/python` with `PYTHONPATH` set to
  the worktree: rc=0; ruff "All checks passed!"; pytest `1972 passed, 19 skipped`.
- `flyto-index verify --strict` in the worktree: rc=0, 20 PASS, no FAIL/WARN.

## Not verified

- Not run against Gazebo, the twin's rosbridge or the physical robot. Nothing in
  the twin publishes `/flyto/cargo_present` on this branch.
- The rclpy transport does not offer the capability.
- `task(action='validate')` through the MCP server was not run (the server is
  pinned to another root); the CLI gates above were.

## Follow-ups

- A simulation publisher for `/flyto/cargo_present` (the sim lane), and a live
  host run with `FLYTO_ROS2_CARGO_TOPIC=/flyto/cargo_present`.
- The physical robot needs a real sensor and topic before hardware use.
