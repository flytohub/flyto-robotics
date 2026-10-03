# Adapter waits end on ROS messages, not on spins or sleeps

Owner: claude
Branch: claude/sd-r1
Date: 2026-10-03

## What changed

`flyto_robotics/generic_ros2_adapter.py`:

- Both backends share `_ObservationState`: callbacks store readings under one
  `threading.Condition` and notify it. `observation(required=...)` returns
  cached fresh readings at once and waits on the condition only for a missing
  key (`pose`, `range` unless operator_present, `map_tf` for a navigate),
  capped by `FLYTO_ROS2_OBSERVATION_WAIT_SECONDS`.
- `RclpyROS2Backend` runs a `SingleThreadedExecutor` on a daemon thread for
  the adapter's life (stopped in `disconnect`, restarted in `reconnect`). The
  fixed `_spin` calls are gone from `observation`, `discover` and
  `_publish_zero`; `_spin_until` became `_wait_future` (done callback plus an
  Event). Action clients are cached per capability so rclpy's polling
  `wait_for_server` is only reached when the server is genuinely not matched.
- `RosbridgeROS2Backend.discover()` is cached until reconnect, a reader-thread
  drop, an action send failure or a failed goal; `GenericROS2Adapter.invoke`
  re-reads it once on a capability miss, and `_publish_zero` re-reads when the
  cache lacks cmd_vel. The keepalive waits on an Event that `disconnect` sets.
- Odometry keeps its twist; `wait_until_stationary(max_seconds)` returns after
  3 consecutive still samples (or pose deltas when there is no twist) and
  reports `drifting` at the cap.
- `GenericROS2Adapter.supports_shared_connection = True`, plus `connected`,
  `add_connection_listener` and `wait_until_stationary` for hosts that keep
  one adapter warm (flyto-cloud `local/robot_adapter_registry.py`, same branch
  name).

## Why

The rclpy path spun to fixed deadlines (about 1.6 s per judged motion), a
navigate on a fresh connection was refused when LiDAR arrived before the map
transform, and rosbridge asked rosapi for the graph 8-10 times per motion.

## Verified

- `make verify` in this worktree: exit 0, `1009 passed`.
- `tests/test_observation_on_state.py` (18 tests) replaces the adapter's
  clock and `time.sleep` and a fake rclpy whose `spin_once` raises.
- `flyto-index verify --strict`: no FAIL or WARN.

## Not verified

- No run against a real ROS 2 graph, the TurtleBot3 or the Gazebo twin. The
  rclpy executor and done-callback behaviour is exercised only with fakes.

## Follow-ups

- Needs a flyto-robotics release and a flyto-cloud pin bump, then a Desktop
  release, before the warm adapter is used on a computer.
- `adapter_provider.discover_resource_manifests` still builds and disconnects
  its own adapter per discovery pass.
