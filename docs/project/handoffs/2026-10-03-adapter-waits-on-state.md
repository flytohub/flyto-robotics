# Adapter waits end on ROS messages, not on spins or sleeps

Owner: claude
Branch: claude/sd-r1
Date: 2026-10-03

## What changed

`flyto_robotics/generic_ros2_adapter.py`:

- Both backends share `_ObservationState`: callbacks store readings under one
  `threading.Condition` and notify it. `observation(required=...)` returns
  cached fresh readings at once and waits on the condition only for a missing
  or stale key (`pose`, `range` unless operator_present, `map_tf` for a
  navigate), capped by `FLYTO_ROS2_OBSERVATION_WAIT_SECONDS`. Stale (older
  than `FLYTO_ROS2_OBSERVATION_MAX_AGE_SECONDS`) counts as missing, and both
  backends forget their readings on `disconnect`/`reconnect`, so a warm
  adapter after an idle gap or a reconnect waits for the next callback
  instead of refusing or passing an old reading off as fresh.
- `RclpyROS2Backend` runs a `SingleThreadedExecutor` on a daemon thread for
  the adapter's life (stopped in `disconnect`, restarted in `reconnect`). The
  fixed `_spin` calls are gone from `observation`, `discover` and
  `_publish_zero`; `_spin_until` became `_wait_future` (done callback plus an
  Event). Action clients are cached per capability so rclpy's polling
  `wait_for_server` is only reached when the server is genuinely not matched.
  `discover()` on a node that has not heard from the robot yet waits for the
  first subscribed message (rmw's graph cache is empty on a new node), capped
  by the observation wait; `_publish_zero` reads the graph without that wait.
  Subscription callbacks are guarded, so one malformed message is dropped
  instead of ending `executor.spin()`; if the executor still dies, the
  backend marks itself disconnected and tells its connection listeners, and
  `is_connected()` (read by `GenericROS2Adapter.connected`) also checks that
  the executor thread is alive.
- `RosbridgeROS2Backend.discover()` is cached until reconnect, a reader-thread
  drop, an action send failure or a failed goal; `GenericROS2Adapter.invoke`
  re-reads it once on a capability miss, `_publish_zero` re-reads when the
  cache lacks cmd_vel, a deployment mismatch is re-checked on a fresh read
  before it refuses, and a preflight whose readings did not arrive drops the
  cache (Nav2 or a driver can restart behind a rosbridge that stays up). The keepalive waits on an Event that `disconnect` sets.
- Per-call state is bounded for a host-lifetime adapter: results and
  execution counts keep the newest `CALL_HISTORY_LIMIT` (256) call ids; rclpy
  goal handles and result futures are dropped once a result or a confirmed
  cancel is known, and on reconnect; rosbridge active goals are dropped on
  reconnect. `safe_stop` on both backends publishes zero velocity first, then
  tries each still-active goal once and forgets it, so an unanswered cancel
  (Nav2 restarted) no longer delays every later emergency stop. When there
  were active goals it publishes zero again after the cancels, so the last
  command on cmd_vel is the stop's even for a motion server that stops
  without a zero of its own (the pre-change order guaranteed that too).
- Connection listeners live on the shared state for both backends; the
  rclpy `reconnect()` announces `True` only when its new executor is alive.
- `RclpyROS2Backend.disconnect()` destroys its node (subscriptions,
  publishers, action clients); `reconnect()` after a disconnect builds a new
  one, while a reconnect after an executor death keeps the node.
- `silent_seconds()` on both backends and on `GenericROS2Adapter`: seconds
  since the robot last sent any subscribed reading (or since listening began),
  None while disconnected. A host reads a long silence as the robot being
  gone even though the transport (an rclpy executor) is still up.
- Odometry keeps its twist; `wait_until_stationary(max_seconds)` returns after
  3 consecutive still samples (or pose deltas when there is no twist) and
  reports `drifting` at the cap.
- `GenericROS2Adapter.supports_shared_connection = True`, plus `connected`,
  `add_connection_listener` and `wait_until_stationary` for hosts that keep
  one adapter warm (flyto-cloud `local/robot_adapter_registry.py`, same branch
  name).

## Round 2: the robot announces itself (review fix)

The review found that a robot coming up after the execution host registered
had no state trigger, so it waited for the host's heartbeat discovery.

- `_ObservationState.add_graph_listener(listener)`: `listener(reason)` runs on
  its own thread, never under the condition. It fires on the first odometry
  of a connection (`odometry_appeared`), odometry after `SILENT_SECONDS`
  (`odometry_returned`), the first map transform (`map_transform_appeared`)
  and every Nav2 lifecycle TransitionEvent (`lifecycle:<topic>`; topics from
  `FLYTO_ROS2_LIFECYCLE_EVENT_TOPICS`, default bt_navigator and
  behavior_server). Each event also drops the cached rosbridge graph. The
  delivery thread ends on `disconnect` and restarts on the next event.
- `presence_only` backends (`build(..., presence_only=True)`) subscribe only
  odometry (rosbridge throttled to 500 ms) and the lifecycle topics.
  `GenericROS2Adapter.invoke` refuses every command on them.
- `adapter_provider.PresenceWatch` / `watch_resources(execution_host_id=,
  notify=)`: one presence connection per resource. It notifies on connect,
  drop, unreachable, graph change, and silence. Silence is a deadline
  re-armed from the last reading. A transport that is not up is retried on a
  1, 2, 5, 10 s back-off: nothing announces a remote rosbridge starting to
  listen, so this is the fallback. A drop is reconnected at once. There is no
  watch when discovery is off, or for rclpy without ROS 2 installed. The
  function is published as `discover_resource_manifests.watch`, so a host
  feature-detects it without a new entry-point group (flyto-cloud
  `claude/sd-r1` uses it).
- `discover_resource_manifests` reads the graph over the watched connection
  (re-read each pass), not over a new node or socket per pass.
- rclpy creates the four motion ActionClients with the node.

## Why

The rclpy path spun to fixed deadlines (about 1.6 s per judged motion), a
navigate on a fresh connection was refused when LiDAR arrived before the map
transform, and rosbridge asked rosapi for the graph 8-10 times per motion.

## Verified

- `make verify` in this worktree: exit 0, ruff clean, `1029 passed`.
- `tests/test_observation_on_state.py` (41 tests) replaces the adapter's
  clock and `time.sleep` and a fake rclpy whose `spin_once` raises. Added
  after review: stale warm readings and reconnect wait for the next callback;
  a new node's discover waits for the first message; a safe stop never waits
  for discovery; a malformed message does not end the executor; a dead
  executor is announced and never announced as back; zero velocity is
  published before cancels and a stale goal is tried once; bounded history.
  Added after the second review: the stop ends on a zero after the cancel
  (also when the cancel times out); disconnect destroys the node and a
  reconnect builds one; silence is measured from the last reading; a cached
  simulator marker is re-read before refusing; missing preflight readings
  drop the cached graph.
- flyto-cloud `tests/unit/local/test_robot_registry_with_ros2_adapter.py`
  drives this adapter's rosbridge backend through the Cloud warm-adapter
  registry (drop, borrow, reconnect) with an in-memory socket.
- `flyto-index verify --strict`: no FAIL or WARN.

Round 2:

- `make verify` in this worktree: exit 0, ruff clean, `1046 passed`.
- `tests/test_presence_on_state.py` (17 tests, `time.sleep` raises): the
  first odometry, odometry after silence, a lifecycle transition and the
  first map transform are announced and drop the cached graph; a reconnect
  announces again; the lifecycle subscriptions exist; a presence connection
  carries odometry and lifecycle only and refuses commands; rclpy subscribes
  lifecycle and creates the action clients with the node. The watch, with a
  scripted wait that records instead of sleeping: connect announced at once;
  a robot not up is tried on back-off and announced once; a drop is
  reconnected without waiting; a failed reconnect is unreachable; silence is
  announced at the deadline and cleared by a reading. Discovery reuses the
  watched connection. The event thread ends on disconnect.
- Live Gazebo twin (rosbridge 127.0.0.1:19090), read-only: the watch
  announced `robot_connected` at 0.023 s and `odometry_appeared` at 0.052 s,
  and discovery over it took 0.015 s with all seven capabilities. A
  TransitionEvent published on `/behavior_server/transition_event` arrived as
  `lifecycle:/behavior_server/transition_event`. Through flyto-cloud's
  dispatcher with `time.sleep` raising: advance 0.1 m and retreat 0.1 m on one
  warm adapter, with `wait_until_stationary` ending in 0.263 s and 0.373 s.
  The twin was put back at (-1.8, 0, yaw 0) with odometry reset.
- `flyto-index verify --strict`: 20 PASS, 0 WARN, 0 FAIL.

## Not verified

- No physical robot. No rclpy transport anywhere (no ROS 2 on this Mac): the
  rclpy executor, lifecycle subscriptions and eager ActionClients are tested
  with fakes only. On the twin: no navigate, rotate, emergency stop,
  power-off, silence or late rosbridge start.

## Follow-ups

- Needs a flyto-robotics release and a flyto-cloud pin bump, then a Desktop
  release, before the warm adapter is used on a computer.
- Without a presence watch (discovery off, or the watch not connected),
  `discover_resource_manifests` still builds and disconnects its own adapter
  for each pass.
- A host that keeps a warm job adapter and a presence watch holds two
  rosbridge connections to the robot. The presence one is light: odometry at
  2 Hz plus lifecycle events.
- Refusal reasons are free text; the Cloud host matches "is not present on
  the ROS 2 graph" and "action server unavailable" to decide on
  rediscovery. A typed reason in `CallResult` would be sturdier.
