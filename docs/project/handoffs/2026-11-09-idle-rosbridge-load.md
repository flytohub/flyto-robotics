# An idle connection no longer makes rosbridge read LiDAR and transforms

Owner: claude
Branch: claude/idle-rosbridge-load
Date: 2026-11-09

## What changed

0.8.4.

- `flyto_robotics/generic_ros2_adapter.py`:
  - `ROSBRIDGE_SUBSCRIPTIONS` (replaces `ROSBRIDGE_CONTINUOUS_SUBSCRIPTIONS`
    and the inline additions in the removed `_subscribe_standard_observations`):
    one `ObservationSubscription` row per subscription, with `held_by` saying
    who holds it.

    | row | topic | throttle / queue | held by |
    |---|---|---|---|
    | odom-presence | /odom | 500 ms / 1 | presence (every connection) |
    | battery | /battery_state | 1000 ms | presence |
    | lifecycle | Nav2 transition_event topics | 0 | presence |
    | collision | /collision_monitor_state | 0 | connected (every full connection) |
    | odom | /odom | 100 ms / 1 | motion, state |
    | scan | /scan | 0 / 1 | motion, state |
    | tf | /tf | 0 | motion, state |
    | command | /cmd_vel_nav, /cmd_vel | 100 ms | motion |
    | clock | /clock (simulation only) | sim_time throttle | motion |

  - `CALL_READINGS`: capability (and the host-called `observe`,
    `wait_until_stationary`) -> `READ_MOTION`, `READ_STATE` or None.
    `motion.navigate*`, `motion.advance/retreat/rotate` -> motion;
    `places.mark`, `observe`, `wait_until_stationary` -> state;
    `motion.halt`, `vision.observe`, `sensing.map`, `places.list` -> nothing.
    Unlisted -> motion (holds the most).
  - `RosbridgeROS2Backend.hold_readings(kind)` (ref-counted context manager),
    `_sync_subscriptions` (diffs wanted vs held, unsubscribes by the id that
    subscribed, then subscribes), `subscribed_topics()`. Motion rows also stay
    held while a goal this connection sent has no result yet (a timed-out
    call); an `action_result`, `cancel` and `safe_stop` re-sync. Reconnect
    re-subscribes whatever is held on the new socket.
  - `GenericROS2Adapter.invoke/observe/wait_until_stationary` run under
    `_holding(_call_reading(...))`; `RosbridgeROS2Backend.observation()` also
    holds `READ_STATE` while it waits (nothing changes inside a call; a tool
    that reads the backend directly, like the twin's scenario runner, gets
    readings sent after it asked). rclpy has no `hold_readings` and is
    unchanged.
  - Freshness: `_ObservationState._require_fresh(kinds)` is called just before
    a row is (re)subscribed; `_missing` and `_snapshot` count pose, range and
    map_tf only from messages that arrived after it. New
    `REQUIRE_SCAN_TIMING`: on a transport with `HOLDS_READINGS_PER_CALL`
    (rosbridge), a motion preflight also waits for 3 scan intervals measured
    since the scan subscription, so the braking guard's latency budget is
    measured as before; missing it refuses nothing (configured latency still
    applies, as on any first scan). `observe()` waits for map_tf too once the
    connection has heard localization (`has_localized`).
- `flyto_robotics/braking_envelope.py`: `LatencyEstimator.break_chain()` (the
  gap while unsubscribed is never an interval; measured history is kept) and
  `intervals_measured`.
- Tests: `tests/test_rosbridge_subscription_cost.py` rewritten (18 tests);
  `test_goal_stalls`, `test_sim_time`, `test_presence_on_state`,
  `test_observation_on_state`, `test_lidar_sweep` updated to take readings under a hold or from a
  socket that streams after subscribe.
- Version 0.8.4 (`setup.py`, `flyto_robotics/__init__.py`), CHANGELOG 0.8.4.
  The entries still under "Unreleased" (arm M1 etc.) are in the same wheel.

## Why

Measured 2026-11-09 on burger-01 (Pi 4), read-only: rosbridge 513-516 ticks
per 10 s (~50 % of a core) with the robot idle. `ss -tni` on :9090 showed two
established clients: one sent 1.21 MB in 10 s (~121 KB/s), the other ~19 KB
(~1.9 KB/s).

Why two connections from one backend: they are not two full connections.
`ros2 node info` merges every client's subscriptions, so it cannot tell them
apart; the byte counts can. They are, by design:

1. flyto-cloud `src/ui/web/backend/local/adapter_registry.py` keeps one warm
   full adapter per (execution host, resource) for the life of the host
   (module docstring lines 1-7, `borrow` 125-171); `_connection_changed` /
   `_recover` (216-299) reconnect it on their own when the robot reboots,
   which is why it was up "at robot boot" with no job running.
2. This repo's `adapter_provider.PresenceWatch` (`watch_resources`) holds one
   presence-only connection.

Nothing in this repo leaked a connection (reconnect/`reopen` disconnect the old
socket first; discovery's own full build is disconnected in `finally`). The
cost was that the warm full connection held /scan, /tf (42 Hz, unthrottled)
and 10 Hz odom continuously. Fixed here, at the subscription table, so it
holds no matter how long a host keeps a connection warm. flyto-cloud is not
changed.

Rejected: throttling /tf (rosbridge throttles per topic; odom->base messages
would crowd out map->odom); dropping the collision-state row (it publishes
only on change, so it costs nothing idle, and a motion is seeded with the
state in force when it began, which a subscription taken at the call cannot
know); a linger timer after a call (adds a timer; re-subscribing is cheap).

## Verified

- Tests that fail on origin/main (86d67a6) and pass here: the new
  `test_rosbridge_subscription_cost.py` (import error on main; behaviourally,
  a probe of the idle-topic test fails on main because /scan, /tf, /cmd_vel*
  are subscribed at connect), `test_goal_stalls::...watched_topics...` and
  `test_presence_on_state::...lifecycle_topics` fail on main.
- `make verify` (final tree): rc 0; ruff "All checks passed!"; pytest
  1960 passed, 19 skipped; no `"passed": false` in the dry-run/contract
  targets.
- `flyto-index verify --strict` in the worktree: PASS, 20 pass, 0 warn, 0 fail.
- `flyto-index task validate`: ruff pass; its pytest step fails at collection
  with `ModuleNotFoundError: No module named 'flyto_robotics'` -- identical on
  the main checkout, so it is the validator's interpreter, not this change.
- Twin closed loop, `./scripts/run_twin_scenario.sh
  scenarios/twin/demo-room-obstacle-stop.json` (Docker, Jazzy image built
  locally, arm64): first run failed -- the scenario runner reads
  `adapter.backend.observation()` directly and got nothing while idle, which
  is why `observation()` now holds `READ_STATE`. Second run
  (`results/twin-scenario/idle-rosbridge-20261109b`, untracked): rc 0, verdict
  `passed: true`, all 10 checks pass (outcome, motion_reason obstacle_blocked,
  floor_not_lowered, stopped_outside_floor 0.401 m vs 0.35 m floor,
  goal_count 1, no goal or command after result, held still). The braking
  profile carried `measured_latency` (scan_period 0.1088 s) from the scans
  taken for the call; call 5.24 s.

## Not verified

- Not run on burger-01: no motion was commanded. The idle cost on the robot
  (expected: rosbridge carrying only 2 Hz odom + 1 Hz battery + lifecycle per
  connection, ~2 KB/s each, and no ROS subscription to /scan or /tf while no
  Live view is open) is an expectation from the per-topic bytes, not a
  measurement. The robot went offline during this session (battery 33 %).
- The time a motion preflight now waits after subscribing on the real Pi
  (rosbridge creating the ROS subscriptions, 3 scan intervals, AMCL's next
  map->odom) is unmeasured; it is bounded by
  `FLYTO_ROS2_OBSERVATION_WAIT_SECONDS` (3 s) and refuses as before when
  readings do not come.
- Idle, the `pose.planar` telemetry is the odometry pose (frame `odom`); while
  a call runs it is the map pose. Between calls in a job the HMI pose frame can
  change. HMI Live view (flyto-cloud `live_source_rosbridge.py`) uses its own
  connection and the unchanged `live_channels` manifest: not affected, but not
  exercised against a robot here.

## Follow-ups

- Measure on burger-01 after deploying 0.8.4 to the owner's Mac: rosbridge
  ticks per 10 s idle, bytes per client, and preflight wait during a short
  `motion.rotate`.
- If an idle map pose is wanted on the HMI: read Nav2 AMCL's `/amcl_pose`
  (transient_local, published only on update) as a presence row instead of /tf.
