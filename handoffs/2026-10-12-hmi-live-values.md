# Live HMI readings: battery, position and navigation as telemetry channels

Owner: claude
Branch: claude/hmi-values-20261012
Date: 2026-10-12
Paired change: flyto-modules-robotics 1.5.0, same branch name (`2026-10-12-hmi-live-values.md` there)

## What changed

- `flyto_robotics/live_telemetry.py` (new): one table, `CHANNELS`, of the
  channels the ROS 2 manifest declares, and the pure functions that build
  their samples.
  - `power.battery`: `percent` (`BatteryState.percentage` x 100, only if it
    is 0..1; NaN or out of range -> null), `voltage_v`.
  - `pose.planar`: `x`, `y`, `heading_deg`, `frame` (`map` while the map
    transform is fresh, else `odom`). `goal_x`/`goal_y` are added only in the
    map frame. `view_x_min`..`view_y_max` is a window that holds both points
    (at least 2 m either side of its centre, 1 m margin).
  - `nav.goal`: `state` (idle, navigating, arrived, blocked, cancelled,
    failed), `distance_m` and `progress_pct` from Nav2's
    `distance_remaining`. Nav2's 0 before it has a path is ignored, and the
    baseline is the largest distance seen. Also the goal, and the ended goal's
    `motion_outcome` reason. `ENDED_STATES` maps a reason to a state.
    `_rule_reached` or a SUCCEEDED status means `arrived`.
- `generic_ros2_adapter.py`:
  - `_ObservationState` keeps the latest battery reading and the latest
    navigation goal (`NAVIGATION_GOALS` table: `motion.navigate` x/y, and the
    last pose of `navigate_through_poses`). It updates them from
    `_begin_track`, `_track_feedback` and `_motion_summary`. `_forget_track`
    marks the goal cancelled. `telemetry_readings()` returns all of it.
  - The rosbridge backend (presence and full) subscribes to
    `FLYTO_ROS2_BATTERY_TOPIC` (`/battery_state`, BatteryState, best effort,
    throttle 1000 ms). The rclpy node subscribes too.
  - A battery message is never a graph event. A dropped connection forgets
    the battery reading.
- `adapter_provider.py`:
  - The manifest declares `live_telemetry.channel_declarations()` (was `[]`).
  - `telemetry_sampler` and `telemetry_interval_seconds = 1.0` are set on
    `discover_resource_manifests`. The sampler reads the presence watch and
    every adapter `build_adapter` built (a WeakSet), only while each one is
    connected. It opens no connection and sends no command.
  - `MANIFEST_EXTENSIONS` adds `presentation`. On request the manifest
    carries the block from entry point `robotics` in group
    `flyto2.resource_presentations` (the module pack). If the pack is
    missing or fails to load, the field is left out.
- Tests: `tests/test_live_telemetry.py` (new). Existing tests updated:
  `test_module_pack_join.py` (extensions tuple) and
  `test_presence_on_state.py` (the presence connection also carries
  `/battery_state`).
- Version 0.8.0. `.env.example` documents `FLYTO_ROS2_BATTERY_TOPIC`.
  `CHANGELOG.md` and `STATE.md` are updated.

## Why

The HMI is server-driven. The pack declares panels over the resource's
telemetry channels, Cloud passes them through, and the App renders them. Before
this change the manifest declared `telemetry_channels: []`, so a panel had
nothing real to show. Cloud's execution host already runs a provider
`telemetry_sampler` (`local/resource_state_telemetry.py`) and already asks for
the `presentation` extension, so flyto-cloud needed no change.

Rejected:
- Subscribing the presence connection to `/tf` for a map pose. A throttled
  `/tf` drops the map->odom messages, and the job adapter already has `/tf`.
- Subscribing to Nav2's hidden `_action/feedback` topics. The job adapter
  already receives feedback for the goals it sent.
- Importing the pack. The two packages stay independent, joined by an entry
  point.

## Verified

- New tests were written first and failed (module missing). After the
  change: `make verify` passed, ruff clean, 1696 passed, 1 skipped, and
  every dry-run/contract target passed.
- `flyto-index verify . --strict`: PASS, 20 pass, 0 warn, 0 fail.
- Cloud contract cross-check, importing Cloud's main checkout read-only with
  `-B` and the cloud venv. The manifest with `module_pack` + `presentation`,
  after `with_connection_channel`, validates as `ResourceManifestDTO`
  (channels power.battery, pose.planar, nav.goal, connection; 12 panels).
  Each sample passes the pump's `envelope()`.

## Not verified

- Nothing ran against the twin or the physical robot. These are unconfirmed:
  - TurtleBot3's real `BatteryState.percentage` scale. If it is not 0..1,
    `percent` stays null and only voltage shows.
  - That rosbridge on the robot delivers `/battery_state` with the QoS asked.
  - Live Nav2 feedback values.
- No App render from Cloud.
- Nothing is deployed to the demo host. Its installed flyto-robotics is
  unchanged, so the demo shows no new values until a deliberate install after
  the demo.

## Follow-ups

- After the demo, install flyto-robotics 0.8.0 and flyto-modules-robotics
  1.5.0 on the execution host. Then confirm the battery scale and values on
  the robot.
- flyto-app's `mobile_robot.json` fixture uses `power.battery.value` and
  `nav.goal.distance`. The real fields are `percent` and `distance_m`.
