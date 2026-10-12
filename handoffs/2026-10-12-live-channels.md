# Live channels for the robot, and the battery percent scale (0.8.1)

Owner: claude
Branch: claude/live-channels-0.8.1
Date: 2026-10-12

## What changed

- `flyto_robotics/live_channels.py` (new): one table, `LIVE_CHANNELS`, and
  `declarations(bridge_url, topics)`, which builds a `flyto.live-channels.v1`
  block:

  | id | kind | transport | max_rate_hz | topic (setting, default) | type |
  |---|---|---|---|---|---|
  | `scan` | scan | ws-json | 5 | `FLYTO_ROS2_SCAN_TOPIC`, `/scan` | `sensor_msgs/msg/LaserScan` |
  | `pose` | pose | ws-json | 5 | `FLYTO_ROS2_ODOM_TOPIC`, `/odom` | `nav_msgs/msg/Odometry` |
  | `camera` | video | ws-jpeg | 5 | `FLYTO_ROS2_CAMERA_COMPRESSED_TOPIC`, `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |

  - `scan` carries `floor_m` from the adapter's `_clearance_floor()` (0.35 m by
    default). Each channel has a `label` `{zh-TW, en}`, the shape the pack's
    presentation block uses.
  - `source` is `rosbridge://<host:port>/<topic>?type=<type>` (`rosbridges://`
    for `wss`), on the bridge URL the adapter itself uses. That is the form
    flyto-cloud's `local/live_source_rosbridge.parse_source` reads.
  - A channel is declared only when the graph the discovery pass just read
    publishes its topic with its type (`/msg/` spelling ignored). The block is
    empty when there is no rosbridge (rclpy), when the bridge URL has a path or
    query, or when nothing matches.
- `adapter_provider.py`: `MANIFEST_EXTENSIONS` adds `live_channels`. On request,
  `_manifest` adds the block. It leaves the field out when the block is empty,
  because Cloud's schema has `minItems: 1` and an empty list would fail
  `ResourceManifestDTO` and drop the robot.
- `generic_ros2_adapter.py`: `GenericROS2Adapter.bridge_url()` and
  `published_topics()`, plus the rosbridge backend's `bridge_url()` (the
  current `_url`, which follows a managed SSH forward) and `published_topics()`
  (a copy of `_topic_types`, empty while disconnected). `_topic_types` is not
  cleared on reconnect, because `_publish_zero` (safe stop) reads it.
- `live_telemetry.py`: `PERCENTAGE_SCALES` (`fraction` [0, 1] x100, `percent`
  (1, 100] x1), `battery_percent()`. `battery_payload` uses it. Anything else is
  null, never clamped.
- Tests: `tests/test_live_channels.py` (new, 28 cases), `tests/test_live_telemetry.py`
  (battery cases 93.88, 0.9388, 0, 1.0, 1.5, 100, 101, 100.01, -0.01, NaN, inf,
  None, True, "93.88"; the table has no gaps or overlaps),
  `tests/test_module_pack_join.py` (extension tuple).
- Version 0.8.1, `CHANGELOG.md`, `STATE.md`, and `README.md` (provider
  discovery paragraph).

## Why

- Live channels: Cloud's host relay, live view route and station Live tab
  were proven only through `flyto-cloud/scripts/live_e2e_host.py --channels`
  with a hand-written block. 0.8.0 declared telemetry channels but no live
  channels, so on the production execution host the Live tab never appeared.
  Cloud's `local/resource_discovery.MANIFEST_EXTENSIONS` already asks for
  `live_channels`, so this repository only had to offer it.
- Battery: burger-01 publishes `BatteryState.percentage` on a 0..100 scale
  (93.88, seen through Cloud's relay on 2026-10-12). 0.8.0 accepted only 0..1,
  so the percent stayed blank.
- Rejected:
  - Declaring the channels unconditionally. A robot without a camera would
    show a tile that never fills.
  - Clearing `_topic_types` on reconnect. It would add a rosapi round trip to
    the first safe stop after a reconnect.
  - A `power` live channel. Battery is already the `power.battery` telemetry
    channel, and Cloud's telemetry live frame reads `percentage` raw.

## Verified

- New tests written first. `test_live_channels.py` failed on import (module
  missing). The battery cases `93.88`, `1.5` and `100`, the scale table test
  and the two extension-tuple pins failed on 0.8.0.
- `make verify` (single process, repo venv): ruff clean, 1758 passed,
  1 skipped. Every dry-run and contract target passed.
- `flyto-index verify --strict` in the worktree: 20 pass, 0 warn, 0 fail.
- `flyto-index task validate`: ruff passed. The pytest step could not import
  `flyto_robotics` (the tool runs an ambient pytest), so it reports fail. The
  main checkout fails the same way, so this is not caused by this change; the
  suite itself ran under `make verify`.
- Cloud cross-check. This was a scratch script, not in the repo. It imported
  flyto-cloud's main checkout read-only (`-B`, the cloud venv) and the
  provider manifest from a `GenericROS2Adapter` over a fake rosbridge graph,
  for both the robot topic set (hardware) and the twin set (+`/clock`,
  simulation), with every extension Cloud asks for. Results:
  - `live_channels_problem` returned None.
  - `ResourceManifestDTO.model_validate` passed, and `live_channels` came back
    unchanged.
  - Each source picked the rosbridge row in `local/live_sources.source_row`,
    and `parse_source` gave `ws://127.0.0.1:19090` plus the topic.
    `normaliser_for` accepted each type for its kind.
  - `redact_manifest` removed every source.
  - An empty list is refused by Cloud, as expected.

## Not verified

- No frame was relayed from the twin or the robot through this provider. Cloud's
  2026-10-12 read-only run (`handoffs/2026-10-12-live-integration-ws-jpeg.md`
  in flyto-cloud) subscribed to the same four topics over 127.0.0.1:19090 with
  a hand-written block. This block was checked against Cloud's parsers, not
  against equipment.
- The twin's compressed camera topic: the 2026-10-02 parity handoff says the
  twin has the robot's topic set. It was not re-read today. If the twin lacks
  it, `camera` is simply not declared there.
- The execution host has not been upgraded. Until the host installs 0.8.1
  (Cloud's pin), production still has no Live tab.
- Production prerequisites outside this repo (Cloud code needs no change):
  - the Space must approve `vision.stream` for the robot (`live_view.CHANNEL_GATES`
    shows `not_approved` otherwise);
  - the host must be online;
  - the relay base must be registered (same-origin on Desktop,
    `FLYTO_LIVE_WS_BASE` for remote and phone viewers);
  - `FLYTO_LIVE_WATCH_KEY` must be shared between Cloud and the host relay.
- With a managed SSH forward on a random local port, a moved port is
  re-declared on the next discovery pass. Until that pass, the relay retries
  the old port.
- Battery: a percent-scale robot at or below 1 % reads as a fraction (1.0 ->
  100 %). One message cannot tell the two apart.

## Follow-ups

- Coordinator: bump flyto-cloud's flyto-robotics pin to the merge commit and
  sha256 reported for this PR.
- After the upgrade, open the station Live tab on the twin first. Check scan,
  pose and camera, and the battery percent on burger-01 (read only).
