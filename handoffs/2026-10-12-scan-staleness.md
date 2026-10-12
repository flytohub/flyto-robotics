# A late scan stops a drive only when the floor needs it; rosbridge reads less

Owner: claude
Branch: claude/scan-stale-0.8.2
Date: 2026-10-12

## What changed

0.8.2.

- `flyto_robotics/braking_envelope.py`: `ScanStalePolicy`, `ScanStaleLimit`,
  `scan_stale_limit`, `blind_time_to_floor`, `period_credit_s`,
  `LatencyEstimator.nominal_scan_period_s` (median interval),
  `BrakingProfile.scan_period_credit_s` (set by `resolve_profile`, unrounded).
- `flyto_robotics/generic_ros2_adapter.py`: one table, `SCAN_STALENESS`, keyed
  by guard kind; `GUARD_POLL_S` (0.05 s, the waiters' poll, also used in the
  floor bound); `_BrakingGuard.last_room_m` (room on the latest scan, start
  room until then); `_check_scan_fresh` uses `scan_stale_limit` and reports
  `basis`, `rate_limit_s`, `floor_limit_s`, `nominal_period_s`,
  `worst_period_s`, `room_m`, `speed_mps`, `scan_stamp_age_s`.
  `ROSBRIDGE_CONTINUOUS_SUBSCRIPTIONS` replaces the inline list: no raw camera,
  no camera_info, scan unthrottled with `queue_length` 1, odom 100 ms with
  `queue_length` 1, tf unchanged. Other subscriptions unchanged.
- `flyto_robotics/motion_outcome.py`: `STOP_RULES` row `blind_after_planned_stop`
  before `guard_blind`; `StopFacts.reached_commanded`.
- Tests: `tests/test_scan_staleness.py`, `tests/test_rosbridge_subscription_cost.py`,
  two rows in `tests/test_stop_rule_contract.py`.

## Why

Physical robot, 2026-10-12 23:29, 0.8.1, run 88ccbec1 (task t-cde05c1b7431d6f4):
`motion.advance` 2.0 m ended `sensor_stale` at 0.612 m. `~/.flyto/sidecar.log`:
`braking guard stopping cap-7c6bd6de...: stale {'scan_age_s': 0.965, 'limit_s': 0.96}`.

The 0.8.1 rule was `max(FLYTO_ROS2_STOP_SCAN_TIMEOUT_S=0.5, 3 x worst of the
last 50 receipt intervals)`, ages from receipt on the Mac's monotonic clock.
The worst interval was 0.32 s, so the limit was 0.96 s; one 0.965 s gap
tripped it. The rule had no relation to the floor: 0.96 s blind is too long
right in front of an obstacle and needlessly short with metres of room.

Read-only measurement through ws://127.0.0.1:19090 (robot at rest, ~20 s,
then 15 s per topic set): header stamps step 0.100 s (one 0.199 s), receipt
intervals median 0.095 s, p99 0.305 s, max 0.356 s; all 19 receipt gaps over
0.2 s stalled odometry at the same time (largest odom-free span 0.15-0.26 s).
So the sensor kept its rate and the transport held scans. Adding an
unthrottled compressed camera subscription for 15 s dropped odom to 10.5/s and
tf to 10.5/s on that connection: rosbridge on the robot is the bottleneck. The
coordinator read rosbridge_websocket at 74.8 % CPU, load ~10 on 4 cores.

New rule (straight drives): limit = min(rate cap, floor bound).
- Rate cap: max(configured 0.5, 15 x median interval, 3 x worst interval),
  at most 2.0 s. Live link: max(0.5, 1.5, 0.96) = 1.5 s.
- Floor bound: `credit + (room - stopping_distance(v)) / v - poll`, credit = the
  scan period the measured latency already contains (0.32 s live), v = max of
  commanded and measured speed, room = last scan's room to the floor.
  Derivation: from where the last scan was taken the robot holds v through
  delivery, the decision age s, and the rest of the latency, then brakes, so
  `v (t - credit + s) + v^2/2a <= room`.
- Example (0.1 m/s, 0.035 m beyond the stopping distance): 0.32 + 0.35 - 0.05
  = 0.62 s. Mid-drive with 1 m of room at 0.12 m/s: 1.5 s (rate). With 0.12 m
  of room at 0.12 m/s: ~0.53 s, so the live 0.965 s stall still stops it there.
- Navigation legs keep the 0.8.1 rule (`SCAN_STALENESS[navigation_leg]`): Nav2
  picks their speed and path, so the guard has no speed to bound blind travel
  with. Not loosened, not tightened.

Where the live trip happened: the drive was planned short of a return ahead
(0.612 m travelled, 0.104 m room at rest, Nav2 status 4 succeeded: it went its
planned distance). The stale trip came as it ended and shortened nothing, so
`blind_after_planned_stop` reports `obstacle_blocked` for that case. With the
new rule the same stall near the planned end may still trip (the room there is
about one stopping distance) and is then reported by that row. The exact plan
values of that run were not recorded anywhere I could read (the checkpoint
holds only the summary line), so the 0.627 m planned distance in the contract
row is reconstructed from room + travelled at the default 0.12 m/s.

`cancel=refused`: not Nav2. After the result (status 4) arrives, `invoke()` pops
the goal from `_active_actions`; the host's `_safety_stop`
(flyto-cloud `local/external_capability_dispatch.py`) then calls
`adapter.cancel()`, which answers `OUTCOME_REFUSED` "no active ROS 2 goal"
without sending anything. The guard's own cancel went out earlier from
`_brake` (zero, cancel, zero); DriveOnHeading had already finished. `safe_stop`
publishes zero velocity before and after cancelling every active goal and
does not depend on any cancel being accepted: it is the guaranteed path, and
it reported `completed`.

Rosbridge cost, one full adapter connection, measured read-only:
before ~884 KiB/s, ~72 msg/s (raw camera 756 KiB/s at 0.9/s, scan 98, odom 13
at 18/s, tf 10 at 27/s, camera_info 6 at 15/s, battery 0.4); after ~116 KiB/s,
~48 msg/s. The raw camera fed only the observation bundle's `camera` digest,
which nothing in flyto-cloud or flyto-core reads; `vision.observe` subscribes
to the compressed topic per capture. `observation()["camera"]` is now null on
rosbridge; rclpy unchanged. Presence watch and the job adapter both read odom,
battery and lifecycle (presence at 500 ms/1 s): small, left as is.

## Verified

- Failing first, on origin/main 3504d68 with the new tests copied in: new
  modules fail to import; a behavioural copy reproduces the live trip exactly
  (`{'limit_s': 0.96, 'scan_age_s': 0.965}`) and finds `/camera/image_raw`
  subscribed; the new stop-rule row fails (`sensor_stale`). All pass on the branch.
- Property test: 20 000 random budgets/speeds/rooms, a stop decided one poll
  after the limit rests at or beyond the floor.
- `make PYTHON=<repo .venv> verify` (single process): exit 0, 1789 passed, 1 skipped.
- `flyto-index verify --strict`: exit 0, 20 PASS.
- `flyto-index task validate`: ruff pass; its pytest step cannot import
  `flyto_robotics` from the indexer's own interpreter (environment), so that
  step did not run; the same suite passed under `make verify`.
- Robot rosbridge accepts `queue_length` 1 + `throttle_rate` 100 (read-only
  subscription: odom 8.7/s, scan 10/s).

## Not verified

- The new rule has not driven the physical robot or the twin from this
  branch (no motion was commanded). The simulator-closed-loop CI job is the
  only closed loop that exercises it.
- CPU relief on the robot was not measured (bytes and messages were).
- Behaviour of rosbridge's queue handler under heavy load on the robot.

## Follow-ups (robot side, after 10/14; no robot service was changed)

- Lower the camera's raw publication (640x480 at ~15 fps) or publish only
  compressed; every raw subscriber costs rosbridge a deserialize per frame.
- nav2 lifecycle_manager at ~35 % CPU: check bond heartbeat settings.
- Disable Nav2 servers the demo never uses (docking, route, smoother) if the
  standard profile allows; the profile currently requires velocity_smoother and
  waypoint_follower.
- flyto-cloud's live relay opens its own connection per channel (scan, pose,
  compressed camera at 5 Hz) while someone watches: that camera is ~760 KiB/s on
  the same rosbridge.
- Legs: bound blind travel with the controller's declared max speed, then give
  them the floor bound too.
