# The floor has to hold where the robot comes to rest

Owner: claude
Branch: claude/braking-distance-stop
Date: 2026-10-05

## What changed

- `flyto_robotics/braking_envelope.py` (new, ROS-free): `BrakingProfile`
  (`stopping_distance(v) = v*t + v^2/(2a)`, `required_clearance(v) = floor +
  stopping_distance(v)`, `max_speed(room)` its exact inverse), `resolve_profile`
  (latency = max(configured, measured); decel = min(configured, declared)),
  `LatencyEstimator` (worst recent scan interval, scan delivery delay with clock
  skew dropped), `room_to_floor(sweep, direction, floor)` (for each return at
  `(x, y)` in the frame of travel: `x - sqrt(floor^2 - y^2)` if `x > 0` and
  `|y| < floor`), `nearest_return`, `direction_label`.
- `flyto_robotics/generic_ros2_adapter.py`:
  - `GenericROS2Adapter._arm_braking`: for `motion.advance`/`motion.retreat`
    under `lidar_clearance`, caps the sent speed at `0.8 * v_max(room)`, refuses
    when that is below 0.02 m/s, and arms the backend's guard.
  - `_ObservationState`: scan timing feeds the latency estimate; every scan
    re-judges each armed guard (`_check_guards`): trip when `room <=
    stopping_distance(max(commanded, |odom v|))`, request a slowdown when `0.8 *
    v_max(room)` falls below `0.9 *` the current speed (at most every 0.5 s).
    A sweep that cannot be read trips `blind`; no scan for
    `FLYTO_ROS2_STOP_SCAN_TIMEOUT_S` (>= 3 scan periods) trips `stale`.
  - Rosbridge transport: `_await_result` carries out trips (zero, cancel, zero)
    and slowdowns (a preempting `send_action_goal` with id `<call>#slowN`, the
    rest of the distance along the heading and the slower speed). Results and
    feedback of superseded goal ids are dropped; `cancel` targets the current id.
  - Rclpy transport: trips only (no re-send); start cap applies as before.
  - Declared deceleration read once per connection from
    `FLYTO_ROS2_DECEL_PARAMETER` (default `/velocity_smoother:max_decel`, x of
    the double array). Rclpy `get_parameter` now returns `double_array_value`.
- `flyto_robotics/motion_outcome.py`: `MotionTrack.braking`; reasons
  `obstacle_blocked` / `sensor_stale` from a guard trip (status 5 is not reported
  as `cancelled`); `stop_clearance` on every summary (nearest return at rest,
  bearing and side from the direction of travel, `floor_held`,
  `travel_room_to_floor_m`, `travel_floor_held`); the operator line names the
  side and whether the floor held, and the guard's trip.
- `tests/test_braking_envelope.py` (new, 80 cases). README section, CHANGELOG,
  version 0.5.0.

## Why

Live, 2026-10-05 22:02 local: `motion.advance` 2.0 m on the physical TurtleBot3
ended `obstacle_blocked; error 723; travelled 1.208 of 2.000 m; nearest LiDAR
return at stop 0.181 m (floor 0.350 m)`.

What the code did, and the evidence:

- The 0.35 m floor was only ever a *start* check
  (`_motion_preflight`: whole-sweep minimum, all directions). Nothing in this
  repository watched the scan while a straight drive ran.
- The stop came from Nav2, not from the floor: 723 is DriveOnHeading
  COLLISION_AHEAD, and the robot's journal shows `Running drive_on_heading` at
  22:02:15.643 and `Collision Ahead - Exiting DriveOnHeading` at 22:02:27.145.
  DriveOnHeading checks the local costmap for the footprint (robot_radius 0.10 m
  in `/etc/ros/nav2-burger-flyto.yaml`) along `simulate_ahead_time` (2.0 s) times
  the speed. That distance knows nothing of the floor and is blind to anything
  beside the 0.2 m-wide swept footprint.
- "nearest LiDAR return at stop 0.181 m" is `minimum_range_at_stop_m`, the
  minimum over **all directions**. The direction-aware value
  (`travel_direction_range_m`, a +/-20 degree wedge) was computed but never put
  in the operator line, and the retained Desktop evidence
  (`flyto-cloud/evidence/971401b6-.../evidence.jsonl`) does not hold the sweep.
  So the direction of that return cannot be recovered from the record. By the
  time of a read-only probe at ~22:06 nothing was within 0.449 m and straight
  ahead was clear to 1.97 m (the host's retries at 22:03 and 22:05 still read
  0.155/0.156 m, so something close was moved away in between).
- It is most consistent with a return beside or diagonal to the path, not
  straight-ahead coasting: 1.208 m in 11.50 s at a commanded 0.12 m/s (the
  default; the job named no speed). Coasting 0.169 m past the floor straight
  ahead at 0.12 m/s would need ~1.38 s of latency at the robot's declared
  2.5 m/s^2. Its own stack measures about a quarter of that (scan 10 Hz,
  costmap 5 Hz, behavior loop 10 Hz). A return beside the footprint is exactly
  what DriveOnHeading lets the robot pass until the footprint grazes its cell.
  Reported as the likely cause, **not proven**.

The fix enforces the floor while moving, with physics rather than a tuned
constant, and counts returns the robot would pass closer than the floor.
Rejected: raising the floor (it must never be lowered and raising it does not
address latency), or tuning Nav2's `simulate_ahead_time` on the robot (that is
robot configuration, invisible to the contract, and still blind beside the
footprint).

Values read from the robot (read-only SSH, 2026-10-05 ~22:05):
`/scan` 10.0 Hz, `scan_time` 0.0967 s, stamp-to-receipt on the robot 1-23 ms;
`/odom` 20 Hz; `base_scan` 0.032 m behind `base_link`; behavior_server
`cycle_frequency` 10, `simulate_ahead_time` 2.0; local costmap update 5 Hz,
`robot_radius` 0.10; velocity_smoother `max_decel [-2.5, 0, -3.2]`,
`max_velocity [0.5, 0, 2.5]`; DWB `decel_lim_x -2.5`; collision_monitor
PolygonStop radius 0.10, `FootprintApproach` 2.0 s; OpenCR
`profile_acceleration 0.0`. Defaults chosen: `t_latency` 0.5 s (above the
~0.4 s budget measured on the twin), `a_decel` 0.5 m/s^2 (a fifth of the
declared 2.5, for slip and load).

## Resource identity guard (same PR, still 0.5.0)

Companion to flyto-cloud `claude/resource-identity-guard`
(`src/ui/web/backend/local/served_resource.py`). Live 2026-10-05 21:54: the
adapter on the execution computer was reconfigured from `turtlebot3-twin`
(simulation) to the physical `burger-01` (hardware). A Cloud job for the twin
was built an adapter "for the twin" (the factory labelled itself with whatever
id it was handed), reached the physical robot, ran, and was recorded as the
twin. A simulated movement is auto-run, so the same path could move the robot
with nobody asked.

- `generic_ros2_adapter.configured_resource_id()`: `FLYTO_ROS2_RESOURCE_ID`,
  else `ros2-<host>-<ROS_DOMAIN_ID>` (moved here from
  `adapter_provider._resource_identity`, which now calls it; discovery output
  unchanged).
- `adapter_provider.build_adapter(resource_id)` and `_serve` (process
  protocol `--resource-id`) raise / answer `ResourceNotServed` for any other
  id, including an empty one, before a transport is opened.
- `GenericROS2Adapter.served_identity()` returns `{"resource_id",
  "deployment_mode"}` (`simulation` / `real`, the manifest's vocabulary, which
  the Cloud host maps through `DEPLOYMENT_CLASS_OF_MODE`). The resource is the
  configured one, never the id the adapter was built with. The mode is the
  configured mode confirmed against the graph read fresh
  (`invalidate_discovery` first): simulation iff the marker topic (`/clock`)
  is on it. It raises `ServedIdentityError`, which the Cloud host turns into a
  refusal, when the mode and graph disagree, when the graph shows no
  interfaces, or when simulation is configured with the marker check disabled
  (hardware with the check disabled is reported `real`, the stricter class).
  Also exposed as the process op `served_identity`.
- `tests/test_served_identity.py` (17 tests, fakes only): the live case
  (configured `burger-01` hardware, asked for `turtlebot3-twin` -> refused,
  nothing built), configured id built, empty id refused, derived default id,
  process protocol refusal and `served_identity` op, real/simulation answers,
  answer ignores the built-with id, both mismatch directions, empty graph,
  marker disabled, and an endpoint swap seen on the next answer.
- Verified for this part: `make verify` exit 0 (1496 passed, 1 skipped);
  `flyto-index verify --strict` 20 pass / 0 warn / 0 fail; `task validate`
  (indexer interpreter, repo on PYTHONPATH: the shim's PYTHONSAFEPATH keeps
  the repo off pytest's path) ruff pass, pytest pass; `pr-risk` 20/medium,
  its one "breaking" flag is the private `adapter_provider._safe_fragment`
  moving to `generic_ros2_adapter` (no other user).
- Not verified: not run against the twin, the robot or a running Cloud host;
  the Cloud side was read, not executed. `open_rmf_adapter.build_adapter` still
  builds for any `fleet:<name>` id it is given (a fleet resource names its
  target in the id; not changed here).


- `make verify`: exit 0; ruff clean; 1496 passed, 1 skipped (with the identity
  guard; 1479 before it); asset, dry-run and contract targets pass.
- `tests/test_braking_envelope.py` covers the kinematic model (scan sampling at
  10 Hz with three phases, latency, constant decel), 12 speeds from 0.02 to
  0.5 m/s (the declared max). New rule: rests >= 0.35 m at every speed and
  phase. Old "stop at the floor" rule: rests inside it by about
  `v*t + v^2/2a` (at 0.12 m/s, 0.298 m; at 0.25, 0.213 m; at 0.5, -0.05 m).
  Also covered: a speed-scaled approach that never crosses the floor;
  `max_speed` as the inverse of `stopping_distance`; the geometry (ahead,
  beside, behind, retreat, unreadable); the profile combination rules; the
  live record re-read with a side return (`left`, `floor NOT held`,
  `travel_floor_held` true); the implied-latency arithmetic; and, through a
  scripted rosbridge, stop (zero/cancel/zero, `obstacle_blocked`, trip
  numbers), slowdown (preempting goal `#slow1` with the remaining 0.2 m, the
  preempted abort ignored, completed), quiet LiDAR (`sensor_stale`), the
  starting speed cap and refusal.
- **Docker twin** (`flyto-turtlebot3-twin`, 127.0.0.1:19090; `/clock` checked
  on the graph before any motion, `FLYTO_ROS2_DEPLOYMENT_MODE=simulation`), two
  guarded advances toward obstacles:
  1. 2.0 m at 0.2 m/s: one slowdown sent (0.2 -> 0.175 m/s at room 0.159 m,
     remaining 0.89 m), then a trip at room 0.134 m (stopping distance 0.141 m).
     Robot covered 0.009 m after the trip; at rest, nearest 0.404 m right,
     floor held, 0.138 m room left along the path. `latency` came out
     `measured` (0.506 s, twin scan interval up to 0.31 s); declared decel
     2.5 read live through rosbridge `get_parameters`.
  2. 0.5 m requested at 0.2 m/s: start speed capped to 0.121 m/s (room 0.098 m).
     Trip at room 0.064 m, cancel answered status 5, reported `obstacle_blocked`
     (not `cancelled`), 0.007 m covered after the trip, nearest at rest 0.359 m
     right, floor held. The limiting return was diagonal (-49 degrees, 0.388 m),
     the class DriveOnHeading's footprint check does not see.
  The twin robot was left where it stopped (near x=2.55, y=-1.62 odom).
- `flyto-index verify --strict`: 20 PASS, 0 WARN, 0 FAIL.
- `flyto-index task validate` (indexer interpreter, repo on PYTHONPATH): ruff
  pass, pytest pass, overall pass.
- `flyto-index pr-risk` on the uncommitted diff: 55/high on keyword heuristics
  ("API routes", "auth/security"); no route, auth or DB code is touched. All four
  suggested test files pass.
- MCP `task(action='plan')` was run but its gates could not proceed, because
  the MCP server is pinned to flyto-indexer's own root (no symbols for this
  repo). Exploration and impact were done with the `flyto-index` CLI
  (`context`, `impact` on `summarize`, `describe`, `_action_goal`,
  `RosbridgeROS2Backend.invoke`, `_update_scan`).

## Not verified

- **Not run on the physical robot** (no motion was commanded; access was
  read-only). Not known until it is:
  - the real stop distance after a trip over WiFi rosbridge. The twin brakes
    almost instantly (7-9 mm), so the 0.5 m/s^2 / 0.5 s defaults are
    unconfirmed on hardware. `braking.trip.measured_stop_distance_m` is
    recorded for exactly this calibration.
  - Nav2 Jazzy DriveOnHeading preemption on the robot (exercised on the twin's
    Nav2, same Jazzy packages).
  - whether a 0.5 s scan timeout produces false stops over the robot's WiFi
    link (`FLYTO_ROS2_STOP_SCAN_TIMEOUT_S` to raise it).
- Clock skew: scan delivery delay is measured from header stamps against this
  computer's wall clock. On the twin, sim-time stamps are dropped as skew, so
  the transport term was 0 there. On the robot it depends on NTP.
- The slowdown window is short at these speeds. `v_max(room)` only binds near
  the floor (at 0.2 m/s with the defaults, a room of 0.163 m down to 0.14 m), so a
  drive mostly approaches at its commanded speed and the guard stop does the
  rest. The stop itself rides the base's own deceleration ramp.
- The direction of the live 0.181 m return: no sweep was retained.
- The on-robot mission controller (`flyto_robotics/mission.py`, the legacy
  runtime path, not used by the external adapter) still stops on a fixed
  `obstacle_stop_distance` with no velocity term.

## Follow-ups

- Release flyto-robotics 0.5.0 from this branch after review/merge. Then bump
  the flyto-robotics pin in `flyto-cloud/src/ui/web/backend/requirements.txt`,
  `requirements-worker.txt` and `requirements-local.txt` (currently archive
  `ea7928d`) so Desktop's AI Space host loads it. No Cloud code change is
  needed: every field is additive and nothing in flyto-cloud or
  flyto-modules-robotics parses `motion_outcome` beyond passing it through.
- With a person present: one guarded advance toward a box at 0.12 m/s and one
  at 0.22 m/s. Read `braking.trip.measured_stop_distance_m` against
  `stopping_distance_m`, and set `FLYTO_ROS2_STOP_LATENCY_S` /
  `FLYTO_ROS2_STOP_DECEL_MPS2` for that resource only if the measurement asks
  for more.
- Consider giving `mission.py` the same envelope.
