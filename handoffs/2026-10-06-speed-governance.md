# A straight drive's speed is governed the way its server can take it

Owner: claude
Branch: claude/speed-governance (off origin/main 2c2ad06, 0.6.3)
Date: 2026-10-06

## Root cause, with evidence

The braking guard (0.5.0) slowed a straight drive by sending a slower goal that
preempted the running one. Nav2's straight-drive behaviors do not support
preemption, in Jazzy or in navigation2 `main`:

- `nav2_behaviors/timed_behavior.hpp` in the twin (Jazzy, nav2_behaviors
  1.3.13) and on navigation2 `main` (read through the GitHub API, 2026-10-06):
  `if (action_server_->is_preempt_requested()) { RCLCPP_ERROR(... "Received a
  preemption request for %s, however feature is currently not implemented.
  Aborting and stopping"); stopRobot(); ... terminate_current(result); }`. The
  action server then starts the pending (slower) goal from rest.
- Twin `~/.ros/log/behavior_server_230_1791281724246.log` (read-only), four
  times on 2026-10-06 at 19:32:35, 20:13:28, 20:20:19 and 20:26:51 local:
  `Running drive_on_heading` / ~0.6-0.7 s later the preemption error,
  `Aborting handle`, `Running drive_on_heading` (the slower goal, from rest),
  and 0.1-0.2 s later `Canceling drive_on_heading` (the guard's trip).
- Desktop log (`backend-main.log`) for the same instants, e.g. 20:13:28:
  `braking guard stopping ...: clearance {'room_m': 0.0483, 'speed_mps':
  0.0838, 'stopping_distance_m': 0.0489, ...}` and the call ended
  `obstacle_blocked; travelled 0.052 of 1.200 m; ... floor held`. So the floor
  always held, but the designed slowdown was a stop and restart, the advance
  ended early, and Cloud ran the detour.
- The running behavior server has no parameter that says whether a behavior
  preempts: `ros2 param list /behavior_server` on the twin lists
  `drive_on_heading.plugin`, `backup.plugin`, `cycle_frequency`,
  `simulate_ahead_time`, ... and nothing about preemption. A preempted goal is
  aborted in both the supported and the unsupported case, so the status of the
  superseded goal cannot tell them apart either.

## What changed (0.6.4)

- `flyto_robotics/braking_envelope.py`:
  - `PREEMPTION_BY_IMPLEMENTATION` (the one table): `nav2_behaviors::DriveOnHeading`,
    `nav2_behaviors::BackUp`, `nav2_msgs/action/DriveOnHeading`,
    `nav2_msgs/action/BackUp` -> not preemptible.
  - `decide_governance(action_type, *, transport_resends, server_plugin)` (the
    one decision): plugin class first, else action type, undeclared -> planned;
    a transport that cannot send a second goal (rclpy) -> planned. Returns
    `SpeedGovernance(mode, action_type, preemption_supported, source,
    server_plugin)`; modes `planned_before_send` / `preemptive_resend`.
  - `plan_drive(profile, room_m, requested speed/distance, mode,
    speed_fraction)`: `v = min(requested, 0.8 * a(-t + sqrt(t^2 + 2 room/a)))`;
    in planned mode `distance = min(requested, room - (v t + v^2/2a))`.
    Refused when `v < 0.02 m/s` or under 0.01 m would be left to drive.
- `flyto_robotics/generic_ros2_adapter.py`:
  - `GenericROS2Adapter._arm_braking` decides governance, plans, refuses with the
    plan in evidence, arms the guard with `governance` and `plan`, and sends the
    planned speed and distance. `_server_plugin` reads `<action>.plugin` from
    `FLYTO_ROS2_BEHAVIOR_SERVER` (default `/behavior_server`) once per action
    per adapter (a failed read is kept as "not said", which can only make a
    drive planned).
  - `RosbridgeROS2Backend.RESENDS_GOALS = True`, `RclpyROS2Backend.RESENDS_GOALS = False`.
  - `_BrakingGuard` gains `governance`, `plan`, `ended`; `_check_guards` only
    requests slowdowns (and only uses the "cannot even go 0.02 m/s" trip) for a
    `preemptive_resend` guard; `_slow_down` refuses for any other guard. The
    hard stop (room <= stopping distance, blind, stale) is unchanged for both.
  - A drive whose server reports success while its plan was shortened gets
    `ended: planned_stop` and is returned `failed` (both transports).
- `flyto_robotics/motion_outcome.py`: `GUARD_END_PLANNED_STOP`; `_reason` maps
  a succeeded planned stop to `obstacle_blocked`; `requested_distance_m` stays
  the caller's request, `commanded_distance_m` added; `describe` adds "ended at
  the stop point planned from X m of room to the floor at send: D m at V m/s"
  and "speed governance <mode> (<source>)" for a tripped or planned-stop drive.
- Evidence (additive, inside `motion_outcome.braking`, which Cloud already
  carries): `governance`, `plan`, `ended`. Cloud's reader is the operator line
  (`describe`) and `motion_outcome.reason`, which flyto-cloud's
  `legacy_host_evidence/host_dispatch.FAILURE_PATHS` and
  `stop_reasons.STOP_REASONS` (`obstacle_blocked` -> `detour=True`) consume;
  no Cloud change needed.
- `tests/test_speed_governance.py` (new, 22 tests); `tests/test_braking_envelope.py`
  slowdown test now arms a declared-preemptible governance (`PREEMPTIVE`).
- README braking section, CHANGELOG, version 0.6.4.

## Design notes for the next person

- Planned distance and guard agree by construction: the guard trips at `room <=
  stopping_distance(v)`, the plan ends at exactly that room. Whichever acts
  first, the call reports `obstacle_blocked` (guard trip, or planned stop).
- Room is `braking.room_to_floor` (the same function the guard uses), not the
  preflight's `floor + robot radius` corridor, so plan and guard can never
  disagree about where the floor is.
- The resend path is unreachable on any current Nav2; it stays for an
  implementation declared preemptible in the table.

## Verified

- `make PYTHON=<repo .venv>/bin/python verify`: exit 0; ruff clean; 1596
  passed, 1 skipped (1574 before); asset, dry-run and contract targets pass.
- Property test: 20000 seeded random cases (room 0-4 m incl. 0, latency and
  deceleration over their full clamp bounds, requested speed 0.02-1.0 m/s,
  distance 0-4 m): every non-refused plan rests at or beyond the floor with
  the robot at full speed through the whole latency then braking; plus 150
  time-stepped runs with the guard scanning, at or beyond the floor (2 mm
  integration tolerance).
- Fakes: Nav2 server through rosbridge (one goal only, never a preempting goal
  even where a resend guard would slow; success at 0.86 m of 2.0 m ->
  `failed`, `obstacle_blocked`, `ended: planned_stop`, governance recorded);
  declared-preemptible server (whole distance, `#slow1` resend, completed);
  guard still a hard stop on a planned drive; plugin read and fallback.
- `flyto-index scan . && flyto-index verify --strict` (worktree): 20 PASS, 0
  WARN, 0 FAIL. `task validate` (indexer interpreter, repo on PYTHONPATH):
  ruff pass, pytest pass.
- Read-only: twin behavior_server logs and parameter list, installed Nav2
  header; navigation2 `main` header via GitHub.

## Not verified

- Not run against the twin (read-only access; another loop may drive it) or the
  physical robot (`flyto-robot.local` did not resolve from this computer; its
  `behavior_server` plugin parameter was not read).
- A real drive's natural end overshoot (DriveOnHeading's own stop) is assumed
  to be within the full stopping distance; not measured.
- The rclpy transport's planned path is covered by the shared code and the
  `RESENDS_GOALS` flag, not by an rclpy fake.

## Follow-ups

- Next twin run of the box loop: an advance toward the box should end with
  `braking.ended: planned_stop` (or a guard trip at the same point), no
  "preemption request" line in the behavior_server log, and travel close to
  `room - stopping_distance(v)`.
- Release: bump the flyto-robotics pin in flyto-cloud's requirements to the
  merge commit so Desktop's AI Space host loads 0.6.4.
