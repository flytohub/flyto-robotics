# A motion cut short by what stands ahead never reports completed

Owner: claude
Branch: claude/obstruction-stop-code (off origin/main 7935354, 0.6.5)
Date: 2026-10-07

## Defect (live, twin, robotics 0.6.5)

Task t-41169022a09d95b6: `motion.advance` 1.2 m with a box ahead. Speed
governance (`planned_before_send`) measured `room_to_floor_at_send` 0.0901 m
and shortened the goal to `commanded_distance_m` 0.0211 (`plan.shortened`);
the braking guard also tripped on `clearance`; Nav2 DriveOnHeading succeeded.
`motion_outcome` reported `reason: completed`, `action_status_name:
succeeded`, 0.0345 of 1.2 m travelled. Cloud started no detour and the task
failed as not proven ("unusable resource.arrival"). Runs that stood further
back drove, were tripped mid-drive (status canceled) and reported
`obstacle_blocked`, so the outcome depended on where the robot stood.

Cause: `_reason` returned `completed` for any succeeded status before looking
at the guard; 0.6.4's planned-stop rule (`braking.ended = planned_stop`) was
only set when the guard had *not* tripped.

## What changed (0.6.6)

- `motion_outcome.STOP_RULES` / `StopFacts` / `stop_rule()` / `stop_facts()`:
  one ordered table over requested, commanded and travelled distance,
  `plan.shortened`, the guard trip, and whether the nearest return ahead is
  inside the floor. It runs before any status-based account. Rows and bases:
  - `no_room_to_drive`: commanded < `braking_envelope.MIN_DISTANCE_M` (0.01 m)
    -> `path_blocked`;
  - `reached_request`: planned or guarded stop within
    `mission.relative_move_tolerance` (min(0.03 m, d/10)) -> `completed`;
  - `guard_blind`: trip blind/stale -> `sensor_stale`;
  - `stopped_short_by_clearance`: short by more than the tolerance and
    shortened or tripped on clearance -> `obstacle_blocked`;
  - `stopped_short_facing_obstacle`: short, nearest return ahead inside the
    floor -> `obstacle_blocked`.
  No match -> the old chain (status, Nav2 code, collision monitor).
  `motion_outcome.stop_rule` records the row, its basis and the numbers.
- `REASON_PATH_BLOCKED` added to `motion_outcome.REASONS`.
- Adapter (rclpy and rosbridge): a succeeded goal, or one the rule found
  reached, takes its outcome from the reason (`_succeeded_outcome`): not
  `completed` -> `failed`. `_ended_at_planned_stop` removed.
- A plan refusal (no distance left after stopping) reports
  `reason_code: path_blocked` instead of `obstacle_blocked`.

## Verified

- `tests/test_stop_rule_contract.py` (15 tests): the contract table (defect
  numbers -> obstacle_blocked; same without trip; retreat; zero room ->
  path_blocked; mid-motion clearance / blind trips keep their codes; full
  length -> completed; shortened but within tolerance -> completed), the rule's
  bases, the tolerance identity, outcome mapping, and the defect end to end
  through the rosbridge transport (fake socket) -> `failed`,
  `obstacle_blocked`.
- Run against an unchanged 7935354 (0.6.5) checkout: the defect row, the
  no-trip and retreat rows, the zero-room row and the rosbridge end-to-end
  test fail (`'completed' == 'obstacle_blocked'` / `'path_blocked'`).
- `make PYTHON=<repo .venv>/bin/python verify`: ruff clean, 1621 passed,
  1 skipped. `flyto-index verify --strict`: see the PR.

## Not verified

- Not run on the twin or the physical robot. No motion was sent anywhere.
- rclpy transport covered only by the shared rule and its existing tests, not
  by a new end-to-end test.
- Rotation and navigation have no requested distance, so the rule does not
  apply to them; they keep the status-based account.
