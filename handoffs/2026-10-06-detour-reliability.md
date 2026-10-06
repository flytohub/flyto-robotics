# The inflation escape no longer narrows an obstacle at an unread LiDAR bin

Owner: claude
Branch: claude/detour-reliability
Date: 2026-10-06

## What changed

- `flyto_robotics/inflation_escape.py` (0.6.1)
  - `obstacle_cluster`: an unread bin no longer ends the obstacle's extent.
    Unread bins inside the object, or just past its last return, count as the
    object (`Beam.seen=False`) at the last return's range. This holds only
    within the `CLUSTER_JUMP_M` arc from that return, so a long unread run
    (open space beyond range) is not taken for the object.
  - `plan_escape(..., arrival_tolerance_m=)` is a required keyword argument.
    The lateral waypoint lies at least `arrival_tolerance_m + margin_m` from
    where the back-off ends.
  - `read_arrival_tolerance(reader, nodes)` returns the largest
    `<plugin>.xy_goal_tolerance` over the controller's `goal_checker_plugins`.
    It falls back to 0.25 m (Nav2's default).
- `flyto_robotics/generic_ros2_adapter.py`: `_escape_decision` reads the
  tolerance from `FLYTO_ROS2_CONTROLLER_NODES` (default `/controller_server`).
- Evidence additions (no existing keys changed): `navigation_escape.arrival_tolerance_m`
  and `navigation_escape.obstacle.unread_bins`.
- Tests: `tests/test_inflation_escape.py`, plus the fixture
  `tests/fixtures/twin_box_sweep_2026-10-06.json`, which is the twin's
  recorded post-stop sweep from task t-cab65d9933087004.

## Why

The test was ten twin runs of "advance 120 cm, photo, go round obstacles" on
production ca2142f28 with robotics 0.6.0. The scene was a 0.30 x 0.40 m box
0.42 m ahead. Every advance was stopped by the braking guard. Cloud's recovery
then sent one `motion.navigate` to the advance's end, and the adapter planned
the escape: back-off, lateral waypoint, then the goal.

- The local Desktop evidence holds the adapter's `navigation_escape` for 8 of
  the 9 successful navigates.
  - 7 have an obstacle extent of about ±0.20 m and waypoints at 0.39–0.41 m.
    Their goal leg took 15–18 s with a single FollowPath.
  - Run 4 (t-1fb4b453fb93a2a8) has `bearing_max 0.2032`, `lateral_max 0.085`
    and only 15 returns, so the waypoint was 0.285 m. In its goal leg the
    controller logged "Failed to make progress" 12 times, and the behaviour
    server ran spin, wait, backup, spin. It took 165 s.
- Run 9 (t-cab65d9933087004) had its waypoint at (-0.19, 0.22).
  - bt_navigator reported it "succeeded" in 20 ms: it was 0.222 m from the
    robot, inside `goal_checker.xy_goal_tolerance` of 0.25.
  - The goal leg therefore started right behind the box. The controller
    logged "Failed to make progress" 14 times and the leg was aborted after
    180 s with error 105.
  - The adapter labelled it `obstacle_blocked` because of a 0.45 s
    collision-monitor APPROACH during Nav2's own recovery backup.
- Replaying the recorded sweep through the 0.6.0 code, with one face bin
  unread, gives 0.226 m (bin ~4°) and 0.284 m (bin ~13°). This code gives
  0.41 m for both.
- The braking guard did not take part: it is armed only for advance and
  retreat, and logged nothing during any navigate. Nav2's collision monitor
  fired once, during the run 9 recovery backup.
- Rejected:
  - Adding the full arrival tolerance on top of the extent (0.65 m in this
    room). It leaves the side path 0.33–0.35 m from the walls, below the floor
    on both sides, so the escape would be refused.
  - Lowering any floor.

## Verified

- `make verify` (lint, 1560 passed / 1 skipped, assets, dry runs, contracts,
  pairing, grant): exit 0.
- `flyto-index verify --strict`: all checks PASS, exit 0.
- The old and new code compared on the recorded sweep, as above.

## Not verified

- No new twin or physical run. The twin was left untouched because the
  Desktop adapter was using it.
- The claim that run 4/9-style failures go away rests on replaying the
  recorded scene and on the 8 successful runs with 0.39–0.41 m waypoints, not
  on new runs.
- How Nav2 behaves from a waypoint at `tolerance + margin` (0.35 m) beside a
  thin post was not driven.

## Follow-ups

- flyto-cloud pins b46379d (0.6.0) in `requirements*.txt` by archive URL plus
  sha256. The pin can move only after this branch is pushed and merged.
- `motion_outcome._reason`: any collision-monitor stop during a goal outranks
  Nav2's no-progress code, including a sub-second APPROACH raised during
  Nav2's own recovery backup. Run 9 was reported as `obstacle_blocked`
  (Cloud: detour-able) when it was `no_progress`. Consider weighing the event
  against the progress checker's window.
