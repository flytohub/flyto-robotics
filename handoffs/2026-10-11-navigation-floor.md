# The floor holds where a navigation leg ends and while it runs (0.6.7)

Owner: claude
Branch: claude/nav-floor-0.6.7 (off ff17794, 0.6.6, the flyto-cloud demo pin), merged into release/0.6.7; ported to main as 0.7.1
Date: 2026-10-11

## Defect (live, real robot, robotics 0.6.6)

flyto-cloud handoffs 2026-10-10-demo-machine-and-robot.md ("Detour finding")
and 2026-10-10-start-spinner-and-decision-tree.md (Follow-ups 3). Run c6d8d79d,
2026-10-09 20:35: a person stood in front of the robot. The advance stopped
correctly at 0.552 m. The inflation escape backed off 0.048 m and moved 0.35 m
left. The goal leg then navigated to the blocked move's original destination,
about 0.1 m in front of the person's shins (11 returns, 0.27 m wide). Minimum
range during the leg 0.135 m, `floor_held: false`; Nav2's collision monitor
only slowed.

Causes: nothing checked that a navigation leg's destination keeps the floor,
and the floor was enforced on straight drives only, never during a Nav2 leg.

## What changed (0.6.7)

- `path_clearance.judge_destination` / `DestinationClearance`: every LiDAR
  return placed in the robot frame (from `map_pose`) and measured against the
  leg's destination; one closer than the floor refuses the leg with
  `reason_code: destination_blocked` and `destination_clearance`. Unreadable
  sweep or map pose -> `path_unreadable`. The adapter judges the call's goal
  before anything moves (`refused`), and in an escape the waypoint after the
  back-off and the goal after the waypoint, each from a fresh sweep (`failed`,
  detail `inflation escape <leg> refused: ...`). Never `completed`.
- `braking_envelope.leg_floor_check` + `_ObservationState.arm_floor_guard`
  (`GUARD_KIND_LEG`): every Nav2 leg (plain goal, escape back-off, waypoint,
  goal) runs under the braking guard's loop. Per scan: a return inside the
  floor in any direction (`inside_floor`) or the room the way the base moves
  (sign of odometry linear speed) within its stopping distance
  (`no_room_to_stop`) -> zero, cancel, zero. Blind/quiet LiDAR stops it as for
  straight drives. Result `failed`, `motion_outcome.reason: obstacle_blocked`,
  `braking: {kind: navigation_leg, tripped: floor_breach, trip, minimum_range_m}`,
  also when the server then reports success (`GUARD_TRIP_FLOOR` outranks status 4).
- `inflation_escape`: the lateral waypoint and its heading bound clear the
  obstacle by floor + margin (was robot radius + margin). Without this the
  floor guard would stop the goal leg as it passed the obstacle (0.2 m beside
  it). Max lateral (1.0 m) unchanged, so wider obstacles reach
  `no_escape_room` sooner.
- Floor value: `_clearance_floor()` unchanged (`FLYTO_ROS2_MIN_CLEARANCE_M`,
  default 0.35). Nothing lowers it.
- `sim/twin/tools/destination_probe.py`: twin-only probe (forces simulation mode).

## Verified

- New tests `tests/test_navigation_floor.py` (18) fail on ff17794 (all 18:
  missing functions, the incident navigate completed via backoff/waypoint/goal)
  and pass here. Escape tests updated to the floor clearance (3 changed
  expectations; they fail on 0.6.6 as intended).
- `make verify` (repo venv): 1639 passed, 1 skipped; lint, assets, dry runs ok.
- `flyto-index verify . --full-scan --strict`: pass.
- Twin (separate container from `flyto-turtlebot3-twin:jazzy`, random port,
  default turtlebot3_world; the shared `flyto-turtlebot3-twin` untouched):
  navigate to 0.15 m in front of the nearest pillar. 0.6.7: `refused`,
  `destination_blocked`, clearance 0.1499 m, 7 returns inside the floor, no
  goal sent. 0.6.6 on the same twin: goal sent, still driving at the adapter
  deadline 0.370 m from the pillar.

## Not verified

- The floor guard on the twin or on the robot: covered only by the rosbridge
  transport over a fake socket (trip -> zero/cancel/zero, succeeded-after-trip
  -> failed). Not run on the physical robot.
- `task validate`: ruff passed; its pytest step cannot import the package in
  a worktree (runs the indexer's own interpreter), so the suite was run with
  `make verify` instead.
- flyto-cloud's `stop_reasons.STOP_REASONS` has no row for
  `destination_blocked`; Cloud reads it as no obstruction and plans no
  detour. Add `StopReason("destination_blocked", obstruction=True,
  no_passage=True)` there when the pin moves.

## Next

- flyto-cloud pins flyto-robotics by hash; the owner approves moving the pin to
  the release/0.6.7 merge commit (see the PR). Not changed here.
- Demo arrangement (box ~1.0 m ahead, 200 cm command, 1 m clear each side):
  re-run act 1 on the twin and the robot; the escape waypoint is now ~0.45 m
  further out from the box edge than in 0.6.6.
