# Motion outcome: adversarial verification of #44/#45

Owner: claude
Branch: claude/verify-abort-reason
Date: 2026-10-04

## What changed

Two defects in the merged motion-outcome work (#45) were confirmed with
failing tests against merged `main` (6fb2d16) and fixed:

1. **A cleared collision-monitor event outranked the real reason.**
   `motion_outcome._reason` treated any STOP/APPROACH event seen during the
   run as `obstacle_blocked`, ahead of every Nav2 error code. A navigate that
   slowed for a box early on the route and later failed with "no valid path"
   (208) was reported as blocked. The monitor also publishes only on a change,
   so a stop already in force when the goal was sent was never seen.
   Now `MotionTrack.collision_now` follows the monitor's current state
   (including "nothing" once it clears), each backend keeps the latest state
   and seeds a new track with it, and the order is: collision error code;
   the monitor's state at the stop; range ahead under the floor; TF and
   no-path codes; earlier monitor events; timeout; no-progress; aborted.
   `evidence.motion_outcome.collision_monitor_at_stop` reports the state.
2. **A failed motion waited for quiet sensors before its result.** The
   rosbridge backend now builds evidence for failed results (new in #45) via
   `observation()`, which blocks up to `FLYTO_ROS2_OBSERVATION_WAIT_SECONDS`
   (3 s) when odometry or LiDAR is older than the max age. The host's safe
   stop follows the result, so a dead LiDAR delayed it by 3 s. Failed and
   timed-out results now read the held snapshot (`_evidence(wait=False)`);
   completed results still wait, as before.

Files: `flyto_robotics/motion_outcome.py`,
`flyto_robotics/generic_ros2_adapter.py`, `tests/test_motion_outcome.py`.

## Verified

- New tests fail on merged `main` and pass here.
- `make verify`: ruff clean, 1089 passed.
- `flyto-index verify --strict`: no FAIL or WARN.
- Reviewed with no defect found: twin params guard (timing keys only, capped,
  twin image only), halt and cancel paths (only a short lock to drop the
  track), Nav2 Jazzy error-code blocks, rosbridge feedback dispatch.

## Not verified

- No live twin run for these two fixes (unit tests only). The twin was not
  moved.
- rclpy backend wiring (rclpy not installed here).
- The physical robot: not touched.

## Still open

- The owner decision recorded in `2026-10-04-motion-abort-reason.md`:
  preflight checks clearance now, not that the requested distance keeps the
  0.35 m floor. Not changed here.
