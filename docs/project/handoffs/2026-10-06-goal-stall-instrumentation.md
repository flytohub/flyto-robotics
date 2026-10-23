# A goal that stands still says what was commanded meanwhile

Owner: claude
Branch: claude/stall-after-abort (off origin/main f5091b4, 0.6.2)
Date: 2026-10-06

## What changed (0.6.3, instrumentation only)

- `flyto_robotics/motion_outcome.py`: `MotionTrack.saw_base()` (odometry) and
  `saw_command(topic, ...)` record `stalls`: spells >= `STALL_MIN_S` (2 s) with
  odometry below 0.01 m/s and 0.05 rad/s while a goal runs, each with
  `at_s`, `duration_s`, pose, and per watched topic `samples`,
  `max_linear_mps`, `max_angular_radps`. `recoveries_at_s` records when
  `number_of_recoveries` rose. Both are additive keys on `motion_outcome`.
- `flyto_robotics/generic_ros2_adapter.py`: the rosbridge transport subscribes
  (read only, 100 ms throttle, type from the graph) to
  `FLYTO_ROS2_COMMAND_WATCH_TOPICS` (default `/cmd_vel_nav,/cmd_vel`) and
  feeds each running track. Odometry feeds `saw_base`. The escape's
  `legs[]` add `started_at_s` and `elapsed_s`.
- `tests/test_goal_stalls.py` (new). `tests/test_presence_on_state.py`
  tolerates a subscription without `type`.

## Why: what was measured, and what was not

The stalled run was t-f044dad913d21612 (Desktop execution 15ef40eb,
20:38:00-20:40:13 local). Sources: Desktop log, `flyto-cloud/evidence`, and the
twin's `~/.ros/log` (read-only).

- **No adapter gap.** The waypoint leg succeeded at +14.30 s and the goal leg
  began at +14.31 s (Nav2 bt_navigator). There was no second plan_escape and
  no braking-guard, lease or timeout wait. The waypoint's "goal reached"
  was reported.
- **Nav2 ran the whole time.** "Passing new path" came every ~1 s, and
  "Failed to make progress" fired at +11.6, 23.3, 35.3, 47.1, 60.6, 72.2,
  89.7, 101.6 s of the goal leg (`movement_time_allowance` 10 s). Recoveries
  were clear local, clear both, spin 1.57, wait 5 s, and backup. The leg
  succeeded 15 s after the backup.
- **No collision-monitor action.** The collision-monitor log has no event
  after 18:56, and the adapter's `/collision_monitor_state` subscription
  recorded none for this goal. **No DWB failure** either: no "no valid
  trajectories" or "patience exceeded" error in controller_server, so DWB
  returned legal commands.
- **Same start, different outcome.** The waypoint heading sent was +0.25
  (`clear_heading_rad` 0, `heading_tolerance_rad` 0.25). The goal leg started
  at map (-0.143, 0.384) yaw +0.484. The fast run 2619db56 started at
  (-0.150, 0.377) yaw +0.492 and took 20.5 s. The obstacle geometry and
  clearances of every escape that day overlap between slow and fast runs.
- **A correlation, not a mechanism.** Of the advances before an escape since
  0.6.1, 20 ended by the adapter's braking-guard cancel and 3 by Nav2
  DriveOnHeading "Collision Ahead" (error 723). The two stalls (20:04,
  20:38) followed two of those three aborts. The third (20:49, recorded)
  was followed by a 14.7 s goal leg. Nothing found in the adapter or in Nav2
  carries state from one to the other: the guard is dropped at the terminal
  status, both paths publish the final zero, and Nav2's later log sequence is
  identical. The abort probably reflects the same fine geometry (box cells vs
  DriveOnHeading's 2 s look-ahead), not a cause.
- **Passive recording of five later goal legs (all fast).** A
  subscriber-only rclpy script in the twin captured `/evaluation`,
  `/cmd_vel_nav`, `/cmd_vel`, `/plan`, `/odom` and the local costmap. Every
  goal leg's first DWB cycles still carried the waypoint leg's RotateToGoal
  state (779 of 819 trajectories illegal, rotation only) until the new plan
  arrived. The base then moved within 0.18-0.37 s. No stall occurred while
  recording, so DWB's choice during a stall was never seen.

**Missing to name the cause:** what Nav2's controller commanded
(`/cmd_vel_nav`) versus what reached the base (`/cmd_vel`) during a stall,
and DWB's evaluation (`/evaluation`: legal count, which critic rejected
translation, best twist) plus `/plan` at that moment. A stationary base with
near-zero controller commands would mean DWB chose to stand (for example a
latched RotateToGoal, or critics preferring zero). Real commands that never
reach the base would point at the smoother, the collision monitor or the
driver. 0.6.3 records the first half on every motion. `/evaluation` is too
heavy to carry in a result. For that, record it on the twin with a
subscriber-only script (see Follow-ups).

## Verified

- `make PYTHON=<repo .venv>/bin/python verify`: ruff clean, 1574 passed,
  1 skipped, remaining verify targets passed.
- `flyto-index scan . && flyto-index verify --strict`: all checks PASS, exit 0.
- New tests: a stall with near-zero commands is recorded with both topics'
  maxima; motion and sub-2 s pauses are not stalls; a stall still open at
  the end is closed by the summary; recovery times only when the count rises;
  Twist and TwistStamped parse; env controls the topics; the rosbridge
  backend subscribes untyped and throttled and feeds a running track; escape
  legs carry timing.

## Not verified

- Not run against the twin (stopped by the coordinator) or the robot.
- rclpy transport: `saw_base` works (shared `_store_odometry`), but no
  command-topic subscription is wired, so stalls there have empty
  `commanded`.
- The recorder script was left at `/tmp/flyto-rec/` inside the stopped twin
  container (it disappears when the container is recreated). It only ever
  subscribed.

## Follow-ups

- Next time the twin runs the box loop with 0.6.3, read `motion_outcome.stalls`
  of any slow goal leg first.
- If `/cmd_vel_nav` is near zero during the stall, rerun with a subscriber-only
  `/evaluation` recorder to see which critic made translation illegal or
  costlier. Candidates seen in the recordings: RotateToGoal carried over
  from the waypoint leg, and GoalAlign/GoalDist. Only then change goal
  construction or propose a Nav2 parameter change that applies to the robot
  too.
- Side finding: Jazzy's DriveOnHeading rejects preemption ("Received a
  preemption request for drive_on_heading, however feature is currently not
  implemented. Aborting and stopping", 4 times at 19:32, 20:13, 20:20, 20:26),
  so the braking guard's slower re-send aborts the drive instead of slowing
  it. It stopped safely each time, but the slow-down path does not do what it
  says on this Nav2.
