# A safe stop for one job stranded another job's goal

Owner: claude
Branch: claude/rosbridge-cancel-scope
Date: 2026-10-04

## What changed

`flyto_robotics/generic_ros2_adapter.py`, `RosbridgeROS2Backend`:

- `cancel()` now reads the goal's `action_result` without taking it
  (`_wait_for(..., consume=False)`). It removes the message only when the
  cancel was confirmed and no `invoke()` for that call id is waiting; the kept
  `CANCELLED` result answers any later `invoke()`.
- `invoke()` registers itself in `_result_waiters` while it waits for its
  goal's result.

Tests: `tests/test_motion_outcome.py`
`test_a_safe_stop_for_one_job_does_not_strand_another_jobs_goal` (fails on
the previous code with `timeout`, the incident's "ROS 2 action still running")
and `test_cancel_with_nobody_waiting_keeps_no_result_behind`. One existing
stub of `_wait_for` in `tests/test_observation_on_state.py` takes the new
keyword.

## Why

Twin rehearsal, 2026-10-04 09:33-09:38 (+08:00), navigate to (1.5, 1.2):

1. The local device job poller received the same Cloud job
   `7b4a5346-...` four times in ten seconds, and each claim returned 200, so
   four executions of one navigate ran concurrently on one shared rosbridge
   connection (`supports_shared_connection = True`). Each new
   `NavigateToPose` goal preempted the previous one in bt_navigator
   ("Received goal preemption request").
2. A preempted execution got status 6 (aborted), its host cancel was refused
   (its own goal had already ended) and its host safe stop cancelled every
   active goal on the connection, which includes the next execution's goal.
3. That cancel *popped* the other goal's `action_result` while confirming
   it. The goal's own `invoke()` never saw its goal end, waited its whole
   300 s deadline, and reported "ROS 2 action still running; cancel=refused"
   (refused because the safe stop had already forgotten the goal). The robot
   had in fact stopped at 09:33:40.

The safe stop still cancels every active goal on the robot; narrowing it to
one call id was rejected because a safe stop is a robot-wide stop.

The duplicate execution itself (point 1) is not in this repository: Cloud
offered and re-granted a claim for a job already claimed, and the local job
executor in flyto-cloud only de-duplicates claims that are still in flight,
not jobs it is already running.

## Verified

- `make verify`: ruff clean, 1146 passed, 1 skipped.
- `flyto-index verify . --full-scan --strict`: no FAIL or WARN.
- The new stranding test fails on `origin/main` code and passes here.

## Not verified

- No live reproduction of four concurrent executions against the twin; the
  sequence above is reconstructed from bt_navigator, controller_server and
  backend logs.
- `RclpyROS2Backend` was not changed; it waits on per-call result futures,
  which a cancel does not consume.

## Follow-ups

- flyto-cloud `local/job_executor.py`: refuse to start a job id that this
  process is already executing; and the Cloud claim endpoint should not
  return 200 for a job that is already claimed.
