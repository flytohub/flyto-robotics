# Fetch-live lane: cargo sensor review fixes, confirmed halt, contract speed

Owner: claude
Branch: claude/fetch-live-sim
Date: 2026-11-02

## What changed

Base: `main` 0f283a0 plus bd26172 (simulated tray bridge, already on this
branch). Commits, oldest first:

- c5efcf3, 0de1963: cherry-picks of 8d303de and 64fd05f from
  `claude/fetch-live-cargo-sensor` (registry conflict resolved by keeping both
  rows). That branch and its worktree `.claude/worktrees/flyto-robotics-cargo-sensor`
  were deleted after `git cherry` and a content diff showed both commits here.
- 7bdf9df, cargo sensor review fixes:
  - `cargo_sensor.configured_topic()` defaults to `/flyto/cargo_present`
    (unset or blank env). The capability is still declared only when rosapi
    lists that topic as `std_msgs/msg/Bool`, so a robot without it declares
    nothing. `sim/fetch_return/cargo_bridge.py` now imports this function
    instead of its own copy.
  - `CargoSensor.record()` clears the last reading on a message without a
    literal boolean.
  - `GenericROS2Adapter.describe()` raises `RuntimeError` instead of declaring
    a capability with an empty interface name (the old `("topic", "", "")`
    fallback is gone).
  - Test that a reconnect clears the reading. Mutation check run: replacing
    `self._cargo.clear()` in `RosbridgeROS2Backend.reconnect` with `pass` makes
    `test_a_reconnect_clears_the_last_reading` fail; restored.
- 95d9ff2, audit P0 `ros2-safe-stop-completed-without-cancel-confirmation`:
  - One implementation, `_ObservationState._halt`, used by both backends'
    `safe_stop`. `completed` only when no goal was running, or every running
    goal's cancel was confirmed (or the goal ended), zero velocity was
    published, and `wait_until_stationary(HALT_STILL_WAIT_S=2.0)` saw the robot
    still. Otherwise `failed`, `evidence.reason_code = "effect_unknown"` (Cloud's
    own code in `services/space_tasks/uncertain_task_effect.py`, read through
    `GENERIC_PATHS` `reason_code`), `effect_unknown: true`, and
    `halt_confirmation` {active_goals, cancelled, unconfirmed, stillness,
    zero_velocity_held}.
  - A refused or unanswered cancel keeps the goal (rclpy handle / rosbridge
    `_active_actions`), so the next halt cancels it again; a daemon thread
    re-sends zero every `ZERO_HOLD_PERIOD_S=0.1` until the goal ends, the
    link drops, or a halt confirms it; `_motion_preflight` refuses motion
    while `held_goals()` is non-empty. Reconnect clears goals (existing
    behaviour), which releases the hold.
  - Root cause found on the way: rosbridge `cancel()` waited on
    `_action_results`, which the goal's own `invoke()` pops, so a confirmed
    cancel could read as unconfirmed. Terminal status is now kept per call in
    `_goal_ended` (never consumed; cleared on a new goal and on reconnect).
  - Tests changed because they locked in the false success:
    `test_a_stop_publishes_first_and_keeps_a_goal_whose_cancel_is_refused`
    (rclpy refused cancel: was completed + goal forgotten),
    `test_a_rosbridge_stop_publishes_first_and_keeps_an_unanswered_goal`
    (was completed + goal forgotten),
    `test_a_stop_after_a_cancel_that_never_reaches_zero_still_ends_on_zero`
    (timed-out cancel: was completed). Two halts with a confirmed cancel
    (`test_rosbridge_deadline_reports_timeout_and_keeps_tracking`,
    `test_a_safe_stop_for_one_job_does_not_strand_another_jobs_goal`) now
    stream odometry, as a robot does, so stillness can be observed.
- 982cc3f, contract speed:
  - `DeclaredArgument.default` (in `to_dict` only when set, and as JSON
    schema `default`). `motion.advance` `speed_mps` 0.02..0.25 default 0.12;
    `motion.retreat` 0.02..0.20 default 0.10 (values unchanged, now declared).
  - `declared_arguments()` / `declared_default()` in
    `generic_ros2_adapter.py`: `FLYTO_ROS2_MAX_LINEAR_SPEED_MPS` (resource top
    speed; `.env.example` 0.22 for a Burger) lowers the declared maximum and
    default, never raises them; unreadable or < 0.02 refuses motion in the
    preflight. `describe()`, `_numeric_arguments`, the braking plan and both
    backends' goal builders read them. `DEFAULT_STRAIGHT_SPEED_MPS` and the
    0.10/0.12 literals are removed. Floor, braking plan, supervised caps
    untouched.

## Why

- Cargo: reviewer findings on the earlier lane.
- Halt: Cloud took a robot still driven by an un-cancelled Nav2 goal as
  stopped.
- Speed: a live 2.0 m advance on the twin took 52 s (~0.04 m/s). The default
  speed was a literal in three places outside the published contract.

## Verified

- Failing tests first: the new halt tests and the three changed ones fail
  against the pre-change adapter (8 FAILED, checked by swapping in the HEAD
  file); the cancel-race test failed before `_goal_ended`; the cargo default /
  malformed / describe tests failed before 7bdf9df; the speed tests failed to
  import before 982cc3f.
- `PYTHONPATH=$PWD make verify PYTHON=<repo>/.venv/bin/python` in the
  worktree: rc 0, ruff clean, 2018 passed, 19 skipped.
- `flyto-index verify --strict` in the worktree: Status PASS, 20 PASS, 0 WARN,
  0 FAIL.
- All with fakes (fake rosbridge sockets, fake rclpy). Nothing pushed.

## Not verified

- Nothing was run against the live twin, Gazebo or the robot (services and
  robot were off-limits). The halt's stillness wait and zero hold are untested
  against Nav2.
- The 52 s advance is not explained by this change: the default was already
  0.12 m/s. Unverified candidates: Gazebo real-time factor (0.12 m/s at RTF
  ~0.33 is 0.04 m/s wall; `sim_time` evidence on that call would show it) and
  braking slowdowns near a wall (`braking.speed_changes` in its evidence).
  The run's evidence was not available here.
- `motion.rotate` has no speed argument; Nav2 Spin turns at its server's
  rate. Not changed.
- Cloud: whether the new `motion.advance` / `motion.retreat` schema hash
  needs a renewed approval; whether a failed `motion.halt` with
  `reason_code: effect_unknown` holds the task end to end (only the Cloud
  reader code was read).
- `task(action='validate')` via MCP was not run (MCP pinned to another
  root); `make verify` ran ruff and pytest.

## Follow-ups

- `sim/fetch_return/rosbridge_link.py` (from bd26172) is a second rosbridge
  client beside `RosbridgeROS2Backend`; fold it in or justify it.
- Re-run the 2.0 m advance on the twin and read `sim_time.rtf_*` and
  `braking.speed_changes` before raising any default.
