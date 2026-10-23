# Arm M1 (simulated): grasp and release confirmed by read-back; the executor reads the one table

Owner: claude
Branch: lane/arm-m1-sim
Date: 2026-10-18

## What changed

- `flyto_robotics/arm_adapter.py`:
  - An optional `gripper` section in `flyto.arm-profile.v1`, all keys required:
    `joint`, `closes_toward`, `release_rad`, `max_speed_rad_s`,
    `position_tolerance_rad`, `object_min_gap_rad` and `hold`.
  - The gripper's travel is its joint's declared limits.
  - `manipulation.grasp` takes no arguments and closes toward the closed
    limit. It is `completed` (`object_held: true`) only when the jaws stopped
    at least `object_min_gap_rad` short of closed and held within
    `hold.max_drift_rad`.
    - Closing fully, or stopping closer than the gap, is `failed` /
      `no_object` / `object_held: false`.
    - Movement during the hold is `slipping`, followed by a stop.
    - Never moving from open is `stalled`, followed by a stop.
    - A grasp is refused unless the jaws are open.
  - `manipulation.release` takes `open_to_rad`, which must lie between
    `release_rad` and the open limit. Out of range is refused before any write
    and never clamped. It is `completed` only when read-back shows the jaws
    there and staying there.
  - The M0 preconditions and bus-fault handling are now shared by move, grasp
    and release (`_motion_refusal`, `_run_guarded`, `_start_pose`,
    `_enable_torque`). A bus fault is `failed`, `state_known: false`,
    `object_held: null`, and is never re-sent. `motion.halt` cancels a grip.
- `flyto_robotics/arm_contracts.py`: grasp and release rows, emitted only when
  a gripper is declared. Both are `movement`, cancellable, not idempotent.
  - Grasp has two `delta` specs on `object_held`: `held` 0 to 1 exactly, and
    jaw drift within the hold tolerance.
  - Release has one `distance_to` spec on `gripper_position` against
    `open_to_rad`.
- `flyto_robotics/arm_bus.py`: `SimulatedArmBus.place_object` adds simulated
  objects that block the jaws, slip after N reads and are let go of when the
  jaws open away from them.
- `flyto_robotics/lerobot_arm_bus.py`: `gripper=` is mapped only when the
  motors bus lists that motor in `motors`. Otherwise `ValueError`.
  Unverified.
- `examples/arm-profiles/am-arm200-follower.json`: a placeholder gripper
  section. `limits_accepted` is still false.
- `flyto_robotics/ros2_interfaces.py`: `Ros2Interface.executed` and
  `executed_interface()` (exactly one row, NavigateToPose).
  - `ros2_action_executor` accepts only that row's kind and type, and loads
    its class through `interface_class`.
  - `standard_ros2_adapter_manifest` takes stack, kind, name and type from the
    same row.
  - The executor still executes only NavigateToPose.
- Docs: `docs/ARM_M0.md` has a "Milestone M1" section and arrival-day steps
  6 to 8. `CHANGELOG.md` is updated.
- Rebased onto b1ac6d5 (state read cancellable). Added
  `test_every_served_manipulation_row_is_cancellable_and_the_stop_is_not`,
  which covers the grip rows.

## Why

M1 is tabletop pick and place. A grasp that reports success on empty jaws is
the failure that matters, so completion is decided by read-back (stopped
short and holding), never by the command. The executor's NavigateToPose
literal was the last copy of the interface table outside it.

Rejected:

- A grasp target argument. The closed limit comes from the profile, and an
  object is detected by where the jaws stop.
- A release with no argument. `distance_to` needs an argument to judge
  against, so release takes `open_to_rad`, bounded to the release range.

## Verified

- Failing first, on the base source (52b558e):
  - `tests/test_arm_gripper.py` and `tests/test_arm_contracts.py` failed to
    collect (`ImportError: cannot import name 'CAPABILITY_GRASP'`).
  - The base adapter answered grasp and release with `refused` ("unsupported
    capability").
  - The base profile parser refused the `gripper` key.
  - The base `LeRobotArmBus` had no `gripper` parameter.
  - All 4 new `tests/test_ros2_interface_contract.py` tests failed:
    `ImportError: executed_interface` (2),
    `TypeError ... unexpected keyword 'executed'` (1),
    `AttributeError ... no attribute 'executed'` (1).
  - All of these pass after the change.
- `make verify` (3.11 venv): ruff clean; 1939 passed, 14 skipped. The
  pre-change baseline was 1883 passed, 8 skipped. The 6 extra skips are
  core-only tests.
- Core-backed checks, with flyto-core at 795d07e (pyproject 2.40.0):
  - `tests/test_arm_contracts.py`: 33 passed.
  - With `test_arm_gripper.py` and `test_arm_adapter.py` added: 117 passed.
- `flyto-index verify --strict`: 20 pass, 0 warn, 0 fail.

## Not verified

- No physical arm or gripper. Evidence level D2: simulation only.
- The gripper profile values (direction, release position, object gap, speed,
  hold) are guesses.
- The independent camera confirmation of M1 is not built.
- The pick-and-place workflow and the pack rows do not exist.
- Everything in `LeRobotArmBus`, including whether `motors` exists and the
  gripper's calibration range.
- `task(action='validate')` was not run. The MCP server is not pinned to this
  worktree; `make verify` ran ruff and pytest instead.
- The executor change was not run against a ROS graph.
- CI was not run.

## Follow-ups

- Run arrival-day steps 6 to 8 in `docs/ARM_M0.md` with the hardware present.
- Build the camera check that confirms the object was placed.
- Add the pack rows once the steps pass.
