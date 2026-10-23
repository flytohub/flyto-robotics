# ROS 2 semantic mapping tests (J06)

Owner: claude
Branch: claude/j06-interop (from claude/robotics-p0-20261018)
Date: 2026-10-18

## What changed

`tests/test_ros2_semantic_mapping.py` (new, 20 cases, test-only): every row
of `ros2_pairing._STACK_INTERFACES` is held to "an interface the table does
not approve for this stack is refused" (each foreign stack's types, look-alike
nav2 types, another package's `NavigateToPose`), plus forward admission of the
standard nav2 manifest, unknown stack, kind/type mismatch and topic message
types. No source change.

## Why

J06 asks for semantic mapping tests only where a real adapter exists. ROS 2
has one (the pairing manifest plus the generic adapter); OPC UA has none in
any Flyto2 repository, so no OPC UA tests were written. A ROS graph
description is never an authorization; the table is.

## Verified

- New tests: 20 passed. They pass on the base too: they are coverage of
  existing behavior, not a fix.
- `make verify`: ruff pass, 1861 passed / 8 skipped, assets, dry runs,
  contracts, ros2-pairing and ros2-execution-grant replay pass.
- `flyto-index verify --strict`: 20 pass, 0 warn, 0 fail.

## Not verified

- No ROS graph, simulator or robot was run. CI not run (billing-locked).

## Follow-ups

- Finding, not changed: `generic_ros2_adapter.DEFAULT_INTERFACES` drives
  `motion.advance` / `motion.retreat` / `motion.rotate` through
  `nav2_msgs/action/DriveOnHeading`, `BackUp` and `Spin`, while the pairing
  table approves only `NavigateToPose` for nav2. The two tables describe the
  same mapping and disagree; and `_action_type` keeps a third copy of the
  capability -> action class map. The robotics owner should decide which is
  the authority and derive the others from it.
