# Nav2 pairing approval and the adapter drive from one table

Owner: claude
Branch: lane/nav2-pairing (from feat/unified-platform-integration-20261018 at 79a1920)
Date: 2026-10-18

## What changed

- New `flyto_robotics/ros2_interfaces.py`: `INTERFACES`, one declarative row
  per standard ROS 2 interface (type, kind, use = declared / own / reserved,
  pairing stack or none, adapter capability id, endpoint env var and default).
  `declared_interfaces()`, `own_interfaces()`, `stack_interfaces()` and
  `interface_class()` are the only views of it.
- `generic_ros2_adapter.DEFAULT_INTERFACES` and `ESCAPE_INTERFACES` are now
  derived from the table (same shape, same env vars and defaults).
  `RclpyROS2Backend._action_type` no longer keeps its own capability -> class
  map; it imports the class named by the row's type.
- `ros2_pairing._STACK_INTERFACES` is derived from the table. Nav2 now
  approves `NavigateToPose`, `DriveOnHeading`, `BackUp`, `Spin` and
  `NavigateThroughPoses`; MoveIt 2 / ros2_control rows are unchanged
  (`reserved`). Topics (camera, map, the zero-`Twist` halt) have no stack and
  can never be paired.
- New `tests/test_ros2_interface_contract.py` (10 cases). In
  `tests/test_ros2_semantic_mapping.py` the look-alike case listed
  `DriveOnHeading` and `NavigateThroughPoses` as unapproved; those encoded the
  drift and are replaced by Nav2 actions the adapter never drives
  (`FollowPath`, `ComputePathToPose`).
- `docs/CONTRACTS.md`: the pairing section names the table and says approval
  is not execution.

## Why

The adapter was right and the pairing table was behind. The adapter really
sends goals to DriveOnHeading / BackUp / Spin (ARCHITECTURE.md lists them as
the standard southbound surface; the twin and physical runs used them), and
every pairing rule they are held to holds: cancellable actions, semantic goals
(distance, speed, yaw; no actuator values), and no raw actuator surface. A
pairing table that refused them could not describe the motions the adapter
performs, so the right fix was to approve what is driven and derive both from
one row each, not to stop the adapter driving them.

Approval is not execution: the grant-bound `ros2_action_executor` still runs
only `NavigateToPose` for semantic navigation and refuses any other authorized
type (unchanged). The 0.35 m clearance floor is untouched.

## Verified

- Failing first: on 79a1920 the new contract test gave 6 failed, 4 passed
  (DriveOnHeading, BackUp, Spin, NavigateThroughPoses not approved; nav2
  approval != driven set; a `robotics.motion.move_relative@1` -> DriveOnHeading
  manifest refused). After the change: 10 passed.
- `make verify` (venv-robotics Python 3.11): ruff all checks passed;
  1881 passed, 8 skipped; assets, dry runs, lab and facility contracts,
  `ros2-pairing` and `ros2-execution-grant` replays pass.
- `flyto-index verify --strict`: 20 pass, 0 warn, 0 fail.

## Not verified

- No ROS graph, twin or robot was run; `_action_type` was exercised only with
  fake `nav2_msgs` modules.
- There is no `twin-scenarios` target in this branch's Makefile, so it could
  not be run; only `ros2-pairing` and `ros2-execution-grant` were.
- `task(action='validate')` was not run: the MCP server is pinned to another
  repository; `make verify` covers ruff + pytest.
- CI not run.

## Follow-ups

- `ros2_action_executor` keeps its own `NavigateToPose` literal for the one
  action it executes; if it ever executes more, derive that from the table too.
