# Planner wire contract: flyto.capability-plan.v1 and resource_id

Owner: claude
Branch: claude/capability-plan-contract
Date: 2026-10-07

## What changed

- flyto-robotics 0.7.0 speaks the planner contract flyto-ai emits since its PR
  #64: plans are `flyto.capability-plan.v1` with `resource_id` (was
  `flyto.robotics.plan.v1` / `robot_id`), requests are
  `flyto.robotics.planner-request.v2` with `resource_id`.
- `flyto_robotics/ai_planner.py` owns the three wire constants
  (`PLAN_CONTRACT_VERSION`, `PLANNER_REQUEST_CONTRACT`,
  `PLANNER_RESPONSE_CONTRACT`); `goal_planner`, `showcase_planning` and
  `planning_session` read them instead of repeating the strings.
- `contracts/plan-v1.schema.json` renamed to
  `contracts/capability-plan-v1.schema.json`; every `examples/plans/*.json`,
  the MCP benchmark and the tests use the new shape.
- `tests/fixtures/capability-plan-exchange.v1.json` +
  `tests/test_capability_plan_exchange.py`: the exact request robotics builds
  and a plan it accepts unchanged. flyto-ai holds a byte-identical copy; both
  repos pin sha256 `a891da42…e877`.

## Why

flyto-ai PR #64 renamed the contract on its side only; a robotics client sent
`robot_id` and rejected any plan not named `flyto.robotics.plan.v1`, so the lab
planner loop (`scripts/run-ai4all-showcase.sh` -> `flyto_ai/robotics_planner_server.py`
on :8787) would have broken in both directions.

No compatibility window was kept. flyto-cloud pins this package (0.6.6,
ff17794) only for the adapter provider and ROS 2 adapter; neither imports the
planner client, and flyto-cloud never imports `flyto_ai.robotics_planning`. So
no released Desktop calls the planner. The only old callers are lab tools
<= 0.6.6, which flyto-ai now refuses by name from its declarative
`RETIRED_PLANNER_REQUEST_CONTRACTS` table with an upgrade instruction.

Job, result, planning-session and MCP tool inputs keep `robot_id`: they are the
executor's contracts, not the planner's.

## Verified

- `make verify` (ruff, 1629 pytest passed / 1 skipped, asset, dry-run, lab,
  facility, ROS 2 pairing and grant checks): exit 0.
- New exchange tests fail on the previous code (the contract constants do not
  exist there) and pass now.
- `flyto-index verify --strict` (see PR).

## Not verified

- No Gazebo or hardware run; the showcase script was not executed end to end
  against a live planner.

## Follow-ups

- flyto-cloud pin bump (owned by another agent) should take 0.7.0 at the merge
  commit of this branch. It changes nothing Cloud imports today.
