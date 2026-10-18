# The arm speaks the shared capability vocabulary

Owner: claude
Branch: lane/arm-vocabulary
Date: 2026-10-18

## What changed

- `flyto_robotics/arm_adapter.py`: the capability ids are now
  - `CAPABILITY_STATE = "manipulation.state"` (was `arm.state`);
  - `CAPABILITY_MOVE = "manipulation.move_joints"` (was `arm.move_joints`);
  - `CAPABILITY_HALT = "motion.halt"` (was `arm.halt`).
  Behaviour is unchanged.
- `flyto_robotics/adapter_contract.py`: `_CAPABILITY_METADATA` (and so
  `KNOWN_CAPABILITY_IDS`) declares four rows, each with the safety class,
  `robot.motion` permission, stop requirement and cancellability that Cloud's
  vocabulary gives them:
  - `manipulation.state`: `read_only`, no permission, not cancellable;
  - `manipulation.move_joints`, `manipulation.grasp`, `manipulation.release`:
    `movement`, require a safe stop.
- `flyto_robotics/arm_contracts.py`, `docs/ARM_M0.md` and `CHANGELOG.md`: the
  ids updated to match.
- `tests/test_arm_contracts.py`: a new test checks that the arm serves exactly
  the vocabulary ids, and that each row's contract agrees with the shared
  metadata.

## Why

Cloud approves only vocabulary capabilities. Before this change it refused
`arm.*` at discovery (`capability_not_in_vocabulary`), so nothing the arm
served could be approved. Cloud now ships the `manipulation` family
(flyto-cloud `lane/arm-vocabulary`), and the stop is the one shipped safe stop
for any machine.

## Verified

- Failing first, the new test reported: "Extra items in the left set:
  'arm.state', 'arm.move_joints', 'arm.halt'". After the change it passes.
- `make verify` with the 3.11 venv: ruff clean; 1,865 passed, 8 skipped.
- `tests/test_arm_contracts.py` against flyto-core's `validate_contract` and
  `judge`: 16 passed. This used
  `PYTHONPATH=flyto-core/src:. flyto-core/.venv/bin/python`.
- `flyto-index verify --strict`: 20 pass, 0 warn, 0 fail.

## Not verified

- No real arm. Grasp and release are vocabulary only; the adapter does not
  serve them yet (M1).
- The arm adapter is not wired into `adapter_provider`'s manifest, so
  discovery of a live arm end to end has not been exercised.
- CI was not run.

## Follow-ups

M1 is still open: a gripper capability in `SerialArmAdapter` serving
`manipulation.grasp` and `manipulation.release`, with measured evidence, and
the manifest wiring.
