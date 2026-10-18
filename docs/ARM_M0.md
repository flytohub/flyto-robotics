# Serial-bus arm, milestone M0

The second P0 device is a tabletop follower arm driven over USB serial (the
AM-ARM200; the leader arm is out of scope). M0 is: read its state, enforce
its joint limits, and stop it with a stop that is **measured**, all behind
`flyto.capability-contract.v1` contracts. The hardware has not arrived, so
everything below runs against a simulated bus. Nothing here has touched a
real arm.

## What is implemented

| Piece | File | What it does |
| --- | --- | --- |
| Bus seam | `flyto_robotics/arm_bus.py` | `ArmBus`: read joint positions, read torque state, write goal positions, enable or disable torque. Radians by joint name. Any failure is `BusError`. |
| Adapter | `flyto_robotics/arm_adapter.py` | `SerialArmAdapter`: `manipulation.state`, `manipulation.move_joints`, `motion.halt` over any `ArmBus`, driven only by an `ArmProfile` (`flyto.arm-profile.v1`). Uses `adapter_contract.CallRequest` / `CallResult`. |
| Contracts | `flyto_robotics/arm_contracts.py` | `capability_rows(profile)`: the three contract rows, built from the same profile the adapter enforces. |
| Profile | `examples/arm-profiles/am-arm200-follower.json` | Placeholder joint names, limits and tolerances. `limits_accepted: false`. |
| Bus over LeRobot | `flyto_robotics/lerobot_arm_bus.py` | Wraps an already-connected LeRobot-style motors bus. **Unverified until hardware acceptance.** Never imports LeRobot, never opens a port. |

Behaviour, all from the profile:

- **State** reads every declared joint's position and torque state and writes
  nothing. A missing or non-finite joint is a failed read, not a partial one.
- **Limits.** `manipulation.move_joints` takes one absolute target per joint
  (`<joint>_rad`), all required. A target outside `[min_rad, max_rad]`, not a
  number, or not finite is refused before the bus is touched. It is never
  clamped. A move is also refused when the arm already stands outside its
  limits, and whenever `limits_accepted` is `false`.
- **Bounded speed.** The adapter steps the goal toward the target at
  `max_joint_speed_rad_s`, so speed does not depend on servo firmware settings.
  Before enabling torque it writes the measured pose as the goal.
- **Completion** needs every joint within `position_tolerance_rad` of its
  target and a settle window whose drift stays within `settle.max_drift_rad`.
  The result carries `observations.joint_positions` for `before`, `after` and
  `settled` in frame `joints`, which the contract's `distance_to` evidence
  judges.
- **Stall.** A joint lagging its goal that moves less than
  `stall.min_progress_rad` for `stall.window_s` ends the move as `failed`
  (`stalled`), followed by a stop.
- **Stop.** `hold` mode writes the measured pose as the goal, reads back that
  torque is on, and samples positions `stop.confirm_samples` times. It is
  `completed` only if no joint drifts more than `stop.max_drift_rad`.
  `torque_off` mode disables torque and is `completed` only if every joint
  reads torque off. Anything else, including a bus error, is a `failed` stop.
  A stop interrupts a running move (the move ends `cancelled`) and is never
  answered from a cache.
- **Fail closed.** A bus error during a move ends it `failed` with
  `state_known: false` and `retry: never`. The same call id returns that
  result again without touching the bus, and every new move is refused until
  a state read succeeds. An unconfirmed stop latches: moves stay refused
  until a later stop is confirmed.

## Contracts

| Capability | Safety class | Contract points |
| --- | --- | --- |
| `manipulation.state` | `read_only` | `actuates: false`, no evidence |
| `manipulation.move_joints` | `movement` | every parameter `number` with finite `min`/`max` equal to the joint limits; one `distance_to` spec per joint on `joint_positions`, frame `joints`, phases `after` + `settled`, tolerance = arrival tolerance + settle drift; `idempotent: false` |
| `motion.halt` | `controlled` | `role: safe_stop`, not cancellable, idempotent; in `hold` mode one `delta` spec per joint (`expect.value` 0, tolerance = stop drift) over the hold's `before`/`after` |

Evidence specs carry the display key `unit` (`rad`), so the rows need
flyto-core 2.40.0 or newer; an older core rejects the key.

A `torque_off` stop declares no evidence: torque state is not something a
measure op expresses. Its proof is the read-back in `stop_confirmation`.

The rows are validated by flyto-core's `validate_contract`, and the adapter's
own results are judged with flyto-core's `judge`, in
`tests/test_arm_contracts.py`. This repository does not depend on flyto-core,
so those tests skip under `make verify` and run with:

```bash
PYTHONPATH=<flyto-core>/src:. <flyto-core venv>/bin/python -m pytest tests/test_arm_contracts.py
```

## What is simulated

`SimulatedArmBus` moves each driven joint toward its goal by a bounded step
per read. It can simulate a stalled joint, a connection that drops after a
number of writes, a torque-off that does not take effect, and a hold that
sags. The tests in `tests/test_arm_adapter.py` cover each of these.

## Explicitly unverified

- The arm's joint count, joint names, travel limits, safe speed and the
  tolerances in the example profile. They are placeholders.
- Whether a hold or torque-off is the right stop for this arm. Torque-off can
  let a loaded arm fall; `hold` is the example's default for that reason.
- Everything in `lerobot_arm_bus.py`: the register names, the method names, the
  degree scaling, how the gripper is calibrated, how fast the bus answers, and
  what happens on unplug.
- Timing: a serial round trip at `control_period_s` 0.05 s.
- Wiring into a host: no entry point, no discovery and no flyto-modules-robotics
  pack row exists for the arm yet.

## Arrival-day checklist

Run each step with the arm clamped to the table, the workspace clear, and a
person at the power switch. Stop at the first step that fails. Record each
result (pass or fail, with the read-back) in a handoff.

1. **Connect.** Power the follower arm, plug in USB, confirm the serial
   device appears. Build the bus (`LeRobotArmBus` around the vendor's
   connected motors bus, or a new `ArmBus`) and confirm each assumption at
   the top of `lerobot_arm_bus.py`. Pull the cable once while reading and
   confirm the read raises `BusError`.
2. **Read state.** Torque off. `manipulation.state` returns every joint with finite
   positions. Move each joint by hand and confirm its sign, unit and that the
   reading follows (compare with a protractor or a known pose).
3. **Limits.** Hand-move each joint to a safe end of travel and record the
   reading. Write those (with a margin) as `min_rad`/`max_rad` in a site copy
   of the profile. Keep `limits_accepted: false` and confirm that a move is
   refused, then confirm out-of-range targets are refused with nothing
   written.
4. **Measured stop.** With torque on and holding, call `motion.halt` in the
   chosen mode. `hold`: confirm `confirmed: true`, then hang a light load and
   confirm a sag is reported as a failed stop. `torque_off`: support the arm
   by hand first, confirm every joint reads torque off. Pull the cable and
   confirm the stop reports `failed`.
5. **Low-speed move.** Only now set `limits_accepted: true` with
   `max_joint_speed_rad_s` at 0.1 or below. Move one joint 0.1 rad and back;
   confirm `completed`, the read-back phases and a `judge` verdict of usable.
   Block a joint by hand during a move and confirm `stalled` plus a confirmed
   stop. Call `motion.halt` mid-move and confirm the move ends `cancelled`.

After all five pass, replace the placeholders in the example profile with the
measured values, and only then add the pack rows to flyto-modules-robotics.
