"""The arm adapter's capabilities as ``flyto.capability-contract.v1`` rows.

Built from an :class:`~flyto_robotics.arm_adapter.ArmProfile`, so the bounds a
host enforces and the tolerances it judges with are exactly the ones the
adapter enforces itself:

* ``arm.state``: read only, no evidence.
* ``arm.move_joints``: one ``number`` parameter per joint (``<joint>_rad``)
  whose ``min``/``max`` are the profile's joint limits, and one ``distance_to``
  evidence spec per joint on ``joint_positions`` in frame ``joints``. The
  tolerance is the adapter's arrival tolerance plus its settle drift, so the
  contract accepts no end pose the adapter would not have reported as arrived.
* ``arm.halt``: ``role: safe_stop``. In ``hold`` mode one ``delta`` spec per
  joint says the joint did not move more than the stop's drift tolerance
  between the hold being written and the end of the confirmation window. A
  ``torque_off`` stop is confirmed by the torque read-back the adapter reports
  in ``stop_confirmation``, which no measure op expresses, so it declares no
  evidence.

The rows have the shape of a ``flyto.pack.v1`` module row (``module_id``,
``provides_capability``, ``params_schema``, ``timeout_ms``, ``contract``;
flyto-core ``docs/specs/PACK_MANIFEST_SPEC.md``). This repository does not
import flyto-core; ``tests/test_arm_contracts.py`` validates the rows with
flyto-core's ``validate_contract`` when it is importable.
"""

from __future__ import annotations

import math
from typing import Any

from .arm_adapter import (
    ARGUMENT_SUFFIX,
    CAPABILITY_HALT,
    CAPABILITY_MOVE,
    CAPABILITY_STATE,
    FRAME,
    OBSERVATION,
    STOP_HOLD,
    ArmProfile,
)

CONTRACT_SCHEMA = "flyto.capability-contract.v1"
# Headroom over the adapter's own budget, for the host's deadline.
DEADLINE_MARGIN_MS = 2000


def _ms(seconds: float) -> int:
    return int(math.ceil(seconds * 1000)) + DEADLINE_MARGIN_MS


def state_row(profile: ArmProfile) -> dict[str, Any]:
    return {
        "module_id": CAPABILITY_STATE,
        "provides_capability": CAPABILITY_STATE,
        "label": {"en": "Arm state"},
        "description": "Read every joint's position and torque state. Writes nothing.",
        "params_schema": {},
        "timeout_ms": DEADLINE_MARGIN_MS * 2,
        "contract": {
            "schema": CONTRACT_SCHEMA,
            "actuates": False,
            "safety_class": "read_only",
            "requires_safe_stop": False,
            "cancellable": False,
            "idempotent": True,
        },
    }


def move_row(profile: ArmProfile) -> dict[str, Any]:
    params: dict[str, Any] = {}
    evidence: list[dict[str, Any]] = []
    tolerance = profile.position_tolerance_rad + profile.settle_max_drift_rad
    for joint in profile.joints:
        argument = joint.name + ARGUMENT_SUFFIX
        params[argument] = {
            "type": "number",
            "required": True,
            "min": joint.min_rad,
            "max": joint.max_rad,
            "unit": "rad",
            "description": f"Target position of joint {joint.name}",
        }
        evidence.append(
            {
                "kind": f"joint.{joint.name}.arrival",
                "observe": OBSERVATION,
                "phases": ["after", "settled"],
                "measure": {"op": "distance_to", "fields": [joint.name], "frame": FRAME},
                "expect": {"arguments": {joint.name: argument}},
                "tolerance": {"absolute": tolerance},
                "settle": {"max_drift": profile.settle_max_drift_rad},
                "unit": "rad",
            }
        )
    budget_s = profile.move_deadline_s + (
        profile.settle_samples * profile.control_period_s
        + profile.stop_confirm_samples * profile.stop_confirm_period_s
    )
    return {
        "module_id": CAPABILITY_MOVE,
        "provides_capability": CAPABILITY_MOVE,
        "label": {"en": "Move arm joints"},
        "description": (
            "Move every joint to an absolute target within its declared limits, "
            "at the profile's bounded speed. Out-of-limit targets are refused."
        ),
        "params_schema": params,
        "timeout_ms": _ms(budget_s),
        "contract": {
            "schema": CONTRACT_SCHEMA,
            "actuates": True,
            "safety_class": "movement",
            "requires_safe_stop": True,
            "cancellable": True,
            # A lost connection mid-move leaves the result unknown; a retry
            # could move the arm again, so it is not declared idempotent.
            "idempotent": False,
            "effects": ["arm.joints-moved"],
            "requires": ["arm.limits-accepted", "arm.workspace-clear"],
            "evidence": evidence,
            "expected_duration_ms": _ms(budget_s),
        },
    }


def halt_row(profile: ArmProfile) -> dict[str, Any]:
    evidence: list[dict[str, Any]] = []
    if profile.stop_mode == STOP_HOLD:
        for joint in profile.joints:
            evidence.append(
                {
                    "kind": f"joint.{joint.name}.held",
                    "observe": OBSERVATION,
                    "phases": ["before", "after"],
                    "measure": {"op": "delta", "fields": [joint.name], "frame": FRAME},
                    "expect": {"value": 0},
                    "tolerance": {"absolute": profile.stop_max_drift_rad},
                    "unit": "rad",
                }
            )
    contract: dict[str, Any] = {
        "schema": CONTRACT_SCHEMA,
        "actuates": True,
        "safety_class": "controlled",
        "requires_safe_stop": False,
        "cancellable": False,
        "idempotent": True,
        "role": "safe_stop",
        "effects": ["arm.stopped"],
        "expected_duration_ms": _ms(
            profile.stop_confirm_samples * profile.stop_confirm_period_s
        ),
    }
    if evidence:
        contract["evidence"] = evidence
    return {
        "module_id": CAPABILITY_HALT,
        "provides_capability": CAPABILITY_HALT,
        "label": {"en": "Stop the arm"},
        "description": (
            f"Stop the arm ({profile.stop_mode}); completed only when the read-back confirms it."
        ),
        "params_schema": {},
        "timeout_ms": contract["expected_duration_ms"],
        "contract": contract,
    }


def capability_rows(profile: ArmProfile) -> list[dict[str, Any]]:
    """Every capability this adapter serves for ``profile``, in a stable order."""
    return [state_row(profile), move_row(profile), halt_row(profile)]
