"""The robot's own cargo sensor: one ``std_msgs/msg/Bool`` topic, read fresh.

``sensing.cargo_present`` answers whether a load is on the robot now. Flyto2
Cloud admits the last step of a delivery only on the literal boolean ``True``
in ``cargo_present`` (``services/space_tasks/cargo_vocabulary.py``), so this
module never produces a value it did not read:

* The capability is opt-in per resource. ``FLYTO_ROS2_CARGO_TOPIC`` names the
  topic (by convention ``/flyto/cargo_present``); unset, the adapter does not
  declare the capability and a planner has no resource offering it.
* A reading is the ``data`` of one message, and only a literal boolean. A
  message without one is no reading.
* Only a reading received within ``FLYTO_ROS2_CARGO_MAX_AGE_S`` (default
  2 s) answers. None at all, or only an older one, is a failed call with a
  typed reason and no value: a missing reading is never false and never true.

``std_msgs/msg/Bool`` carries no stamp, so age is measured from when this host
received the message. The subscription is volatile: a latched value published
before the call is never delivered as if it were new.

Pure: no ROS and no socket. The rosbridge backend records messages here and
builds the call's result from :func:`call_result`.
"""

from __future__ import annotations

import math
import os
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from .adapter_contract import OUTCOME_COMPLETED, OUTCOME_FAILED, CallResult

CAPABILITY_ID = "sensing.cargo_present"
MESSAGE_TYPE = "std_msgs/msg/Bool"
TOPIC_ENV = "FLYTO_ROS2_CARGO_TOPIC"
MAX_AGE_ENV = "FLYTO_ROS2_CARGO_MAX_AGE_S"
#: The topic name the Flyto2 simulation publishes; a host still has to set it.
CONVENTIONAL_TOPIC = "/flyto/cargo_present"
DEFAULT_MAX_AGE_S = 2.0
MIN_MAX_AGE_S = 0.1
MAX_MAX_AGE_S = 30.0

#: The one field of a completed call's evidence Cloud reads as the answer.
RESULT_FIELD = "cargo_present"
#: Where the reading's provenance (or why there is none) is reported.
READING_KEY = "cargo_reading"
REASON_MISSING = "cargo_reading_missing"
REASON_STALE = "cargo_reading_stale"


def configured_topic() -> str:
    """The cargo topic configured for this resource, or ``""`` when none is."""
    return os.getenv(TOPIC_ENV, "").strip()


def max_age_s() -> float:
    """The freshness window in seconds.

    A value that is not a finite number inside 0.1..30 s is a configuration
    mistake; the default is used rather than a widened or narrowed window.
    """
    try:
        value = float(os.getenv(MAX_AGE_ENV, str(DEFAULT_MAX_AGE_S)))
    except ValueError:
        return DEFAULT_MAX_AGE_S
    if not math.isfinite(value) or not MIN_MAX_AGE_S <= value <= MAX_MAX_AGE_S:
        return DEFAULT_MAX_AGE_S
    return value


def interface_observation(topic: str) -> str:
    return f"{topic}:{MESSAGE_TYPE}"


@dataclass(frozen=True)
class Reading:
    value: bool
    received: float


@dataclass(frozen=True)
class Verdict:
    """A fresh value (``reason`` None), or why there is none."""

    value: bool | None
    reason: str | None
    age_s: float | None


class CargoSensor:
    """The newest boolean reading and when it was received (monotonic seconds)."""

    def __init__(self) -> None:
        self.latest: Reading | None = None

    def record(self, message: Any, *, received: float) -> bool:
        """Keep a message's reading; False (and nothing kept) when it has none."""
        data = message.get("data") if isinstance(message, Mapping) else None
        if type(data) is not bool:
            return False
        self.latest = Reading(data, received)
        return True

    def clear(self) -> None:
        self.latest = None

    def verdict(self, *, now: float, max_age_s: float) -> Verdict:
        latest = self.latest
        if latest is None:
            return Verdict(None, REASON_MISSING, None)
        age = max(0.0, now - latest.received)
        if age > max_age_s:
            return Verdict(None, REASON_STALE, age)
        return Verdict(latest.value, None, age)


def call_result(
    call_id: str,
    verdict: Verdict,
    *,
    topic: str,
    max_age: float,
    deployment_mode: str,
) -> CallResult:
    """The call's result: the value with its provenance, or a typed failure without one."""
    reading: dict[str, Any] = {
        "topic": topic,
        "message_type": MESSAGE_TYPE,
        "max_age_s": max_age,
        # ``simulation`` or ``real``: what the graph confirmed this reading came from.
        "deployment_mode": deployment_mode,
    }
    if verdict.age_s is not None:
        reading["age_s"] = round(verdict.age_s, 3)
    if verdict.reason is None and verdict.value is not None:
        return CallResult(
            call_id,
            OUTCOME_COMPLETED,
            evidence={RESULT_FIELD: verdict.value, READING_KEY: reading},
        )
    reason = verdict.reason or REASON_MISSING
    if reason == REASON_STALE:
        detail = (
            f"{reason}: the newest {topic} reading is {verdict.age_s:.2f} s old, "
            f"older than {max_age:g} s"
        )
    else:
        detail = f"{reason}: no {MESSAGE_TYPE} reading on {topic} within {max_age:g} s"
    return CallResult(
        call_id,
        OUTCOME_FAILED,
        evidence={READING_KEY: {**reading, "reason": reason}},
        detail=detail,
    )
