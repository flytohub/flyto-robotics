"""The robot's own cargo sensor: one ``std_msgs/msg/Bool`` topic, read fresh.

``sensing.cargo_present`` answers whether a load is on the robot now. Flyto2
Cloud admits the last step of a delivery only on the literal boolean ``True``
in ``cargo_present`` (``services/space_tasks/cargo_vocabulary.py``), so this
module never produces a value it did not read:

* The topic is the one ``FLYTO_ROS2_CARGO_TOPIC`` names. Only a served
  simulation whose graph carries the simulator marker (``/clock``) falls back
  to the shared interface's ``/flyto/cargo_present``, which the simulated tray
  publishes: a hardware robot declares the capability only on a topic
  configured for it, so simulated state can never answer for a real robot.
  The capability is declared only when the topic is on the robot's graph as
  ``std_msgs/msg/Bool``, so a robot without a cargo sensor declares nothing
  and a planner has no resource offering it. Every result names the
  deployment mode it was read in.
* A reading is the ``data`` of one message, and only a literal boolean. A
  message without one is no reading.
* Only a reading received within ``FLYTO_ROS2_CARGO_MAX_AGE_S`` (default
  2 s) answers. None at all, or only an older one, is a failed call with a
  typed reason and no value: a missing reading is never false and never true.
* A message without a boolean discards the last reading: a sensor that just
  sent garbage is not still reporting its previous value.

``std_msgs/msg/Bool`` carries no stamp, so age is measured from when this host
received the message. The subscription is volatile: a latched value published
before the call is never delivered as if it were new. A publisher of this
topic (a tray sensor driver, or the simulated tray in
``sim/fetch_return/cargo_bridge.py``) therefore republishes its current value
at least ``MIN_PUBLISH_HZ`` times a second and never relies on latching; at
2 Hz a reading is at most 0.5 s old, inside the default 2 s window even with
one message lost. This is the one place that contract is written down.

Pure: no ROS and no socket. The rosbridge backend records messages here and
builds the call's result from :func:`call_result`.
"""

from __future__ import annotations

import math
import os
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from .adapter_contract import OUTCOME_COMPLETED, OUTCOME_FAILED, OUTCOME_REFUSED, CallResult

CAPABILITY_ID = "sensing.cargo_present"
MESSAGE_TYPE = "std_msgs/msg/Bool"
TOPIC_ENV = "FLYTO_ROS2_CARGO_TOPIC"
MAX_AGE_ENV = "FLYTO_ROS2_CARGO_MAX_AGE_S"
#: The shared interface's topic: what the simulated tray publishes. The default
#: only for a served simulation with its marker on the graph (``served_topic``).
CONVENTIONAL_TOPIC = "/flyto/cargo_present"
#: A publisher repeats its current value at least this often (see above).
MIN_PUBLISH_HZ = 2.0
DEFAULT_MAX_AGE_S = 2.0
MIN_MAX_AGE_S = 0.1
MAX_MAX_AGE_S = 30.0

#: The one field of a completed call's evidence Cloud reads as the answer.
RESULT_FIELD = "cargo_present"
#: Where the reading's provenance (or why there is none) is reported.
READING_KEY = "cargo_reading"
REASON_MISSING = "cargo_reading_missing"
REASON_STALE = "cargo_reading_stale"
REASON_NO_TOPIC = "cargo_topic_not_configured"
#: The graph did not confirm the configured deployment (or a simulation runs
#: with the marker check disabled): a value from the wrong robot is no answer.
REASON_DEPLOYMENT_UNCONFIRMED = "deployment_not_confirmed"
REASON_DISCONNECTED = "transport_disconnected"


def configured_topic() -> str | None:
    """The cargo topic configured for this resource, or None when none is."""
    return os.getenv(TOPIC_ENV, "").strip() or None


def served_topic(*, deployment_mode: str, sim_marker_on_graph: bool) -> str | None:
    """The topic to read: the configured one, else the shared one on a simulation.

    ``deployment_mode`` is the served one (``simulation`` / ``real``) and
    ``sim_marker_on_graph`` whether the graph carries the simulator marker.
    Anything but a marked simulation without a configured topic has none.
    """
    explicit = configured_topic()
    if explicit is not None:
        return explicit
    if deployment_mode == "simulation" and sim_marker_on_graph:
        return CONVENTIONAL_TOPIC
    return None


def refused_result(
    call_id: str, *, reason: str, detail: str, deployment_mode: str
) -> CallResult:
    """A refused read: no value, and the evidence names the reason and deployment."""
    return CallResult(
        call_id,
        OUTCOME_REFUSED,
        evidence={
            READING_KEY: {
                "message_type": MESSAGE_TYPE,
                "deployment_mode": deployment_mode,
                "reason": reason,
            }
        },
        detail=detail,
    )


def no_topic_result(call_id: str, *, deployment_mode: str) -> CallResult:
    """A read with no cargo topic for this deployment: refused, no value."""
    return refused_result(
        call_id,
        reason=REASON_NO_TOPIC,
        deployment_mode=deployment_mode,
        detail=(
            f"{REASON_NO_TOPIC}: no cargo topic for this {deployment_mode} deployment; "
            f"a robot reads one only when {TOPIC_ENV} names it"
        ),
    )


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
        """Keep a message's reading; False, and the last one discarded, when it has none."""
        data = message.get("data") if isinstance(message, Mapping) else None
        if type(data) is not bool:
            self.latest = None
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
