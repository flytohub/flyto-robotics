"""SIMULATION: what the arm's and the mission's hosts push about the tray.

The signal is opt-in and pushed, never polled: the host that owns the MuJoCo
arm world (and runs the mission) appends one JSON line per event to a report
file, and the cargo bridge reads the new lines. Nothing here reads, steps or locks anyone's
simulation. (flyto-modules-lerobot has no read-only state endpoint: its
``MujocoWorld.step()`` runs on every reading, and the Cloud e2e host's
``GET /sim/truth`` steps the world under its lock. So the bridge never calls
either one.)

Each line is ``flyto.sim-arm-report.v1`` with ``simulation: true``, a wall
clock ``at`` (``time.time()`` on the same host as the bridge) and a ``kind``:

``place``
    The result of ``manipulation.place`` into the tray proxy: ``outcome``,
    ``object_in_container`` (the camera's look after the release),
    ``released``, ``object_held``, and ``started_at``, the wall time taken
    before the place was commanded (``at`` is when it ended). The robot must
    have been parked from ``started_at`` on.
``tray_reading``
    The arm world's own reading of the tray proxy after the place:
    ``inside``, ``held``, ``block`` ``[x, y, z]`` and ``sim_t`` (simulated
    seconds). The bridge judges the block settled from a run of these.
``retract``
    The result of ``manipulation.move_joints`` back to ``pose`` ``ready``.
``unload``
    The mission delivered the load at home and took it off: ``outcome``.
``loss``
    The mission lost the load (a fault, or a test injection): ``cause``.
``mission_end``
    The mission ended, ``state`` ``done`` or ``failed``. Nothing reported
    before it can load the tray again.

The last three only ever empty the tray.

A malformed line is rejected and counted; it can never load the tray.
"""

from __future__ import annotations

import json
import math
import os
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SCHEMA = "flyto.sim-arm-report.v1"
KIND_PLACE = "place"
KIND_READING = "tray_reading"
KIND_RETRACT = "retract"
KIND_UNLOAD = "unload"
KIND_LOSS = "loss"
KIND_MISSION_END = "mission_end"

#: A block is settled after this much simulated time of consecutive readings
#: inside the tray proxy, released, and moving less than SETTLE_SPREAD_M.
SETTLE_S = 0.5
SETTLE_SPREAD_M = 0.002
SETTLE_MIN_READINGS = 3


@dataclass(frozen=True)
class ArmReport:
    kind: str
    at: float
    fields: Mapping[str, Any]


def _finite(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _envelope(kind: str, at: float | None) -> dict[str, Any]:
    return {
        "schema": SCHEMA,
        "simulation": True,
        "kind": kind,
        "at": time.time() if at is None else float(at),
    }


def place_report(
    outcome: str,
    evidence: Mapping[str, Any],
    *,
    started_at: float,
    at: float | None = None,
) -> dict[str, Any]:
    """A report from a ``manipulation.place`` CallResult's outcome and evidence.

    ``started_at`` is the wall time read before the place was commanded.
    """
    after = evidence.get("perception_after")
    inside = after.get("object_in_container") if isinstance(after, Mapping) else None
    return {
        **_envelope(KIND_PLACE, at),
        "started_at": float(started_at),
        "outcome": str(outcome),
        "object_in_container": inside is True,
        "released": evidence.get("released") is True,
        "object_held": evidence.get("object_held") is not False,
    }


def tray_reading(
    *, inside: bool, held: bool, block: Any, sim_t: float, at: float | None = None
) -> dict[str, Any]:
    return {
        **_envelope(KIND_READING, at),
        "inside": bool(inside),
        "held": bool(held),
        "block": [float(v) for v in block],
        "sim_t": float(sim_t),
    }


def retract_report(outcome: str, *, pose: str = "ready", at: float | None = None) -> dict[str, Any]:
    return {**_envelope(KIND_RETRACT, at), "outcome": str(outcome), "pose": str(pose)}


def unload_report(outcome: str, *, at: float | None = None) -> dict[str, Any]:
    return {**_envelope(KIND_UNLOAD, at), "outcome": str(outcome)}


def loss_report(cause: str, *, at: float | None = None) -> dict[str, Any]:
    return {**_envelope(KIND_LOSS, at), "cause": str(cause)}


def mission_end_report(state: str, *, at: float | None = None) -> dict[str, Any]:
    return {**_envelope(KIND_MISSION_END, at), "state": str(state)}


def parse_report(raw: Any) -> ArmReport | None:
    """A validated report, or None for anything else."""
    if not isinstance(raw, Mapping) or raw.get("schema") != SCHEMA:
        return None
    if raw.get("simulation") is not True:
        return None
    at = _finite(raw.get("at"))
    kind = raw.get("kind")
    if at is None:
        return None
    if kind == KIND_PLACE:
        flags = ("object_in_container", "released", "object_held")
        started_at = _finite(raw.get("started_at"))
        if (
            not isinstance(raw.get("outcome"), str)
            or any(not isinstance(raw.get(flag), bool) for flag in flags)
            or started_at is None
            or started_at > at
        ):
            return None
        fields = {
            "outcome": raw["outcome"],
            "started_at": started_at,
            **{flag: raw[flag] for flag in flags},
        }
    elif kind == KIND_READING:
        block = raw.get("block")
        sim_t = _finite(raw.get("sim_t"))
        if (
            not isinstance(raw.get("inside"), bool)
            or not isinstance(raw.get("held"), bool)
            or sim_t is None
            or not isinstance(block, list)
            or len(block) != 3
            or any(_finite(v) is None for v in block)
        ):
            return None
        fields = {
            "inside": raw["inside"],
            "held": raw["held"],
            "block": tuple(float(v) for v in block),
            "sim_t": sim_t,
        }
    elif kind == KIND_RETRACT:
        if not isinstance(raw.get("outcome"), str) or not isinstance(raw.get("pose"), str):
            return None
        fields = {"outcome": raw["outcome"], "pose": raw["pose"]}
    elif kind in (KIND_UNLOAD, KIND_LOSS, KIND_MISSION_END):
        name = {KIND_UNLOAD: "outcome", KIND_LOSS: "cause", KIND_MISSION_END: "state"}[kind]
        if not isinstance(raw.get(name), str):
            return None
        fields = {name: raw[name]}
    else:
        return None
    return ArmReport(kind=kind, at=at, fields=fields)


def placement_observed(report: ArmReport) -> bool:
    """The place completed, the camera saw the block inside, and the jaws let go."""
    f = report.fields
    return (
        report.kind == KIND_PLACE
        and f["outcome"] == "completed"
        and f["object_in_container"] is True
        and f["released"] is True
        and f["object_held"] is False
    )


def retracted(report: ArmReport) -> bool:
    return (
        report.kind == KIND_RETRACT
        and report.fields["outcome"] == "completed"
        and report.fields["pose"] == "ready"
    )


def reading_in_tray(report: ArmReport) -> bool:
    return report.kind == KIND_READING and report.fields["inside"] and not report.fields["held"]


def settled_run(readings: list[ArmReport]) -> list[ArmReport] | None:
    """The trailing run of readings that shows the block settled in the tray, or None.

    Only the readings after the last one that was out of the tray (or held)
    count; they must span ``SETTLE_S`` of simulated time, in order, with the
    block inside ``SETTLE_SPREAD_M`` of where the run began.
    """
    run: list[ArmReport] = []
    for reading in readings:
        if not reading_in_tray(reading):
            run = []
            continue
        if run and reading.fields["sim_t"] < run[-1].fields["sim_t"]:
            run = []
        run.append(reading)
    if len(run) < SETTLE_MIN_READINGS:
        return None
    if run[-1].fields["sim_t"] - run[0].fields["sim_t"] < SETTLE_S:
        return None
    origin = run[0].fields["block"]
    if any(math.dist(origin, r.fields["block"]) > SETTLE_SPREAD_M for r in run):
        return None
    return run


def append_report(path: Path, record: Mapping[str, Any]) -> None:
    """Append one report as a single line (one write on an O_APPEND descriptor)."""
    if parse_report(record) is None:
        raise ValueError("not a valid flyto.sim-arm-report.v1 record")
    line = (json.dumps(dict(record), separators=(",", ":"), allow_nan=False) + "\n").encode()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
    try:
        os.write(fd, line)
    finally:
        os.close(fd)


class ArmReportTail:
    """Reads the lines appended to a report file since the last read.

    It starts at the file's current end: a report written before the bridge
    started describes a place nobody saw the robot parked for.
    """

    def __init__(self, path: Path):
        self.path = Path(path)
        self.rejected = 0
        self._partial = b""
        try:
            self._offset = self.path.stat().st_size
        except FileNotFoundError:
            self._offset = 0

    def read_new(self) -> list[ArmReport]:
        try:
            size = self.path.stat().st_size
        except FileNotFoundError:
            return []
        if size < self._offset:  # truncated or replaced: read it from the start
            self._offset, self._partial = 0, b""
        if size == self._offset:
            return []
        with self.path.open("rb") as handle:
            handle.seek(self._offset)
            chunk = handle.read(size - self._offset)
        self._offset += len(chunk)
        data = self._partial + chunk
        lines = data.split(b"\n")
        self._partial = lines.pop()
        reports: list[ArmReport] = []
        for line in lines:
            if not line.strip():
                continue
            try:
                report = parse_report(json.loads(line))
            except ValueError:
                report = None
            if report is None:
                self.rejected += 1
            else:
                reports.append(report)
        return reports
