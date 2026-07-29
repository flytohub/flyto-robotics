"""Gazebo laboratory scenario validation, evidence checks, and report rendering."""

from __future__ import annotations

import hashlib
import json
import math
import re
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath

from .ai_planner import load_plan
from .contracts import load_job

LAB_SCENARIO_CONTRACT_VERSION = "flyto.robotics.lab-scenario.v1"
LAB_REPORT_CONTRACT_VERSION = "flyto.robotics.lab-report.v1"
IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")
MAX_SCENARIO_BYTES = 128 * 1024
MAX_REQUIRED_ITEMS = 64


class LabValidationError(ValueError):
    """Raised when a laboratory scenario or evidence envelope is unsafe."""


@dataclass(frozen=True)
class PoseBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass(frozen=True)
class LabExpectations:
    status: str
    gazebo_physics: bool
    min_safety_stop_count: int
    max_elapsed_seconds: float
    required_event_kinds: tuple[str, ...]
    required_capabilities: tuple[str, ...]
    required_actor_ids: tuple[str, ...]
    required_capture_labels: tuple[str, ...]
    min_world_displacement: float
    final_pose: PoseBounds


@dataclass(frozen=True)
class LabScenario:
    contract_version: str
    scenario_id: str
    title: str
    description: str
    world: str
    job: str
    plan: str
    model: str
    bridge: str
    soak_runs: int
    expectations: LabExpectations


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _require_mapping(value: object, label: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise LabValidationError(f"{label} must be an object")
    return value


def _require_exact_fields(
    value: dict[str, object],
    *,
    required: set[str],
    label: str,
) -> None:
    missing = sorted(required - value.keys())
    extras = sorted(value.keys() - required)
    if missing:
        raise LabValidationError(f"{label} is missing fields: {', '.join(missing)}")
    if extras:
        raise LabValidationError(f"{label} has unsupported fields: {', '.join(extras)}")


def _identifier(value: object, label: str) -> str:
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise LabValidationError(f"{label} must be a safe identifier")
    return value


def _bounded_text(value: object, label: str, maximum: int) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > maximum:
        raise LabValidationError(f"{label} must contain 1 to {maximum} characters")
    if any(ord(character) < 32 and character not in "\n\t" for character in value):
        raise LabValidationError(f"{label} contains unsupported control characters")
    return value.strip()


def _safe_relative_path(value: object, label: str, suffixes: tuple[str, ...]) -> str:
    if not isinstance(value, str) or not value:
        raise LabValidationError(f"{label} must be a relative path")
    candidate = PurePosixPath(value)
    if candidate.is_absolute() or ".." in candidate.parts or "\\" in value:
        raise LabValidationError(f"{label} must stay within the project")
    if candidate.suffix.lower() not in suffixes:
        raise LabValidationError(f"{label} has an unsupported file extension")
    return candidate.as_posix()


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LabValidationError(f"{label} must be a number")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise LabValidationError(f"{label} must be finite")
    return parsed


def _identifier_list(value: object, label: str) -> tuple[str, ...]:
    if not isinstance(value, list) or len(value) > MAX_REQUIRED_ITEMS:
        raise LabValidationError(f"{label} must be a bounded array")
    parsed = tuple(_identifier(item, f"{label} item") for item in value)
    if len(parsed) != len(set(parsed)):
        raise LabValidationError(f"{label} must not contain duplicates")
    return parsed


def parse_lab_scenario(value: object) -> LabScenario:
    """Validate a strict, language-neutral Gazebo evidence scenario."""
    root = _require_mapping(value, "scenario")
    root_fields = {
        "contract_version",
        "scenario_id",
        "title",
        "description",
        "assets",
        "soak_runs",
        "expectations",
    }
    _require_exact_fields(root, required=root_fields, label="scenario")
    if root["contract_version"] != LAB_SCENARIO_CONTRACT_VERSION:
        raise LabValidationError("unsupported lab scenario contract_version")

    assets = _require_mapping(root["assets"], "assets")
    _require_exact_fields(
        assets,
        required={"world", "job", "plan", "model", "bridge"},
        label="assets",
    )
    expected = _require_mapping(root["expectations"], "expectations")
    _require_exact_fields(
        expected,
        required={
            "status",
            "gazebo_physics",
            "min_safety_stop_count",
            "max_elapsed_seconds",
            "required_event_kinds",
            "required_capabilities",
            "required_actor_ids",
            "required_capture_labels",
            "min_world_displacement",
            "final_pose",
        },
        label="expectations",
    )
    pose = _require_mapping(expected["final_pose"], "expectations.final_pose")
    _require_exact_fields(
        pose,
        required={"min_x", "max_x", "min_y", "max_y"},
        label="expectations.final_pose",
    )

    soak_runs = root["soak_runs"]
    if isinstance(soak_runs, bool) or not isinstance(soak_runs, int):
        raise LabValidationError("soak_runs must be an integer")
    if not 1 <= soak_runs <= 500:
        raise LabValidationError("soak_runs must be between 1 and 500")
    status = expected["status"]
    if status not in {"succeeded", "failed", "cancelled"}:
        raise LabValidationError("expectations.status is invalid")
    gazebo_physics = expected["gazebo_physics"]
    if not isinstance(gazebo_physics, bool):
        raise LabValidationError("expectations.gazebo_physics must be boolean")
    minimum_stops = expected["min_safety_stop_count"]
    if isinstance(minimum_stops, bool) or not isinstance(minimum_stops, int):
        raise LabValidationError("min_safety_stop_count must be an integer")
    if not 0 <= minimum_stops <= 1000:
        raise LabValidationError("min_safety_stop_count is outside the supported range")
    max_elapsed = _finite_number(
        expected["max_elapsed_seconds"], "max_elapsed_seconds"
    )
    if not 0.1 <= max_elapsed <= 3600:
        raise LabValidationError("max_elapsed_seconds is outside the supported range")
    min_world_displacement = _finite_number(
        expected["min_world_displacement"], "min_world_displacement"
    )
    if not 0 <= min_world_displacement <= 10000:
        raise LabValidationError("min_world_displacement is outside the supported range")

    bounds = PoseBounds(
        min_x=_finite_number(pose["min_x"], "final_pose.min_x"),
        max_x=_finite_number(pose["max_x"], "final_pose.max_x"),
        min_y=_finite_number(pose["min_y"], "final_pose.min_y"),
        max_y=_finite_number(pose["max_y"], "final_pose.max_y"),
    )
    if bounds.min_x > bounds.max_x or bounds.min_y > bounds.max_y:
        raise LabValidationError("final_pose minimums must not exceed maximums")

    return LabScenario(
        contract_version=LAB_SCENARIO_CONTRACT_VERSION,
        scenario_id=_identifier(root["scenario_id"], "scenario_id"),
        title=_bounded_text(root["title"], "title", 160),
        description=_bounded_text(root["description"], "description", 2000),
        world=_safe_relative_path(assets["world"], "assets.world", (".sdf",)),
        job=_safe_relative_path(assets["job"], "assets.job", (".json",)),
        plan=_safe_relative_path(assets["plan"], "assets.plan", (".json",)),
        model=_safe_relative_path(assets["model"], "assets.model", (".sdf",)),
        bridge=_safe_relative_path(
            assets["bridge"], "assets.bridge", (".yaml", ".yml")
        ),
        soak_runs=soak_runs,
        expectations=LabExpectations(
            status=status,
            gazebo_physics=gazebo_physics,
            min_safety_stop_count=minimum_stops,
            max_elapsed_seconds=max_elapsed,
            required_event_kinds=_identifier_list(
                expected["required_event_kinds"], "required_event_kinds"
            ),
            required_capabilities=_identifier_list(
                expected["required_capabilities"], "required_capabilities"
            ),
            required_actor_ids=_identifier_list(
                expected["required_actor_ids"], "required_actor_ids"
            ),
            required_capture_labels=_identifier_list(
                expected["required_capture_labels"], "required_capture_labels"
            ),
            min_world_displacement=min_world_displacement,
            final_pose=bounds,
        ),
    )


def load_lab_scenario(path: Path, *, project_root: Path) -> LabScenario:
    """Load a scenario after size, JSON, asset, contract, and XML validation."""
    try:
        if path.stat().st_size > MAX_SCENARIO_BYTES:
            raise LabValidationError("lab scenario exceeds 131072 bytes")
        decoded = json.loads(path.read_text(encoding="utf-8"))
    except LabValidationError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise LabValidationError("lab scenario must contain readable UTF-8 JSON") from exc
    scenario = parse_lab_scenario(decoded)
    for relative in (
        scenario.world,
        scenario.job,
        scenario.plan,
        scenario.model,
        scenario.bridge,
    ):
        asset = project_root / relative
        if not asset.is_file():
            raise LabValidationError(f"scenario asset is missing: {relative}")
    ET.parse(project_root / scenario.world)
    ET.parse(project_root / scenario.model)
    load_job(project_root / scenario.job)
    load_plan(project_root / scenario.plan)
    return scenario


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _check(checks: list[dict[str, object]], name: str, passed: bool, detail: str) -> None:
    checks.append({"name": name, "passed": bool(passed), "detail": detail[:512]})


def evaluate_lab_result(
    scenario: LabScenario,
    result: object,
    *,
    project_root: Path,
    result_path: Path | None = None,
    evidence_dir: Path | None = None,
) -> dict[str, object]:
    """Evaluate result semantics, safety evidence, captures, and provenance."""
    decoded = _require_mapping(result, "result")
    checks: list[dict[str, object]] = []
    expected = scenario.expectations
    events_raw = decoded.get("events", [])
    events = events_raw if isinstance(events_raw, list) else []
    simulation_raw = decoded.get("simulation", {})
    simulation = simulation_raw if isinstance(simulation_raw, dict) else {}

    _check(
        checks,
        "result_contract",
        decoded.get("contract_version") == "flyto.robotics.result.v1",
        str(decoded.get("contract_version")),
    )
    _check(
        checks,
        "terminal_status",
        decoded.get("status") == expected.status,
        f"expected={expected.status}; actual={decoded.get('status')}",
    )
    _check(
        checks,
        "gazebo_physics",
        simulation.get("gazebo_physics") is expected.gazebo_physics,
        f"expected={expected.gazebo_physics}; actual={simulation.get('gazebo_physics')}",
    )
    stops = decoded.get("safety_stop_count")
    stop_count = stops if isinstance(stops, int) and not isinstance(stops, bool) else -1
    _check(
        checks,
        "safety_stop_count",
        stop_count >= expected.min_safety_stop_count,
        f"minimum={expected.min_safety_stop_count}; actual={stop_count}",
    )
    elapsed_raw = decoded.get("elapsed_seconds")
    elapsed = (
        float(elapsed_raw)
        if isinstance(elapsed_raw, (int, float))
        and not isinstance(elapsed_raw, bool)
        and math.isfinite(float(elapsed_raw))
        else math.inf
    )
    _check(
        checks,
        "elapsed_budget",
        0 <= elapsed <= expected.max_elapsed_seconds,
        f"maximum={expected.max_elapsed_seconds}; actual={elapsed}",
    )

    sequences = [
        item.get("sequence")
        for item in events
        if isinstance(item, dict) and isinstance(item.get("sequence"), int)
    ]
    _check(
        checks,
        "event_sequence_contiguous",
        len(sequences) == len(events) and sequences == list(range(1, len(events) + 1)),
        f"events={len(events)}; sequences={len(sequences)}",
    )
    kinds = {
        str(item.get("kind"))
        for item in events
        if isinstance(item, dict) and isinstance(item.get("kind"), str)
    }
    capabilities = {
        str(item.get("capability"))
        for item in events
        if isinstance(item, dict) and isinstance(item.get("capability"), str)
    }
    actors = {
        str(item.get("actor_id"))
        for item in events
        if isinstance(item, dict) and isinstance(item.get("actor_id"), str)
    }
    for kind in expected.required_event_kinds:
        _check(checks, f"event:{kind}", kind in kinds, f"observed={sorted(kinds)}")
    for capability in expected.required_capabilities:
        _check(
            checks,
            f"capability:{capability}",
            capability in capabilities,
            f"observed={sorted(capabilities)}",
        )
    for actor in expected.required_actor_ids:
        _check(checks, f"actor:{actor}", actor in actors, f"observed={sorted(actors)}")

    pose_raw = decoded.get("final_pose")
    pose = pose_raw if isinstance(pose_raw, dict) else {}
    x = pose.get("x")
    y = pose.get("y")
    pose_valid = all(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        for value in (x, y)
    )
    bounds = expected.final_pose
    within_bounds = bool(
        pose_valid
        and bounds.min_x <= float(x) <= bounds.max_x
        and bounds.min_y <= float(y) <= bounds.max_y
    )
    _check(
        checks,
        "final_pose_bounds",
        within_bounds,
        f"x={x}; y={y}; x_range=[{bounds.min_x},{bounds.max_x}]; "
        f"y_range=[{bounds.min_y},{bounds.max_y}]",
    )

    captures: list[dict[str, str]] = []
    driver_evidence: dict[str, object] = {}
    if evidence_dir is not None and evidence_dir.is_dir():
        driver_manifest = evidence_dir / "driver-manifest.json"
        if driver_manifest.is_file():
            try:
                loaded_manifest = json.loads(driver_manifest.read_text(encoding="utf-8"))
                if isinstance(loaded_manifest, dict):
                    driver_evidence = loaded_manifest
            except (OSError, UnicodeError, json.JSONDecodeError):
                driver_evidence = {}
        for image in sorted(evidence_dir.glob("*.png")):
            captures.append(
                {
                    "name": image.name,
                    "sha256": _sha256(image),
                    "path": f"{evidence_dir.name}/{image.name}",
                }
            )
    capture_names = {str(item["name"]) for item in captures}
    for label in expected.required_capture_labels:
        matched = any(
            f"-{label}-" in name or name.startswith(f"{label}-")
            for name in capture_names
        )
        _check(
            checks,
            f"capture:{label}",
            matched,
            f"observed={sorted(capture_names)}",
        )
    _check(
        checks,
        "driver_evidence_contract",
        driver_evidence.get("contract_version")
        == "flyto.robotics.lab-driver-evidence.v1",
        str(driver_evidence.get("contract_version")),
    )
    displacement_raw = driver_evidence.get("world_displacement")
    world_displacement = (
        float(displacement_raw)
        if isinstance(displacement_raw, (int, float))
        and not isinstance(displacement_raw, bool)
        and math.isfinite(float(displacement_raw))
        else None
    )
    _check(
        checks,
        "gazebo_world_displacement",
        world_displacement is not None
        and world_displacement >= expected.min_world_displacement,
        f"minimum={expected.min_world_displacement}; actual={world_displacement}",
    )

    provenance: list[dict[str, str]] = []
    for relative in (
        scenario.world,
        scenario.job,
        scenario.plan,
        scenario.model,
        scenario.bridge,
    ):
        asset = project_root / relative
        provenance.append(
            {"path": relative, "sha256": _sha256(asset), "kind": "scenario_input"}
        )
    if result_path is not None and result_path.is_file():
        provenance.append(
            {
                "path": str(result_path),
                "sha256": _sha256(result_path),
                "kind": "mission_result",
            }
        )
    if evidence_dir is not None:
        driver_manifest = evidence_dir / "driver-manifest.json"
        if driver_manifest.is_file():
            provenance.append(
                {
                    "path": f"{evidence_dir.name}/{driver_manifest.name}",
                    "sha256": _sha256(driver_manifest),
                    "kind": "driver_evidence",
                }
            )

    return {
        "contract_version": LAB_REPORT_CONTRACT_VERSION,
        "scenario_id": scenario.scenario_id,
        "title": scenario.title,
        "generated_at": _timestamp(),
        "passed": all(bool(item["passed"]) for item in checks),
        "checks": checks,
        "metrics": {
            "elapsed_seconds": None if math.isinf(elapsed) else elapsed,
            "safety_stop_count": stop_count,
            "event_count": len(events),
            "capture_count": len(captures),
            "final_pose": pose if pose_valid else None,
            "gazebo_world_displacement": world_displacement,
        },
        "captures": captures,
        "provenance": provenance,
    }


def render_lab_markdown(report: dict[str, object]) -> str:
    """Render a human-reviewable report without hiding failed checks."""
    verdict = "PASS" if report.get("passed") is True else "FAIL"
    lines = [
        f"# Gazebo Lab Report — {report.get('title', 'Untitled')}",
        "",
        f"- Scenario: `{report.get('scenario_id')}`",
        f"- Verdict: **{verdict}**",
        f"- Generated: `{report.get('generated_at')}`",
        "",
        "## Checks",
        "",
        "| Check | Result | Detail |",
        "|---|---:|---|",
    ]
    checks = report.get("checks", [])
    if isinstance(checks, list):
        for item in checks:
            if not isinstance(item, dict):
                continue
            result = "PASS" if item.get("passed") is True else "FAIL"
            detail = str(item.get("detail", "")).replace("|", "\\|").replace("\n", " ")
            lines.append(f"| `{item.get('name')}` | {result} | {detail} |")
    lines.extend(["", "## Metrics", "", "```json"])
    lines.append(json.dumps(report.get("metrics", {}), ensure_ascii=False, indent=2))
    lines.extend(["```", "", "## Gazebo images", ""])
    captures = report.get("captures", [])
    if isinstance(captures, list) and captures:
        for capture in captures:
            if isinstance(capture, dict):
                lines.append(f"![{capture.get('name')}]({capture.get('path')})")
                lines.append("")
    else:
        lines.append("_No image evidence was supplied._")
        lines.append("")
    lines.extend(
        [
            "## Provenance",
            "",
            "| Artifact | Kind | SHA-256 |",
            "|---|---|---|",
        ]
    )
    provenance = report.get("provenance", [])
    if isinstance(provenance, list):
        for item in provenance:
            if isinstance(item, dict):
                lines.append(
                    f"| `{item.get('path')}` | {item.get('kind')} | "
                    f"`{item.get('sha256')}` |"
                )
    return "\n".join(lines) + "\n"


def render_lab_junit(report: dict[str, object]) -> str:
    """Render each evidence assertion as one CI-visible JUnit testcase."""
    checks = report.get("checks", [])
    normalized = (
        [item for item in checks if isinstance(item, dict)]
        if isinstance(checks, list)
        else []
    )
    failures = sum(1 for item in normalized if item.get("passed") is not True)
    suite = ET.Element(
        "testsuite",
        {
            "name": str(report.get("scenario_id", "gazebo-lab")),
            "tests": str(len(normalized)),
            "failures": str(failures),
        },
    )
    for item in normalized:
        case = ET.SubElement(
            suite,
            "testcase",
            {"name": str(item.get("name", "unnamed")), "classname": "gazebo.lab"},
        )
        if item.get("passed") is not True:
            failure = ET.SubElement(case, "failure", {"message": str(item.get("detail", ""))})
            failure.text = str(item.get("detail", ""))
    return ET.tostring(suite, encoding="unicode", xml_declaration=True)


def write_text_atomic(destination: Path, value: str) -> None:
    """Write a UTF-8 report atomically."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            prefix=f".{destination.name}.",
            suffix=".tmp",
            dir=destination.parent,
            delete=False,
        ) as handle:
            handle.write(value)
            handle.flush()
            temporary_path = Path(handle.name)
        temporary_path.replace(destination)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
