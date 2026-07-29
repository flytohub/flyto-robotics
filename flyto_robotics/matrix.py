"""Aggregation and evidence rendering for independent Gazebo lab runs."""

from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

LAB_MATRIX_CONTRACT_VERSION = "flyto.robotics.lab-matrix.v1"


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _finite_metric(metrics: dict[str, object], name: str) -> float | None:
    value = metrics.get(name)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        return None
    return float(value)


def aggregate_lab_reports(report_paths: list[Path]) -> dict[str, object]:
    """Fail closed unless all bounded reports are valid, passing, and consistent."""
    if not 1 <= len(report_paths) <= 20:
        raise ValueError("between 1 and 20 Gazebo reports are required")
    entries: list[dict[str, object]] = []
    scenario_ids: set[str] = set()
    check_signatures: set[str] = set()
    for index, path in enumerate(report_paths, start=1):
        try:
            decoded = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"unreadable Gazebo report: {path}") from exc
        if not isinstance(decoded, dict):
            raise ValueError(f"Gazebo report must be an object: {path}")
        checks_raw = decoded.get("checks")
        checks = checks_raw if isinstance(checks_raw, list) else []
        signature_items = [
            (item.get("name"), item.get("passed"))
            for item in checks
            if isinstance(item, dict)
        ]
        signature = hashlib.sha256(
            json.dumps(signature_items, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        check_signatures.add(signature)
        scenario_id = decoded.get("scenario_id")
        if isinstance(scenario_id, str):
            scenario_ids.add(scenario_id)
        metrics_raw = decoded.get("metrics")
        metrics = metrics_raw if isinstance(metrics_raw, dict) else {}
        check_count = len(checks)
        passed_checks = sum(
            1
            for item in checks
            if isinstance(item, dict) and item.get("passed") is True
        )
        entry_passed = bool(
            decoded.get("contract_version") == "flyto.robotics.lab-report.v1"
            and decoded.get("passed") is True
            and check_count > 0
            and passed_checks == check_count
        )
        entries.append(
            {
                "run": index,
                "run_id": path.parent.name[:128],
                "path": str(path),
                "sha256": _sha256(path),
                "passed": entry_passed,
                "check_count": check_count,
                "passed_checks": passed_checks,
                "elapsed_seconds": _finite_metric(metrics, "elapsed_seconds"),
                "safety_stop_count": metrics.get("safety_stop_count"),
                "event_count": metrics.get("event_count"),
                "gazebo_world_displacement": _finite_metric(
                    metrics, "gazebo_world_displacement"
                ),
            }
        )
    elapsed_values = [
        float(entry["elapsed_seconds"])
        for entry in entries
        if isinstance(entry["elapsed_seconds"], (int, float))
    ]
    displacement_values = [
        float(entry["gazebo_world_displacement"])
        for entry in entries
        if isinstance(entry["gazebo_world_displacement"], (int, float))
    ]
    passed_runs = sum(1 for entry in entries if entry["passed"] is True)
    consistent = len(scenario_ids) == 1 and len(check_signatures) == 1
    return {
        "contract_version": LAB_MATRIX_CONTRACT_VERSION,
        "generated_at": _timestamp(),
        "passed": passed_runs == len(entries) and consistent,
        "scenario_id": next(iter(scenario_ids), None),
        "requested_runs": len(entries),
        "passed_runs": passed_runs,
        "failed_runs": len(entries) - passed_runs,
        "pass_rate": round(passed_runs / len(entries), 6),
        "consistent_assertions": consistent,
        "metrics": {
            "elapsed_seconds_min": min(elapsed_values, default=None),
            "elapsed_seconds_max": max(elapsed_values, default=None),
            "elapsed_seconds_mean": (
                round(sum(elapsed_values) / len(elapsed_values), 6)
                if elapsed_values
                else None
            ),
            "world_displacement_min": min(displacement_values, default=None),
            "world_displacement_max": max(displacement_values, default=None),
        },
        "runs": entries,
    }


def render_matrix_markdown(report: dict[str, object]) -> str:
    """Render the Gazebo cold-start matrix for human review."""
    verdict = "PASS" if report.get("passed") is True else "FAIL"
    lines = [
        "# Gazebo Independent-Run Matrix",
        "",
        f"- Scenario: `{report.get('scenario_id')}`",
        f"- Verdict: **{verdict}**",
        f"- Runs: `{report.get('passed_runs')}/{report.get('requested_runs')}`",
        f"- Pass rate: `{report.get('pass_rate')}`",
        f"- Assertion set consistent: `{report.get('consistent_assertions')}`",
        f"- Generated: `{report.get('generated_at')}`",
        "",
        "| Run | Result | Checks | Seconds | Stops | Events | World displacement |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    entries = report.get("runs")
    if isinstance(entries, list):
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            verdict = "PASS" if entry.get("passed") is True else "FAIL"
            lines.append(
                f"| `{entry.get('run_id')}` | {verdict} | "
                f"{entry.get('passed_checks')}/{entry.get('check_count')} | "
                f"{entry.get('elapsed_seconds')} | {entry.get('safety_stop_count')} | "
                f"{entry.get('event_count')} | "
                f"{entry.get('gazebo_world_displacement')} |"
            )
    lines.extend(["", "## Aggregate metrics", "", "```json"])
    lines.append(json.dumps(report.get("metrics", {}), indent=2, sort_keys=True))
    lines.extend(["```", ""])
    return "\n".join(lines)


def render_matrix_junit(report: dict[str, object]) -> str:
    """Render one JUnit case per independent Gazebo run plus consistency."""
    entries_raw = report.get("runs")
    entries = (
        [entry for entry in entries_raw if isinstance(entry, dict)]
        if isinstance(entries_raw, list)
        else []
    )
    failures = sum(1 for entry in entries if entry.get("passed") is not True)
    if report.get("consistent_assertions") is not True:
        failures += 1
    suite = ET.Element(
        "testsuite",
        {
            "name": "robotics.gazebo-matrix",
            "tests": str(len(entries) + 1),
            "failures": str(failures),
        },
    )
    for entry in entries:
        case = ET.SubElement(
            suite,
            "testcase",
            {
                "name": str(entry.get("run_id", "unknown")),
                "classname": "robotics.gazebo",
            },
        )
        if entry.get("passed") is not True:
            detail = (
                f"checks={entry.get('passed_checks')}/{entry.get('check_count')}"
            )
            failure = ET.SubElement(case, "failure", {"message": detail})
            failure.text = detail
    consistent_case = ET.SubElement(
        suite,
        "testcase",
        {"name": "consistent-assertions", "classname": "robotics.gazebo"},
    )
    if report.get("consistent_assertions") is not True:
        detail = "scenario IDs or assertion sets differ"
        failure = ET.SubElement(consistent_case, "failure", {"message": detail})
        failure.text = detail
    return ET.tostring(suite, encoding="unicode", xml_declaration=True)
