"""Deterministic repeated-run verification for AI-composed robot plans."""

from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from collections.abc import Callable
from datetime import datetime, timezone

SOAK_REPORT_CONTRACT_VERSION = "flyto.robotics.soak-report.v1"


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _fingerprint(result: dict[str, object]) -> str:
    canonical = dict(result)
    canonical.pop("generated_at", None)
    encoded = json.dumps(
        canonical,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _run_checks(result: dict[str, object]) -> tuple[bool, list[str]]:
    failures: list[str] = []
    if result.get("contract_version") != "flyto.robotics.result.v1":
        failures.append("result_contract")
    if result.get("status") != "succeeded":
        failures.append("terminal_status")
    if result.get("final_state") != "completed":
        failures.append("final_state")
    events_raw = result.get("events")
    events = events_raw if isinstance(events_raw, list) else []
    sequences = [
        event.get("sequence")
        for event in events
        if isinstance(event, dict) and isinstance(event.get("sequence"), int)
    ]
    if len(sequences) != len(events) or sequences != list(range(1, len(events) + 1)):
        failures.append("event_sequence_contiguous")
    kinds = {
        event.get("kind")
        for event in events
        if isinstance(event, dict) and isinstance(event.get("kind"), str)
    }
    capabilities = {
        event.get("capability")
        for event in events
        if isinstance(event, dict) and isinstance(event.get("capability"), str)
    }
    required_kinds = {"mission_accepted", "mission_completed"}
    capability_evidence = {
        "ask_human": {"human_approval_requested", "human_approved"},
        "resume": {"resume_authorized"},
        "wait_until_clear": {"clearance_window_started", "clearance_blocked"},
    }
    for capability, evidence in capability_evidence.items():
        if capability in capabilities:
            required_kinds.update(evidence)
    for kind in sorted(required_kinds - kinds):
        failures.append(f"event:{kind}")
    elapsed = result.get("elapsed_seconds")
    if (
        isinstance(elapsed, bool)
        or not isinstance(elapsed, (int, float))
        or not math.isfinite(float(elapsed))
        or float(elapsed) <= 0
    ):
        failures.append("elapsed_seconds")
    return not failures, failures


def run_deterministic_soak(
    *,
    runs: int,
    run_once: Callable[[], dict[str, object]],
) -> dict[str, object]:
    """Run one plan repeatedly and prove invariants plus byte-level determinism."""
    if isinstance(runs, bool) or not isinstance(runs, int) or not 1 <= runs <= 500:
        raise ValueError("runs must be an integer between 1 and 500")
    entries: list[dict[str, object]] = []
    fingerprints: set[str] = set()
    for index in range(1, runs + 1):
        result = run_once()
        passed, failures = _run_checks(result)
        fingerprint = _fingerprint(result)
        fingerprints.add(fingerprint)
        events = result.get("events")
        entries.append(
            {
                "run": index,
                "passed": passed,
                "failures": failures,
                "fingerprint": fingerprint,
                "elapsed_seconds": result.get("elapsed_seconds"),
                "safety_stop_count": result.get("safety_stop_count"),
                "event_count": len(events) if isinstance(events, list) else 0,
            }
        )
    passed_runs = sum(1 for entry in entries if entry["passed"] is True)
    deterministic = len(fingerprints) == 1
    return {
        "contract_version": SOAK_REPORT_CONTRACT_VERSION,
        "generated_at": _timestamp(),
        "passed": passed_runs == runs and deterministic,
        "requested_runs": runs,
        "passed_runs": passed_runs,
        "failed_runs": runs - passed_runs,
        "pass_rate": round(passed_runs / runs, 6),
        "deterministic": deterministic,
        "unique_fingerprints": len(fingerprints),
        "runs": entries,
    }


def render_soak_markdown(report: dict[str, object]) -> str:
    """Render a concise, reviewer-readable soak report."""
    verdict = "PASS" if report.get("passed") is True else "FAIL"
    lines = [
        "# Deterministic AI Plan Soak Report",
        "",
        f"- Verdict: **{verdict}**",
        f"- Runs: `{report.get('passed_runs')}/{report.get('requested_runs')}`",
        f"- Pass rate: `{report.get('pass_rate')}`",
        f"- Unique fingerprints: `{report.get('unique_fingerprints')}`",
        f"- Generated: `{report.get('generated_at')}`",
        "",
        "| Run | Result | Elapsed | Stops | Events | Fingerprint |",
        "|---:|---:|---:|---:|---:|---|",
    ]
    entries = report.get("runs")
    if isinstance(entries, list):
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            result = "PASS" if entry.get("passed") is True else "FAIL"
            lines.append(
                f"| {entry.get('run')} | {result} | {entry.get('elapsed_seconds')} | "
                f"{entry.get('safety_stop_count')} | {entry.get('event_count')} | "
                f"`{entry.get('fingerprint')}` |"
            )
    return "\n".join(lines) + "\n"


def render_soak_junit(report: dict[str, object]) -> str:
    """Render every soak iteration and determinism as JUnit cases."""
    entries_raw = report.get("runs")
    entries = (
        [entry for entry in entries_raw if isinstance(entry, dict)]
        if isinstance(entries_raw, list)
        else []
    )
    failures = sum(1 for entry in entries if entry.get("passed") is not True)
    if report.get("deterministic") is not True:
        failures += 1
    suite = ET.Element(
        "testsuite",
        {
            "name": "robotics.deterministic-soak",
            "tests": str(len(entries) + 1),
            "failures": str(failures),
        },
    )
    for entry in entries:
        case = ET.SubElement(
            suite,
            "testcase",
            {
                "name": f"run-{entry.get('run')}",
                "classname": "robotics.soak",
            },
        )
        if entry.get("passed") is not True:
            detail = ",".join(str(item) for item in entry.get("failures", []))
            failure = ET.SubElement(case, "failure", {"message": detail})
            failure.text = detail
    deterministic_case = ET.SubElement(
        suite,
        "testcase",
        {"name": "deterministic-fingerprint", "classname": "robotics.soak"},
    )
    if report.get("deterministic") is not True:
        detail = f"unique_fingerprints={report.get('unique_fingerprints')}"
        failure = ET.SubElement(deterministic_case, "failure", {"message": detail})
        failure.text = detail
    return ET.tostring(suite, encoding="unicode", xml_declaration=True)
