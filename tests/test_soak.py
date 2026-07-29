from __future__ import annotations

from pathlib import Path

from flyto_robotics.cli import dry_run_plan
from flyto_robotics.soak import (
    SOAK_REPORT_CONTRACT_VERSION,
    render_soak_junit,
    render_soak_markdown,
    run_deterministic_soak,
)

PROJECT_ROOT = Path(__file__).resolve().parent.parent
JOB = PROJECT_ROOT / "examples/jobs/pharmacy-to-ward.json"
PLAN = PROJECT_ROOT / "examples/plans/careflow-human-gate.json"


def test_real_ai_plan_is_deterministic_across_repeated_runs() -> None:
    report = run_deterministic_soak(
        runs=5,
        run_once=lambda: dry_run_plan(JOB, PLAN),
    )
    assert report["contract_version"] == SOAK_REPORT_CONTRACT_VERSION
    assert report["passed"] is True
    assert report["passed_runs"] == 5
    assert report["unique_fingerprints"] == 1
    assert "Verdict: **PASS**" in render_soak_markdown(report)
    assert 'failures="0"' in render_soak_junit(report)


def test_soak_exposes_one_bad_and_one_nondeterministic_run() -> None:
    call_count = 0

    def runner() -> dict[str, object]:
        nonlocal call_count
        call_count += 1
        result = dry_run_plan(JOB, PLAN)
        if call_count == 2:
            result["status"] = "failed"
            result["reason"] = "injected_test_failure"
        return result

    report = run_deterministic_soak(runs=2, run_once=runner)
    assert report["passed"] is False
    assert report["failed_runs"] == 1
    assert report["deterministic"] is False
    assert 'failures="2"' in render_soak_junit(report)


def test_soak_rejects_unbounded_run_counts() -> None:
    try:
        run_deterministic_soak(runs=501, run_once=lambda: dry_run_plan(JOB, PLAN))
    except ValueError as exc:
        assert "between 1 and 500" in str(exc)
    else:
        raise AssertionError("unbounded soak count was accepted")
