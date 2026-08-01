from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/benchmark_robot_mcp.py"


def _module():
    spec = importlib.util.spec_from_file_location("benchmark_robot_mcp", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_real_mcp_benchmark_runs_distinct_tiers_and_verifies_digest(
    tmp_path: Path,
) -> None:
    benchmark = _module()

    report = benchmark.run_benchmark(case_count=6, minimum_cases=6)
    evidence_digest = report.pop("evidence_sha256")

    assert report["passed"] is True
    assert report["real_execution"]["mocked"] is False
    assert report["real_execution"]["transport"] == "production-stdio-subprocess"
    assert report["distinct_case_count"] == 6
    assert report["successes"] == 6
    assert set(report["tools"]) == benchmark.EXPECTED_TOOLS
    assert set(report["tiers"]) == {"standard", "intermediate", "advanced"}
    assert all(tier["passed"] for tier in report["tiers"].values())
    assert evidence_digest == benchmark._digest(report)

    report["evidence_sha256"] = evidence_digest
    output = benchmark.write_report_atomic(report, tmp_path)
    stored = json.loads(output.read_text(encoding="utf-8"))
    assert output.name == f"robot-mcp-benchmark-{evidence_digest}.json"
    assert stored == report
    assert output.stat().st_mode & 0o777 == 0o600


def test_cli_refuses_to_count_fewer_than_101_cases(tmp_path: Path) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--cases",
            "100",
            "--output-dir",
            str(tmp_path),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
        timeout=10,
    )

    assert completed.returncode == 2
    assert "at least 101" in completed.stderr
    assert list(tmp_path.iterdir()) == []
