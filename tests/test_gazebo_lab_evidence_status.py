"""A missing mission result is never a pass.

``run-gazebo-lab.sh`` checks that the launch produced ``mission-result.json``
before it evaluates anything, and reported ``exit ${launch_status:-3}`` when the
file was absent. The default only applies when ``launch_status`` is unset, and
it is always set -- so a launch that exited 0 without writing a result handed
back 0, and the run was recorded as green on the strength of a file that does
not exist.

These tests execute the real guard block lifted from the script, so they hold
the shipped text rather than a copy of it, and they need neither Docker nor
Gazebo.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/run-gazebo-lab.sh"

GUARD_HEAD = "if [[ ! -f /workspace/${run_directory}/mission-result.json ]]; then"
GUARD_TAIL = "\n    fi\n"
RESULT_PATH = "/workspace/${run_directory}/mission-result.json"

EVIDENCE_MISSING_STATUS = 3


def _guard_block() -> str:
    """The missing-result guard, exactly as the script ships it."""
    source = SCRIPT.read_text(encoding="utf-8")
    start = source.index(GUARD_HEAD)
    end = source.index(GUARD_TAIL, start) + len(GUARD_TAIL)
    return source[start:end]


def _run_guard(launch_status: int, result_file: Path) -> subprocess.CompletedProcess:
    """Run the guard with a given launch status and result-file state.

    The block lives inside a double-quoted ``bash -lc`` payload, so ``$`` is
    escaped in the source and unescaped here; the container-absolute result path
    is redirected at the test's own file.
    """
    block = _guard_block().replace("\\$", "$").replace(RESULT_PATH, str(result_file))
    script = f"set -euo pipefail\nlaunch_status={launch_status}\n{block}\nexit 0\n"
    return subprocess.run(
        ["bash", "-c", script], capture_output=True, text=True, check=False
    )


def test_the_guard_block_was_actually_found() -> None:
    block = _guard_block()
    assert "Gazebo did not produce a mission result" in block
    assert block.rstrip().endswith("fi")


def test_a_clean_launch_with_no_result_is_not_reported_as_success(
    tmp_path: Path,
) -> None:
    """The defect: launch exits 0, writes nothing, and the run reads as green."""
    completed = _run_guard(0, tmp_path / "mission-result.json")
    assert completed.returncode != 0
    assert "Gazebo did not produce a mission result" in completed.stderr


def test_the_missing_evidence_status_is_deterministic(tmp_path: Path) -> None:
    codes = {
        _run_guard(0, tmp_path / "mission-result.json").returncode for _ in range(3)
    }
    assert codes == {EVIDENCE_MISSING_STATUS}


def test_a_failing_launch_keeps_its_own_status(tmp_path: Path) -> None:
    """A nonzero launch status is the more precise account of the failure, so
    the missing-evidence default must not overwrite it."""
    for status in (1, 7, 124):
        completed = _run_guard(status, tmp_path / "mission-result.json")
        assert completed.returncode == status


def test_a_present_result_lets_the_run_continue(tmp_path: Path) -> None:
    result_file = tmp_path / "mission-result.json"
    result_file.write_text("{}", encoding="utf-8")
    assert _run_guard(0, result_file).returncode == 0


def test_the_false_zero_form_is_gone_and_the_script_still_parses() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    guard = _guard_block()
    # The exact defect: the only exit from the guard was the launch status,
    # defaulted for an unset variable that is always set.
    assert "exit \\${launch_status:-3}" not in guard
    assert "exit \\${evidence_status}" in guard
    subprocess.run(["bash", "-n", str(SCRIPT)], check=True)
    assert source.startswith("#!/usr/bin/env bash")
