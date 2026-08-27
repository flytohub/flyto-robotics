#!/usr/bin/env python3
"""Require tests to kill fixed safety and acceptance-oracle mutations."""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {
    ".flyto-index",
    ".git",
    ".pytest_cache",
    ".ruff_cache",
    ".venv",
    "__pycache__",
    "build",
    "install",
    "log",
    "output",
    "results",
    "tmp",
}

MUTATIONS = (
    {
        "id": "lifecycle_shutdown_skipped",
        "path": "flyto_robotics/ros2_closed_loop_lab.py",
        "old": "    _shutdown_lifecycle_managers(node, shutdown_clients)\n",
        "new": "    # mutation: lifecycle shutdown skipped\n",
        "probe": (
            "from pathlib import Path; "
            "s=Path('flyto_robotics/ros2_closed_loop_lab.py').read_text(); "
            "assert '    _shutdown_lifecycle_managers(node, shutdown_clients)\\n' in s"
        ),
    },
    {
        "id": "unexpected_death_is_accepted",
        "path": "flyto_robotics/ros2_acceptance.py",
        "old": "            unexpected.append(death)\n",
        "new": "            expected.append(death)\n",
        "probe": (
            "from flyto_robotics.ros2_acceptance import classify_process_deaths; "
            "r=classify_process_deaths('[ERROR] [controller-1]: process has died "
            "[pid 1, exit code 7, cmd x]'); "
            "assert not r['expected'] and r['unexpected']"
        ),
    },
    {
        "id": "shutdown_soak_volume_lowered",
        "path": "flyto_robotics/ros2_acceptance.py",
        "old": '        "expected_runs": 50,\n',
        "new": '        "expected_runs": 1,\n',
        "probe": (
            "from flyto_robotics.ros2_acceptance import ROS2_ACCEPTANCE_PROFILES; "
            "assert ROS2_ACCEPTANCE_PROFILES['shutdown-soak']['expected_runs'] == 50"
        ),
    },
    {
        "id": "run_verdict_forced_true",
        "path": "flyto_robotics/ros2_acceptance.py",
        "old": '    summary["passed"] = all(checks.values())\n',
        "new": '    summary["passed"] = True\n',
        "probe": (
            "from pathlib import Path; from flyto_robotics.ros2_acceptance import "
            "_build_run_summary,ROS2_ACCEPTANCE_PROFILES; "
            "r=_build_run_summary(root=Path('.'),"
            "output_dir=Path('results/ros2-resilience/missing'),"
            "profile=ROS2_ACCEPTANCE_PROFILES['shutdown-smoke'],index=1); "
            "assert not r['passed']"
        ),
    },
    {
        "id": "aggregate_verdict_forced_true",
        "path": "flyto_robotics/ros2_acceptance.py",
        "old": '        "passed": all(aggregate_checks.values()),\n',
        "new": '        "passed": True,\n',
        "probe": (
            "from pathlib import Path; from flyto_robotics.ros2_acceptance import "
            "build_ros2_acceptance_report; "
            "p=Path('results/ros2-resilience/mutation-empty'); p.mkdir(parents=True); "
            "r=build_ros2_acceptance_report(root=Path('.'),output_dir=p,"
            "profile_id='shutdown-smoke',control_id='mutation',"
            "container_image_id='sha256:'+'a'*64); assert not r['passed']"
        ),
    },
)


def _digest(value: object) -> str:
    encoded = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode(
        "utf-8"
    )
    return hashlib.sha256(encoded).hexdigest()


def _ignore(_directory: str, names: list[str]) -> set[str]:
    return {
        name for name in names if name in EXCLUDED or name == "core" or name.startswith("core.")
    }


def _apply_mutation(copy_root: Path, mutation: dict[str, object]) -> tuple[str, str]:
    path = copy_root / str(mutation["path"])
    source = path.read_text(encoding="utf-8")
    old = str(mutation["old"])
    if source.count(old) != 1:
        raise RuntimeError(
            f"mutation {mutation['id']} expected one exact source match, found {source.count(old)}"
        )
    changed = source.replace(old, str(mutation["new"]), 1)
    path.write_text(changed, encoding="utf-8")
    return hashlib.sha256(source.encode()).hexdigest(), hashlib.sha256(changed.encode()).hexdigest()


def main() -> int:
    results: list[dict[str, object]] = []
    with tempfile.TemporaryDirectory(prefix="flyto-acceptance-mutations-") as temporary:
        temporary_root = Path(temporary)
        for mutation in MUTATIONS:
            copy_root = temporary_root / str(mutation["id"])
            shutil.copytree(ROOT, copy_root, ignore=_ignore)
            source_sha256, mutated_sha256 = _apply_mutation(copy_root, mutation)
            completed = subprocess.run(
                [sys.executable, "-c", str(mutation["probe"])],
                cwd=copy_root,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=90,
                check=False,
            )
            if completed.returncode == 0:
                status = "survived"
            elif completed.returncode == 1:
                status = "killed"
            else:
                status = "infrastructure_error"
            receipt = {
                "mutation_id": mutation["id"],
                "source_path": mutation["path"],
                "source_sha256": source_sha256,
                "mutated_sha256": mutated_sha256,
                "probe_sha256": hashlib.sha256(str(mutation["probe"]).encode("utf-8")).hexdigest(),
                "status": status,
                "pytest_exit_code": completed.returncode,
                "task_completion_eligible": False,
            }
            receipt["receipt_sha256"] = _digest(receipt)
            results.append(receipt)
            print(f"[{mutation['id']}] {status}", flush=True)

    summary = {
        "contract_version": "flyto.robotics.acceptance-mutation-gate.v1",
        "mutation_count": len(results),
        "killed": sum(item["status"] == "killed" for item in results),
        "passed": all(item["status"] == "killed" for item in results),
        "results": results,
    }
    summary["snapshot"] = _digest(summary)
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
