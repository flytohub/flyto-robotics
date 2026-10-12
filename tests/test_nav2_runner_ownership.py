"""The Nav2 runners hand container-written evidence back to the host user.

The scenario containers run as root and write evidence through
``write_json_atomic`` (mode 0600). Docker Desktop on macOS maps bind-mount
ownership to the host user, so this never showed locally; on a Linux host
(the hosted CI runner) the host-side verifier then cannot read the file
("Permission denied: .../success.json"). Every runner that reads evidence on
the host restores ownership after each container and before the first read.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _between(text: str, start: str, end: str) -> str:
    begin = text.index(start)
    return text[begin : text.index(end, begin)]


def _restore_function(text: str) -> str:
    match = re.search(r"restore_ownership\(\) \{\n(?P<body>.*?)\n\}", text, re.S)
    assert match is not None, "restore_ownership() is missing"
    return match.group("body")


def _assert_restores_as_host_user(body: str) -> None:
    assert "docker run --rm" in body
    assert '-v "${repository_root}:/workspace"' in body
    assert 'chown -R "$(id -u):$(id -g)"' in body


def test_closed_loop_runner_restores_ownership_before_reading_evidence() -> None:
    text = (ROOT / "scripts" / "run_nav2_closed_loop.sh").read_text(encoding="utf-8")
    _assert_restores_as_host_user(_restore_function(text))
    # An early exit on a failed scenario still leaves readable evidence for
    # the uploaded artifact.
    assert "trap 'restore_ownership \"${run_directory}\" || true' EXIT" in text

    loop = _between(text, "for scenario in success cancel emergency_stop; do", "\ndone\n")
    launch = loop.index("ros2 launch flyto_robotics nav2_closed_loop.launch.py")
    restore = loop.index('restore_ownership "${run_directory}"')
    first_read = loop.index('test -s "${evidence}"')
    assert launch < restore < first_read
    assert loop.index("verify-ros2-execution-evidence") > restore


def test_stress_runner_restores_ownership_before_reading_evidence() -> None:
    text = (ROOT / "scripts" / "run_nav2_stress.sh").read_text(encoding="utf-8")
    _assert_restores_as_host_user(_restore_function(text))

    scenario = _between(text, "run_scenario() {", "\n}\n")
    launch = scenario.index("ros2 launch flyto_robotics nav2_closed_loop.launch.py")
    status = scenario.index('scenario_status="${PIPESTATUS[0]}"')
    restore = scenario.index('restore_ownership "${round_directory}"')
    first_read = scenario.index('test -s "${pressure_evidence}"')
    assert launch < status < restore < first_read


def test_resilience_series_restores_ownership_before_reading_evidence() -> None:
    path = ROOT / "scripts" / "run_ros2_resilience_series.py"
    text = path.read_text(encoding="utf-8")
    tree = ast.parse(text, filename=str(path))
    functions = {
        node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)
    }
    helper = ast.unparse(functions["_restore_host_ownership"])
    assert "'chown', '-R'" in helper
    assert "os.getuid()" in helper and "os.getgid()" in helper

    nav_run = ast.unparse(functions["_nav_run"])
    remove = nav_run.index("'docker', 'rm', '-f', container_name")
    restore = nav_run.index("_restore_host_ownership(image, episode_dir)")
    first_read = nav_run.index("pressure_path.is_file()")
    assert remove < restore < first_read
