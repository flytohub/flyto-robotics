"""Installed-artifact boundary for the external ROS 2 adapter architecture.

The current tree and built wheel must not recreate the retired TurtleBot3
appliance. Historical evidence belongs in Git history and handoffs, not in
executable current source.

The wheel ships only external/lab tools plus upstream-only ROS 2 service examples.
"""

from __future__ import annotations

import configparser
import os
import shutil
import subprocess
import zipfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SETUP_PY = REPO_ROOT / "setup.py"

EXTERNAL_CONSOLES = {
    "flyto2-adapter-provider-ros2-generic": "flyto_robotics.adapter_provider:main",
    "flyto-robotics": "flyto_robotics.cli:main",
    "flyto-device-events": "flyto_robotics.device_event_cli:main",
    "flyto-robot-mcp": "flyto_robotics.mcp_server:main",
    "flyto-camera-gateway": "flyto_robotics.camera_gateway:main",
    "flyto-resource-agent": "flyto_robotics.resource_agent:main",
    "flyto-ros2-readiness-probe": "flyto_robotics.ros2_probe_node:main",
    "parameter_bridge_guard": "flyto_robotics.bridge_guard:main",
    "ros2_closed_loop_lab": "flyto_robotics.ros2_closed_loop_lab:main",
    "ros2_safety_supervisor": "flyto_robotics.ros2_safety_node:main",
    "ros2_sensor_guard": "flyto_robotics.ros2_sensor_guard:main",
    "gazebo_lab_driver": "flyto_robotics.gazebo_lab_driver:main",
    "mission_controller": "flyto_robotics.ros2_node:main",
    "robotics_planning_session": "flyto_robotics.planning_session:main",
    "shortcut_controller": "flyto_robotics.shortcut_ros2_node:main",
    "shortcut_gazebo_driver": "flyto_robotics.shortcut_gazebo_driver:main",
    "showcase_gazebo_observer": "flyto_robotics.showcase_gazebo_observer:main",
}

RETIRED_APPLIANCE_CONSOLES = {
    "flyto-job-runner",
    "flyto-robot",
    "flyto-robot-doctor",
    "flyto-recovery-portal",
}

RETIRED_APPLIANCE_SOURCE_PATHS = {
    "deploy/flyto_job_runner.py",
    "flyto_robotics/delivery_gateway.py",
    "flyto_robotics/lifecycle.py",
    "flyto_robotics/recovery_portal.py",
    "flyto_robotics/robot_doctor.py",
    "scripts/install-robot-recovery.sh",
    "scripts/provision-device-credential.sh",
    "launch/turtlebot3_bringup_supervised.launch.py",
}

_BUILD_INPUTS = (
    "pyproject.toml",
    "setup.py",
    "setup.cfg",
    "package.xml",
    "resource",
    "deploy",
    "flyto_robotics",
    "launch",
    "config",
    "contracts",
    "examples",
    "scenarios",
    "worlds",
    "maps",
    "models",
)


def _console_script_lines() -> list[str]:
    text = SETUP_PY.read_text(encoding="utf-8")
    _, _, tail = text.partition('"console_scripts"')
    body, _, _ = tail.partition("]")
    return [
        line.strip().strip(",").strip('"').strip("'")
        for line in body.splitlines()
        if "=" in line and ":" in line
    ]


def test_current_tree_has_no_robot_appliance_source() -> None:
    for relative in RETIRED_APPLIANCE_SOURCE_PATHS:
        assert not (REPO_ROOT / relative).exists(), relative


def test_packaging_exposes_external_tools_and_no_robot_appliance_launchers() -> None:
    declared = _console_script_lines()
    scripts = {
        line.split("=", 1)[0].strip(): line.split("=", 1)[1].strip()
        for line in declared
    }

    assert scripts == EXTERNAL_CONSOLES
    assert RETIRED_APPLIANCE_CONSOLES.isdisjoint(scripts)
    assert len(scripts) == len(declared), "a console script name is declared twice"


_BUILD_WHEEL = """
import sys
from setuptools import build_meta
sys.stdout.write(build_meta.build_wheel(sys.argv[1]))
"""


def _build_wheel(tmp_path: Path) -> Path:
    source = tmp_path / "src"
    source.mkdir()
    for name in _BUILD_INPUTS:
        origin = REPO_ROOT / name
        if not origin.exists():
            continue
        if origin.is_dir():
            shutil.copytree(
                origin,
                source / name,
                ignore=shutil.ignore_patterns(
                    "__pycache__", "*.pyc", "*.egg-info", "build", "install", "log"
                ),
            )
        else:
            shutil.copy2(origin, source / name)

    outdir = tmp_path / "dist"
    outdir.mkdir()
    python311 = shutil.which("python3.11")
    assert python311 is not None, "Python 3.11 is required for the wheel proof"
    completed = subprocess.run(
        [python311, "-c", _BUILD_WHEEL, str(outdir)],
        cwd=str(source),
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    wheels = sorted(outdir.glob("*.whl"))
    assert len(wheels) == 1
    return wheels[0]


def test_built_wheel_contains_adapter_lab_code_but_not_pi_runtime(tmp_path: Path) -> None:
    wheel = _build_wheel(tmp_path)

    with zipfile.ZipFile(wheel) as archive:
        names = set(archive.namelist())

        # There is no installable robot-appliance package.
        assert "deploy/__init__.py" not in names
        for relative in RETIRED_APPLIANCE_SOURCE_PATHS:
            assert relative not in names

        # External ROS 2 / evidence tooling remains installable.
        for module in (
            "flyto_robotics/cli.py",
            "flyto_robotics/ros2_adapter.py",
            "flyto_robotics/ros2_action_executor.py",
            "flyto_robotics/ros2_execution.py",
            "flyto_robotics/ros2_execution_evidence.py",
            "flyto_robotics/ros2_observation_bundle.py",
            "flyto_robotics/adapter_contract.py",
            "flyto_robotics/adapter_provider.py",
            "flyto_robotics/generic_ros2_adapter.py",
            "flyto_robotics/ros2_closed_loop_lab.py",
            "flyto_robotics/camera_gateway.py",
            "flyto_robotics/resource_agent.py",
        ):
            assert module in names, module

        native_units = {
            name
            for name in names
            if "/native_ros2/" in name and name.endswith((".service", "README.md"))
        }
        assert any(
            name.endswith("/native_ros2/turtlebot3-bringup.service")
            for name in native_units
        )
        assert any(name.endswith("/native_ros2/camera-v4l2.service") for name in native_units)
        assert any(name.endswith("/native_ros2/nav2.service") for name in native_units)
        assert any(
            name.endswith("/native_ros2/slam-toolbox.service")
            for name in native_units
        )
        entry_points = [name for name in names if name.endswith(".dist-info/entry_points.txt")]
        assert len(entry_points) == 1
        parser = configparser.ConfigParser()
        parser.read_string(archive.read(entry_points[0]).decode("utf-8"))
        scripts = dict(parser.items("console_scripts"))
        external_adapters = dict(parser.items("flyto2.external_adapters"))
        resource_discoverers = dict(parser.items("flyto2.resource_discoverers"))

    assert scripts == EXTERNAL_CONSOLES
    assert external_adapters == {
        "ros2.generic": "flyto_robotics.adapter_provider:build_adapter"
    }
    assert resource_discoverers == {
        "ros2.generic": "flyto_robotics.adapter_provider:discover_resource_manifests"
    }
    assert RETIRED_APPLIANCE_CONSOLES.isdisjoint(scripts)


def test_installed_wheel_has_no_retired_cli_and_external_cli_imports(
    tmp_path: Path,
) -> None:
    wheel = _build_wheel(tmp_path)
    python311 = shutil.which("python3.11")
    assert python311 is not None

    installed = tmp_path / "installed"
    created = subprocess.run(
        [python311, "-m", "venv", str(installed)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert created.returncode == 0, created.stdout + created.stderr

    bindir = installed / ("Scripts" if os.name == "nt" else "bin")
    pip = bindir / ("pip.exe" if os.name == "nt" else "pip")
    install = subprocess.run(
        [str(pip), "install", "--no-index", "--no-deps", str(wheel)],
        capture_output=True,
        text=True,
        check=False,
        env={key: value for key, value in os.environ.items() if key != "PYTHONPATH"},
    )
    assert install.returncode == 0, install.stdout + install.stderr

    for name in RETIRED_APPLIANCE_CONSOLES:
        executable = bindir / (f"{name}.exe" if os.name == "nt" else name)
        assert not executable.exists(), f"retired robot-appliance CLI shipped: {name}"

    python = bindir / ("python.exe" if os.name == "nt" else "python")
    proof = subprocess.run(
        [
            str(python),
            "-c",
            (
                "import json, flyto_robotics.cli as c, "
                "flyto_robotics.ros2_adapter as a, "
                "flyto_robotics.ros2_execution as e, "
                "flyto_robotics.ros2_execution_evidence as v; "
                "print(json.dumps([c.__file__, a.__file__, e.__file__, v.__file__]))"
            ),
        ],
        cwd=str(tmp_path),
        capture_output=True,
        text=True,
        check=False,
        env={key: value for key, value in os.environ.items() if key != "PYTHONPATH"},
    )
    assert proof.returncode == 0, proof.stdout + proof.stderr
    paths = __import__("json").loads(proof.stdout)
    assert len(paths) == 4
    assert all(Path(path).resolve().is_relative_to(installed.resolve()) for path in paths)
