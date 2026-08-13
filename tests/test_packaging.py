"""The shipped artifact, not the checkout.

Every other lifecycle test proves what ``flyto_robotics`` does when it is
imported from this working tree. None of them can see the two ways the *wheel*
breaks a customer who never had the tree:

* the ``flyto-robot`` console script is missing, duplicated, or points at the
  wrong callable, so the one command the runbook names does not exist; and
* ``data/lifecycle-profiles.json`` is left behind by ``find_packages``, so the
  installed package starts and then cannot render a single unit.

Both are invisible from a source checkout, because a checkout has the file on
disk and runs the module by path. So the assertion has to be made against a
built wheel -- a real one, unzipped and read -- not against the source tree that
would have satisfied it either way.
"""

from __future__ import annotations

import configparser
import hashlib
import importlib
import json
import os
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
SETUP_PY = REPO_ROOT / "setup.py"

CUSTOMER_CONSOLES = {
    "flyto-camera-gateway": "flyto_robotics.camera_gateway:main",
    "flyto-job-runner": "deploy:job_runner_main",
    "flyto-robot": "flyto_robotics.robot_cli:main",
    "flyto-device-events": "flyto_robotics.device_event_cli:main",
}
PROFILE_REGISTRY = "flyto_robotics/data/lifecycle-profiles.json"

#: The least that has to be copied for ``setup.py`` to build. Everything else it
#: names is matched by a glob, and an empty glob is legal; these three are named
#: as literal files and are not.
_BUILD_INPUTS = (
    "pyproject.toml",
    "setup.py",
    "setup.cfg",
    "package.xml",
    "resource",
    "deploy",
    "flyto_robotics",
)


def _console_script_lines() -> list[str]:
    """Every declared console script, read from the packaging source of truth."""

    text = SETUP_PY.read_text(encoding="utf-8")
    _, _, tail = text.partition('"console_scripts"')
    body, _, _ = tail.partition("]")
    return [
        line.strip().strip(",").strip('"').strip("'")
        for line in body.splitlines()
        if "=" in line and ":" in line
    ]


def test_the_customer_console_entries_are_declared_exactly_once() -> None:
    """One name, one target.

    A duplicated ``console_scripts`` entry is not a harmless repeat: the two
    lines can drift onto different callables, and which one wins is decided by
    the order a build backend happens to emit them in. The command in the
    runbook then works on the machine it was built on and not on the next one.
    """

    declared = _console_script_lines()
    for name, target in CUSTOMER_CONSOLES.items():
        entries = [line for line in declared if line.split("=", 1)[0].strip() == name]
        assert entries == [f"{name} = {target}"], declared
    # And no other script may be declared twice either.
    names = [line.split("=", 1)[0].strip() for line in declared]
    assert sorted(names) == sorted(set(names)), "a console script name is declared twice"


def test_job_runner_launcher_never_mutates_and_delegates_only_without_arguments(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Package imports cannot replace the runner that owns device behavior."""

    deploy = importlib.import_module("deploy")
    runner = importlib.import_module("deploy.flyto_job_runner")
    canonical_main = runner.main
    importlib.import_module("deploy.device_executor_contract")
    importlib.import_module("deploy.device_executor_registry")
    assert runner.main is canonical_main

    calls: list[None] = []

    def canonical_stub() -> int:
        calls.append(None)
        return 23

    monkeypatch.setattr(runner, "main", canonical_stub)
    monkeypatch.setattr(sys, "argv", ["flyto-job-runner", "--help"])
    with pytest.raises(SystemExit) as help_exit:
        deploy.job_runner_main()
    assert help_exit.value.code == 0
    assert calls == []

    monkeypatch.setattr(sys, "argv", ["flyto-job-runner"])
    assert deploy.job_runner_main() == 23
    assert calls == [None]

    pair_calls: list[None] = []
    monkeypatch.setattr(runner, "pair_main", lambda: pair_calls.append(None) or 29)
    monkeypatch.setattr(sys, "argv", ["flyto-job-runner", "pair"])
    assert deploy.job_runner_main() == 29
    assert pair_calls == [None]
    assert calls == [None]


def test_job_runner_pair_never_echoes_or_accepts_an_argv_secret(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    deploy = importlib.import_module("deploy")
    secret = "argv-secret-must-not-escape"
    monkeypatch.setattr(sys, "argv", ["flyto-job-runner", "pair", secret])
    assert deploy.job_runner_main() != 0
    output = capsys.readouterr()
    assert json.loads(output.out) == {
        "action_code": "use_pairing_code_environment",
        "ok": False,
        "reason": "pairing_argument_refused",
    }
    assert output.err == ""
    assert secret not in output.out + output.err


#: Build the wheel by calling the PEP 517 backend this project already declares,
#: in a subprocess running *this* interpreter.
#:
#: Not the ``build`` frontend: probing for it with ``find_spec`` answers for the
#: importing process, whose ``sys.path`` may have been polluted by whatever ran
#: the suite, while the wheel is built by ``sys.executable`` -- a different
#: environment that may not have it at all. That mismatch is how this test
#: "found" build, spawned an interpreter without it, and failed the pinned check
#: instead of skipping. The backend needs only setuptools, which
#: ``pyproject.toml`` already requires, so there is nothing to probe for and
#: nothing to skip: the proof runs on every machine that can run the suite.
_BUILD_WHEEL = """
import sys
from setuptools import build_meta

sys.stdout.write(build_meta.build_wheel(sys.argv[1]))
"""


def test_a_built_wheel_installs_offline_with_profiles_and_customer_commands(
    tmp_path: Path,
) -> None:
    """Build and offline-install a real wheel, then execute its customer CLIs.

    The build runs against a copy of the tree so that neither the wheel nor the
    ``build/``/``*.egg-info`` debris a backend leaves behind can land in a
    product path, and it is offline by construction: the point is to inspect
    this project's metadata, not to resolve a build environment.
    """

    source = tmp_path / "src"
    source.mkdir()
    for name in _BUILD_INPUTS:
        origin = REPO_ROOT / name
        if not origin.exists():
            pytest.skip(f"{name} is absent from this tree")
        if origin.is_dir():
            shutil.copytree(
                origin,
                source / name,
                ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.egg-info"),
            )
        else:
            shutil.copy2(origin, source / name)

    outdir = tmp_path / "dist"
    outdir.mkdir()
    python311 = shutil.which("python3.11")
    assert python311 is not None, "Python 3.11 is required for the clean-wheel proof"
    completed = subprocess.run(
        [python311, "-c", _BUILD_WHEEL, str(outdir)],
        cwd=str(source),
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr

    wheels = sorted(outdir.glob("*.whl"))
    assert len(wheels) == 1, [w.name for w in wheels]

    with zipfile.ZipFile(wheels[0]) as archive:
        names = archive.namelist()
        # The unit templates and the runbook are data. `find_packages` collects
        # modules only, so without explicit package data the wheel installs a
        # lifecycle that cannot render a single unit.
        assert PROFILE_REGISTRY in names, names
        registry = json.loads(archive.read(PROFILE_REGISTRY))
        profiles = registry["profiles"]
        generic = profiles["generic"]
        camera = profiles["camera"]
        ros2 = profiles["ros2"]
        assert generic.get("extends") is None
        assert "middleware" in generic["description"].lower()
        assert "vendor" in generic["description"].lower()
        assert all("ros" not in unit["name"].lower() for unit in generic["units"])
        assert camera["extends"] == "generic"
        assert [unit["name"] for unit in camera["units"]] == [
            "flyto-camera-gateway.service"
        ]
        assert ros2["extends"] == "generic"
        assert [unit["name"] for unit in ros2["units"]] == ["flyto-robot-ros2.service"]
        # Every module the shipped units and the lifecycle now execute. A wheel
        # that installs the registry and not the code that reads it, or the
        # readiness contract and not the snapshot it is resolved from, is a
        # device that starts and cannot answer a single question about itself.
        for module in (
            "flyto_robotics/robot_service.py",
            "flyto_robotics/readiness.py",
            "flyto_robotics/activation_snapshot.py",
            "flyto_robotics/lifecycle.py",
            "flyto_robotics/lifecycle_profiles.py",
            "flyto_robotics/robot_cli.py",
            "flyto_robotics/health_codes.py",
            "flyto_robotics/camera_observation.py",
            "flyto_robotics/camera_gateway.py",
            "flyto_robotics/camera_sources.py",
            "flyto_robotics/ros2_adapter.py",
        ):
            assert module in names, module
        for module in (
            "deploy/__init__.py",
            "deploy/flyto_job_runner.py",
            "deploy/device_executor_contract.py",
            "deploy/device_executor_registry.py",
        ):
            assert names.count(module) == 1, module

        entry_points = [n for n in names if n.endswith(".dist-info/entry_points.txt")]
        assert len(entry_points) == 1, entry_points
        parser = configparser.ConfigParser()
        parser.read_string(archive.read(entry_points[0]).decode("utf-8"))

    assert parser.has_section("console_scripts")
    scripts = dict(parser.items("console_scripts"))
    for name, target in CUSTOMER_CONSOLES.items():
        assert scripts.get(name) == target, scripts

    # Metadata alone is not an installation proof. Install exactly the wheel
    # just inspected into a fresh environment, without an index, dependencies,
    # or the checkout on PYTHONPATH, then execute the generated launchers. This
    # catches wheels whose entry points are correct on paper but cannot import
    # from an installed artifact.
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
        [str(pip), "install", "--no-index", "--no-deps", str(wheels[0])],
        cwd=str(tmp_path),
        capture_output=True,
        text=True,
        check=False,
        env={key: value for key, value in os.environ.items() if key != "PYTHONPATH"},
    )
    assert install.returncode == 0, install.stdout + install.stderr
    python = bindir / ("python.exe" if os.name == "nt" else "python")
    imported = subprocess.run(
        [
            str(python),
            "-c",
            (
                "import json, deploy; "
                "import deploy.flyto_job_runner as runner; "
                "canonical = runner.main; "
                "import deploy.device_executor_contract as contract; "
                "import deploy.device_executor_registry as registry; "
                "import flyto_robotics.ros2_adapter as ros2_adapter; "
                "print(json.dumps({'paths': [deploy.__file__, runner.__file__, "
                "contract.__file__, registry.__file__, ros2_adapter.__file__], "
                "'canonical_unchanged': runner.main is canonical, "
                "'launcher_is_distinct': deploy.job_runner_main is not runner.main}))"
            ),
        ],
        cwd=str(tmp_path),
        capture_output=True,
        text=True,
        check=False,
        env={key: value for key, value in os.environ.items() if key != "PYTHONPATH"},
    )
    assert imported.returncode == 0, imported.stdout + imported.stderr
    import_proof = json.loads(imported.stdout)
    assert import_proof["canonical_unchanged"] is True
    assert import_proof["launcher_is_distinct"] is True
    assert len(import_proof["paths"]) == 5
    for imported_path in import_proof["paths"]:
        assert Path(imported_path).resolve().is_relative_to(installed.resolve())
        assert not Path(imported_path).resolve().is_relative_to(REPO_ROOT.resolve())

    release = tmp_path / "release-payload"
    payload_result = subprocess.run(
        [
            str(python),
            "-c",
            (
                "import hashlib, json, pathlib; "
                "from flyto_robotics.robot_cli import build_package_payload; "
                "root=build_package_payload(pathlib.Path(__import__('sys').argv[1])); "
                "files=sorted(p for p in root.rglob('*') if p.is_file()); "
                "print(json.dumps({str(p.relative_to(root)): "
                "hashlib.sha256(p.read_bytes()).hexdigest() for p in files}))"
            ),
            str(release),
        ],
        cwd=str(tmp_path),
        capture_output=True,
        text=True,
        check=False,
        env={key: value for key, value in os.environ.items() if key != "PYTHONPATH"},
    )
    assert payload_result.returncode == 0, payload_result.stdout + payload_result.stderr
    payload_hashes = json.loads(payload_result.stdout)
    canonical_deploy = {
        "deploy/__init__.py",
        "deploy/flyto_job_runner.py",
        "deploy/device_executor_contract.py",
        "deploy/device_executor_registry.py",
    }
    assert {name for name in payload_hashes if name.startswith("deploy/")} == canonical_deploy
    assert any(name.startswith("flyto_robotics/") for name in payload_hashes)
    for name in canonical_deploy:
        installed_file = installed / "lib" / "python3.11" / "site-packages" / name
        assert payload_hashes[name] == hashlib.sha256(installed_file.read_bytes()).hexdigest()
        assert (release / name).resolve().is_relative_to(release.resolve())
    for name in CUSTOMER_CONSOLES:
        executable = bindir / (f"{name}.exe" if os.name == "nt" else name)
        help_result = subprocess.run(
            [str(executable), "--help"],
            cwd=str(tmp_path),
            capture_output=True,
            text=True,
            check=False,
            env={key: value for key, value in os.environ.items() if key != "PYTHONPATH"},
        )
        assert help_result.returncode == 0, help_result.stdout + help_result.stderr
        assert f"usage: {name}" in help_result.stdout.lower()
    camera = bindir / (
        "flyto-camera-gateway.exe" if os.name == "nt" else "flyto-camera-gateway"
    )
    checked = subprocess.run(
        [str(camera), "--check-settings"],
        cwd=str(tmp_path),
        capture_output=True,
        text=True,
        check=False,
        env={key: value for key, value in os.environ.items() if key != "PYTHONPATH"},
    )
    assert checked.returncode == 0, checked.stdout + checked.stderr
    assert json.loads(checked.stdout) == {
        "action_code": "none",
        "ok": True,
        "reason": "camera_settings_valid",
        "usable": False,
    }
    # No build output may have been dropped into the real tree.
    assert not list(REPO_ROOT.glob("*.whl"))
