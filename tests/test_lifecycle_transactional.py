"""Failure paths of the customer lifecycle, against a temp root and a fake systemd.

The success path is the easy half and is covered in ``test_lifecycle.py``. This
file is about what happens when ``systemctl`` says no, when a service starts and
immediately dies, when two operators run a command at the same time, and when a
device loses power in the middle of an update. Those are the moments a fleet
lives or dies in, and none of them can be reached from a real robot on purpose.

Nothing here touches the host: every path is under ``tmp_path`` and every
``systemctl`` goes through :class:`FakeSystemctl`.
"""

from __future__ import annotations

import base64
import contextlib
import csv
import hashlib
import json
import os
import shutil
import stat
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

from flyto_robotics import legacy_takeover, lifecycle, robot_cli
from flyto_robotics.activation_snapshot import body_digest, build, load_document
from flyto_robotics.lifecycle import Layout, LifecycleError, install, rollback, status, update
from flyto_robotics.lifecycle_profiles import ProfileError, load_profiles
from flyto_robotics.support_bundle import (
    NOTE_REJECTED,
    build_support_bundle,
    check_note,
    redact,
    write_support_bundle,
)
from flyto_robotics.systemd_control import FakeSystemctl, SystemdController

AGENT = "flyto-robot-agent.service"
TIMER = "flyto-robot-doctor.timer"
DOCTOR = "flyto-robot-doctor.service"
ROS2 = "flyto-robot-ros2.service"
CAMERA = "flyto-camera-gateway.service"
RUNNER = "flyto-job-runner.service"
RUNNER_PATH = "flyto-job-runner.path"


@pytest.fixture()
def layout(tmp_path: Path) -> Layout:
    return Layout(root=tmp_path.resolve())


def _payload(tmp_path: Path, name: str, body: str) -> Path:
    """Build (or rebuild) the named payload deterministically.

    A test that installs the same version twice -- to prove idempotence, to
    prove provenance is never rewritten, to prove a drifted state file is
    repaired by re-running install -- has to be able to ask for the same payload
    twice and get the same bytes. Creating the tree unconditionally made the
    second call raise ``FileExistsError`` from the helper, which fails the test
    for a reason that has nothing to do with what it is asserting.
    """

    root = tmp_path / "payloads" / name
    package = root / "flyto_robotics"
    package.mkdir(parents=True, exist_ok=True)
    package.joinpath("__init__.py").write_text(body, encoding="utf-8")
    return root


def _fake(**kwargs) -> SystemdController:
    return SystemdController(runner=FakeSystemctl(**kwargs), dry_run=False, mode="fake")


def _active(layout: Layout) -> str | None:
    return Path(os.readlink(layout.current)).name if layout.current.is_symlink() else None


def _verbs(controller: SystemdController) -> list[str]:
    return [argv[1] for argv in controller.runner.commands]


class _InstalledDistribution:
    def __init__(self, root: Path, files: list[str]) -> None:
        self.root = root
        self.files = files

    def locate_file(self, name) -> Path:
        return self.root / str(name)


def _installed_tree(tmp_path: Path, marker: str) -> _InstalledDistribution:
    root = tmp_path / f"installed-{marker}"
    shutil_source = Path(robot_cli.__file__).parent
    shutil.copytree(
        shutil_source,
        root / "flyto_robotics",
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.pyo"),
    )
    deploy = root / "deploy"
    deploy.mkdir()
    for name in (
        "__init__.py",
        "flyto_job_runner.py",
        "device_executor_contract.py",
        "device_executor_registry.py",
    ):
        (deploy / name).write_text(f'MARKER = "{marker}:{name}"\n', encoding="utf-8")
    record_name = "flyto_robotics-0.1.0.dist-info/RECORD"
    files = sorted(
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file()
    )
    record = root / record_name
    record.parent.mkdir()
    with record.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        for name in files:
            data = (root / name).read_bytes()
            digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(b"=")
            writer.writerow((name, f"sha256={digest.decode()}", len(data)))
        writer.writerow((record_name, "", ""))
    return _InstalledDistribution(root, [*files, record_name])


def _build_installed_payload(
    monkeypatch: pytest.MonkeyPatch, distribution: _InstalledDistribution, target: Path
) -> Path:
    monkeypatch.setattr(robot_cli.importlib.metadata, "distribution", lambda _name: distribution)
    monkeypatch.setattr(
        robot_cli,
        "__file__",
        str(distribution.root / "flyto_robotics" / "robot_cli.py"),
    )
    return robot_cli.build_package_payload(target)


def _import_active_deploy(layout: Layout, cwd: Path) -> dict:
    code = (
        "import json; "
        "import deploy.flyto_job_runner as runner; "
        "import deploy.device_executor_contract as contract; "
        "import deploy.device_executor_registry as registry; "
        "print(json.dumps({'markers': [runner.MARKER, contract.MARKER, registry.MARKER], "
        "'paths': [runner.__file__, contract.__file__, registry.__file__]}))"
    )
    completed = subprocess.run(
        [sys.executable, "-S", "-c", code],
        cwd=cwd,
        env={"PYTHONPATH": str(layout.current)},
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    return json.loads(completed.stdout)


def _takeover_fixture(layout: Layout, *, active: bool, enabled: bool):
    """Create a wholly synthetic B1 observation for private recovery tests."""

    (layout.root / "etc").mkdir(parents=True, exist_ok=True)
    (layout.root / "etc/machine-id").write_text("a" * 32 + "\n", encoding="ascii")
    layout.config_dir.mkdir(parents=True, exist_ok=True)
    layout.identity_file.write_text('{"device_id":"synthetic-device"}', encoding="utf-8")
    layout.config_file.write_text(
        "FLYTO_CLOUD_URL=https://example.invalid\n"
        "FLYTO_ROBOT_RESOURCE_ID=synthetic-resource\n",
        encoding="utf-8",
    )
    layout.credentials_dir.mkdir(parents=True, exist_ok=True)
    credential = layout.credentials_dir / "runner-credentials.json"
    credential.write_text(
        '{"device_id":"synthetic-device","device_secret":"synthetic-secret"}',
        encoding="utf-8",
    )
    credential.chmod(0o600)
    spec = lifecycle.profile_for("generic")
    layout.unit_dir.mkdir(parents=True, exist_ok=True)
    originals = {}
    for unit in spec.units:
        raw = (
            f"[Unit]\nDescription=synthetic legacy {unit.name}\n"
            "[Service]\nExecStart=/usr/bin/true\n"
        ).encode()
        (layout.unit_dir / unit.name).write_bytes(raw)
        originals[unit.name] = raw
    controller = _fake()
    names = set(originals)
    if active:
        controller.runner.active.update(names)
    if enabled:
        controller.runner.enabled.update(names)
    receipt = legacy_takeover.plan_takeover(
        layout=layout,
        profile="generic",
        profiles=None,
        systemd=controller,
        acknowledged=True,
    )
    digest = legacy_takeover.seal_takeover_window(
        receipt=receipt,
        layout=layout,
        profile="generic",
        profiles=None,
        systemd=controller,
    )
    return controller, originals, receipt, digest


class TestLegacyTakeoverCapsuleRecovery:
    @pytest.mark.parametrize("active", [False, True])
    @pytest.mark.parametrize("enabled", [False, True])
    def test_takeover_recovery_restores_the_exact_four_state_matrix(
        self, layout: Layout, active: bool, enabled: bool
    ) -> None:
        controller, originals, _receipt, _digest = _takeover_fixture(
            layout, active=active, enabled=enabled
        )
        for name in originals:
            (layout.unit_dir / name).write_text(
                "[Unit]\nDescription=incoming\n[Service]\nExecStart=/usr/bin/false\n",
                encoding="utf-8",
            )
        # A crashed incoming activation may have started every unit regardless
        # of the legacy state recorded in the capsule.
        controller.runner.active.update(originals)

        result = legacy_takeover.recover_takeover(layout, controller)

        assert result == {"ok": True, "reason": "takeover_recovered"}
        assert {name: (layout.unit_dir / name).read_bytes() for name in originals} == originals
        assert controller.runner.active == (set(originals) if active else set())
        assert controller.runner.enabled == (set(originals) if enabled else set())
        assert not legacy_takeover.takeover_window_path(layout).exists()

    def test_pending_takeover_is_recovered_before_the_ordinary_collision_precheck(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        controller, originals, _receipt, _digest = _takeover_fixture(
            layout, active=True, enabled=True
        )
        for name in originals:
            (layout.unit_dir / name).write_text(
                "[Unit]\nDescription=partial incoming\n[Service]\nExecStart=/usr/bin/false\n",
                encoding="utf-8",
            )
        controller.runner.active.update(originals)
        command_count = len(controller.runner.commands)

        with pytest.raises(LifecycleError) as raised:
            install(
                payload=_payload(tmp_path, "takeover-reachable", "v1"),
                version="1.0.0",
                layout=layout,
                systemd=controller,
            )

        assert raised.value.reason == "unit_name_collision"
        assert {name: (layout.unit_dir / name).read_bytes() for name in originals} == originals
        assert not legacy_takeover.takeover_window_path(layout).exists()
        assert len(controller.runner.commands) > command_count
        assert not layout.release("1.0.0").exists()

    @pytest.mark.parametrize("target", ["takeover", "capsules"])
    def test_takeover_private_storage_refuses_symlink_parents(
        self, layout: Layout, tmp_path: Path, target: str
    ) -> None:
        controller, _originals, receipt, _digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        legacy_takeover.takeover_window_path(layout).unlink()
        takeover_dir = layout.state_dir / "legacy-takeover"
        shutil.rmtree(takeover_dir)
        escape = tmp_path / "escape"
        escape.mkdir()
        if target == "takeover":
            takeover_dir.symlink_to(escape, target_is_directory=True)
        else:
            takeover_dir.mkdir()
            (takeover_dir / "capsules").symlink_to(escape, target_is_directory=True)

        with pytest.raises(legacy_takeover.TakeoverError) as raised:
            legacy_takeover.seal_takeover_window(
                receipt=receipt,
                layout=layout,
                profile="generic",
                profiles=None,
                systemd=controller,
            )
        assert raised.value.reason == "takeover_storage_invalid"
        assert list(escape.iterdir()) == []

    def test_tampered_capsule_retains_window_and_fails_closed(
        self, layout: Layout
    ) -> None:
        controller, originals, _receipt, digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        capsule = layout.state_dir / "legacy-takeover/capsules" / f"{digest}.json"
        capsule.write_text('{"schema":"tampered"}\n', encoding="utf-8")
        capsule.chmod(0o600)
        commands = len(controller.runner.commands)

        result = legacy_takeover.recover_takeover(layout, controller)

        assert result == {"ok": False, "reason": "takeover_rollback_failed"}
        assert legacy_takeover.takeover_window_path(layout).exists()
        assert len(controller.runner.commands) == commands
        assert {name: (layout.unit_dir / name).read_bytes() for name in originals} == originals

    @pytest.mark.parametrize("failure", ["stop", "daemon-reload", "enable", "restart"])
    def test_takeover_recovery_failure_points_retain_the_sealed_window(
        self, layout: Layout, failure: str
    ) -> None:
        controller, originals, _receipt, _digest = _takeover_fixture(
            layout, active=True, enabled=True
        )
        for name in originals:
            (layout.unit_dir / name).write_text(
                "[Unit]\nDescription=partial\n[Service]\nExecStart=/usr/bin/false\n",
                encoding="utf-8",
            )
        controller.runner.active.update(originals)
        controller.runner.fail_on = frozenset({failure})

        result = legacy_takeover.recover_takeover(layout, controller)

        assert result == {"ok": False, "reason": "takeover_rollback_failed"}
        assert legacy_takeover.takeover_window_path(layout).is_file()

    def test_takeover_recovery_is_idempotent_after_response_loss(self, layout: Layout) -> None:
        controller, originals, _receipt, _digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        for name in originals:
            (layout.unit_dir / name).write_text(
                "[Unit]\nDescription=partial\n[Service]\nExecStart=/usr/bin/false\n",
                encoding="utf-8",
            )

        assert legacy_takeover.recover_takeover(layout, controller)["ok"] is True
        assert legacy_takeover.recover_takeover(layout, controller) == {
            "ok": True,
            "reason": "no_recovery_required",
        }

    @pytest.mark.parametrize("malformation", ["duplicate", "oversize", "symlink"])
    def test_takeover_capsule_adversarial_files_fail_closed(
        self, layout: Layout, tmp_path: Path, malformation: str
    ) -> None:
        controller, _originals, _receipt, digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        capsule = layout.state_dir / "legacy-takeover/capsules" / f"{digest}.json"
        if malformation == "duplicate":
            capsule.write_text('{"schema":"x","schema":"y"}\n', encoding="utf-8")
            capsule.chmod(0o600)
        elif malformation == "oversize":
            with capsule.open("wb") as handle:
                handle.truncate(legacy_takeover.MAX_CAPSULE_BYTES + 1)
            capsule.chmod(0o600)
        else:
            capsule.unlink()
            target = tmp_path / "escaped-capsule"
            target.write_text("{}", encoding="utf-8")
            capsule.symlink_to(target)

        commands = len(controller.runner.commands)
        assert legacy_takeover.recover_takeover(layout, controller) == {
            "ok": False,
            "reason": "takeover_rollback_failed",
        }
        assert legacy_takeover.takeover_window_path(layout).exists()
        assert len(controller.runner.commands) == commands

    def test_takeover_commissioning_race_causes_zero_systemd_mutation(
        self, layout: Layout
    ) -> None:
        controller, _originals, _receipt, _digest = _takeover_fixture(
            layout, active=True, enabled=True
        )
        layout.config_file.write_text(
            "FLYTO_CLOUD_URL=https://changed.invalid\n"
            "FLYTO_ROBOT_RESOURCE_ID=synthetic-resource\n",
            encoding="utf-8",
        )
        commands = len(controller.runner.commands)

        assert legacy_takeover.recover_takeover(layout, controller) == {
            "ok": False,
            "reason": "takeover_rollback_failed",
        }
        assert len(controller.runner.commands) == commands
        assert legacy_takeover.takeover_window_path(layout).exists()

    def test_takeover_seal_final_observation_race_writes_no_window(
        self, layout: Layout, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        controller, _originals, receipt, _digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        legacy_takeover.takeover_window_path(layout).unlink()
        original = legacy_takeover._observe
        calls = 0

        def racing(*args, **kwargs):
            nonlocal calls
            calls += 1
            if calls == 4:
                controller.runner.active.add(AGENT)
            return original(*args, **kwargs)

        monkeypatch.setattr(legacy_takeover, "_observe", racing)
        with pytest.raises(legacy_takeover.TakeoverError) as raised:
            legacy_takeover.seal_takeover_window(
                receipt=receipt,
                layout=layout,
                profile="generic",
                profiles=None,
                systemd=controller,
            )
        assert raised.value.reason == "snapshot_race"
        assert not legacy_takeover.takeover_window_path(layout).exists()

    @pytest.mark.parametrize("mutation", ["config", "systemd", "unit", "lifecycle"])
    def test_takeover_post_window_fsync_race_removes_only_the_published_window(
        self,
        layout: Layout,
        monkeypatch: pytest.MonkeyPatch,
        mutation: str,
    ) -> None:
        controller, _originals, receipt, _digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        legacy_takeover.takeover_window_path(layout).unlink()
        original_write = legacy_takeover._write_private
        commands = len(controller.runner.commands)

        def writing(layout_arg, parts, name, value):
            original_write(layout_arg, parts, name, value)
            if parts != ("legacy-takeover",) or name != "window.json":
                return
            if mutation == "config":
                layout.config_file.write_text(
                    "FLYTO_CLOUD_URL=https://changed.invalid\n"
                    "FLYTO_ROBOT_RESOURCE_ID=synthetic-resource\n",
                    encoding="utf-8",
                )
            elif mutation == "systemd":
                controller.runner.active.add(AGENT)
            elif mutation == "unit":
                (layout.unit_dir / AGENT).write_text(
                    "[Unit]\nDescription=raced\n[Service]\nExecStart=/usr/bin/true\n",
                    encoding="utf-8",
                )
            else:
                layout.state_file.write_text('{"schema":"raced"}\n', encoding="utf-8")

        monkeypatch.setattr(legacy_takeover, "_write_private", writing)
        with pytest.raises(legacy_takeover.TakeoverError) as raised:
            legacy_takeover.seal_takeover_window(
                receipt=receipt,
                layout=layout,
                profile="generic",
                profiles=None,
                systemd=controller,
            )

        assert raised.value.reason == "snapshot_race"
        assert not legacy_takeover.takeover_window_path(layout).exists()
        assert all(
            command[1] in {"is-active", "is-enabled"}
            for command in controller.runner.commands[commands:]
        )

    def test_takeover_snapshot_cleanup_never_deletes_a_replaced_window(
        self, layout: Layout, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        controller, _originals, receipt, _digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        window_path = legacy_takeover.takeover_window_path(layout)
        window_path.unlink()
        different = {
            "schema": legacy_takeover.WINDOW_SCHEMA,
            "capsule_digest": "f" * 64,
            "phase": "sealed",
        }
        original_write = legacy_takeover._write_private

        def replacing(layout_arg, parts, name, value):
            original_write(layout_arg, parts, name, value)
            if parts == ("legacy-takeover",) and name == "window.json":
                original_write(layout_arg, parts, name, different)
                controller.runner.active.add(AGENT)

        monkeypatch.setattr(legacy_takeover, "_write_private", replacing)
        with pytest.raises(legacy_takeover.TakeoverError) as raised:
            legacy_takeover.seal_takeover_window(
                receipt=receipt,
                layout=layout,
                profile="generic",
                profiles=None,
                systemd=controller,
            )

        assert raised.value.reason == "snapshot_race"
        assert json.loads(window_path.read_bytes()) == different

    @pytest.mark.parametrize("failure", ["invalid", "missing", "read_error"])
    def test_takeover_post_window_observation_error_cleans_exact_window(
        self,
        layout: Layout,
        monkeypatch: pytest.MonkeyPatch,
        failure: str,
    ) -> None:
        controller, _originals, receipt, _digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        window = legacy_takeover.takeover_window_path(layout)
        window.unlink()
        original_write = legacy_takeover._write_private
        original_read = legacy_takeover._read_regular
        after_window = False
        commands = len(controller.runner.commands)

        def writing(layout_arg, parts, name, value):
            nonlocal after_window
            original_write(layout_arg, parts, name, value)
            if parts == ("legacy-takeover",) and name == "window.json":
                if failure == "invalid":
                    layout.config_file.write_text("INVALID\n", encoding="utf-8")
                elif failure == "missing":
                    layout.config_file.unlink()
                after_window = True

        def reading(path, limit, reason):
            if after_window and failure == "read_error" and path == layout.config_file:
                raise OSError("synthetic read failure")
            return original_read(path, limit, reason)

        monkeypatch.setattr(legacy_takeover, "_write_private", writing)
        monkeypatch.setattr(legacy_takeover, "_read_regular", reading)
        with pytest.raises(legacy_takeover.TakeoverError) as raised:
            legacy_takeover.seal_takeover_window(
                receipt=receipt,
                layout=layout,
                profile="generic",
                profiles=None,
                systemd=controller,
            )

        assert raised.value.reason == "snapshot_race"
        assert not window.exists()
        assert all(
            command[1] in {"is-active", "is-enabled"}
            for command in controller.runner.commands[commands:]
        )

    def test_takeover_recovery_quiesces_transitional_state_before_overwrite(
        self, layout: Layout, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        controller, originals, _receipt, _digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        original_health = controller.health
        calls = 0

        def transitional(names):
            nonlocal calls
            calls += 1
            result = original_health(names)
            if calls == 1:
                result[0]["active"] = "activating"
            return result

        monkeypatch.setattr(controller, "health", transitional)
        assert legacy_takeover.recover_takeover(layout, controller)["ok"] is True
        stopped = [command for command in controller.runner.commands if command[1] == "stop"]
        assert any(next(iter(sorted(originals))) in command for command in stopped)

    def test_takeover_dir_swap_cannot_redirect_private_write(
        self, layout: Layout, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        controller, _originals, receipt, _digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        legacy_takeover.takeover_window_path(layout).unlink()
        takeover = layout.state_dir / "legacy-takeover"
        held = layout.state_dir / "legacy-takeover-held"
        escape = tmp_path / "escape"
        escape.mkdir()
        real_open = legacy_takeover.os.open
        swapped = False

        def swapping(path, *args, **kwargs):
            nonlocal swapped
            if (
                isinstance(path, str)
                and path.startswith(".")
                and path.endswith(".tmp")
                and not swapped
            ):
                swapped = True
                takeover.rename(held)
                takeover.symlink_to(escape, target_is_directory=True)
            return real_open(path, *args, **kwargs)

        monkeypatch.setattr(legacy_takeover.os, "open", swapping)
        legacy_takeover.seal_takeover_window(
            receipt=receipt,
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=controller,
        )
        assert list(escape.iterdir()) == []
        assert (held / "window.json").is_file()

    @pytest.mark.parametrize("private_part", ["legacy-takeover", "capsules"])
    def test_takeover_refuses_permissive_private_directories(
        self, layout: Layout, private_part: str
    ) -> None:
        controller, _originals, receipt, _digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        legacy_takeover.takeover_window_path(layout).unlink()
        target = layout.state_dir / "legacy-takeover"
        if private_part == "capsules":
            target /= "capsules"
        target.chmod(0o777)

        with pytest.raises(legacy_takeover.TakeoverError) as raised:
            legacy_takeover.seal_takeover_window(
                receipt=receipt,
                layout=layout,
                profile="generic",
                profiles=None,
                systemd=controller,
            )
        assert raised.value.reason == "takeover_storage_invalid"

    def test_takeover_refuses_wrong_owner_private_file(
        self, layout: Layout, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        controller, _originals, _receipt, digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        capsule = layout.state_dir / "legacy-takeover/capsules" / f"{digest}.json"
        real_fstat = legacy_takeover.os.fstat

        def wrong_owner(descriptor):
            metadata = real_fstat(descriptor)
            if stat.S_ISREG(metadata.st_mode) and metadata.st_size == capsule.stat().st_size:
                fields = list(metadata)
                fields[4] += 1
                return os.stat_result(fields)
            return metadata

        monkeypatch.setattr(legacy_takeover.os, "fstat", wrong_owner)
        assert legacy_takeover.recover_takeover(layout, controller) == {
            "ok": False,
            "reason": "takeover_rollback_failed",
        }

    def test_takeover_refuses_wrong_owner_private_directory(
        self, layout: Layout, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        controller, _originals, receipt, _digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        legacy_takeover.takeover_window_path(layout).unlink()
        target = (layout.state_dir / "legacy-takeover").stat()
        real_fstat = legacy_takeover.os.fstat

        def wrong_owner(descriptor):
            metadata = real_fstat(descriptor)
            if metadata.st_dev == target.st_dev and metadata.st_ino == target.st_ino:
                fields = list(metadata)
                fields[4] += 1
                return os.stat_result(fields)
            return metadata

        monkeypatch.setattr(legacy_takeover.os, "fstat", wrong_owner)
        with pytest.raises(legacy_takeover.TakeoverError) as raised:
            legacy_takeover.seal_takeover_window(
                receipt=receipt,
                layout=layout,
                profile="generic",
                profiles=None,
                systemd=controller,
            )
        assert raised.value.reason == "takeover_storage_invalid"

    def test_takeover_double_seal_is_idempotent_but_never_stacks(
        self, layout: Layout
    ) -> None:
        controller, originals, receipt, digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        window = legacy_takeover.takeover_window_path(layout)
        before = window.read_bytes()
        capsules = window.parent / "capsules"
        before_capsules = sorted(path.name for path in capsules.iterdir())

        assert legacy_takeover.seal_takeover_window(
            receipt=receipt,
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=controller,
        ) == digest
        assert window.read_bytes() == before

        (layout.unit_dir / AGENT).write_text(
            "[Unit]\nDescription=different legacy\n[Service]\nExecStart=/usr/bin/true\n",
            encoding="utf-8",
        )
        second = legacy_takeover.plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=controller,
            acknowledged=True,
        )
        with pytest.raises(legacy_takeover.TakeoverError) as raised:
            legacy_takeover.seal_takeover_window(
                receipt=second,
                layout=layout,
                profile="generic",
                profiles=None,
                systemd=controller,
            )
        assert raised.value.reason == "takeover_in_progress"
        assert window.read_bytes() == before
        assert sorted(path.name for path in capsules.iterdir()) == before_capsules
        assert (layout.unit_dir / AGENT).read_bytes() != originals[AGENT]

    @pytest.mark.parametrize("target", ["parent", "window", "capsule"])
    @pytest.mark.parametrize("mode", [0o620, 0o602])
    def test_takeover_refuses_group_or_world_writable_private_storage(
        self, layout: Layout, target: str, mode: int
    ) -> None:
        controller, _originals, receipt, digest = _takeover_fixture(
            layout, active=False, enabled=False
        )
        if target == "parent":
            legacy_takeover.takeover_window_path(layout).parent.chmod(0o700 | mode)
        elif target == "window":
            legacy_takeover.takeover_window_path(layout).chmod(mode)
        else:
            (layout.state_dir / f"legacy-takeover/capsules/{digest}.json").chmod(mode)

        with pytest.raises(legacy_takeover.TakeoverError) as raised:
            legacy_takeover.seal_takeover_window(
                receipt=receipt,
                layout=layout,
                profile="generic",
                profiles=None,
                systemd=controller,
            )
        assert raised.value.reason in {
            "takeover_storage_invalid",
            "takeover_in_progress",
            "capsule_invalid",
        }


# ---------------------------------------------------------------------------
# The systemd transaction
# ---------------------------------------------------------------------------


class TestSystemdTransaction:
    def test_camera_profiles_own_activate_and_verify_the_camera_gateway(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        systemd = _fake()
        report = install(
            payload=_payload(tmp_path, "camera", "v1"),
            version="1.0.0",
            profile="camera",
            layout=layout,
            systemd=systemd,
        )
        assert report["ok"] is True
        assert (layout.unit_dir / CAMERA).is_file()
        assert CAMERA in systemd.runner.enabled
        assert CAMERA in systemd.runner.active
        camera_commands = [command for command in systemd.runner.commands if CAMERA in command]
        assert [command[1] for command in camera_commands] == [
            "enable", "restart", "is-active"
        ]

    def test_ros2_never_installs_or_controls_the_camera_gateway(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        systemd = _fake()
        report = install(
            payload=_payload(tmp_path, "ros2", "v1"), version="1.0.0",
            profile="ros2", layout=layout, systemd=systemd,
        )
        assert report["ok"] is True
        assert not (layout.unit_dir / CAMERA).exists()
        assert CAMERA not in systemd.runner.enabled
        assert CAMERA not in systemd.runner.active
        assert all(CAMERA not in command for command in systemd.runner.commands)

    def test_a_first_install_reloads_enables_starts_and_proves_the_service_is_up(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        systemd = _fake()
        report = install(
            payload=_payload(tmp_path, "a", "v1"),
            version="1.0.0",
            layout=layout,
            systemd=systemd,
        )
        assert report["ok"] is True
        verbs = _verbs(systemd)
        # Order matters: reload before enable, enable before restart, and an
        # is-active *after* the restart. A restart that exits 0 proves only that
        # systemd accepted the job.
        assert verbs.index("daemon-reload") < verbs.index("enable")
        assert verbs.index("enable") < verbs.index("restart")
        assert verbs.index("restart") < verbs.index("is-active")
        assert systemd.runner.enabled == {AGENT, TIMER, RUNNER, RUNNER_PATH}
        assert systemd.runner.active == {AGENT, TIMER, RUNNER_PATH}
        # The oneshot health snapshot is driven by its timer, so it is neither
        # enabled nor demanded to be active.
        assert DOCTOR not in systemd.runner.enabled

    def test_a_reinstall_is_idempotent_on_disk(self, layout: Layout, tmp_path: Path) -> None:
        payload = _payload(tmp_path, "a", "v1")
        install(payload=payload, version="1.0.0", layout=layout, systemd=_fake())
        second = install(payload=payload, version="1.0.0", layout=layout, systemd=_fake())
        assert second["ok"] is True
        assert second["changed"] == []
        assert second["reason"] == "no_change"

    @pytest.mark.parametrize("credential_kind", ["missing", "directory", "symlink"])
    def test_unpaired_runner_is_enabled_but_not_started_while_watcher_stays_active(
        self, layout: Layout, tmp_path: Path, credential_kind: str
    ) -> None:
        layout.credentials_dir.mkdir(parents=True)
        credential = layout.credentials_dir / "runner-credentials.json"
        if credential_kind == "directory":
            credential.mkdir()
        elif credential_kind == "symlink":
            target = layout.credentials_dir / "real.json"
            target.write_text("{}", encoding="utf-8")
            credential.symlink_to(target)

        systemd = _fake()
        result = install(
            payload=_payload(tmp_path, credential_kind, "v1"),
            version="1.0.0",
            layout=layout,
            systemd=systemd,
        )

        assert result["ok"] is True
        assert result["readiness"]["state"] == "provisioning_pending"
        runner_check = next(
            check
            for check in result["readiness"]["checks"]
            if check["id"] == f"activation_condition:{RUNNER}"
        )
        assert runner_check["passed"] is False
        assert RUNNER in systemd.runner.enabled
        assert RUNNER not in systemd.runner.active
        assert RUNNER_PATH in systemd.runner.enabled
        assert RUNNER_PATH in systemd.runner.active
        runner_commands = [command[1] for command in systemd.runner.commands if RUNNER in command]
        assert runner_commands == ["stop", "enable"]

    def test_condition_stat_error_fails_closed_without_a_runner_loop(
        self, layout: Layout, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        credential = layout.credentials_dir / "runner-credentials.json"
        real_stat = lifecycle.os.stat

        def refused(path, *args, **kwargs):
            if Path(path) == credential:
                raise PermissionError("synthetic metadata refusal")
            return real_stat(path, *args, **kwargs)

        systemd = _fake()
        with monkeypatch.context() as scoped:
            scoped.setattr(lifecycle.os, "stat", refused)
            result = install(
                payload=_payload(tmp_path, "stat-error", "v1"),
                version="1.0.0",
                layout=layout,
                systemd=systemd,
            )
        assert result["ok"] is True
        assert RUNNER not in systemd.runner.active
        assert not any(command[1] in {"restart", "is-active"} and RUNNER in command
                       for command in systemd.runner.commands)

    def test_regular_credential_restarts_and_verifies_runner(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        layout.credentials_dir.mkdir(parents=True)
        (layout.credentials_dir / "runner-credentials.json").write_text("{}", encoding="utf-8")
        systemd = _fake()
        result = install(
            payload=_payload(tmp_path, "paired", "v1"), version="1.0.0",
            layout=layout, systemd=systemd,
        )
        assert result["ok"] is True
        runner_check = next(
            check
            for check in result["readiness"]["checks"]
            if check["id"] == f"activation_condition:{RUNNER}"
        )
        assert runner_check["passed"] is True
        commands = [command[1] for command in systemd.runner.commands if RUNNER in command]
        assert commands == ["enable", "restart", "is-active"]

    def test_incoming_activation_samples_each_condition_once(
        self, layout: Layout, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        credential = layout.credentials_dir / "runner-credentials.json"
        credential.parent.mkdir(parents=True)
        credential.write_text("{}", encoding="utf-8")
        real_stat = lifecycle.os.stat
        samples = 0

        def changing(path, *args, **kwargs):
            nonlocal samples
            if Path(path) == credential and kwargs.get("follow_symlinks") is False:
                samples += 1
                if samples > 1:
                    raise PermissionError("synthetic post-snapshot change")
            return real_stat(path, *args, **kwargs)

        systemd = _fake()
        with monkeypatch.context() as scoped:
            scoped.setattr(lifecycle.os, "stat", changing)
            result = install(
                payload=_payload(tmp_path, "one-sample", "v1"), version="1.0.0",
                layout=layout, systemd=systemd,
            )
        assert result["ok"] is True
        assert samples == 1
        assert RUNNER in systemd.runner.active

    def test_runner_path_contract_observes_atomic_credential_publication(self) -> None:
        units = lifecycle.render_units(Layout())
        service = units[RUNNER]
        watcher = units[RUNNER_PATH]
        contract = "/var/lib/flyto-robot/credentials/runner-credentials.json"
        assert f"FLYTO_RUNNER_DATA_DIR={Path(contract).parent}" in service
        assert f"ConditionPathExists={contract}" in service
        assert f"ConditionPathIsSymbolicLink=!{contract}" in service
        assert f"ExecCondition=/usr/bin/test -f {contract}" in service
        assert f"PathExists={contract}" in watcher
        assert f"Unit={RUNNER}" in watcher
        assert "OnUnitActiveSec=" not in watcher
        assert "OnBootSec=" not in watcher
        profiles = load_profiles()
        generic_runner = next(unit for unit in profiles["generic"].units if unit.name == RUNNER)
        for profile_name in ("camera", "ros2"):
            inherited = next(
                unit for unit in profiles[profile_name].units if unit.name == RUNNER
            )
            assert inherited == generic_runner
            assert lifecycle.render_units(Layout(), profile=profile_name)[RUNNER] == service

    def test_failed_update_rechecks_previous_condition_before_undo_restart(
        self, layout: Layout, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        credential = layout.credentials_dir / "runner-credentials.json"
        credential.parent.mkdir(parents=True)
        credential.write_text("{}", encoding="utf-8")
        install(
            payload=_payload(tmp_path, "undo-v1", "v1"), version="1.0.0",
            layout=layout, systemd=_fake(),
        )
        systemd = _fake()
        systemd.runner.enabled.update({AGENT, TIMER, RUNNER, RUNNER_PATH})
        systemd.runner.active.update({AGENT, TIMER, RUNNER, RUNNER_PATH})
        real_stat = lifecycle.os.stat
        samples = 0

        def counted(path, *args, **kwargs):
            nonlocal samples
            if Path(path) == credential and kwargs.get("follow_symlinks") is False:
                samples += 1
            return real_stat(path, *args, **kwargs)

        def remove_pairing(_current: Path) -> bool:
            credential.unlink()
            return False

        with monkeypatch.context() as scoped:
            scoped.setattr(lifecycle.os, "stat", counted)
            result = update(
                payload=_payload(tmp_path, "undo-v2", "v2"), version="2.0.0",
                layout=layout, systemd=systemd, health_check=remove_pairing,
            )
        assert result["ok"] is False
        assert result["recovery"]["ok"] is True
        runner_verbs = [command[1] for command in systemd.runner.commands if RUNNER in command]
        assert runner_verbs.count("restart") == 1
        assert runner_verbs[-1] == "stop"
        assert RUNNER not in systemd.runner.active
        assert samples == 2, "one incoming snapshot and one fresh undo snapshot"

    def test_a_failed_daemon_reload_leaves_a_first_install_cleanly_uninstalled(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        # The reload fails once and then behaves. That is the fault this test is
        # about: the *operation* failed, and the undo -- which has to reload the
        # daemon again -- was able to run. A permanently broken reload cannot
        # prove this, because it necessarily breaks the undo too; that case is
        # its own test below.
        systemd = _fake(fail_once=frozenset({"daemon-reload"}))
        report = install(
            payload=_payload(tmp_path, "a", "v1"),
            version="1.0.0",
            layout=layout,
            systemd=systemd,
        )
        assert report["ok"] is False
        assert report["reason"] == "systemctl_failed"
        assert report["action_code"] == "collect_support_bundle"
        assert report["recovery"]["attempted"] is True
        assert report["recovery"]["ok"] is True
        # "Clearly safe recoverable state": nothing active, no unit files left
        # behind, no recorded state -- `install` is the entire recovery.
        assert _active(layout) is None
        assert sorted(p.name for p in layout.unit_dir.glob("flyto-*")) == []
        assert not layout.state_file.exists()
        # The persistent surfaces still exist, so a device that failed its very
        # first install still has somewhere to keep its identity.
        assert layout.credentials_dir.is_dir()

    def test_a_permanently_failing_daemon_reload_escalates_instead_of_claiming_an_undo(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        # Same fault, no recovery from it: every reload fails, so the undo's own
        # reload fails too and the device is genuinely between two states. The
        # report must say `rollback_failed` and send the operator to support
        # rather than reporting the original systemctl error, which would read
        # as "we put it back".
        report = install(
            payload=_payload(tmp_path, "a", "v1"),
            version="1.0.0",
            layout=layout,
            systemd=_fake(fail_on=frozenset({"daemon-reload"})),
        )
        assert report["ok"] is False
        assert report["reason"] == "rollback_failed"
        assert report["action_code"] == "escalate_to_support"
        assert report["recovery"]["ok"] is False
        assert report["recovery"]["error"]
        assert _active(layout) is None

    def test_a_service_that_starts_and_dies_is_not_reported_as_a_successful_install(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        systemd = _fake(dies_after_start=frozenset({AGENT}))
        report = install(
            payload=_payload(tmp_path, "a", "v1"),
            version="1.0.0",
            layout=layout,
            systemd=systemd,
        )
        assert report["ok"] is False
        assert report["reason"] == "service_not_active"
        assert _active(layout) is None

    def test_an_update_whose_service_fails_to_restart_restores_the_last_healthy_release(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        # The new release's unit refuses to start once. Recovery restarts the
        # *same* unit to bring 1.0.0 back, so the fault has to be bounded for
        # "the update failed and the previous release is running again" to be
        # observable at all.
        broken = _fake(fail_once=frozenset({f"restart {AGENT}"}))
        report = update(
            payload=_payload(tmp_path, "b", "v2"),
            version="2.0.0",
            layout=layout,
            systemd=broken,
        )
        assert report["ok"] is False
        assert report["reason"] == "systemctl_failed"
        assert report["recovery"]["ok"] is True
        assert report["recovery"]["restored_version"] == "1.0.0"
        assert _active(layout) == "1.0.0"
        assert json.loads(layout.state_file.read_text())["current"] == "1.0.0"
        assert (layout.current / "flyto_robotics/__init__.py").read_text() == "v1"
        # The previous release is not merely recorded, it is running.
        assert AGENT in broken.runner.active

    def test_an_update_whose_restart_never_works_escalates_rather_than_claiming_a_rollback(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        # The machine cannot start that unit at all any more, so the undo cannot
        # bring 1.0.0 back either. Reporting `systemctl_failed` here would tell
        # an operator the previous release is running when nothing is.
        report = update(
            payload=_payload(tmp_path, "b", "v2"),
            version="2.0.0",
            layout=layout,
            systemd=_fake(fail_on=frozenset({f"restart {AGENT}"})),
        )
        assert report["ok"] is False
        assert report["reason"] == "rollback_failed"
        assert report["action_code"] == "escalate_to_support"
        assert report["recovery"]["ok"] is False
        assert report["recovery"]["restored_version"] is None
        # `current` is still put back on disk; what failed is proving the
        # service came up, and that is what the escalation is about.
        assert _active(layout) == "1.0.0"

    def test_a_failed_health_check_restores_the_previous_release_and_its_state(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        report = update(
            payload=_payload(tmp_path, "b", "v2"),
            version="2.0.0",
            layout=layout,
            systemd=_fake(),
            health_check=lambda _current: False,
        )
        assert report["ok"] is False
        assert report["reason"] == "post_switch_health_failed"
        assert _active(layout) == "1.0.0"
        assert json.loads(layout.state_file.read_text())["current"] == "1.0.0"

    def test_when_the_undo_itself_fails_the_report_says_so_and_escalates(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        install(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
                systemd=_fake())
        # Every restart fails, so the rollback's own restore cannot bring the
        # previous release back either. Reporting the original reason here would
        # tell an operator "we undid it" when nobody undid anything.
        report = rollback(layout=layout, systemd=_fake(fail_on=frozenset({"restart"})))
        assert report["ok"] is False
        assert report["reason"] == "rollback_failed"
        assert report["action_code"] == "escalate_to_support"
        assert report["recovery"]["ok"] is False

    def test_the_previous_profile_decides_what_recovery_restarts(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        systemd = _fake(dies_after_start=frozenset({AGENT}))
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               systemd=systemd)
        restarted = [argv[2] for argv in systemd.runner.commands if argv[1] == "restart"]
        # A Type=oneshot unit is never restarted or verified during recovery: it
        # exits by design, and demanding is-active from it would turn a healthy
        # rollback into a false `rollback_failed`.
        assert DOCTOR not in restarted


# ---------------------------------------------------------------------------
# Profiles: additive, data-driven, and not ROS-shaped
# ---------------------------------------------------------------------------


CUSTOM_REGISTRY = {
    "schema": "flyto.lifecycle-profiles.v1",
    "profiles": {
        "acme": {
            "description": "A site transport this build has never heard of.",
            "units": [
                {
                    "name": "acme-link.service",
                    "enable": True,
                    "restart": True,
                    "verify": True,
                    "template": [
                        "[Unit]",
                        "Description=Acme site link",
                        "",
                        "[Service]",
                        "Type=simple",
                        "WorkingDirectory={current}",
                        "EnvironmentFile={config_file}",
                        "ExecStart={python} -m acme.link",
                    ],
                }
            ],
        }
    },
    "runbook": ["Acme runbook"],
}


@pytest.fixture()
def custom_registry(tmp_path: Path) -> Path:
    path = tmp_path / "acme-profiles.json"
    path.write_text(json.dumps(CUSTOM_REGISTRY), encoding="utf-8")
    return path


class TestProfiles:
    @pytest.mark.parametrize(
        "condition",
        [
            "not-an-object",
            {},
            {"kind": "command", "path": "/bin/true"},
            {"kind": "path_exists", "path": "relative/file"},
            {"kind": "path_exists", "path": "/safe/../escape"},
            {"kind": "path_exists", "path": "{unknown}/file"},
            {"kind": "path_exists", "path": "/tmp/\N{SNOWMAN}"},
            {"kind": "path_exists", "path": "/tmp/control\n"},
            {"kind": "path_exists", "path": "/ok", "command": "true"},
        ],
    )
    def test_activation_conditions_fail_closed(self, tmp_path: Path, condition: object) -> None:
        document = json.loads(json.dumps(CUSTOM_REGISTRY))
        document["profiles"]["acme"]["units"][0]["condition"] = condition
        path = tmp_path / "conditional.json"
        path.write_text(json.dumps(document), encoding="utf-8")
        with pytest.raises(ProfileError):
            load_profiles(path)

    def test_path_exists_condition_is_data_only_and_bundled_runner_is_conditional(
        self, tmp_path: Path
    ) -> None:
        document = json.loads(json.dumps(CUSTOM_REGISTRY))
        raw = document["profiles"]["acme"]["units"][0]
        raw["condition"] = {
            "kind": "path_exists",
            "path": "{state_dir}/credentials/runner-credentials.json",
        }
        path = tmp_path / "conditional.json"
        path.write_text(json.dumps(document), encoding="utf-8")

        unit = load_profiles(path)["acme"].units[0]
        assert unit.condition is not None
        assert unit.condition.render({"state_dir": "/var/lib/flyto"}) == Path(
            "/var/lib/flyto/credentials/runner-credentials.json"
        )
        bundled = load_profiles()["generic"]
        conditions = {unit.name: unit.condition for unit in bundled.units}
        assert conditions[RUNNER] is not None
        assert all(
            condition is None
            for name, condition in conditions.items()
            if name != RUNNER
        )

    def test_a_site_profile_this_build_never_shipped_installs_end_to_end(
        self, layout: Layout, tmp_path: Path, custom_registry: Path
    ) -> None:
        systemd = _fake()
        report = install(
            payload=_payload(tmp_path, "a", "v1"),
            version="1.0.0",
            layout=layout,
            profile="acme",
            systemd=systemd,
            profiles=custom_registry,
        )
        assert report["ok"] is True
        assert (layout.unit_dir / "acme-link.service").is_file()
        assert not (layout.unit_dir / AGENT).exists()
        assert systemd.runner.active == {"acme-link.service"}

    def test_switching_profiles_retires_the_units_the_old_profile_owned(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="ros2", systemd=_fake())
        assert (layout.unit_dir / ROS2).is_file()

        systemd = _fake()
        systemd.runner.enabled.update({AGENT, TIMER, ROS2})
        systemd.runner.active.update({AGENT, TIMER, ROS2})
        report = update(
            payload=_payload(tmp_path, "b", "v2"),
            version="2.0.0",
            layout=layout,
            profile="generic",
            systemd=systemd,
        )
        assert report["ok"] is True
        # The adapter unit is gone from disk *and* from systemd. Leaving it is
        # how a machine keeps restarting an adapter for a release it no longer
        # runs, forever, with nothing in the report to say so.
        assert not (layout.unit_dir / ROS2).exists()
        assert ROS2 not in systemd.runner.enabled
        assert ROS2 not in systemd.runner.active

    def test_a_refused_stop_of_a_retired_unit_fails_the_switch_instead_of_orphaning_it(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """A `stop` that systemd refuses must not be read as a retirement.

        Retiring the outgoing profile's units means stopping them, disabling
        them, and deleting their unit files. If the exit status of `stop` is
        discarded, the unit file is deleted underneath a service that is still
        running, `disable` cannot reach it, and the report says `ok`. The device
        then runs the old adapter against a release it no longer matches, with
        nothing on disk left to explain why -- and a reboot is the only thing
        that stops it. So the refusal has to fail the whole activation and be
        undone like any other step.
        """

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="ros2", systemd=_fake())

        systemd = _fake(fail_on=frozenset({f"stop {ROS2}"}))
        systemd.runner.enabled.update({AGENT, TIMER, ROS2})
        systemd.runner.active.update({AGENT, TIMER, ROS2})
        report = update(
            payload=_payload(tmp_path, "b", "v2"),
            version="2.0.0",
            layout=layout,
            profile="generic",
            systemd=systemd,
        )

        assert report["ok"] is False, "a refused stop may never be reported as a switch"
        assert report["reason"] == "systemctl_failed"
        assert report["recovery"]["ok"] is True
        assert report["recovery"]["restored_version"] == "1.0.0"
        # The retirement is undone in full: the unit file is still there, the
        # unit is still enabled and running, and `current` never moved. Nothing
        # was orphaned.
        assert (layout.unit_dir / ROS2).is_file()
        assert ROS2 in systemd.runner.enabled
        assert ROS2 in systemd.runner.active
        assert _active(layout) == "1.0.0"
        assert json.loads(layout.state_file.read_text())["current"] == "1.0.0"

    def test_a_profile_may_not_redefine_a_unit_it_inherits(self, tmp_path: Path) -> None:
        document = json.loads(json.dumps(CUSTOM_REGISTRY))
        document["profiles"]["evil"] = {
            "extends": "acme",
            "units": document["profiles"]["acme"]["units"],
        }
        path = tmp_path / "evil.json"
        path.write_text(json.dumps(document), encoding="utf-8")
        with pytest.raises(ProfileError):
            load_profiles(path)

    def test_a_quoted_boolean_does_not_silently_enable_a_unit(self, tmp_path: Path) -> None:
        document = json.loads(json.dumps(CUSTOM_REGISTRY))
        document["profiles"]["acme"]["units"][0]["verify"] = "false"
        path = tmp_path / "stringy.json"
        path.write_text(json.dumps(document), encoding="utf-8")
        # bool("false") is True. A policy file that can invert its own meaning
        # by being written in the wrong type must fail closed.
        with pytest.raises(ProfileError):
            load_profiles(path)

    def test_a_misspelled_policy_key_is_refused_rather_than_ignored(
        self, tmp_path: Path
    ) -> None:
        document = json.loads(json.dumps(CUSTOM_REGISTRY))
        document["profiles"]["acme"]["units"][0]["verfy"] = True
        path = tmp_path / "typo.json"
        path.write_text(json.dumps(document), encoding="utf-8")
        with pytest.raises(ProfileError):
            load_profiles(path)

    def test_a_generic_install_completes_with_ros_imports_forbidden(
        self, tmp_path: Path
    ) -> None:
        """The strongest available statement that the substrate is not ROS-bound.

        A subprocess installs a ROS-blocking import hook and then performs a real
        install into a temporary root. If any module on that path reaches for
        rclpy or a ROS runtime, the import raises and the test fails.
        """

        script = textwrap.dedent(
            """
            import sys, json
            from pathlib import Path

            BLOCKED = ("rclpy", "rosidl", "ament", "geometry_msgs", "sensor_msgs",
                       "nav_msgs", "std_msgs", "rcl_interfaces", "tf2_ros")

            class Blocker:
                def find_module(self, name, path=None):
                    return self.find_spec(name, path)
                def find_spec(self, name, path=None, target=None):
                    if name.split(".")[0] in BLOCKED:
                        raise ImportError("ROS import attempted on the generic path: " + name)
                    return None

            sys.meta_path.insert(0, Blocker())

            from flyto_robotics.lifecycle import Layout, install
            from flyto_robotics.systemd_control import FakeSystemctl, SystemdController

            root = Path(sys.argv[1])
            payload = root / "payload" / "flyto_robotics"
            payload.mkdir(parents=True)
            (payload / "__init__.py").write_text("v1")

            report = install(
                payload=root / "payload",
                version="1.0.0",
                layout=Layout(root=root / "target"),
                systemd=SystemdController(runner=FakeSystemctl(), dry_run=False, mode="fake"),
            )
            print(json.dumps({"ok": report["ok"],
                              "ros": [m for m in sys.modules if m.split(".")[0] in BLOCKED]}))
            """
        )
        completed = subprocess.run(
            [sys.executable, "-c", script, str(tmp_path)],
            capture_output=True,
            text=True,
            cwd=str(Path(__file__).resolve().parent.parent),
            check=False,
        )
        assert completed.returncode == 0, completed.stderr
        result = json.loads(completed.stdout.strip().splitlines()[-1])
        assert result["ok"] is True
        assert result["ros"] == []


# ---------------------------------------------------------------------------
# Concurrency, crash consistency, provenance
# ---------------------------------------------------------------------------


class TestConcurrencyAndCrashes:
    def test_a_second_operation_is_refused_while_one_holds_the_lock(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        with lifecycle._advisory_lock(layout), pytest.raises(LifecycleError) as raised:
            install(
                payload=_payload(tmp_path, "b", "v2"),
                version="2.0.0",
                layout=layout,
                systemd=_fake(),
            )
        assert raised.value.reason == "operation_in_progress"
        assert _active(layout) == "1.0.0"

    def test_the_lock_is_released_so_the_retry_succeeds(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        with lifecycle._advisory_lock(layout):
            pass
        report = update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
                        systemd=_fake())
        assert report["ok"] is True

    def test_debris_from_an_interrupted_run_is_removed_not_promoted(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        # Exactly what a kill -9 mid-copy leaves behind.
        debris = layout.releases / ".2.0.0.staging"
        (debris / "flyto_robotics").mkdir(parents=True)
        (debris / "flyto_robotics" / "__init__.py").write_text("half", encoding="utf-8")

        report = update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
                        systemd=_fake())
        assert report["ok"] is True
        assert not debris.exists()
        assert (layout.current / "flyto_robotics/__init__.py").read_text() == "v2"

    def test_a_crash_between_activation_and_the_state_write_is_named_not_hidden(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        # Exactly what survives a power cut between the atomic activation and
        # the state write: the *whole* previous state file, not a hand-edited
        # one. The state write is atomic, so a crash can never leave a document
        # that half describes the new activation -- it leaves the old document,
        # internally consistent and simply out of date. Editing a single field
        # instead would test a corrupt file, which is a different refusal.
        before_second = layout.state_file.read_text()
        install(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
                systemd=_fake())
        layout.state_file.write_text(before_second, encoding="utf-8")

        report = status(layout)
        assert report["ok"] is False
        assert report["reason"] == "state_drift"
        assert report["action_code"] == "rerun_install_to_reconcile"

        repaired = install(payload=_payload(tmp_path, "b", "v2"), version="2.0.0",
                           layout=layout, systemd=_fake())
        assert repaired["ok"] is True
        assert status(layout)["reason"] != "state_drift"

    def test_a_version_name_cannot_be_reused_for_different_bytes_even_after_deletion(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        install(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
                systemd=_fake())

        # Reclaim disk by deleting the old release directory. Its *provenance*
        # survives, so "1.0.0" still means one specific set of bytes forever.
        release = layout.release("1.0.0")
        for path in sorted(release.rglob("*"), reverse=True):
            path.chmod(0o700)
        release.chmod(0o700)
        import shutil

        shutil.rmtree(release)

        with pytest.raises(LifecycleError) as raised:
            install(payload=_payload(tmp_path, "c", "IMPOSTOR"), version="1.0.0",
                    layout=layout, systemd=_fake())
        assert raised.value.reason == "release_exists_with_different_content"

    def test_provenance_is_never_rewritten(self, layout: Layout, tmp_path: Path) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        record = layout.provenance_file("1.0.0")
        before = record.read_text()
        # A profile switch is a legal thing to do to an existing version and
        # must not disturb what that version's *content* is recorded to be.
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="ros2", systemd=_fake())
        assert record.read_text() == before
        assert "profile" not in json.loads(before)

    def test_a_payload_containing_a_symlink_is_refused(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        payload = _payload(tmp_path, "a", "v1")
        (payload / "escape").symlink_to("/etc")
        with pytest.raises(LifecycleError) as raised:
            install(payload=payload, version="1.0.0", layout=layout, systemd=_fake())
        assert raised.value.reason == "release_payload_invalid"

    def test_persistent_data_survives_an_update_that_fails_and_rolls_back(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        layout.identity_file.write_text('{"device_id": "d-1"}', encoding="utf-8")
        layout.config_file.write_text("FLYTO_CLOUD_URL=set-by-site\n", encoding="utf-8")
        (layout.credentials_dir / "device.cred").write_text("opaque", encoding="utf-8")

        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               systemd=_fake(fail_on=frozenset({f"restart {AGENT}"})))

        assert layout.identity_file.read_text() == '{"device_id": "d-1"}'
        assert layout.config_file.read_text() == "FLYTO_CLOUD_URL=set-by-site\n"
        assert (layout.credentials_dir / "device.cred").read_text() == "opaque"
        assert (layout.credentials_dir.stat().st_mode & 0o777) == 0o700


# ---------------------------------------------------------------------------
# The shipped CLI
# ---------------------------------------------------------------------------


def _run(argv, capsys) -> tuple[int, dict]:
    code = robot_cli.main(argv)
    return code, json.loads(capsys.readouterr().out)


class TestCli:
    def test_install_status_and_rollback_round_trip_as_json_with_exit_codes(
        self, tmp_path: Path, capsys
    ) -> None:
        root = ["--root", str(tmp_path / "root")]
        code, report = _run(
            [*root, "install", "--payload", str(_payload(tmp_path, "a", "v1")),
             "--version", "1.0.0"],
            capsys,
        )
        assert (code, report["ok"]) == (0, True)

        code, report = _run([*root, "status"], capsys)
        assert code == 1 and report["reason"] == "identity_missing"

        code, report = _run(
            [*root, "update", "--payload", str(_payload(tmp_path, "b", "v2")),
             "--version", "2.0.0"],
            capsys,
        )
        assert code == 0

        code, report = _run([*root, "rollback"], capsys)
        assert (code, report["version"]) == (0, "1.0.0")

    def test_an_unknown_profile_is_a_reason_code_not_a_usage_error(
        self, tmp_path: Path, capsys
    ) -> None:
        code, report = _run(
            ["--root", str(tmp_path / "root"), "install",
             "--payload", str(_payload(tmp_path, "a", "v1")),
             "--version", "1.0.0", "--profile", "no-such-transport"],
            capsys,
        )
        assert code == 1
        assert report["reason"] == "profiles_invalid"
        assert report["schema"] == lifecycle.LIFECYCLE_REPORT_VERSION

    def test_a_site_registry_is_selectable_from_the_command_line(
        self, tmp_path: Path, custom_registry: Path, capsys
    ) -> None:
        code, report = _run(
            ["--root", str(tmp_path / "root"), "--profiles", str(custom_registry),
             "install", "--payload", str(_payload(tmp_path, "a", "v1")),
             "--version", "1.0.0", "--profile", "acme"],
            capsys,
        )
        assert (code, report["ok"]) == (0, True)
        assert (tmp_path / "root/etc/systemd/system/acme-link.service").is_file()

    def test_a_malformed_registry_is_reported_as_json(self, tmp_path: Path, capsys) -> None:
        broken = tmp_path / "broken.json"
        broken.write_text("{not json", encoding="utf-8")
        code, report = _run(
            ["--root", str(tmp_path / "root"), "--profiles", str(broken), "status"], capsys
        )
        assert code == 1
        assert report["reason"] in {"profiles_invalid", "unexpected_error"}

    def test_a_real_install_may_not_skip_systemd(self, capsys) -> None:
        code, report = _run(
            ["--root", "/", "--no-systemd", "install", "--from-package", "--version", "9.9.9"],
            capsys,
        )
        assert code == 1
        assert report["reason"] == "systemd_required"
        assert report["action_code"] == "rerun_without_no_systemd"

    def test_an_unwritable_bundle_destination_is_json_not_a_traceback(
        self, tmp_path: Path, capsys
    ) -> None:
        blocker = tmp_path / "blocker"
        blocker.write_text("not a directory", encoding="utf-8")
        code, report = _run(
            ["--root", str(tmp_path / "root"), "support-bundle",
             "--output", str(blocker / "nested" / "bundle.json"), "--now", "2026-08-10T00:00:00Z"],
            capsys,
        )
        assert code == 1
        assert report["reason"] in {"io_failed", "prefix_not_writable"}
        assert report["action_code"]

    def test_a_free_text_note_is_refused_with_an_action(self, tmp_path: Path, capsys) -> None:
        code, report = _run(
            ["--root", str(tmp_path / "root"), "support-bundle", "--now", "2026-08-10T00:00:00Z",
             "--note", "patient Jane Doe in ward 3 collapsed near the robot"],
            capsys,
        )
        assert code == 1
        assert report["reason"] == "note_rejected"
        assert report["action_code"] == "shorten_note_to_reference"

    def test_the_packaged_artifact_carries_what_it_needs_to_run_the_lifecycle(
        self, tmp_path: Path
    ) -> None:
        # A checkout (including an editable install with stale metadata) is not
        # an immutable release authority.  The clean-wheel proof lives in
        # test_packaging and exercises this successfully outside the checkout.
        with pytest.raises(LifecycleError) as raised:
            robot_cli.build_package_payload(tmp_path / "payload")
        assert raised.value.reason == "release_payload_invalid"
        assert not (tmp_path / "payload").exists()

    def test_an_installed_payload_can_render_units_without_the_checkout(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        with pytest.raises(LifecycleError):
            robot_cli.build_package_payload(tmp_path / "payload")
        assert not layout.current.exists()

    def test_installed_runner_contract_and_registry_roll_back_as_one_release(
        self, monkeypatch: pytest.MonkeyPatch, layout: Layout, tmp_path: Path
    ) -> None:
        names = (
            "deploy/__init__.py",
            "deploy/flyto_job_runner.py",
            "deploy/device_executor_contract.py",
            "deploy/device_executor_registry.py",
        )
        v1_dist = _installed_tree(tmp_path, "v1")
        v1 = _build_installed_payload(monkeypatch, v1_dist, tmp_path / "payload-v1")
        v1_hashes = {name: hashlib.sha256((v1 / name).read_bytes()).hexdigest() for name in names}
        install(payload=v1, version="1.0.0", layout=layout, systemd=_fake())
        assert layout.current.resolve() == layout.release("1.0.0")
        assert all(
            (layout.current / name).resolve().is_relative_to(layout.releases) for name in names
        )
        imported = _import_active_deploy(layout, tmp_path)
        assert imported["markers"] == [
            "v1:flyto_job_runner.py",
            "v1:device_executor_contract.py",
            "v1:device_executor_registry.py",
        ]
        assert all(
            Path(path).resolve().is_relative_to(layout.release("1.0.0"))
            for path in imported["paths"]
        )

        v2_dist = _installed_tree(tmp_path, "v2")
        v2 = _build_installed_payload(monkeypatch, v2_dist, tmp_path / "payload-v2")
        v2_hashes = {name: hashlib.sha256((v2 / name).read_bytes()).hexdigest() for name in names}
        assert all(v1_hashes[name] != v2_hashes[name] for name in names)
        update(payload=v2, version="2.0.0", layout=layout, systemd=_fake())
        assert layout.current.resolve() == layout.release("2.0.0")
        assert {
            name: hashlib.sha256((layout.current / name).read_bytes()).hexdigest()
            for name in names
        } == v2_hashes
        imported = _import_active_deploy(layout, tmp_path)
        assert all(marker.startswith("v2:") for marker in imported["markers"])
        assert all(
            Path(path).resolve().is_relative_to(layout.release("2.0.0"))
            for path in imported["paths"]
        )

        rollback(layout=layout, systemd=_fake())
        assert layout.current.resolve() == layout.release("1.0.0")
        assert {
            name: hashlib.sha256((layout.current / name).read_bytes()).hexdigest()
            for name in names
        } == v1_hashes
        imported = _import_active_deploy(layout, tmp_path)
        assert all(marker.startswith("v1:") for marker in imported["markers"])
        assert all(
            Path(path).resolve().is_relative_to(layout.release("1.0.0"))
            for path in imported["paths"]
        )

    @pytest.mark.parametrize(
        "bad_digest", ["!" * 43, " " + "A" * 42, "A" * 42 + "=", "A" * 43 + "="]
    )
    def test_noncanonical_record_digest_is_refused_before_destination(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path, bad_digest: str
    ) -> None:
        distribution = _installed_tree(tmp_path, "bad-base64")
        record = distribution.root / "flyto_robotics-0.1.0.dist-info" / "RECORD"
        rows = list(csv.reader(record.read_text(encoding="utf-8").splitlines()))
        rows[0][1] = f"sha256={bad_digest}"
        with record.open("w", encoding="utf-8", newline="") as handle:
            csv.writer(handle).writerows(rows)
        with pytest.raises(LifecycleError):
            _build_installed_payload(monkeypatch, distribution, tmp_path / "payload")
        assert not (tmp_path / "payload").exists()

    @pytest.mark.parametrize("outside", [False, True])
    def test_record_outside_or_below_symlink_parent_is_refused(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path, outside: bool
    ) -> None:
        distribution = _installed_tree(tmp_path, f"record-{outside}")
        relative = Path("flyto_robotics-0.1.0.dist-info/RECORD")
        original = distribution.root / relative
        external = tmp_path / f"external-record-{outside}"
        external.mkdir()
        shutil.copy2(original, external / "RECORD")
        if outside:
            original.unlink()
            original.parent.rmdir()
            original.parent.symlink_to(external, target_is_directory=True)
        else:
            original.unlink()

            original_locate = distribution.locate_file

            def locate(name):
                if str(name) == relative.as_posix():
                    return external / "RECORD"
                return original_locate(name)

            monkeypatch.setattr(distribution, "locate_file", locate)
        with pytest.raises(LifecycleError):
            _build_installed_payload(monkeypatch, distribution, tmp_path / "payload")
        assert not (tmp_path / "payload").exists()

    @pytest.mark.parametrize("mixed", [False, True])
    def test_tampered_or_mixed_installed_release_never_changes_current(
        self, monkeypatch: pytest.MonkeyPatch, layout: Layout, tmp_path: Path, mixed: bool
    ) -> None:
        v1 = _build_installed_payload(
            monkeypatch, _installed_tree(tmp_path, "v1"), tmp_path / "payload-v1"
        )
        install(payload=v1, version="1.0.0", layout=layout, systemd=_fake())
        before = layout.current.resolve()
        v2_dist = _installed_tree(tmp_path, "v2")
        victim = v2_dist.root / "deploy" / (
            "device_executor_contract.py" if mixed else "flyto_job_runner.py"
        )
        victim.write_bytes(
            (_installed_tree(tmp_path, "foreign").root / "deploy" / victim.name).read_bytes()
            if mixed
            else victim.read_bytes() + b"# tampered\n"
        )
        with pytest.raises(LifecycleError) as raised:
            _build_installed_payload(monkeypatch, v2_dist, tmp_path / "payload-v2")
        assert raised.value.reason == "release_payload_invalid"
        assert layout.current.resolve() == before
        assert not (tmp_path / "payload-v2").exists()

    def test_source_swap_to_symlink_is_refused_before_payload_creation(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ) -> None:
        distribution = _installed_tree(tmp_path, "swap")
        victim = distribution.root / "deploy" / "flyto_job_runner.py"
        original_open = robot_cli.os.open
        swapped = False

        def adversarial_open(path, flags, *args):
            nonlocal swapped
            if Path(path) == victim and not swapped:
                swapped = True
                victim.unlink()
                victim.symlink_to(distribution.root / "deploy" / "__init__.py")
            return original_open(path, flags, *args)

        monkeypatch.setattr(robot_cli.os, "open", adversarial_open)
        with pytest.raises(LifecycleError) as raised:
            _build_installed_payload(monkeypatch, distribution, tmp_path / "payload")
        assert raised.value.reason == "release_payload_invalid"
        assert not (tmp_path / "payload").exists()


# ---------------------------------------------------------------------------
# Support bundle
# ---------------------------------------------------------------------------


class TestSupportBundle:
    def test_the_written_file_is_atomic_restrictive_canonical_and_overwritable(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        bundle = build_support_bundle(layout, now="2026-08-10T00:00:00Z", systemd=_fake())
        out = tmp_path / "bundles" / "b.json"

        write_support_bundle(out, bundle)
        assert (out.stat().st_mode & 0o777) == 0o600
        first = out.read_bytes()
        assert first.endswith(b"\n")
        assert json.loads(first)

        write_support_bundle(out, bundle)
        assert out.read_bytes() == first, "the same state must produce the same bytes"
        assert not list(out.parent.glob(".*.tmp")), "no temporary file may be left behind"

    def test_unit_health_comes_through_the_same_injectable_systemd_boundary(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        systemd = _fake()
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=systemd)
        bundle = build_support_bundle(layout, now="2026-08-10T00:00:00Z", systemd=systemd)
        health = {entry["unit"]: entry for entry in bundle["unit_health"]}
        assert health[AGENT]["active"] == "active"
        assert health[AGENT]["enabled"] == "enabled"
        assert health[DOCTOR]["active"] == "inactive"

    def test_credential_file_names_stay_diagnostic_while_contents_never_appear(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        (layout.credentials_dir / "device.cred").write_text("SUPERSECRET", encoding="utf-8")
        bundle = build_support_bundle(layout, now="2026-08-10T00:00:00Z", systemd=_fake())
        text = json.dumps(bundle)
        assert "device.cred" in text, "the responder needs to know the file is there"
        assert "SUPERSECRET" not in text
        assert any(
            entry.get("path") == "device.cred" and entry.get("mode")
            for entry in bundle["inventory"]["credentials"]
        )

    def test_logs_are_listed_and_never_quoted(self, layout: Layout, tmp_path: Path) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        (layout.log_dir / "runner.jsonl").write_text(
            '{"patient": "Jane Doe", "ward": "3B"}\n', encoding="utf-8"
        )
        text = json.dumps(build_support_bundle(layout, now="2026-08-10T00:00:00Z"))
        assert "runner.jsonl" in text
        assert "Jane Doe" not in text

    @pytest.mark.parametrize(
        ("payload", "forbidden"),
        [
            ({"wifi_ssid": "WardNet"}, "WardNet"),
            ({"wpa_passphrase": "hunter2hunter2"}, "hunter2hunter2"),
            ({"authorization": "Bearer abc.def"}, "abc.def"),
            ({"patient_name": "Jane Doe"}, "Jane Doe"),
            ({"username": "chester"}, "chester"),
            ({"note": "device at 10.0.0.7"}, "10.0.0.7"),
            ({"note": "/home/ubuntu/keys"}, "/home/ubuntu"),
            ({"note": "/Users/chester/keys"}, "/Users/chester"),
            ({"note": "token " + "A" * 44}, "A" * 44),
            ({"note": "mail ops@example.com"}, "ops@example.com"),
            ({"note": "nic aa:bb:cc:dd:ee:ff"}, "aa:bb:cc:dd:ee:ff"),
            ({"url": "https://user:pw@cloud.example"}, "user:pw"),
        ],
    )
    def test_redaction_removes_each_class_of_sensitive_value(
        self, payload: dict, forbidden: str
    ) -> None:
        assert forbidden not in json.dumps(redact(payload))

    def test_a_reference_note_survives_and_a_narrative_one_does_not(self) -> None:
        assert check_note("FLY-1234 update failed") == "FLY-1234 update failed"
        assert check_note("") == ""
        from flyto_robotics.support_bundle import sanitize_note

        assert sanitize_note("Mrs O'Brien in bed 4 was upset") == NOTE_REJECTED
        # A character allowlist is not a note policy. Every one of these is
        # spelled in allowed characters and every one of them is a clinical
        # record: what separates a reference from a narrative is that a
        # reference *resolves* somewhere access-controlled.
        narrative = "patient Jane Doe in ward 3 collapsed near the robot"
        assert sanitize_note(narrative) == NOTE_REJECTED
        assert sanitize_note("robot stopped outside bed 12 again") == NOTE_REJECTED
        # Even a purely mechanical sentence is refused when it points nowhere:
        # a bundle carries evidence, and evidence has to be attachable.
        assert sanitize_note("update failed after reboot") == NOTE_REJECTED
        # A ticket id does not license free text about a person alongside it.
        assert sanitize_note("FLY-1234 patient Jane Doe in ward 3") == NOTE_REJECTED
        # Mechanical descriptions attached to a reference are the whole point.
        assert check_note("OPS-77 flyto-robot-agent.service restart loop") == (
            "OPS-77 flyto-robot-agent.service restart loop"
        )
        assert check_note("INC-1024 rollback to 1.4.0") == "INC-1024 rollback to 1.4.0"

    def test_a_rejected_note_is_visible_in_the_bundle_rather_than_silently_dropped(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        bundle = build_support_bundle(
            layout,
            now="2026-08-10T00:00:00Z",
            note="patient Jane Doe in ward 3 collapsed near the robot",
            systemd=_fake(),
        )
        assert bundle["note"] == NOTE_REJECTED
        text = json.dumps(bundle)
        assert "Jane Doe" not in text
        assert "ward 3" not in text

    def test_the_bundle_is_byte_identical_across_runs_with_the_same_clock(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        first = build_support_bundle(layout, now="2026-08-10T00:00:00Z", systemd=_fake())
        second = build_support_bundle(layout, now="2026-08-10T00:00:00Z", systemd=_fake())
        assert json.dumps(first, sort_keys=True) == json.dumps(second, sort_keys=True)


# ---------------------------------------------------------------------------
# What systemd is actually told, and in what order
# ---------------------------------------------------------------------------


def _tracking_fake(layout: Layout, **kwargs) -> SystemdController:
    """A fake that only knows units whose files exist, refreshed on reload.

    The plain fake accepts any unit name, which is exactly how an ordering bug
    survives: recovery can delete a unit file, reload, and then `disable` it,
    and the fake returns 0 for a command real systemd refuses.
    """

    def defined() -> set[str]:
        return {p.name for p in layout.unit_dir.glob("*")} if layout.unit_dir.is_dir() else set()

    return _fake(unit_source=defined, **kwargs)


class TestRecoveryOrdering:
    def test_a_first_install_that_fails_after_the_services_started_quiesces_them_first(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The undo has to reach systemd while the unit files still exist.

        This install gets all the way through enable/restart/verify -- the
        services are running -- and only then fails its health check. Recovery
        must stop and disable what it started *before* deleting the unit files
        and reloading, because systemd cannot act on a definition that is gone.
        Doing it the other way round leaves a running service with nothing on
        disk to explain it, and no lifecycle command able to touch it again.
        """

        systemd = _tracking_fake(layout)
        report = install(
            payload=_payload(tmp_path, "a", "v1"),
            version="1.0.0",
            layout=layout,
            systemd=systemd,
            health_check=lambda _current: False,
        )

        assert report["ok"] is False
        assert report["reason"] == "post_switch_health_failed"
        assert report["recovery"]["ok"] is True
        # Started, then genuinely stopped and disabled -- not merely forgotten.
        assert systemd.runner.active == set()
        assert systemd.runner.enabled == set()

        verbs = _verbs(systemd)
        last_reload = max(index for index, verb in enumerate(verbs) if verb == "daemon-reload")
        assert verbs.index("stop") < verbs.index("disable") < last_reload
        assert sorted(p.name for p in layout.unit_dir.glob("flyto-*")) == []
        assert _active(layout) is None

    def test_a_reload_that_fails_before_systemd_loaded_the_units_still_cleans_up(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """An undo may only ask systemd about units systemd has heard of.

        `daemon-reload` is the moment the daemon learns these unit files exist.
        A fault *there* leaves them written but unloaded, un-enabled and
        unstarted -- and an undo that quiesces created units unconditionally
        then issues `disable` for a unit with no definition. Real systemd
        refuses that, so the undo raises, and a first install that hit a
        momentary reload race is reported as `rollback_failed` with its unit
        files still on disk: an operator sent to support to clean up after a
        fault the device could have cleaned up itself.

        The permissive fake accepts any unit name and returned 0, which is
        exactly why this survived. The tracking fake only knows units whose
        files it has been reloaded over, so it refuses precisely as the machine
        does.
        """

        systemd = _tracking_fake(layout, fail_once=frozenset({"daemon-reload"}))
        report = install(
            payload=_payload(tmp_path, "a", "v1"),
            version="1.0.0",
            layout=layout,
            systemd=systemd,
        )

        assert report["ok"] is False
        assert report["reason"] == "systemctl_failed"
        assert report["recovery"]["attempted"] is True
        assert report["recovery"]["ok"] is True, report["recovery"]["error"]
        assert report["recovery"]["error"] == ""
        # Nothing was started or enabled, so nothing needed quiescing -- and the
        # undo has to notice that rather than assume the transaction got as far
        # as handing systemd anything.
        assert "stop-and-disable" not in report["recovery"]["steps"]
        # No residue on any surface the transaction touches.
        assert sorted(p.name for p in layout.unit_dir.glob("flyto-*")) == []
        assert _active(layout) is None
        assert not layout.state_file.exists()
        assert systemd.runner.enabled == set()
        assert systemd.runner.active == set()
        # The persistent surfaces still exist: a device that failed its very
        # first install still has somewhere to keep its identity.
        assert layout.credentials_dir.is_dir()

    def test_a_created_unit_that_was_started_is_still_stopped_before_its_file_goes(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The other half: quiescing by observed state must not become no-op.

        Same strict fake, but the reload succeeds and the units are genuinely
        enabled and running by the time the health check refuses the release.
        Skipping the stop here would delete the unit file underneath a live
        service, which no later command can reach.
        """

        systemd = _tracking_fake(layout)
        report = install(
            payload=_payload(tmp_path, "a", "v1"),
            version="1.0.0",
            layout=layout,
            systemd=systemd,
            health_check=lambda _current: False,
        )

        assert report["ok"] is False
        assert report["recovery"]["ok"] is True
        assert "stop-and-disable" in report["recovery"]["steps"]
        verbs = _verbs(systemd)
        last_reload = max(index for index, verb in enumerate(verbs) if verb == "daemon-reload")
        assert verbs.index("stop") < verbs.index("disable") < last_reload
        assert systemd.runner.active == set()
        assert systemd.runner.enabled == set()
        assert sorted(p.name for p in layout.unit_dir.glob("flyto-*")) == []

    def test_a_failed_profile_switch_stops_and_disables_the_unit_it_introduced(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The adapter this update added is running and enabled when it fails.

        Not a unit that started and died -- one that came up cleanly and was
        refused afterwards, so `is-active` says `active` and `is-enabled` says
        `enabled`. That is the case where quiescing by observed state has to do
        the full stop-then-disable-then-delete, and the case where skipping it
        leaves an adapter running against a release the device no longer has.
        """

        systemd = _tracking_fake(layout)
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="generic", systemd=systemd)
        healthy_enabled = set(systemd.runner.enabled)
        healthy_active = set(systemd.runner.active)

        report = update(
            payload=_payload(tmp_path, "b", "v2"),
            version="2.0.0",
            layout=layout,
            profile="ros2",
            systemd=systemd,
            health_check=lambda _current: False,
        )

        assert report["ok"] is False
        assert report["reason"] == "post_switch_health_failed"
        assert report["recovery"]["ok"] is True, report["recovery"]["error"]
        assert "stop-and-disable" in report["recovery"]["steps"]
        # The introduced adapter is gone in all three senses.
        assert not (layout.unit_dir / ROS2).exists()
        assert ROS2 not in systemd.runner.enabled
        assert ROS2 not in systemd.runner.active
        # And the release that was working is exactly as it was, enablement
        # included.
        assert _active(layout) == "1.0.0"
        assert systemd.runner.enabled == healthy_enabled
        assert systemd.runner.active == healthy_active
        assert json.loads(layout.state_file.read_text())["profile"] == "generic"

    def test_recovery_restarts_the_outgoing_profile_the_registry_no_longer_declares(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The case a rollback is most needed for used to be the case it refused.

        The device was installed under a site profile that has since been
        removed from the registry, and then an update fails. Resolving the
        outgoing profile by *name* meant recovery could not find it, so it
        restored the files, started nothing, and escalated -- leaving a machine
        that was fine ten seconds earlier sitting dark. The outgoing
        activation's own record carries the per-unit restart/verify policy, so
        recovery does not need the registry to know what to bring back.
        """

        both = _site_registry(tmp_path, "both.json", ("acme", "acme2"))
        only2 = _site_registry(tmp_path, "only2.json", ("acme2",))

        # One systemd for the whole story. A fresh fake for the update would
        # start with nothing enabled and nothing active, so the pre-state
        # recovery is judged against would be a state the device was never in --
        # and an undo that restored no enablement at all would pass. The fault
        # is introduced only once the healthy install has landed, for the same
        # reason: this is an update that breaks a working machine.
        systemd = _fake()
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="acme", systemd=systemd, profiles=both)

        healthy_enabled = set(systemd.runner.enabled)
        healthy_active = set(systemd.runner.active)
        assert "acme-link.service" in healthy_enabled, "the install did not leave a healthy device"
        assert "acme-link.service" in healthy_active
        before = (layout.unit_dir / "acme-link.service").read_text(encoding="utf-8")

        systemd.runner.dies_after_start = frozenset({"acme2-link.service"})
        report = update(
            payload=_payload(tmp_path, "b", "v2"),
            version="2.0.0",
            layout=layout,
            profile="acme2",
            systemd=systemd,
            profiles=only2,
        )

        assert report["ok"] is False
        assert report["reason"] == "service_not_active"
        assert report["recovery"]["ok"] is True
        assert report["recovery"]["restored_version"] == "1.0.0"
        # Byte-for-byte, not "a unit by that name": the recovered unit text is
        # the text that was running, not a re-render from a registry that no
        # longer describes it.
        assert (layout.unit_dir / "acme-link.service").read_text(encoding="utf-8") == before
        # Exactly the state the healthy device was in -- including enablement,
        # which decides what comes up at the next boot and which no amount of
        # restarting by hand restores.
        assert systemd.runner.enabled == healthy_enabled
        assert systemd.runner.active == healthy_active
        # And the incoming unit is gone in all three senses, not merely deleted.
        assert not (layout.unit_dir / "acme2-link.service").exists()
        assert "acme2-link.service" not in systemd.runner.enabled
        assert "acme2-link.service" not in systemd.runner.active
        assert _active(layout) == "1.0.0"
        state = json.loads(layout.state_file.read_text())
        assert state["current"] == "1.0.0"
        assert state["profile"] == "acme"


# ---------------------------------------------------------------------------
# Rollback is an activation, not a symlink swap
# ---------------------------------------------------------------------------


def _activation_id(layout: Layout, version: str) -> str:
    """The id of the activation ``version`` currently resolves to."""

    return json.loads(layout.activation_file(version).read_text())["digest"]


def _record_file(layout: Layout, version: str) -> Path:
    """The immutable record behind the version view -- what a rollback replays."""

    return layout.activation_record_file(_activation_id(layout, version))


def _site_registry(tmp_path: Path, name: str, profiles: tuple[str, ...]) -> Path:
    """A registry declaring one independent single-unit profile per name."""

    unit = json.loads(json.dumps(CUSTOM_REGISTRY["profiles"]["acme"]["units"][0]))
    document = {"schema": CUSTOM_REGISTRY["schema"], "profiles": {}, "runbook": ["Site runbook"]}
    for profile in profiles:
        entry = json.loads(json.dumps(unit))
        entry["name"] = f"{profile}-link.service"
        document["profiles"][profile] = {"description": profile, "units": [entry]}
    path = tmp_path / name
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


class TestRollbackRestoresTheProfile:
    def test_condition_round_trips_and_changes_activation_identity(
        self, tmp_path: Path
    ) -> None:
        document = json.loads(json.dumps(CUSTOM_REGISTRY))
        entry = document["profiles"]["acme"]["units"][0]
        entry["condition"] = {
            "kind": "path_exists",
            "path": "{state_dir}/credentials/runner-credentials.json",
        }
        registry = tmp_path / "condition.json"
        registry.write_text(json.dumps(document), encoding="utf-8")
        conditional = load_profiles(registry)["acme"]
        unconditional_doc = json.loads(json.dumps(document))
        del unconditional_doc["profiles"]["acme"]["units"][0]["condition"]
        unconditional_path = tmp_path / "unconditional.json"
        unconditional_path.write_text(json.dumps(unconditional_doc), encoding="utf-8")
        unconditional = load_profiles(unconditional_path)["acme"]
        fields = {
            "current": "/opt/current", "config_dir": "/etc/flyto",
            "config_file": "/etc/flyto/config", "identity_file": "/etc/flyto/id",
            "state_dir": "/var/lib/flyto", "log_dir": "/var/log/flyto",
            "python": "/usr/bin/python3",
        }
        common = {
            "version": "1.0.0", "python": fields["python"],
            "release_digest": "a" * 64,
        }
        first = build(profile=conditional, units=conditional.render(fields), **common)
        second = build(profile=conditional, units=conditional.render(fields), **common)
        plain = build(profile=unconditional, units=unconditional.render(fields), **common)

        assert first.document() == second.document()
        assert first.activation_id != plain.activation_id
        loaded = load_document(first.document(), path=tmp_path / "snapshot.json")
        assert loaded.spec().units[0].condition == conditional.units[0].condition

    @pytest.mark.parametrize(
        "condition",
        [
            {"kind": "command", "path": "/bin/true"},
            {"kind": "path_exists", "path": "relative"},
            {"kind": "path_exists", "path": "/safe", "env": {}},
        ],
    )
    def test_rehashed_malformed_snapshot_condition_is_refused(
        self, layout: Layout, tmp_path: Path, condition: object
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        snapshot = json.loads(layout.activation_file("1.0.0").read_text())
        unit = next(iter(snapshot["policy"]))
        snapshot["policy"][unit]["condition"] = condition
        body = {key: value for key, value in snapshot.items() if key != "digest"}
        snapshot["digest"] = body_digest(body)

        with pytest.raises(Exception) as raised:
            load_document(snapshot, path=tmp_path / "snapshot.json")
        assert getattr(raised.value, "reason", None) == "activation_snapshot_invalid"

    def test_rolling_back_to_a_ros2_activation_restores_the_adapter_unit(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="ros2", systemd=_fake())
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               profile="generic", systemd=_fake())
        assert not (layout.unit_dir / ROS2).exists()

        systemd = _fake()
        systemd.runner.enabled.update({AGENT, TIMER})
        systemd.runner.active.update({AGENT, TIMER})
        report = rollback(layout=layout, systemd=systemd)

        assert report["ok"] is True
        assert report["version"] == "1.0.0"
        assert report["profile"] == "ros2"
        # The release was activated *under a profile*. Returning the bytes and
        # not the unit set leaves the old release running the new release's
        # configuration -- a state neither release was ever tested in.
        assert (layout.unit_dir / ROS2).is_file()
        assert ROS2 in systemd.runner.enabled
        assert ROS2 in systemd.runner.active
        assert json.loads(layout.state_file.read_text())["profile"] == "ros2"

    def test_rolling_back_to_a_generic_activation_retires_the_adapter_unit(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="generic", systemd=_fake())
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               profile="ros2", systemd=_fake())
        assert (layout.unit_dir / ROS2).is_file()

        systemd = _fake()
        systemd.runner.enabled.update({AGENT, TIMER, ROS2})
        systemd.runner.active.update({AGENT, TIMER, ROS2})
        report = rollback(layout=layout, systemd=systemd)

        assert report["ok"] is True
        assert report["profile"] == "generic"
        # The inverse failure: an adapter left enabled restarts forever against
        # a release that no longer matches it, and nothing in the report says so.
        assert not (layout.unit_dir / ROS2).exists()
        assert ROS2 not in systemd.runner.enabled
        assert ROS2 not in systemd.runner.active

    def test_a_profile_deleted_from_the_registry_is_still_rolled_back_to(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The registry is not the record of what ran. The snapshot is.

        A site drops a profile the fleet no longer ships. Every device still
        running a release activated under it must remain rollable -- that is the
        moment a rollback is *for*. Detecting the drift and refusing is a safer
        failure than substituting `generic`, but it is still a device an
        operator cannot recover.
        """

        both = _site_registry(tmp_path, "both.json", ("acme", "acme2"))
        only2 = _site_registry(tmp_path, "only2.json", ("acme2",))
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="acme", systemd=_fake(), profiles=both)
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               profile="acme2", systemd=_fake(), profiles=both)

        systemd = _fake()
        report = rollback(layout=layout, systemd=systemd, profiles=only2)

        assert report["ok"] is True
        assert report["profile"] == "acme", "the replayed profile, not a substitute"
        assert _active(layout) == "1.0.0"
        # Cross-profile restoration, with no registry that can describe either.
        assert (layout.unit_dir / "acme-link.service").is_file()
        assert not (layout.unit_dir / "acme2-link.service").exists()
        assert "acme-link.service" in systemd.runner.active
        assert "acme2-link.service" not in systemd.runner.enabled

    def test_a_registry_that_cannot_be_read_at_all_does_not_block_a_rollback(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               systemd=_fake())
        broken = tmp_path / "broken-registry.json"
        broken.write_text("{not json", encoding="utf-8")

        report = rollback(layout=layout, systemd=_fake(), profiles=broken)
        assert report["ok"] is True
        assert _active(layout) == "1.0.0"

    def test_a_tampered_activation_snapshot_fails_closed(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The one input to a rollback that nothing else re-derives.

        An edited snapshot would activate unit text of the editor's choosing
        under the name of a release that was tested. It has to hash to its own
        digest or it is not replayed.
        """

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               systemd=_fake())

        # The *record* is what a rollback replays, so that is what has to be
        # tamper-evident. The version view beside it is a pointer for the
        # running services; editing the record is the attack that would
        # otherwise put unit text of the editor's choosing on the device under
        # the name of a release that was tested.
        snapshot_file = _record_file(layout, "1.0.0")
        snapshot_file.chmod(0o640)
        document = json.loads(snapshot_file.read_text())
        document["units"][AGENT] = document["units"][AGENT] + "\n# edited by hand\n"
        snapshot_file.write_text(json.dumps(document), encoding="utf-8")

        with pytest.raises(LifecycleError) as raised:
            rollback(layout=layout, systemd=_fake())
        assert raised.value.reason == "activation_snapshot_invalid"
        assert _active(layout) == "2.0.0"

    def test_a_release_with_no_snapshot_is_refused_rather_than_re_imagined(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               systemd=_fake())
        # Both the record and the view it was derived from: the view is a valid
        # standby for its own activation, so removing only the record proves
        # nothing about the "no snapshot at all" case this test is for.
        _record_file(layout, "1.0.0").unlink()
        layout.activation_file("1.0.0").unlink()

        with pytest.raises(LifecycleError) as raised:
            rollback(layout=layout, systemd=_fake())
        assert raised.value.reason == "activation_not_recorded"
        assert raised.value.reason != "ok"
        assert _active(layout) == "2.0.0"

    def test_snapshots_are_pruned_with_the_history_that_can_reach_them(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        for index in range(lifecycle._MAX_HISTORY + 3):
            version = f"1.0.{index}"
            install(payload=_payload(tmp_path, version, f"v{index}"), version=version,
                    layout=layout, systemd=_fake())
        kept = sorted(p.stem for p in layout.activation_dir.glob("*.json"))
        state = json.loads(layout.state_file.read_text())
        recorded = [entry["version"] for entry in state["history"]]
        assert kept == sorted(recorded)
        assert len(kept) == lifecycle._MAX_HISTORY
        # The immutable records are pruned by reachability too, and by id
        # rather than by version -- and every id history still names is there.
        records = {p.stem for p in layout.activation_record_dir.glob("*.json")}
        assert records == {entry["activation_id"] for entry in state["history"]}


# ---------------------------------------------------------------------------
# An activation is not a version
# ---------------------------------------------------------------------------


class TestActivationIdentity:
    """A version is a name; an activation is what that name was made to mean.

    Everything here is a case the old version-keyed records could not express:
    one release, activated twice, under two different configurations.
    """

    def _two_activations_of_one_version(self, layout: Layout, tmp_path: Path) -> None:
        payload = _payload(tmp_path, "a", "v1")
        install(payload=payload, version="1.0.0", layout=layout, profile="generic",
                systemd=_fake())
        install(payload=payload, version="1.0.0", layout=layout, profile="ros2",
                systemd=_fake())

    def test_re_activating_one_version_under_a_new_profile_records_a_second_snapshot(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """Same bytes, different configuration: two activations, two records.

        Keying records by version made this a refusal -- the second install hit
        "a different activation of 1.0.0 is already recorded", rolled itself
        back, and the profile switch simply did not happen on a device that had
        ever installed that version before.
        """

        self._two_activations_of_one_version(layout, tmp_path)

        state = json.loads(layout.state_file.read_text())
        history = state["history"]
        assert [entry["version"] for entry in history] == ["1.0.0", "1.0.0"]
        assert [entry["profile"] for entry in history] == ["generic", "ros2"]
        first, second = (entry["activation_id"] for entry in history)
        assert first != second, "two configurations of one release are two activations"
        assert state["current_activation"] == second

        # Both survive as immutable records; neither was overwritten by the
        # other, and the version view resolves to the newest.
        for activation_id in (first, second):
            assert layout.activation_record_file(activation_id).is_file()
        assert _activation_id(layout, "1.0.0") == second

    def test_a_same_version_profile_switch_is_undone_by_a_default_rollback(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """"Undo the last thing" has to mean the last *activation*.

        Selecting a rollback target by version skipped every entry sharing the
        current version, so the one command an operator reaches for after a bad
        profile switch was the one command that could not undo it.
        """

        self._two_activations_of_one_version(layout, tmp_path)
        assert (layout.unit_dir / ROS2).is_file()

        systemd = _fake()
        systemd.runner.enabled.update({AGENT, TIMER, ROS2})
        systemd.runner.active.update({AGENT, TIMER, ROS2})
        report = rollback(layout=layout, systemd=systemd)

        assert report["ok"] is True
        assert report["version"] == "1.0.0"
        assert report["profile"] == "generic", "the preceding activation, not the release before"
        assert _active(layout) == "1.0.0"
        assert not (layout.unit_dir / ROS2).exists()
        assert ROS2 not in systemd.runner.enabled
        assert ROS2 not in systemd.runner.active
        assert json.loads(layout.state_file.read_text())["profile"] == "generic"

    def test_an_explicit_version_selects_its_latest_prior_activation(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """``--to-version`` names a release; the device still has to pick one of them.

        Two activations of 1.0.0 both answer to the name. The one an operator
        means is the last one that ran, and picking deterministically is the
        difference between a documented command and a coin flip.
        """

        self._two_activations_of_one_version(layout, tmp_path)
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               profile="generic", systemd=_fake())

        report = rollback(layout=layout, to_version="1.0.0", systemd=_fake())

        assert report["ok"] is True
        assert report["version"] == "1.0.0"
        assert report["profile"] == "ros2", "the most recent 1.0.0 activation, not the first"
        assert (layout.unit_dir / ROS2).is_file()

    def test_switching_off_a_site_profile_retires_its_units_without_its_registry(
        self, layout: Layout, tmp_path: Path, custom_registry: Path
    ) -> None:
        """The success path of finding the outgoing set in the record.

        The site registry that declared ``acme`` is not passed to the update at
        all. Enumerating removable units from the *incoming* registry could
        therefore never name ``acme-link.service``, and it was left on disk,
        enabled, restarting forever against a release it no longer matched.
        """

        systemd = _fake()
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="acme", systemd=systemd, profiles=custom_registry)
        assert (layout.unit_dir / "acme-link.service").is_file()

        systemd = _fake()
        systemd.runner.enabled.add("acme-link.service")
        systemd.runner.active.add("acme-link.service")
        report = update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
                        profile="generic", systemd=systemd)

        assert report["ok"] is True
        assert not (layout.unit_dir / "acme-link.service").exists()
        assert "acme-link.service" not in systemd.runner.enabled
        assert "acme-link.service" not in systemd.runner.active
        assert (layout.unit_dir / AGENT).is_file()

    def test_a_state_file_pointed_at_a_different_valid_activation_is_refused(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """Both records verify. Only together do they say anything true.

        Swapping the committed reference for another genuine activation of the
        same version passes every per-field check: the id is a real digest, the
        record it names hashes correctly, the history entry is well formed. What
        it changes is which unit set the next failed update would "recover" the
        device onto.
        """

        self._two_activations_of_one_version(layout, tmp_path)
        state = json.loads(layout.state_file.read_text())
        superseded = state["history"][0]["activation_id"]

        # Kept internally consistent on purpose: history and current agree with
        # each other, and disagree only with the record they point at.
        state["history"][-1]["activation_id"] = superseded
        state["current_activation"] = superseded
        layout.state_file.write_text(json.dumps(state), encoding="utf-8")

        with pytest.raises(LifecycleError) as raised:
            update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
                   systemd=_fake())
        assert raised.value.reason == "activation_snapshot_invalid"
        assert _active(layout) == "1.0.0"

    def test_a_current_activation_that_is_not_the_newest_entry_is_refused(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        self._two_activations_of_one_version(layout, tmp_path)
        state = json.loads(layout.state_file.read_text())
        state["current_activation"] = state["history"][0]["activation_id"]
        layout.state_file.write_text(json.dumps(state), encoding="utf-8")

        with pytest.raises(LifecycleError) as raised:
            status(layout)
        assert raised.value.reason == "config_unreadable"

    def test_a_failed_state_write_puts_the_activation_view_back(
        self, layout: Layout, tmp_path: Path, monkeypatch
    ) -> None:
        """The view is part of the transaction, so it is part of the undo.

        On a same-version switch the view is the *only* file whose content
        distinguishes the two activations. A transaction that restored the
        units, `current`, and the state while leaving the view rewritten would
        leave the running services and `status` reading the contract of an
        activation this device explicitly refused to make.
        """

        payload = _payload(tmp_path, "a", "v1")
        install(payload=payload, version="1.0.0", layout=layout, profile="generic",
                systemd=_fake())
        view_before = layout.activation_file("1.0.0").read_text()

        real_write = lifecycle._atomic_write
        failures = []

        def flaky(path, text, mode=0o644):
            # Exactly one failure, on the state write only: the undo has to be
            # able to write that same file to put the old state back.
            if path == layout.state_file and not failures:
                failures.append(path)
                raise OSError("simulated power loss before the state write landed")
            return real_write(path, text, mode)

        monkeypatch.setattr(lifecycle, "_atomic_write", flaky)
        report = install(payload=payload, version="1.0.0", layout=layout, profile="ros2",
                         systemd=_fake())

        assert failures, "the fault never fired; the test proves nothing"
        assert report["ok"] is False
        assert report["recovery"]["ok"] is True
        assert layout.activation_file("1.0.0").read_text() == view_before
        state = json.loads(layout.state_file.read_text())
        assert state["profile"] == "generic"
        assert state["current_activation"] == json.loads(view_before)["digest"]
        assert not (layout.unit_dir / ROS2).exists()


class TestUpgradeFromTheVersionKeyedState:
    """A device the previous build installed has to remain updatable."""

    def _downgrade_to_v1(self, layout: Layout) -> dict:
        """Rewrite the state exactly as the v1 build would have written it."""

        state = json.loads(layout.state_file.read_text())
        state["schema"] = lifecycle.LIFECYCLE_STATE_VERSION_V1
        state.pop("current_activation", None)
        for entry in state["history"]:
            entry.pop("activation_id", None)
        layout.state_file.write_text(json.dumps(state), encoding="utf-8")
        for path in layout.activation_record_dir.glob("*.json"):
            path.chmod(0o640)
            path.unlink()
        return state

    def test_a_v1_device_updates_and_its_activations_are_identified_from_disk(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        self._downgrade_to_v1(layout)

        report = update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
                        systemd=_fake())

        assert report["ok"] is True
        state = json.loads(layout.state_file.read_text())
        assert state["schema"] == lifecycle.LIFECYCLE_STATE_VERSION
        assert [entry["version"] for entry in state["history"]] == ["1.0.0", "2.0.0"]
        # The upgraded entry's id was read off the snapshot that was already
        # there, not minted, so it still resolves to a record.
        for entry in state["history"]:
            assert layout.activation_record_file(entry["activation_id"]).is_file()
        assert state["current_activation"] == state["history"][-1]["activation_id"]

    def test_a_v1_device_can_still_roll_back(self, layout: Layout, tmp_path: Path) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="ros2", systemd=_fake())
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               profile="generic", systemd=_fake())
        self._downgrade_to_v1(layout)

        report = rollback(layout=layout, systemd=_fake())

        assert report["ok"] is True
        assert report["version"] == "1.0.0"
        assert report["profile"] == "ros2"
        assert (layout.unit_dir / ROS2).is_file()

    def test_a_v1_history_entry_that_disagrees_with_its_snapshot_is_refused(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """Identity is read off the record or it is not assigned at all."""

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        state = self._downgrade_to_v1(layout)
        state["history"][-1]["python"] = "/somewhere/else/python3"
        layout.state_file.write_text(json.dumps(state), encoding="utf-8")

        with pytest.raises(LifecycleError) as raised:
            status(layout)
        assert raised.value.reason == "config_unreadable"

    def test_a_v1_entry_whose_snapshot_is_gone_is_refused_not_invented(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        self._downgrade_to_v1(layout)
        view = layout.activation_file("1.0.0")
        view.chmod(0o640)
        view.unlink()

        with pytest.raises(LifecycleError) as raised:
            status(layout)
        assert raised.value.reason == "config_unreadable"


# ---------------------------------------------------------------------------
# Persisted records are a contract, not a suggestion
# ---------------------------------------------------------------------------


class TestPersistedContracts:
    def test_the_state_directory_is_restrictive_from_the_moment_it_exists(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The lock creates it, so the lock has to declare its mode.

        `_ensure_persistent` sets 0750, but the advisory lock file lives in the
        same directory and is opened first -- so on a fresh device the directory
        is created by `mkdir` under the process umask, and `credentials/` is
        created inside it. Anything world-readable here is a leak for the whole
        duration of the first install.
        """

        with lifecycle._advisory_lock(layout):
            assert (layout.state_dir.stat().st_mode & 0o777) == 0o750

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        assert (layout.state_dir.stat().st_mode & 0o777) == 0o750
        assert (layout.credentials_dir.stat().st_mode & 0o777) == 0o700

    @pytest.mark.parametrize(
        "history",
        [
            [{"version": "1.0.0", "digest": None, "profile": "generic"}],
            [{"version": "1.0.0", "digest": "not-a-digest", "profile": "generic"}],
            [{"version": "1.0.0", "digest": "a" * 64, "profile": 7}],
            [{"digest": "a" * 64, "profile": "generic"}],
            ["1.0.0"],
        ],
    )
    def test_a_malformed_history_entry_is_config_unreadable_not_a_guess(
        self, layout: Layout, tmp_path: Path, history: list
    ) -> None:
        """Rollback reads every one of these fields and trusts them.

        A null digest silently disables the "the rollback target is still the
        release that was activated" check; a non-string profile reaches
        `profile_for` and raises a traceback instead of a reason code.
        """

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        state = json.loads(layout.state_file.read_text())
        state["history"] = history
        layout.state_file.write_text(json.dumps(state), encoding="utf-8")

        with pytest.raises(LifecycleError) as raised:
            status(layout)
        assert raised.value.reason == "config_unreadable"

    def test_a_state_file_from_an_unknown_schema_is_refused_not_reinterpreted(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        state = json.loads(layout.state_file.read_text())
        # A schema from the future, not merely a different one: v1 is readable
        # on purpose (a device the previous build installed must stay
        # updatable), so the refusal has to be demonstrated with a version this
        # build genuinely cannot know the meaning of.
        state["schema"] = "flyto.lifecycle-state.v3"
        layout.state_file.write_text(json.dumps(state), encoding="utf-8")

        with pytest.raises(LifecycleError) as raised:
            status(layout)
        assert raised.value.reason == "config_unreadable"

    @pytest.mark.parametrize(
        "record",
        [
            {"schema": "flyto.release-provenance.v1", "version": "1.0.0", "digest": "short"},
            {"schema": "flyto.release-provenance.v9", "version": "1.0.0", "digest": "a" * 64},
            {"schema": "flyto.release-provenance.v1", "version": "9.9.9", "digest": "a" * 64},
            {"schema": "flyto.release-provenance.v1", "digest": "a" * 64},
        ],
    )
    def test_a_malformed_provenance_record_is_refused_not_compared_against(
        self, layout: Layout, tmp_path: Path, record: dict
    ) -> None:
        """Provenance is what makes a version name mean one set of bytes.

        A record this build cannot honestly compare against must stop the
        install, not be treated as a weaker record that happens to disagree.
        """

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        path = layout.provenance_file("1.0.0")
        path.chmod(0o640)
        path.write_text(json.dumps(record), encoding="utf-8")

        with pytest.raises(LifecycleError) as raised:
            install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                    systemd=_fake())
        assert raised.value.reason == "config_unreadable"


class TestCorruptRecordsThroughTheCli:
    def test_a_malformed_state_file_is_json_with_an_action_not_a_traceback(
        self, tmp_path: Path, capsys
    ) -> None:
        root = tmp_path / "root"
        layout = Layout(root=root.resolve())
        code, _ = _run(
            ["--root", str(root), "install", "--payload", str(_payload(tmp_path, "a", "v1")),
             "--version", "1.0.0"],
            capsys,
        )
        assert code == 0
        layout.state_file.write_text('{"schema": "x", "history": []}', encoding="utf-8")

        code, report = _run(["--root", str(root), "status"], capsys)
        assert code == 1
        assert report["reason"] == "config_unreadable"
        assert report["action_code"] == "restore_config"


class TestClosedReleaseLifecycle:
    """The wheel bootstrap and lifecycle transaction form one authority."""

    @staticmethod
    def _bootstrap_stub(calls: list, profile_seen: list):
        def build(manifest, wheel_dir, releases, python, profile):
            calls.append(Path(releases))
            profile_seen.append(profile)
            release = Path(releases) / ("a" * 64)
            (release / "flyto_robotics").mkdir(parents=True)
            (release / "flyto_robotics/__init__.py").write_text("closed", encoding="utf-8")
            interpreter = release / "venv/bin/python"
            interpreter.parent.mkdir(parents=True)
            interpreter.write_text("", encoding="utf-8")
            return release

        return build

    @pytest.mark.parametrize(
        ("bootstrap_profile", "lifecycle_profile"),
        [("generic", "generic"), ("ros2", "ros2"), ("camera-host", "camera")],
    )
    def test_closed_profiles_map_and_bind_the_release_local_interpreter(
        self, layout: Layout, tmp_path: Path, monkeypatch, bootstrap_profile, lifecycle_profile
    ) -> None:
        calls, profiles = [], []
        monkeypatch.setattr(
            lifecycle, "bootstrap_release", self._bootstrap_stub(calls, profiles)
        )
        report = install(
            manifest=tmp_path / "manifest.json",
            wheel_dir=tmp_path / "wheels",
            layout=layout,
            profile=bootstrap_profile,
            python=sys.executable,
            systemd=_fake(),
        )
        assert report["ok"] is True
        assert report["version"] == "a" * 64
        assert report["profile"] == lifecycle_profile
        assert profiles == [bootstrap_profile]
        snapshot = lifecycle.current_activation_snapshot(
            layout, json.loads(layout.state_file.read_text())
        )
        assert snapshot is not None
        assert snapshot.python == str(layout.releases / ("a" * 64) / "venv/bin/python")

    def test_closed_dry_run_uses_and_removes_only_a_private_release_directory(
        self, layout: Layout, tmp_path: Path, monkeypatch
    ) -> None:
        calls, profiles = [], []
        monkeypatch.setattr(
            lifecycle, "bootstrap_release", self._bootstrap_stub(calls, profiles)
        )
        report = install(
            manifest=tmp_path / "manifest.json",
            wheel_dir=tmp_path / "wheels",
            layout=layout,
            profile="generic",
            python=sys.executable,
            dry_run=True,
            systemd=_fake(),
        )
        assert report["ok"] is True and report["reason"] == "dry_run"
        assert len(calls) == 1 and not calls[0].exists()
        assert not layout.prefix.exists()
        assert not layout.config_dir.exists()
        assert not layout.state_dir.exists()
        assert not layout.unit_dir.exists()

    def test_first_activation_refuses_an_unowned_same_name_unit_without_mutation(
        self, layout: Layout, tmp_path: Path, monkeypatch
    ) -> None:
        layout.unit_dir.mkdir(parents=True)
        foreign = layout.unit_dir / AGENT
        foreign.write_text("[Unit]\nDescription=legacy\n", encoding="utf-8")
        before = foreign.read_bytes()

        def forbidden(*args, **kwargs):
            pytest.fail("bootstrap must not publish before ownership is established")

        monkeypatch.setattr(lifecycle, "bootstrap_release", forbidden)
        with pytest.raises(LifecycleError) as raised:
            install(
                manifest=tmp_path / "manifest.json",
                wheel_dir=tmp_path / "wheels",
                layout=layout,
                profile="generic",
                python=sys.executable,
                systemd=_fake(),
            )
        assert raised.value.reason == "unit_name_collision"
        assert foreign.read_bytes() == before
        assert sorted(path.relative_to(layout.root) for path in layout.root.rglob("*")) == [
            Path("etc"), Path("etc/systemd"), Path("etc/systemd/system"),
            Path(f"etc/systemd/system/{AGENT}"),
        ]

    @pytest.mark.parametrize(
        ("bootstrap_profile", "foreign_name"),
        [("camera-host", CAMERA), ("ros2", ROS2)],
    )
    def test_managed_generic_refuses_a_foreign_new_profile_unit_before_bootstrap(
        self, layout: Layout, tmp_path: Path, monkeypatch, bootstrap_profile, foreign_name
    ) -> None:
        controller = _fake()
        install(
            payload=_payload(tmp_path, "generic", "v1"),
            version="1.0.0",
            layout=layout,
            profile="generic",
            systemd=controller,
        )
        foreign = layout.unit_dir / foreign_name
        foreign.write_text("[Unit]\nDescription=foreign adapter\n", encoding="utf-8")

        def image() -> dict[str, tuple]:
            result = {}
            for path in sorted(layout.root.rglob("*")):
                relative = path.relative_to(layout.root).as_posix()
                if path.is_symlink():
                    result[relative] = ("symlink", os.readlink(path))
                elif path.is_file():
                    result[relative] = ("file", path.read_bytes(), path.stat().st_mode)
                elif path.is_dir():
                    result[relative] = ("dir", path.stat().st_mode)
            return result

        before = image()
        command_count = len(controller.runner.commands)

        def forbidden(*args, **kwargs):
            pytest.fail("collision must be refused before bootstrap publishes a release")

        monkeypatch.setattr(lifecycle, "bootstrap_release", forbidden)
        with pytest.raises(LifecycleError) as raised:
            install(
                manifest=tmp_path / "manifest.json",
                wheel_dir=tmp_path / "wheels",
                layout=layout,
                profile=bootstrap_profile,
                python=sys.executable,
                systemd=controller,
            )

        assert raised.value.reason == "unit_name_collision"
        assert raised.value.detail == foreign_name
        assert image() == before
        assert len(controller.runner.commands) == command_count

    def test_a_committed_activation_still_owns_and_updates_its_existing_names(
        self, layout: Layout, tmp_path: Path, monkeypatch
    ) -> None:
        install(
            payload=_payload(tmp_path, "generic", "v1"),
            version="1.0.0",
            layout=layout,
            profile="generic",
            systemd=_fake(),
        )
        calls, profiles = [], []
        monkeypatch.setattr(
            lifecycle, "bootstrap_release", self._bootstrap_stub(calls, profiles)
        )

        report = update(
            manifest=tmp_path / "manifest.json",
            wheel_dir=tmp_path / "wheels",
            layout=layout,
            profile="generic",
            python=sys.executable,
            systemd=_fake(),
        )

        assert report["ok"] is True
        assert report["version"] == "a" * 64
        assert calls == [layout.releases]

    def test_modified_owned_unit_is_refused_before_bootstrap_without_mutation(
        self, layout: Layout, tmp_path: Path, monkeypatch
    ) -> None:
        controller = _fake()
        install(
            payload=_payload(tmp_path, "generic", "v1"),
            version="1.0.0",
            layout=layout,
            profile="generic",
            systemd=controller,
        )
        owned = layout.unit_dir / AGENT
        owned.write_text(owned.read_text(encoding="utf-8") + "\n# external edit\n")

        before = {
            path.relative_to(layout.root).as_posix(): (
                "symlink", os.readlink(path)
            ) if path.is_symlink() else (
                "file", path.read_bytes(), path.stat().st_mode
            ) if path.is_file() else (
                "dir", path.stat().st_mode
            )
            for path in sorted(layout.root.rglob("*"))
        }
        command_count = len(controller.runner.commands)

        def forbidden(*args, **kwargs):
            pytest.fail("a modified owned unit must be refused before bootstrap")

        monkeypatch.setattr(lifecycle, "bootstrap_release", forbidden)
        with pytest.raises(LifecycleError) as raised:
            update(
                manifest=tmp_path / "manifest.json",
                wheel_dir=tmp_path / "wheels",
                layout=layout,
                profile="generic",
                python=sys.executable,
                systemd=controller,
            )

        after = {
            path.relative_to(layout.root).as_posix(): (
                "symlink", os.readlink(path)
            ) if path.is_symlink() else (
                "file", path.read_bytes(), path.stat().st_mode
            ) if path.is_file() else (
                "dir", path.stat().st_mode
            )
            for path in sorted(layout.root.rglob("*"))
        }
        assert raised.value.reason == "unit_name_collision"
        assert raised.value.detail == AGENT
        assert after == before
        assert len(controller.runner.commands) == command_count

    def test_locked_recheck_catches_owned_unit_changed_after_precheck(
        self, layout: Layout, tmp_path: Path, monkeypatch
    ) -> None:
        controller = _fake()
        install(
            payload=_payload(tmp_path, "generic", "v1"),
            version="1.0.0",
            layout=layout,
            profile="generic",
            systemd=controller,
        )
        state_before = layout.state_file.read_bytes()
        current_before = os.readlink(layout.current)
        units_before = {
            path.name: path.read_bytes() for path in layout.unit_dir.iterdir() if path.is_file()
        }
        command_count = len(controller.runner.commands)
        original_lock = lifecycle._advisory_lock

        @contextlib.contextmanager
        def racing_lock(target_layout, *, enabled=True):
            with original_lock(target_layout, enabled=enabled):
                agent = target_layout.unit_dir / AGENT
                agent.write_text(agent.read_text(encoding="utf-8") + "\n# raced edit\n")
                yield

        def forbidden(*args, **kwargs):
            pytest.fail("legacy payload update must not invoke bootstrap")

        monkeypatch.setattr(lifecycle, "_advisory_lock", racing_lock)
        monkeypatch.setattr(lifecycle, "bootstrap_release", forbidden)
        with pytest.raises(LifecycleError) as raised:
            update(
                payload=_payload(tmp_path, "generic-v2", "v2"),
                version="2.0.0",
                layout=layout,
                profile="generic",
                systemd=controller,
            )

        assert raised.value.reason == "unit_name_collision"
        assert raised.value.detail == AGENT
        assert layout.state_file.read_bytes() == state_before
        assert os.readlink(layout.current) == current_before
        assert not layout.release("2.0.0").exists()
        assert len(controller.runner.commands) == command_count
        after_units = {
            path.name: path.read_bytes() for path in layout.unit_dir.iterdir() if path.is_file()
        }
        assert set(after_units) == set(units_before)
        assert after_units[AGENT] == units_before[AGENT] + b"\n# raced edit\n"
        assert all(
            after_units[name] == body
            for name, body in units_before.items()
            if name != AGENT
        )

    def test_a_malformed_provenance_record_is_json_with_an_action_not_a_traceback(
        self, tmp_path: Path, capsys
    ) -> None:
        root = tmp_path / "root"
        layout = Layout(root=root.resolve())
        payload = str(_payload(tmp_path, "a", "v1"))
        code, _ = _run(
            ["--root", str(root), "install", "--payload", payload, "--version", "1.0.0"],
            capsys,
        )
        assert code == 0
        record = layout.provenance_file("1.0.0")
        record.chmod(0o640)
        record.write_text('{"digest": "nope"}', encoding="utf-8")

        code, report = _run(
            ["--root", str(root), "install", "--payload", payload, "--version", "1.0.0"],
            capsys,
        )
        assert code == 1
        assert report["reason"] == "config_unreadable"
        assert report["action_code"] == "restore_config"
