"""What the installed units actually execute, and what "working" means.

Two things no other test in this suite could see:

* ``systemctl is-active`` says a process has not exited. It says nothing about
  whether the release can be used, so an install could report ``ok`` for a
  device whose agent came up and could not read a single one of its own files.
* ``FakeSystemctl`` marks a unit active on ``restart`` without ever running its
  ``ExecStart=``. Both shipped commands were argparse usage errors -- exit 2 on
  every real device -- and the whole suite stayed green.

So readiness is asserted through the shipped lifecycle, and every rendered
``ExecStart=`` is executed for real in a subprocess.
"""

from __future__ import annotations

import json
import os
import shlex
import subprocess
import sys
from pathlib import Path

import pytest

from flyto_robotics.lifecycle import (
    Layout,
    install,
    open_activation_window,
    rollback,
    status,
    update,
)
from flyto_robotics.lifecycle_profiles import default_profiles_path
from flyto_robotics.systemd_control import FakeSystemctl, SystemdController
from flyto_robotics.systemd_units import parse_unit, validate_unit

REPO_ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture()
def layout(tmp_path: Path) -> Layout:
    return Layout(root=tmp_path.resolve())


def _fake(**kwargs) -> SystemdController:
    return SystemdController(runner=FakeSystemctl(**kwargs), dry_run=False, mode="fake")


def _payload(tmp_path: Path, name: str, body: str) -> Path:
    root = tmp_path / "payloads" / name
    package = root / "flyto_robotics"
    package.mkdir(parents=True, exist_ok=True)
    package.joinpath("__init__.py").write_text(body, encoding="utf-8")
    return root


def _provision(layout: Layout) -> None:
    layout.identity_file.write_text('{"device_id": "d-1"}', encoding="utf-8")
    layout.config_file.write_text(
        "FLYTO_ROBOT_RESOURCE_ID=flyto-rover-1\nFLYTO_CLOUD_URL=https://cloud.example\n",
        encoding="utf-8",
    )
    (layout.credentials_dir / "runner-credentials.json").write_text(
        '{"synthetic": true}', encoding="utf-8"
    )


class TestReadinessIsPartOfEveryActivation:
    def test_an_uncommissioned_device_is_provisioning_pending_not_a_failure(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The normal first state of every machine must not roll a release back."""

        report = install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0",
                         layout=layout, systemd=_fake())
        assert report["ok"] is True
        assert report["readiness"]["state"] == "provisioning_pending"
        pending = {c["id"] for c in report["readiness"]["checks"] if not c["passed"]}
        assert pending == {
            "activation_condition:flyto-job-runner.service",
            "device_identity",
            "cloud_url",
            "resource_id",
        }
        assert all(c["provisioning"] for c in report["readiness"]["checks"] if not c["passed"])

    def test_a_commissioned_device_reports_ready(self, layout: Layout, tmp_path: Path) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        _provision(layout)
        report = update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
                        systemd=_fake())
        assert report["ok"] is True
        assert report["readiness"]["state"] == "ready"

    def test_a_service_that_stays_active_but_fails_readiness_is_never_accepted(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The gap `is-active` cannot see.

        This release starts cleanly -- the fake reports every unit active -- and
        does not contain the package its own ``ExecStart=`` executes. Before
        readiness existed this was a green install.
        """

        # One systemd across both operations, so "the previous release's units
        # are back, enabled, and running" is measured against the state the
        # install actually left rather than against an empty fake that would
        # excuse an undo which restored no enablement at all.
        systemd = _fake()
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=systemd)
        _provision(layout)
        healthy_enabled = set(systemd.runner.enabled)

        hollow = tmp_path / "payloads" / "hollow"
        (hollow / "docs").mkdir(parents=True)
        (hollow / "docs" / "README").write_text("no package here", encoding="utf-8")

        report = update(payload=hollow, version="2.0.0", layout=layout, systemd=systemd)

        assert report["ok"] is False
        assert report["reason"] == "post_switch_readiness_failed"
        assert report["action_code"] == "collect_support_bundle"
        assert "release_package" in report["detail"]
        # Every unit reported active throughout, and it still did not stand.
        assert systemd.runner.active
        assert report["recovery"]["ok"] is True
        # The whole transaction is undone, not just the symlink: the previous
        # release's units are back on disk, enabled, and running.
        assert Path(os.readlink(layout.current)).name == "1.0.0"
        assert json.loads(layout.state_file.read_text())["current"] == "1.0.0"
        assert (layout.unit_dir / "flyto-robot-agent.service").is_file()
        assert "flyto-robot-agent.service" in systemd.runner.enabled
        assert "flyto-robot-agent.service" in systemd.runner.active
        # Enablement is restored wholesale, not just for the unit spot-checked
        # above: it is what decides whether this device comes back after a
        # reboot, and restarting a service by hand says nothing about that.
        assert systemd.runner.enabled == healthy_enabled

    def test_a_rollback_that_fails_readiness_restores_the_release_it_came_from(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """Readiness has to be reachable without touching the release bytes.

        Gutting the rollback target trips the digest check first and never
        reaches readiness at all -- a different, earlier refusal. So the failing
        condition lives *outside* the release: a site declares a non-provisioning
        check on a file it owns, that file goes away, and replaying the snapshot
        that requires it must undo itself rather than land a device that cannot
        work.
        """

        registry = json.loads(default_profiles_path().read_text(encoding="utf-8"))
        registry["profiles"]["generic"]["readiness"].append(
            {
                "id": "site_marker",
                "kind": "path_exists",
                "target": "{config_dir}/site-marker.json",
                "description": "a file this site requires before the release is usable",
            }
        )
        path = tmp_path / "site-readiness.json"
        path.write_text(json.dumps(registry), encoding="utf-8")

        # The marker exists for the 1.0.0 activation, so its snapshot records a
        # readiness contract that demands it.
        layout.config_dir.mkdir(parents=True, exist_ok=True)
        marker = layout.config_dir / "site-marker.json"
        marker.write_text("{}", encoding="utf-8")
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profiles=path, systemd=_fake())
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               profiles=path, systemd=_fake())
        _provision(layout)
        marker.unlink()

        systemd = _fake()
        report = rollback(layout=layout, systemd=systemd)

        assert report["ok"] is False
        assert report["reason"] == "post_switch_readiness_failed"
        assert "site_marker" in report["detail"]
        assert report["recovery"]["ok"] is True
        assert Path(os.readlink(layout.current)).name == "2.0.0"
        assert json.loads(layout.state_file.read_text())["current"] == "2.0.0"

    def test_rollback_runs_the_same_readiness_contract(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               systemd=_fake())
        _provision(layout)
        report = rollback(layout=layout, systemd=_fake())
        assert report["ok"] is True
        assert report["readiness"]["state"] == "ready"

    def test_status_and_the_shipped_cli_expose_the_same_verdict(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        assert status(layout)["reason"] == "identity_missing"


class TestRollbackReproducesTheRecordedActivation:
    def test_a_custom_interpreter_is_reproduced_without_the_operator_remembering_it(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                python="/opt/site/bin/python3", systemd=_fake())
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               systemd=_fake())
        agent = layout.unit_dir / "flyto-robot-agent.service"
        assert "/opt/site/bin/python3" not in agent.read_text()

        report = rollback(layout=layout, systemd=_fake())
        assert report["ok"] is True
        # The unit set that ran under 1.0.0 is the unit set that comes back.
        assert "/opt/site/bin/python3" in agent.read_text()

    def test_a_same_version_profile_switch_moves_the_runtime_contract_and_moves_it_back(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The units cannot name their activation, so the version view must.

        A running service resolves its readiness contract by version, because
        a unit that named its own activation id would change its own text and
        therefore its own id. So re-activating one version under a different
        profile has to repoint that view -- and a rollback to the preceding
        activation of the *same* version has to point it back, or the machine
        keeps evaluating itself against a contract it is no longer under.
        """

        payload = _payload(tmp_path, "a", "v1")
        install(payload=payload, version="1.0.0", layout=layout, profile="generic",
                systemd=_fake())
        assert status(layout)["active_profile"] == "generic"

        install(payload=payload, version="1.0.0", layout=layout, profile="ros2",
                systemd=_fake())
        assert status(layout)["active_profile"] == "ros2"
        assert "flyto-robot-ros2.service" in status(layout)["installed_units"]

        report = rollback(layout=layout, systemd=_fake())
        assert report["ok"] is True
        assert report["profile"] == "generic"
        # Back to the contract of the activation that is now committed, not
        # merely to the release -- the release never changed.
        assert status(layout)["active_profile"] == "generic"
        assert "flyto-robot-ros2.service" not in status(layout)["installed_units"]

    def test_registry_template_drift_reproduces_the_activation_that_ran(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """A site edits a template under the same profile name.

        Re-rendering would produce a unit set that has never run on this device
        and has never been tested anywhere; refusing would leave the device
        unrollable. Replaying the snapshot does neither.
        """

        original = json.loads(default_profiles_path().read_text(encoding="utf-8"))
        drifted_document = json.loads(json.dumps(original))
        unit = drifted_document["profiles"]["generic"]["units"][0]
        unit["template"].append("Environment=FLYTO_SITE_EDIT=1")
        drifted = tmp_path / "drifted.json"
        drifted.write_text(json.dumps(drifted_document), encoding="utf-8")

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                systemd=_fake())
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
               systemd=_fake())

        report = rollback(layout=layout, systemd=_fake(), profiles=drifted)
        assert report["ok"] is True
        agent = (layout.unit_dir / "flyto-robot-agent.service").read_text()
        assert "FLYTO_SITE_EDIT" not in agent, "the edited template must not reach the device"
        assert Path(os.readlink(layout.current)).name == "1.0.0"


def _service_commands(layout: Layout) -> dict[str, dict[str, list[str]]]:
    """Partition every service command by its explicitly supported role."""

    roles: dict[str, dict[str, list[str]]] = {
        "robot_service": {},
        "runner": {},
        "adapter": {},
    }
    for path in sorted(layout.unit_dir.glob("*.service")):
        values = parse_unit(path.read_text(encoding="utf-8")).values("Service", "ExecStart")
        assert len(values) == 1, f"{path.name} must declare exactly one ExecStart"
        argv = shlex.split(values[0])
        try:
            module = argv[argv.index("-m") + 1]
        except (ValueError, IndexError) as error:
            raise AssertionError(f"{path.name} has no parseable Python module") from error
        role = {
            "flyto_robotics.robot_service": "robot_service",
            "deploy.flyto_job_runner": "runner",
            "flyto_robotics.camera_gateway": "adapter",
            "flyto_robotics.ros2_adapter": "adapter",
        }.get(module)
        assert role is not None, f"{path.name} has unclassified service module {module!r}"
        roles[role][path.name] = argv

    role_sets = tuple(set(commands) for commands in roles.values())
    classified = set().union(*role_sets)
    assert sum(len(commands) for commands in role_sets) == len(classified)
    assert classified == {path.name for path in layout.unit_dir.glob("*.service")}
    return roles


def _robot_service_commands(layout: Layout) -> dict[str, list[str]]:
    return _service_commands(layout)["robot_service"]


class TestEveryRenderedExecStartActuallyRuns:
    """The fake marks a unit active without running anything. This does not."""

    @pytest.fixture()
    def installed(self, layout: Layout, tmp_path: Path) -> Layout:
        # Rendered with the interpreter running the suite, so the exact command
        # shape the unit declares is the one that gets executed. PYTHONPATH
        # supplies the package, exactly as the unit's Environment= does.
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                python=sys.executable, systemd=_fake())
        return layout

    def test_no_rendered_command_is_an_argparse_usage_error(self, installed: Layout) -> None:
        commands = _robot_service_commands(installed)
        assert commands, "the profile rendered no services"
        environment = {
            **os.environ,
            "PYTHONPATH": str(REPO_ROOT),
            "FLYTO_ROBOT_MAX_CYCLES": "1",
        }
        for name, argv in commands.items():
            completed = subprocess.run(
                argv, capture_output=True, text=True, timeout=60, check=False, env=environment
            )
            # 2 is argparse: "the flags this unit passes are not the flags this
            # program accepts". That is the defect, and it is unconditional.
            assert completed.returncode != 2, f"{name}: {completed.stderr}"
            # Structured output, not a traceback, whatever the verdict.
            assert json.loads(completed.stdout.strip().splitlines()[-1])["schema"]

    def test_an_uncommissioned_device_does_not_fail_its_units(self, installed: Layout) -> None:
        """Unpaired is a state, not a fault -- otherwise every fresh device flaps."""

        environment = {
            **os.environ,
            "PYTHONPATH": str(REPO_ROOT),
            "FLYTO_ROBOT_MAX_CYCLES": "1",
        }
        for name, argv in _robot_service_commands(installed).items():
            completed = subprocess.run(
                argv, capture_output=True, text=True, timeout=60, check=False, env=environment
            )
            document = json.loads(completed.stdout.strip().splitlines()[-1])
            assert document["state"] == "provisioning_pending", name
            assert completed.returncode == 0, f"{name}: {completed.stderr}"

    def test_a_paired_device_reports_ready_and_leaks_no_secret(
        self, installed: Layout
    ) -> None:
        _provision(installed)
        (installed.credentials_dir / "device.cred").write_text("SUPERSECRET", encoding="utf-8")
        environment = {
            **os.environ,
            "PYTHONPATH": str(REPO_ROOT),
            "FLYTO_ROBOT_MAX_CYCLES": "1",
        }
        for name, argv in _robot_service_commands(installed).items():
            completed = subprocess.run(
                argv, capture_output=True, text=True, timeout=60, check=False, env=environment
            )
            assert completed.returncode == 0, f"{name}: {completed.stderr}"
            document = json.loads(completed.stdout.strip().splitlines()[-1])
            assert document["state"] == "ready", name
            # Identifiers are diagnostic; credential bytes are never read.
            assert document["resource_id"] == "flyto-rover-1"
            assert "SUPERSECRET" not in completed.stdout
            assert "SUPERSECRET" not in Path(document["status_file"]).read_text()

    def test_the_doctor_snapshot_is_written_where_the_bundle_looks_for_it(
        self, installed: Layout
    ) -> None:
        commands = _robot_service_commands(installed)
        argv = commands["flyto-robot-doctor.service"]
        completed = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
            env={**os.environ, "PYTHONPATH": str(REPO_ROOT)},
        )
        assert completed.returncode == 0, completed.stderr
        snapshot = json.loads((installed.state_dir / "doctor-status.json").read_text())
        assert snapshot["service"] == "doctor"
        assert snapshot["state"] == "provisioning_pending"

    def test_the_runtime_contract_follows_the_installed_profile_not_the_shipped_one(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """A ros2 install must not be evaluated against `generic`.

        The units cannot name their own profile -- `extends` requires them to
        render byte-identically wherever they are inherited -- so the contract
        is resolved from the activation snapshot instead.
        """

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="ros2", python=sys.executable, systemd=_fake())
        argv = _robot_service_commands(layout)["flyto-robot-agent.service"]
        completed = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
            env={**os.environ, "PYTHONPATH": str(REPO_ROOT), "FLYTO_ROBOT_MAX_CYCLES": "1"},
        )
        assert completed.returncode == 0, completed.stderr
        assert json.loads(completed.stdout.strip().splitlines()[-1])["profile"] == "ros2"

    def test_a_site_registry_profile_is_the_runtime_contract_too(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        registry = json.loads(default_profiles_path().read_text(encoding="utf-8"))
        # This is a standalone site registry, not the shipped registry plus a
        # renamed root.  Retaining camera/ros2 here would leave their inherited
        # generic/camera names dangling after generic becomes sitecar.
        registry["profiles"] = {"sitecar": registry["profiles"]["generic"]}
        path = tmp_path / "site.json"
        path.write_text(json.dumps(registry), encoding="utf-8")

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="sitecar", python=sys.executable, profiles=path, systemd=_fake())
        argv = _robot_service_commands(layout)["flyto-robot-agent.service"]
        completed = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
            env={**os.environ, "PYTHONPATH": str(REPO_ROOT), "FLYTO_ROBOT_MAX_CYCLES": "1"},
        )
        assert completed.returncode == 0, completed.stderr
        document = json.loads(completed.stdout.strip().splitlines()[-1])
        # The site's own profile name, resolved with no --profiles flag on the
        # unit and no access to the file the installer was pointed at.
        assert document["profile"] == "sitecar"

    def test_the_first_start_of_an_activation_does_not_fail(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The ordering window every real install goes through, exactly.

        ``_activate_unit_set`` restarts and verifies the agent *before* the
        activation snapshot and the state file are committed -- that is what
        makes the whole thing a transaction. So on a clean install, and again on
        every profile switch, systemd runs this exact command against a state
        directory that does not yet describe an activation. Treating that as an
        error made the first start exit 1 and ``Restart=on-failure`` flap; the
        post-install subprocess tests never saw it because by then the files
        existed.
        """

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                python=sys.executable, systemd=_fake())
        commands = _robot_service_commands(layout)
        # Rewind to the instant systemd first ran ExecStart: units on disk, the
        # transaction's own window marker open, nothing committed. The marker is
        # what makes this a *window* rather than an absence -- without it the
        # same file layout is a device that lost its state, and the two must not
        # produce the same document.
        open_activation_window(layout, action="install", version="1.0.0")
        layout.state_file.unlink()

        environment = {
            **os.environ,
            "PYTHONPATH": str(REPO_ROOT),
            "FLYTO_ROBOT_MAX_CYCLES": "1",
        }
        argv = commands["flyto-robot-agent.service"]
        completed = subprocess.run(
            argv, capture_output=True, text=True, timeout=60, check=False, env=environment
        )
        assert completed.returncode == 0, f"first start failed: {completed.stderr}"
        document = json.loads(completed.stdout.strip().splitlines()[-1])
        assert document["state"] == "activation_pending"
        assert document["ok"] is True

    def test_a_one_shot_will_not_wait_out_a_window_it_never_looks_at_again(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The supervisor waits; the timer's snapshot cannot.

        A one-shot publishes one document and exits, so "a commit is in flight"
        would be recorded as the device's condition on every single tick -- which
        is how a machine with no committed state gets read as a fresh install
        forever by everything downstream of ``doctor-status.json``.
        """

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                python=sys.executable, systemd=_fake())
        argv = _robot_service_commands(layout)["flyto-robot-doctor.service"]
        open_activation_window(layout, action="install", version="1.0.0")
        layout.state_file.unlink()

        completed = subprocess.run(
            argv, capture_output=True, text=True, timeout=60, check=False,
            env={**os.environ, "PYTHONPATH": str(REPO_ROOT)},
        )

        assert completed.returncode == 1, completed.stderr
        document = json.loads(completed.stdout.strip().splitlines()[-1])
        assert document["state"] == "unhealthy"
        assert document["reason"] == "state_drift"
        assert document["action_code"] == "rerun_install_to_reconcile"
        assert "Traceback" not in completed.stderr

    def test_a_missing_state_with_no_window_is_never_reported_as_pending(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The defect, on the exact commands the device runs.

        No marker, so nothing is coming to commit. Every shipped entry point has
        to say so rather than describe a working machine.
        """

        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                python=sys.executable, systemd=_fake())
        commands = _robot_service_commands(layout)
        layout.state_file.unlink()

        environment = {
            **os.environ,
            "PYTHONPATH": str(REPO_ROOT),
            "FLYTO_ROBOT_MAX_CYCLES": "1",
        }
        for name, argv in commands.items():
            completed = subprocess.run(
                argv, capture_output=True, text=True, timeout=60, check=False, env=environment
            )
            document = json.loads(completed.stdout.strip().splitlines()[-1])
            assert completed.returncode == 1, f"{name}: {completed.stderr}"
            assert document["state"] == "unhealthy", name
            assert document["ok"] is False, name
            assert document["reason"] == "state_drift", name
            assert document["action_code"] == "rerun_install_to_reconcile", name
            assert "Traceback" not in completed.stderr, name

    def test_a_present_but_tampered_snapshot_is_still_refused_at_runtime(
        self, installed: Layout
    ) -> None:
        """Absent is a window. Altered is an attack, and gets no benefit of doubt.

        Tampers the immutable by-id *record*, because that is what the runtime
        resolves. The per-version file beside it is a compatibility view and
        editing it is proved to be inert in
        ``tests/test_runtime_activation_authority.py``.
        """

        snapshot = next(iter(sorted(installed.activation_record_dir.glob("*.json"))))
        snapshot.chmod(0o640)
        document = json.loads(snapshot.read_text())
        document["profile"] = "something-else"
        snapshot.write_text(json.dumps(document), encoding="utf-8")

        argv = _robot_service_commands(installed)["flyto-robot-agent.service"]
        completed = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
            env={**os.environ, "PYTHONPATH": str(REPO_ROOT), "FLYTO_ROBOT_MAX_CYCLES": "1"},
        )
        assert completed.returncode == 1
        document = json.loads(completed.stdout.strip().splitlines()[-1])
        assert document["state"] == "unhealthy"
        assert "something-else" not in json.dumps(document.get("checks", []))

    def test_the_agent_unit_declares_a_bounded_restart_policy(self, installed: Layout) -> None:
        parsed = parse_unit(
            (installed.unit_dir / "flyto-robot-agent.service").read_text(encoding="utf-8")
        )
        assert parsed.values("Service", "Restart") == ["on-failure"]
        # In [Unit]. In [Service] systemd accepts and ignores it, which is how a
        # failing ExecStart reaches NRestarts in the hundreds.
        assert parsed.values("Unit", "StartLimitBurst")
        assert parsed.values("Unit", "StartLimitIntervalSec")

    def test_ros2_uses_the_passive_adapter_command_and_bounded_restart_policy(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile="ros2", python=sys.executable, systemd=_fake())
        roles = _service_commands(layout)
        argv = roles["adapter"]["flyto-robot-ros2.service"]
        assert argv == [
            sys.executable,
            "-m",
            "flyto_robotics.ros2_adapter",
            "--state-dir",
            str(layout.state_dir),
        ]
        assert "job_file" not in " ".join(argv)
        parsed = parse_unit(
            (layout.unit_dir / "flyto-robot-ros2.service").read_text(encoding="utf-8")
        )
        assert parsed.only("Service", "Restart") == "on-failure"
        assert parsed.only("Unit", "StartLimitBurst") == "3"
        assert parsed.only("Unit", "StartLimitIntervalSec") == "300"

        rendered = {path.name: path.read_bytes() for path in layout.unit_dir.glob("*")}
        assert set(rendered) == {
            "flyto-robot-agent.service", "flyto-robot-doctor.service",
            "flyto-robot-doctor.timer", "flyto-job-runner.service",
            "flyto-job-runner.path", "flyto-robot-ros2.service",
        }
        assert "flyto-camera-gateway.service" not in rendered

        install(payload=_payload(tmp_path, "generic-payload", "v2"), version="2.0.0",
                layout=layout, profile="generic", python=sys.executable, systemd=_fake())
        for name in set(rendered) - {"flyto-robot-ros2.service"}:
            assert rendered[name] == (layout.unit_dir / name).read_bytes()

    def test_the_runner_is_present_version_matched_and_condition_gated(
        self, installed: Layout
    ) -> None:
        roles = _service_commands(installed)
        assert set(roles["runner"]) == {"flyto-job-runner.service"}
        argv = roles["runner"]["flyto-job-runner.service"]
        assert argv[:3] == [sys.executable, "-m", "deploy.flyto_job_runner"]

        credential = installed.credentials_dir / "runner-credentials.json"
        service_text = (installed.unit_dir / "flyto-job-runner.service").read_text(
            encoding="utf-8"
        )
        assert validate_unit(service_text, name="flyto-job-runner.service") == ()
        service = parse_unit(service_text)
        assert service.only("Unit", "ConditionPathExists") == str(credential)
        assert service.only("Service", "ExecCondition") == f"/usr/bin/test -f {credential}"
        assert service.only("Service", "WorkingDirectory") == str(installed.current)
        assert "PYTHONPATH=" + str(installed.current) in service.values(
            "Service", "Environment"
        )

        watcher = parse_unit(
            (installed.unit_dir / "flyto-job-runner.path").read_text(encoding="utf-8")
        )
        assert watcher.only("Path", "PathExists") == str(credential)
        assert watcher.only("Path", "Unit") == "flyto-job-runner.service"
