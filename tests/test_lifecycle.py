"""Install, update, roll back, and support -- against a temporary root.

Every test here runs as an ordinary user with no systemd, no ROS, no network,
and no hardware. `Layout(root=tmp_path)` moves the entire product tree into a
temporary directory, which is the only reason these invariants can be asserted
at all: a lifecycle you can only exercise on a real robot is a lifecycle nobody
tests until it fails in the field.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from flyto_robotics import lifecycle
from flyto_robotics.lifecycle import Layout, LifecycleError, install, rollback, status, update
from flyto_robotics.lifecycle_profiles import ProfileError, default_profiles_path, load_profiles
from flyto_robotics.support_bundle import build_support_bundle, redact
from flyto_robotics.systemd_units import parse_unit, validate_unit


@pytest.fixture()
def layout(tmp_path: Path) -> Layout:
    return Layout(root=tmp_path.resolve())


def _payload(tmp_path: Path, name: str, body: str) -> Path:
    """Build (or rebuild) the named payload deterministically.

    Asking for the same payload twice inside one test must yield the same bytes
    rather than raising ``FileExistsError`` out of the helper -- otherwise every
    "run it again and prove nothing changed" assertion is unwritable.
    """

    root = tmp_path / "payloads" / name
    package = root / "flyto_robotics"
    package.mkdir(parents=True, exist_ok=True)
    package.joinpath("__init__.py").write_text(body, encoding="utf-8")
    return root


def _active(layout: Layout) -> str:
    return Path(os.readlink(layout.current)).name


class TestInstall:
    def test_a_first_install_activates_the_release_and_reports_json(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        report = install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        assert report["ok"] is True
        assert report["reason"] == "ok"
        assert report["action_code"] == "none"
        assert report["schema"] == "flyto.lifecycle-report.v1"
        assert _active(layout) == "1.0.0"
        # The report must survive a round trip: it is what a fleet tool parses.
        assert json.loads(json.dumps(report))["version"] == "1.0.0"

    @pytest.mark.skipif(os.name != "posix", reason="file modes are POSIX-only here")
    def test_the_lifecycle_lock_is_tightened_on_the_open_inode(self, layout: Layout) -> None:
        layout.state_dir.mkdir(parents=True, mode=0o750)
        layout.lock_file.write_text("", encoding="utf-8")
        layout.lock_file.chmod(0o666)
        with lifecycle._advisory_lock(layout):
            assert layout.lock_file.stat().st_mode & 0o777 == 0o600

    def test_running_the_same_install_twice_changes_nothing(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        payload = _payload(tmp_path, "a", "v1")
        install(payload=payload, version="1.0.0", layout=layout)
        second = install(payload=payload, version="1.0.0", layout=layout)
        assert second["changed"] == []
        assert second["reason"] == "no_change"

    def test_a_dry_run_writes_absolutely_nothing(self, layout: Layout, tmp_path: Path) -> None:
        report = install(
            payload=_payload(tmp_path, "a", "v1"),
            version="1.0.0",
            layout=layout,
            dry_run=True,
        )
        assert report["reason"] == "dry_run"
        assert report["changed"], "a dry run must still say what it would do"
        # Not one path from the layout may exist afterwards.
        for path in (layout.prefix, layout.config_dir, layout.state_dir, layout.log_dir):
            assert not path.exists(), f"dry run created {path}"

    def test_a_release_directory_is_immutable_once_published(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        with pytest.raises(LifecycleError) as raised:
            install(payload=_payload(tmp_path, "b", "TAMPERED"), version="1.0.0", layout=layout)
        assert raised.value.reason == "release_exists_with_different_content"
        assert (layout.release("1.0.0") / "flyto_robotics/__init__.py").read_text() == "v1"

    def test_the_rendered_units_are_valid_and_name_no_home_directory(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        names = sorted(p.name for p in layout.unit_dir.glob("flyto-*"))
        assert names == [
            "flyto-job-runner.path",
            "flyto-job-runner.service",
            "flyto-robot-agent.service",
            "flyto-robot-doctor.service",
            "flyto-robot-doctor.timer",
        ]
        for name in names:
            text = (layout.unit_dir / name).read_text()
            assert validate_unit(text, name=name) == (), f"{name} would not behave as written"
            assert "/home/" not in text
            # The unit points at the stable symlink, never at a version: a unit
            # naming a version would have to be rewritten on every update, and a
            # rollback would leave it aimed at an inactive release.
            assert str(layout.releases) not in text

    def test_camera_and_ros2_profiles_are_purely_additive(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        generic = lifecycle.render_units(layout, profile="generic")
        camera_units = lifecycle.render_units(layout, profile="camera")
        ros2 = lifecycle.render_units(layout, profile="ros2")
        assert set(generic) < set(camera_units)
        assert set(generic) < set(ros2)
        for name, text in generic.items():
            assert camera_units[name] == text, f"{name} differs between profiles"
            assert ros2[name] == text, f"{name} differs between profiles"
        assert set(camera_units) - set(generic) == {"flyto-camera-gateway.service"}
        assert set(ros2) - set(generic) == {"flyto-robot-ros2.service"}
        assert "flyto-camera-gateway.service" not in ros2
        for adapter in set(ros2) - set(generic):
            assert validate_unit(ros2[adapter], name=adapter) == ()
        camera = parse_unit(camera_units["flyto-camera-gateway.service"])
        assert camera.only("Service", "NoNewPrivileges") == "yes"
        assert camera.only("Service", "PrivateTmp") == "yes"
        assert camera.only("Service", "ProtectSystem") == "strict"
        assert camera.only("Service", "ProtectHome") == "yes"
        assert camera.only("Service", "ProtectControlGroups") == "yes"
        assert camera.only("Service", "ProtectKernelModules") == "yes"
        assert camera.only("Service", "ProtectKernelTunables") == "yes"
        assert camera.only("Service", "RestrictAddressFamilies") == (
            "AF_INET AF_INET6 AF_NETLINK AF_UNIX"
        )
        assert camera.only("Service", "RestrictSUIDSGID") == "yes"
        assert camera.only("Service", "LockPersonality") == "yes"
        assert camera.only("Service", "MemoryDenyWriteExecute") == "yes"
        assert camera.only("Service", "Restart") == "on-failure"
        assert camera.only("Service", "TimeoutStopSec") == "20"
        assert camera.values("Service", "EnvironmentFile") == [
            str(layout.config_file), f"-{layout.config_dir}/camera.env"
        ]
        assert "{current}" not in camera.only("Service", "ExecStart")
        adapter = parse_unit(ros2["flyto-robot-ros2.service"])
        assert adapter.only("Service", "PrivateTmp") == "yes"
        assert adapter.only("Service", "ProtectSystem") == "strict"
        assert adapter.only("Service", "ReadWritePaths") == str(layout.state_dir)
        assert adapter.only("Service", "RestrictAddressFamilies") == (
            "AF_INET AF_INET6 AF_NETLINK AF_UNIX"
        )

    def test_shipped_profile_chain_resolves_and_dangling_custom_extends_is_refused(
        self, tmp_path: Path
    ) -> None:
        profiles = load_profiles()
        generic_names = tuple(unit.name for unit in profiles["generic"].units)
        camera_names = tuple(unit.name for unit in profiles["camera"].units)
        ros2_names = tuple(unit.name for unit in profiles["ros2"].units)

        assert camera_names == (*generic_names, "flyto-camera-gateway.service")
        assert ros2_names == (*generic_names, "flyto-robot-ros2.service")
        assert profiles["camera"].readiness == profiles["generic"].readiness
        assert profiles["ros2"].readiness == profiles["generic"].readiness

        registry = json.loads(default_profiles_path().read_text(encoding="utf-8"))
        registry["profiles"] = {
            "site": {"extends": "missing", "description": "synthetic dangling profile"}
        }
        dangling = tmp_path / "dangling-profiles.json"
        dangling.write_text(json.dumps(registry), encoding="utf-8")

        with pytest.raises(ProfileError, match="unknown profile 'missing'"):
            load_profiles(dangling)


class TestUpdate:
    def test_a_successful_update_moves_current_and_keeps_the_old_release(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        report = update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout)
        assert report["action"] == "update"
        assert _active(layout) == "2.0.0"
        assert layout.release("1.0.0").is_dir(), "the rollback target must survive"

    def test_an_update_with_invalid_units_leaves_the_last_good_release_running(
        self, layout: Layout, tmp_path: Path, monkeypatch
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)

        def broken(layout_arg, **kwargs):
            # StartLimitBurst in [Service] is exactly the defect that silently
            # restores an unbounded restart loop. It must never be activated.
            return {
                "flyto-robot-agent.service": (
                    "[Unit]\nDescription=x\n"
                    "[Service]\nExecStart=/bin/true\nRestart=always\nStartLimitBurst=3\n"
                )
            }

        monkeypatch.setattr(lifecycle, "render_units", broken)
        report = update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout)

        assert report["ok"] is False
        assert report["reason"] == "unit_validation_failed"
        assert report["action_code"] == "inspect_report_defects"
        assert {d["code"] for d in report["defects"]} >= {"start_limit_wrong_section"}
        assert _active(layout) == "1.0.0", "a failed update must not move current"
        assert json.loads(layout.state_file.read_text())["current"] == "1.0.0"

    def test_an_update_that_fails_its_health_check_reverts_itself(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        report = update(
            payload=_payload(tmp_path, "b", "v2"),
            version="2.0.0",
            layout=layout,
            health_check=lambda _current: False,
        )
        assert report["reason"] == "post_switch_health_failed"
        assert report["action_code"] == "collect_support_bundle"
        assert _active(layout) == "1.0.0"


class TestRollback:
    def test_rollback_returns_to_the_previous_activated_release(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout)
        report = rollback(layout=layout)
        assert report["ok"] is True
        assert report["version"] == "1.0.0"
        assert _active(layout) == "1.0.0"
        assert (layout.current / "flyto_robotics/__init__.py").read_text() == "v1"

    def test_a_dry_run_rollback_does_not_move_current(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout)
        rollback(layout=layout, dry_run=True)
        assert _active(layout) == "2.0.0"

    def test_rollback_refuses_when_there_is_nothing_to_return_to(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        with pytest.raises(LifecycleError) as raised:
            rollback(layout=layout)
        assert raised.value.reason == "no_rollback_target"

    def test_rollback_refuses_a_target_whose_bytes_no_longer_match(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout)
        tampered = layout.release("1.0.0") / "flyto_robotics/__init__.py"
        tampered.chmod(0o644)
        tampered.write_text("not what was activated", encoding="utf-8")
        with pytest.raises(LifecycleError) as raised:
            rollback(layout=layout)
        assert raised.value.reason == "release_payload_invalid"


class TestPersistence:
    def test_config_identity_credentials_and_journals_survive_every_release_move(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)

        layout.config_file.write_text("FLYTO_ROBOT_RESOURCE_ID=site-owned\n", encoding="utf-8")
        layout.identity_file.write_text('{"device_id": "d-1"}', encoding="utf-8")
        (layout.credentials_dir / "device.cred").write_text("opaque", encoding="utf-8")
        (layout.log_dir / "runner.jsonl").write_text("line\n", encoding="utf-8")
        (layout.diagnostics_dir / "latest.json").write_text("{}", encoding="utf-8")

        update(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout)
        rollback(layout=layout)

        assert layout.config_file.read_text() == "FLYTO_ROBOT_RESOURCE_ID=site-owned\n"
        assert layout.identity_file.read_text() == '{"device_id": "d-1"}'
        assert (layout.credentials_dir / "device.cred").read_text() == "opaque"
        assert (layout.log_dir / "runner.jsonl").read_text() == "line\n"
        assert (layout.diagnostics_dir / "latest.json").read_text() == "{}"

    def test_no_persistent_path_lives_inside_a_release(self, layout: Layout) -> None:
        for path in layout.persistent_paths():
            assert layout.releases not in path.parents
            assert path != layout.releases


class TestStatus:
    def test_status_reports_not_installed_before_any_install(self, layout: Layout) -> None:
        report = status(layout)
        assert report["ok"] is False
        assert report["reason"] == "not_installed"
        assert report["action_code"] == "run_install"

    def test_status_reports_a_missing_identity_after_install(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        report = status(layout)
        assert report["reason"] == "identity_missing"
        assert report["action_code"] == "provision_identity"
        assert report["installed_releases"] == ["1.0.0"]


class TestCli:
    def test_the_cli_emits_one_json_object_and_a_meaningful_exit_code(
        self, tmp_path: Path, capsys
    ) -> None:
        payload = _payload(tmp_path, "a", "v1")
        code = lifecycle.main(
            [
                "--root",
                str(tmp_path / "root"),
                "install",
                "--payload",
                str(payload),
                "--version",
                "1.0.0",
            ]
        )
        assert code == 0
        assert json.loads(capsys.readouterr().out)["reason"] == "ok"

    def test_the_cli_reports_a_refusal_as_a_reason_code_not_a_traceback(
        self, tmp_path: Path, capsys
    ) -> None:
        code = lifecycle.main(["--root", str(tmp_path / "root"), "rollback"])
        assert code == 1
        report = json.loads(capsys.readouterr().out)
        assert report["reason"] == "not_installed"
        assert report["action_code"] == "run_install"


class TestSupportBundle:
    def test_the_bundle_is_byte_identical_across_runs(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        first = build_support_bundle(layout, now="2026-08-10T00:00:00Z")
        second = build_support_bundle(layout, now="2026-08-10T00:00:00Z")
        assert json.dumps(first, sort_keys=True) == json.dumps(second, sort_keys=True)

    def test_the_bundle_lists_credential_files_without_their_contents(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        (layout.credentials_dir / "device.cred").write_text("SUPERSECRET", encoding="utf-8")
        text = json.dumps(build_support_bundle(layout, now="2026-08-10T00:00:00Z"))
        assert "device.cred" in text
        assert "SUPERSECRET" not in text

    def test_redaction_covers_keys_and_value_shapes_alike(self) -> None:
        result = redact(
            {
                "device_secret": "abc",
                "note": "joined ssid HomeNet at 192.168.1.44 via aa:bb:cc:dd:ee:ff",
                "path": "/home/ubuntu/flyto-robotics",
                "url": "https://user:pw@cloud.example",
                "contact": "ops@example.com",
                "count": 3,
            }
        )
        assert result["device_secret"] == "[redacted]"
        assert "192.168.1.44" not in result["note"]
        assert "aa:bb:cc:dd:ee:ff" not in result["note"]
        assert result["path"] == "/home/[user]/flyto-robotics"
        assert "user:pw" not in result["url"]
        assert result["contact"] == "[email]"
        assert result["count"] == 3

    def test_the_bundle_carries_unit_validation_verdicts(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout)
        bundle = build_support_bundle(layout, now="2026-08-10T00:00:00Z")
        assert bundle["units"]
        assert all(unit["ok"] for unit in bundle["units"])
        assert bundle["lifecycle"]["active_release"] == "1.0.0"
