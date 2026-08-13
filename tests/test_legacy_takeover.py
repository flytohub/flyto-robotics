from __future__ import annotations

import json
from io import StringIO

import pytest

from flyto_robotics import activation_snapshot, lifecycle
from flyto_robotics.legacy_takeover import (
    TakeoverError,
    plan_takeover,
    revalidate_takeover_receipt,
)
from flyto_robotics.lifecycle import LIFECYCLE_STATE_VERSION, Layout, render_units
from flyto_robotics.robot_cli import main


class ReadOnlySystemd:
    def __init__(self, *, active="active", enabled="enabled", race=False):
        self.active = active
        self.enabled = enabled
        self.race = race
        self.calls = 0
        self.mutations = []

    def health(self, names):
        self.calls += 1
        active = "inactive" if self.race and self.calls == 2 else self.active
        return [{"unit": name, "active": active, "enabled": self.enabled} for name in sorted(names)]


def prepared(tmp_path):
    layout = Layout(tmp_path.resolve())
    layout.credentials_dir.mkdir(parents=True)
    layout.config_dir.mkdir(parents=True)
    layout.unit_dir.mkdir(parents=True)
    (layout.root / "etc/machine-id").write_text("0123456789abcdef0123456789abcdef\n")
    credential = layout.credentials_dir / "runner-credentials.json"
    credential.write_text(
        json.dumps({"device_id": "synthetic-device", "device_secret": "synthetic-secret"})
    )
    credential.chmod(0o600)
    layout.identity_file.write_text(json.dumps({"device_id": "synthetic-device"}))
    layout.config_file.write_text(
        "FLYTO_CLOUD_URL=https://synthetic.invalid\nFLYTO_ROBOT_RESOURCE_ID=synthetic-robot\n"
    )
    (layout.unit_dir / "flyto-robot-agent.service").write_text(
        "[Service]\nExecStart=/opt/legacy/bin/agent --token hidden\nEnvironment=SECRET=hidden\n"
    )
    return layout


def invoke(layout, systemd, *extra):
    output = StringIO()
    code = main(
        ["--root", str(layout.root), "plan-takeover", "--profile", "generic", *extra],
        stream=output,
        systemd=systemd,
    )
    return code, json.loads(output.getvalue()), output.getvalue()


def record_activation(layout, profile="generic", python="/usr/bin/python3"):
    spec = lifecycle.profile_for(profile)
    units = render_units(layout, profile=profile, python=python)
    snapshot = activation_snapshot.build(
        version="1.0.0",
        profile=spec,
        python=python,
        release_digest="b" * 64,
        units=units,
    )
    layout.activation_record_dir.mkdir(parents=True, exist_ok=True)
    layout.activation_record_file(snapshot.activation_id).write_text(
        json.dumps(snapshot.document())
    )
    layout.state_file.write_text(
        json.dumps(
            {
                "schema": LIFECYCLE_STATE_VERSION,
                "current": "1.0.0",
                "current_activation": snapshot.activation_id,
                "profile": profile,
                "history": [
                    {
                        "version": "1.0.0",
                        "digest": "b" * 64,
                        "profile": profile,
                        "python": python,
                        "units": lifecycle.unit_digests(units),
                        "activation_id": snapshot.activation_id,
                    }
                ],
            }
        )
    )
    return units, snapshot


def test_acknowledgement_is_required_and_read_only(tmp_path):
    layout = prepared(tmp_path)
    systemd = ReadOnlySystemd()
    code, report, _ = invoke(layout, systemd)
    assert code == 1
    assert report == {
        "schema": "flyto.legacy-takeover-plan.v1",
        "ok": False,
        "reason": "legacy_takeover_not_acknowledged",
    }
    assert systemd.calls == 0
    assert systemd.mutations == []


@pytest.mark.parametrize(
    ("active", "enabled"),
    [("active", "enabled"), ("inactive", "disabled"), ("failed", "static")],
)
def test_receipt_is_deterministic_bounded_and_projects_no_secrets(tmp_path, active, enabled):
    layout = prepared(tmp_path)
    first = plan_takeover(
        layout=layout,
        profile="generic",
        profiles=None,
        systemd=ReadOnlySystemd(active=active, enabled=enabled),
        acknowledged=True,
    )
    second = plan_takeover(
        layout=layout,
        profile="generic",
        profiles=None,
        systemd=ReadOnlySystemd(active=active, enabled=enabled),
        acknowledged=True,
    )
    assert first == second
    assert set(first) == {
        "schema",
        "ok",
        "authority_digest",
        "commissioning_digest",
        "lifecycle_generation",
        "systemd_generation",
        "receipt_digest",
    }
    encoded = json.dumps(first)
    assert "hidden" not in encoded
    assert "synthetic-secret" not in encoded
    assert "Environment" not in encoded


def test_receipt_binds_stable_host_root_and_generation(tmp_path):
    layout = prepared(tmp_path)
    base = plan_takeover(
        layout=layout,
        profile="generic",
        profiles=None,
        systemd=ReadOnlySystemd(),
        acknowledged=True,
    )
    other_layout = prepared(tmp_path / "other-root")
    (other_layout.root / "etc/machine-id").write_text("fedcba9876543210fedcba9876543210\n")
    other_authority = plan_takeover(
        layout=other_layout,
        profile="generic",
        profiles=None,
        systemd=ReadOnlySystemd(),
        acknowledged=True,
    )
    assert base["receipt_digest"] != other_authority["receipt_digest"]
    assert base["authority_digest"] != other_authority["authority_digest"]
    assert "root" not in base and "host_authority" not in base
    other_root = prepared(tmp_path / "same-machine-other-root")
    same_machine = plan_takeover(
        layout=other_root,
        profile="generic",
        profiles=None,
        systemd=ReadOnlySystemd(),
        acknowledged=True,
    )
    assert base["authority_digest"] != same_machine["authority_digest"]
    other_profile = plan_takeover(
        layout=layout,
        profile="ros2",
        profiles=None,
        systemd=ReadOnlySystemd(),
        acknowledged=True,
    )
    assert base["authority_digest"] != other_profile["authority_digest"]
    (layout.unit_dir / "flyto-robot-agent.service").write_text("[Service]\nExecStart=/opt/other\n")
    changed = plan_takeover(
        layout=layout,
        profile="generic",
        profiles=None,
        systemd=ReadOnlySystemd(),
        acknowledged=True,
    )
    assert base["systemd_generation"] != changed["systemd_generation"]
    assert base["receipt_digest"] != changed["receipt_digest"]


def test_missing_invalid_and_symlink_prerequisites_refuse(tmp_path):
    layout = prepared(tmp_path)
    layout.identity_file.unlink()
    with pytest.raises(TakeoverError, match="identity_missing"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )
    layout.identity_file.symlink_to(layout.config_file)
    with pytest.raises(TakeoverError, match="identity_invalid"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )


@pytest.mark.parametrize(
    ("document", "mode"),
    [
        ({"device_id": "device", "device_secret": "secret", "extra": "x"}, 0o600),
        ({"device_id": "bad id", "device_secret": "secret"}, 0o600),
        ({"device_id": "device", "device_secret": ""}, 0o600),
        ({"device_id": "device", "device_secret": "has space"}, 0o600),
        ({"device_id": "device", "device_secret": "secret"}, 0o644),
    ],
)
def test_credential_requires_canonical_paired_schema_and_private_mode(tmp_path, document, mode):
    layout = prepared(tmp_path)
    credential = layout.credentials_dir / "runner-credentials.json"
    credential.write_text(json.dumps(document))
    credential.chmod(mode)
    with pytest.raises(TakeoverError, match="credential_invalid"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )


@pytest.mark.parametrize(
    "document",
    [
        {},
        {"resource_id": "synthetic-device"},
        {"device_id": ""},
        {"device_id": "bad id"},
        {"device_id": "synthetic-device", "extra": True},
        {"device_id": 7},
    ],
)
def test_identity_requires_exact_bounded_device_contract(tmp_path, document):
    layout = prepared(tmp_path)
    layout.identity_file.write_text(json.dumps(document))
    with pytest.raises(TakeoverError, match="identity_invalid"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )


def test_identity_must_match_paired_credential(tmp_path):
    layout = prepared(tmp_path)
    layout.identity_file.write_text(json.dumps({"device_id": "other-device"}))
    with pytest.raises(TakeoverError, match="identity_invalid"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )


@pytest.mark.parametrize(
    "content",
    [
        "FLYTO_CLOUD_URL=notaurl\nFLYTO_ROBOT_RESOURCE_ID=robot\n",
        "FLYTO_CLOUD_URL=https://user:pass@example.invalid\nFLYTO_ROBOT_RESOURCE_ID=robot\n",
        "FLYTO_CLOUD_URL=https://example.invalid/path\nFLYTO_ROBOT_RESOURCE_ID=robot\n",
        "FLYTO_CLOUD_URL=https://example.invalid\nFLYTO_ROBOT_RESOURCE_ID=bad id\n",
        "FLYTO_CLOUD_URL=https://example.invalid\nFLYTO_ROBOT_RESOURCE_ID=robot\nEXTRA=value\n",
        "FLYTO_CLOUD_URL=https://one.invalid\nFLYTO_CLOUD_URL=https://two.invalid\nFLYTO_ROBOT_RESOURCE_ID=robot\n",
        "export FLYTO_CLOUD_URL=https://example.invalid\nFLYTO_ROBOT_RESOURCE_ID=robot\n",
    ],
)
def test_robot_env_requires_exact_commissioning_contract(tmp_path, content):
    layout = prepared(tmp_path)
    layout.config_file.write_text(content)
    with pytest.raises(TakeoverError, match="config_invalid"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )


def test_malformed_oversized_unit_and_observation_race_refuse(tmp_path):
    layout = prepared(tmp_path)
    unit = layout.unit_dir / "flyto-robot-agent.service"
    unit.write_text("not-a-unit\n")
    with pytest.raises(TakeoverError, match="unit_malformed"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )
    unit.write_bytes(b"x" * (128 * 1024 + 1))
    with pytest.raises(TakeoverError, match="unit_invalid"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )
    unit.write_text("[Service]\nExecStart=/opt/legacy/agent\n")
    with pytest.raises(TakeoverError, match="snapshot_race"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(race=True),
            acknowledged=True,
        )


def test_hostile_systemd_error_maps_to_stable_code(tmp_path):
    layout = prepared(tmp_path)

    class Hostile:
        def health(self, _names):
            raise RuntimeError("secret host detail")

    code, report, encoded = invoke(layout, Hostile(), "--acknowledge-legacy-takeover")
    assert code == 1
    assert report["reason"] == "systemd_observation_failed"
    assert "secret host detail" not in encoded


def test_no_foreign_target_refuses_without_systemd_mutation(tmp_path):
    layout = prepared(tmp_path)
    (layout.unit_dir / "flyto-robot-agent.service").unlink()
    systemd = ReadOnlySystemd()
    with pytest.raises(TakeoverError, match="no_legacy_units"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=systemd,
            acknowledged=True,
        )
    assert systemd.calls == 2
    assert systemd.mutations == []


def test_exact_committed_units_are_already_managed(tmp_path):
    layout = prepared(tmp_path)
    units, _snapshot = record_activation(layout)
    for name, text in units.items():
        (layout.unit_dir / name).write_text(text)
    with pytest.raises(TakeoverError, match="already_managed"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )


def test_complete_snapshot_rejects_unit_change_at_health_boundary(tmp_path):
    layout = prepared(tmp_path)
    unit = layout.unit_dir / "flyto-robot-agent.service"

    class BoundaryRace(ReadOnlySystemd):
        def health(self, names):
            result = super().health(names)
            if self.calls == 1:
                unit.write_text("[Service]\nExecStart=/opt/replaced/agent\n")
            return result

    systemd = BoundaryRace()
    with pytest.raises(TakeoverError, match="snapshot_race"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=systemd,
            acknowledged=True,
        )
    assert systemd.mutations == []


def test_duplicate_critical_directive_and_symlink_parent_refuse(tmp_path):
    layout = prepared(tmp_path)
    unit = layout.unit_dir / "flyto-robot-agent.service"
    unit.write_text("[Service]\nExecStart=/opt/one\nExecStart=/opt/two\n")
    with pytest.raises(TakeoverError, match="unit_duplicate_directive"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )
    unit.unlink()
    layout.unit_dir.rmdir()
    outside = layout.root / "outside-units"
    outside.mkdir()
    (outside / "flyto-robot-agent.service").write_text("[Service]\nExecStart=/opt/outside/agent\n")
    layout.unit_dir.symlink_to(outside, target_is_directory=True)
    with pytest.raises(TakeoverError, match="unit_symlink"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )


@pytest.mark.parametrize("record_case", ["missing", "tampered", "forged"])
def test_unvalidated_activation_record_never_grants_ownership(tmp_path, record_case):
    layout = prepared(tmp_path)
    units, snapshot = record_activation(layout)
    for name, text in units.items():
        (layout.unit_dir / name).write_text(text)
    record = layout.activation_record_file(snapshot.activation_id)
    if record_case == "missing":
        record.unlink()
    elif record_case == "tampered":
        document = json.loads(record.read_text())
        document["units"]["flyto-robot-agent.service"] += "# tampered\n"
        record.write_text(json.dumps(document))
    else:
        record.unlink()
        state = json.loads(layout.state_file.read_text())
        state["current_activation"] = "a" * 64
        state["history"][-1]["activation_id"] = "a" * 64
        layout.state_file.write_text(json.dumps(state))
    receipt = plan_takeover(
        layout=layout,
        profile="generic",
        profiles=None,
        systemd=ReadOnlySystemd(),
        acknowledged=True,
    )
    assert receipt["ok"] is True
    assert not any(name in json.dumps(receipt) for name in units)


def test_mixed_profile_receipts_only_foreign_additive_units(tmp_path):
    layout = prepared(tmp_path)
    managed, _snapshot = record_activation(layout, profile="generic")
    ros2 = render_units(layout, profile="ros2", python="/usr/bin/python3")
    additive = sorted(set(ros2) - set(managed))
    assert additive
    for name, text in managed.items():
        (layout.unit_dir / name).write_text(text)
    for name in additive:
        (layout.unit_dir / name).write_text(ros2[name])
    receipt = plan_takeover(
        layout=layout,
        profile="ros2",
        profiles=None,
        systemd=ReadOnlySystemd(),
        acknowledged=True,
    )
    assert receipt["ok"] is True
    assert not any(name in json.dumps(receipt) for name in additive)


def test_receipt_revalidation_and_tamper_or_generation_refusal(tmp_path):
    layout = prepared(tmp_path)
    receipt = plan_takeover(
        layout=layout,
        profile="generic",
        profiles=None,
        systemd=ReadOnlySystemd(),
        acknowledged=True,
    )
    valid = revalidate_takeover_receipt(
        receipt=receipt,
        layout=layout,
        profile="generic",
        profiles=None,
        systemd=ReadOnlySystemd(),
    )
    assert valid == {
        "schema": "flyto.legacy-takeover-revalidation.v1",
        "ok": True,
        "reason": "valid",
    }
    tampered = {**receipt, "extra": True}
    assert revalidate_takeover_receipt(
        receipt=tampered,
        layout=layout,
        profile="generic",
        profiles=None,
        systemd=ReadOnlySystemd(),
    )["reason"] == "receipt_invalid"
    layout.config_file.write_text(
        "FLYTO_CLOUD_URL=https://replacement.invalid\n"
        "FLYTO_ROBOT_RESOURCE_ID=synthetic-robot\n"
    )
    changed = revalidate_takeover_receipt(
        receipt=receipt,
        layout=layout,
        profile="generic",
        profiles=None,
        systemd=ReadOnlySystemd(),
    )
    assert changed["reason"] == "commissioning_changed"


def test_authority_change_across_snapshots_refuses(tmp_path):
    layout = prepared(tmp_path)

    class AuthorityRace(ReadOnlySystemd):
        def health(self, names):
            result = super().health(names)
            if self.calls == 1:
                (layout.root / "etc/machine-id").write_text("fedcba9876543210fedcba9876543210\n")
            return result

    systemd = AuthorityRace()
    with pytest.raises(TakeoverError, match="snapshot_race"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=systemd,
            acknowledged=True,
        )
    assert systemd.mutations == []


def test_source_path_existing_symlink_parent_refuses(tmp_path):
    layout = prepared(tmp_path)
    outside = layout.root / "outside-source"
    outside.mkdir()
    (layout.root / "opt").symlink_to(outside, target_is_directory=True)
    with pytest.raises(TakeoverError, match="unit_symlink"):
        plan_takeover(
            layout=layout,
            profile="generic",
            profiles=None,
            systemd=ReadOnlySystemd(),
            acknowledged=True,
        )
