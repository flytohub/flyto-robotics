from __future__ import annotations

import json
from dataclasses import replace

import pytest

from flyto_robotics.robot_doctor import (
    DiagnosticObservation,
    build_telemetry,
    classify_observation,
    diagnostic_payload,
    main,
)


def observation(**changes) -> DiagnosticObservation:
    base = DiagnosticObservation(
        wifi_present=True,
        wifi_operstate="up",
        wifi_associated=True,
        wifi_has_address=True,
        default_route=True,
        dns_ready=True,
        cloud_reachable=True,
        cloud_init_status="done",
        service_states={"flyto-delivery.service": "active"},
        usb_recovery_present=True,
        usb_recovery_has_address=True,
    )
    return replace(base, **changes)


def test_healthy_ethernet_path_does_not_require_wifi_association():
    value = observation(
        wifi_operstate="dormant",
        wifi_associated=False,
        wifi_has_address=False,
    )
    assert classify_observation(value) == ("healthy", "good")


@pytest.mark.parametrize(
    ("changes", "reason"),
    [
        ({"cloud_init_status": "degraded", "cloud_reachable": False}, "provisioning_degraded"),
        ({"wifi_present": False, "cloud_reachable": False}, "wifi_interface_missing"),
        ({"wifi_operstate": "down", "cloud_reachable": False}, "wifi_interface_down"),
        ({"wifi_associated": False, "cloud_reachable": False}, "wifi_not_associated"),
        ({"wifi_has_address": False, "cloud_reachable": False}, "wifi_no_address"),
        ({"default_route": False, "cloud_reachable": False}, "default_route_missing"),
        ({"dns_ready": False, "cloud_reachable": False}, "dns_unavailable"),
        ({"cloud_reachable": None}, "cloud_endpoint_unconfigured"),
        ({"cloud_reachable": False}, "cloud_unreachable"),
        (
            {"service_states": {"flyto-delivery.service": "failed"}},
            "robot_service_unhealthy",
        ),
    ],
)
def test_reason_codes_are_stable_and_ordered(changes, reason):
    assert classify_observation(observation(**changes))[0] == reason


def test_telemetry_is_generic_content_addressed_and_contains_no_network_identity():
    report = build_telemetry(
        observation(cloud_reachable=False),
        resource_id="flyto-tb3-lab-001",
        sequence=7,
        observed_at="2026-08-08T10:00:00Z",
    )
    encoded = json.dumps(report, sort_keys=True).lower()

    assert report["contract"] == "flyto.resource-telemetry.v1"
    assert report["channel_id"] == "system.diagnostics"
    assert report["payload"]["primary_reason_code"] == "cloud_unreachable"
    assert len(report["payload_hash"]) == 64
    assert "ssid" not in encoded.replace('"ssid_included": false', "")
    assert "password" not in encoded
    assert "192.168." not in encoded


def test_payload_exposes_only_action_codes_not_shell_commands():
    payload, _quality = diagnostic_payload(
        observation(wifi_associated=False, cloud_reachable=False)
    )
    assert payload["action_codes"] == ["configure_known_wifi", "apply_netplan"]
    assert all(" " not in item for item in payload["action_codes"])


def test_main_preserves_last_failure_after_recovery(monkeypatch, tmp_path, capsys):
    state = {"value": observation(cloud_reachable=False)}
    monkeypatch.setattr(
        "flyto_robotics.robot_doctor.collect_observation",
        lambda **_kwargs: state["value"],
    )
    latest = tmp_path / "latest.json"
    failure = tmp_path / "last-failure.json"
    args = [
        "--resource-id",
        "robot-1",
        "--output",
        str(latest),
        "--last-failure",
        str(failure),
    ]
    assert main(args) == 0
    failed_snapshot = failure.read_text()

    state["value"] = observation()
    assert main(args) == 0
    assert json.loads(latest.read_text())["quality"] == "good"
    assert failure.read_text() == failed_snapshot
    assert capsys.readouterr().out


class TestAServiceStateThatCouldNotBeRead:
    """`unknown` is the absence of an answer, not a clean bill of health.

    _service_state returns it when systemctl is missing, hangs past the three
    second timeout, or replies with anything unrecognised. It used to be
    filtered out alongside "active", so a robot nobody could inspect reported
    healthy / good / services.healthy true, with no action codes at all.
    """

    UNREADABLE = {"flyto-delivery.service": "unknown"}

    def test_an_unreadable_service_is_not_healthy(self):
        reason, quality = classify_observation(observation(service_states=self.UNREADABLE))
        assert reason == "service_state_unknown"
        assert quality == "degraded"

    def test_every_service_unreadable_is_the_worst_case_and_still_not_healthy(self):
        """systemctl absent: nothing at all could be read."""
        states = dict.fromkeys(
            ("flyto-delivery.service", "turtlebot3-bringup.service"), "unknown"
        )
        assert classify_observation(observation(service_states=states))[0] != "healthy"

    def test_a_known_failure_outranks_an_unread_one(self):
        """One names a fix; the other names an absence. Report the fixable one."""
        states = {
            "flyto-delivery.service": "failed",
            "turtlebot3-bringup.service": "unknown",
        }
        assert (
            classify_observation(observation(service_states=states))[0]
            == "robot_service_unhealthy"
        )

    def test_the_payload_does_not_claim_the_services_are_healthy(self):
        payload, _ = diagnostic_payload(observation(service_states=self.UNREADABLE))
        assert payload["services"]["healthy"] is False

    def test_the_payload_names_the_unread_services_separately(self):
        """Not merged into unhealthy: they may be fine, and saying they failed
        would send an operator chasing a fault that was never observed."""
        payload, _ = diagnostic_payload(observation(service_states=self.UNREADABLE))
        assert payload["services"]["unknown_service_ids"] == ["flyto-delivery.service"]
        assert payload["services"]["unhealthy_service_ids"] == []

    def test_the_reason_carries_actions_to_take(self):
        payload, _ = diagnostic_payload(observation(service_states=self.UNREADABLE))
        assert payload["action_codes"], "a degraded reason with no action is a dead end"
        assert "retry_service_query" in payload["action_codes"]

    def test_a_fully_readable_robot_is_still_plainly_healthy(self):
        """The refusal must not cost a good robot its clean report."""
        payload, _ = diagnostic_payload(observation())
        assert payload["primary_reason_code"] == "healthy"
        assert payload["services"]["healthy"] is True
        assert payload["services"]["unknown_service_ids"] == []
