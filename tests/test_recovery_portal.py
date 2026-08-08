from __future__ import annotations

from flyto_robotics.recovery_portal import render_html, report_view
from flyto_robotics.robot_doctor import DiagnosticObservation, build_telemetry


def report():
    return build_telemetry(
        DiagnosticObservation(
            wifi_present=True,
            wifi_operstate="dormant",
            wifi_associated=False,
            wifi_has_address=False,
            default_route=False,
            dns_ready=False,
            cloud_reachable=False,
            cloud_init_status="done",
            service_states={},
            usb_recovery_present=True,
            usb_recovery_has_address=True,
        ),
        resource_id="robot-1",
        sequence=1,
        observed_at="2026-08-08T10:00:00Z",
    )


def test_portal_view_explains_the_stable_reason_without_raw_evidence():
    view = report_view(report())

    assert view["reason_code"] == "wifi_not_associated"
    assert "not associated" in view["summary"]
    assert view["action_codes"] == ["configure_known_wifi", "apply_netplan"]
    assert "payload_hash" not in view


def test_html_escapes_report_fields_and_links_machine_readable_view():
    view = report_view(report())
    view["reason_code"] = "<script>alert(1)</script>"
    rendered = render_html(view)

    assert "<script>" not in rendered
    assert "&lt;script&gt;" in rendered
    assert 'href="/v1/diagnostics"' in rendered
