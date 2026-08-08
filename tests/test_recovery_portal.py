from __future__ import annotations

from datetime import datetime, timedelta, timezone

from flyto_robotics.recovery_portal import (
    STALE_AFTER_SECONDS,
    render_html,
    report_view,
)
from flyto_robotics.robot_doctor import DiagnosticObservation, build_telemetry

# Pinned rather than relative: an age check that reads the wall clock makes
# every assertion below depend on when the suite happens to run.
OBSERVED_AT = datetime(2026, 8, 8, 10, 0, 0, tzinfo=timezone.utc)


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
        observed_at=OBSERVED_AT.isoformat().replace("+00:00", "Z"),
    )


def fresh_view(**kwargs):
    """The snapshot read a moment after it was written."""
    return report_view(report(), now=OBSERVED_AT + timedelta(seconds=1), **kwargs)


def test_portal_view_explains_the_stable_reason_without_raw_evidence():
    view = fresh_view()

    assert view["reason_code"] == "wifi_not_associated"
    assert "not associated" in view["summary"]
    assert view["action_codes"] == ["configure_known_wifi", "apply_netplan"]
    assert "payload_hash" not in view


def test_html_escapes_report_fields_and_links_machine_readable_view():
    view = fresh_view()
    view["reason_code"] = "<script>alert(1)</script>"
    rendered = render_html(view)

    assert "<script>" not in rendered
    assert "&lt;script&gt;" in rendered
    assert 'href="/v1/diagnostics"' in rendered


class TestASnapshotThatIsNoLongerCurrent:
    """A frozen latest.json is the normal consequence of the writer dying —
    which is exactly when somebody opens this page. It used to render
    identically to a live one, with the age left in a timestamp at the foot
    that nobody reads once the headline says the robot is fine."""

    def aged(self, seconds):
        return report_view(report(), now=OBSERVED_AT + timedelta(seconds=seconds))

    def test_a_recent_snapshot_is_not_stale(self):
        """The doctor timer runs every 60s, so a minute old is normal."""
        assert self.aged(60).get("stale") is False

    def test_a_snapshot_just_inside_the_window_is_still_trusted(self):
        assert self.aged(STALE_AFTER_SECONDS - 1).get("stale") is False

    def test_a_snapshot_past_the_window_is_marked_stale(self):
        assert self.aged(STALE_AFTER_SECONDS + 1).get("stale") is True

    def test_the_quality_field_stops_claiming_the_reading_is_good(self):
        assert self.aged(3600)["quality"] == "stale"

    def test_the_summary_says_how_old_it_is_rather_than_what_it_found(self):
        summary = self.aged(3600)["summary"]
        assert "60 minutes old" in summary
        assert "predates" in summary

    def test_the_age_is_reported_so_a_machine_consumer_can_judge_too(self):
        assert self.aged(3600)["age_seconds"] == 3600.0

    def test_an_action_is_offered_for_the_stopped_writer(self):
        assert "inspect_diagnostic_timer" in self.aged(3600)["action_codes"]

    def test_an_unreadable_timestamp_is_treated_as_not_current(self):
        """Not knowing the age is not evidence of freshness."""
        broken = dict(report())
        broken["observed_at"] = "some time last tuesday"
        view = report_view(broken, now=OBSERVED_AT)
        assert view["stale"] is True
        assert view["age_seconds"] is None
        assert "no readable timestamp" in view["summary"]

    def test_the_page_says_so_where_it_cannot_be_missed(self):
        rendered = render_html(self.aged(3600))
        assert "This reading is not current." in rendered

    def test_a_live_page_carries_no_such_banner(self):
        assert "not current" not in render_html(self.aged(30))
