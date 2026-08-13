from __future__ import annotations

import json
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path

import pytest

from flyto_robotics.device_events import (
    DEVICE_EVENT_CONTRACT,
    SEQUENCE_LIMIT,
    DeviceEventJournal,
    event_sequence,
)
from flyto_robotics.robot_doctor import (
    EXIT_EVENT_NOT_RECORDED,
    SERVICE_NAMES,
    DiagnosticObservation,
    _commit_recovery_state,
    _record_degradation,
    build_telemetry,
    classify_observation,
    collect_service_recoveries,
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
    tmp_path = tmp_path.resolve()
    latest = tmp_path / "latest.json"
    failure = tmp_path / "last-failure.json"
    args = [
        "--resource-id",
        "robot-1",
        "--output",
        str(latest),
        "--last-failure",
        str(failure),
        "--event-journal",
        str(tmp_path / "events" / "device-events.jsonl"),
    ]
    assert main(args) == 0
    failed_snapshot = failure.read_text()

    state["value"] = observation()
    assert main(args) == 0
    assert json.loads(latest.read_text())["quality"] == "good"
    assert failure.read_text() == failed_snapshot
    assert capsys.readouterr().out


class TestWhatTheDoctorRecordsForSomeoneElseToRead:
    """The journal half, read back through the real DeviceEventJournal.

    Every path is ``.resolve()``d: on macOS ``tempfile`` hands back
    ``/var/folders/...`` and ``/var`` is a symlink to ``/private/var``, which the
    journal correctly refuses to write through.
    """

    def run(self, monkeypatch, tmp_path, value, *, resource_id="robot-1"):
        monkeypatch.setattr(
            "flyto_robotics.robot_doctor.collect_observation",
            lambda **_kwargs: value,
        )
        root = tmp_path.resolve()
        journal = root / "events" / "device-events.jsonl"
        code = main(
            [
                "--resource-id",
                resource_id,
                "--output",
                str(root / "latest.json"),
                "--last-failure",
                str(root / "last-failure.json"),
                "--event-journal",
                str(journal),
            ]
        )
        return code, journal, json.loads((root / "latest.json").read_text())

    def records(self, journal: Path) -> list[dict]:
        return DeviceEventJournal(journal).read_all()

    def test_a_degraded_snapshot_appends_exactly_one_event(self, monkeypatch, tmp_path):
        code, journal, _report = self.run(
            monkeypatch, tmp_path, observation(cloud_reachable=False)
        )
        assert code == 0
        records = self.records(journal)
        assert len(records) == 1
        event = records[0]["event"]
        assert event["contract"] == DEVICE_EVENT_CONTRACT
        assert event["status"] == "unavailable"
        assert event["reason_code"] == "cloud_unreachable"

    def test_the_event_is_linked_to_the_snapshot_by_its_payload_hash(self, monkeypatch, tmp_path):
        """Not by timestamp arithmetic, which needs two clocks to agree."""
        _code, journal, report = self.run(monkeypatch, tmp_path, observation(cloud_reachable=False))
        event = self.records(journal)[0]["event"]
        assert event["correlation_id"] == report["payload_hash"]
        assert event["details"]["telemetry"]["payload_hash"] == report["payload_hash"]
        # A periodic health check belongs to no run, and inventing one would
        # make unrelated snapshots look like a sequence.
        assert event["run_id"] == ""

    def test_a_degraded_service_snapshot_is_recorded_too(self, monkeypatch, tmp_path):
        code, journal, _report = self.run(
            monkeypatch,
            tmp_path,
            observation(service_states={"flyto-delivery.service": "failed"}),
        )
        assert code == 0
        event = self.records(journal)[0]["event"]
        assert (event["status"], event["severity"]) == ("degraded", "warning")
        assert event["reason_code"] == "robot_service_unhealthy"
        assert event["action_codes"] == ["inspect_service_journal", "restart_failed_service"]

    def test_a_healthy_snapshot_records_nothing_at_all(self, monkeypatch, tmp_path):
        """A stream padded with "still fine" is a stream nobody reads."""
        code, journal, report = self.run(monkeypatch, tmp_path, observation())
        assert code == 0
        assert report["quality"] == "good"
        assert not journal.exists()

    def test_a_second_degraded_run_appends_one_more_not_two(self, monkeypatch, tmp_path):
        root = tmp_path.resolve()
        for _ in range(2):
            self.run(monkeypatch, root, observation(cloud_reachable=False))
        assert len(self.records(root / "events" / "device-events.jsonl")) == 2

    def test_the_journal_is_owner_only_on_disk(self, monkeypatch, tmp_path):
        _code, journal, _report = self.run(
            monkeypatch, tmp_path, observation(cloud_reachable=False)
        )
        assert journal.stat().st_mode & 0o077 == 0
        assert journal.parent.stat().st_mode & 0o077 == 0

    def test_the_sequence_is_inside_the_contract_bound_and_is_not_nanoseconds(
        self, monkeypatch, tmp_path
    ):
        """time.time_ns() is the obvious thing to reach for and it is refused:
        an epoch nanosecond count passed 2**53 in 1970."""
        _code, journal, report = self.run(monkeypatch, tmp_path, observation(cloud_reachable=False))
        sequence = self.records(journal)[0]["event"]["sequence"]
        assert sequence > 0
        assert sequence <= SEQUENCE_LIMIT
        # Microseconds, so roughly 1.7e15 in 2026 — three orders of magnitude
        # below a nanosecond count, which would be over the bound.
        assert sequence < 2**53
        assert sequence * 1000 > SEQUENCE_LIMIT, "nanoseconds would not have fit"
        # The telemetry and the event were stamped from one instant.
        assert report["sequence"] == sequence

    def test_the_timestamp_is_the_one_dialect_the_contract_accepts(self, monkeypatch, tmp_path):
        _code, journal, report = self.run(monkeypatch, tmp_path, observation(cloud_reachable=False))
        observed = self.records(journal)[0]["event"]["observed_at"]
        assert observed.endswith("Z") and "+" not in observed
        assert report["observed_at"] == observed

    def test_the_event_carries_no_identity_no_secret_and_no_free_log(
        self, monkeypatch, tmp_path
    ):
        _code, journal, _report = self.run(
            monkeypatch, tmp_path, observation(cloud_reachable=False)
        )
        event = self.records(journal)[0]["event"]
        encoded = json.dumps(event, sort_keys=True).lower()
        for forbidden in ("ssid", "password", "secret", "token", "192.168.", "10.77.", "http://"):
            assert forbidden not in encoded
        assert event["redaction"] == {
            "policy": "flyto.device-event.redaction.v1",
            "free_text": True,
            "raw_logs_included": False,
            "credentials_included": False,
            "personal_data_included": False,
            "redacted_key_count": 0,
        }
        # Fixed sentences, not a formatted string built from what was observed.
        assert event["message"] == "The configured Cloud endpoint is unreachable."

    def test_a_journal_that_cannot_be_written_is_loud_and_nonzero(
        self, monkeypatch, tmp_path, capsys
    ):
        """The diagnosis still has to reach the operator first. A doctor that
        exits before printing because it could not file the paperwork has
        withheld the one thing it was run for."""
        root = tmp_path.resolve()
        monkeypatch.setattr(
            "flyto_robotics.robot_doctor.collect_observation",
            lambda **_kwargs: observation(cloud_reachable=False),
        )
        latest = root / "latest.json"
        failure = root / "last-failure.json"
        # A journal path whose parent is a regular file: the directory cannot be
        # created or opened, and no amount of retrying will change that.
        blocked = root / "wall"
        blocked.write_text("not a directory")

        code = main(
            [
                "--resource-id",
                "robot-1",
                "--output",
                str(latest),
                "--last-failure",
                str(failure),
                "--event-journal",
                str(blocked / "device-events.jsonl"),
            ]
        )
        captured = capsys.readouterr()
        assert code == EXIT_EVENT_NOT_RECORDED
        assert code != 0
        assert json.loads(captured.out)["quality"] == "error"
        assert latest.exists() and failure.exists()
        assert "could not be recorded" in captured.err


def test_the_sequence_never_repeats_or_goes_backwards_within_a_process():
    """A wall clock can be stepped backwards by NTP or by an operator, and two
    events that compare equal cannot be ordered by a reader at all."""
    from datetime import datetime, timezone

    fixed = datetime(2026, 8, 10, 12, 0, 0, tzinfo=timezone.utc)
    issued = [event_sequence(fixed) for _ in range(5)]
    assert issued == sorted(issued)
    assert len(set(issued)) == 5
    # Stepping the clock back does not reissue a value already handed out.
    assert event_sequence(datetime(2020, 1, 1, tzinfo=timezone.utc)) > issued[-1]


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


class TestBoundedServiceRecoveryDiagnostics:
    def systemd(self, monkeypatch, *, restarts: int, watchdogs: int, message: str = ""):
        def fake_run(command, timeout=3.0):
            assert timeout == 3.0
            if command[:2] == ["systemctl", "show"]:
                return str(restarts)
            if command[0] == "journalctl":
                assert "--boot=0" in command
                assert "--lines=128" in command
                assert not any(value.startswith("--unit=") for value in command)
                unit = next(value for value in command if value.startswith("UNIT="))
                unit = unit.removeprefix("UNIT=")
                record = {"UNIT": unit, "UNIT_RESULT": "watchdog", "MESSAGE": message}
                records = [record] * watchdogs
                if not records:
                    records = [{"UNIT": unit, "UNIT_RESULT": "success", "MESSAGE": message}]
                return "\n".join(json.dumps(record) for record in records)
            raise AssertionError(command)

        monkeypatch.setattr("flyto_robotics.robot_doctor._run", fake_run)

    def test_zero_to_one_watchdog_transition_is_new(self, monkeypatch, tmp_path):
        self.systemd(monkeypatch, restarts=1, watchdogs=1, message="arbitrary localized text")
        values = collect_service_recoveries(tmp_path / "missing.json")
        assert all(item["restart_count"] == 1 for item in values.values())
        assert all(item["watchdog_count"] == 1 for item in values.values())
        assert all(item["new_count"] == 1 for item in values.values())
        assert all(
            item["current_recovery_kind"] == "watchdog_timeout"
            for item in values.values()
        )

    def test_same_watchdog_is_deduped_and_non_watchdog_restart_is_not_mislabelled(
        self, monkeypatch, tmp_path
    ):
        state = tmp_path / "state.json"
        state.write_text(json.dumps({
            "contract": "flyto.service-recovery-state.v1",
            "boot_id": "",
            "restart_counts": dict.fromkeys(SERVICE_NAMES, 1),
            "watchdog_counts": dict.fromkeys(SERVICE_NAMES, 1),
        }))
        self.systemd(monkeypatch, restarts=1, watchdogs=1, message="watchdog timeout words ignored")
        assert all(item["new_count"] == 0 for item in collect_service_recoveries(state).values())

        self.systemd(monkeypatch, restarts=2, watchdogs=1, message="watchdog timeout words ignored")
        later = collect_service_recoveries(state)
        assert all(item["restart_count"] == 2 for item in later.values())
        assert all(item["new_count"] == 0 for item in later.values())
        assert all(item["current_recovery_kind"] is None for item in later.values())
        payload, _quality = diagnostic_payload(observation(service_recoveries=later))
        restarts = payload["recovery"]["service_restarts"]
        assert payload["primary_reason_code"] == "healthy"
        assert restarts["current_boot_watchdog_total"] == len(SERVICE_NAMES)
        assert restarts["current_recovery_kind"] is None

        self.systemd(monkeypatch, restarts=3, watchdogs=2, message="unrelated localized text")
        second = collect_service_recoveries(state)
        assert all(item["watchdog_count"] == 2 for item in second.values())
        assert all(item["new_count"] == 1 for item in second.values())

    def test_message_text_never_changes_classification(self, monkeypatch, tmp_path):
        self.systemd(monkeypatch, restarts=1, watchdogs=0, message="WATCHDOG timeout failed")
        values = collect_service_recoveries(tmp_path / "missing.json")
        assert all(item["new_count"] == 0 for item in values.values())
        assert all(item["current_recovery_kind"] is None for item in values.values())

    def test_corrupt_state_is_explicitly_unknown_and_queries_nothing(
        self, monkeypatch, tmp_path
    ):
        state = tmp_path / "state.json"
        state.write_text("not json")
        monkeypatch.setattr(
            "flyto_robotics.robot_doctor._run",
            lambda *_args, **_kwargs: pytest.fail("corrupt state must fail closed"),
        )
        values = collect_service_recoveries(state)
        assert all(item["status"] == "unknown" for item in values.values())
        payload, quality = diagnostic_payload(observation(service_recoveries=values))
        assert quality == "good"
        assert payload["recovery"]["service_restarts"]["status"] == "unknown"

    def test_watchdog_event_is_once_then_later_increment_is_observable(
        self, monkeypatch, tmp_path
    ):
        root = tmp_path.resolve()
        state = root / "recovery-state.json"
        journal = root / "events" / "device-events.jsonl"
        current = {"count": 1}

        def fake_collect(**_kwargs):
            recoveries = {
                "flyto-delivery.service": {
                    "status": "known",
                    "restart_count": current["count"],
                    "watchdog_count": current["count"],
                    "new_count": current["count"]
                    - json.loads(state.read_text())["watchdog_counts"].get(
                        "flyto-delivery.service", 0
                    )
                    if state.exists()
                    else current["count"],
                    "current_recovery_kind": (
                        "watchdog_timeout" if current["count"] - (
                            json.loads(state.read_text())["watchdog_counts"].get(
                                "flyto-delivery.service", 0
                            ) if state.exists() else 0
                        ) else None
                    ),
                }
            }
            return observation(service_recoveries=recoveries)

        monkeypatch.setattr("flyto_robotics.robot_doctor.collect_observation", fake_collect)
        args = [
            "--resource-id", "robot-1",
            "--output", str(root / "latest.json"),
            "--last-failure", str(root / "last-failure.json"),
            "--recovery-state", str(state),
            "--event-journal", str(journal),
        ]
        assert main(args) == 0
        first_failure = (root / "last-failure.json").read_text()
        assert main(args) == 0
        assert len(DeviceEventJournal(journal).read_all()) == 1
        assert json.loads((root / "latest.json").read_text())["quality"] == "good"
        assert (root / "last-failure.json").read_text() == first_failure

        current["count"] = 2
        assert main(args) == 0
        events = DeviceEventJournal(journal).read_all()
        assert len(events) == 2
        event = events[-1]["event"]
        assert event["reason_code"] == "managed_service_recovered"
        assert event["action_codes"] == ["inspect_service_recovery"]
        assert event["message"] == "A managed service recovered after a watchdog restart."

    def test_partial_unknown_preserves_same_boot_baseline(self, tmp_path):
        state = tmp_path / "state.json"
        state.write_text(json.dumps({
            "contract": "flyto.service-recovery-state.v1",
            "boot_id": "",
            "restart_counts": {
                "flyto-delivery.service": 3,
                "flyto-job-runner.service": 4,
            },
            "watchdog_counts": {
                "flyto-delivery.service": 1,
                "flyto-job-runner.service": 2,
            },
        }))
        _commit_recovery_state(state, {
            "flyto-delivery.service": {
                "status": "known", "restart_count": 5, "watchdog_count": 2,
            },
            "flyto-job-runner.service": {
                "status": "unknown", "restart_count": 0, "watchdog_count": 0,
            },
        })
        stored = json.loads(state.read_text())
        assert stored["restart_counts"]["flyto-delivery.service"] == 5
        assert stored["restart_counts"]["flyto-job-runner.service"] == 4
        assert stored["watchdog_counts"]["flyto-delivery.service"] == 2
        assert stored["watchdog_counts"]["flyto-job-runner.service"] == 2

    def test_concurrent_retry_records_one_recovery_event(self, tmp_path):
        root = tmp_path.resolve()
        journal = root / "events" / "device-events.jsonl"
        value = observation(service_recoveries={
            "flyto-delivery.service": {
                "status": "known", "restart_count": 1, "watchdog_count": 1,
                "new_count": 1, "current_recovery_kind": "watchdog_timeout",
            }
        })
        report = build_telemetry(
            value, resource_id="robot-1", sequence=1,
            observed_at="2026-08-13T00:00:00Z",
        )
        def record():
            return _record_degradation(
                value, report, resource_id="robot-1", sequence=1,
                observed_at="2026-08-13T00:00:00Z", journal_path=journal,
            )
        with ThreadPoolExecutor(max_workers=2) as pool:
            assert list(pool.map(lambda _item: record(), range(2))) == [0, 0]
        records = DeviceEventJournal(journal).read_all()
        assert [item["event"]["reason_code"] for item in records] == [
            "managed_service_recovered"
        ]

    def test_other_primary_reason_does_not_swallow_recovery(self, tmp_path):
        root = tmp_path.resolve()
        journal = root / "events" / "device-events.jsonl"
        recovery = {
            "flyto-delivery.service": {
                "status": "known", "restart_count": 1, "watchdog_count": 1,
                "new_count": 1, "current_recovery_kind": "watchdog_timeout",
            }
        }
        value = observation(cloud_reachable=False, service_recoveries=recovery)
        report = build_telemetry(
            value, resource_id="robot-1", sequence=1,
            observed_at="2026-08-13T00:00:00Z",
        )
        assert report["payload"]["primary_reason_code"] == "cloud_unreachable"
        assert _record_degradation(
            value, report, resource_id="robot-1", sequence=1,
            observed_at="2026-08-13T00:00:00Z", journal_path=journal,
        ) == 0
        recovered = observation(service_recoveries={
            "flyto-delivery.service": {
                **recovery["flyto-delivery.service"],
                "new_count": 0, "current_recovery_kind": None,
            }
        })
        healthy_report = build_telemetry(
            recovered, resource_id="robot-1", sequence=2,
            observed_at="2026-08-13T00:01:00Z",
        )
        assert _record_degradation(
            recovered, healthy_report, resource_id="robot-1", sequence=2,
            observed_at="2026-08-13T00:01:00Z", journal_path=journal,
        ) == 0
        reasons = [item["event"]["reason_code"] for item in DeviceEventJournal(journal).read_all()]
        assert reasons.count("managed_service_recovered") == 1

    def test_raw_journal_text_never_enters_telemetry(self):
        report = build_telemetry(
            observation(
                service_recoveries={
                    "opaque.service": {
                        "status": "known",
                        "restart_count": 1,
                        "watchdog_count": 1,
                        "new_count": 1,
                        "current_recovery_kind": "watchdog_timeout",
                        "raw": "patient password secret-token",
                    }
                }
            ),
            resource_id="robot-1",
            sequence=1,
            observed_at="2026-08-13T00:00:00Z",
        )
        encoded = json.dumps(report).lower()
        assert "patient" not in encoded
        assert "password" not in encoded
        assert "secret-token" not in encoded
        assert report["payload"]["recovery"]["service_restarts"][
            "raw_journal_included"
        ] is False

    @pytest.mark.parametrize(
        ("state", "reason"),
        [("failed", "robot_service_unhealthy"), ("unknown", "service_state_unknown")],
    )
    def test_current_failure_or_unknown_outranks_recovery(self, state, reason):
        value = observation(
            service_states={"flyto-delivery.service": state},
            service_recoveries={
                "flyto-delivery.service": {
                    "status": "known",
                    "restart_count": 1,
                    "watchdog_count": 1,
                    "new_count": 1,
                    "current_recovery_kind": "watchdog_timeout",
                }
            },
        )
        assert diagnostic_payload(value)[0]["primary_reason_code"] == reason
