"""Site limits a job may tighten and never loosen.

The numbers were never hardcoded — SafetyLimits has always been a job field
with clamps. What was wrong was who decides: a job carried its own limits, so
"this robot, in this building, may not exceed X" had to be written into every
job and could be forgotten in any one of them.

The part most likely to be got backwards is the direction. Lower speed is
safer; *higher* stop distance is safer. An implementation that treated them the
same would let a job drive closer to things than the site allows, and would
look correct in a test that only ever checked speeds.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from flyto_robotics.contracts import JobValidationError, load_job, parse_job
from flyto_robotics.safety_profile import (
    CONSTRAINABLE,
    PROFILE_CONTRACT_VERSION,
    SafetyProfileError,
    apply_profile,
    change_record,
    is_more_conservative,
    load_profile,
    parse_profile,
    update_profile,
)

SITE = {
    "max_linear_speed": 0.20,
    "obstacle_stop_distance": 0.60,
}


def document(**limits):
    return {"contract_version": PROFILE_CONTRACT_VERSION, "limits": limits}


class TestTheDirectionOfSafer:
    """The whole point of the module, asserted per field rather than assumed."""

    def test_every_constrainable_field_declares_a_direction(self):
        assert set(CONSTRAINABLE.values()) <= {"at_most", "at_least"}
        assert CONSTRAINABLE, "an empty table would silently constrain nothing"

    @pytest.mark.parametrize("field", ["max_linear_speed", "max_angular_speed"])
    def test_speeds_are_ceilings(self, field):
        assert CONSTRAINABLE[field] == "at_most"
        assert is_more_conservative(field, 0.1, 0.2) is True
        assert is_more_conservative(field, 0.3, 0.2) is False

    @pytest.mark.parametrize(
        "field",
        ["obstacle_stop_distance", "lateral_stop_distance", "emergency_stop_distance"],
    )
    def test_stop_distances_are_floors(self, field):
        assert CONSTRAINABLE[field] == "at_least"
        assert is_more_conservative(field, 0.9, 0.5) is True, "further away is safer"
        assert is_more_conservative(field, 0.3, 0.5) is False

    def test_a_field_outside_the_table_cannot_be_judged(self):
        with pytest.raises(SafetyProfileError, match="not a constrainable limit"):
            is_more_conservative("mission_timeout_seconds", 1.0, 2.0)


class TestFoldingAJobIntoTheSite:
    def test_a_job_asking_for_more_speed_gets_the_site_ceiling(self):
        outcome = apply_profile(SITE, {"max_linear_speed": 0.45})
        assert outcome.values["max_linear_speed"] == 0.20
        assert outcome.constrained

    def test_a_job_asking_to_stop_later_gets_the_site_floor(self):
        """Backwards handling would show up here and nowhere else."""
        outcome = apply_profile(SITE, {"obstacle_stop_distance": 0.30})
        assert outcome.values["obstacle_stop_distance"] == 0.60

    def test_a_stricter_job_is_left_exactly_as_it_asked(self):
        outcome = apply_profile(
            SITE, {"max_linear_speed": 0.10, "obstacle_stop_distance": 1.20}
        )
        assert outcome.values == {"max_linear_speed": 0.10, "obstacle_stop_distance": 1.20}
        assert not outcome.constrained

    def test_a_field_the_site_does_not_constrain_passes_through(self):
        outcome = apply_profile(SITE, {"max_angular_speed": 1.5})
        assert outcome.values["max_angular_speed"] == 1.5
        assert not outcome.constrained

    def test_an_unset_job_field_is_not_invented(self):
        """lateral_stop_distance defaults to None and must stay None."""
        outcome = apply_profile({"lateral_stop_distance": 0.4}, {"lateral_stop_distance": None})
        assert "lateral_stop_distance" not in outcome.values

    def test_each_override_is_reported_with_both_numbers(self):
        outcome = apply_profile(SITE, {"max_linear_speed": 0.45})
        (adjustment,) = outcome.adjustments
        assert adjustment.requested == 0.45
        assert adjustment.applied == 0.20
        assert "0.45" in adjustment.describe() and "0.2" in adjustment.describe()

    def test_a_job_is_constrained_rather_than_rejected(self):
        """Refusing would make the profile something operators route around."""
        outcome = apply_profile(SITE, {"max_linear_speed": 0.5})
        assert outcome.values["max_linear_speed"] == 0.20


class TestReadingAProfile:
    def test_a_valid_document_parses(self):
        assert parse_profile(document(max_linear_speed=0.2)) == {"max_linear_speed": 0.2}

    def test_an_unknown_limit_is_refused_not_ignored(self):
        """A typo in a safety file that silently does nothing is worse than one
        that fails: the site would believe a limit was in force."""
        with pytest.raises(SafetyProfileError, match="cannot constrain"):
            parse_profile(document(max_speed=0.2))

    def test_a_missing_contract_version_is_refused(self):
        with pytest.raises(SafetyProfileError, match="contract_version"):
            parse_profile({"limits": {"max_linear_speed": 0.2}})

    @pytest.mark.parametrize("value", [0, -1, "0.2", True, None, float("inf"), float("nan")])
    def test_a_value_that_is_not_a_positive_number_is_refused(self, value):
        with pytest.raises(SafetyProfileError):
            parse_profile(document(max_linear_speed=value))

    def test_an_absent_file_means_no_constraint(self, tmp_path):
        """Correct default for a robot not yet commissioned into a site."""
        assert load_profile(tmp_path / "nothing-here.json") == {}

    def test_an_unreadable_file_is_an_error_not_an_absence(self, tmp_path):
        """A site that wrote a profile and had it ignored would believe a limit
        was in force that was not."""
        path = tmp_path / "safety-profile.json"
        path.write_text("{ not json")
        with pytest.raises(SafetyProfileError, match="not readable JSON"):
            load_profile(path)

    def test_an_oversized_file_is_refused(self, tmp_path):
        path = tmp_path / "safety-profile.json"
        path.write_text(" " * (17 * 1024))
        with pytest.raises(SafetyProfileError, match="exceeds"):
            load_profile(path)


class TestTheSiteCannotBeBypassed:
    """Enforced where SafetyLimits is built, not at a call site somebody can
    forget. A site limit enforced everywhere except one hurried code path is
    not a site limit."""

    # Built from the shipped example rather than by hand, so this keeps
    # working as the job contract gains fields.
    EXAMPLE = Path(__file__).resolve().parents[1] / "examples/jobs/tb3-lab-shortcut.json"

    def job_document(self):
        document = json.loads(self.EXAMPLE.read_text())
        document["safety"] = {
            **document.get("safety", {}),
            "max_linear_speed": 0.45,
            "obstacle_stop_distance": 0.20,
        }
        return document

    def sited(self, monkeypatch, tmp_path, **limits):
        path = tmp_path / "safety-profile.json"
        path.write_text(json.dumps(document(**limits)))
        monkeypatch.setenv("FLYTO_SAFETY_PROFILE", str(path))

    def test_parse_job_is_constrained(self, monkeypatch, tmp_path):
        self.sited(monkeypatch, tmp_path, max_linear_speed=0.20)
        job = parse_job(self.job_document())
        assert job.safety.max_linear_speed == 0.20

    def test_load_job_is_constrained_too(self, monkeypatch, tmp_path):
        self.sited(monkeypatch, tmp_path, obstacle_stop_distance=0.60)
        job_file = tmp_path / "job.json"
        job_file.write_text(json.dumps(self.job_document()))
        job = load_job(job_file)
        assert job.safety.obstacle_stop_distance == 0.60

    def test_with_no_profile_the_job_gets_what_it_asked_for(self, monkeypatch, tmp_path):
        monkeypatch.setenv("FLYTO_SAFETY_PROFILE", str(tmp_path / "absent.json"))
        job = parse_job(self.job_document())
        assert job.safety.max_linear_speed == 0.45

    def test_a_broken_profile_stops_the_job_rather_than_running_unconstrained(
        self, monkeypatch, tmp_path
    ):
        """The dangerous default would be to shrug and use the job's numbers."""
        path = tmp_path / "safety-profile.json"
        path.write_text("{ not json")
        monkeypatch.setenv("FLYTO_SAFETY_PROFILE", str(path))
        with pytest.raises((SafetyProfileError, JobValidationError)):
            parse_job(self.job_document())

    def test_the_override_is_logged_where_someone_will_see_it(
        self, monkeypatch, tmp_path, caplog
    ):
        """A job quietly running slower than it asked is a day of debugging for
        whoever is standing next to the robot."""
        self.sited(monkeypatch, tmp_path, max_linear_speed=0.20)
        with caplog.at_level("WARNING"):
            parse_job(self.job_document())
        assert any("site safety profile" in record.message for record in caplog.records)


class TestTheAuditRecord:
    def before_after(self, before, after):
        return change_record(
            changed_by="operator@example",
            at="2026-08-09T00:00:00Z",
            before=before,
            after=after,
            reason="synthetic",
        )

    @pytest.mark.skipif(os.name != "posix", reason="file modes are POSIX-only here")
    def test_profile_and_audit_are_private_even_under_a_permissive_umask(self, tmp_path):
        profile = tmp_path / "safety-profile.json"
        audit = tmp_path / "safety-profile.audit.jsonl"
        previous = os.umask(0)
        try:
            update_profile(
                profile,
                audit,
                limits={"max_linear_speed": 0.2},
                changed_by="synthetic-operator",
                reason="synthetic",
                at="2026-08-27T00:00:00Z",
            )
        finally:
            os.umask(previous)
        assert profile.stat().st_mode & 0o777 == 0o600
        assert audit.stat().st_mode & 0o777 == 0o600

    def test_a_symlink_cannot_redirect_the_audit_append(self, tmp_path):
        outside = tmp_path / "outside.jsonl"
        outside.write_text("unchanged\n", encoding="utf-8")
        audit = tmp_path / "audit.jsonl"
        audit.symlink_to(outside)
        with pytest.raises(SafetyProfileError, match="could not be written"):
            update_profile(
                tmp_path / "profile.json",
                audit,
                limits={"max_linear_speed": 0.2},
                changed_by="synthetic-operator",
                reason="synthetic",
                at="2026-08-27T00:00:00Z",
            )
        assert outside.read_text(encoding="utf-8") == "unchanged\n"

    def test_a_tightening_is_not_flagged_as_relaxing(self):
        record = self.before_after({"max_linear_speed": 0.3}, {"max_linear_speed": 0.2})
        assert record["relaxes_safety"] is False

    def test_a_widened_speed_is_flagged(self):
        record = self.before_after({"max_linear_speed": 0.2}, {"max_linear_speed": 0.4})
        assert record["relaxed_limits"] == ["max_linear_speed"]

    def test_a_shortened_stop_distance_is_flagged(self):
        """The direction trap again, this time in the audit."""
        record = self.before_after(
            {"obstacle_stop_distance": 0.6}, {"obstacle_stop_distance": 0.3}
        )
        assert record["relaxes_safety"] is True

    def test_removing_a_limit_is_the_widest_relaxation_there_is(self):
        record = self.before_after({"max_linear_speed": 0.2}, {})
        assert record["relaxed_limits"] == ["max_linear_speed"]

    def test_adding_a_limit_only_tightens(self):
        record = self.before_after({}, {"max_linear_speed": 0.2})
        assert record["relaxes_safety"] is False

    def test_it_does_not_take_the_operators_word_for_it(self):
        """The entry an audit exists to catch: a relaxation described as a
        tightening. relaxes_safety is computed from the numbers."""
        record = change_record(
            changed_by="operator@example",
            at="2026-08-09T00:00:00Z",
            before={"max_linear_speed": 0.2},
            after={"max_linear_speed": 0.5},
            reason="tightening the site limit for safety week",
        )
        assert record["relaxes_safety"] is True

    def test_it_records_who_and_when(self):
        record = self.before_after({}, {"max_linear_speed": 0.2})
        assert record["changed_by"] == "operator@example"
        assert record["at"] == "2026-08-09T00:00:00Z"
        assert record["reason"] == "synthetic"


class TestTheOperatorInterface:
    """Reading and changing the site limits over the gateway.

    Loopback-only and token-authorised, which is what an offline installation
    has instead of a cloud console.
    """

    TOKEN = "synthetic-safety-profile-token"

    def gateway(self, monkeypatch, tmp_path):
        from flyto_robotics.capabilities import default_capability_registry
        from flyto_robotics.mission_gateway import MissionGateway

        monkeypatch.setenv("FLYTO_SAFETY_PROFILE", str(tmp_path / "safety-profile.json"))
        monkeypatch.setenv("FLYTO_SAFETY_PROFILE_AUDIT", str(tmp_path / "audit.jsonl"))
        return MissionGateway(default_capability_registry(), self.TOKEN)

    def call(self, gateway, method, path, body=None):
        import urllib.error
        import urllib.request

        host, port = gateway.address
        request = urllib.request.Request(
            f"http://{host}:{port}{path}",
            method=method,
            data=json.dumps(body).encode() if body is not None else None,
            headers={"Authorization": f"Bearer {self.TOKEN}"},
        )
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())

    def test_an_uncommissioned_robot_reports_no_limits_rather_than_failing(
        self, monkeypatch, tmp_path
    ):
        with self.gateway(monkeypatch, tmp_path) as gateway:
            status, body = self.call(gateway, "GET", "/v1/safety-profile")
        assert status == 200
        assert body["limits"] == {}

    def test_the_view_says_which_way_safer_runs_for_each_field(
        self, monkeypatch, tmp_path
    ):
        """An operator setting a stop distance needs to know it is a floor."""
        with self.gateway(monkeypatch, tmp_path) as gateway:
            _, body = self.call(gateway, "GET", "/v1/safety-profile")
        assert body["constrainable"]["max_linear_speed"] == "at_most"
        assert body["constrainable"]["obstacle_stop_distance"] == "at_least"

    def test_a_change_is_stored_and_read_back(self, monkeypatch, tmp_path):
        with self.gateway(monkeypatch, tmp_path) as gateway:
            status, _ = self.call(
                gateway,
                "POST",
                "/v1/safety-profile",
                {
                    "limits": {"max_linear_speed": 0.18},
                    "changed_by": "operator@site",
                    "reason": "corridor is narrower than the default assumes",
                },
            )
            assert status == 200
            _, body = self.call(gateway, "GET", "/v1/safety-profile")
        assert body["limits"] == {"max_linear_speed": 0.18}

    def test_the_change_appears_in_the_history_with_who_and_why(
        self, monkeypatch, tmp_path
    ):
        with self.gateway(monkeypatch, tmp_path) as gateway:
            self.call(
                gateway,
                "POST",
                "/v1/safety-profile",
                {
                    "limits": {"max_linear_speed": 0.18},
                    "changed_by": "operator@site",
                    "reason": "narrow corridor",
                },
            )
            _, body = self.call(gateway, "GET", "/v1/safety-profile")
        (entry,) = body["recent_changes"]
        assert entry["changed_by"] == "operator@site"
        assert entry["reason"] == "narrow corridor"
        assert entry["relaxes_safety"] is False

    def test_a_relaxation_is_recorded_as_one(self, monkeypatch, tmp_path):
        with self.gateway(monkeypatch, tmp_path) as gateway:
            for speed in (0.15, 0.40):
                self.call(
                    gateway,
                    "POST",
                    "/v1/safety-profile",
                    {
                        "limits": {"max_linear_speed": speed},
                        "changed_by": "operator@site",
                        "reason": "synthetic",
                    },
                )
            _, body = self.call(gateway, "GET", "/v1/safety-profile")
        assert body["recent_changes"][-1]["relaxes_safety"] is True

    @pytest.mark.parametrize(
        ("body", "missing"),
        [
            ({"limits": {"max_linear_speed": 0.2}, "reason": "x"}, "changed_by"),
            ({"limits": {"max_linear_speed": 0.2}, "changed_by": "a"}, "reason"),
        ],
    )
    def test_an_unattributed_change_is_refused(self, monkeypatch, tmp_path, body, missing):
        with self.gateway(monkeypatch, tmp_path) as gateway:
            status, response = self.call(gateway, "POST", "/v1/safety-profile", body)
        assert status == 400
        assert missing in response["detail"]

    def test_an_unknown_limit_is_refused(self, monkeypatch, tmp_path):
        with self.gateway(monkeypatch, tmp_path) as gateway:
            status, response = self.call(
                gateway,
                "POST",
                "/v1/safety-profile",
                {
                    "limits": {"max_speed": 0.2},
                    "changed_by": "a",
                    "reason": "b",
                },
            )
        assert status == 400
        assert "cannot constrain" in response["detail"]

    def test_it_needs_the_token(self, monkeypatch, tmp_path):
        import urllib.error
        import urllib.request

        with self.gateway(monkeypatch, tmp_path) as gateway:
            host, port = gateway.address
            request = urllib.request.Request(f"http://{host}:{port}/v1/safety-profile")
            try:
                with urllib.request.urlopen(request, timeout=5) as response:
                    status = response.status
            except urllib.error.HTTPError as exc:
                status = exc.code
        assert status == 401

    def test_a_change_that_cannot_be_recorded_does_not_happen(
        self, monkeypatch, tmp_path
    ):
        """No change without a record.

        A limit that can be moved without leaving a trace is not governed, and
        the moment it matters is exactly the moment someone would rather it
        left none.
        """
        from flyto_robotics.safety_profile import update_profile

        unwritable = tmp_path / "readonly"
        unwritable.mkdir(mode=0o500)
        profile = tmp_path / "safety-profile.json"
        with pytest.raises(SafetyProfileError, match="could not be written"):
            update_profile(
                profile,
                unwritable / "audit.jsonl",
                limits={"max_linear_speed": 0.2},
                changed_by="operator@site",
                reason="synthetic",
                at="2026-08-09T00:00:00Z",
            )
        assert not profile.exists(), "the profile changed despite the audit failing"
