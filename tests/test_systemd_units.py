"""What a systemd unit *means*, not what its text contains.

Every assertion the project makes about a unit has to survive the failure mode
recorded in handoffs/2026-08-07: a directive in the wrong section is parsed,
accepted, and ignored, so a substring check passes on a unit that does nothing.
"""

from __future__ import annotations

from flyto_robotics.systemd_units import parse_unit, validate_unit


def _codes(text: str, **kwargs) -> set[str]:
    return {defect.code for defect in validate_unit(text, **kwargs)}


class TestContinuationSemantics:
    def test_an_escaped_backslash_does_not_swallow_the_next_directive(self) -> None:
        # The bug this replaces: `line.endswith("\\")` cannot tell a continuation
        # from an escaped literal backslash, so it ate the following line whole.
        # Restart=always then read as *absent*, and a test asserting the unit had
        # no unbounded restart would pass on a unit that restarts forever.
        parsed = parse_unit("[Service]\nExecStart=/bin/echo c:\\\\\nRestart=always\n")
        assert parsed.values("Service", "ExecStart") == ["/bin/echo c:\\\\"]
        assert parsed.values("Service", "Restart") == ["always"]

    def test_an_odd_run_of_backslashes_still_continues(self) -> None:
        parsed = parse_unit("[Service]\nExecStart=/bin/sh -c 'a \\\nb'\n")
        assert parsed.values("Service", "ExecStart") == ["/bin/sh -c 'a b'"]

    def test_a_continuation_dangling_at_end_of_file_is_applied_and_reported(self) -> None:
        # systemd warns and still applies the accumulated directive. Silently
        # discarding it made a real directive read as absent to every caller.
        parsed = parse_unit("[Unit]\nDescription=Flyto \\\n")
        assert parsed.values("Unit", "Description") == ["Flyto"]
        assert [defect.code for defect in parsed.defects] == ["dangling_continuation"]

    def test_a_continued_comment_cannot_capture_the_directive_below_it(self) -> None:
        parsed = parse_unit("[Service]\n# wrapped note \\\nstill a note\nExecStart=/bin/true\n")
        assert parsed.values("Service", "ExecStart") == ["/bin/true"]

    def test_a_directive_before_any_section_is_reported_not_dropped(self) -> None:
        parsed = parse_unit("Restart=always\n[Service]\nExecStart=/bin/true\n")
        assert [defect.code for defect in parsed.defects] == ["directive_outside_section"]

    def test_repeats_and_embedded_equals_survive(self) -> None:
        parsed = parse_unit("[Service]\nEnvironment=A=1\nEnvironment=B=2\n")
        assert parsed.values("Service", "Environment") == ["A=1", "B=2"]


class TestValidation:
    def test_start_limit_in_the_service_section_is_a_defect(self) -> None:
        text = (
            "[Unit]\nDescription=x\n"
            "[Service]\nExecStart=/bin/true\nRestart=always\n"
            "StartLimitBurst=3\nStartLimitIntervalSec=300\n"
        )
        assert "start_limit_wrong_section" in _codes(text)

    def test_restart_always_without_a_start_limit_is_a_defect(self) -> None:
        text = "[Unit]\nDescription=x\n[Service]\nExecStart=/bin/true\nRestart=always\n"
        assert "unbounded_restart" in _codes(text)

    def test_a_burst_too_high_to_trip_is_a_defect(self) -> None:
        # 300s/20 cannot be reached when one failure cycle takes minutes: the
        # counter resets before the limiter ever fires. That is not a limit.
        text = (
            "[Unit]\nDescription=x\nStartLimitIntervalSec=300\nStartLimitBurst=20\n"
            "[Service]\nExecStart=/bin/true\nRestart=always\n"
        )
        assert "start_limit_unreachable" in _codes(text)

    def test_a_home_directory_path_is_a_defect_for_a_product_unit(self) -> None:
        text = (
            "[Unit]\nDescription=x\nStartLimitIntervalSec=300\nStartLimitBurst=3\n"
            "[Service]\nWorkingDirectory=/home/ubuntu/flyto-robotics\n"
            "ExecStart=/bin/true\nRestart=on-failure\n"
        )
        assert "home_path_hardcoded" in _codes(text)

    def test_a_correct_unit_has_no_defects(self) -> None:
        text = (
            "[Unit]\nDescription=x\nStartLimitIntervalSec=300\nStartLimitBurst=3\n"
            "[Service]\nWorkingDirectory=/opt/flyto-robot/current\n"
            "ExecStart=/usr/bin/python3 -m flyto_robotics.resource_agent\n"
            "Restart=on-failure\n"
        )
        assert validate_unit(text, name="flyto-robot-agent.service") == ()
