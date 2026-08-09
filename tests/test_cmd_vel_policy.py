"""The cmd_vel type decision, exercised without a ROS installation.

Publishing the wrong velocity type matches zero subscribers and DDS reports
nothing, so every mistake this module can make is silent. These tests exist
because that is precisely the kind of mistake a running system will not tell
you about.
"""

from __future__ import annotations

import pytest

from flyto_robotics.cmd_vel_policy import (
    CMD_VEL_TYPE_AUTO,
    CMD_VEL_TYPE_TWIST,
    CMD_VEL_TYPE_TWIST_STAMPED,
    TWIST_STAMPED_TYPE_NAME,
    UNDECIDED,
    resolve_cmd_vel_type,
    validated_topic,
)

TWIST_TYPE_NAME = "geometry_msgs/msg/Twist"


def auto(**overrides):
    """resolve_cmd_vel_type with the auto-detection defaults filled in."""
    kwargs = {
        "configured": CMD_VEL_TYPE_AUTO,
        "subscriber_types": (),
        "discovery_expired": False,
    }
    kwargs.update(overrides)
    return resolve_cmd_vel_type(**kwargs)


class TestOperatorOverride:
    @pytest.mark.parametrize(
        "configured", [CMD_VEL_TYPE_TWIST, CMD_VEL_TYPE_TWIST_STAMPED]
    )
    def test_an_explicit_type_is_final_and_ignores_discovery(self, configured):
        """An operator who names a type has out-argued the graph."""
        decision = auto(
            configured=configured,
            subscriber_types=(TWIST_STAMPED_TYPE_NAME,),
            discovery_expired=True,
        )
        assert decision == (configured, False)


class TestDiscoveredDriver:
    def test_twist_stamped_subscriber_decides_twist_stamped(self):
        decision = auto(subscriber_types=(TWIST_STAMPED_TYPE_NAME,))
        assert decision == (CMD_VEL_TYPE_TWIST_STAMPED, False)

    def test_twist_stamped_wins_when_both_types_are_present(self):
        """A Gazebo bridge and a real driver can share the topic.

        Sending Twist would reach the bridge and leave the robot still. Sending
        TwistStamped reaches the driver, which is the one that moves wheels.
        """
        decision = auto(subscriber_types=(TWIST_TYPE_NAME, TWIST_STAMPED_TYPE_NAME))
        assert decision == (CMD_VEL_TYPE_TWIST_STAMPED, False)

    def test_a_plain_twist_subscriber_decides_twist(self):
        decision = auto(subscriber_types=(TWIST_TYPE_NAME,))
        assert decision == (CMD_VEL_TYPE_TWIST, False)

    def test_an_unrecognised_subscriber_still_counts_as_listening(self):
        """Someone is there and it is not TwistStamped, so Twist is the guess."""
        decision = auto(subscriber_types=("some_pkg/msg/Other",))
        assert decision == (CMD_VEL_TYPE_TWIST, False)


class TestBeforeDiscoveryFinishes:
    def test_an_ordinary_command_waits_rather_than_guessing(self):
        """Publishing nothing for a few milliseconds beats publishing into a void.

        The robot has not been told to move, so a skipped tick is safe; a wrong
        type is not, because nothing reports it.
        """
        assert auto() == (UNDECIDED, True)

    def test_a_stop_never_waits(self):
        decision = auto(stop_cannot_wait=True)
        assert decision == (CMD_VEL_TYPE_TWIST, True)

    def test_the_stop_binding_stays_provisional(self):
        """A held-still controller publishes a stop every tick.

        If forcing a stop produced a final answer, the very first tick of every
        mission would settle the type before discovery had a chance to run.
        """
        assert auto(stop_cannot_wait=True).provisional is True


class TestAfterTheWindowExpires:
    def test_the_fallback_is_twist(self):
        assert auto(discovery_expired=True).message_type == CMD_VEL_TYPE_TWIST

    def test_the_fallback_stays_provisional(self):
        """The regression this module was split out to fix.

        The timeout guess used to be returned as decided, which cached it for
        the life of the process. Discovery on a Pi over Wi-Fi has been measured
        at 2.6-9.1s against a 3.0s window, so the driver routinely arrives just
        after the deadline — and a final guess meant that arrival was never
        read. The mission published Twist at a TwistStamped topic for its whole
        run and failed with no DDS error to point at.
        """
        assert auto(discovery_expired=True).provisional is True

    def test_a_driver_discovered_after_the_deadline_is_still_adopted(self):
        """The sequence that used to be unreachable, start to finish."""
        before = auto()
        assert before == (UNDECIDED, True)

        timed_out = auto(discovery_expired=True)
        assert timed_out == (CMD_VEL_TYPE_TWIST, True), "must remain revisable"

        late = auto(discovery_expired=True, subscriber_types=(TWIST_STAMPED_TYPE_NAME,))
        assert late == (CMD_VEL_TYPE_TWIST_STAMPED, False)


class TestValidatedTopic:
    @pytest.mark.parametrize(
        ("value", "expected"),
        [
            ("/cmd_vel", "/cmd_vel"),
            ("cmd_vel", "/cmd_vel"),
            ("/flyto/scan", "/flyto/scan"),
            ("~private", "/~private"),
        ],
    )
    def test_accepts_topic_names_and_makes_them_absolute(self, value, expected):
        assert validated_topic(value, "cmd_vel_topic") == expected

    @pytest.mark.parametrize(
        "value",
        [
            "",
            "/cmd vel",
            "/cmd_vel; rm -rf /",
            "/cmd-vel",
            "1_leading_digit",
            "/" + "a" * 200,
            None,
            42,
        ],
    )
    def test_rejects_anything_else(self, value):
        with pytest.raises(ValueError, match="valid ROS topic name"):
            validated_topic(value, "cmd_vel_topic")

    def test_the_error_names_the_offending_field(self):
        with pytest.raises(ValueError, match="scan_topic"):
            validated_topic("not a topic", "scan_topic")
