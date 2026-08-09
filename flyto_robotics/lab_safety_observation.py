"""What a bystander saw the lab robot do, decided without importing ROS.

The Gazebo lab driver watches the robot from outside — it subscribes to the
same ``/cmd_vel`` the controller publishes and records whether motion stopped
when the lidar closed in, and whether it resumed once the way was clear. That
recording is the evidence a demonstration rests on, so it has to be right.

It lived inside an 819-line node that imports ``rclpy`` at module scope, which
made it unimportable on a development machine. The tests that covered it
therefore read the driver's *source* and asserted that certain strings appeared
in it — including one that asserted a particular line break. Those pass whether
or not the logic works, and fail on reformatting that changes nothing. They
report the behaviour as verified without executing a line of it.

Nothing here is a safety gate: it observes, it does not command. Skipping an
observation cannot move a robot. The extraction is about being able to test it
at all.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

#: A command this small is a stop, allowing for float noise on the wire.
STOP_EPSILON = 0.001

#: Above this the robot is meaningfully under way, rather than drifting.
MOTION_EPSILON = 0.01

STOP_OBSERVED = "safety_stop_observed"
MOTION_RESUMED = "motion_resumed_observed"


@dataclass(frozen=True)
class CommandedVelocity:
    """One ``/cmd_vel`` message, as the observer read it."""

    linear_x: float
    angular_z: float

    @property
    def is_zero(self) -> bool:
        return abs(self.linear_x) <= STOP_EPSILON and abs(self.angular_z) <= STOP_EPSILON

    @property
    def is_moving(self) -> bool:
        return abs(self.linear_x) > MOTION_EPSILON


@dataclass
class SafetyObservation:
    """Latching record of a stop and the resume that followed it.

    Each transition is reported once and only once: a demonstration that
    recorded the same stop on every tick would be describing one event as
    dozens.
    """

    stop_distance: float
    motion_before_obstacle: bool = False
    stop_observed: bool = False
    resume_observed: bool = False

    def observe(
        self,
        *,
        obstacle_entered: bool,
        obstacle_exited: bool,
        minimum_range: float,
        command: CommandedVelocity | None,
    ) -> str | None:
        """Fold one tick in, returning the transition it produced, if any."""
        if command is None:
            return None

        # Motion seen before the obstacle is what makes a later stop meaningful.
        # A robot that never moved has not been stopped by anything.
        if not obstacle_entered and command.is_moving:
            self.motion_before_obstacle = True

        # An infinite range is nothing within reach, so there is no proximity
        # stop to attribute anything to. Not a gate: refusing to record an
        # event cannot move a robot.
        measured = math.isfinite(minimum_range)

        if (
            obstacle_entered
            and not self.stop_observed
            and self.motion_before_obstacle
            and measured
            and minimum_range < self.stop_distance
            and command.is_zero
        ):
            self.stop_observed = True
            return STOP_OBSERVED

        if (
            self.stop_observed
            and obstacle_exited
            and not self.resume_observed
            and measured
            and minimum_range >= self.stop_distance
            and command.is_moving
        ):
            self.resume_observed = True
            return MOTION_RESUMED

        return None
