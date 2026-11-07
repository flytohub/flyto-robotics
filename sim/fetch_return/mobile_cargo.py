"""SIMULATION: the mobile robot's cargo tray state, one implementation.

``run.py`` (the offline fetch-and-return mission) and ``cargo_bridge.py`` (the
live twin's ``/flyto/cargo_present`` publisher) both hold the tray through
:class:`MobileCargoSimulator`. It is a symbolic simulator, NOT a sensor and NOT
coupled physics: the stationary arm's MuJoCo world verifies the place into a
docking-bay *proxy* of the tray while the robot is parked, and from the handoff
onward this object alone says whether the mobile robot carries the block.
"""

from __future__ import annotations

from typing import Any

from sim.fetch_return.contract import MissionFailure

MOBILE_RESOURCE = "turtlebot3-mobile-sim"
ARM_RESOURCE = "am-arm200-fixed-station-sim"


class MobileCargoSimulator:
    """Independent mobile tray state; never read the fixed arm after departure.

    The stationary arm's MuJoCo model verifies placement into the tray *proxy*
    while docked. A separate cargo simulator owns the transported object's
    presence thereafter. This boundary is NOT a physical cargo simulation.
    """

    def __init__(self):
        self._carrying = False
        self._verified_transfer = False

    @property
    def carrying(self) -> bool:
        return self._carrying

    def accept_station_handoff(
        self,
        *,
        is_docked: bool,
        fixed_arm_retracted: bool,
        observed_placement: bool,
        physics_placement: bool,
    ) -> dict[str, Any]:
        if not is_docked:
            raise MissionFailure("handoff", "robot_not_docked")
        if not fixed_arm_retracted:
            raise MissionFailure("handoff", "stationary_arm_not_retracted")
        if not observed_placement or not physics_placement:
            raise MissionFailure("handoff", "cargo_transfer_not_verified")
        self._carrying = True
        self._verified_transfer = True
        return {
            "source": "station_mujoco_physics_plus_simulated_mobile_cargo_sensor",
            "from_resource_id": ARM_RESOURCE,
            "to_resource_id": MOBILE_RESOURCE,
            "handoff_verified": True,
            "cargo_present": True,
            "simulation_only": True,
        }

    def report_loss(self) -> None:
        """The load is no longer aboard (observed lost, or a test fault)."""
        self._carrying = False

    def unload(self) -> dict[str, Any]:
        """The load was taken off (delivered at home, or the mission ended)."""
        was = self._carrying
        self._carrying = False
        return {
            "source": "mobile_cargo_sensor_simulation_not_physical",
            "resource_id": MOBILE_RESOURCE,
            "unloaded": was,
            "cargo_present": False,
            "simulation_only": True,
        }

    def inject_loss(self) -> None:
        """Fault injection for the offline mission: the payload is lost in transit."""
        self.report_loss()

    def observe_on_mobile(self) -> dict[str, Any]:
        return {
            "source": "mobile_cargo_sensor_simulation_not_physical",
            "resource_id": MOBILE_RESOURCE,
            "cargo_present": self._carrying,
            "handoff_verified": self._verified_transfer,
        }
