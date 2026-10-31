"""Provider-neutral simulated navigation contract and failure status."""

from __future__ import annotations

from dataclasses import dataclass

WORKSPACE_FRAME = "local-map-sim-v1"


class MissionFailure(RuntimeError):
    def __init__(self, stage: str, reason: str):
        super().__init__(reason)
        self.stage, self.reason = stage, reason


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float = 0.0

    def as_dict(self) -> dict[str, float]:
        return {"x": round(self.x, 5), "y": round(self.y, 5), "yaw": round(self.yaw, 5)}
