from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_yaw_deg(value: float) -> float:
    wrapped = (float(value) + 180.0) % 360.0 - 180.0
    return 0.0 if abs(wrapped) < 0.0005 else wrapped


@dataclass(frozen=True)
class RelativePoseDelta:
    dx: float
    dy: float
    dz: float
    dyaw_deg: float

    def __post_init__(self) -> None:
        values = (self.dx, self.dy, self.dz, self.dyaw_deg)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("relative pose contains a non-finite value")


@dataclass(frozen=True)
class WorldPose:
    x: float
    y: float
    z: float
    yaw_deg: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(float(value)) for value in (self.x, self.y, self.z, self.yaw_deg)):
            raise ValueError("world pose contains a non-finite value")

