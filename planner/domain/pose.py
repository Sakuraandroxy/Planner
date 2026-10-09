from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_yaw_deg(value: float) -> float:
    wrapped = (float(value) + 180.0) % 360.0 - 180.0
    return 0.0 if abs(wrapped) < 0.0005 else wrapped


def resolve_yaw_delta_deg(start: float, end: float, signed_delta: float | None = None) -> float:
    """Retain explicit turn direction; legacy endpoints use the shortest turn."""
    if signed_delta is None:
        return (end-start+180.0) % 360.0-180.0
    if (isinstance(signed_delta, bool) or not isinstance(signed_delta, (int, float)) or
            not math.isfinite(signed_delta)):
        raise ValueError("signed yaw delta must be a finite number")
    # Do not use wrap_yaw_deg here: its small-angle snapping hides mismatches.
    mismatch = (start+signed_delta-end+180.0) % 360.0-180.0
    if abs(mismatch) > 1e-6:
        raise ValueError("signed yaw delta disagrees with endpoint yaw")
    return float(signed_delta)


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

