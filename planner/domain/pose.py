"""Pose conventions shared by the whole stack.

World frame (planner):
    x  forward / north
    y  right / east
    z  down
    yaw_deg  rotation about +z, positive means clockwise seen from above
             (a right turn)

Body frame:
    +dx forward, +dy right, +dz down, +dyaw right turn.

CARLA uses a left-handed, Z-up frame (metres in this build's map data). The
CARLA adapter is the only place that converts between the two.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_deg(angle: float) -> float:
    """Wrap an angle in degrees to (-180, 180]."""
    wrapped = (float(angle) + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def wrap_yaw_deg(value: float) -> float:
    """Reference wrap used by streaming transforms; snaps sub-millidegree noise."""
    wrapped = (float(value) + 180.0) % 360.0 - 180.0
    return 0.0 if abs(wrapped) < 0.0005 else wrapped


def resolve_yaw_delta_deg(
    start: float, end: float, signed_delta: float | None = None
) -> float:
    """Retain explicit turn direction; legacy endpoints use the shortest turn."""
    if signed_delta is None:
        return (end - start + 180.0) % 360.0 - 180.0
    if (
        isinstance(signed_delta, bool)
        or not isinstance(signed_delta, (int, float))
        or not math.isfinite(signed_delta)
    ):
        raise ValueError("signed yaw delta must be a finite number")
    # Do not use wrap_deg here: its small-angle snapping hides mismatches.
    mismatch = (start + signed_delta - end + 180.0) % 360.0 - 180.0
    if abs(mismatch) > 1e-6:
        raise ValueError("signed yaw delta disagrees with endpoint yaw")
    return float(signed_delta)


def relative_to_world(
    yaw_deg: float, dx: float, dy: float, dz: float
) -> tuple[float, float, float]:
    """Rotate a body-frame offset into the world frame.

    At yaw=90 deg a forward offset points to world +y (right), which is the
    expected right turn for a clockwise-positive yaw.
    """
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    return (dx * c - dy * s, dx * s + dy * c, dz)


def _finite_number(value: float) -> bool:
    """Reject bools, non-numeric values and non-finite numbers."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class WorldPose:
    x: float
    y: float
    z: float
    yaw_deg: float

    def offset(
        self,
        dx: float,
        dy: float,
        dz: float = 0.0,
        dyaw_deg: float = 0.0,
    ) -> "WorldPose":
        wx, wy, wz = relative_to_world(self.yaw_deg, dx, dy, dz)
        return WorldPose(
            self.x + wx,
            self.y + wy,
            self.z + wz,
            wrap_deg(self.yaw_deg + dyaw_deg),
        )

    def with_yaw(self, yaw_deg: float) -> "WorldPose":
        return WorldPose(self.x, self.y, self.z, wrap_deg(yaw_deg))

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.yaw_deg)

    def at_height(self, z: float) -> "WorldPose":
        """Return the same pose clamped to a fixed world z (ground vehicle)."""
        return WorldPose(self.x, self.y, float(z), self.yaw_deg)


@dataclass(frozen=True)
class RelativePoseDelta:
    """Body-frame motion request, matching the trajectory model output."""

    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    dyaw_deg: float = 0.0

    def __post_init__(self) -> None:
        values = (self.dx, self.dy, self.dz, self.dyaw_deg)
        if not all(_finite_number(value) for value in values):
            raise ValueError("relative pose contains a non-finite value")

    def apply(self, pose: WorldPose) -> WorldPose:
        return pose.offset(self.dx, self.dy, self.dz, self.dyaw_deg)

    def without_vertical(self) -> "RelativePoseDelta":
        """Drop dz for ground vehicles that cannot change altitude."""
        return RelativePoseDelta(self.dx, self.dy, 0.0, self.dyaw_deg)

    def is_zero(self) -> bool:
        return (
            self.dx == 0.0
            and self.dy == 0.0
            and self.dz == 0.0
            and self.dyaw_deg == 0.0
        )