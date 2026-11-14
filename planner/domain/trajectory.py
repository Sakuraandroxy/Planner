"""Trajectory result types."""

from __future__ import annotations

from dataclasses import dataclass

from .pose import RelativePoseDelta, WorldPose, resolve_yaw_delta_deg


@dataclass(frozen=True)
class RelativeTrajectory:
    """Ordered body-frame waypoints, each relative to the preceding pose."""

    points: tuple[RelativePoseDelta, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.points, tuple):
            raise ValueError("waypoints must be supplied as a tuple")
        if any(not isinstance(point, RelativePoseDelta) for point in self.points):
            raise ValueError("waypoints must be RelativePoseDelta values")


@dataclass(frozen=True)
class WorldTrajectory:
    """World-frame poses; optional per-segment yaw preserves +180 vs -180."""

    start: WorldPose
    poses: tuple[WorldPose, ...]
    yaw_deltas_deg: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        if self.yaw_deltas_deg is None:
            return
        if not isinstance(self.yaw_deltas_deg, tuple) or len(
            self.yaw_deltas_deg
        ) != len(self.poses):
            raise ValueError("world yaw deltas must match the trajectory segments")
        previous = self.start
        for end, delta in zip(self.poses, self.yaw_deltas_deg):
            resolve_yaw_delta_deg(previous.yaw_deg, end.yaw_deg, delta)
            if abs(delta) > 180:
                raise ValueError("world yaw step must not exceed 180 degrees")
            previous = end


@dataclass(frozen=True)
class MotionSegment:
    """One executable segment between two world poses."""

    start: WorldPose
    end: WorldPose
    profile: str
    duration_s: float | None = None
    target_speed_mps: float | None = None
    yaw_delta_deg: float | None = None

    def __post_init__(self) -> None:
        resolve_yaw_delta_deg(
            self.start.yaw_deg, self.end.yaw_deg, self.yaw_delta_deg
        )


@dataclass(frozen=True)
class MotionTrajectory:
    """Complete executable motion made of ordered segments."""

    segments: tuple[MotionSegment, ...]


@dataclass(frozen=True)
class SegmentResult:
    start: WorldPose
    end: WorldPose
    frames: int
    duration_s: float
    cancelled: bool = False