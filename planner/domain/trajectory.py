from __future__ import annotations

from dataclasses import dataclass

from .pose import RelativePoseDelta, WorldPose


@dataclass(frozen=True)
class RelativeTrajectory:
    points: tuple[RelativePoseDelta, ...]


@dataclass(frozen=True)
class WorldTrajectory:
    start: WorldPose
    poses: tuple[WorldPose, ...]


@dataclass(frozen=True)
class MotionSegment:
    start: WorldPose
    end: WorldPose
    duration_s: float | None = None
    target_speed_mps: float | None = None
    profile: str = "linear"


@dataclass(frozen=True)
class MotionTrajectory:
    segments: tuple[MotionSegment, ...]

