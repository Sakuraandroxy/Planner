from __future__ import annotations

from dataclasses import dataclass

from .pose import RelativePoseDelta, WorldPose


@dataclass(frozen=True)
class RelativeTrajectory:
    #相对增量轨迹
    points: tuple[RelativePoseDelta, ...]


@dataclass(frozen=True)
class WorldTrajectory:
    #世界坐标轨迹
    start: WorldPose
    poses: tuple[WorldPose, ...]


@dataclass(frozen=True)
class MotionSegment:
    #运动段，表示从一个航点到下一个航点的飞行过程
    start: WorldPose
    end: WorldPose
    profile: str
    duration_s: float | None = None #执行时长，单位秒
    target_speed_mps: float | None = None


@dataclass(frozen=True)
class MotionTrajectory:
    #由多个可执行运动段组成的完整运动轨迹
    segments: tuple[MotionSegment, ...]

