from __future__ import annotations

from dataclasses import dataclass

from .pose import RelativePoseDelta, WorldPose, resolve_yaw_delta_deg


@dataclass(frozen=True)
class RelativeTrajectory:
    #相对增量轨迹
    points: tuple[RelativePoseDelta, ...]


@dataclass(frozen=True)
class WorldTrajectory:
    #世界坐标轨迹
    start: WorldPose
    poses: tuple[WorldPose, ...]
    # Optional per-segment metadata preserves +180 vs -180 without changing
    # the external four-value relative waypoint protocol.
    yaw_deltas_deg: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        if self.yaw_deltas_deg is None:
            return
        if not isinstance(self.yaw_deltas_deg, tuple) or len(self.yaw_deltas_deg) != len(self.poses):
            raise ValueError("world yaw deltas must match the trajectory segments")
        previous = self.start
        for end, delta in zip(self.poses, self.yaw_deltas_deg):
            resolve_yaw_delta_deg(previous.yaw_deg, end.yaw_deg, delta)
            if abs(delta) > 180:
                raise ValueError("world yaw step must not exceed 180 degrees")
            previous = end


@dataclass(frozen=True)
class MotionSegment:
    #运动段，表示从一个航点到下一个航点的飞行过程
    start: WorldPose
    end: WorldPose
    profile: str
    duration_s: float | None = None #执行时长，单位秒
    target_speed_mps: float | None = None
    yaw_delta_deg: float | None = None

    def __post_init__(self) -> None:
        resolve_yaw_delta_deg(self.start.yaw_deg, self.end.yaw_deg, self.yaw_delta_deg)


@dataclass(frozen=True)
class MotionTrajectory:
    #由多个可执行运动段组成的完整运动轨迹
    segments: tuple[MotionSegment, ...]

