"""Simulator-independent synchronized translation/yaw profiles."""
from dataclasses import dataclass
import math

from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose, wrap_yaw_deg
from planner.domain.trajectory import MotionSegment, MotionTrajectory, WorldTrajectory

SMOOTH_PROFILE = "synchronized_quintic"

"""
同步平滑运动规划器里的核心物理计算函数,计算从一个航点飞到下一个航点，至少应该安排多少秒
"""
def duration_for(start: WorldPose, end: WorldPose, speed: float, limits: MotionLimits) -> float:
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("motion speed must be finite and positive")
    distance = math.dist((start.x, start.y, start.z), (end.x, end.y, end.z))
    angle = abs(wrap_yaw_deg(end.yaw_deg - start.yaw_deg))
    # Max derivatives of 10u^3 - 15u^4 + 6u^5: 1.875 and 10/sqrt(3).
    peak_accel = 10 / math.sqrt(3)#峰值加速度
    return max(
        limits.min_duration_s, 1.875 * distance / speed,
        1.875 * angle / limits.max_yaw_rate_deg_s,
        math.sqrt(peak_accel * distance / limits.max_acceleration_mps2),
        math.sqrt(peak_accel * angle / limits.max_yaw_acceleration_deg_s2),
    )


@dataclass(frozen=True)
class MotionSample:
    pose: WorldPose
    velocity: tuple[float, float, float]
    yaw_rate_deg_s: float


def sample_segment(segment: MotionSegment, elapsed: float) -> MotionSample:
    duration = segment.duration_s
    if duration is None or not math.isfinite(duration) or duration <= 0:
        raise ValueError("smooth segment requires a positive finite duration")
    u = min(1.0, max(0.0, elapsed / duration))
    progress = 10*u**3 - 15*u**4 + 6*u**5
    rate = 30*u**2 * (1-u)**2 / duration
    a, b = segment.start, segment.end
    delta = (b.x-a.x, b.y-a.y, b.z-a.z)
    yaw_delta = wrap_yaw_deg(b.yaw_deg-a.yaw_deg)
    return MotionSample(
        WorldPose(a.x+delta[0]*progress, a.y+delta[1]*progress, a.z+delta[2]*progress,
                  wrap_yaw_deg(a.yaw_deg+yaw_delta*progress)),
        tuple(value*rate for value in delta), yaw_delta*rate,
    )

#同步平滑运动规划器
class SynchronizedMotionPlanner:
    """在每个航点处停稳，且平移与偏航在同一个平滑周期内同步完成"""

    def __init__(self, speed_mps: float, limits: MotionLimits):
        self.speed_mps = speed_mps
        self.limits = limits

    def create_motion(self, trajectory: WorldTrajectory) -> MotionTrajectory:
        previous = trajectory.start
        segments = []
        for pose in trajectory.poses:
            segments.append(MotionSegment(
                start=previous,
                end=pose,
                profile=SMOOTH_PROFILE,
                duration_s=duration_for(previous, pose, self.speed_mps, self.limits),
                target_speed_mps=self.speed_mps,
            ))
            previous = pose
        return MotionTrajectory(tuple(segments))
