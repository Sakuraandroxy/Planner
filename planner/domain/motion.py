from dataclasses import dataclass, fields
import math

from .trajectory import MotionSegment, MotionTrajectory


@dataclass(frozen=True)
class ContinuousTrackingSettings:
    """Local streaming reference settings; do not affect stopped/basic motion."""

    lookahead_time_s: float = 0.6
    min_lookahead_m: float = 0.3
    max_lookahead_m: float = 1.5
    max_jerk_mps3: float = 3.0
    max_yaw_jerk_deg_s3: float = 60.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"continuous.{field.name} must be finite and positive")
        if self.min_lookahead_m > self.max_lookahead_m:
            raise ValueError("continuous minimum lookahead exceeds maximum")


@dataclass(frozen=True)
class ContinuousReference:
    position: tuple[float, float, float]
    distance_m: float
    remaining_head_m: float
    segment_length_m: float
    progress_m: float
    aligning: bool
    pure_rotation: bool


@dataclass(frozen=True)
class MotionLimits:
    """通用的物理运动限制与跟踪容差，单位为米 / 度 / 秒"""

    max_acceleration_mps2: float = 1.5
    max_yaw_rate_deg_s: float = 30.0
    max_yaw_acceleration_deg_s2: float = 30.0
    control_hz: float = 20.0
    position_gain: float = 1.0
    yaw_gain: float = 2.0
    position_tolerance_m: float = 0.3
    yaw_tolerance_deg: float = 2.0
    stopped_speed_mps: float = 0.15
    stopped_yaw_rate_deg_s: float = 2.0
    settle_time_s: float = 0.3
    min_duration_s: float = 1.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"motion.{field.name} must be finite and positive")
        if not 1 <= self.control_hz <= 50:
            raise ValueError("motion.control_hz must be between 1 and 50")

__all__ = ["MotionSegment", "MotionTrajectory", "MotionLimits", "ContinuousTrackingSettings", "ContinuousReference"]

