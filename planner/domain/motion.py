from dataclasses import dataclass, fields
import math

from .trajectory import MotionSegment, MotionTrajectory


@dataclass(frozen=True)
class MotionLimits:
    """Shared physical limits and tracking tolerances, in meters/degrees/seconds."""

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
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"motion.{field.name} must be finite and positive")
        if not 1 <= self.control_hz <= 50:
            raise ValueError("motion.control_hz must be between 1 and 50")

__all__ = ["MotionSegment", "MotionTrajectory", "MotionLimits"]

