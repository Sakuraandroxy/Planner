import math
from dataclasses import dataclass, fields

from .trajectory import MotionSegment, MotionTrajectory


@dataclass(frozen=True)
class ContinuousTrackingSettings:
    """Streaming lookahead and smoothing; jerk limits also serve position hold."""

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
    #: 位置保持的航向比例增益。与 ``yaw_gain`` 分开是因为两者面对的对象不同：
    #: 飞行路径跟踪一个有前馈的参考，保持环只对着一个固定角度，后者对执行链
    #: 延迟更敏感（执行链实测见 ``docs/archive/measurements.md`` 的「偏航执行链」）。
    heading_hold_gain: float = 0.4
    #: 航向阻尼系数，乘以实测偏航角速度后从指令中扣除，抵消执行器滞后。
    #: 执行链延迟超过 90° 相位时它会变成助推，那种情况下应当取零。
    heading_rate_damping: float = 0.0
    #: 航向死区，误差与实测角速度都小于阈值时不再输出指令。
    heading_deadband_deg: float = 0.5
    heading_rate_deadband_dps: float = 1.5

    #: 允许取零的参数：零表示关掉该项，其余参数必须为正。
    OPTIONAL = frozenset(
        {"heading_rate_damping", "heading_deadband_deg", "heading_rate_deadband_dps"}
    )

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f"motion.{field.name} must be finite numeric")
            if value < 0:
                raise ValueError(f"motion.{field.name} must be non-negative")
            if value == 0 and field.name not in self.OPTIONAL:
                raise ValueError(f"motion.{field.name} must be positive")
        if not 1 <= self.control_hz <= 50:
            raise ValueError("motion.control_hz must be between 1 and 50")

__all__ = ["ContinuousReference", "ContinuousTrackingSettings", "MotionLimits", "MotionSegment", "MotionTrajectory"]