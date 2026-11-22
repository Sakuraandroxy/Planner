import math
from dataclasses import dataclass, field, fields
from enum import Enum

from .trajectory import MotionSegment, MotionTrajectory


class MotionModel(str, Enum):
    """How a vehicle moves: free translation, or forward-only with steering."""

    HOLONOMIC = "holonomic"
    ACKERMANN = "ackermann"


@dataclass(frozen=True)
class MotionCapability:
    """What one vehicle's kinematics allow; computed once at construction."""

    model: MotionModel
    max_lateral_speed_mps: float
    min_turn_radius_m: float | None
    allows_reverse: bool

    def __post_init__(self):
        if not isinstance(self.model, MotionModel):
            raise ValueError("motion capability model must be a MotionModel")
        speed = self.max_lateral_speed_mps
        if (
            isinstance(speed, bool)
            or not isinstance(speed, (int, float))
            or not math.isfinite(speed)
            or speed < 0
        ):
            raise ValueError(
                "motion capability max_lateral_speed_mps must be finite and non-negative"
            )
        if not isinstance(self.allows_reverse, bool):
            raise ValueError("motion capability allows_reverse must be a boolean")
        if self.model is MotionModel.ACKERMANN:
            if speed != 0:
                raise ValueError("an ackermann vehicle cannot move laterally")
            radius = self.min_turn_radius_m
            if (
                isinstance(radius, bool)
                or not isinstance(radius, (int, float))
                or not math.isfinite(radius)
                or radius <= 0
            ):
                raise ValueError(
                    "an ackermann vehicle requires a positive min_turn_radius_m"
                )
            return
        if speed <= 0:
            raise ValueError(
                "a holonomic vehicle requires a positive max_lateral_speed_mps"
            )
        if self.min_turn_radius_m is not None:
            raise ValueError("a holonomic vehicle has no min_turn_radius_m")


@dataclass(frozen=True)
class HolonomicCommand:
    """World-frame velocity and yaw rate for one control period."""

    vx: float
    vy: float
    vz: float
    yaw_rate_deg_s: float


@dataclass(frozen=True)
class CarCommand:
    """Signed forward speed and yaw rate for one control period."""

    forward_speed_mps: float
    yaw_rate_deg_s: float
    reverse: bool

    def __post_init__(self):
        if not isinstance(self.reverse, bool):
            raise ValueError("car command reverse must be a boolean")
        if self.reverse is not (self.forward_speed_mps < 0):
            raise ValueError(
                "car command reverse must match the sign of forward_speed_mps"
            )


#: 一台载具接受哪种命令由它的 ``MotionModel`` 决定，联合类型只用于标注。
MotionCommand = HolonomicCommand | CarCommand

_COMMAND_FOR_MODEL = {
    MotionModel.HOLONOMIC: HolonomicCommand,
    MotionModel.ACKERMANN: CarCommand,
}


def command_type_for(capability: MotionCapability) -> type:
    """The command class a vehicle with this capability accepts."""
    return _COMMAND_FOR_MODEL[capability.model]


def require_supported(command: MotionCommand, capability: MotionCapability) -> None:
    """Reject a command this vehicle cannot execute."""
    expected = command_type_for(capability)
    if not isinstance(command, expected):
        raise ValueError(
            f"{type(command).__name__} cannot drive a {capability.model.value} vehicle"
        )
    if isinstance(command, CarCommand) and command.reverse and not capability.allows_reverse:
        raise ValueError("this vehicle does not support reversing")


@dataclass(frozen=True)
class CollisionRecoverySettings:
    """接触后退避的距离与预算；第 n 次连续恢复后退 ``backoff_m * n``。

    ``reset_after_progress_m`` 不参与单次退避距离，只在累计前进超过它之后把连续
    次数归零：无界任务里要防的是卡在同一个障碍物前，不是限制任务总时长。
    """

    backoff_m: float = 3.0
    maximum_recoveries: int = 3
    reset_after_progress_m: float = 20.0

    def __post_init__(self) -> None:
        for name in ("backoff_m", "reset_after_progress_m"):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"collision {name} must be finite and positive")
        if type(self.maximum_recoveries) is not int or self.maximum_recoveries < 1:
            raise ValueError("collision maximum_recoveries must be a positive integer")
        if self.reset_after_progress_m <= self.backoff_m:
            raise ValueError(
                "collision reset_after_progress_m must exceed backoff_m"
            )


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
class HeadingHoldSettings:
    """位置保持的航向环参数，只由 ``runtime/position_hold.py`` 读。"""

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
        for item in fields(self):
            value = getattr(self, item.name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f"motion.heading.{item.name} must be finite numeric")
            if value < 0:
                raise ValueError(f"motion.heading.{item.name} must be non-negative")
            if value == 0 and item.name not in self.OPTIONAL:
                raise ValueError(f"motion.heading.{item.name} must be positive")


@dataclass(frozen=True)
class MotionLimits:
    """物理运动包络与跟踪容差，单位为米 / 度 / 秒。

    配置侧与跟踪侧的字段合在这一份里：``planner/config.py`` 从
    ``config/core/<profile>/base.yaml`` 的 ``motion`` 段构造它，规划器、跟踪器与
    位置保持直接读它，不再有转换对象。
    """

    max_speed_mps: float = 3.0
    max_acceleration_mps2: float = 1.5
    max_yaw_rate_deg_s: float = 30.0
    max_yaw_acceleration_deg_s2: float = 60.0
    control_hz: float = 20.0
    position_gain: float = 1.0
    yaw_gain: float = 2.0
    position_tolerance_m: float = 0.3
    yaw_tolerance_deg: float = 2.0
    #: “已驶过端点”判据允许的横向走廊。全向载具（无人机）能直接压到点上，
    #: 用 position_tolerance_m 即可；阿克曼车不能横移，冲过端点时只能带着
    #: 横向残差回头，走廊太窄会让队伍头永远弹不掉 → 目标落到身后 → 倒车 →
    #: 原地卡死。默认与 position 相同以保持全向行为不变，无人车适配层放宽。
    passed_lateral_tolerance_m: float = 0.3
    stopped_speed_mps: float = 0.15
    stopped_yaw_rate_deg_s: float = 2.0
    settle_time_s: float = 0.3
    min_duration_s: float = 1.0
    heading: HeadingHoldSettings = field(default_factory=HeadingHoldSettings)

    def __post_init__(self):
        if not isinstance(self.heading, HeadingHoldSettings):
            raise ValueError("motion.heading must be HeadingHoldSettings")
        for item in fields(self):
            if item.name == "heading":
                continue
            value = getattr(self, item.name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f"motion.{item.name} must be finite numeric")
            if value < 0:
                raise ValueError(f"motion.{item.name} must be non-negative")
            if value == 0:
                raise ValueError(f"motion.{item.name} must be positive")
        if not 1 <= self.control_hz <= 50:
            raise ValueError("motion.control_hz must be between 1 and 50")

__all__ = [
    "CarCommand",
    "ContinuousReference",
    "ContinuousTrackingSettings",
    "HeadingHoldSettings",
    "HolonomicCommand",
    "MotionCapability",
    "MotionCommand",
    "MotionLimits",
    "MotionModel",
    "MotionSegment",
    "MotionTrajectory",
    "command_type_for",
    "require_supported",
]