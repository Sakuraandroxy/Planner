"""段执行的合成律：参考采样、加速度限幅、偏航角速度伺服。

纯策略住在 ``runtime/motion/`` 的纯函数里（``sample_segment``、
``tracked_heading_rate_deg_s``、``duration_for``），这里只持有上一拍的期望速度
与偏航角速度，并按 ``MotionCapability`` 把世界系期望转成该载具收得下的命令。
"""

from __future__ import annotations

import math

from planner.errors import ExecutionError
from planner.domain import (
    CarCommand,
    ContinuousTrackingSettings,
    HolonomicCommand,
    MotionModel,
    RelativePoseDelta,
    wrap_yaw_deg,
)

from .heading_servo import tracked_heading_rate_deg_s
from .synchronized import sample_segment


def measured_action(start, end, yaw):
    """实测位移换算到起点航向系，附上带符号的偏航增量。"""
    angle = math.radians(start.yaw_deg)
    x, y = end.x - start.x, end.y - start.y
    return RelativePoseDelta(
        math.cos(angle) * x + math.sin(angle) * y,
        -math.sin(angle) * x + math.cos(angle) * y,
        end.z - start.z,
        yaw,
    )


def limit_vector(values, maximum):
    """把一个向量按范数限制在 ``maximum`` 内；非有限值直接拒绝。"""
    if not math.isfinite(maximum) or maximum <= 0 or not all(
        math.isfinite(v) for v in values
    ):
        raise ExecutionError("non-finite motion vector or invalid limit")
    norm = math.hypot(*values)
    if not math.isfinite(norm):
        raise ExecutionError("motion vector magnitude overflowed")
    scale = min(1.0, maximum / norm) if norm else 1.0
    return tuple(value * scale for value in values)


def command_for(capability, velocity, yaw_rate_deg_s, yaw_deg):
    """世界系期望速度转成载具命令类型。

    阿克曼车只能沿车头方向走，世界系速度投影成带符号的前向速度；全向载具直接
    收世界系三分量。类型选错由引擎的 ``stream_command`` 拒绝，不在这里兜底。
    """
    if capability.model is MotionModel.ACKERMANN:
        angle = math.radians(yaw_deg)
        forward = velocity[0] * math.cos(angle) + velocity[1] * math.sin(angle)
        return CarCommand(forward, yaw_rate_deg_s, forward < 0)
    return HolonomicCommand(*velocity, yaw_rate_deg_s)


class SegmentSynthesizer:
    """一段五次曲线的期望命令，按拍推进。"""

    def __init__(self, limits, tracking: ContinuousTrackingSettings | None = None):
        self.limits = limits
        self.tracking = tracking or ContinuousTrackingSettings()
        self.velocity = (0.0, 0.0, 0.0)
        self.yaw_rate = 0.0

    def reset(self) -> None:
        self.velocity, self.yaw_rate = (0.0, 0.0, 0.0), 0.0

    def command(self, segment, elapsed: float, pose, period: float):
        """参考位姿在 ``elapsed`` 处的期望世界速度与偏航角速度。"""
        limits = self.limits
        reference = sample_segment(segment, elapsed)
        error = (
            reference.pose.x - pose.x,
            reference.pose.y - pose.y,
            reference.pose.z - pose.z,
        )
        desired = limit_vector(
            tuple(
                v + limits.position_gain * e
                for v, e in zip(reference.velocity, error)
            ),
            segment.target_speed_mps,
        )
        change = limit_vector(
            tuple(v - p for v, p in zip(desired, self.velocity)),
            limits.max_acceleration_mps2 * period,
        )
        self.velocity = tuple(p + d for p, d in zip(self.velocity, change))
        # 这条路径拿不到实测偏航角速度（只读位姿），因此关闭阻尼项：
        # 用位姿差分做阻尼会把抖动放大成指令。大误差的超调交给刹车曲线。
        self.yaw_rate = tracked_heading_rate_deg_s(
            reference.yaw_rate_deg_s,
            wrap_yaw_deg(reference.pose.yaw_deg - pose.yaw_deg),
            last_rate_dps=self.yaw_rate,
            period_s=period,
            gain=limits.yaw_gain,
            maximum_rate_dps=limits.max_yaw_rate_deg_s,
            acceleration_dps2=limits.max_yaw_acceleration_deg_s2,
            jerk_dps3=self.tracking.max_yaw_jerk_deg_s3,
        )
        return self.velocity, self.yaw_rate
