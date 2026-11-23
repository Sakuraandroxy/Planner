"""越点走廊：按运动模型的默认值，配置只能抬高（裁决 R6、R17）。"""

from __future__ import annotations

from planner.domain import MotionLimits, MotionModel

#: 阿克曼车冲过端点后可容忍的横向残差，车宽量级。
#:
#: 出处：经验值，未标定。裁决 R17 把它从 0.6 抬到 1.0，理由是 0.6 为手工挑值、
#: 实测失败率偏高；1.0 是量级依据，不是标定结果，代码库里没有对应的标定记录。
#: 影响因素：车宽与轴距、最小转弯半径、段末航向误差、``position_tolerance_m``、
#: 段周期（周期越粗，越点判据拿到残差的时机越稀）。标定方法：固定路线重复跑越点
#: 判据，记录冲过端点时的横向残差分布，取上界。
ACKERMANN_PASSED_LATERAL_TOLERANCE_M = 1.0


def corridor_default(model: MotionModel, limits: MotionLimits) -> float:
    """Corridor a motion model gets before configuration is applied."""
    if model is MotionModel.ACKERMANN:
        return ACKERMANN_PASSED_LATERAL_TOLERANCE_M
    if model is MotionModel.HOLONOMIC:
        return limits.position_tolerance_m
    raise ValueError("corridor default needs a MotionModel")


def passed_lateral_corridor(model: MotionModel, limits: MotionLimits) -> float:
    """Configured corridor, raised to the model default when it is narrower."""
    return max(limits.passed_lateral_tolerance_m, corridor_default(model, limits))
