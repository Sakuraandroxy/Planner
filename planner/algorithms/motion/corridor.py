"""越点走廊：按运动模型的默认值，配置只能抬高（裁决 R6、R17）。"""

from __future__ import annotations

from planner.domain import MotionLimits, MotionModel

#: 阿克曼车冲过端点后可容忍的横向残差，车宽量级；裁决 R17 从 0.6 抬到 1.0，
#: 理由是 0.6 为手工挑值、实测失败率偏高。这个数是量级依据，不是标定值。
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
