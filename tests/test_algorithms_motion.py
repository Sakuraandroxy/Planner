"""算法层运动律入口：模长限幅与越点走廊取值。"""

import pytest

from planner.algorithms import bounded, corridor_default, passed_lateral_corridor
from planner.domain import MotionLimits, MotionModel


def test_bounded_scales_only_above_the_limit():
    assert bounded((3.0, 4.0), 2.0) == pytest.approx((1.2, 1.6))
    assert bounded((0.6, 0.8), 2.0) == (0.6, 0.8)


def test_bounded_keeps_the_zero_vector():
    assert bounded((0.0, 0.0, 0.0), 2.0) == (0.0, 0.0, 0.0)


def test_corridor_default_rejects_an_unknown_model():
    with pytest.raises(ValueError):
        corridor_default("ackermann", MotionLimits())


@pytest.mark.parametrize(
    "configured, ackermann, holonomic",
    [(0.2, 1.0, 0.3), (0.3, 1.0, 0.3), (1.5, 1.5, 1.5)],
)
def test_configuration_only_raises_the_corridor(configured, ackermann, holonomic):
    limits = MotionLimits(passed_lateral_tolerance_m=configured)
    assert passed_lateral_corridor(MotionModel.ACKERMANN, limits) == pytest.approx(ackermann)
    assert passed_lateral_corridor(MotionModel.HOLONOMIC, limits) == pytest.approx(holonomic)
