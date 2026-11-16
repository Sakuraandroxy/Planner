"""The streamed state carries the measured yaw rate the heading servo damps with."""
import pytest

from planner.domain.pose import WorldPose
from planner.domain.streaming import TimedVehicleState


def test_yaw_rate_defaults_to_none_and_accepts_a_measurement():
    pose = WorldPose(1.0, 2.0, -3.0, 45.0)
    assert TimedVehicleState(pose, 1_000_000, 0.0).yaw_rate_deg_s is None
    state = TimedVehicleState(pose, 1_000_000, 0.0, yaw_rate_deg_s=-12.5)
    assert state.yaw_rate_deg_s == pytest.approx(-12.5)


def test_yaw_rate_must_be_finite_when_supplied():
    pose = WorldPose(1.0, 2.0, -3.0, 45.0)
    with pytest.raises(ValueError, match="finite"):
        TimedVehicleState(pose, 1_000_000, 0.0, yaw_rate_deg_s=float("nan"))
    with pytest.raises(ValueError, match="finite"):
        TimedVehicleState(pose, 1_000_000, 0.0, yaw_rate_deg_s=True)
