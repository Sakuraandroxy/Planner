import pytest

from planner.domain.pose import WorldPose
from planner.domain.trajectory import MotionSegment, MotionTrajectory
from planner.errors import ExecutionError
from planner.services.execution import ExecutionService
from tests.fakes.components import FakeVehicle


def test_execution_failure_cancels_and_hovers():
    vehicle = FakeVehicle(fail=True)
    trajectory = MotionTrajectory((MotionSegment(WorldPose(0, 0, 0, 0), WorldPose(1, 0, 0, 0)),))
    with pytest.raises(ExecutionError):
        ExecutionService(vehicle).execute(trajectory)
    assert vehicle.cancelled and vehicle.hovered

