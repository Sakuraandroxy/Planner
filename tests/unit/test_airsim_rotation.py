import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from planner.adapters.airsim.vehicle import AirSimVehicle
from planner.domain.pose import WorldPose
from planner.domain.trajectory import MotionSegment, MotionTrajectory
from planner.errors import ExecutionError
from planner.services.execution import ExecutionService


def pose(yaw):
    return WorldPose(28, -10, -14, yaw)


@pytest.mark.parametrize("start,target,actual", [(43.76, -46.24, -46.0), (170, -170, -171)])
def test_pure_rotation_uses_absolute_yaw(start, target, actual):
    connection = Mock()
    vehicle = AirSimVehicle(connection, 2, 60)
    vehicle.current_pose = Mock(side_effect=[pose(start), pose(actual)])
    vehicle.execute_segment(MotionSegment(pose(start), pose(target)))
    connection.call_async_and_wait.assert_called_once_with(
        "rotateToYawAsync", pytest.approx(target), timeout_sec=60, margin=2.0,
    )


def test_failed_rotation_cancels_and_hovers():
    connection = Mock()
    vehicle = AirSimVehicle(connection, 2, 60)
    vehicle.current_pose = Mock(return_value=pose(40))
    trajectory = MotionTrajectory((MotionSegment(pose(40), pose(-50)),))
    with pytest.raises(ExecutionError, match="rotation did not reach"):
        ExecutionService(vehicle).execute(trajectory)
    connection.call.assert_called_once_with("cancelLastTask")
    assert connection.call_async_and_wait.call_args.args == ("hoverAsync",)


def test_yaw_wrap_already_reached():
    connection = Mock()
    vehicle = AirSimVehicle(connection, 2, 60)
    vehicle.current_pose = Mock(return_value=pose(179.5))
    vehicle.execute_segment(MotionSegment(pose(170), pose(-180)))
    connection.call_async_and_wait.assert_not_called()


def test_translation_keeps_independent_yaw(monkeypatch):
    yaw_mode = Mock()
    monkeypatch.setitem(sys.modules, "airsim", SimpleNamespace(
        DrivetrainType=SimpleNamespace(MaxDegreeOfFreedom=0), YawMode=yaw_mode,
    ))
    connection = Mock()
    vehicle = AirSimVehicle(connection, 2, 60)
    vehicle.execute_segment(MotionSegment(pose(0), WorldPose(29, -9, -14, 45)))
    assert connection.call_async_and_wait.call_args.args == ("moveToPositionAsync", 29, -9, -14, 2)
    yaw_mode.assert_called_once_with(is_rate=False, yaw_or_rate=45)
