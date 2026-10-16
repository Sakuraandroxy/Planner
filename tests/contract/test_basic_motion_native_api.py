"""Native basic-motion commands; no AirSim server or real model calls."""
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from planner.adapters.airsim.vehicle import AirSimVehicle
from planner.domain.pose import WorldPose


@pytest.fixture
def vehicle(monkeypatch):
    monkeypatch.setitem(sys.modules, "airsim", SimpleNamespace(
        DrivetrainType=SimpleNamespace(MaxDegreeOfFreedom=0),
        YawMode=lambda **kwargs: SimpleNamespace(**kwargs)))
    result = AirSimVehicle(Mock(), 2, 120)
    result.motion_tracker.execute = Mock(side_effect=AssertionError("checked tracker invoked"))
    return result


def test_ascend_sends_native_position_target_without_observation_or_collision(vehicle):
    vehicle.execute_basic_motion(WorldPose(3, 4, -1.675, 0), WorldPose(3, 4, -11.675, 0), 0)
    call, hover = vehicle.connection.call_async_and_wait.call_args_list
    assert call.args == ("moveToPositionAsync", 3, 4, -11.675, 2)
    assert call.kwargs["yaw_mode"].is_rate and call.kwargs["yaw_mode"].yaw_or_rate == 0
    assert hover.args == ("hoverAsync",)
    vehicle.connection.call.assert_not_called()


@pytest.mark.parametrize("angle", [-450, -180, 180, 450])
def test_pure_turn_preserves_requested_sign_and_full_rotations(vehicle, angle):
    vehicle.execute_basic_motion(WorldPose(0, 0, -4, 0), WorldPose(0, 0, -4, angle), angle)
    call = vehicle.connection.call_async_and_wait.call_args_list[0]
    assert call.args[0] == "rotateByYawRateAsync"
    assert call.args[1]*call.args[2] == pytest.approx(angle)
    vehicle.connection.call.assert_not_called()


def test_simultaneous_translation_and_yaw_use_world_velocity_and_signed_yaw_rate(vehicle):
    vehicle.execute_basic_motion(WorldPose(1, 2, -4, 90), WorldPose(-1, 7, -14, 105), 15)
    call = vehicle.connection.call_async_and_wait.call_args_list[0]
    assert call.args[0] == "moveByVelocityAsync"
    duration = call.args[4]
    assert tuple(v*duration for v in call.args[1:4]) == pytest.approx((-2, 5, -10))
    assert call.kwargs["yaw_mode"].yaw_or_rate*duration == pytest.approx(15)
    assert call.kwargs["drivetrain"] == 0
    vehicle.connection.call.assert_not_called()
