"""Path following hands the whole path to the engine, with the baseline's parameters.

Both adapters have to pass one world-frame path down to the engine's own path
command and wait for it, so these tests pin the parameters item by item and check
that no per-point ``moveToPosition`` call happens instead. The numbers are the
3DG-VLN baseline's (``airsim_plugin/AirVLNSimulatorClientTool.py:365-388``).
"""

import asyncio
import sys
import types

import pytest

from planner.domain.pose import WorldPose
from planner.errors import ExecutionError
from planner.simulators.airsim.vehicle import AirSimVehicle
from planner.simulators.projectairsim.drone import ProjectAirSimDrone
from planner.simulators.projectairsim.rover import ProjectAirSimRover
from tests.test_projectairsim_drone import (
    CAMERA as DRONE_CAMERA,
    FakeClock,
    FakeConnection as DroneConnection,
    FakeRobot as DroneRobot,
)
from tests.test_projectairsim_rover import (
    CAMERA as ROVER_CAMERA,
    FakeConnection as RoverConnection,
    FakeRobot as RoverRobot,
)

PATH = [WorldPose(1.0, 2.0, 3.0, 0.0), WorldPose(4.0, 5.0, 6.0, 0.0)]


# -- AirSim (UE4) ---------------------------------------------------------
class _Connection:
    def __init__(self, ack=True):
        self.ack = ack
        self.issued: list[tuple] = []

    def call_async_and_wait(self, name, *args, **kwargs):
        self.issued.append((name, args, kwargs))
        return self.ack

    def call(self, name, *args, **kwargs):
        return object()


class _Holding:
    def __init__(self):
        self.released = 0

    def release(self):
        self.released += 1


@pytest.fixture(autouse=True)
def fake_airsim(monkeypatch):
    module = types.ModuleType("airsim")
    module.DrivetrainType = types.SimpleNamespace(ForwardOnly=1, MaxDegreeOfFreedom=0)
    module.YawMode = lambda **kwargs: kwargs
    module.Vector3r = lambda x, y, z: (x, y, z)
    monkeypatch.setitem(sys.modules, "airsim", module)


def _airsim_vehicle(*, ack=True):
    connection = _Connection(ack)
    vehicle = AirSimVehicle(connection, speed_mps=2.0, timeout_s=1.0)
    vehicle.holding = _Holding()
    return vehicle, connection


def test_airsim_hands_the_whole_path_to_move_on_path():
    vehicle, connection = _airsim_vehicle()
    vehicle.follow_path(PATH, 1.0, timeout_s=30.0)
    assert [name for name, _, _ in connection.issued] == ["moveOnPathAsync"]
    _, args, kwargs = connection.issued[0]
    assert args[0] == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    assert args[1] == 1.0
    # Every value the baseline passes, plus the caller's deadline as the
    # engine-side timeout (the baseline leaves AirSim's 3e38 default in place).
    assert kwargs == {
        "timeout_sec": 30.0,
        "drivetrain": 1,
        "yaw_mode": {"is_rate": False},
        "lookahead": 3,
        "adaptive_lookahead": 1,
    }
    assert vehicle.holding.released == 1


def test_airsim_never_falls_back_to_point_commands():
    vehicle, connection = _airsim_vehicle()
    vehicle.follow_path(PATH, 1.0, timeout_s=30.0)
    names = [name for name, _, _ in connection.issued]
    assert "moveToPositionAsync" not in names
    assert "moveByVelocityAsync" not in names


def test_airsim_reports_a_path_the_engine_did_not_finish():
    vehicle, _ = _airsim_vehicle(ack=False)
    with pytest.raises(ExecutionError, match=r"inside 30s"):
        vehicle.follow_path(PATH, 1.0, timeout_s=30.0)


# -- ProjectAirSim (UE5) --------------------------------------------------
class _PathRobotMixin:
    """A native path command whose response is a future, like the real client."""

    def __init__(self):
        super().__init__()
        self.path_calls: list[dict] = []
        self.path_result = True
        self.path_never_completes = False

    async def move_on_path_async(self, path, velocity, timeout_sec=3e38, **kwargs):
        self.path_calls.append(
            {"path": [list(point) for point in path], "velocity": velocity,
             "timeout_sec": timeout_sec, **kwargs}
        )
        future = asyncio.get_running_loop().create_future()
        if not self.path_never_completes:
            future.set_result(self.path_result)
        return future

    async def move_to_position_async(self, *args, **kwargs):
        raise AssertionError("path following must not use point commands")


class PathRobot(_PathRobotMixin, DroneRobot):
    pass


class PathRover(_PathRobotMixin, RoverRobot):
    def __init__(self):
        super().__init__()
        self.cancels = 0

    def cancel_last_task(self):
        self.cancels += 1
        return True


def _drone(robot):
    clock = FakeClock()
    return ProjectAirSimDrone(
        DroneConnection(robot),
        robot,
        camera=DRONE_CAMERA,
        camera_id="FrontCamera",
        clock=clock,
        sleep=clock.sleep,
    )


def _rover(robot):
    connection = RoverConnection()
    return ProjectAirSimRover(connection, robot, camera=ROVER_CAMERA), connection


def test_projectairsim_drone_follows_the_path_with_the_baseline_parameters():
    robot = PathRobot()
    drone = _drone(robot)
    drone.start()
    drone.follow_path(PATH, 1.0, timeout_s=45.0)
    assert len(robot.path_calls) == 1
    call = robot.path_calls[0]
    assert call["path"] == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    assert call["velocity"] == 1.0
    assert call["timeout_sec"] == 45.0
    # ForwardOnly / absolute yaw with the baseline's lookahead, which is what
    # makes the drone face the direction of travel instead of a fixed heading.
    assert call["yaw_control_mode"] == 1
    assert call["yaw_is_rate"] is False
    assert call["yaw"] == 0.0
    assert call["lookahead"] == 3
    assert call["adaptive_lookahead"] == 1
    assert robot.commands == [], "no per-period velocity command during follow_path"
    drone.close()


def test_projectairsim_drone_reports_a_refused_path():
    robot = PathRobot()
    robot.path_result = False
    drone = _drone(robot)
    drone.start()
    with pytest.raises(ExecutionError, match="refused the path"):
        drone.follow_path(PATH, 1.0, timeout_s=45.0)
    drone.close()


def test_projectairsim_drone_reports_a_path_that_never_finishes():
    robot = PathRobot()
    robot.path_never_completes = True
    drone = _drone(robot)
    drone.start()
    with pytest.raises(ExecutionError, match=r"did not complete in 0\.05s"):
        drone.follow_path(PATH, 1.0, timeout_s=0.05)
    assert robot.cancels == 1, "the native path has to be stopped"
    drone.close()


def test_projectairsim_rover_follows_the_path_with_the_baseline_lookahead():
    robot = PathRover()
    rover, _ = _rover(robot)
    rover.start()
    rover.follow_path(PATH, 1.0, timeout_s=45.0)
    call = robot.path_calls[0]
    assert call["path"] == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    assert call["velocity"] == 1.0
    assert call["timeout_sec"] == 45.0
    # Same lookahead pair as the drone; this package's rover walks the waypoints
    # through MoveToPosition, so timeout_sec is per segment rather than per path.
    assert call["lookahead"] == 3
    assert call["adaptive_lookahead"] == 1
    rover.close()


def test_projectairsim_rover_cancels_and_brakes_a_path_that_never_finishes():
    robot = PathRover()
    robot.path_never_completes = True
    rover, connection = _rover(robot)
    rover.start()
    with pytest.raises(ExecutionError, match=r"did not complete in 0\.05s"):
        rover.follow_path(PATH, 1.0, timeout_s=0.05)
    assert robot.cancels == 1, "the native path has to be cancelled"
    assert connection.control_requests()[-1] == {
        "engine": 0.0,
        "steering_angle": 0.0,
        "brake": 1.0,
    }, "the car has to be braked after the cancel"
    rover.close()


def test_an_empty_path_is_rejected_before_any_command():
    vehicle, connection = _airsim_vehicle()
    with pytest.raises(ExecutionError, match="at least one point"):
        vehicle.follow_path([], 1.0, timeout_s=1.0)
    assert connection.issued == []

    robot = PathRobot()
    drone = _drone(robot)
    drone.start()
    with pytest.raises(ExecutionError, match="at least one point"):
        drone.follow_path([], 1.0, timeout_s=1.0)
    assert robot.path_calls == []
    drone.close()
