"""Path following hands the whole path to the engine, with the baseline's parameters.

Both adapters pass one world-frame path down to the engine's own path command and
wait for it, so these tests pin the parameters item by item, check that no per-point
``moveToPosition`` call happens instead, and check that a path which does not
demonstrably finish is cancelled and held. The parameter values are the 3DG-VLN
baseline's (``airsim_plugin/AirVLNSimulatorClientTool.py:365-388``).
"""

import asyncio
import sys
import types

import pytest
from projectairsim.drone import YawControlMode

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
    """``AirSimConnection`` as the vehicle uses it, recording what it hands over.

    ``error`` stands in for the client-side deadline firing: the real connection
    raises it when the native call has not answered inside ``timeout_s``.
    """

    def __init__(self, ack=True, error=None):
        self.ack = ack
        self.error = error
        self.issued: list[tuple] = []

    def call_async_and_wait(self, name, *args, timeout_s=None, **kwargs):
        self.issued.append((name, args, {"timeout_s": timeout_s, **kwargs}))
        if self.error is not None:
            raise self.error
        return self.ack

    def call(self, name, *args, **kwargs):
        self.issued.append((name, args, kwargs))
        return True

    @property
    def names(self):
        return [name for name, _, _ in self.issued]


class _Holding:
    """The position-hold service, as far as path following uses it."""

    def __init__(self):
        self.released = 0
        self.started: list = []
        self.ticks = 0

    def release(self):
        self.released += 1

    def start(self, target=None):
        self.started.append(target)

    def tick(self):
        self.ticks += 1


@pytest.fixture(autouse=True)
def fake_airsim(monkeypatch):
    module = types.ModuleType("airsim")
    module.DrivetrainType = types.SimpleNamespace(ForwardOnly=1, MaxDegreeOfFreedom=0)
    module.YawMode = lambda **kwargs: kwargs
    module.Vector3r = lambda x, y, z: (x, y, z)
    monkeypatch.setitem(sys.modules, "airsim", module)


def _airsim_vehicle(*, ack=True, error=None):
    connection = _Connection(ack, error)
    vehicle = AirSimVehicle(connection, speed_mps=2.0, timeout_s=1.0)
    vehicle.holding = _Holding()
    return vehicle, connection


def test_airsim_hands_the_whole_path_to_move_on_path():
    vehicle, connection = _airsim_vehicle()
    vehicle.follow_path(PATH, 1.0, timeout_s=30.0)
    assert connection.names == ["moveOnPathAsync"]
    _, args, kwargs = connection.issued[0]
    assert args[0] == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    assert args[1] == 1.0
    # Every value the baseline passes, plus the caller's deadline twice: once as
    # the engine-side timeout and once as the client-side limit that keeps a
    # silent server from blocking the control thread.
    assert kwargs == {
        "timeout_s": 30.0,
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
    assert "moveToPositionAsync" not in connection.names
    assert "moveByVelocityAsync" not in connection.names


def test_airsim_cancels_and_holds_when_the_ack_says_the_path_did_not_finish():
    """A false ack is not proof the vehicle stopped: this build has returned
    false acks for takeoff and ``moveToPositionAsync`` while moving."""
    vehicle, connection = _airsim_vehicle(ack=False)
    with pytest.raises(ExecutionError, match=r"answered False.*inside 30s"):
        vehicle.follow_path(PATH, 1.0, timeout_s=30.0)
    assert connection.names == ["moveOnPathAsync", "cancelLastTask"]
    assert vehicle.holding.started == [None], "the hold has to be restarted"
    assert vehicle.holding.ticks == 1


def test_airsim_cancels_and_holds_when_the_client_deadline_fires():
    deadline = ExecutionError("AirSim moveOnPathAsync did not answer inside 30s")
    vehicle, connection = _airsim_vehicle(error=deadline)
    with pytest.raises(ExecutionError, match=r"inside 30s"):
        vehicle.follow_path(PATH, 1.0, timeout_s=30.0)
    assert connection.issued[0][2]["timeout_s"] == 30.0, (
        "the caller's deadline has to reach the client, not only the server"
    )
    assert connection.names == ["moveOnPathAsync", "cancelLastTask"]
    assert vehicle.holding.started == [None]
    assert vehicle.holding.ticks == 1


def test_airsim_stops_the_vehicle_when_the_transport_fails():
    """A broken transport is not an ``ExecutionError``; it must still brake."""
    from planner.errors import SimulatorConnectionError

    vehicle, connection = _airsim_vehicle(
        error=SimulatorConnectionError("AirSim transport failed")
    )
    with pytest.raises(SimulatorConnectionError):
        vehicle.follow_path(PATH, 1.0, timeout_s=30.0)
    assert connection.names == ["moveOnPathAsync", "cancelLastTask"]
    assert vehicle.holding.started == [None]


def test_a_failure_while_stopping_does_not_replace_the_original_error():
    """Stopping is best effort: the caller's reason is the one worth keeping."""

    class _Broken(_Connection):
        def call(self, name, *args, **kwargs):
            raise RuntimeError("the transport is gone")

    connection = _Broken(ack=False)
    vehicle = AirSimVehicle(connection, speed_mps=2.0, timeout_s=1.0)
    holding = _Holding()
    vehicle.holding = holding
    with pytest.raises(ExecutionError, match=r"answered False"):
        vehicle.follow_path(PATH, 1.0, timeout_s=30.0)


# -- ProjectAirSim (UE5) --------------------------------------------------
class _PathState:
    """State both native fakes keep: what was asked for, and how it ended."""

    def __init__(self):
        super().__init__()
        self.path_calls: list[dict] = []
        self.path_result = True
        self.path_never_completes = False

    def _record(self, **parameters):
        self.path_calls.append(
            {name: [list(point) for point in value]
             if name == "path" else value
             for name, value in parameters.items()}
        )
        future = asyncio.get_running_loop().create_future()
        if not self.path_never_completes:
            future.set_result(self.path_result)
        return future

    async def move_to_position_async(self, *args):
        raise AssertionError("path following must not use point commands")


class PathRobot(_PathState, DroneRobot):
    """``Drone.move_on_path_async`` with the installed package's signature."""

    async def move_on_path_async(self, path, velocity, timeout_sec=3e38,
                                 yaw_control_mode=YawControlMode.MaxDegreeOfFreedom,
                                 yaw_is_rate=True, yaw=0.0, lookahead=-1,
                                 adaptive_lookahead=1, callback=None):
        return self._record(path=path, velocity=velocity, timeout_sec=timeout_sec,
                            yaw_control_mode=yaw_control_mode,
                            yaw_is_rate=yaw_is_rate, yaw=yaw, lookahead=lookahead,
                            adaptive_lookahead=adaptive_lookahead, callback=callback)

    def cancel_last_task(self):
        self.cancels += 1
        return True


class PathRover(_PathState, RoverRobot):
    """``Rover.move_on_path_async`` with the installed package's signature."""

    def __init__(self):
        super().__init__()
        self.cancels = 0

    async def move_on_path_async(self, path, velocity, timeout_sec=3e38,
                                 yaw_rate_max=-1, lookahead=-1.0,
                                 adaptive_lookahead=1.0, callback=None):
        return self._record(path=path, velocity=velocity, timeout_sec=timeout_sec,
                            yaw_rate_max=yaw_rate_max, lookahead=lookahead,
                            adaptive_lookahead=adaptive_lookahead, callback=callback)

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
    assert call["yaw_control_mode"] == YawControlMode.ForwardOnly
    assert call["yaw_is_rate"] is False
    assert call["yaw"] == 0.0
    assert call["lookahead"] == 3
    assert call["adaptive_lookahead"] == 1
    assert call["callback"] is None
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
    assert robot.path_calls == [
        {
            "path": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
            "velocity": 1.0,
            # Same lookahead pair as the drone; this package's rover walks the
            # waypoints through MoveToPosition, so timeout_sec is per segment and
            # the caller's deadline for the batch is split across the points.
            "timeout_sec": 22.5,
            "lookahead": 3,
            "adaptive_lookahead": 1,
            "yaw_rate_max": -1,
            "callback": None,
        }
    ]
    rover.close()


def test_projectairsim_drone_cancels_a_refused_path():
    robot = PathRobot()
    robot.path_result = False
    drone = _drone(robot)
    drone.start()
    with pytest.raises(ExecutionError, match="refused the path"):
        drone.follow_path(PATH, 1.0, timeout_s=45.0)
    assert robot.cancels == 1, "a refusal may still leave the native task running"
    drone.close()


def test_projectairsim_rover_cancels_and_brakes_a_refused_path():
    robot = PathRover()
    robot.path_result = False
    rover, connection = _rover(robot)
    rover.start()
    with pytest.raises(ExecutionError, match="refused the path"):
        rover.follow_path(PATH, 1.0, timeout_s=45.0)
    assert robot.cancels == 1
    assert connection.control_requests()[-1]["brake"] == 1.0
    rover.close()


def test_the_ue5_fakes_keep_the_installed_signatures():
    """The fakes stand in for package methods; a package upgrade must show up here.

    They are hand-written, so this is the only automatic link between them and the
    installed ``projectairsim``. Renaming or reordering a parameter there fails
    here instead of being absorbed by a permissive fake.
    """
    import inspect

    from projectairsim.drone import Drone
    from projectairsim.rover import Rover

    for mine, installed in (
        (PathRobot.move_on_path_async, Drone.move_on_path_async),
        (PathRover.move_on_path_async, Rover.move_on_path_async),
    ):
        ours = list(inspect.signature(mine).parameters.values())[1:]
        theirs = list(inspect.signature(installed).parameters.values())[1:]
        assert [(p.name, p.kind, p.default) for p in ours] == [
            (p.name, p.kind, p.default) for p in theirs
        ], installed


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
