"""Path following: one path in, the engine's own commands out, arrival measured.

The UE4 adapter hands the whole path to ``moveOnPath`` in one call and these tests
pin its parameters item by item, including that no per-point ``moveToPosition``
call happens instead. The UE5 adapter drives the same path point by point through
``MoveToPosition``, because that scene does not register ``MoveOnPath`` at all, and
these tests pin that too: one command per point, arrival read from the pose, and
cancel-and-hold whenever a path does not demonstrably finish. The parameter values
are the 3DG-VLN baseline's (``airsim_plugin/AirVLNSimulatorClientTool.py:365-388``).
"""

import asyncio
import math
import sys
import types

import pytest
from projectairsim.drone import YawControlMode

from planner.domain.pose import WorldPose
from planner.errors import ExecutionError
from planner.simulators.airsim.vehicle import AirSimVehicle
from planner.simulators.projectairsim import drone as path_follow
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
        if name == "simGetVehiclePose":
            # A 45 degree heading in quaternion form, which is what the path
            # command has to hold.
            half = math.radians(45.0) / 2.0
            return types.SimpleNamespace(
                position=types.SimpleNamespace(x_val=0.0, y_val=0.0, z_val=0.0),
                orientation=types.SimpleNamespace(
                    w_val=math.cos(half), x_val=0.0, y_val=0.0, z_val=math.sin(half),
                ),
            )
        return True

    @property
    def names(self):
        return [name for name, _, _ in self.issued if name != "simGetVehiclePose"]


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
    _, args, kwargs = [entry for entry in connection.issued if entry[0] == "moveOnPathAsync"][0]
    assert args[0] == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    assert args[1] == 1.0
    # Every value the baseline passes, plus the caller's deadline twice: once as
    # the engine-side timeout and once as the client-side limit that keeps a
    # silent server from blocking the control thread.
    # The nose holds the heading it started the path with. ``ForwardOnly`` would
    # turn the frame the model answers in and compound a left offset into a spiral.
    assert kwargs == {
        "timeout_s": 30.0,
        "timeout_sec": 30.0,
        "drivetrain": 0,
        "yaw_mode": {"is_rate": False, "yaw_or_rate": pytest.approx(45.0)},
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
    path_call = [entry for entry in connection.issued if entry[0] == "moveOnPathAsync"][0]
    assert path_call[2]["timeout_s"] == 30.0, (
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
        """The transport dies on the stopping call, not on the heading read."""

        def call(self, name, *args, **kwargs):
            if name == "simGetVehiclePose":
                return super().call(name, *args, **kwargs)
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
        raise AssertionError("the rover must not use point commands")


class PathRobot(_PathState, DroneRobot):
    """``Drone.move_to_position_async`` with the installed package's signature."""

    def __init__(self):
        super().__init__()
        self.point_calls: list[dict] = []
        # The engine answers when it takes the command, so the fake has to say
        # whether the aircraft got there: it moves to the point unless a test wants
        # a command that is accepted and never flown.
        self.point_arrives = True
        self.ack = True
        self.never_answers = False

    async def move_to_position_async(self, north, east, down, velocity,
                                     timeout_sec=3e38,
                                     yaw_control_mode=YawControlMode.MaxDegreeOfFreedom,
                                     yaw_is_rate=True, yaw=0.0, lookahead=-1.0,
                                     adaptive_lookahead=1.0, callback=None):
        self.point_calls.append(
            {"north": north, "east": east, "down": down, "velocity": velocity,
             "timeout_sec": timeout_sec, "yaw_control_mode": yaw_control_mode,
             "yaw_is_rate": yaw_is_rate, "yaw": yaw,
             "lookahead": lookahead, "adaptive_lookahead": adaptive_lookahead,
             "callback": callback}
        )
        future = asyncio.get_running_loop().create_future()
        if not self.never_answers:
            future.set_result(self.ack)
            if self.point_arrives:
                self.x, self.y, self.z = float(north), float(east), float(down)
        return future

    async def move_on_path_async(self, *args, **kwargs):
        raise AssertionError("the drone must not use MoveOnPath: this scene "
                             "rejects it and answers True anyway")

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


def test_projectairsim_drone_drives_the_path_point_by_point():
    """This scene rejects ``MoveOnPath``, so each point is its own command."""
    robot = PathRobot()
    drone = _drone(robot)
    drone.start()
    drone.follow_path(PATH, 1.0, timeout_s=45.0)
    assert robot.path_calls == [], "a path command here is accepted and never flown"
    assert [(c["north"], c["east"], c["down"]) for c in robot.point_calls] == [
        (1.0, 2.0, 3.0), (4.0, 5.0, 6.0)
    ]
    first = robot.point_calls[0]
    assert first["velocity"] == 1.0
    # The caller's remaining deadline is what the engine may spend on this point.
    assert first["timeout_sec"] == pytest.approx(45.0, abs=0.5)
    # ``ForwardOnly`` is rejected on this build (ProjectAirSim issue 81), so the
    # command holds its heading through the maximum-degree-of-freedom mode.
    assert first["yaw_control_mode"] == YawControlMode.MaxDegreeOfFreedom
    assert first["yaw_is_rate"] is True
    assert first["yaw"] == 0.0
    assert first["callback"] is None
    assert len(robot.commands) == 1, "only the trailing hold command moves the aircraft"
    assert drone._holding.active is True, "the path ends in a controlled hover"
    drone.close()


def test_projectairsim_drone_reports_a_point_it_never_reaches():
    """An accepted command is not a flown one: the ack arrives at once.

    Measured on the 4090 machine: a 1.5 m path answered ``True`` after 0.00 s with
    the aircraft still at its start, so the arrival has to be read from the pose.
    """
    robot = PathRobot()
    robot.point_arrives = False
    drone = _drone(robot)
    drone.start()
    with pytest.raises(ExecutionError, match="did not arrive"):
        drone.follow_path(PATH, 1.0, timeout_s=0.05)
    assert robot.cancels >= 1, "the native task has to be cancelled"
    assert drone._holding.active is True, "the hold still has to take the aircraft"
    drone.close()


def test_projectairsim_drone_skips_a_point_it_cannot_close(monkeypatch):
    """A point the engine will not close is left behind, not turned into a failure.

    The engine stops a position command about 0.5 m short of its target, so a
    point inside that band can never satisfy a distance check. The path carries
    on to the next point and the round replans from the measured pose.
    """
    # The per-point cap is 20 s of wall clock in production; here it only has to
    # be short enough that the point is given up.
    monkeypatch.setattr(path_follow, "PATH_POINT_WAIT_S", 0.05)
    robot = PathRobot()
    robot.point_arrives = False
    drone = _drone(robot)
    drone.start()
    drone.follow_path(PATH, 1.0, timeout_s=45.0)
    assert len(robot.point_calls) == 2, "the second point is still issued"
    assert drone._holding.active is True
    drone.close()


def test_projectairsim_drone_keeps_a_point_the_engine_answered_false():
    """This build answers ``False`` for commands it carries out, so arrival decides."""
    robot = PathRobot()
    robot.ack = False
    drone = _drone(robot)
    drone.start()
    drone.follow_path(PATH, 1.0, timeout_s=45.0)
    assert len(robot.point_calls) == 2
    drone.close()


def test_projectairsim_drone_holds_the_flown_path_end():
    """The hold after a flown path takes the last point, not an older reference."""
    robot = PathRobot()
    drone = _drone(robot)
    drone.start()
    drone.follow_path(PATH, 1.0, timeout_s=45.0)
    target = drone._holding.target
    assert (target.x, target.y, target.z) == (4.0, 5.0, 6.0)
    drone.close()


def test_projectairsim_drone_reports_a_point_that_never_answers():
    robot = PathRobot()
    robot.never_answers = True
    drone = _drone(robot)
    drone.start()
    with pytest.raises(ExecutionError, match=r"did not complete in 0\.05s"):
        drone.follow_path(PATH, 1.0, timeout_s=0.05)
    assert drone._holding.active is True, "the deadline still ends in a controlled hover"
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


def test_projectairsim_drone_cancels_a_point_it_never_reached():
    """A false ack is not the error by itself; a point never reached is."""
    robot = PathRobot()
    robot.ack = False
    robot.point_arrives = False
    drone = _drone(robot)
    drone.start()
    with pytest.raises(ExecutionError, match="did not arrive"):
        drone.follow_path(PATH, 1.0, timeout_s=0.05)
    assert robot.cancels >= 1, "the native task has to be cancelled"
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
        (PathRobot.move_to_position_async, Drone.move_to_position_async),
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
    assert robot.point_calls == []
    drone.close()
