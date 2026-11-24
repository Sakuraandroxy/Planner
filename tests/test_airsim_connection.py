"""Every AirSim call must reach the vehicle the settings file actually names.

``config/airsim_settings.json`` registers ``Drone_1``. With a blank ``vehicle_name``
the server still answers state reads, so a smoke test looks healthy right up to the
first control call: ``moveToPositionAsync`` returns ``False``, ``landAsync`` raises an
RPC exception, and the vehicle does not move. The adapter used to send the blank name
on all 41 of its call sites.

The fake client below keeps the real signatures, because the injection decision
follows them: methods without a ``vehicle_name`` parameter must not be given one.
"""

import inspect
import sys
import threading
import time
import types

import pytest


class _Future:
    def __init__(self, value):
        self._value = value

    def get(self):
        return self._value

    def attach_callback(self, callback):
        pass


class _Loop:
    """The part of the SDK's msgpack-rpc loop a waiting future touches.

    ``configure_rpc_loop`` wraps this in ``OwnerRpcLoop``, so ``start`` is what a
    pending ``Future.get()`` calls in a loop and ``stop``/``abandon`` are what the
    connection uses to end that wait.
    """

    def __init__(self):
        self._ioloop = types.SimpleNamespace(
            dettach_periodic_callback=lambda: None, close=lambda: None
        )
        self.stops = 0

    def start(self):
        # The real loop pumps I/O here and returns on its periodic timeout tick.
        threading.Event().wait(0.001)

    def stop(self):
        self.stops += 1


class _StuckFuture:
    """A native call that never answers until the test releases it.

    ``get`` mimics msgpack-rpc: it pumps the client's loop until the answer lands,
    which is exactly the loop ``OwnerRpcLoop.abandon`` has to break.
    """

    def __init__(self, client):
        self._client = client
        self.released = threading.Event()

    def get(self):
        loop = self._client.client._loop
        while not self.released.is_set():
            loop.start()
        return False

    def attach_callback(self, callback):
        pass


class _Pose:
    class position:
        x_val = y_val = 0.0
        z_val = 0.67

    class orientation:
        x_val = y_val = z_val = 0.0
        w_val = 1.0


class _State:
    """What getMultirotorState answers: just enough for ``_ground_z()``."""

    class _Kinematics:
        def __init__(self, z):
            class P:
                pass
            p = P()
            p.x_val = p.y_val = 0.0
            p.z_val = z
            self.position = p

    def __init__(self, z):
        self.kinematics_estimated = _State._Kinematics(z)


class _Client:
    """Signature-shaped stand-in for ``airsim.MultirotorClient``."""

    def __init__(self, **kwargs):
        self.seen: dict[str, object] = {}
        # What getMultirotorState answers, in order. A list that runs out repeats its
        # last value, which is how a vehicle that has settled behaves.
        self.z_values: list[float] = [0.67]
        self.takeoff_ack = True
        # Path calls answer at once unless a test says the server went silent.
        self.path_never_answers = False
        self.path_future: _StuckFuture | None = None
        # configure_rpc_loop() wraps this; a pending future pumps it.
        self.client = types.SimpleNamespace(_loop=_Loop(), close=lambda: None)

    def _z(self):
        return self.z_values.pop(0) if len(self.z_values) > 1 else self.z_values[0]

    def simGetVehiclePose(self, vehicle_name=""):
        self.seen["simGetVehiclePose"] = vehicle_name
        return _Pose()

    def enableApiControl(self, is_enable, vehicle_name=""):
        self.seen["enableApiControl"] = (is_enable, vehicle_name)
        return True

    def moveToPositionAsync(self, x, y, z, v, timeout_sec=None, drivetrain=None,
                            yaw_mode=None, vehicle_name=""):
        self.seen["moveToPositionAsync"] = vehicle_name
        return _Future(True)

    def listVehicles(self):
        self.seen["listVehicles"] = "no-parameter"
        return ["Drone_1"]

    def moveOnPathAsync(self, path, velocity, timeout_sec=3e38, drivetrain=None,
                        yaw_mode=None, lookahead=-1, adaptive_lookahead=1,
                        vehicle_name=""):
        self.seen["moveOnPathAsync"] = (velocity, timeout_sec, drivetrain,
                                        yaw_mode, lookahead, adaptive_lookahead,
                                        vehicle_name)
        self.path_future = _StuckFuture(self) if self.path_never_answers else _Future(True)
        return self.path_future

    def ping(self):
        return True

    def getMultirotorState(self, timestamp=-1.0, vehicle_name=""):
        self.seen["getMultirotorState"] = vehicle_name
        return _State(self._z())

    def takeoffAsync(self, max_seconds_z_err=None, vehicle_name=""):
        self.seen["takeoffAsync"] = vehicle_name
        return _Future(self.takeoff_ack)

    def hoverAsync(self, vehicle_name=""):
        return _Future(True)


@pytest.fixture(autouse=True)
def fake_airsim(monkeypatch):
    module = types.ModuleType("airsim")
    module.MultirotorClient = _Client
    # The vehicle adapter builds these three; the connection tests that go
    # through the real vehicle need them too.
    module.DrivetrainType = types.SimpleNamespace(ForwardOnly=1, MaxDegreeOfFreedom=0)
    module.YawMode = lambda **kwargs: kwargs
    module.Vector3r = lambda x, y, z: (x, y, z)
    monkeypatch.setitem(sys.modules, "airsim", module)


def _connection(**kwargs):
    from planner.simulators.airsim.connection import AirSimConnection

    connection = AirSimConnection(host="", port=41451, timeout_s=5, **kwargs)
    return connection, connection.client


def test_named_vehicle_reaches_state_and_control_calls():
    connection, client = _connection(vehicle_name="Drone_1")
    connection.call("simGetVehiclePose")
    connection.call("enableApiControl", True)
    assert client.seen["simGetVehiclePose"] == "Drone_1"
    assert client.seen["enableApiControl"] == (True, "Drone_1")


def test_the_async_channel_carries_it_too():
    """This is the call that silently returned ``False`` on the real machine."""
    connection, client = _connection(vehicle_name="Drone_1")
    assert connection.call_async_and_wait("moveToPositionAsync", 0.0, 0.0, -2.0, 2.0) is True
    assert client.seen["moveToPositionAsync"] == "Drone_1"


def test_a_path_call_that_never_answers_is_cut_by_the_client_deadline():
    """``Future.get()`` has no timeout: the limit has to be the connection's."""
    from planner.errors import ExecutionError

    connection, client = _connection(vehicle_name="Drone_1")
    client.path_never_answers = True
    started = time.monotonic()
    with pytest.raises(ExecutionError, match=r"did not answer inside 0\.05s"):
        connection.call_async_and_wait("moveOnPathAsync", [], 1.0, timeout_s=0.05)
    assert time.monotonic() - started < 2.0, "the wait was not bounded by timeout_s"
    # The wait was unwound, not abandoned: no thread is left pumping the loop.
    assert [t for t in threading.enumerate() if t.name.startswith("airsim-wait-")] == []
    # And the connection is still usable afterwards.
    connection.call("simGetVehiclePose")
    assert client.seen["simGetVehiclePose"] == "Drone_1"
    client.path_future.released.set()


def test_a_wait_that_times_out_leaves_no_thread_behind():
    """A second pump on the same loop would spin and race the transport close."""
    from planner.errors import ExecutionError

    connection, client = _connection()
    client.path_never_answers = True
    for _ in range(3):
        with pytest.raises(ExecutionError):
            connection.call_async_and_wait("moveOnPathAsync", [], 1.0, timeout_s=0.01)
    assert [t for t in threading.enumerate() if t.name.startswith("airsim-wait-")] == []
    client.path_future.released.set()


def test_a_timeout_that_is_not_a_positive_number_is_rejected():
    connection, client = _connection()
    for bad in (0, -1.0, float("inf"), float("nan")):
        with pytest.raises(ValueError, match="finite positive"):
            connection.call_async_and_wait("ping", timeout_s=bad)
    assert "moveOnPathAsync" not in client.seen


def test_the_vehicle_path_call_fits_the_sdk_keywords():
    """The vehicle's keywords are checked against a signature-shaped client.

    ``_Client.moveOnPathAsync`` declares AirSim's parameters explicitly, so a
    misspelt keyword raises ``TypeError`` instead of being swallowed the way it
    would be by a ``**kwargs`` fake.
    """
    from planner.domain.pose import WorldPose
    from planner.simulators.airsim.vehicle import AirSimVehicle

    class _Holding:
        def __init__(self):
            self.started = []
            self.ticks = 0

        def release(self):
            pass

        def start(self, target=None):
            self.started.append(target)

        def tick(self):
            self.ticks += 1

    connection, client = _connection(vehicle_name="Drone_1")
    vehicle = AirSimVehicle(connection, speed_mps=2.0, timeout_s=5.0)
    vehicle.holding = _Holding()
    vehicle.follow_path(
        [WorldPose(1.0, 2.0, 3.0, 0.0), WorldPose(4.0, 5.0, 6.0, 0.0)],
        1.0,
        timeout_s=30.0,
    )
    velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive, name = (
        client.seen["moveOnPathAsync"]
    )
    assert (velocity, timeout_sec, lookahead, adaptive) == (1.0, 30.0, 3, 1)
    assert drivetrain == 1 and yaw_mode == {"is_rate": False}
    assert name == "Drone_1"


def test_the_client_deadline_and_air_sims_timeout_are_separate_keywords():
    """``timeout_s`` bounds the wait here; ``timeout_sec`` is sent to the server."""
    connection, client = _connection(vehicle_name="Drone_1")
    assert connection.call_async_and_wait(
        "moveOnPathAsync", [], 1.0, timeout_s=5.0, timeout_sec=30.0
    ) is True
    velocity, timeout_sec, _drivetrain, _yaw_mode, lookahead, adaptive, vehicle_name = (
        client.seen["moveOnPathAsync"]
    )
    assert (velocity, timeout_sec, lookahead, adaptive) == (1.0, 30.0, -1, 1)
    assert vehicle_name == "Drone_1", "the deadline must not drop the vehicle name"


def test_methods_without_the_parameter_are_left_alone():
    connection, client = _connection(vehicle_name="Drone_1")
    assert connection.call("listVehicles") == ["Drone_1"]
    assert client.seen["listVehicles"] == "no-parameter"


def test_a_caller_that_named_a_vehicle_wins():
    connection, client = _connection(vehicle_name="Drone_1")
    connection.call("simGetVehiclePose", vehicle_name="Drone_2")
    assert client.seen["simGetVehiclePose"] == "Drone_2"


def test_an_empty_name_injects_nothing():
    """Builds whose vehicle is the client default must not get a surprise key."""
    connection, client = _connection(vehicle_name="")
    connection.call("simGetVehiclePose")
    assert client.seen["simGetVehiclePose"] == ""


def test_takeoff_succeeds_on_state_even_when_the_ack_says_false():
    """Measured on Blocks: ``takeoffAsync().get()`` is False and the aircraft rises.

    0.68 to -0.86, i.e. 1.54 m of climb in UE coordinates where up is negative.
    Reading that ack as a verdict is what made every UE4 task fail before it flew.
    """
    connection, client = _connection(vehicle_name="Drone_1")
    client.z_values = [0.68, -0.86]
    client.takeoff_ack = False
    connection.takeoff(hover_after=False)
    assert client.seen["takeoffAsync"] == "Drone_1"


def test_takeoff_fails_when_the_vehicle_never_lifts():
    connection, client = _connection(vehicle_name="Drone_1")
    client.z_values = [0.68, 0.68]
    with pytest.raises(Exception, match="did not lift the vehicle"):
        connection.takeoff(hover_after=False, settle_timeout_s=0.0)


def test_the_signature_probe_survives_a_wrapped_method():
    """The cache is keyed by __qualname__; a missing signature means "do not inject"."""
    from planner.simulators.airsim.connection import _accepts_vehicle_name

    def with_name(vehicle_name=""):
        pass

    def without_name():
        pass

    def catch_all(**kwargs):
        pass

    assert _accepts_vehicle_name(without_name) is False
    assert _accepts_vehicle_name(with_name) is True
    assert _accepts_vehicle_name(catch_all) is True
    assert _accepts_vehicle_name(len) is False       # builtins have no signature
    assert inspect.isfunction(with_name)
