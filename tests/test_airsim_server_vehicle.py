"""The takeoff decision cannot rest on ``landed_state`` alone.

Measured on Blocks: after any landing, AirSim keeps reporting ``landed_state = 1``
(Flying) even with the drone back on its spawn height. A second task on that engine
then skipped takeoff, was commanded to fly along the ground, and failed 60 seconds
later as "synchronized motion timed out" -- a confusing failure whose real cause was
one stale enum value.
"""

import types

import pytest

from planner.simulators.airsim.server_vehicle import ServerVehicle


class _Connection:
    def __init__(self, z, landed_state):
        self.z, self.landed_state = z, landed_state
        self.calls: list[str] = []
        self.timestamp = 0

    def call(self, name, *args, **kwargs):
        self.calls.append(name)
        if name == "getMultirotorState":
            self.timestamp += 1
            quat = types.SimpleNamespace(x_val=0.0, y_val=0.0, z_val=0.0, w_val=1.0)
            zero = types.SimpleNamespace(x_val=0.0, y_val=0.0, z_val=0.0)
            return types.SimpleNamespace(
                landed_state=self.landed_state,
                timestamp=self.timestamp,
                kinematics_estimated=types.SimpleNamespace(
                    position=types.SimpleNamespace(x_val=0.0, y_val=0.0, z_val=self.z),
                    orientation=quat,
                    linear_velocity=zero,
                    angular_velocity=zero,
                ),
            )
        return True

    def call_async_and_wait(self, name, *args, **kwargs):
        self.calls.append(name)
        return True

    def call_streaming(self, name, *args, **kwargs):
        self.calls.append(name)
        return None

    def prepare_vehicle(self, *, arm=True):
        self.calls.append("prepare_vehicle")

    def takeoff(self, **kwargs):
        self.calls.append("takeoff")
        self.z = self.z - 1.5


@pytest.fixture(autouse=True)
def fake_airsim(monkeypatch):
    module = types.ModuleType("airsim")

    class LandedState:
        Landed = 0
        Flying = 1

    module.LandedState = LandedState
    module.DrivetrainType = types.SimpleNamespace(MaxDegreeOfFreedom=0)
    module.YawMode = lambda **kwargs: kwargs
    monkeypatch.setitem(sys_modules := __import__("sys").modules, "airsim", module)


class _FakeHold:
    """Stand in for the position hold: this test is about the takeoff decision."""

    active = True

    def __init__(self):
        self.started = 0
        self.waited = 0

    def start(self, target=None):
        self.started += 1

    def wait(self, **kwargs):
        self.waited += 1

    def tick(self):
        pass

    def release(self):
        pass

    def cancel(self):
        pass


def _vehicle(z, landed_state):
    from planner.domain.motion import MotionLimits

    connection = _Connection(z, landed_state)
    vehicle = ServerVehicle(connection, MotionLimits(), 5.0)
    vehicle._wait_ready = lambda: connection.call("getMultirotorState")
    vehicle.holding = _FakeHold()
    return connection, vehicle


def test_on_the_ground_by_height_it_takes_off_even_while_landed_state_lies():
    connection, vehicle = _vehicle(z=0.68, landed_state=1)      # Flying, but standing on it
    vehicle.prepare(takeoff_if_landed=True, ground_z=0.68)
    assert "takeoff" in connection.calls, connection.calls


def test_airborne_by_height_it_does_not_take_off_again():
    connection, vehicle = _vehicle(z=-0.85, landed_state=1)
    vehicle.prepare(takeoff_if_landed=True, ground_z=0.68)
    assert "takeoff" not in connection.calls, connection.calls


def test_without_a_ground_reference_the_old_behaviour_stands():
    """No record means no claim: the enum alone decides, as before this fix."""
    connection, vehicle = _vehicle(z=0.68, landed_state=1)
    vehicle.prepare(takeoff_if_landed=True)
    assert "takeoff" not in connection.calls, connection.calls
