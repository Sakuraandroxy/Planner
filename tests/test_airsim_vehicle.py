"""Basic motion is decided from the vehicle's state, not from an ack that lies.

Measured on the target build: ``moveToPositionAsync().get()`` and
``takeoffAsync().get()`` both answer ``False`` while the aircraft does exactly what
was asked. The adapter used to read that bool as the verdict, so every basic motion
ended in ``ExecutionError: AirSim basic motion did not complete`` before the vehicle
had finished climbing.

These tests drive the endpoint check with a scripted ``current_pose`` and a stand-in
hold service, so nothing here needs AirSim installed.
"""

import sys
import types

import pytest

from planner.domain.pose import WorldPose
from planner.errors import ExecutionError
from planner.simulators.airsim.vehicle import AirSimVehicle


class _Connection:
    def __init__(self, ack=False):
        self.ack = ack
        self.issued: list[tuple] = []

    def call_async_and_wait(self, name, *args, **kwargs):
        self.issued.append((name, args, kwargs))
        return self.ack

    def call(self, name, *args, **kwargs):
        return object()


class _Holding:
    """The position-hold service, as far as basic motion is concerned."""

    active = True

    def __init__(self):
        self.started: list = []
        self.waited = 0
        self.released = 0

    def approach_target(self, start, target):
        return target

    def start(self, target=None):
        self.started.append(target)

    def wait(self, budget=None):
        self.waited += 1

    def tick(self):
        pass

    def release(self):
        self.released += 1


def _vehicle(poses, *, ack=False, holding=None):
    vehicle = AirSimVehicle(_Connection(ack), speed_mps=2.0, timeout_s=0.2)
    sequence = list(poses)
    vehicle.current_pose = (
        lambda: sequence.pop(0) if len(sequence) > 1 else sequence[0]
    )
    vehicle.holding = holding
    return vehicle


@pytest.fixture(autouse=True)
def fake_airsim(monkeypatch):
    module = types.ModuleType("airsim")
    module.DrivetrainType = types.SimpleNamespace(MaxDegreeOfFreedom=0)
    module.YawMode = lambda **kwargs: kwargs
    monkeypatch.setitem(sys.modules, "airsim", module)


START = WorldPose(0.0, 0.0, 0.0, 0.0)
TARGET = WorldPose(0.0, 0.0, -2.0, 0.0)


def test_arrival_check_passes_inside_the_tolerance():
    vehicle = _vehicle([TARGET])
    vehicle._await_arrival(TARGET)          # returns without raising


def test_arrival_check_reports_what_it_measured():
    vehicle = _vehicle([WorldPose(0.0, 0.0, 0.0, 0.0)])
    with pytest.raises(ExecutionError, match=r"did not reach its endpoint: 2\.00 m"):
        vehicle._await_arrival(TARGET)


def test_yaw_only_commands_are_checked_by_heading_too():
    vehicle = _vehicle([WorldPose(TARGET.x, TARGET.y, TARGET.z, 5.0)])
    with pytest.raises(ExecutionError, match="yaw error"):
        vehicle._await_arrival(WorldPose(0.0, 0.0, 0.0, 90.0), yaw_delta_deg=90.0)


def test_a_false_ack_does_not_override_the_state():
    """The failure the machine produced: the command ran, the verdict said no."""
    holding = _Holding()
    vehicle = _vehicle([TARGET], ack=False, holding=holding)
    vehicle.execute_basic_motion(START, TARGET, 0.0)
    assert holding.started == [TARGET]
    assert holding.waited == 1


def test_the_endpoint_still_has_to_show_up_in_state():
    vehicle = _vehicle([WorldPose(0.0, 0.0, 0.5, 0.0)], ack=False, holding=_Holding())
    with pytest.raises(ExecutionError, match="did not reach its endpoint"):
        vehicle.execute_basic_motion(START, TARGET, 0.0)


def test_a_short_translation_hands_the_endpoint_to_the_hold_service():
    """``approach_target`` answering None means "no native call needed"."""
    holding = _Holding()
    holding.approach_target = lambda start, target: None
    vehicle = _vehicle([TARGET], ack=False, holding=holding)
    vehicle.execute_basic_motion(START, TARGET, 0.0)
    assert vehicle.connection.issued == [], "no motion RPC for a skipped segment"
    assert holding.started == [TARGET]


def test_basic_motion_without_a_hold_service_says_so():
    """Not a supported path: the flight ends in ``hover()``, which needs the service.

    Writing this down because the shape of the error matters -- an unassembled
    session must not look like a vehicle that refused to move.
    """
    vehicle = _vehicle([TARGET], holding=None)
    with pytest.raises(ExecutionError, match="position hold service was not assembled"):
        vehicle.execute_basic_motion(START, TARGET, 0.0)
