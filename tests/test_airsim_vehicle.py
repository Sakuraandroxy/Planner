"""The vehicle translates the hold service's name for the wait it asks for.

The gate asks for ``wait_for_observation``; the hold service calls it ``wait``.
Measured on Blocks: with capture working, the visual-goal stage then died on
"'PositionHold' object has no attribute 'wait_for_observation'". The vehicle is the
only place that knows both names, so the translation lives there -- and these tests
keep the two from drifting apart again.

Nothing here needs AirSim installed: the connection is a stand-in that fails loudly.
"""

import pytest

from planner.errors import ExecutionError
from planner.simulators.airsim.vehicle import AirSimVehicle


class _Connection:
    def call(self, name, *args, **kwargs):
        raise AssertionError(f"these tests never call the engine ({name})")


class _Holding:
    """The position-hold service, as far as waiting is concerned."""

    active = True

    def __init__(self):
        self.waited = 0

    def wait(self, *, budget=None, checkpoint=None):
        self.waited += 1

    def tick(self):
        pass


def _vehicle(holding):
    vehicle = AirSimVehicle(_Connection(), speed_mps=2.0, timeout_s=0.2)
    vehicle.holding = holding
    return vehicle


def test_the_vehicle_waits_on_the_hold_service_by_its_real_name():
    holding = _Holding()
    calls = []
    holding.wait = lambda budget=None, checkpoint=None: calls.append((budget, checkpoint))
    vehicle = _vehicle(holding)
    checkpoint = lambda: None
    vehicle.wait_for_observation(budget="B", checkpoint=checkpoint)
    assert calls == [("B", checkpoint)], calls


def test_a_vehicle_without_the_hold_service_refuses_to_wait():
    vehicle = _vehicle(None)
    with pytest.raises(ExecutionError, match="position hold service was not assembled"):
        vehicle.wait_for_observation()
