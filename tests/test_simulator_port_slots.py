"""Every declared slot must be filled by a real object, on both engines.

``PORT_SLOTS`` is what the console is told before a simulator runs. Until now
nothing compared it with the bundle the live session hands over, so UE5 advertised
``controlled_capture``, ``models`` and ``frames`` without an object behind them and
nobody noticed. ``build_ports`` now runs the comparison at construction; these cases
pin the pass and the failure for each package, and the name in the error.
"""

import pytest

from planner.simulator import SimulatorPorts
from planner.simulators.airsim import runtime as airsim_runtime
from planner.simulators.projectairsim import runtime as projectairsim_runtime


class _Object:
    """Stands in for a live port object; the check only looks at presence."""


def _projectairsim_slots():
    return {
        "vehicle": _Object(),
        "observation": _Object(),
        "streaming": _Object(),
        "moving_capture": _Object(),
        "weather": _Object(),
        "views": _Object(),
        "models": _Object(),
        "frames": _Object(),
        "home": _Object(),
    }


def test_airsim_bundle_fills_exactly_what_it_declares():
    ports = airsim_runtime.build_ports(
        vehicle=_Object(), observation=_Object(), weather=_Object()
    )
    assert set(ports.implemented()) == set(airsim_runtime.PORT_SLOTS)


def test_airsim_names_the_slot_it_cannot_fill():
    with pytest.raises(ValueError, match=r"missing \['weather'\]"):
        airsim_runtime.build_ports(
            vehicle=_Object(), observation=_Object(), weather=None
        )


def test_projectairsim_bundle_fills_exactly_what_it_declares():
    ports = projectairsim_runtime.build_ports(**_projectairsim_slots())
    assert set(ports.implemented()) == set(projectairsim_runtime.PORT_SLOTS)


def test_projectairsim_names_the_slot_it_cannot_fill():
    slots = _projectairsim_slots()
    slots["models"] = None
    with pytest.raises(ValueError, match=r"missing \['models'\]"):
        projectairsim_runtime.build_ports(**slots)


def test_a_filled_slot_that_was_never_declared_is_an_error_too():
    """The other direction: a capability the console cannot offer must not exist."""
    bundle = SimulatorPorts(
        vehicle=_Object(), observation=_Object(), home=_Object()
    )
    with pytest.raises(ValueError, match=r"undeclared \['home'\]"):
        bundle.verify_slots(("vehicle", "observation"), simulator="test")
