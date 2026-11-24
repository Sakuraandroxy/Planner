"""The engine path-following port, and the slot the bare pipeline needs.

The bare pipeline is the only one that hands a whole path to the engine, so it is
the only one that cannot start without ``path_follow``. That is a table row read
through the same ``missing`` check as every other requirement, not a branch in the
assembly.
"""

from __future__ import annotations

import inspect
from dataclasses import fields

from planner.ports.path_vehicle import PathVehicle
from planner.simulator.ports import (
    PIPELINE_REQUIREMENTS,
    TASK_REQUIREMENTS,
    SimulatorPorts,
)


def test_follow_path_takes_the_whole_path_and_a_timeout():
    signature = inspect.signature(PathVehicle.follow_path)
    assert list(signature.parameters) == ["self", "points", "speed_mps", "timeout_s"]
    assert (
        signature.parameters["timeout_s"].kind
        is inspect.Parameter.KEYWORD_ONLY
    )


def test_the_bundle_carries_the_path_slot_as_optional():
    names = {item.name for item in fields(SimulatorPorts)}
    assert "path_follow" in names
    assert SimulatorPorts.from_slots(("vehicle", "observation")).path_follow is None
    assert (
        SimulatorPorts.from_slots(("vehicle", "observation", "path_follow")).path_follow
        is not None
    )


def test_the_bare_pipeline_is_the_only_one_that_requires_path_follow():
    assert set(PIPELINE_REQUIREMENTS) == {"staged", "bare"}
    assert "path_follow" in PIPELINE_REQUIREMENTS["bare"]
    assert "path_follow" not in PIPELINE_REQUIREMENTS["staged"]


def test_a_missing_path_slot_is_named_by_missing():
    declared = SimulatorPorts.from_slots(("vehicle", "observation"))
    complete = SimulatorPorts.from_slots(PIPELINE_REQUIREMENTS["bare"])
    assert declared.missing(PIPELINE_REQUIREMENTS["bare"]) == ("path_follow",)
    assert complete.missing(PIPELINE_REQUIREMENTS["bare"]) == ()


def test_every_requirement_names_a_slot_the_bundle_has():
    known = {item.name for item in fields(SimulatorPorts)}
    for table in (TASK_REQUIREMENTS, PIPELINE_REQUIREMENTS):
        for kind, required in table.items():
            unknown = set(required) - known
            assert not unknown, f"{kind} requires slots the bundle has no field for: {unknown}"
