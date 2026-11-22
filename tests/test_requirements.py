"""The requirement table is the task packages' declaration, not the engine's.

``planner/simulator/ports.py`` describes slots only; the task-to-slot table is
read from ``planner.workflows.<task>`` and lives in ``planner.assembly``.
"""

import sys
import types

import pytest

from planner.assembly import requirements
from planner.domain import TaskKind
from planner.workflows import basic_motion, free_navigation, visual_goal_navigation

PACKAGES = (basic_motion, visual_goal_navigation, free_navigation)

EVERY_SLOT = ("vehicle", "observation", "streaming")


def test_every_task_package_declares_a_kind_and_its_slots():
    for package in PACKAGES:
        assert isinstance(package.TASK_KIND, TaskKind), package.__name__
        assert isinstance(package.REQUIRED_PORTS, tuple), package.__name__
        assert package.REQUIRED_PORTS, package.__name__
        unknown = [
            slot for slot in package.REQUIRED_PORTS
            if slot not in requirements.SLOT_NAMES
        ]
        assert not unknown, f"{package.__name__} requires {unknown}"


def test_the_table_is_what_the_packages_declare():
    declared = {
        package.TASK_KIND.value: package.REQUIRED_PORTS for package in PACKAGES
    }
    assert requirements.TASK_REQUIREMENTS == declared
    # The console renders task_kinds in table order, which is package order.
    assert list(requirements.TASK_REQUIREMENTS) == [
        package.TASK_KIND.value for package in PACKAGES
    ]
    assert all(
        isinstance(slot, str)
        for required in requirements.TASK_REQUIREMENTS.values()
        for slot in required
    )


def test_a_kind_is_offered_when_its_slots_are_part_of_the_bundle():
    assert requirements.available_kinds(("vehicle",)) == ("basic_motion",)
    assert requirements.available_kinds(("vehicle", "observation")) == (
        "basic_motion",
        "visual_goal_navigation",
    )
    assert requirements.available_kinds(EVERY_SLOT) == (
        "basic_motion",
        "visual_goal_navigation",
        "free_navigation",
    )
    # Slots no task asks for change nothing, and an empty bundle runs no task.
    assert requirements.available_kinds(("vehicle", "weather")) == ("basic_motion",)
    assert requirements.available_kinds(()) == ()


def test_enabled_tasks_narrows_the_offer_without_adding_one():
    assert requirements.available_kinds(EVERY_SLOT, ("basic_motion",)) == (
        "basic_motion",
    )
    # A simulator that lists nothing is unrestricted, not disabled.
    assert requirements.available_kinds(EVERY_SLOT, ()) == requirements.available_kinds(
        EVERY_SLOT
    )
    # Naming a task the slots cannot run does not make it available.
    assert "free_navigation" not in requirements.available_kinds(
        ("vehicle",), ("free_navigation",)
    )


def test_missing_slots_lists_what_the_bundle_lacks():
    assert requirements.missing_slots("free_navigation", ("vehicle",)) == (
        "observation",
        "streaming",
    )
    assert requirements.missing_slots("basic_motion", EVERY_SLOT) == ()
    with pytest.raises(ValueError, match="unknown task kind"):
        requirements.missing_slots("teleport", EVERY_SLOT)


def test_a_required_slot_the_bundle_has_no_field_for_is_rejected(monkeypatch):
    stub = types.ModuleType("fake_task_package")
    stub.TASK_KIND = TaskKind.BASIC_MOTION
    stub.REQUIRED_PORTS = ("vehicle", "teleporter")
    monkeypatch.setitem(sys.modules, "fake_task_package", stub)

    with pytest.raises(ValueError, match="teleporter"):
        requirements.build_table(("fake_task_package",))
