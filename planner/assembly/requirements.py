"""What each task needs from a simulator bundle, and the flags that follow.

The declarations live in the task packages: ``TASK_KIND`` and ``REQUIRED_PORTS``
on ``planner.workflows.<task>``. This module puts them in one table and answers
the question the console asks: which task kinds, and which optional controls,
the bundle a simulator hands over can actually run.

The table is not kept beside the ports because a second copy drifts the same way
a ``capabilities`` block in YAML would: the code gains a slot, the table does
not, and the console offers a control nothing implements.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import fields
from importlib import import_module

from ..domain import Capabilities, TaskKind
from ..simulator import SimulatorPorts

#: Task packages declaring ``TASK_KIND`` and ``REQUIRED_PORTS``. Order decides
#: the order of ``capabilities.task_kinds``.
TASK_PACKAGES: tuple[str, ...] = (
    "planner.workflows.basic_motion",
    "planner.workflows.visual_goal_navigation",
    "planner.workflows.free_navigation",
)

#: Port slot -> the ``actions`` flag it unlocks. Only engine-dependent controls
#: belong here; trajectory telemetry, logs and the task list are service features.
ACTION_PORTS: dict[str, str] = {
    "home": "home",
    "models": "models",
    "video": "frames",
    "weather": "weather",
}

#: Names a task may require: every port slot the bundle declares. ``vehicles``
#: carries data, not a port.
SLOT_NAMES: frozenset[str] = frozenset(
    item.name for item in fields(SimulatorPorts)
) - {"vehicles"}


def build_table(
    packages: Iterable[str] = TASK_PACKAGES,
) -> dict[str, tuple[str, ...]]:
    """Read the declarations from each task package, rejecting typos."""
    table: dict[str, tuple[str, ...]] = {}
    for package in packages:
        module = import_module(package)
        kind = getattr(module, "TASK_KIND", None)
        required = getattr(module, "REQUIRED_PORTS", None)
        if not isinstance(kind, TaskKind):
            raise ValueError(
                f"{package} must declare TASK_KIND as a TaskKind; got {kind!r}"
            )
        if not isinstance(required, tuple) or not all(
            isinstance(slot, str) for slot in required
        ):
            raise ValueError(
                f"{package} must declare REQUIRED_PORTS as a tuple of slot names; "
                f"got {required!r}"
            )
        unknown = [slot for slot in required if slot not in SLOT_NAMES]
        if unknown:
            raise ValueError(
                f"{package} requires port slots {unknown}, which SimulatorPorts "
                f"does not declare; known slots are {sorted(SLOT_NAMES)}"
            )
        table[kind.value] = required
    return table


TASK_REQUIREMENTS: dict[str, tuple[str, ...]] = build_table()


def available_kinds(slots: Iterable[str]) -> tuple[str, ...]:
    """Task kinds whose required slots are all present."""
    present = set(slots)
    return tuple(
        kind for kind in TASK_REQUIREMENTS if not missing_slots(kind, present)
    )


def missing_slots(kind: str, slots: Iterable[str]) -> tuple[str, ...]:
    required = TASK_REQUIREMENTS.get(kind)
    if required is None:
        raise ValueError(
            f"unknown task kind {kind!r}; declared kinds are {list(TASK_REQUIREMENTS)}"
        )
    present = set(slots)
    return tuple(slot for slot in required if slot not in present)


def capabilities_for(ports: SimulatorPorts) -> Capabilities:
    """The console's feature flags for one bundle."""
    slots = ports.implemented()
    view_modes: tuple[str, ...] = ()
    if ports.views is not None:
        view_modes = tuple(ports.views.view_modes())
    weather_fields: dict[str, str] = {}
    weather_presets: tuple[str, ...] = ()
    if ports.weather is not None:
        weather_fields = dict(ports.weather.fields())
        weather_presets = tuple(ports.weather.presets())
    present = set(slots)
    return Capabilities(
        vehicles=tuple(ports.vehicles),
        view_modes=view_modes,
        actions=tuple(
            action for action, slot in ACTION_PORTS.items() if slot in present
        ),
        task_kinds=available_kinds(slots),
        weather_fields=weather_fields,
        weather_presets=weather_presets,
    )
