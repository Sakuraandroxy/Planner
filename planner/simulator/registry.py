"""Resolve a simulator id to the capabilities it advertises.

The registry asks each simulator package for its declared port slots and derives
the flags. There is deliberately no capability data in ``config``: a simulator
package that gains or loses a port changes what the console offers, and a config
file cannot disagree with the code about it.
"""

from __future__ import annotations

from importlib import import_module

from .ports import SimulatorPorts

#: Simulator id -> module holding ``describe()`` and ``PORT_SLOTS``. Adding a
#: simulator means one row here plus the package.
RUNTIME_MODULES: dict[str, str] = {
    "airsim": "planner.simulators.airsim.runtime",
    "projectairsim": "planner.simulators.projectairsim.runtime",
}


def known() -> tuple[str, ...]:
    return tuple(sorted(RUNTIME_MODULES))


def ports_for(simulator: str) -> SimulatorPorts:
    module_name = RUNTIME_MODULES.get(simulator)
    if module_name is None:
        raise ValueError(
            f"unknown simulator {simulator!r}; configured simulators are "
            f"{list(RUNTIME_MODULES)}"
        )
    return import_module(module_name).describe()


def capabilities_for(simulator: str):
    return ports_for(simulator).capabilities()


def build_session(simulator: str, *, config, map_entry, family, resources, log):
    """Build the mission session that drives one map.

    Each simulator package owns its own construction: UE5 needs a scene JSONC and a
    dual-vehicle world, UE4 needs an RPC port and a vehicle name. The caller says
    which map it chose and this resolves the package -- nobody else branches on a
    simulator name.
    """
    module_name = RUNTIME_MODULES.get(simulator)
    if module_name is None:
        raise ValueError(
            f"unknown simulator {simulator!r}; configured simulators are "
            f"{list(RUNTIME_MODULES)}"
        )
    module = import_module(module_name)
    builder = getattr(module, "build_session", None)
    if builder is None:
        raise ValueError(
            f"simulator {simulator!r} advertises no mission session; {module_name} "
            "must define build_session(config=..., map_entry=..., family=..., "
            "resources=..., log=...) before its maps can fly a task"
        )
    return builder(
        config=config, map_entry=map_entry, family=family, resources=resources, log=log
    )


def labels() -> dict[str, str]:
    """Console-facing names. Kept here so no caller imports a simulator package."""
    return {
        "airsim": "AirSim (UE4)",
        "projectairsim": "ProjectAirSim (UE5)",
    }
