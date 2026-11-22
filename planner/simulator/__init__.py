"""Simulator infrastructure shared by every simulator: process stack and worker.

Nothing in here may import a simulator client library. Simulator-specific code
lives in ``planner.simulators.<name>``.

``SimulatorPorts`` is re-exported here because it is the contract every engine
hands back: simulators and the assembly read it from this entry, never from
``planner.simulator.ports``.
"""

from .ports import SimulatorPorts

__all__ = ["SimulatorPorts"]
