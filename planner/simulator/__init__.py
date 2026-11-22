"""Simulator infrastructure shared by every simulator: process stack and worker.

Nothing in here may import a simulator client library. Simulator-specific code
lives in ``planner.simulators.<name>``.

``SimulatorPorts`` is re-exported here because it is the contract every engine
hands back: simulators and the assembly read it from this entry, never from
``planner.simulator.ports``. ``BackendWorker`` is re-exported for the same
reason: the assembly drives the process stack through this protocol.
"""

from .ports import SimulatorPorts
from .worker_protocol import BackendWorker

__all__ = ["BackendWorker", "SimulatorPorts"]
