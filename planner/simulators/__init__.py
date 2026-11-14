"""One package per simulator, as peers.

Each package owns the code that imports its client library, plus the assembly
that fills in ``planner.simulator.ports.SimulatorPorts``. Nothing here is the
default: the shared layer receives a port bundle and never learns a package name.
"""
