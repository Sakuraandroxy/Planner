"""Architecture rules for the two-peer simulator layout.

Three properties hold the design together, and each one is cheap to break
silently, which is why they are tests rather than comments:

1. Only ``planner/simulators/<name>/`` may import a simulator client library.
   This is what lets AirSim's ``msgpack-rpc`` with its pinned old Tornado and
   ProjectAirSim's ``pynng`` live on one machine.
2. The shared layer never imports a simulator package. If it did, the shared
   task logic would stop being shared.
3. Capability flags are derived from port slots, so no module needs to know
   which simulator is running.

Checks run over files or in a subprocess, so test order cannot mask a failure.
"""

from __future__ import annotations

import ast
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

#: Import roots that belong to exactly one simulator.
CLIENT_MODULES = frozenset({"airlab", "airsim", "msgpackrpc", "projectairsim", "pynng"})

#: Clients the serving process must never load. AirSim drags in
#: ``msgpack-rpc-python`` with its pinned old Tornado, which is the collision
#: the separate UE4 runtime process exists to avoid.
#:
#: ``projectairsim`` and ``pynng`` are deliberately absent: UE5 still runs its
#: mission loop in-process on a worker thread, so the serving interpreter does
#: load them today. Moving UE5 behind the same child-process boundary would let
#: this list cover every client, and that is the remaining step, not a rule
#: worth failing the build over now.
FORBIDDEN_IN_MAIN_PROCESS = frozenset({"airlab", "airsim", "msgpackrpc"})
SIMULATOR_PACKAGE = "planner.simulators"
#: Layers that must stay simulator-agnostic.
SHARED_PACKAGES = (
    "planner/domain",
    "planner/runtime",
    "planner/algorithms",
    "planner/workflows",
    "planner/ports",
    "planner/assembly",
    "planner/adapters/http",
    "planner/simulator",
)
MAIN_PROCESS_MODULES = (
    "planner.config",
    "planner.server_bootstrap",
    "planner.adapters.http.server_api",
    "planner.assembly.controller",
)


def _imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            found.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            found.add(node.module)
    return found


def _shared_files():
    for package in SHARED_PACKAGES:
        for path in sorted((ROOT / package).rglob("*.py")):
            if "__pycache__" in path.parts:
                continue
            yield path


def test_shared_layer_never_imports_a_simulator_client():
    offenders = {}
    for path in _shared_files():
        hit = sorted(name for name in _imports(path) if name.split(".")[0] in CLIENT_MODULES)
        if hit:
            offenders[str(path.relative_to(ROOT))] = hit
    assert not offenders, (
        "these shared modules import a simulator client; move the code into "
        f"{SIMULATOR_PACKAGE}/<name>/ instead: {offenders}"
    )


def test_shared_layer_never_imports_a_simulator_package():
    offenders = [
        str(path.relative_to(ROOT))
        for path in _shared_files()
        if any(name.startswith(SIMULATOR_PACKAGE) for name in _imports(path))
    ]
    assert not offenders, f"shared layer depends on a concrete simulator: {offenders}"


def test_main_process_never_loads_a_simulator_client():
    """Checked in a fresh interpreter so other tests cannot mask the result."""
    script = (
        "import importlib, json, sys; "
        f"[importlib.import_module(m) for m in {MAIN_PROCESS_MODULES!r}]; "
        "print(json.dumps(sorted(m for m in sys.modules "
        f"if m.split('.')[0] in {sorted(FORBIDDEN_IN_MAIN_PROCESS)!r})))"
    )
    result = subprocess.run(
        [sys.executable, "-c", script], cwd=ROOT, capture_output=True, text=True, timeout=180,
        check=False,
    )
    assert result.returncode == 0, result.stderr[-2000:]
    loaded = json.loads(result.stdout.strip().rsplit("\n", 1)[-1])
    assert not loaded, (
        f"the HTTP process pulled in the AirSim client stack: {loaded}. It must "
        "stay inside planner/simulators/airsim, which runs in its own process"
    )


def test_every_registered_simulator_declares_its_slots():
    from planner.assembly import requirements
    from planner.simulator import registry

    assert set(registry.known()) == {"airsim", "projectairsim"}
    assert set(registry.labels()) == {"airsim", "projectairsim"}
    for simulator in registry.known():
        ports = registry.ports_for(simulator)
        assert ports.slot("vehicle") is not None, f"{simulator} has no vehicle port"
        assert requirements.capabilities_for(ports).task_kinds, (
            f"{simulator} offers no task kind"
        )


def test_airsim_slots_are_backed_by_real_code():
    """PORT_SLOTS may not claim a capability nothing implements."""
    from dataclasses import fields as dc_fields

    from planner.ports import StreamingVehicle
    from planner.ports import Vehicle
    from planner.simulator.ports import SimulatorPorts
    from planner.simulators.airsim.vehicle import AirSimVehicle
    from planner.simulators.projectairsim.drone import ProjectAirSimDrone
    from planner.simulators.projectairsim.rover import ProjectAirSimRover

    known = {item.name for item in dc_fields(SimulatorPorts)}
    for simulator in ("airsim", "projectairsim"):
        module = __import__(f"{SIMULATOR_PACKAGE}.{simulator}.runtime", fromlist=["PORT_SLOTS"])
        unknown = [slot for slot in module.PORT_SLOTS if slot not in known]
        assert not unknown, f"{simulator} declares slots the bundle has no field for: {unknown}"

    adapters = (AirSimVehicle, ProjectAirSimRover, ProjectAirSimDrone)
    for port in (Vehicle, StreamingVehicle):
        for cls in adapters:
            for name, value in vars(port).items():
                if callable(value) and not name.startswith("_") and name != "__subclasshook__":
                    assert hasattr(cls, name), f"{cls.__name__} lost {port.__name__}.{name}"


