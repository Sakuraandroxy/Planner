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
import inspect
import json
import subprocess
import sys
from pathlib import Path

import pytest

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
    from planner.runtime import GuidedVehicle
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
    #: 段执行由 runtime 的 ``GuidedVehicle`` 兑现，引擎不再留委派。
    delegated = {"execute_segment": GuidedVehicle}
    for port in (Vehicle, StreamingVehicle):
        for cls in adapters:
            for name, value in vars(port).items():
                if callable(value) and not name.startswith("_") and name != "__subclasshook__":
                    owner = delegated.get(name, cls)
                    assert hasattr(owner, name), (
                        f"{owner.__name__} lost {port.__name__}.{name}"
                    )




#: 端口 -> 两种引擎上的实现体。UE4 不出 home、views、models、frames 四个端口，端口方法
#: 在实现体上缺席是允许的；缺席的成员不比对，但每个实现体至少要命中一个成员，否则这条
#: 用例对它等于空跑。
def _port_implementations():
    import planner.ports as ports
    from planner.simulators.airsim.vehicle import AirSimVehicle
    from planner.simulators.projectairsim.drone import ProjectAirSimDrone
    from planner.simulators.projectairsim.rover import ProjectAirSimRover
    from planner.simulators.projectairsim.session import ProjectAirSimMissionSession

    vehicles = (AirSimVehicle, ProjectAirSimRover, ProjectAirSimDrone)
    return {
        ports.Vehicle: vehicles,
        ports.StreamingVehicle: vehicles,
        ports.ReturnHome: (ProjectAirSimMissionSession, ProjectAirSimRover, ProjectAirSimDrone),
        ports.StreamingViews: (ProjectAirSimMissionSession,),
    }


def _parameter_shape(signature):
    return [
        (
            parameter.name,
            parameter.kind.name,
            "" if parameter.default is inspect.Parameter.empty else repr(parameter.default),
        )
        for parameter in signature.parameters.values()
    ]


def _return_shape(signature):
    annotation = signature.return_annotation
    if annotation is inspect.Signature.empty:
        return None
    text = annotation if isinstance(annotation, str) else getattr(annotation, "__name__", str(annotation))
    return text.strip("\"'")


@pytest.mark.parametrize(
    "port_name", ["Vehicle", "StreamingVehicle", "ReturnHome", "StreamingViews"]
)
def test_port_signatures_match_their_implementations(port_name):
    """端口形状只许有一份：实现体的参数名、默认值与返回标注必须与端口声明一致。

    ``hasattr`` 只能证明名字在，签名漂移它看不出来：``return_home`` 曾经三种形状、
    ``set_view_mode`` 声明 bool 却返回 str。返回标注只在一侧缺失，不参与比对。
    """
    import planner.ports as ports

    implementations = _port_implementations()
    port = getattr(ports, port_name)
    for cls in implementations[port]:
        compared = 0
        for name, member in vars(port).items():
            if name.startswith("_") or name == "__subclasshook__":
                continue
            implementation = inspect.getattr_static(cls, name, None)
            if isinstance(member, property):
                assert isinstance(implementation, property), (
                    f"{cls.__name__}.{name} 必须是 property：端口把它声明成属性"
                )
                port_signature = inspect.signature(member.fget)
                implementation_signature = inspect.signature(implementation.fget)
            elif callable(member):
                if implementation is None:
                    continue
                port_signature = inspect.signature(member)
                implementation_signature = inspect.signature(implementation)
            else:
                continue
            compared += 1
            expected_parameters = _parameter_shape(port_signature)
            actual_parameters = _parameter_shape(implementation_signature)
            assert actual_parameters == expected_parameters, (
                f"{cls.__name__}.{name} 的参数与 {port.__name__} 不一致；"
                f"端口是 {expected_parameters}，实现是 {actual_parameters}"
            )
            expected_return = _return_shape(port_signature)
            actual_return = _return_shape(implementation_signature)
            if expected_return is not None and actual_return is not None:
                assert actual_return == expected_return, (
                    f"{cls.__name__}.{name} 的返回标注与 {port.__name__} 不一致；"
                    f"端口是 {expected_return}，实现是 {actual_return}"
                )
        assert compared, f"{cls.__name__} 一个 {port.__name__} 成员都没实现"
