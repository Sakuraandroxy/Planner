"""The UE4 stack must stay out of the HTTP process.

``planner/ue4`` is a verbatim copy of the old branch, and it pulls in the
AirSim client plus ``msgpack-rpc`` with its pinned old Tornado. Those
dependencies are what forced the per-family runtime process, so the boundary
that makes them safe is worth a test: if a developer imports the UE4 stack from
the main process, this fails and the isolation is gone.
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest

MAIN_PROCESS_MODULES = (
    "planner.config",
    "planner.server_bootstrap",
    "planner.adapters.http.server_api",
    "planner.application.server_controller",
    "planner.domain.runtime",
)

ROOT = Path(__file__).resolve().parents[1]

FORBIDDEN = ("planner.ue4", "airlab", "airsim", "msgpackrpc")


def loaded_forbidden(name):
    """Report forbidden modules after importing ``name`` in a fresh interpreter.

    A same-process ``sys.modules`` check would depend on test order, because
    another test imports the UE4 stack on purpose.
    """
    script = (
        "import importlib, json, sys; "
        f"importlib.import_module({name!r}); "
        "print(json.dumps([m for m in sys.modules "
        f"if m.startswith({FORBIDDEN!r})]))"
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stderr[-2000:]
    return set(json.loads(result.stdout.strip().rsplit("\n", 1)[-1]))


@pytest.mark.parametrize("name", MAIN_PROCESS_MODULES)
def test_main_process_never_loads_the_ue4_stack(name):
    assert not loaded_forbidden(name), (
        f"{name} pulled the UE4 stack into the HTTP process; the AirSim client "
        "and its pinned old Tornado must stay in the runtime child"
    )


def test_copied_ue4_stack_is_self_contained():
    """Every copied import resolves inside ``planner.ue4``."""
    import importlib

    for name in (
        "planner.ue4.config",
        "planner.ue4.bootstrap",
        "planner.ue4.server_worker",
        "planner.ue4.adapters.airsim.above_camera",
        "planner.ue4.workflows.visual_goal_above.workflow",
        "planner.ue4.capabilities.targeting.top_geometry",
    ):
        importlib.import_module(name)


def test_copied_config_package_still_finds_its_own_task_rules():
    """The copy kept its layout, so the packaged ``tasks/*.yaml`` resolves."""
    from pathlib import Path as _Path

    from planner.ue4.config.visual_goal_above import VisualGoalAboveConfig
    from planner.ue4.config.visual_goal_above import visual_goal_above_config

    packaged = _Path("planner/ue4/config/tasks/visual_goal_above.yaml")
    assert packaged.is_file(), "the UE4 task rules were not carried over with the copy"
    settings = visual_goal_above_config({}, _Path("config/base.yaml"))
    assert isinstance(settings, VisualGoalAboveConfig)
