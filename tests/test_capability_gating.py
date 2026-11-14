"""Capability gating: the console's optional controls answer from port slots.

These tests use the shipped ``config/maps.yaml`` and the real registry, so they
also prove the derivation is wired into the API rather than bolted on beside it.
"""

from pathlib import Path

import pytest
import yaml
from fastapi.testclient import TestClient

from planner.adapters.http.server_api import create_api
from planner.config import ServerSettings, SimulatorSettings, load_map_catalog, projectairsim_family
from planner.domain.runtime import parse_family_catalog
from planner.domain.server import ServerState
from planner.services.frame_hub import FrameHub
from planner.services.trajectory_hub import TrajectoryHub

from tests.test_server_api import FakeController

ROOT = Path(__file__).resolve().parents[1]

AIRSIM_MAP = "airsimnh"
PAS_MAP = "cityenviron"


def _families():
    return parse_family_catalog(
        yaml.safe_load((ROOT / "config/simulators.yaml").read_text(encoding="utf-8")),
        fallback=projectairsim_family(SimulatorSettings()),
    )


@pytest.fixture()
def client():
    """A controller whose active map can be flipped between simulators."""
    controller = FakeController(ServerState.IDLE)
    controller.maps = load_map_catalog(ROOT / "config/maps.yaml")
    app = create_api(
        controller,
        ServerSettings(),
        frames=FrameHub(max_fps=5.0),
        trajectory_hub=TrajectoryHub(),
        families=_families(),
    )
    with TestClient(app) as live:
        yield live, controller


def test_status_publishes_the_active_simulator_and_derived_flags():
    """Flags come from the registry, so they appear without any family catalog."""
    controller = FakeController(ServerState.IDLE)
    controller.maps = load_map_catalog(ROOT / "config/maps.yaml")
    with TestClient(create_api(controller, ServerSettings())) as live:
        payload = live.get("/api/status").json()
        assert payload["simulator"] == "projectairsim"
        assert payload["capabilities"]["vehicles"] == ["rover", "drone"]
        assert "home" in payload["capabilities"]["actions"]
        # No gate without a family catalog: the endpoint still answers.
        assert live.post("/api/home").status_code == 202


def test_status_publishes_the_active_simulator(client):
    live, controller = client
    payload = live.get("/api/status").json()
    assert payload["simulator"] == "projectairsim"
    assert "visual_goal_above" not in payload["capabilities"]["task_kinds"]

    controller.map = AIRSIM_MAP
    payload = live.get("/api/status").json()
    assert payload["simulator"] == "airsim"
    assert payload["capabilities"]["vehicles"] == ["drone"]
    assert payload["capabilities"]["view_modes"] == []
    assert payload["capabilities"]["actions"] == ["weather"]
    assert "visual_goal_above" in payload["capabilities"]["task_kinds"]


def test_world_options_map_every_scene_to_a_simulator(client):
    live, _ = client
    payload = live.get("/api/world/options").json()
    assert payload["map_simulators"][AIRSIM_MAP] == "airsim"
    assert payload["map_simulators"][PAS_MAP] == "projectairsim"
    assert set(payload["simulator_labels"]) == {"airsim", "projectairsim"}
    # The console builds its map list from this, so AirSim scenes must appear.
    assert AIRSIM_MAP in payload["maps"]


@pytest.mark.parametrize(
    "method,path,payload",
    [
        ("POST", "/api/home", None),
        ("GET", "/api/models", None),
        # A body that would pass validation, so 501 proves the gate ran.
        ("POST", "/api/models", {"models": {"drone": "quadrotor"}}),
        ("GET", "/api/trajectory", None),
        ("GET", "/api/video", None),
    ],
)
def test_airsim_answers_501_for_ports_it_does_not_implement(client, method, path, payload):
    live, controller = client
    controller.map = AIRSIM_MAP
    response = live.request(method, path, json=payload)
    assert response.status_code == 501, response.text
    detail = response.json()["detail"]
    assert detail["simulator"] == "airsim"
    assert detail["unsupported"] in path


def test_projectairsim_still_offers_those_controls(client):
    live, _ = client
    assert live.post("/api/home").status_code == 202
    assert live.get("/api/models").status_code == 200
    assert live.get("/api/trajectory").status_code == 200


def test_weather_is_offered_by_both_simulators(client):
    live, controller = client
    assert live.get("/api/weather").status_code == 200
    controller.map = AIRSIM_MAP
    assert live.get("/api/weather").status_code == 200


def test_task_submission_needs_no_optional_port(client):
    """Flying is the base contract; both simulators accept a task."""
    live, controller = client
    assert live.post("/api/tasks", json={"instruction": "前进一米"}).status_code == 202
    controller.map = AIRSIM_MAP
    controller.state_value = ServerState.IDLE
    assert live.post("/api/tasks", json={"instruction": "前进一米"}).status_code == 202


def test_airsim_map_is_refused_until_its_process_driver_exists(client):
    """The xpra_start driver is not implemented yet.

    Handing an AirSim map to the ProjectAirSim launch script would start the
    wrong binary on the wrong ports, so the API refuses the switch and names the
    missing piece instead.
    """
    live, controller = client
    response = live.post("/api/world", json={"map": AIRSIM_MAP})
    assert response.status_code == 501, response.text
    detail = response.json()["detail"]
    assert detail["simulator"] == "airsim"
    assert detail["unsupported"] == "driver:xpra_start"
    assert controller.world_calls == []
    assert live.post("/api/world", json={"map": "neighborhood"}).status_code == 202
