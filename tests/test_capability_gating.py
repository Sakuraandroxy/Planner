"""Capability gating: the console's optional controls answer from port slots.

These tests use the shipped ``config/maps.yaml`` and the real registry, so they
also prove the derivation is wired into the API rather than bolted on beside it.
"""

from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from planner.adapters.http.server_api import create_api
from planner.config import ServerSettings, load_map_catalog
from planner.domain import ServerState
from planner.runtime import FrameHub
from planner.runtime import TrajectoryHub
from tests.test_server_api import FakeController

ROOT = Path(__file__).resolve().parents[1]

AIRSIM_MAP = "airsimnh"
PAS_MAP = "cityenviron"


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

    controller.map = AIRSIM_MAP
    payload = live.get("/api/status").json()
    assert payload["simulator"] == "airsim"
    assert payload["capabilities"]["vehicles"] == ["drone"]
    assert payload["capabilities"]["view_modes"] == []
    assert payload["capabilities"]["actions"] == ["weather"]


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


def test_trajectory_read_is_not_a_simulator_capability(client):
    """The remaining path is a service feature, so no family may lose it.

    The shared workflows write the hub and the endpoint reads it; the port
    bundle used to hold the same hub only so a flag could hide the card. UE4
    answered 501 for a trajectory it was already producing.
    """
    live, controller = client
    assert live.get("/api/trajectory").status_code == 200
    controller.map = AIRSIM_MAP
    response = live.get("/api/trajectory")
    assert response.status_code == 200, response.text
    assert "points" in response.json()


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


def test_airsim_map_switch_reaches_the_controller(client):
    """The xpra_start driver exists, so an AirSim map must not be refused.

    Routing goes through the family router, so handing a UE4 map to the
    ProjectAirSim launch script is no longer a failure mode this layer guards.
    """
    live, controller = client
    response = live.post("/api/world", json={"map": AIRSIM_MAP})
    assert response.status_code == 202, response.text
    assert controller.world_calls == [AIRSIM_MAP]
