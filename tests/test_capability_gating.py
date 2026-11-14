"""Capability gating: optional controls answer from the map's family flags.

The shipped ``config/simulators.yaml`` and ``config/maps.yaml`` are used on
purpose, so these tests also prove the real files parse into a working gate and
that UE4 maps really advertise drone-only with no home, models or trajectory.
"""

from pathlib import Path

import pytest
import yaml
from fastapi.testclient import TestClient

from planner.adapters.http.server_api import create_api
from planner.config import ServerSettings, SimulatorSettings, load_map_catalog
from planner.domain.runtime import parse_family_catalog, ue5_family
from planner.domain.server import ServerState
from planner.services.frame_hub import FrameHub
from planner.services.trajectory_hub import TrajectoryHub

from tests.test_server_api import FakeController

ROOT = Path(__file__).resolve().parents[1]

UE4_MAP = "airsimnh"


@pytest.fixture()
def client():
    """A controller whose active map can be flipped between families."""
    # The files the repository ships are the ones under test.
    families = parse_family_catalog(
        yaml.safe_load((ROOT / "config/simulators.yaml").read_text(encoding="utf-8")),
        fallback=ue5_family(SimulatorSettings()),
    )
    maps = load_map_catalog(ROOT / "config/maps.yaml")
    controller = FakeController(ServerState.IDLE)
    controller.maps = maps
    controller.vehicles = tuple(families.capabilities("ue5").vehicles)
    app = create_api(
        controller,
        ServerSettings(),
        frames=FrameHub(max_fps=5.0),
        trajectory_hub=TrajectoryHub(),
        families=families,
    )
    with TestClient(app) as live:
        yield live, controller


def test_status_publishes_the_active_family_and_its_flags(client):
    live, controller = client
    payload = live.get("/api/status").json()
    assert payload["family"] == "ue5"
    assert payload["capabilities"]["vehicles"] == ["rover", "drone"]
    assert "home" in payload["capabilities"]["actions"]

    controller.map = UE4_MAP
    payload = live.get("/api/status").json()
    assert payload["family"] == "ue4"
    assert payload["capabilities"]["vehicles"] == ["drone"]
    assert payload["capabilities"]["view_modes"] == []
    assert payload["capabilities"]["actions"] == ["weather"]


def test_world_options_maps_every_scene_to_a_family(client):
    live, _ = client
    payload = live.get("/api/world/options").json()
    assert payload["map_families"][UE4_MAP] == "ue4"
    assert payload["map_families"]["cityenviron"] == "ue5"
    assert set(payload["family_labels"]) == {"ue5", "ue4"}
    # The console renders its map list from this, so UE4 must appear.
    assert UE4_MAP in payload["maps"]


@pytest.mark.parametrize(
    "method,path,payload",
    [
        ("post", "/api/home", None),
        ("get", "/api/models", None),
        # A body that would pass validation, so 501 proves the gate ran.
        ("post", "/api/models", {"models": {"drone": "quadrotor"}}),
        ("get", "/api/trajectory", None),
        ("get", "/api/video", None),
    ],
)
def test_ue4_answers_501_for_controls_it_does_not_offer(client, method, path, payload):
    live, controller = client
    controller.map = UE4_MAP
    response = live.request(method.upper(), path, json=payload)
    assert response.status_code == 501, response.text
    assert response.json()["detail"]["family"] == "ue4"
    assert response.json()["detail"]["unsupported"] in path


def test_ue5_still_offers_those_controls(client):
    live, _ = client
    assert live.post("/api/home").status_code == 202
    assert live.get("/api/models").status_code == 200
    assert live.get("/api/trajectory").status_code == 200


def test_weather_is_offered_on_both_families(client):
    live, controller = client
    assert live.get("/api/weather").status_code == 200
    controller.map = UE4_MAP
    assert live.get("/api/weather").status_code == 200


def test_ue4_refuses_the_rover_it_cannot_drive(client):
    live, controller = client
    controller.map = UE4_MAP
    response = live.post("/api/vehicle", json={"vehicle": "rover"})
    assert response.status_code == 400
    assert response.json()["detail"]["message"]
    assert controller.activate_calls == []
    # The drone stays reachable on the same map.
    assert live.post("/api/vehicle", json={"vehicle": "drone"}).status_code == 202


def test_api_without_a_catalog_keeps_the_old_shape():
    """Deployments with no families configured must not see new fields or 501s."""
    controller = FakeController(ServerState.IDLE)
    app = create_api(controller, ServerSettings())
    with TestClient(app) as live:
        assert "capabilities" not in live.get("/api/status").json()
        assert "map_families" not in live.get("/api/world/options").json()
        assert live.post("/api/home").status_code == 202


def test_ue4_map_is_refused_until_its_process_driver_exists(client):
    """The xpra_start driver is not implemented yet.

    Handing a UE4 map to the UE5 launch script would start the wrong binary on
    the wrong ports, so the API refuses the switch and names the missing piece.
    """
    live, controller = client
    response = live.post("/api/world", json={"map": UE4_MAP})
    assert response.status_code == 501, response.text
    detail = response.json()["detail"]
    assert detail["family"] == "ue4"
    assert detail["unsupported"] == "driver:xpra_start"
    assert controller.world_calls == []
    # The UE5 maps still switch normally.
    assert live.post("/api/world", json={"map": "neighborhood"}).status_code == 202
