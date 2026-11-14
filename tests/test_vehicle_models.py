"""The hexarotor scene variants stay in step with the default dual scenes."""

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

VARIANTS = {
    "cityenviron": ROOT / "config/scenes/city/scene_city_dual_hexarotor.jsonc",
    "neighborhood": (
        ROOT / "config/scenes/neighborhood/scene_neighborhood_dual_hexarotor.jsonc"
    ),
}

DEFAULT_SCENES = {
    "cityenviron": ROOT / "config/scenes/city/scene_city_dual.jsonc",
    "neighborhood": (
        ROOT / "config/scenes/neighborhood/scene_neighborhood_dual.jsonc"
    ),
}


def _robot_config(scene_path: Path, name: str) -> str:
    scene = json.loads(scene_path.read_text(encoding="utf-8"))
    for actor in scene["actors"]:
        if actor.get("type") == "robot" and actor.get("name") == name:
            return actor["robot-config"]
    raise AssertionError(f"{scene_path} has no robot actor {name}")


def _origins(scene_path: Path) -> dict[str, str]:
    scene = json.loads(scene_path.read_text(encoding="utf-8"))
    return {
        actor["name"]: actor["origin"]["xyz"]
        for actor in scene["actors"]
        if actor.get("type") == "robot"
    }


@pytest.mark.parametrize("map_id", sorted(VARIANTS))
def test_variant_swaps_only_the_drone_robot_config(map_id):
    variant = VARIANTS[map_id]
    default = DEFAULT_SCENES[map_id]
    assert _robot_config(variant, "Drone1") == "robot_drone_hexarotor.jsonc"
    assert _robot_config(variant, "Rover1") == _robot_config(default, "Rover1")
    assert _origins(variant) == _origins(default)


@pytest.mark.parametrize("map_id", sorted(VARIANTS))
def test_hexarotor_robot_config_declares_six_rotors(map_id):
    config_path = VARIANTS[map_id].parent / "robot_drone_hexarotor.jsonc"
    config = json.loads(config_path.read_text(encoding="utf-8"))

    visual = config["links"][0]["visual"]["geometry"]
    assert visual == {
        "type": "unreal_mesh",
        "name": "/Drone/SimpleHexarotor",
    }
    props = [link for link in config["links"] if link["name"] != "Frame"]
    assert len(props) == 6
    assert {
        link["visual"]["geometry"]["name"] for link in props
    } == {"/Drone/Hexarotor_Prop"}

    actuators = [item for item in config["actuators"] if item["type"] == "rotor"]
    assert len(actuators) == 6
    assert len({item["name"] for item in actuators}) == 6

    controller = config["controller"]
    assert controller["type"] == "simple-flight-api"
    assert controller["airframe-setup"] == "hexarotor-x"
    order = controller["simple-flight-api-settings"]["actuator-order"]
    assert len(order) == 6
    assert {item["id"] for item in order} == {
        f"{actuator['name']}" for actuator in actuators
    }

    sensor_ids = {sensor["id"] for sensor in config["sensors"]}
    assert {"FrontCamera", "Chase", "DownCamera"} <= sensor_ids
