"""Dual-vehicle scenes spawn both vehicles at the same road-centre start.

The console shows only one vehicle at a time; the inactive one is parked far
away at startup and restored when selected. Both must therefore share the
same horizontal spawn point so switching vehicles does not move the camera
to a different part of the map.
"""

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

SCENES = {
    "city": "config/scenes/city/scene_city_dual.jsonc",
    "neighborhood": (
        "config/scenes/neighborhood/scene_neighborhood_dual.jsonc"
    ),
}


def _origins(scene_path: Path) -> dict[str, str]:
    scene = json.loads(scene_path.read_text(encoding="utf-8"))
    return {
        actor["name"]: actor["origin"]["xyz"]
        for actor in scene["actors"]
        if actor.get("type") == "robot"
    }


def _xy(xyz: str) -> tuple[float, float]:
    north, east, _down = (float(part) for part in xyz.split())
    return north, east


@pytest.mark.parametrize("map_id", sorted(SCENES))
def test_both_vehicles_exist(map_id):
    origins = _origins(ROOT / SCENES[map_id])
    assert {"Rover1", "Drone1"} <= set(origins)


@pytest.mark.parametrize("map_id", sorted(SCENES))
def test_vehicles_share_the_start_point(map_id):
    origins = _origins(ROOT / SCENES[map_id])
    rover_n, rover_e = _xy(origins["Rover1"])
    drone_n, drone_e = _xy(origins["Drone1"])
    distance = ((rover_n - drone_n) ** 2 + (rover_e - drone_e) ** 2) ** 0.5
    assert distance < 0.5, (
        f"{map_id}: Rover1 and Drone1 spawn {distance:.2f} m apart; "
        "both must start at the same road-centre point"
    )


@pytest.mark.parametrize("map_id", sorted(SCENES))
def test_vehicles_start_on_the_same_ground_point(map_id):
    origins = _origins(ROOT / SCENES[map_id])
    # AirSim coordinates are NED; equal z means both bodies are authored on
    # the road surface. The session lifts the inactive one before the clock
    # ever advances, so the two colliders never overlap.
    assert origins["Drone1"] == origins["Rover1"], (
        f"{map_id}: both vehicles must share the exact spawn point "
        f"(drone {origins['Drone1']!r} vs rover {origins['Rover1']!r})"
    )


@pytest.mark.parametrize("map_id", sorted(SCENES))
def test_scene_clock_starts_paused(map_id):
    scene = json.loads((ROOT / SCENES[map_id]).read_text(encoding="utf-8"))
    assert scene["clock"]["pause-on-start"] is True, (
        f"{map_id}: the clock must stay paused until the session has lifted "
        "the inactive vehicle off the shared spawn point"
    )
