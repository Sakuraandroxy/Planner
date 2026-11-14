"""Simulator family catalog: defaults, validation and per-family capabilities."""

from pathlib import Path

import pytest
import yaml

from planner.config import MapEntry, SimulatorSettings, load_config
from planner.domain.runtime import (
    Capabilities,
    SimulatorFamily,
    parse_family_catalog,
    ue5_family,
)

ROOT = Path(__file__).resolve().parents[1]


def catalog(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def test_repository_file_declares_both_families():
    families = parse_family_catalog(catalog(ROOT / "config/simulators.yaml"))
    assert families.ids == ("ue5", "ue4")
    assert families.default_family == "ue5"

    ue5 = families.require("ue5")
    assert ue5.driver == "script"
    assert ue5.ports["services"] == 8992
    assert "free_navigation" in ue5.capabilities.task_kinds
    assert "visual_goal_above" not in ue5.capabilities.task_kinds

    ue4 = families.require("ue4")
    assert ue4.driver == "xpra_start"
    assert ue4.ports == {"xpra": 10010, "rpc": 41451}
    # UE4 AirSim is drone-only and has no streaming camera cycle.
    assert ue4.capabilities.vehicles == ("drone",)
    assert ue4.capabilities.view_modes == ()
    assert ue4.capabilities.supports_views is False
    # Only weather is offered; home/models/trajectory/video are absent.
    assert ue4.capabilities.actions == ("weather",)
    assert not ue4.capabilities.supports("home")
    assert "visual_goal_above" in ue4.capabilities.task_kinds
    # The one AirSim spelling difference is data, not an if-branch.
    assert ue4.capabilities.weather_fields["road_wetness"] == "roadwetness"
    assert ue5.capabilities.weather_fields["road_wetness"] == "road_wetness"


def test_ue4_argv_carries_the_airsim_settings_file_through():
    families = parse_family_catalog(catalog(ROOT / "config/simulators.yaml"))
    ue4 = families.require("ue4")
    assert ue4.settings_file == "/data/airsim/settings.json"
    child = " ".join(ue4.child_argv)
    assert "-settings={settings_file}" in child
    assert "{map.map_name}" in child
    assert any(part.startswith("/usr/bin/xpra") for part in ue4.argv)
    assert "--exit-with-children=yes" in ue4.argv
    assert ue4.stop_argv[:2] == ("/usr/bin/xpra", "stop")


def test_missing_catalog_file_keeps_todays_behaviour():
    settings = SimulatorSettings()
    with pytest.raises(ValueError, match="catalog is empty"):
        parse_family_catalog({}, fallback=None)

    families = parse_family_catalog({}, fallback=ue5_family(settings))
    assert families.ids == ("ue5",)
    assert families.default_family == "ue5"
    ue5 = families.require("ue5")
    assert ue5.launch_script == settings.launch_script
    assert ue5.stop_script == settings.stop_script
    assert ue5.ports == {
        "xpra": settings.xpra_port,
        "topics": settings.topics_port,
        "services": settings.services_port,
    }
    assert ue5.capabilities.supports("models")
    assert ue5.capabilities.view_modes == (
        "rover_front",
        "rover_chase",
        "drone_front",
        "drone_chase",
    )


def test_unknown_setting_is_rejected_rather_than_ignored():
    raw = {"ue5": {"label": "x", "driver": "script", "launch_script": "a", "stop_script": "b",
                   "ports": {"xpra": 1}, "typo_port": 9}}
    with pytest.raises(ValueError, match="unknown settings"):
        parse_family_catalog(raw)


def test_unknown_capability_key_is_rejected():
    raw = {"ue5": {"driver": "script", "launch_script": "a", "stop_script": "b",
                   "ports": {"xpra": 1}, "capabilities": {"vehicle": ["drone"]}}}
    with pytest.raises(ValueError, match="capabilities has unknown settings"):
        parse_family_catalog(raw)


def test_unknown_action_is_rejected():
    with pytest.raises(ValueError, match="unknown entries"):
        Capabilities(actions=("teleport",))


def family(**overrides):
    """A valid xpra_start family with one field broken per case."""
    options = dict(id="ue4", label="x", driver="xpra_start", ports={"rpc": 41451},
                   argv=("xpra", "start"), stop_argv=("xpra", "stop"),
                   capabilities=Capabilities(task_kinds=("basic_motion",)))
    options.update(overrides)
    return SimulatorFamily(**options)


def test_xpra_start_family_needs_both_argv_templates():
    with pytest.raises(ValueError, match="needs an argv template"):
        family(argv=())
    with pytest.raises(ValueError, match="needs a stop_argv template"):
        family(stop_argv=())


def test_script_family_needs_its_scripts():
    with pytest.raises(ValueError, match="needs launch_script and stop_script"):
        family(driver="script", argv=(), stop_argv=())


def test_unknown_driver_is_rejected():
    with pytest.raises(ValueError, match="driver must be one of"):
        family(driver="docker")


def test_binary_map_takes_its_launch_target_from_map_name():
    entry = MapEntry(id="blocks", label="Blocks", binary="/bin/Blocks", map_name="Blocks",
                     family="ue4")
    assert entry.launch_map == "Blocks"
    assert entry.uses_scene_config is False


def test_half_a_pair_is_rejected():
    with pytest.raises(ValueError, match="half a binary pair"):
        MapEntry(id="m", label="m", binary="/bin/m")
    with pytest.raises(ValueError, match="half a scene pair"):
        MapEntry(id="m", label="m", scene_file="s.jsonc")
    with pytest.raises(ValueError, match="must declare either"):
        MapEntry(id="m", label="m")


def test_repository_config_accepts_the_grown_catalog():
    config = load_config(ROOT / "config/base.yaml")
    assert set(config.maps.family_ids()) == {"ue5", "ue4"}
    # Appearance models stay scoped to the family that can switch them.
    assert {entry.id for entry in config.maps.maps if entry.family == "ue4"} <= set(
        config.maps.ids
    )
