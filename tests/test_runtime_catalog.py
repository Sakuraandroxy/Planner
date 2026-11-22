"""Simulator catalog: connection data only, capabilities derived from ports."""

from pathlib import Path

import pytest
import yaml

from planner.config import (
    MapEntry,
    MapScene,
    default_config_path,
    load_config,
    load_map_catalog,
)
from planner.domain.runtime import parse_family_catalog
from planner.simulator import registry
from planner.simulators.airsim import runtime as airsim_runtime
from planner.simulators.projectairsim import runtime as pas_runtime

ROOT = Path(__file__).resolve().parents[1]


def catalog(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def test_repository_file_declares_only_connection_and_process_data():
    raw = catalog(ROOT / "config/simulators.yaml")
    families = parse_family_catalog(raw)
    assert set(families.ids) == {"projectairsim", "airsim"}
    for item in raw.values():
        if isinstance(item, dict):
            assert "capabilities" not in item


def test_capabilities_block_in_yaml_is_rejected():
    """A config file cannot state capabilities; the ports decide."""
    raw = {"airsim": {"ports": {"rpc": 41451}, "capabilities": {"vehicles": ["drone"]}}}
    with pytest.raises(ValueError, match="must not declare capabilities"):
        parse_family_catalog(raw)


def test_unknown_setting_is_rejected_rather_than_ignored():
    raw = {"airsim": {"ports": {"rpc": 1}, "typo_port": 9}}
    with pytest.raises(ValueError, match="unknown settings"):
        parse_family_catalog(raw)


def test_the_retired_script_driver_is_refused_with_a_sentence():
    """``driver: script`` existed, drove UE5 through a shell script, and is gone.

    An operator with an old config would otherwise meet "driver must be one of
    ['xpra_start']" and have no idea what to do about it.
    """
    with pytest.raises(ValueError, match="launch scripts were removed"):
        parse_family_catalog({"projectairsim": {"driver": "script", "ports": {"xpra": 1}}})


def test_derived_airsim_capabilities():
    caps = registry.capabilities_for("airsim")
    assert caps.vehicles == ("drone",)
    assert caps.view_modes == ()
    assert caps.actions == ("weather",)
    assert not caps.supports("home") and not caps.supports("models")
    assert "free_navigation" not in caps.task_kinds
    # The single AirSim spelling difference is carried by the adapter module.
    assert caps.weather_fields["road_wetness"] == "roadwetness"


def test_derived_projectairsim_capabilities():
    caps = registry.capabilities_for("projectairsim")
    assert caps.vehicles == ("rover", "drone")
    assert len(caps.view_modes) == 4
    assert set(caps.actions) == {"home", "models", "video", "weather"}
    assert "free_navigation" in caps.task_kinds
    assert caps.weather_fields["road_wetness"] == "road_wetness"


def test_unknown_simulator_names_itself_in_the_error():
    with pytest.raises(ValueError, match="unknown simulator"):
        registry.capabilities_for("ue9")


def test_an_empty_catalog_is_an_error_rather_than_a_default():
    """There used to be a fallback that synthesised the ProjectAirSim family.

    It kept a second copy of the argv template in Python, so the config file and
    the code could describe different commands, and a deployment without
    config/simulators.yaml would launch a stack its driver could not stop.
    """
    with pytest.raises(ValueError, match="config/simulators.yaml must define"):
        parse_family_catalog({})
    with pytest.raises(ValueError, match="at least one family"):
        parse_family_catalog({"default_simulator": "airsim"})


def test_map_without_simulator_takes_the_catalog_default(tmp_path):
    """Single-simulator deployments need not repeat the field in YAML."""
    (tmp_path / "maps.yaml").write_text(
        "default_map: m\nmaps:\n  - id: m\n    label: M\n    launch_map: m\n"
        "    scene:\n      file: scene_city_dual.jsonc\n"
        "      config_path: config/scenes/city\n",
        encoding="utf-8",
    )
    entry = load_map_catalog(
        tmp_path / "maps.yaml", default_simulator="projectairsim"
    ).entry("m")
    assert entry.simulator == "projectairsim"
    # But the dataclass itself has no opinion: an empty value is refused.
    with pytest.raises(ValueError, match="must be non-empty text"):
        MapEntry(id="m", label="m", simulator="")
    with pytest.raises(ValueError, match="must not contain whitespace"):
        MapEntry(id="m", label="m", simulator="two words")


def test_binary_map_takes_its_launch_target_from_map_name():
    entry = MapEntry(id="blocks", label="Blocks", simulator="airsim",
                     binary="/bin/Blocks", map_name="Blocks")
    assert entry.launch_map == "Blocks"
    assert entry.uses_scene_config is False


def test_half_a_pair_is_rejected():
    with pytest.raises(ValueError, match="half a binary pair"):
        MapEntry(id="m", label="m", simulator="airsim", binary="/bin/m")
    with pytest.raises(ValueError, match="half a pair"):
        MapScene(file="s.jsonc")


def test_scene_defaults_come_from_the_simulator(tmp_path):
    """A map writes only the scene settings it changes."""
    for name in ("a", "b"):
        (tmp_path / "scenes" / name).mkdir(parents=True)
        (tmp_path / "scenes" / name / f"scene_{name}.jsonc").write_text(
            "{}", encoding="utf-8"
        )
    (tmp_path / "maps.yaml").write_text(
        yaml.safe_dump(
            {
                "default_map": "a",
                "maps": [
                    {
                        "id": "a",
                        "label": "A",
                        "simulator": "pas",
                        "binary": "/sim/A",
                        "map_name": "A",
                        "scene": {
                            "file": "scene_a.jsonc",
                            "config_path": str(tmp_path / "scenes" / "a"),
                        },
                    },
                    {
                        "id": "b",
                        "label": "B",
                        "simulator": "pas",
                        "binary": "/sim/B",
                        "map_name": "B",
                        "scene": {
                            "file": "scene_b.jsonc",
                            "config_path": str(tmp_path / "scenes" / "b"),
                            "drone_name": "UAV2",
                        },
                    },
                ],
            }
        ),
        encoding="utf-8",
    )
    maps = load_map_catalog(
        tmp_path / "maps.yaml",
        scene_defaults={
            "pas": {
                "rover_name": "Rover9",
                "drone_name": "Drone9",
                "front_camera": "FrontCamera",
                "view_cycle_start": "rover_front",
            }
        },
    )
    inherits = maps.entry("a").scene
    assert inherits.rover_name == "Rover9"
    assert inherits.view_cycle_start == "rover_front"
    assert maps.entry("b").scene.drone_name == "UAV2"
    assert maps.entry("b").scene.rover_name == "Rover9"


def test_shipped_scene_defaults_fill_the_ue5_maps():
    """The three UE5 entries write only file, config path and variants."""
    config = load_config(default_config_path())
    scene = config.maps.entry("cityenviron").scene
    assert scene.rover_name == "Rover1"
    assert scene.front_camera == "FrontCamera"
    assert scene.view_cycle_start == "drone_front"
    assert scene.delay_after_load_s == 2.0
    assert scene.uncapturable_views == ("rover_chase",)


def test_map_binaries_hang_off_the_catalog_root(tmp_path):
    """One root line replaces a machine-specific prefix in every entry."""
    (tmp_path / "maps.yaml").write_text(
        yaml.safe_dump(
            {
                "root": "/opt/sims",
                "default_map": "a",
                "maps": [
                    {
                        "id": "a",
                        "label": "A",
                        "simulator": "airsim",
                        "binary": "pkg/UE/Bin",
                        "map_name": "A",
                    },
                    {
                        "id": "b",
                        "label": "B",
                        "simulator": "airsim",
                        "binary": "/elsewhere/Bin",
                        "map_name": "B",
                    },
                ],
            }
        ),
        encoding="utf-8",
    )
    maps = load_map_catalog(tmp_path / "maps.yaml")
    assert maps.entry("a").binary == "/opt/sims/pkg/UE/Bin"
    # An absolute path is kept: one entry may point outside the root.
    assert maps.entry("b").binary == "/elsewhere/Bin"


def test_a_relative_binary_needs_a_root(tmp_path):
    (tmp_path / "maps.yaml").write_text(
        yaml.safe_dump(
            {
                "default_map": "a",
                "maps": [
                    {
                        "id": "a",
                        "label": "A",
                        "simulator": "airsim",
                        "binary": "pkg/Bin",
                        "map_name": "A",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="declares no root"):
        load_map_catalog(tmp_path / "maps.yaml")


def test_a_map_without_scene_or_binary_is_rejected(tmp_path):
    (tmp_path / "maps.yaml").write_text(
        "default_map: m\nmaps:\n  - id: m\n    label: M\n    simulator: airsim\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="must declare either"):
        load_map_catalog(tmp_path / "maps.yaml")


def test_map_naming_an_unconfigured_simulator_is_rejected(tmp_path):
    (tmp_path / "maps.yaml").write_text(
        "default_map: m\nmaps:\n  - id: m\n    label: M\n    simulator: nothere\n"
        "    binary: /bin/m\n    map_name: M\n", encoding="utf-8")
    maps = load_map_catalog(tmp_path / "maps.yaml")
    assert maps.simulator_ids() == ("nothere",)


def test_port_slots_match_the_shipped_yaml_drivers():
    """The launch data in YAML and the package that owns it stay in step.

    Both families name the same driver: one process driver for every simulator, so
    the lifecycle knowledge has a single home. The xpra session flags are shared by
    YAML anchor, so equality here is the check that nobody forked them into two
    lists that can drift apart on the next edit.
    """
    raw = catalog(ROOT / "config/simulators.yaml")
    assert raw["airsim"]["driver"] == raw["projectairsim"]["driver"] == "xpra_start"
    assert raw["airsim"]["argv"] == raw["projectairsim"]["argv"]
    assert raw["airsim"]["stop_argv"] == raw["projectairsim"]["stop_argv"]
    assert raw["airsim"]["ports"]["rpc"] == 41451
    assert raw["projectairsim"]["ports"]["topics"] == 8991
    assert pas_runtime.PORT_SLOTS and airsim_runtime.PORT_SLOTS
    assert "controlled_capture" in pas_runtime.PORT_SLOTS
    assert "controlled_capture" not in airsim_runtime.PORT_SLOTS
