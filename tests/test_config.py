import pytest
import yaml

from pathlib import Path

from planner.config import (
    MapCatalog,
    MapEntry,
    ServerSettings,
    SimulatorSettings,
    load_completion_rules,
    load_config,
    load_map_catalog,
)
from planner.domain.visual_goal_completion import VisualGoalTriggerRules


def _write(tmp_path, data):
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def test_server_defaults():
    settings = ServerSettings()
    assert settings.host == "127.0.0.1"
    assert settings.port == 8000
    assert settings.detector == "manual"
    assert settings.planner == "geometric"
    assert settings.allowed_origins == ("*",)
    assert settings.video_fps == 10.0


def test_server_section_is_parsed(tmp_path):
    path = _write(
        tmp_path,
        {
            "server": {
                "host": "0.0.0.0",
                "port": 8123,
                "detector": "vlm",
                "planner": "vlm",
                "manual_bbox": [0.1, 0.2, 0.3, 0.4],
                "manual_surface_points": [[0.2, 0.3], [0.25, 0.3]],
                "allowed_origins": ["http://localhost:5173"],
                "log_buffer_size": 42,
                "reconnect_interval_s": 1.5,
                "startup_timeout_s": 30.0,
                "video_fps": 6.0,
            }
        },
    )
    config = load_config(path)
    server = config.server
    assert server.host == "0.0.0.0"
    assert server.port == 8123
    assert server.detector == "vlm"
    assert server.planner == "vlm"
    assert server.manual_bbox == (0.1, 0.2, 0.3, 0.4)
    assert server.manual_surface_points == ((0.2, 0.3), (0.25, 0.3))
    assert server.allowed_origins == ("http://localhost:5173",)
    assert server.log_buffer_size == 42
    assert server.video_fps == 6.0


def test_empty_config_uses_server_defaults(tmp_path):
    config = load_config(_write(tmp_path, {}))
    assert config.server == ServerSettings()


@pytest.mark.parametrize(
    "server",
    [
        {"detector": "grounding"},
        {"planner": "fast"},
        {"port": 0},
        {"manual_bbox": [0.6, 0.2, 0.4, 0.3]},
        {"manual_surface_points": [[0.2, 0.3], [0.2, 0.3], [0.2, 0.3],
                                   [0.2, 0.3], [0.2, 0.3], [0.2, 0.3]]},
        {"allowed_origins": []},
        {"log_buffer_size": 0},
        {"reconnect_interval_s": 0},
        {"startup_timeout_s": 0},
        {"video_fps": 0},
        {"video_fps": 61},
    ],
)
def test_invalid_server_sections_are_rejected(tmp_path, server):
    with pytest.raises(ValueError):
        load_config(_write(tmp_path, {"server": server}))


def test_malformed_surface_points_are_rejected(tmp_path):
    path = _write(
        tmp_path, {"server": {"manual_surface_points": [[0.2, 0.3, 0.4]]}}
    )
    with pytest.raises(ValueError):
        load_config(path)


def test_completion_rules_default_to_the_bundled_reference_file(tmp_path):
    config = load_config(_write(tmp_path, {}))
    settings = config.visual_goal_navigation
    assert settings.completion_rules_file is None
    assert settings.completion_rules.radius_m == 7.0
    assert settings.completion_rules.pause_radius_m == 6.5
    assert settings.completion_rules.maximum_completion_checks == 10
    assert len(settings.completion_rules.required_conditions) == 3


def test_completion_rules_file_is_relative_to_the_config(tmp_path):
    tasks = tmp_path / "tasks"
    tasks.mkdir()
    (tasks / "custom.yaml").write_text(
        yaml.safe_dump(
            {
                "radius_m": 5.0,
                "pause_radius_m": 4.5,
                "rearm_translation_m": 0.4,
                "rearm_yaw_deg": 8.0,
                "maximum_completion_checks": 6,
                "required_conditions": ["first", "second"],
            }
        ),
        encoding="utf-8",
    )
    config = load_config(
        _write(
            tmp_path,
            {
                "visual_goal_navigation": {
                    "completion_rules_file": "tasks/custom.yaml"
                }
            },
        )
    )
    rules = config.visual_goal_navigation.completion_rules
    assert rules.radius_m == 5.0
    assert rules.pause_radius_m == 4.5
    assert rules.maximum_completion_checks == 6
    assert rules.required_conditions == ("first", "second")


def test_completion_rules_can_be_loaded_directly(tmp_path):
    path = tmp_path / "rules.yaml"
    path.write_text(
        yaml.safe_dump({"radius_m": 3.0, "required_conditions": ["only"]}),
        encoding="utf-8",
    )
    rules = load_completion_rules(tmp_path / "config.yaml", "rules.yaml")
    assert rules.radius_m == 3.0
    assert rules.effective_pause_radius_m == 3.0
    assert rules.required_conditions == ("only",)


def test_disabled_visual_goal_skips_the_rules_file(tmp_path):
    config = load_config(
        _write(
            tmp_path,
            {
                "visual_goal_navigation": {
                    "enabled": False,
                    "completion_rules_file": "missing.yaml",
                }
            },
        )
    )
    assert config.visual_goal_navigation.completion_rules == VisualGoalTriggerRules()


@pytest.mark.parametrize(
    "rules",
    [
        {"required_conditions": "text"},
        {"required_conditions": []},
        {"radius_m": 0},
        {"maximum_completion_checks": 0},
        {"unexpected": 1},
    ],
)
def test_invalid_completion_rules_are_rejected(tmp_path, rules):
    path = tmp_path / "rules.yaml"
    path.write_text(yaml.safe_dump(rules), encoding="utf-8")
    with pytest.raises(ValueError):
        load_completion_rules(tmp_path / "config.yaml", "rules.yaml")


def test_missing_completion_rules_file_is_rejected(tmp_path):
    with pytest.raises(OSError):
        load_completion_rules(tmp_path / "config.yaml", "absent.yaml")


@pytest.mark.parametrize(
    "settings",
    [
        {"completion_rules_file": ""},
        {"completion_rules_file": 3},
    ],
)
def test_empty_completion_rules_file_is_rejected(tmp_path, settings):
    with pytest.raises(ValueError):
        load_config(_write(tmp_path, {"visual_goal_navigation": settings}))


def test_projectairsim_defaults(tmp_path):
    config = load_config(_write(tmp_path, {}))
    assert config.projectairsim.address == "127.0.0.1"
    assert config.projectairsim.port_topics == 8991
    assert config.projectairsim.port_services == 8992
    assert config.projectairsim.scene_file == "scene_city_dual.jsonc"
    assert config.projectairsim.rover_name == "Rover1"
    assert config.projectairsim.drone_name == "Drone1"


def test_projectairsim_section_is_parsed(tmp_path):
    config = load_config(
        _write(
            tmp_path,
            {
                "projectairsim": {
                    "address": "10.0.0.5",
                    "port_topics": 9991,
                    "port_services": 9992,
                    "scene_file": "scene_neighborhood_dual.jsonc",
                    "sim_config_path": "config/scenes/neighborhood",
                    "delay_after_load_s": 3.0,
                    "rover_name": "Rover2",
                    "drone_name": "Drone2",
                    "front_camera": "EgoCamera",
                }
            },
        )
    )
    assert config.projectairsim.address == "10.0.0.5"
    assert config.projectairsim.port_topics == 9991
    assert config.projectairsim.scene_file == "scene_neighborhood_dual.jsonc"
    assert config.projectairsim.sim_config_path == "config/scenes/neighborhood"
    assert config.projectairsim.delay_after_load_s == 3.0
    assert config.projectairsim.rover_name == "Rover2"
    assert config.projectairsim.drone_name == "Drone2"
    assert config.projectairsim.front_camera == "EgoCamera"


@pytest.mark.parametrize(
    "projectairsim",
    [
        {"address": ""},
        {"port_topics": 0},
        {"port_services": 70000},
        {"timeout_s": 0.0},
        {"scene_file": ""},
        {"sim_config_path": ""},
        {"delay_after_load_s": -1.0},
        {"rover_name": ""},
        {"drone_name": ""},
    ],
)
def test_projectairsim_settings_are_validated(tmp_path, projectairsim):
    with pytest.raises(ValueError):
        load_config(_write(tmp_path, {"projectairsim": projectairsim}))


def test_camera_defaults_match_the_rover_front_camera(tmp_path):
    config = load_config(_write(tmp_path, {}))
    assert config.camera.width == 1280
    assert config.camera.height == 720
    assert config.camera.offset_m == (1.8, 0.0, -0.8)
    assert config.camera.mount_rpy_deg == (0.0, -3.0, 0.0)
    assert config.camera.depth_is_range is True


def test_simulator_defaults(tmp_path):
    config = load_config(_write(tmp_path, {}))
    simulator = config.simulator
    assert simulator.display == ":104"
    assert simulator.xpra_port == 10010
    assert simulator.topics_port == config.projectairsim.port_topics
    assert simulator.services_port == config.projectairsim.port_services
    assert simulator.runtime_dir == "output/xpra"
    assert simulator.map_marker_file == "output/xpra/current_map"
    assert simulator.restart_attempts == 2


def test_simulator_ports_must_match_projectairsim(tmp_path):
    with pytest.raises(ValueError, match="must match"):
        load_config(
            _write(tmp_path, {"simulator": {"topics_port": 9991}})
        )
    with pytest.raises(ValueError, match="must match"):
        load_config(
            _write(tmp_path, {"simulator": {"services_port": 9992}})
        )


@pytest.mark.parametrize(
    "simulator",
    [
        {"display": "104"},
        {"display": ":x"},
        {"xpra_port": 0},
        {"launch_timeout_s": 0},
        {"restart_attempts": 0},
        {"launch_script": ""},
    ],
)
def test_invalid_simulator_sections_are_rejected(tmp_path, simulator):
    with pytest.raises(ValueError):
        load_config(_write(tmp_path, {"simulator": simulator}))


def test_map_catalog_falls_back_to_one_entry(tmp_path):
    config = load_config(_write(tmp_path, {}))
    assert config.maps.ids == ("default",)
    assert config.maps.default_map == "default"
    entry = config.maps.maps[0]
    assert entry.label == "默认地图"
    assert entry.scene_file == config.projectairsim.scene_file
    assert entry.sim_config_path == config.projectairsim.sim_config_path
    assert entry.launch_map == "cityenviron"


def test_map_catalog_is_loaded_from_the_config_directory(tmp_path):
    (tmp_path / "maps.yaml").write_text(
        yaml.safe_dump(
            {
                "default_map": "neighborhood",
                "maps": [
                    {
                        "id": "cityenviron",
                        "label": "城市",
                        "launch_map": "cityenviron",
                        "scene_file": "scene_city_dual.jsonc",
                        "sim_config_path": "config/scenes/city",
                    },
                    {
                        "id": "neighborhood",
                        "label": "郊区",
                        "launch_map": "neighborhood",
                        "scene_file": "scene_neighborhood_dual.jsonc",
                        "sim_config_path": "config/scenes/neighborhood",
                    },
                ],
            }
        ),
        encoding="utf-8",
    )
    config = load_config(_write(tmp_path, {}))
    assert config.maps.default_map == "neighborhood"
    assert config.maps.ids == ("cityenviron", "neighborhood")
    assert config.maps.labels["neighborhood"] == "郊区"


def test_map_catalog_rejects_duplicate_ids_and_a_missing_default(tmp_path):
    base = {
        "label": "城市",
        "launch_map": "cityenviron",
        "scene_file": "scene_city_dual.jsonc",
        "sim_config_path": "config/scenes/city",
    }
    (tmp_path / "maps.yaml").write_text(
        yaml.safe_dump(
            {
                "default_map": "cityenviron",
                "maps": [
                    {"id": "cityenviron", **base},
                    {"id": "cityenviron", **base},
                ],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="unique"):
        load_config(_write(tmp_path, {}))

    (tmp_path / "maps.yaml").write_text(
        yaml.safe_dump(
            {
                "default_map": "moon",
                "maps": [{"id": "cityenviron", **base}],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="default_map"):
        load_config(_write(tmp_path, {}))


def test_map_catalog_checks_scene_paths_exist(tmp_path):
    (tmp_path / "maps.yaml").write_text(
        yaml.safe_dump(
            {
                "default_map": "cityenviron",
                "maps": [
                    {
                        "id": "cityenviron",
                        "label": "城市",
                        "launch_map": "cityenviron",
                        "scene_file": "scene_city_dual.jsonc",
                        "sim_config_path": "config/scenes/absent",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="sim_config_path"):
        load_config(_write(tmp_path, {}))

    (tmp_path / "maps.yaml").write_text(
        yaml.safe_dump(
            {
                "default_map": "cityenviron",
                "maps": [
                    {
                        "id": "cityenviron",
                        "label": "城市",
                        "launch_map": "cityenviron",
                        "scene_file": "absent_scene.jsonc",
                        "sim_config_path": "config/scenes/city",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="scene_file"):
        load_config(_write(tmp_path, {}))


def test_map_catalog_without_a_fallback_requires_the_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_map_catalog(tmp_path / "absent.yaml")


def test_repository_catalog_lists_the_supported_maps():
    root = Path(__file__).resolve().parents[1]
    config = load_config(root / "config/base.yaml")
    assert config.maps.ids == ("cityenviron", "neighborhood")
    assert config.maps.default_map == "cityenviron"
    assert "blocks" not in {entry.launch_map for entry in config.maps.maps}


def test_vehicle_models_default_to_the_packaged_look(tmp_path):
    config = load_config(_write(tmp_path, {}))
    assert config.vehicle_models.defaults == {
        "rover": "offroad",
        "drone": "quadrotor",
    }
    assert config.vehicle_models.group("drone").ids == ("quadrotor",)
    assert config.vehicle_models.group("rover").labels == {"offroad": "越野车"}


def test_repository_vehicle_models_offer_the_hexarotor(tmp_path):
    root = Path(__file__).resolve().parents[1]
    config = load_config(root / "config/base.yaml")
    drone = config.vehicle_models.group("drone")
    assert drone.ids == ("quadrotor", "hexarotor")
    assert drone.option("hexarotor").scene_for(
        "cityenviron", ""
    ) == "scene_city_dual_hexarotor.jsonc"
    assert drone.option("hexarotor").scene_for(
        "neighborhood", ""
    ) == "scene_neighborhood_dual_hexarotor.jsonc"
    assert config.vehicle_models.group("rover").ids == ("offroad",)


def test_vehicle_models_are_parsed_with_scene_variants(tmp_path):
    config = load_config(
        _write(
            tmp_path,
            {
                "vehicle_models": {
                    "rover": {
                        "default": "offroad",
                        "options": [{"id": "offroad", "label": "越野车"}],
                    },
                    "drone": {
                        "default": "quadrotor",
                        "options": [
                            {"id": "quadrotor", "label": "四旋翼"},
                            {
                                "id": "hexarotor",
                                "label": "六旋翼",
                                "scene_files": {
                                    "default": "scene_city_dual_hexarotor.jsonc"
                                },
                            },
                        ],
                    },
                }
            },
        )
    )
    drone = config.vehicle_models.group("drone")
    assert drone.option("hexarotor").scene_for("default", "fallback") == (
        "scene_city_dual_hexarotor.jsonc"
    )
    assert drone.option("quadrotor").scene_for("default", "fallback") == "fallback"


@pytest.mark.parametrize(
    "section",
    [
        {"boat": {"default": "offroad", "options": [{"id": "offroad", "label": "x"}]}},
        {"rover": {"default": "sports", "options": [{"id": "offroad", "label": "x"}]}},
        {"rover": {"default": "offroad", "options": []}},
        {
            "drone": {
                "default": "quadrotor",
                "options": [
                    {"id": "quadrotor", "label": "x"},
                    {
                        "id": "hexarotor",
                        "label": "x",
                        "scene_files": {"default": "absent_scene.jsonc"},
                    },
                ],
            }
        },
        {
            "drone": {
                "default": "quadrotor",
                "options": [
                    {"id": "quadrotor", "label": "x"},
                    {
                        "id": "hexarotor",
                        "label": "x",
                        "scene_files": {"moon": "scene_city_dual_hexarotor.jsonc"},
                    },
                ],
            }
        },
    ],
)
def test_invalid_vehicle_models_are_rejected(tmp_path, section):
    with pytest.raises(ValueError):
        load_config(_write(tmp_path, {"vehicle_models": section}))
