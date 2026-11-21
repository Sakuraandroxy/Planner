import pytest
import yaml

from planner.config import (
    ServerSettings,
    default_config_path,
    load_completion_rules,
    load_config,
    load_map_catalog,
)
from planner.domain.visual_goal_completion import VisualGoalTriggerRules
from tests.support import write_config

# One helper for every synthetic config directory in the suite.
_write = write_config


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
        {},
        app={
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


def test_server_and_process_come_from_app_yml(tmp_path):
    """A profile says nothing about the HTTP service or the simulator processes."""
    config = load_config(_write(tmp_path, {}))
    assert config.server.host == "127.0.0.1"
    assert config.server.port == 8000
    assert config.server.vehicles == ("rover", "drone")
    assert config.simulator.display == ":104"
    with pytest.raises(ValueError, match="do not belong to a profile"):
        load_config(_write(tmp_path, {"server": {"port": 8123}}, app={}))


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


def test_view_cycle_start_must_name_a_known_view(tmp_path):
    """A typo here silently shifts every view button by the same amount."""
    (tmp_path / "maps.yaml").write_text(
        yaml.safe_dump(
            {
                "default_map": "m",
                "maps": [
                    {
                        "id": "m",
                        "label": "M",
                        "simulator": "projectairsim",
                        "binary": "/bin/m",
                        "map_name": "M",
                        "scene": {
                            "view_cycle_start": "rover_top",
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="view_cycle_start"):
        load_config(_write(tmp_path, {}))


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
    assert simulator.runtime_dir == "output/xpra"
    assert simulator.map_marker_file == "output/xpra/current_map"
    assert simulator.restart_attempts == 2


def test_simulator_ports_live_in_the_family_block(tmp_path):
    """The process section used to carry xpra/topics/services ports too.

    Nothing read them except a check that they equalled the projectairsim section,
    which is what two homes for one number looks like from the outside. Ports are
    declared once, per family, in config/simulators.yaml.
    """
    config = load_config(_write(tmp_path, {}))
    assert not hasattr(config.simulator, "topics_port")
    families = config.families.require("projectairsim")
    assert families.ports == {"xpra": 10010, "topics": 8991, "services": 8992}


@pytest.mark.parametrize(
    "process",
    [
        {"display": "104"},
        {"display": ":x"},
        {"runtime_dir": " "},
        {"map_marker_file": " "},
        {"launch_timeout_s": 0},
        {"restart_attempts": 0},
    ],
)
def test_invalid_process_sections_are_rejected(tmp_path, process):
    """Process ownership lives with the families, not in the navigation core."""
    path = _write(tmp_path, {})
    catalog = tmp_path / "simulators.yaml"
    raw = yaml.safe_load(catalog.read_text(encoding="utf-8"))
    raw["process"] = process
    catalog.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ValueError):
        load_config(path)


def test_map_catalog_falls_back_to_one_entry(tmp_path):
    config = load_config(_write(tmp_path, {}))
    assert config.maps.ids == ("default",)
    assert config.maps.default_map == "default"
    entry = config.maps.maps[0]
    assert entry.label == "默认地图"
    # A catalog-less config carries its scene inline; the shipped core declares none.
    assert entry.scene_file == ""
    assert entry.launch_map == ""


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
                        "scene": {
                            "file": "scene_city_dual.jsonc",
                            "config_path": "config/scenes/city",
                        },
                    },
                    {
                        "id": "neighborhood",
                        "label": "郊区",
                        "launch_map": "neighborhood",
                        "scene": {
                            "file": "scene_neighborhood_dual.jsonc",
                            "config_path": "config/scenes/neighborhood",
                        },
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
        "scene": {
            "file": "scene_city_dual.jsonc",
            "config_path": "config/scenes/city",
        },
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
                        "scene": {
                            "file": "scene_city_dual.jsonc",
                            "config_path": "config/scenes/absent",
                        },
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
                        "scene": {
                            "file": "absent_scene.jsonc",
                            "config_path": "config/scenes/city",
                        },
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
    config = load_config(default_config_path())
    pas = tuple(e.id for e in config.maps.maps if e.simulator == "projectairsim")
    air = tuple(e.id for e in config.maps.maps if e.simulator == "airsim")
    assert pas == ("cityenviron", "neighborhood", "jiaoliucenter")
    # AirSim UE4 binaries come from airsim-flight-console's scene table.
    assert air == (
        "airsimnh", "abandonedpark", "msbuild2018", "trapcam", "blocks", "shanghai",
        "campus",
    )
    assert config.maps.default_map == "cityenviron"
    assert all(e.scene_file for e in config.maps.maps if e.simulator == "projectairsim")
    assert all(not e.sim_config_path for e in config.maps.maps if e.simulator == "airsim")
    assert set(config.families.ids) == {"projectairsim", "airsim"}
    # Scene files are a ProjectAirSim concern; AirSim maps launch a binary.
    pas_launches = {e.launch_map for e in config.maps.maps if e.simulator == "projectairsim"}
    assert "blocks" not in pas_launches


def test_vehicle_models_default_to_the_packaged_look(tmp_path):
    config = load_config(_write(tmp_path, {}))
    assert config.vehicle_models.defaults == {
        "rover": "offroad",
        "drone": "quadrotor",
    }
    # The list comes from the family that produces the meshes.
    assert config.vehicle_models.group("drone").ids == ("quadrotor", "hexarotor")
    assert config.vehicle_models.group("rover").labels == {"offroad": "越野车"}


def test_maps_declare_their_own_scene_variants(tmp_path):
    """A scene variant belongs to the map that pins it, not to the model list."""
    config = load_config(default_config_path())
    assert config.vehicle_models.group("drone").ids == ("quadrotor", "hexarotor")
    assert config.vehicle_models.group("rover").ids == ("offroad",)
    city = config.maps.entry("cityenviron").scene
    assert city.scene_for("quadrotor") == "scene_city_dual.jsonc"
    assert city.scene_for("hexarotor") == "scene_city_dual_hexarotor.jsonc"


def test_vehicle_model_scene_files_are_rejected(tmp_path):
    """The old per-model map table is refused instead of silently ignored."""
    with pytest.raises(ValueError, match="do not belong to a profile"):
        load_config(
            _write(
                tmp_path,
                {
                    "vehicle_models": {
                        "drone": {
                            "default": "quadrotor",
                            "options": [
                                {"id": "quadrotor", "label": "四旋翼"},
                                {
                                    "id": "hexarotor",
                                    "label": "六旋翼",
                                    "scene_files": {"default": "x.jsonc"},
                                },
                            ],
                        }
                    }
                },
            )
        )

@pytest.mark.parametrize(
    "section",
    [
        {"boat": {"default": "offroad", "options": [{"id": "offroad", "label": "x"}]}},
        {"rover": {"default": "sports", "options": [{"id": "offroad", "label": "x"}]}},
        {"rover": {"default": "offroad", "options": []}},
    ],
)
def test_invalid_vehicle_models_are_rejected(tmp_path, section):
    with pytest.raises(ValueError):
        load_config(_write(tmp_path, {"vehicle_models": section}))


def test_heading_settings_are_read_from_the_config_file(tmp_path):
    path = _write(tmp_path, {
        "motion": {
            "max_speed_mps": 4.0,
            "heading_hold_gain": 0.8,
            "heading_deadband_deg": 0.7,
        }
    })
    config = load_config(path)
    assert config.motion.heading_hold_gain == pytest.approx(0.8)
    assert config.motion.heading_deadband_deg == pytest.approx(0.7)
    tracking = config.motion.to_tracking_limits()
    assert tracking.heading_hold_gain == pytest.approx(0.8)


def test_heading_settings_fall_back_to_the_values_the_flight_was_tuned_with(tmp_path):
    """A config that omits the heading keys must get the validated pair."""
    motion = load_config(_write(tmp_path, {})).motion

    assert motion.heading_hold_gain == pytest.approx(0.4)
    assert motion.heading_rate_damping == pytest.approx(0.0)


def test_drone_takeoff_climb_is_configured_and_cannot_be_negative(tmp_path):
    path = _write(tmp_path, {"drone": {"takeoff_climb_m": 3.5}})
    assert load_config(path).drone.takeoff_climb_m == pytest.approx(3.5)
    assert load_config(_write(tmp_path, {})).drone.takeoff_climb_m == pytest.approx(1.0)
    with pytest.raises(ValueError, match="takeoff_climb_m"):
        load_config(_write(tmp_path, {"drone": {"takeoff_climb_m": -1.0}}))
