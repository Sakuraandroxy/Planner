"""Task types are available by default; readiness still requires explicit evidence."""
from pathlib import Path
import sys

import pytest

from config import load_config
from planner.errors import ConfigurationError
from run_airsim_cli import parse_args
from tests.fakes.configuration import (
    MAIN_CONFIG, VISUAL_RULES_REFERENCE, main_config_data, visual_config_data, write_config,
)


def test_cli_default_and_shared_settings_follow_current_config(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["run_airsim_cli.py"])
    assert Path(parse_args().config).resolve() == MAIN_CONFIG
    config = load_config(MAIN_CONFIG)
    assert config.sliding_window.enabled
    assert config.sliding_window.max_observation_age_s == 120
    assert config.trajectory_planner.timeout_s == 120
    assert config.airsim.depth_horizontal_fov_deg == 90.0
    assert config.airsim.maximum_pair_time_gap_s == .05
    assert config.visual_goal.enabled and config.visual_goal.geometry_verified
    assert not config.basic_motion.free_space.confirmed and not config.progress_review.enabled
    assert config.visual_goal.completion_rules_file == VISUAL_RULES_REFERENCE


def test_four_backend_roles_keep_independent_credentials(monkeypatch):
    roles = {
        "task_parser": "TASK_PARSER_API_KEY",
        "trajectory_planner": "TRAJECTORY_PLANNER_API_KEY",
        "target_detection": "TARGET_DETECTION_API_KEY",
        "task_completion": "TASK_COMPLETION_API_KEY",
    }
    raw = main_config_data()
    for role, env_name in roles.items():
        assert raw[role]["api_key_env"] == env_name
        assert raw[role]["api_key"] in ("", "no-key")
        monkeypatch.setenv(env_name, "offline-" + role)
    config = load_config(MAIN_CONFIG)
    for role in roles:
        assert getattr(config, role).api_key == "offline-" + role


def test_explicit_visual_rule_is_relative_to_config_not_working_directory(tmp_path, monkeypatch):
    raw = visual_config_data()
    raw["visual_goal_navigation"]["enabled"] = True
    path = write_config(tmp_path / "mission.yaml", raw)
    other = tmp_path / "another-working-directory"
    other.mkdir()
    monkeypatch.chdir(other)
    config = load_config(path)
    assert config.visual_goal.completion_rules.radius_m == 4
    assert len(config.visual_goal.completion_rules.required_conditions) == 3


@pytest.mark.parametrize("case", ["missing", "malformed", "custom_missing"])
def test_invalid_explicit_rules_do_not_fall_back_to_repository_defaults(tmp_path, case):
    raw = visual_config_data()
    raw["visual_goal_navigation"]["enabled"] = True
    if case == "custom_missing":
        raw["visual_goal_navigation"]["completion_rules_file"] = "tasks/not-present.yaml"
    if case == "malformed":
        target = tmp_path / VISUAL_RULES_REFERENCE
        target.parent.mkdir()
        target.write_text("radius_m: -1\n", encoding="utf-8")
    path = write_config(tmp_path / "invalid-rules.yaml", raw, copy_visual_rules=False)
    with pytest.raises(ConfigurationError):
        load_config(path)


def test_unused_visual_task_does_not_read_its_rule_file(tmp_path):
    raw = main_config_data()
    raw["visual_goal_navigation"]["enabled"] = False
    raw["visual_goal_navigation"]["completion_rules_file"] = "tasks/not-present.yaml"
    config = load_config(write_config(tmp_path / "basic.yaml", raw, copy_visual_rules=False))
    assert not config.visual_goal.enabled
    assert not (tmp_path / "tasks").exists()


@pytest.mark.parametrize("filename", [
    "deepseek.yaml", "deepseek.example.yaml",
    "visual_goal_navigation.example.yaml", "visual_goal_navigation.local.yaml",
])
def test_removed_config_path_fails_explicitly_instead_of_loading_main(filename):
    with pytest.raises(ConfigurationError, match="invalid config"):
        load_config(MAIN_CONFIG.parent / filename)


def test_visual_task_is_enabled_when_flag_is_omitted(tmp_path):
    from config.visual_goal import VisualGoalConfig
    assert VisualGoalConfig().enabled
    raw = main_config_data()
    raw["visual_goal_navigation"].pop("enabled")
    # Use the packaged rules when testing the type default independently.
    raw["visual_goal_navigation"].pop("completion_rules_file")
    assert load_config(write_config(tmp_path / "default-task.yaml", raw)).visual_goal.enabled
