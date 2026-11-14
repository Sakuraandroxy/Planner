import pytest
import yaml

from planner.config import (
    DepthSettings,
    TaskCompletionSettings,
    TaskParserSettings,
    VisualGoalNavigationSettings,
    load_completion_rules,
    load_config,
)
from planner.domain.motion import ContinuousTrackingSettings
from planner.domain.sliding_window import SlidingWindowSettings


def _write(tmp_path, data):
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def test_reference_sections_have_defaults(tmp_path):
    config = load_config(_write(tmp_path, {}))
    assert config.depth == DepthSettings()
    assert config.continuous == ContinuousTrackingSettings()
    assert config.sliding_window == SlidingWindowSettings()
    assert config.task_completion == TaskCompletionSettings()
    assert config.task_parser == TaskParserSettings()
    assert config.visual_goal_navigation.enabled is True
    assert config.visual_goal_navigation.limits.confirmation_count == 2
    assert config.visual_goal_navigation.depth_foreground_filter.enabled is True


def test_reference_sections_are_parsed(tmp_path):
    config = load_config(
        _write(
            tmp_path,
            {
                "depth": {"mapping": "log_inverse_gray_v1", "min_m": 1.0, "max_m": 50.0},
                "motion": {"continuous": {"lookahead_time_s": 0.8}},
                "sliding_window": {
                    "enabled": True,
                    "max_buffer_points": 8,
                    "planning_interval_s": 4.0,
                },
                "task_completion": {
                    "url": "http://localhost:9000/v1",
                    "model": "judge",
                    "timeout_s": 12.0,
                    "max_completion_tokens": 512,
                    "request_format": "deepseek",
                },
                "trajectory_planner": {
                    "url": "https://api.anthropic.com",
                    "model": "claude-plan",
                    "request_format": "anthropic",
                    "reasoning_effort": "none",
                    "max_tokens": 512,
                },
                "visual_goal_navigation": {
                    "enabled": False,
                    "geometry_verified": True,
                    "collision_check_enabled": True,
                    "output_root": "tmp/visual",
                    "limits": {"confirmation_count": 3},
                    "depth_foreground_filter": {"pixel_radius": 2},
                },
            },
        )
    )
    assert config.depth.min_m == 1.0
    assert config.depth.max_m == 50.0
    assert config.continuous.lookahead_time_s == 0.8
    assert config.sliding_window.enabled is True
    assert config.sliding_window.max_buffer_points == 8
    assert config.task_completion.model == "judge"
    assert config.task_completion.request_format == "deepseek"
    assert config.trajectory_planner.model == "claude-plan"
    assert config.trajectory_planner.request_format == "anthropic"
    assert config.trajectory_planner.reasoning_effort == "none"
    assert config.trajectory_planner.max_tokens == 512
    assert config.visual_goal_navigation.enabled is False
    assert config.visual_goal_navigation.geometry_verified is True
    assert config.visual_goal_navigation.collision_check_enabled is True
    assert config.visual_goal_navigation.limits.confirmation_count == 3
    assert config.visual_goal_navigation.depth_foreground_filter.pixel_radius == 2


def test_task_parser_section_is_parsed(tmp_path):
    config = load_config(
        _write(
            tmp_path,
            {
                "task_parser": {
                    "url": "http://model/v1",
                    "model": "qwen-parser",
                    "api_key": "inline-secret",
                    "api_key_env": "PARSER_KEY",
                    "timeout_s": 30.0,
                    "max_completion_tokens": 512,
                    "reasoning_effort": "low",
                    "request_format": "deepseek",
                }
            },
        )
    )
    parser = config.task_parser
    assert parser.url == "http://model/v1"
    assert parser.model == "qwen-parser"
    assert parser.api_key == "inline-secret"
    assert parser.api_key_env == "PARSER_KEY"
    assert parser.timeout_s == 30.0
    assert parser.max_completion_tokens == 512
    assert parser.reasoning_effort == "low"
    assert parser.request_format == "deepseek"


@pytest.mark.parametrize(
    "sections",
    [
        {"depth": {"mapping": "unknown_mapping"}},
        {"depth": {"mapping": "log_inverse_gray_v1", "min_m": 0.0}},
        {"motion": {"continuous": {"unknown": 1.0}}},
        {"sliding_window": {"unknown": 1}},
        {"task_completion": {"request_format": "xml"}},
        {"task_completion": {"reasoning_effort": "extreme"}},
        {"task_completion": {"max_completion_tokens": 0}},
        {"task_parser": {"timeout_s": 0}},
        {"task_parser": {"max_completion_tokens": 0}},
        {"task_parser": {"request_format": "xml"}},
        {"task_parser": {"request_format": "gemini"}},
        {"task_parser": {"request_format": "anthropic", "reasoning_effort": "high"}},
        {"trajectory_planner": {"request_format": "xml"}},
        {"trajectory_planner": {"request_format": "anthropic", "reasoning_effort": "low"}},
        {"trajectory_planner": {"reasoning_effort": "extreme"}},
        {"task_parser": {"reasoning_effort": "extreme"}},
        {"visual_goal_navigation": {"enabled": 1}},
        {"visual_goal_navigation": {"limits": {"confirmation_count": 1}}},
        {"visual_goal_navigation": {"limits": {"maximum_target_position_jump_m": 1.0}}},
        {"visual_goal_navigation": {"limits": {"target_center_margin": 1.0}}},
        {"visual_goal_navigation": {"depth_foreground_filter": {"pixel_radius": 0}}},
    ],
)
def test_invalid_reference_sections_are_rejected(tmp_path, sections):
    with pytest.raises(ValueError):
        load_config(_write(tmp_path, sections))


def test_completion_rules_load_from_the_reference_file(tmp_path):
    config_path = _write(tmp_path, {})
    rules_path = tmp_path / "rules.yaml"
    rules_path.write_text(
        yaml.safe_dump(
            {
                "radius_m": 7.0,
                "pause_radius_m": 6.5,
                "required_conditions": ["target matches", "inside radius"],
            }
        ),
        encoding="utf-8",
    )
    rules = load_completion_rules(config_path, "rules.yaml")
    assert rules.radius_m == 7.0
    assert rules.effective_pause_radius_m == 6.5
    assert rules.required_conditions == ("target matches", "inside radius")


def test_completion_rules_reject_unknown_fields(tmp_path):
    config_path = _write(tmp_path, {})
    rules_path = tmp_path / "rules.yaml"
    rules_path.write_text(
        yaml.safe_dump({"radius_m": 7.0, "unexpected": True}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        load_completion_rules(config_path, "rules.yaml")


def test_visual_goal_settings_validate_text_fields():
    with pytest.raises(ValueError):
        VisualGoalNavigationSettings(output_root="")
    with pytest.raises(ValueError):
        VisualGoalNavigationSettings(completion_rules_file="")