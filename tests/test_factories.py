import json

import pytest
import yaml

from planner.adapters.diagnostics import VisualGoalTrace
from planner.adapters.task_completion.prompts import VisualGoalCompletionPrompt
from planner.adapters.task_parser import OpenAITaskParser
from planner.adapters.trajectory_planner import QwenVLPlanner
from planner.adapters.trajectory_planner.prompts import (
    AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT,
    AERIAL_VISUAL_GOAL_SYSTEM_PROMPT,
    FreeNavigationTrajectoryPrompt,
    VisualGoalTrajectoryPrompt,
)
from planner.application.factories import (
    build_completion_checker,
    build_free_navigation_planner,
    build_planner,
    build_task_parser,
    build_trace_factory,
)
from planner.config import load_config
from planner.domain.mission import TaskKind


def _config(tmp_path, data):
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return load_config(path)


def test_build_completion_checker_uses_configured_endpoint(tmp_path):
    config = _config(tmp_path, {
        "task_completion": {
            "url": "http://model/v1",
            "model": "qwen-test",
            "timeout_s": 7.0,
            "max_completion_tokens": 1024,
            "reasoning_effort": "low",
            "request_format": "deepseek",
        },
    })
    checker = build_completion_checker(config)
    assert isinstance(checker.prompt_builder, VisualGoalCompletionPrompt)
    assert checker.transport.url == "http://model/v1/chat/completions"
    assert checker.transport.model == "qwen-test"
    assert checker.transport.timeout_s == pytest.approx(7.0)
    assert checker.transport.max_completion_tokens == 1024
    assert checker.transport.reasoning_effort == "low"
    assert checker.transport.request_format == "deepseek"


def test_build_completion_checker_requires_an_endpoint(tmp_path):
    with pytest.raises(ValueError):
        build_completion_checker(_config(tmp_path, {}))


def test_build_vlm_planner_uses_visual_goal_prompt(tmp_path):
    config = _config(tmp_path, {
        "trajectory_planner": {
            "url": "http://model/v1",
            "model": "qwen-test",
            "timeout_s": 12.0,
            "max_tokens": 256,
        },
    })
    planner = build_planner(config, "vlm")
    assert isinstance(planner, QwenVLPlanner)
    assert isinstance(planner.prompt_builder, VisualGoalTrajectoryPrompt)
    assert planner.transport.url == "http://model/v1/chat/completions"
    assert planner.transport.max_completion_tokens == 256
    assert planner.timeout_s == pytest.approx(12.0)
    assert planner.expected_points == 5


def test_build_vlm_planner_uses_the_aerial_prompt_when_asked(tmp_path):
    config = _config(tmp_path, {
        "trajectory_planner": {
            "url": "http://model/v1",
            "model": "qwen-test",
        },
    })
    planner = build_planner(config, "vlm", aerial=True)
    assert isinstance(planner.prompt_builder, VisualGoalTrajectoryPrompt)
    assert planner.prompt_builder.system_prompt() == AERIAL_VISUAL_GOAL_SYSTEM_PROMPT


def test_build_vlm_planner_uses_the_configured_request_format(tmp_path):
    config = _config(tmp_path, {
        "trajectory_planner": {
            "url": "https://api.anthropic.com",
            "model": "claude-plan",
            "request_format": "anthropic",
            "reasoning_effort": "none",
        },
    })
    planner = build_planner(config, "vlm")
    assert planner.transport.url == "https://api.anthropic.com/v1/messages"
    assert planner.transport.request_format == "anthropic"


def test_build_task_parser_is_none_without_an_endpoint(tmp_path):
    assert build_task_parser(_config(tmp_path, {})) is None


def test_build_task_parser_uses_configured_endpoint(tmp_path, monkeypatch):
    monkeypatch.setenv("VLN_API_KEY", "secret")
    config = _config(tmp_path, {
        "task_parser": {
            "url": "http://model/v1",
            "model": "qwen-test",
            "timeout_s": 12.0,
            "max_completion_tokens": 256,
            "reasoning_effort": "low",
            "request_format": "deepseek",
        },
    })
    parser = build_task_parser(config)
    assert isinstance(parser, OpenAITaskParser)
    assert parser.url == "http://model/v1/chat/completions"
    assert parser.model == "qwen-test"
    assert parser.api_key == "secret"
    assert parser.timeout_s == pytest.approx(12.0)
    assert parser.max_completion_tokens == 256
    assert parser.reasoning_effort == "low"
    assert parser.request_format == "deepseek"
    assert {codec.kind for codec in parser.codecs} == {
        TaskKind.BASIC_MOTION,
        TaskKind.VISUAL_GOAL_NAVIGATION,
        TaskKind.FREE_NAVIGATION,
    }


def test_build_task_parser_filters_codecs_by_registered_kinds(tmp_path):
    config = _config(tmp_path, {
        "task_parser": {
            "url": "http://model/v1",
            "model": "qwen-test",
        },
    })
    parser = build_task_parser(config, kinds={TaskKind.BASIC_MOTION})
    assert {codec.kind for codec in parser.codecs} == {
        TaskKind.BASIC_MOTION,
    }
    assert "free_navigation" not in parser.system_prompt


def test_build_task_parser_uses_the_uav_prompt_for_the_drone(tmp_path):
    config = _config(tmp_path, {
        "task_parser": {
            "url": "http://model/v1",
            "model": "qwen-test",
        },
    })
    parser = build_task_parser(config, aerial=True)
    assert "UAV mission parser" in parser.system_prompt
    assert "The controlled entity is a car" not in parser.system_prompt


def test_build_free_navigation_planner_uses_the_road_following_prompt(
    tmp_path,
):
    config = _config(tmp_path, {
        "trajectory_planner": {
            "url": "http://model/v1",
            "model": "qwen-test",
            "timeout_s": 9.0,
        },
    })
    planner = build_free_navigation_planner(config)
    assert isinstance(planner, QwenVLPlanner)
    assert isinstance(planner.prompt_builder, FreeNavigationTrajectoryPrompt)
    assert planner.transport.url == "http://model/v1/chat/completions"
    assert planner.timeout_s == pytest.approx(9.0)


def test_build_free_navigation_planner_uses_the_aerial_prompt_when_asked(
    tmp_path,
):
    config = _config(tmp_path, {
        "trajectory_planner": {
            "url": "http://model/v1",
            "model": "qwen-test",
        },
    })
    planner = build_free_navigation_planner(config, aerial=True)
    assert isinstance(planner.prompt_builder, FreeNavigationTrajectoryPrompt)
    assert (
        planner.prompt_builder.system_prompt()
        == AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT
    )


def test_build_free_navigation_planner_is_none_without_an_endpoint(
    tmp_path,
):
    assert build_free_navigation_planner(_config(tmp_path, {})) is None


def test_build_task_parser_prefers_the_inline_api_key(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("VLN_API_KEY", "env-secret")
    config = _config(tmp_path, {
        "task_parser": {
            "url": "http://model/v1",
            "model": "qwen-test",
            "api_key": "inline-secret",
        },
    })
    assert build_task_parser(config).api_key == "inline-secret"


def test_build_task_parser_falls_back_to_the_api_key_env(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("CUSTOM_KEY", "env-secret")
    config = _config(tmp_path, {
        "task_parser": {
            "url": "http://model/v1",
            "model": "qwen-test",
            "api_key_env": "CUSTOM_KEY",
        },
    })
    assert build_task_parser(config).api_key == "env-secret"


def test_build_trace_factory_writes_task_configuration(tmp_path):
    traces = tmp_path / "traces"
    config = _config(tmp_path, {
        "visual_goal_navigation": {"output_root": str(traces)},
    })
    trace = build_trace_factory(config)()
    assert isinstance(trace, VisualGoalTrace)
    assert trace.path.parent == traces
    event = json.loads((trace.path / "events.jsonl").read_text().splitlines()[0])
    assert event["event"] == "runtime_configuration"
    assert event["waypoint_fields"] == ["dx", "dy", "dz", "dyaw_deg"]
    assert event["execution_mode"] == "stopped_segments"