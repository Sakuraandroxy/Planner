import json
from unittest.mock import Mock

import pytest

from config import load_config
from planner.adapters.trajectory_planner.qwen_vl_planner import QwenVLPlanner
from planner.adapters.trajectory_planner.response_parser import parse_trajectory
from planner.errors import ProtocolError
from planner.services.trajectory_validation import TrajectoryValidator


@pytest.mark.parametrize("count", [1, 3, 4, 5])
def test_variable_points(count):
    trajectory = parse_trajectory(json.dumps([[0, 0, -2, 0]] * count))
    TrajectoryValidator(5, 30, 90).validate_relative(trajectory)


def test_excess_points_rejected():
    with pytest.raises(ProtocolError):
        parse_trajectory(json.dumps([[0, 0, -2, 0]] * 6))


@pytest.mark.parametrize("thinking", [None, "disabled"])
def test_backend_request(monkeypatch, thinking):
    import planner.adapters.trajectory_planner.qwen_vl_planner as adapter
    monkeypatch.setattr(adapter, "_image_url", lambda *args: "data:image/png;base64,test")
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": "[[0,0,-12,0]]"}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr(adapter.requests, "post", post)
    prompt_builder = Mock()
    prompt_builder.build.return_value = "test"
    planner = QwenVLPlanner("https://example.com", "test", "test-key", 120,
                            thinking=thinking, prompt_builder=prompt_builder)
    assert len(planner.plan(Mock(), "ascend 12m").points) == 1
    payload = post.call_args.kwargs["json"]
    assert len(payload["messages"][0]["content"]) == 3
    prompt_builder.build.assert_called_once()
    if thinking:
        assert payload["thinking"] == {"type": "disabled"}
    else:
        assert "thinking" not in payload


def test_deepseek_config(monkeypatch):
    monkeypatch.setenv("DEEPSEEK_API_KEY", "test-key")
    config = load_config("config/deepseek.example.yaml")
    assert config.trajectory_planner.thinking == "disabled"
    assert config.trajectory_planner.model == "deepseek-flash"
    assert "checkpoint_700" in load_config("config/base.yaml").trajectory_planner.model


def test_inline_key_fallback(monkeypatch):
    from config.loader import _api_config
    from planner.errors import ConfigurationError

    monkeypatch.delenv("DEEPSEEK_API_KEY", raising=False)
    raw = dict(url="https://api.deepseek.com", model="deepseek-flash", timeout_s=120,
               api_key_env="DEEPSEEK_API_KEY", api_key="test-inline-key", require_api_key=True)
    assert _api_config(raw).api_key == "test-inline-key"
    raw["api_key"] = ""
    with pytest.raises(ConfigurationError):
        _api_config(raw)
