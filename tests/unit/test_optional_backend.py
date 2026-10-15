from tests.fakes.configuration import alternate_backend_config_data, write_config
import json
from unittest.mock import Mock

import pytest

from config import load_config
from planner.adapters.trajectory_planner.qwen_vl_planner import QwenVLPlanner
from planner.adapters.trajectory_planner.response_parser import parse_trajectory
from planner.errors import ProtocolError
from planner.services.trajectory_validation import TrajectoryValidator
from tests.fakes.visual_goal import wall_observation


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
    response.json.return_value = {"choices": [{"message": {"content": json.dumps([[0,0,-2.4,0]] * 5)}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr(adapter.requests, "post", post)
    prompt_builder = Mock()
    prompt_builder.system_prompt.return_value = "system"
    prompt_builder.build_user_prompt.return_value = "user"
    planner = QwenVLPlanner("https://example.com", "test", "test-key", 120,
                            thinking=thinking, prompt_builder=prompt_builder)
    assert len(planner.plan(wall_observation(), "ascend 12m").points) == 5
    payload = post.call_args.kwargs["json"]
    assert payload["messages"][0] == {"role": "system", "content": "system"}
    assert payload["messages"][1]["role"] == "user"
    assert len(payload["messages"][1]["content"]) == 3
    prompt_builder.system_prompt.assert_called_once_with()
    prompt_builder.build_user_prompt.assert_called_once()
    assert prompt_builder.build_user_prompt.call_args.kwargs["pending_trajectory"] is None
    if thinking:
        assert payload["thinking"] == {"type": "disabled"}
    else:
        assert "thinking" not in payload


def test_deepseek_config(tmp_path, monkeypatch):
    monkeypatch.setenv("DEEPSEEK_API_KEY", "test-key")
    config = load_config(write_config(tmp_path / "backend.yaml", alternate_backend_config_data()))
    assert config.trajectory_planner.thinking == "disabled"
    assert config.trajectory_planner.model == "deepseek-flash"
    from tests.fakes.configuration import main_config_data
    assert load_config("config/base.yaml").trajectory_planner.model == main_config_data()["trajectory_planner"]["model"]


@pytest.mark.parametrize("count", [1,3,6])
def test_config_requires_five_model_points(tmp_path, monkeypatch, count):
    monkeypatch.setenv("DEEPSEEK_API_KEY", "test-key")
    raw = alternate_backend_config_data()
    raw["trajectory"]["point_count"] = count
    path = write_config(tmp_path / "invalid-point-count.yaml", raw)

    from planner.errors import ConfigurationError

    with pytest.raises(ConfigurationError, match="exactly 5"):
        load_config(path)


@pytest.mark.parametrize("count", [1,3,4])
def test_model_rejects_short_reply(monkeypatch, count):
    import planner.adapters.trajectory_planner.qwen_vl_planner as adapter
    monkeypatch.setattr(adapter, "_image_url", lambda *a: "image")
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": json.dumps([[1,0,0,0]] * count)}}]}
    monkeypatch.setattr(adapter.requests, "post", Mock(return_value=response))
    builder = Mock()
    planner = QwenVLPlanner("http://invalid/v1", "fake", "no-key", 20, prompt_builder=builder)
    with pytest.raises(ProtocolError, match="exactly 5"):
        planner.plan(wall_observation(), "follow the road")


def test_oversized_pending_is_rejected_before_request(monkeypatch):
    import planner.adapters.trajectory_planner.qwen_vl_planner as adapter
    from planner.domain.trajectory import RelativeTrajectory
    from planner.domain.pose import RelativePoseDelta
    post = Mock()
    monkeypatch.setattr(adapter.requests, "post", post)
    planner = QwenVLPlanner("http://invalid/v1", "fake", "no-key", 20)
    pending = RelativeTrajectory((RelativePoseDelta(1,0,0,15),)*6)
    with pytest.raises(ProtocolError, match="0..5"):
        planner.plan(Mock(), "approach", pending_trajectory=pending)
    post.assert_not_called()


def test_explicit_empty_pending_is_serialized_and_accepted(monkeypatch):
    import planner.adapters.trajectory_planner.qwen_vl_planner as adapter
    from planner.domain.trajectory import RelativeTrajectory
    from tests.fakes.visual_goal import wall_observation
    monkeypatch.setattr(adapter, "_image_url", lambda *a: "image")
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": json.dumps([[1,0,0,0]]*5)}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr(adapter.requests, "post", post)
    planner = QwenVLPlanner("http://invalid/v1", "fake", "no-key", 20)
    result = planner.plan(wall_observation(), "follow the road", pending_trajectory=RelativeTrajectory(()))
    assert len(result.points) == 5
    text = post.call_args.kwargs["json"]["messages"][1]["content"][-1]["text"]
    assert "Pending incremental pose waypoints: []" in text
    assert "Output exactly 5 NEW" in text


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
