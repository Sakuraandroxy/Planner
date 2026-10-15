import yaml
from tests.fakes.configuration import main_config_data, write_config
import json
from dataclasses import replace
from unittest.mock import Mock

import pytest

from config import load_config
from planner.adapters.task_parser import OpenAITaskParser
from planner.adapters.trajectory_planner.qwen_vl_planner import QwenVLPlanner
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.errors import ConfigurationError
from tests.fakes.visual_goal import wall_observation


@pytest.mark.parametrize("setting", ["enabled: 'false'", "refill_pending_count: 0",
    "refill_pending_count: 6", "latency_margin_s: .nan", "max_task_duration_s: 0",
    "max_observation_age_s: .inf", "max_consecutive_plan_failures: true", "unknown: 1"])
def test_migrated_settings_fail_closed(tmp_path, setting):
    raw = main_config_data()
    raw["sliding_window"] = yaml.safe_load(setting)
    path = write_config(tmp_path / "invalid.yaml", raw)
    with pytest.raises(ConfigurationError):
        load_config(path)


def test_single_main_config_supports_explicit_serial_mode(tmp_path):
    assert load_config("config/base.yaml").sliding_window.enabled
    raw = main_config_data()
    raw["sliding_window"]["enabled"] = False
    configured = load_config(write_config(tmp_path / "window.yaml", raw))
    assert not configured.sliding_window.enabled and configured.visual_goal.enabled


def test_actual_trajectory_request_preserves_bilingual_task_pending_and_history_anchors(monkeypatch):
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": json.dumps([[.5,0,0,0]]*5)}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr("requests.post", post)
    planner = QwenVLPlanner("http://offline/v1", "fake", "no-key", 20)
    instruction = "Approach the first car from left to right. 接近从左往右第一辆车。"
    pending = RelativeTrajectory((RelativePoseDelta(1,0,0,15),)*3)
    history = RelativeTrajectory((RelativePoseDelta(1,0,0,0),)*2)
    result = planner.plan(wall_observation(), instruction, pending_trajectory=pending,
        previous_trajectory=history, previous_trajectory_start=WorldPose(-2,0,-4,0), timeout_s=2)
    assert len(result.points) == 5 and post.call_args.kwargs["timeout"] == 2
    messages = post.call_args.kwargs["json"]["messages"]
    assert [message["role"] for message in messages] == ["system", "user"]
    text = messages[1]["content"][-1]["text"]
    assert instruction in text and "Current guidance:\nNot supplied." in text
    assert "Pending incremental pose waypoints: [[1,0,0,15],[1,0,0,15],[1,0,0,15]]" in text
    assert "Execution-history reference start pose: [-2,0,-4,0]" in text
    assert "Output exactly 5 NEW" in text


def test_parser_requests_and_preserves_equivalent_bilingual_stage_instruction(monkeypatch):
    instruction = "Ascend 2 meters. 上升2米。"
    response = Mock()
    response.json.return_value = {"choices": [{"finish_reason": "stop", "message": {"content": json.dumps({
        "protocol_version":"mission_plan_v1", "stages":[{"stage_id":"one","kind":"basic_motion",
        "parameters":{"instruction":instruction,"forward_m":0,"right_m":0,"up_m":2,"yaw_right_deg":0}}]})}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr("requests.post", post)
    result = OpenAITaskParser("http://offline/v1", "fake", "no-key", 10).parse("上升2米")
    assert result.stages[0].parameters.instruction == instruction
    messages = post.call_args.kwargs["json"]["messages"]
    assert "equivalent Simplified Chinese translation" in messages[0]["content"]
    assert "a single space" in messages[1]["content"]
