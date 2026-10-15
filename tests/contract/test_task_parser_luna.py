"""Task-parser migration contracts; no real API or simulator is contacted."""
from tests.fakes.configuration import visual_config_data, write_config
import json
from unittest.mock import Mock

import pytest

from config import load_config
from config.loader import _task_parser_config
from planner.adapters.task_parser.openai_task_parser import OpenAITaskParser
from planner.bootstrap import build_runtime
from planner.domain.mission import TaskKind
from planner.errors import ConfigurationError, ProtocolError


def mission():
    return {"protocol_version": "mission_plan_v1", "stages": [{
        "stage_id": "stage_1", "kind": "basic_motion", "parameters": {
            "instruction": "Ascend 12 meters", "forward_m": 0, "right_m": 0,
            "up_m": 12, "yaw_right_deg": 0}}]}


def test_deepseek_request_preserves_mission_protocol_and_uses_own_credentials(monkeypatch):
    monkeypatch.setenv("TASK_PARSER_API_KEY", "parser-test-key")
    monkeypatch.setenv("TASK_COMPLETION_API_KEY", "completion-test-key")
    config = load_config("config/base.yaml")
    response = Mock()
    response.json.return_value = {"choices": [{"finish_reason": "stop", "message": {"content": json.dumps(mission())}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr("planner.adapters.task_parser.openai_task_parser.requests.post", post)
    backend = config.task_parser
    parser = OpenAITaskParser(backend.url, backend.model, backend.api_key, backend.timeout_s,
        max_completion_tokens=backend.max_completion_tokens, reasoning_effort=backend.reasoning_effort,
        request_format=backend.request_format)
    result = parser.parse("上升12米")
    assert result.protocol_version == "mission_plan_v1" and result.stages[0].parameters.up_m == 12
    assert post.call_args.args[0] == "https://api.deepseek.com/chat/completions"
    assert post.call_args.kwargs["headers"]["Authorization"] == "Bearer parser-test-key"
    payload = post.call_args.kwargs["json"]
    assert payload["model"] == "deepseek-flash"
    assert payload["max_tokens"] == 2048 and payload["thinking"] == {"type": "disabled"}
    assert "max_completion_tokens" not in payload and "reasoning_effort" not in payload
    assert payload["messages"][0] == {"role": "system", "content": parser.system_prompt}
    assert payload["messages"][1]["role"] == "user" and "上升12米" in payload["messages"][1]["content"]
    assert config.task_completion.api_key == "completion-test-key"


def test_runtime_injects_separate_parser_and_completion_backends(tmp_path, monkeypatch):
    raw = visual_config_data()
    raw['visual_goal_navigation'].update(enabled=True, geometry_verified=True)
    raw["target_detection"]["api_key"] = "detector-test-key"
    raw["task_parser"].update(url="https://parser.invalid/v1", model="parser-model", max_completion_tokens=321)
    raw["task_completion"].update(url="https://completion.invalid/v1", model="completion-model", max_completion_tokens=456)
    monkeypatch.setenv("TASK_PARSER_API_KEY", "parser-test-key")
    monkeypatch.setenv("TASK_COMPLETION_API_KEY", "completion-test-key")
    path = tmp_path/"independent.yaml"
    write_config(path, raw)
    monkeypatch.setattr("planner.bootstrap.AirSimConnection", Mock())
    runtime = build_runtime(load_config(path))
    parser = runtime.application.task_parser
    deferred = runtime.application.runner.router._workflows[TaskKind.VISUAL_GOAL_NAVIGATION]
    deferred.prepare()
    workflow = deferred.instance
    checker = workflow.verification.checker.transport
    assert parser.url == "https://parser.invalid/v1/chat/completions" and parser.model == "parser-model"
    assert checker.url == "https://completion.invalid/v1/chat/completions" and checker.model == "completion-model"
    assert parser.api_key == "parser-test-key" and checker.api_key == "completion-test-key"
    assert parser.max_completion_tokens == 321 and checker.max_completion_tokens == 456
    assert parser.reasoning_effort == "none" and checker.reasoning_effort == "none"


def test_parser_key_never_falls_back_to_completion_key(tmp_path, monkeypatch):
    monkeypatch.delenv("TASK_PARSER_API_KEY", raising=False)
    monkeypatch.setenv("TASK_COMPLETION_API_KEY", "completion-test-key")
    config = load_config(write_config(tmp_path / "independent-keys.yaml", visual_config_data()))
    assert config.task_parser.api_key == "no-key"
    assert config.task_completion.api_key == "completion-test-key"


def test_old_parser_config_keeps_legacy_request_options(monkeypatch):
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": json.dumps(mission())}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr("planner.adapters.task_parser.openai_task_parser.requests.post", post)
    parser = OpenAITaskParser("http://local.invalid/v1", "local-model", "no-key", 20)
    assert parser.parse("上升12米").stages
    payload = post.call_args.kwargs["json"]
    assert payload["max_tokens"] == 1024 and payload["temperature"] == 0
    assert "max_completion_tokens" not in payload and "reasoning_effort" not in payload


@pytest.mark.parametrize("options", [{"max_completion_tokens": 0}, {"max_completion_tokens": True},
    {"max_completion_tokens": "2048"}, {"reasoning_effort": "disabled"}, {"reasoning_effort": []}])
def test_invalid_parser_request_settings_rejected(options):
    raw = {"url": "https://parser.invalid/v1", "model": "model", "timeout_s": 20, **options}
    with pytest.raises(ConfigurationError):
        _task_parser_config(raw)


@pytest.mark.parametrize("choice", [
    {"finish_reason": "length", "message": {"content": json.dumps(mission())}},
    {"finish_reason": "stop", "message": {"content": json.dumps(mission()), "refusal": "refused"}},
    {"message": {"content": None}},
])
def test_truncated_refused_and_missing_text_never_dispatch_a_mission(monkeypatch, choice):
    response = Mock()
    response.json.return_value = {"choices": [choice]}
    monkeypatch.setattr("planner.adapters.task_parser.openai_task_parser.requests.post", Mock(return_value=response))
    parser = OpenAITaskParser("https://parser.invalid/v1", "model", "test-key", 20,
                            max_completion_tokens=2048, reasoning_effort="none")
    with pytest.raises(ProtocolError):
        parser.parse("上升12米")
