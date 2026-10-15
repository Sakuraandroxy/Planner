"""Official DeepSeek payloads and explicit per-role protocol selection, offline only."""
import json
from unittest.mock import Mock

import pytest

from config import load_config
from planner.bootstrap import build_runtime
from planner.domain.mission import TaskKind
from planner.domain.target_detection import TargetDetectionInput
from planner.errors import ConfigurationError
from tests.fakes.configuration import visual_config_data, write_config
from tests.fakes.visual_goal import wall_observation
from tests.fakes.target_detection import detection_json
from tests.unit.test_visual_goal_completion_client import request as completion_input


@pytest.mark.parametrize("formats", [
    ("deepseek", "deepseek", "deepseek"), ("openai", "deepseek", "openai"),
])
def test_runtime_roles_send_correct_independent_payloads(tmp_path, monkeypatch, formats):
    roles = ("task_parser", "target_detection", "task_completion")
    raw = visual_config_data()
    raw['visual_goal_navigation'].update(geometry_verified=True)
    for role, dialect in zip(roles, formats):
        monkeypatch.delenv(raw[role]["api_key_env"], raising=False)
        raw[role].update(api_key="offline-"+role, request_format=dialect)
    monkeypatch.setattr("planner.bootstrap.AirSimConnection", Mock())
    mission = {"protocol_version":"mission_plan_v1", "stages":[{
        "stage_id":"one", "kind":"basic_motion", "parameters":{
            "instruction":"Ascend 2 meters. 上升2米。", "forward_m":0,"right_m":0,"up_m":2,"yaw_right_deg":0}}]}
    responses = []
    for content in (json.dumps(mission), detection_json(((.1,.2),(.7,.8))), "true"):
        response = Mock()
        response.json.return_value = {"choices":[{"finish_reason":"stop", "message":{"content":content}}]}
        responses.append(response)
    post = Mock(side_effect=responses)
    monkeypatch.setattr("requests.post", post)
    runtime = build_runtime(load_config(write_config(tmp_path/"roles.yaml",raw)))
    try:
        runtime.application.task_parser.parse("上升2米")
        deferred = runtime.application.runner.router._workflows[TaskKind.VISUAL_GOAL_NAVIGATION]
        deferred.prepare()
        workflow = deferred.instance
        detection = workflow.goal_observer.detector.detect(
            TargetDetectionInput(wall_observation(),"Find the car. 找到汽车。","car"),timeout_s=10)
        assert detection.found
        assert workflow.verification.checker.check(completion_input(),timeout_s=10)
        assert post.call_count == 3
        for call, role, dialect, limit in zip(post.call_args_list,roles,formats,(2048,64,2048)):
            body = call.kwargs["json"]
            assert call.args[0] == "https://api.deepseek.com/chat/completions"
            assert call.kwargs["headers"]["Authorization"] == "Bearer offline-"+role
            assert body["model"] == "deepseek-flash"
            if dialect == "deepseek":
                assert body["max_tokens"] == limit and body["thinking"] == {"type":"disabled"}
                assert "max_completion_tokens" not in body and "reasoning_effort" not in body
            else:
                assert body["max_completion_tokens"] == limit and body["reasoning_effort"] == "none"
                assert "max_tokens" not in body and "thinking" not in body
        assert len(post.call_args_list[1].kwargs["json"]["messages"][1]["content"]) == 2
        assert len(post.call_args_list[2].kwargs["json"]["messages"][1]["content"]) == 3
        runtime.connection.prepare_vehicle.assert_not_called()
        runtime.connection.takeoff.assert_not_called()
    finally:
        runtime.close()


@pytest.mark.parametrize("role",["task_parser","target_detection","task_completion"])
@pytest.mark.parametrize("dialect",["auto","deepseek-flash",True,None,{}])
def test_unknown_request_format_is_rejected_before_any_network(tmp_path,role,dialect):
    raw=visual_config_data()
    raw[role]["request_format"]=dialect
    with pytest.raises(ConfigurationError, match="request_format"):
        load_config(write_config(tmp_path/"invalid.yaml",raw))
