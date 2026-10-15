from dataclasses import replace
from unittest.mock import Mock

import pytest

from planner.adapters.task_completion import OpenAICompatibleTaskCompletionChecker
from planner.adapters.task_completion.prompts import VisualGoalCompletionPrompt
from planner.domain.visual_goal_completion import VisualGoalCompletionInput, VisualGoalTriggerRules
from planner.domain.visual_goal import GoalGeometry, VisualGoalNavigationParameters
from planner.errors import ProtocolError
from tests.fakes.visual_goal import wall_observation, VISIBLE


def request():
    observation = wall_observation(3.5)
    return VisualGoalCompletionInput('stage_1', VisualGoalNavigationParameters('Approach the first car from left to right. 飞到从左往右第一辆车旁。', 'first car from left to right'), observation, observation.vehicle_pose, VISIBLE, GoalGeometry(True, 'verified', 3.5, (3.5, 0, -4)), VisualGoalTriggerRules(), 0.3, None, 2, 2, 'bound car')


@pytest.mark.parametrize("raw,expected", [("true", True), ("false", False), (" true\n", True)])
def test_boolean_protocol_and_separate_multimodal_payload(monkeypatch, raw, expected):
    response = Mock()
    response.json.return_value = {"choices": [{"finish_reason": "stop", "message": {"content": raw}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr("planner.adapters.multimodal_chat.requests.post", post)
    client = OpenAICompatibleTaskCompletionChecker("https://worldcodes.online/v1", "gpt-5.6-luna", "test-key", 20, prompt_builder=VisualGoalCompletionPrompt())
    assert client.check(request(), timeout_s=10) is expected
    body = post.call_args.kwargs["json"]
    assert post.call_args.args[0] == "https://worldcodes.online/v1/chat/completions"
    assert body["model"] == "gpt-5.6-luna"
    assert body["reasoning_effort"] == "none" and body["max_completion_tokens"] == 2048
    content = body["messages"][1]["content"]
    assert [item["type"] for item in content] == ["image_url", "image_url", "text"]
    assert content[0]["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert "stage_1" in content[2]["text"] and "bound car" in content[2]["text"]
    assert "飞到从左往右第一辆车旁" in content[2]["text"]
    assert "不得按当前画面重新排序" in content[2]["text"]


@pytest.mark.parametrize("raw", ["True", "1", '{"completed":true}', "```true```", "true because nearby", ""])
def test_non_boolean_responses_rejected(raw):
    client = OpenAICompatibleTaskCompletionChecker("https://worldcodes.online/v1", "model", "test-key", 20, prompt_builder=VisualGoalCompletionPrompt())
    client.transport.request = Mock(return_value=raw)
    with pytest.raises(ProtocolError):
        client.check(request(), timeout_s=10)


def test_unconfirmed_observation_never_requests_model():
    client = OpenAICompatibleTaskCompletionChecker("https://worldcodes.online/v1", "model", "test-key", 20, prompt_builder=VisualGoalCompletionPrompt())
    client.transport.request = Mock()
    pending = replace(request(), confirmation_count=1)
    with pytest.raises(ProtocolError):
        client.check(pending, timeout_s=10)
    client.transport.request.assert_not_called()
