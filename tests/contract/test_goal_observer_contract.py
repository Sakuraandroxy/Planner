import json
from unittest.mock import Mock

import pytest

from planner.adapters.goal_observer.response_parser import parse_goal_observation
from planner.adapters.goal_observer.vlm_goal_observer import VLMGoalObserver
from planner.domain.execution import ExecutionBudget
from planner.errors import BudgetExceeded, ProtocolError
from tests.fakes.visual_goal import wall_observation


def payload(**updates):
    return dict(protocol_version="goal_observation_v1", status="visible",
                bbox=[0.2, 0.2, 0.8, 0.8], surface_point=[0.5, 0.5],
                description="red house", evidence="red facade visible", same_target=True, **updates)


@pytest.mark.parametrize("update", [
    {"bbox": [0, 0, 300, 400]}, {"surface_point": [True, 0.5]},
    {"surface_point": [float("nan"), 0.5]}, {"surface_point": [0.9, 0.5]},
    {"same_target": "true"}, {"unknown": 1}, {"status": "not_visible"},
    {"protocol_version": "v2"}, {"bbox": [0.8, 0.2, 0.2, 0.8]},
])
def test_goal_response_strictly_rejects_invalid_contract(update):
    value = payload()
    value.update(update)
    with pytest.raises(ProtocolError):
        parse_goal_observation(json.dumps(value))


def test_duplicate_fields_rejected():
    with pytest.raises(ProtocolError):
        parse_goal_observation('{"status":"visible","status":"not_visible"}')


def test_goal_request_is_multimodal_and_late_result_is_discarded(monkeypatch):
    import planner.adapters.goal_observer.vlm_goal_observer as adapter
    now = [100.0]
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": json.dumps(payload())}}]}

    def post(*args, **kwargs):
        assert kwargs["json"]["messages"][0]["role"] == "system"
        assert kwargs["json"]["messages"][1]["content"][0]["type"] == "image_url"
        assert kwargs["timeout"] <= 1
        now[0] = 102.0
        return response

    monkeypatch.setattr(adapter.requests, "post", post)
    observer = VLMGoalObserver("http://example.test/v1", "vision", "no-key", 20, clock=lambda: now[0])
    with pytest.raises(BudgetExceeded):
        observer.observe(wall_observation(), "red house", budget=ExecutionBudget(101))
