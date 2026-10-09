import json
from unittest.mock import Mock

import pytest

from planner.adapters.goal_observer.response_parser import parse_goal_observation
from planner.adapters.goal_observer.vlm_goal_observer import VLMGoalObserver
from planner.domain.execution import ExecutionBudget
from planner.errors import BudgetExceeded, ProtocolError
from tests.fakes.visual_goal import wall_observation
from planner.capabilities.targeting.association import make_reference
from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.domain.visual_goal_limits import VisualGoalLimits
from tests.fakes.visual_goal import VISIBLE


def payload(**updates):
    value = dict(protocol_version="goal_observation_v2", status="visible",
                 bbox=[0.2, 0.2, 0.8, 0.8], surface_candidates=[candidate()],
                 description="red house", evidence="red facade visible", same_target=True)
    value.update(updates)
    return value


def candidate(**updates):
    value = dict(candidate_id="a", point=[0.5, 0.5], description="central solid wall",
                 matches_reference_region=False)
    value.update(updates)
    return value


@pytest.mark.parametrize("update", [
    {"bbox": [0, 0, 300, 400]}, {"surface_candidates": [candidate(point=[True, 0.5])]},
    {"surface_candidates": [candidate(point=[float("nan"), 0.5])]},
    {"surface_candidates": [candidate(point=[0.9, 0.5])]},
    {"same_target": "true"}, {"unknown": 1}, {"status": "not_visible"},
    {"protocol_version": "v2"}, {"bbox": [0.8, 0.2, 0.2, 0.8]},
    {"protocol_version": "goal_observation_v1"}, {"surface_candidates": []},
    {"surface_candidates": [candidate(), candidate()]},
    {"surface_candidates": [candidate(), candidate(candidate_id="b")]},
    {"surface_candidates": [candidate(description="")]},
    {"surface_candidates": [candidate(matches_reference_region="true")]},
    {"surface_candidates": [candidate(unknown=1)]},
    {"surface_candidates": [candidate(matches_reference_region=True),
                            candidate(candidate_id="b", point=[0.6, 0.5], matches_reference_region=True)]},
    {"surface_candidates": [candidate(candidate_id=str(i), point=[0.3+i*0.05, 0.5]) for i in range(6)]},
])
def test_goal_response_strictly_rejects_invalid_contract(update):
    value = payload()
    value.update(update)
    with pytest.raises(ProtocolError):
        parse_goal_observation(json.dumps(value))


def test_duplicate_fields_rejected():
    with pytest.raises(ProtocolError):
        parse_goal_observation('{"status":"visible","status":"not_visible"}')


def test_multi_candidate_contract_and_explicit_limit():
    value = payload(surface_candidates=[candidate(), candidate(candidate_id="b", point=[0.6, 0.5])])
    result = parse_goal_observation(json.dumps(value))
    assert len(result.surface_candidates) == 2
    with pytest.raises(ProtocolError):
        parse_goal_observation(json.dumps(value), maximum_candidates=1)
    result = parse_goal_observation(json.dumps(payload(status="not_visible", bbox=None, surface_candidates=[])))
    assert not result.surface_candidates


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


def test_reference_request_separates_object_and_region_in_one_call(monkeypatch):
    import planner.adapters.goal_observer.vlm_goal_observer as adapter
    obs = wall_observation()
    geometry = DepthGeometry(0.5, 200, True).estimate(obs, VISIBLE, VisualGoalLimits())
    reference = make_reference(obs, VISIBLE, geometry, "central facade")
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": json.dumps(payload())}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr(adapter.requests, "post", post)
    observer = VLMGoalObserver("http://example.test/v1", "vision", "no-key", 20, maximum_candidates=3)
    observer.observe(obs, "red house", reference)
    assert post.call_count == 1
    content = post.call_args.kwargs["json"]["messages"][1]["content"]
    assert [item["type"] for item in content] == ["image_url"]*3+["text"]
    text = json.loads(content[-1]["text"])
    assert text["maximum_surface_candidates"] == 3
    assert text["reference"]["selected_region_description"] == "central facade"
    assert text["reference"]["selected_world_anchor"] == list(reference.world_point)
    assert text["reference"]["selected_point_in_image_3"] == list(reference.region_point)


def test_region_match_without_reference_is_protocol_error(monkeypatch):
    import planner.adapters.goal_observer.vlm_goal_observer as adapter
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": json.dumps(payload(
        surface_candidates=[candidate(matches_reference_region=True)]))}}]}
    monkeypatch.setattr(adapter.requests, "post", Mock(return_value=response))
    with pytest.raises(ProtocolError, match="without a reference"):
        VLMGoalObserver("http://example.test/v1", "vision", "no-key", 20).observe(wall_observation(), "house")


def test_expired_image_encoding_does_not_send_a_request(monkeypatch):
    import planner.adapters.goal_observer.vlm_goal_observer as adapter
    now = [100.0]
    def slow_encoding(image):
        now[0] = 102.0
        return "data:image/png;base64,unused"
    post = Mock()
    monkeypatch.setattr(adapter, "image_url", slow_encoding)
    monkeypatch.setattr(adapter.requests, "post", post)
    observer = VLMGoalObserver("http://example.test/v1", "vision", "no-key", 20, clock=lambda: now[0])
    with pytest.raises(BudgetExceeded, match="encoding"):
        observer.observe(wall_observation(), "house", budget=ExecutionBudget(101))
    post.assert_not_called()
