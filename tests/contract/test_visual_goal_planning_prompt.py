import json
import math
from dataclasses import replace
from unittest.mock import Mock

import pytest

from planner.adapters.trajectory_planner.prompts import BaseTrajectoryPrompt, BASE_TRAJECTORY_SYSTEM_PROMPT
from planner.adapters.trajectory_planner.prompts.visual_goal_navigation import VisualGoalTrajectoryPrompt
from planner.adapters.trajectory_planner.qwen_vl_planner import QwenVLPlanner
from planner.domain.execution import ExecutionBudget
from planner.domain.pose import RelativePoseDelta
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.visual_goal import GoalGeometry, VisualGoalPlanningContext, VisualGoalState
from planner.errors import BudgetExceeded, ProtocolError
from tests.fakes.visual_goal import wall_observation, VISIBLE, Clock


def context():
    return VisualGoalPlanningContext('red house', VISIBLE, GoalGeometry(True, 'surface', 6, (6, 0, -4)), VisualGoalState.APPROACHING, 4, 0.3, None)


def test_task_prompt_inherits_weather_and_coordinate_rules_and_adds_typed_evidence():
    builder = VisualGoalTrajectoryPrompt()
    assert builder.system_prompt().startswith(BASE_TRAJECTORY_SYSTEM_PROMPT)
    system = " ".join(builder.system_prompt().split())
    assert "If no pending queue is supplied, treat it as empty" in system
    assert "forward motion is not mandatory" in system
    assert "rather than preserving its initial bearing" in system
    assert "executor follows valid trajectory points in order" in system
    assert "Local checks determine the executable prefix" not in system
    user = builder.build_user_prompt("approach red house", wall_observation(), 5, task_context=context())
    assert "Planning round: 1" in user and '"arrival_radius_m":4' in user
    assert '"maximum_motion_s"' not in user and '"target_relative_to_observation_body_m":[6.0,0.0,0]' in user
    assert '"world_point"' not in user and '"bbox"' not in user
    assert "Pending incremental pose waypoints: []" in user
    assert "Output exactly 5 NEW" in user


@pytest.mark.parametrize("keep_in_view", [False, True])
@pytest.mark.parametrize("has_pending", [False, True])
def test_request_ends_with_complete_format_example_after_all_task_context(
        monkeypatch, keep_in_view, has_pending):
    pending = RelativeTrajectory((RelativePoseDelta(1, 0, 0, 0),)) if has_pending else None
    goal_context = replace(context(), keep_target_in_view=keep_in_view)
    builder = VisualGoalTrajectoryPrompt()
    example = (
        "[[0.5,0.1,-0.2,5.0],[0.6,0.1,-0.1,4.0],[0.5,0.0,0.0,2.0],"
        "[0.3,-0.1,0.0,-1.0],[0.1,0.0,0.0,0.0]]"
    )
    post = Mock(return_value=Mock(json=Mock(return_value={
        "choices": [{"message": {"content": example}, "finish_reason": "stop"}],
    })))
    monkeypatch.setattr("planner.adapters.trajectory_planner.qwen_vl_planner.requests.post", post)
    planner = QwenVLPlanner("http://offline/v1", "offline-model", "no-key", 20,
                            thinking="disabled", prompt_builder=builder)
    result = planner.plan(wall_observation(), "approach red house",
                          task_context=goal_context, pending_trajectory=pending)
    body = post.call_args.kwargs["json"]
    assert body["max_tokens"] == 256
    assert body["thinking"] == {"type": "disabled"}
    system, user_message = body["messages"]
    assert system["role"] == "system" and system["content"] == builder.system_prompt()
    assert "ONE outer array" in system["content"]
    assert "Do not output scene descriptions, calculations, reasoning" in system["content"]
    assert user_message["role"] == "user"
    assert [part["type"] for part in user_message["content"]] == ["image_url", "image_url", "text"]
    user = user_message["content"][-1]["text"]
    assert user.index("Visual goal context") < user.index("Final response:")
    assert ("Visual goal viewing guidance:" in user) is keep_in_view
    if keep_in_view:
        assert user.index("Visual goal viewing guidance:") < user.index("Final response:")
    assert '"new_actions_reference":"' + ("pending_tail" if has_pending else "observation") + '"' in user
    footer = user.split("Final response:", 1)[1]
    assert "Do not include pending waypoints" in footer
    assert "example demonstrates the output format only" in footer
    assert "```" not in footer
    assert user.endswith(example)
    # The example delivered to the model is valid under the actual five-point parser.
    parsed_example = json.loads(user.rsplit("\n", 1)[1])
    assert len(parsed_example) == len(result.points) == 5
    assert all(len(row) == 4 and all(type(v) in (int, float) and math.isfinite(v) for v in row)
               for row in parsed_example)
    assert result.points[0] == RelativePoseDelta(*parsed_example[0])


def test_legacy_prompt_rejects_new_context_instead_of_dropping_it():
    planner = QwenVLPlanner("http://invalid/v1", "fake", "no-key", 20,
                            prompt_builder=BaseTrajectoryPrompt())
    with pytest.raises(ProtocolError, match="context"):
        planner.plan(wall_observation(), "approach", task_context=context())


def test_late_trajectory_reply_is_not_used_to_fly(monkeypatch):
    clock = Clock()
    monkeypatch.setattr("planner.adapters.trajectory_planner.qwen_vl_planner.time.monotonic", clock)
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": "[[1,0,0,0]]"}}]}
    def post(*args, **kwargs):
        assert kwargs["timeout"] <= 1
        assert kwargs["json"]["messages"][0]["role"] == "system"
        assert kwargs["json"]["messages"][0]["content"] == VisualGoalTrajectoryPrompt().system_prompt()
        assert len(kwargs["json"]["messages"][1]["content"]) == 3
        clock.sleep(2)
        return response
    monkeypatch.setattr("planner.adapters.trajectory_planner.qwen_vl_planner.requests.post", post)
    planner = QwenVLPlanner("http://invalid/v1", "fake", "no-key", 20,
                            prompt_builder=VisualGoalTrajectoryPrompt())
    with pytest.raises(BudgetExceeded, match="late"):
        planner.plan(wall_observation(clock=clock), "approach", task_context=context(),
                     budget=ExecutionBudget(clock()+1))
