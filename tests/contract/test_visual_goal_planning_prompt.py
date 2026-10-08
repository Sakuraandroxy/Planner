from unittest.mock import Mock

import pytest

from planner.adapters.trajectory_planner.prompts import NavigationTrajectoryPrompt, BASE_TRAJECTORY_SYSTEM_PROMPT
from planner.adapters.trajectory_planner.prompts.visual_goal_navigation import VisualGoalTrajectoryPrompt
from planner.adapters.trajectory_planner.qwen_vl_planner import QwenVLPlanner
from planner.domain.execution import ExecutionBudget
from planner.domain.visual_goal import GoalGeometry, VisualGoalPlanningContext, VisualGoalState
from planner.errors import BudgetExceeded, ProtocolError
from tests.fakes.visual_goal import wall_observation, VISIBLE, Clock


def context():
    return VisualGoalPlanningContext("red house", VISIBLE,
        GoalGeometry(True, "surface", 6, (6, 0, -4)), VisualGoalState.APPROACHING,
        4, 1, 6, 8, 1, 30, 1.8)


def test_task_prompt_inherits_weather_and_coordinate_rules_and_adds_typed_evidence():
    builder = VisualGoalTrajectoryPrompt()
    assert builder.system_prompt().startswith(BASE_TRAJECTORY_SYSTEM_PROMPT)
    user = builder.build_user_prompt("approach red house", wall_observation(), 5, visual_goal_context=context())
    assert "Planning round: 1" in user and '"standoff_m":4' in user
    assert '"maximum_motion_s":8' in user and '"world_point":[6,0,-4]' in user


def test_legacy_prompt_rejects_new_context_instead_of_dropping_it():
    planner = QwenVLPlanner("http://invalid/v1", "fake", "no-key", 20,
                            prompt_builder=NavigationTrajectoryPrompt())
    with pytest.raises(ProtocolError, match="context"):
        planner.plan(wall_observation(), "approach", visual_goal_context=context())


def test_late_trajectory_reply_is_not_used_to_fly(monkeypatch):
    clock = Clock()
    monkeypatch.setattr("planner.adapters.trajectory_planner.qwen_vl_planner.time.monotonic", clock)
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": "[[1,0,0,0]]"}}]}
    def post(*args, **kwargs):
        assert kwargs["timeout"] <= 1
        assert kwargs["json"]["messages"][0]["role"] == "system"
        assert len(kwargs["json"]["messages"][1]["content"]) == 3
        clock.sleep(2)
        return response
    monkeypatch.setattr("planner.adapters.trajectory_planner.qwen_vl_planner.requests.post", post)
    planner = QwenVLPlanner("http://invalid/v1", "fake", "no-key", 20,
                            prompt_builder=VisualGoalTrajectoryPrompt())
    with pytest.raises(BudgetExceeded, match="late"):
        planner.plan(wall_observation(clock=clock), "approach", visual_goal_context=context(),
                     budget=ExecutionBudget(clock()+1))
