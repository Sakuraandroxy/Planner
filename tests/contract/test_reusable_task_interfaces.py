"""Non-visual typed requests exercise the common model and window interfaces."""
from dataclasses import dataclass
import hashlib
from unittest.mock import Mock

import pytest

from planner.adapters.task_completion import OpenAICompatibleTaskCompletionChecker
from planner.adapters.task_completion.prompt import CompletionMessages
from planner.adapters.trajectory_planner.prompts import BaseTrajectoryPrompt
from planner.adapters.trajectory_planner.prompts.visual_goal_navigation import VisualGoalTrajectoryPrompt
from planner.adapters.trajectory_planner.qwen_vl_planner import QwenVLPlanner
from planner.domain.motion import MotionLimits
from planner.domain.planning_window import PlanningWindowInput
from planner.domain.pose import RelativePoseDelta
from planner.domain.sliding_window import SlidingWindowSettings
from planner.domain.trajectory import RelativeTrajectory
from planner.services.planning_window import PlanningWindow
from planner.services.trajectory_validation import TrajectoryValidator
from tests.contract.test_visual_goal_planning_prompt import context
from tests.fakes.visual_goal import Clock, wall_observation
from tests.unit.test_visual_goal_completion_client import request


@dataclass(frozen=True)
class CorridorContext:
    side: str


class CorridorPrompt:
    def system_prompt(self):
        return "test-only corridor planning"

    def build_user_prompt(self, instruction, observation, max_points, *, task_context, **execution):
        assert isinstance(task_context, CorridorContext)
        return instruction + ": " + task_context.side


def test_planner_accepts_another_typed_task_context_without_visual_import_branches(monkeypatch):
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": "[[1,0,0,0],[1,0,0,0],[1,0,0,0],[1,0,0,0],[1,0,0,0]]"}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr("requests.post", post)
    planner = QwenVLPlanner("https://offline.invalid/v1", "fake", "no-key", 10,
                            prompt_builder=CorridorPrompt())
    result = planner.plan(wall_observation(), "continue", task_context=CorridorContext("left"))
    assert len(result.points) == 5
    assert post.call_args.kwargs["json"]["messages"][1]["content"][-1]["text"] == "continue: left"


def test_shared_window_uses_typed_context_without_a_visual_session():
    clock = Clock()
    observation = wall_observation(clock=clock)
    contexts, calls = [], []
    class Planner:
        def plan(self, observation, instruction, *, task_context, **execution):
            assert not observation.depth_meters.flags.writeable
            contexts.append(task_context)
            return RelativeTrajectory(tuple(RelativePoseDelta(.2, 0, 0, 0) for _ in range(5)))
    window = PlanningWindow(Planner(), TrajectoryValidator(5, 30, 180),
        SlidingWindowSettings(enabled=True), MotionLimits(), clock)
    try:
        window.begin(deadline_monotonic=clock()+20, maximum_planning_rounds=2)
        world = window.next_world(observation, CorridorContext("left"), "continue",
            PlanningWindowInput(observation.vehicle_pose, 10, 5), lambda: calls.append(1), lambda: None)
        assert len(world.poses) == 5 and world.poses[-1].x == pytest.approx(1)
        assert contexts == [CorridorContext("left")] and calls == [1]
    finally:
        window.close()


@dataclass(frozen=True)
class InspectionEvidence:
    inspected: bool


class InspectionCompletionPrompt:
    def prepare(self, evidence: InspectionEvidence) -> CompletionMessages:
        assert isinstance(evidence, InspectionEvidence)
        return CompletionMessages("test-only inspection", str(evidence.inspected), ())


def test_common_completion_client_accepts_task_owned_request_and_prompt():
    client = OpenAICompatibleTaskCompletionChecker("https://offline.invalid/v1", "other-model",
        "no-key", 10, prompt_builder=InspectionCompletionPrompt())
    client.transport.request = Mock(return_value="false")
    assert client.check(InspectionEvidence(False), timeout_s=5) is False
    assert client.transport.request.call_args.args[:3] == ("test-only inspection", "False", ())


def test_base_wire_prompt_is_preserved_and_visual_extensions_use_radius():
    from planner.adapters.task_completion.prompts import VisualGoalCompletionPrompt
    digest = lambda value: hashlib.sha256(value.encode()).hexdigest()
    assert digest(BaseTrajectoryPrompt().build_user_prompt("approach red house", wall_observation(), 5)) == (
        "35b748bf0b3403a07ab2b988c4b22496d009b209f47f955c7d2bf8cbe9886203")
    visual = VisualGoalTrajectoryPrompt().build_user_prompt(
        "approach red house", wall_observation(), 5, task_context=context())
    assert visual.startswith(BaseTrajectoryPrompt().build_user_prompt("approach red house", wall_observation(), 5))
    assert '"arrival_radius_m":4' in visual and '"standoff_m"' not in visual
    completion = VisualGoalCompletionPrompt.build_user_prompt(request())
    assert "完成距离上限：4.0米" in completion and "±" not in completion
    assert "未额外限制相对起点高度" in completion


@pytest.mark.parametrize("age,timeout", [(0, 1), (1, 0), (float("nan"), 1), (1, float("inf"))])
def test_invalid_window_request_limits_are_rejected(age, timeout):
    with pytest.raises(ValueError):
        PlanningWindowInput(wall_observation().vehicle_pose, age, timeout)


@pytest.mark.parametrize("deadline,rounds", [(float("nan"), 2), (float("inf"), 2), (200, 0), (200, True)])
def test_window_begin_rejects_invalid_budget_before_creating_requests(deadline, rounds):
    clock = Clock()
    window = PlanningWindow(Mock(), TrajectoryValidator(5, 30, 180),
        SlidingWindowSettings(enabled=True), MotionLimits(), clock)
    try:
        with pytest.raises(ValueError):
            window.begin(deadline_monotonic=deadline, maximum_planning_rounds=rounds)
        assert not window.active and not window.worker.has_pending_call
    finally:
        window.close()
