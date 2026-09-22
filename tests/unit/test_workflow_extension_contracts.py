from unittest.mock import Mock

from planner.domain.progress import ExitStatus, ProgressDecision, ProgressStatus
from planner.workflows.termination import NavigationExitContext, NavigationExitPolicy


def observation(x=0, yaw=0):
    value = Mock()
    value.vehicle_pose.x = x
    value.vehicle_pose.y = 0
    value.vehicle_pose.z = 0
    value.vehicle_pose.yaw_deg = yaw
    return value


def test_navigation_exit_policy_has_task_lifecycle_and_explainable_limit():
    policy = NavigationExitPolicy(max_rounds=2)
    policy.begin()
    progress = ProgressDecision(ProgressStatus.CONTINUE, "keep moving", "forward")
    first = policy.evaluate(NavigationExitContext(1, observation(0), observation(1), progress))
    final = policy.evaluate(NavigationExitContext(2, observation(1), observation(2), progress))
    assert first.status is ExitStatus.CONTINUE
    assert final.status is ExitStatus.LIMIT_REACHED
    policy.begin()
    assert policy.evaluate(NavigationExitContext(1, observation(0), observation(0), progress)).status is ExitStatus.CONTINUE


def test_complete_and_blocked_reviews_map_to_common_exit_status():
    policy = NavigationExitPolicy(max_rounds=10)
    policy.begin()
    complete = ProgressDecision(ProgressStatus.COMPLETE, "at goal")
    blocked = ProgressDecision(ProgressStatus.BLOCKED, "target unknown")
    context = lambda decision: NavigationExitContext(1, observation(), observation(), decision)
    assert policy.evaluate(context(complete)).status is ExitStatus.COMPLETED
    assert policy.evaluate(context(blocked)).status is ExitStatus.BLOCKED


def test_task_specific_prompt_builder_is_injected_into_planner(monkeypatch):
    from planner.adapters.trajectory_planner.qwen_vl_planner import QwenVLPlanner
    import planner.adapters.trajectory_planner.qwen_vl_planner as adapter

    monkeypatch.setattr(adapter, "_image_url", lambda *args: "image")
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": "[[1,0,0,0]]"}}]}
    monkeypatch.setattr(adapter.requests, "post", Mock(return_value=response))
    builder = Mock()
    builder.build.return_value = "inspection-specific-prompt"
    planner = QwenVLPlanner("https://example.com", "model", "key", 10, prompt_builder=builder)
    planner.plan(Mock(), "inspect tower")
    builder.build.assert_called_once_with("inspect tower", builder.build.call_args.args[1], 5)
