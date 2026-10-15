from unittest.mock import Mock

from planner.domain.progress import ExitDecision, ExitStatus
from planner.workflows.termination import ExitPolicy
from tests.fakes.visual_goal import wall_observation


def test_common_exit_contract_remains_available_without_legacy_navigation():
    assert ExitPolicy is not None
    continuing = ExitDecision(ExitStatus.CONTINUE, "continue")
    assert not continuing.should_stop and not continuing.completed
    for status in (ExitStatus.COMPLETED, ExitStatus.BLOCKED, ExitStatus.STUCK, ExitStatus.LIMIT_REACHED):
        decision = ExitDecision(status, "explicit evidence")
        assert decision.should_stop
        assert decision.completed == (status is ExitStatus.COMPLETED)


def test_task_specific_prompt_builder_is_injected_into_planner(monkeypatch):
    from planner.adapters.trajectory_planner.qwen_vl_planner import QwenVLPlanner
    import planner.adapters.trajectory_planner.qwen_vl_planner as adapter

    monkeypatch.setattr(adapter, "_image_url", lambda *args: "image")
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": "[[1,0,0,0],[1,0,0,0],[1,0,0,0],[1,0,0,0],[1,0,0,0]]"}}]}
    monkeypatch.setattr(adapter.requests, "post", Mock(return_value=response))
    builder = Mock()
    builder.system_prompt.return_value = "trajectory-system-prompt"
    builder.build_user_prompt.return_value = "inspection-specific-prompt"
    planner = QwenVLPlanner("https://example.com", "model", "key", 10, prompt_builder=builder)
    planner.plan(wall_observation(), "inspect tower")
    builder.system_prompt.assert_called_once_with()
    call = builder.build_user_prompt.call_args
    assert call.args[0] == "inspect tower"
    assert call.args[2] == 5
    assert call.kwargs["round_index"] == 1
    assert call.kwargs["previous_trajectory"] is None
    assert call.kwargs["pending_trajectory"] is None
