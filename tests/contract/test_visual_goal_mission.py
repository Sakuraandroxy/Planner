from tests.fakes.configuration import visual_config_data, write_config
from copy import deepcopy
from unittest.mock import Mock

import pytest

from config.loader import load_config
from planner.adapters.task_parser.prompt import TASK_PARSER_SYSTEM_PROMPT, task_parser_system_prompt
from planner.adapters.task_parser.task_factory import mission_from_dict
from planner.adapters.task_parser.response_schema import parse_json_object
from planner.application.mission_runner import MissionRunner
from tests.fakes.missions import mission_validator as MissionValidator
from planner.application.workflow_router import WorkflowRouter
from planner.bootstrap import build_runtime
from planner.domain.mission import TaskKind
from planner.domain.result import StageResult, StageSummary, CompletionBasis
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.errors import ConfigurationError, ProtocolError


def goal_parameters():
    return {"instruction": "Approach red house", "target_description": "red house",
            "arrival_relation": "near", "keep_target_in_view": True}


def plan(parameters=None):
    return mission_from_dict({"protocol_version": "mission_plan_v1", "stages": [
        {"stage_id": "first", "kind": "basic_motion", "parameters": {
            "instruction": "Turn right 90 degrees", "forward_m": 0, "right_m": 0,
            "up_m": 0, "yaw_right_deg": 90}},
        {"stage_id": "second", "kind": "visual_goal_navigation", "parameters": parameters or goal_parameters()},
    ]})


@pytest.mark.parametrize("field,value", [("arrival_relation", "above"), ("standoff_m", True),
    ("standoff_m", float("nan")), ("keep_target_in_view", "false"), ("target_description", ""), ("unknown", 1)])
def test_strict_new_task_parameters(field, value):
    parameters = goal_parameters()
    parameters[field] = value
    with pytest.raises(ProtocolError):
        plan(parameters)


@pytest.mark.parametrize("text", ['{"stages":[],"stages":[1]}', '{"standoff_m":NaN}'])
def test_mission_json_rejects_duplicate_and_nonfinite_fields(text):
    with pytest.raises(ProtocolError):
        parse_json_object(text)


def test_kind_requires_enabled_capabilities_and_rejects_legacy_distance():
    with pytest.raises(ProtocolError):
        MissionValidator({TaskKind.BASIC_MOTION}).validate(plan())
    parameters = goal_parameters()
    parameters["standoff_m"] = 2
    with pytest.raises(ProtocolError):
        MissionValidator(set(TaskKind), VisualGoalLimits()).validate(plan(parameters))


def test_mixed_stages_route_in_order_and_failure_stops_following_stages():
    calls = []
    class FakeWorkflow:
        def __init__(self, success):
            self.success = success
        def prepare(self):
            pass
        def close(self):
            pass
        def run(self, stage):
            calls.append(stage.kind)
            return StageResult(stage.stage_id, self.success, error=None if self.success else "blocked",
                summary=StageSummary("completed", "fake local evidence",
                    CompletionBasis.LOCAL_MEASUREMENT, True) if self.success else None)
    router = WorkflowRouter({TaskKind.BASIC_MOTION: FakeWorkflow(False),
                             TaskKind.VISUAL_GOAL_NAVIGATION: FakeWorkflow(True)})
    assert not MissionRunner(router).run(plan()).success
    assert calls == [TaskKind.BASIC_MOTION]
    calls.clear()
    router = WorkflowRouter({kind: FakeWorkflow(True) for kind in TaskKind})
    assert MissionRunner(router).run(plan()).success
    assert calls == [TaskKind.BASIC_MOTION, TaskKind.VISUAL_GOAL_NAVIGATION]


def test_default_config_registers_all_tasks_without_constructing_goal_backend(monkeypatch):
    monkeypatch.setattr("planner.bootstrap.AirSimConnection", Mock())
    config = load_config("config/base.yaml")
    assert config.visual_goal.enabled and config.target_detection is not None
    factory = Mock(side_effect=AssertionError("unused visual workflow must not be constructed"))
    post = Mock(side_effect=AssertionError("configuration presence must not call a model"))
    monkeypatch.setattr("planner.visual_goal_bootstrap.build_visual_goal_workflow", factory)
    monkeypatch.setattr("requests.post", post)
    runtime = build_runtime(config)  # Assembly only: never connects or takes off.
    factory.assert_not_called()
    post.assert_not_called()
    assert runtime.application.task_parser.system_prompt == task_parser_system_prompt(set(TaskKind))
    assert TASK_PARSER_SYSTEM_PROMPT in runtime.application.task_parser.system_prompt
    assert runtime.application.runner.router.supported_kinds == set(TaskKind)
    assert "visual_goal_navigation" in task_parser_system_prompt(set(TaskKind))


def test_main_config_requires_verified_backend_and_geometry(tmp_path):
    raw = visual_config_data()
    raw['visual_goal_navigation'].update(enabled=True, geometry_verified=False)
    def load(value):
        path = tmp_path/"test.yaml"
        write_config(path, value)
        from config.visual_goal_readiness import validate_visual_goal_readiness
        result = load_config(path)
        validate_visual_goal_readiness(result)
        return result
    with pytest.raises(ConfigurationError, match="TARGET_DETECTION_API_KEY"):
        load(raw)
    raw["target_detection"]["api_key"] = "detector-test-key"
    with pytest.raises(ConfigurationError, match="geometry"):
        load(raw)
    raw['visual_goal_navigation'].update(geometry_verified=True)
    with pytest.raises(ConfigurationError, match="TASK_COMPLETION_API_KEY"):
        load(raw)
    raw["task_completion"]["api_key"] = "test-only-key"
    assert load(raw).visual_goal.enabled
    broken = deepcopy(raw)
    broken["target_detection"]["timeout_s"] = float("inf")
    with pytest.raises(ConfigurationError, match="timeout"):
        load(broken)
