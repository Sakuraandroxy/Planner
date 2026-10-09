from copy import deepcopy
from pathlib import Path
from unittest.mock import Mock

import pytest
import yaml

from config.loader import load_config
from planner.adapters.task_parser.prompt import TASK_PARSER_SYSTEM_PROMPT, task_parser_system_prompt
from planner.adapters.task_parser.task_factory import mission_from_dict
from planner.adapters.task_parser.response_schema import parse_json_object
from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.application.workflow_router import WorkflowRouter
from planner.bootstrap import build_runtime
from planner.domain.mission import TaskKind
from planner.domain.result import StageResult
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.errors import ConfigurationError, ProtocolError


def goal_parameters():
    return {"instruction": "Approach red house", "target_description": "red house",
            "arrival_relation": "near", "standoff_m": None, "keep_target_in_view": True}


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


def test_kind_requires_enabled_capabilities_and_safe_standoff():
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
        def run(self, stage):
            calls.append(stage.kind)
            return StageResult(stage.stage_id, self.success, error=None if self.success else "blocked")
    router = WorkflowRouter({TaskKind.BASIC_MOTION: FakeWorkflow(False),
                             TaskKind.VISUAL_GOAL_NAVIGATION: FakeWorkflow(True)})
    assert not MissionRunner(router).run(plan()).success
    assert calls == [TaskKind.BASIC_MOTION]
    calls.clear()
    router = WorkflowRouter({kind: FakeWorkflow(True) for kind in TaskKind})
    assert MissionRunner(router).run(plan()).success
    assert calls == [TaskKind.BASIC_MOTION, TaskKind.VISUAL_GOAL_NAVIGATION]


def test_default_config_assembles_basic_motion_and_no_goal_backend(monkeypatch):
    monkeypatch.setattr("planner.bootstrap.AirSimConnection", Mock())
    config = load_config("config/base.yaml")
    assert not config.visual_goal.enabled and config.goal_observer is None
    runtime = build_runtime(config)  # Assembly only: never connects or takes off.
    assert runtime.application.task_parser.system_prompt == task_parser_system_prompt({TaskKind.BASIC_MOTION})
    assert TASK_PARSER_SYSTEM_PROMPT in runtime.application.task_parser.system_prompt
    assert runtime.application.runner.router.supported_kinds == {TaskKind.BASIC_MOTION}
    assert "visual_goal_navigation" in task_parser_system_prompt(set(TaskKind))


def test_new_template_requires_verified_backend_and_geometry(tmp_path):
    raw = yaml.safe_load(Path("config/visual_goal_navigation.example.yaml").read_text())
    raw["visual_goal_navigation"]["enabled"] = True
    def load(value):
        path = tmp_path/"test.yaml"
        path.write_text(yaml.safe_dump(value), encoding="utf-8")
        return load_config(path)
    with pytest.raises(ConfigurationError, match="goal_observer"):
        load(raw)
    raw["goal_observer"]["model"] = "verified-image-model"
    with pytest.raises(ConfigurationError, match="geometry"):
        load(raw)
    raw["visual_goal_navigation"].update(geometry_verified=True, initial_clearance_confirmed=True)
    assert load(raw).visual_goal.enabled
    broken = deepcopy(raw)
    broken["goal_observer"]["timeout_s"] = float("inf")
    with pytest.raises(ConfigurationError, match="timeout"):
        load(broken)


def test_candidate_limit_is_injected_into_both_observer_and_selector(tmp_path, monkeypatch):
    raw = yaml.safe_load(Path("config/visual_goal_navigation.example.yaml").read_text())
    raw["goal_observer"]["model"] = "verified-image-model"
    raw["visual_goal_navigation"].update(enabled=True, geometry_verified=True, initial_clearance_confirmed=True)
    raw["visual_goal_navigation"]["limits"]["maximum_surface_candidates"] = 3
    path = tmp_path/"test.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    monkeypatch.setattr("planner.bootstrap.AirSimConnection", Mock())
    runtime = build_runtime(load_config(path))
    workflow = runtime.application.runner.router._workflows[TaskKind.VISUAL_GOAL_NAVIGATION]
    assert workflow.goal_observer.maximum_candidates == 3
    assert workflow.surface_selector.limits.maximum_surface_candidates == 3
