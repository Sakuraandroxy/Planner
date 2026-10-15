"""Task admission must be complete and deterministic before any flight."""
from dataclasses import replace
from enum import Enum
from unittest.mock import Mock

import pytest

from planner.adapters.task_parser.openai_task_parser import OpenAITaskParser
from planner.adapters.task_parser.prompt import task_parser_system_prompt
from planner.adapters.task_parser.task_factory import mission_from_dict
from planner.adapters.task_parser.tasks import TaskCodec, builtin_task_codecs
from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.application.workflow_router import WorkflowRouter
from planner.capabilities.task_validation.basic_motion import BasicMotionStageValidator
from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.domain.result import CompletionBasis, StageResult, StageSummary
from planner.errors import ProtocolError
from planner.extensions.registry import ExtensionRegistry, TaskRegistration
from tests.contract.test_basic_motion_contract import envelope


class LocalOnlyWorkflow:
    def __init__(self):
        self.prepared = 0
        self.seen = []

    def prepare(self):
        self.prepared += 1

    def run(self, stage):
        self.seen.append(stage.stage_id)
        return StageResult(stage.stage_id, True, summary=StageSummary(
            "completed", "measured locally", CompletionBasis.LOCAL_MEASUREMENT, True))

    def close(self):
        pass


def registration():
    codec = next(c for c in builtin_task_codecs() if c.kind is TaskKind.BASIC_MOTION)
    return TaskRegistration(codec.kind, codec, BasicMotionStageValidator(), LocalOnlyWorkflow())


def test_registry_rejects_duplicate_missing_validator_and_incomplete_lifecycle():
    registry = ExtensionRegistry()
    task = registration()
    registry.register(task)
    with pytest.raises(ValueError, match="duplicate"):
        registry.register(task)
    with pytest.raises(ValueError, match="validator"):
        replace(task, validator=None)
    with pytest.raises(ValueError, match="lifecycle"):
        replace(task, workflow=object())
    with pytest.raises(ValueError, match="matching"):
        replace(task, codec=next(c for c in builtin_task_codecs() if c.kind is not task.kind))


def test_finite_kind_and_missing_validator_cannot_silently_admit_a_future_task():
    class FutureKind(str, Enum):
        ROAD = "road_follow"
    task = registration()
    with pytest.raises(ValueError, match="finite"):
        replace(task, kind=FutureKind.ROAD)
    stage = MissionStage("future", FutureKind.ROAD, object())
    with pytest.raises(ProtocolError, match="unsupported"):
        MissionValidator({task.kind: task.validator}).validate(MissionPlan("mission_plan_v1", (stage,)))
    with pytest.raises(ProtocolError, match="unsupported"):
        MissionValidator({}).validate(mission_from_dict(envelope()))


def test_parser_prompt_and_decoder_use_the_same_allowed_subset():
    codecs = tuple(c for c in builtin_task_codecs() if c.kind is TaskKind.VISUAL_GOAL_NAVIGATION)
    with pytest.raises(ProtocolError, match="not admitted"):
        mission_from_dict(envelope(), codecs=codecs)
    prompt = task_parser_system_prompt({TaskKind.VISUAL_GOAL_NAVIGATION}, codecs=codecs)
    assert "visual_goal_navigation: identify" in prompt
    assert "basic_motion" not in prompt
    with pytest.raises(ValueError, match="codec"):
        task_parser_system_prompt({TaskKind.BASIC_MOTION}, codecs=codecs)
    with pytest.raises(ProtocolError, match="duplicate"):
        mission_from_dict(envelope(), codecs=(*codecs, *codecs))


def test_one_common_parser_accepts_explicit_task_rules_without_task_branches(monkeypatch):
    import json
    original = registration().codec
    codec = TaskCodec(original.kind, "CUSTOM_NUMERIC_TASK_RULES", original.decode)
    response = Mock()
    response.json.return_value = {"choices": [{"message": {"content": json.dumps(envelope())}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr("requests.post", post)
    parser = OpenAITaskParser("https://offline.invalid/v1", "fake", "no-key", 10,
                             {codec.kind}, codecs=(codec,))
    assert parser.parse("ascend 12m").stages[0].parameters.up_m == 12
    assert "CUSTOM_NUMERIC_TASK_RULES" in post.call_args.kwargs["json"]["messages"][0]["content"]
    assert "visual_goal_navigation" not in parser.system_prompt


def test_local_workflow_needs_no_detection_or_completion_dependency(monkeypatch):
    monkeypatch.setattr("requests.post", Mock(side_effect=AssertionError("no model calls")))
    task = registration()
    plan = mission_from_dict(envelope(), codecs=(task.codec,))
    repeated = replace(plan, stages=(plan.stages[0], replace(plan.stages[0], stage_id="second")))
    validator = MissionValidator({task.kind: task.validator})
    runner = MissionRunner(WorkflowRouter({task.kind: task.workflow}))
    validator.validate(repeated)
    runner.prepare(repeated)
    assert runner.run(repeated).success
    assert task.workflow.prepared == 1
    assert task.workflow.seen == ["s1", "second"]


def test_wrong_parameters_and_execution_only_success_are_not_mission_completion():
    task = registration()
    plan = mission_from_dict(envelope())
    with pytest.raises(ProtocolError, match="invalid parameters"):
        MissionValidator({task.kind: task.validator}).validate(
            replace(plan, stages=(replace(plan.stages[0], parameters=object()),)))
    task.workflow.run = lambda stage: StageResult(stage.stage_id, True)
    result = MissionRunner(WorkflowRouter({task.kind: task.workflow})).run(plan)
    assert not result.success and "completion evidence" in result.error
    task.workflow.run = lambda stage: StageResult("another-stage", True,
        summary=StageSummary("done", "measured", CompletionBasis.LOCAL_MEASUREMENT, True))
    assert not MissionRunner(WorkflowRouter({task.kind: task.workflow})).run(plan).success


def test_duplicate_parser_codecs_are_rejected_before_any_http_call(monkeypatch):
    codec = registration().codec
    post = Mock()
    monkeypatch.setattr("requests.post", post)
    with pytest.raises(ValueError, match="duplicate"):
        OpenAITaskParser("https://offline.invalid/v1", "fake", "no-key", 10,
                        {codec.kind}, codecs=(codec, codec))
    post.assert_not_called()
