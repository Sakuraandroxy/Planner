"""Real basic/visual workflows share execution; all external services remain fakes."""
from dataclasses import replace
from unittest.mock import Mock

import pytest

from planner.application.application import PlannerApplication
from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.application.workflow_router import WorkflowRouter
from planner.capabilities.basic_motion_conversion import BasicMotionConverter
from planner.capabilities.basic_motion_safety import BasicMotionSafety
from planner.capabilities.completion.basic_motion import BasicMotionCompletion
from planner.capabilities.task_validation.basic_motion import BasicMotionStageValidator
from planner.capabilities.task_validation.visual_goal_navigation import VisualGoalStageValidator
from planner.domain.basic_motion import BasicMotionParameters, ConfirmedFreeSpace
from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.domain.motion import MotionLimits
from planner.domain.pose import RelativePoseDelta
from planner.domain.result import CompletionBasis
from planner.errors import ProtocolError
from planner.workflows.basic_motion import BasicMotionWorkflow
from tests.integration.test_visual_goal_workflow import setup


def pipeline():
    visual, stage, source, observer, planner, vehicle, clock = setup()
    planner.points = tuple(RelativePoseDelta(.5, 0, 0, 0) for _ in range(5))
    basic = BasicMotionWorkflow(source, BasicMotionConverter(),
        BasicMotionSafety(ConfirmedFreeSpace(True, (-30, -30, -30), (30, 30, 30))),
        BasicMotionCompletion(MotionLimits()), visual.validator, visual.motion, visual.execution,
        MotionLimits(), 60, clock)
    first = MissionStage("move", TaskKind.BASIC_MOTION,
        BasicMotionParameters("Fly forward one meter. 向前飞一米。", forward_m=1))
    plan = MissionPlan("mission_plan_v1", (first, stage))
    validator = MissionValidator({
        TaskKind.BASIC_MOTION: BasicMotionStageValidator(),
        TaskKind.VISUAL_GOAL_NAVIGATION: VisualGoalStageValidator(visual.limits),
    })
    parser = Mock()
    parser.parse.return_value = plan
    runner = MissionRunner(WorkflowRouter({
        TaskKind.BASIC_MOTION: basic, TaskKind.VISUAL_GOAL_NAVIGATION: visual}))
    return PlannerApplication(parser, validator, runner), basic, visual, vehicle, source


def test_real_workflows_run_in_order_with_distinct_completion_evidence(monkeypatch):
    monkeypatch.setattr("requests.post", Mock(side_effect=AssertionError("offline only")))
    app, basic, visual, vehicle, source = pipeline()
    try:
        result = app.execute("move, then approach")
        assert result.success, result.error
        assert [stage.stage_id for stage in result.stages] == ["move", "goal"]
        assert result.stages[0].summary.completion_basis is CompletionBasis.LOCAL_MEASUREMENT
        assert result.stages[1].summary.completion_basis is CompletionBasis.MODEL_WITH_LOCAL_CHECKS
        assert result.stages[0].value.target.x == pytest.approx(1)
        assert result.stages[1].value.completion_confirmed
        assert not result.stages[1].value.remaining_motion.segments
    finally:
        visual.close()
        basic.close()


def test_missing_later_completion_dependency_rejects_before_first_stage_moves():
    app, basic, visual, vehicle, source = pipeline()
    checker_calls = visual.verification.calls
    visual.verification = None
    try:
        with pytest.raises(ProtocolError, match="completion checker"):
            app.prepare("move, then approach")
        assert not vehicle.segments and source.captures == 0
    finally:
        checker_calls.close()
        visual.close()
        basic.close()
