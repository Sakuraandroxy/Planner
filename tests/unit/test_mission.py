import pytest

from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.application.workflow_router import WorkflowRouter
from planner.domain.mission import MissionPlan, MissionStage, NavigationParameters, TaskKind
from planner.domain.result import StageResult


class Workflow:
    def __init__(self):
        self.seen = []

    def run(self, stage):
        self.seen.append(stage.stage_id)
        return StageResult(stage.stage_id, True)


def test_validator_and_runner_preserve_stage_order():
    stages = tuple(MissionStage(f"stage_{i}", TaskKind.NAVIGATION, NavigationParameters(f"move {i}")) for i in (1, 2))
    plan = MissionPlan("mission_plan_v1", stages)
    workflow = Workflow()
    router = WorkflowRouter({TaskKind.NAVIGATION: workflow})
    MissionValidator(router.supported_kinds).validate(plan)
    assert MissionRunner(router).run(plan).success
    assert workflow.seen == ["stage_1", "stage_2"]


def test_validator_rejects_unknown_protocol():
    with pytest.raises(Exception):
        MissionValidator({TaskKind.NAVIGATION}).validate(MissionPlan("legacy", ()))

