import pytest

from planner.application.mission_runner import MissionRunner
from tests.fakes.missions import mission_validator as MissionValidator
from planner.application.workflow_router import WorkflowRouter
from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.result import StageResult, StageSummary, CompletionBasis


class Workflow:
    def __init__(self):
        self.seen = []

    def prepare(self):
        pass

    def close(self):
        pass

    def run(self, stage):
        self.seen.append(stage.stage_id)
        return StageResult(stage.stage_id, True, summary=StageSummary("completed", "fake measured pose",
            CompletionBasis.LOCAL_MEASUREMENT, True))


def test_validator_and_runner_preserve_stage_order():
    stages = tuple(MissionStage(f"stage_{i}", TaskKind.BASIC_MOTION,
                               BasicMotionParameters(f"move {i}m", forward_m=i)) for i in (1, 2))
    plan = MissionPlan("mission_plan_v1", stages)
    workflow = Workflow()
    router = WorkflowRouter({TaskKind.BASIC_MOTION: workflow})
    MissionValidator(router.supported_kinds).validate(plan)
    assert MissionRunner(router).run(plan).success
    assert workflow.seen == ["stage_1", "stage_2"]


def test_validator_rejects_unknown_protocol():
    with pytest.raises(Exception):
        MissionValidator({TaskKind.BASIC_MOTION}).validate(MissionPlan("legacy", ()))

