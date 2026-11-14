import pytest

from planner.application.application import PlannerApplication
from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.application.reporting import mission_report
from planner.application.workflow_router import WorkflowRouter
from planner.capabilities.task_validation.basic_motion import (
    BasicMotionStageValidator,
)
from planner.capabilities.task_validation.free_navigation import (
    FreeNavigationStageValidator,
)
from planner.capabilities.task_validation.visual_goal_navigation import (
    VisualGoalStageValidator,
)
from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.free_navigation import FreeNavigationParameters
from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.domain.result import (
    CompletionBasis,
    MissionResult,
    ResultMeasurement,
    StageResult,
    StageSummary,
)
from planner.domain.visual_goal import VisualGoalNavigationParameters
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.errors import ProtocolError


def _basic(stage_id="stage_1"):
    return MissionStage(
        stage_id,
        TaskKind.BASIC_MOTION,
        BasicMotionParameters("drive forward 5 metres", forward_m=5.0),
    )


def _visual(stage_id="stage_2"):
    return MissionStage(
        stage_id,
        TaskKind.VISUAL_GOAL_NAVIGATION,
        VisualGoalNavigationParameters("drive to the red car", "the red car"),
    )


def _free(stage_id="stage_3"):
    return MissionStage(
        stage_id,
        TaskKind.FREE_NAVIGATION,
        FreeNavigationParameters("keep driving along the road"),
    )


def _plan(*stages):
    return MissionPlan("mission_plan_v1", tuple(stages))


def _summary(confirmed=True, status="completed", reason="ok"):
    return StageSummary(
        status,
        reason,
        (
            CompletionBasis.LOCAL_MEASUREMENT
            if confirmed
            else CompletionBasis.NOT_VERIFIED
        ),
        confirmed,
    )


def _ok(stage_id):
    return StageResult(stage_id, True, summary=_summary())


def _failed(stage_id, error="boom"):
    return StageResult(
        stage_id,
        False,
        error=error,
        summary=StageSummary("failed", error),
    )


def _validator():
    return MissionValidator(
        {
            TaskKind.BASIC_MOTION: BasicMotionStageValidator(),
            TaskKind.VISUAL_GOAL_NAVIGATION: VisualGoalStageValidator(
                VisualGoalLimits()
            ),
        }
    )


def test_validator_accepts_a_well_formed_plan():
    _validator().validate(_plan(_basic(), _visual()))


def _terminal_validator():
    return MissionValidator(
        {
            TaskKind.BASIC_MOTION: BasicMotionStageValidator(),
            TaskKind.VISUAL_GOAL_NAVIGATION: VisualGoalStageValidator(
                VisualGoalLimits()
            ),
            TaskKind.FREE_NAVIGATION: FreeNavigationStageValidator(),
        },
        terminal_kinds={TaskKind.FREE_NAVIGATION},
    )


def test_terminal_kind_must_be_the_final_stage():
    validator = _terminal_validator()
    validator.validate(_plan(_basic(), _free()))
    with pytest.raises(ProtocolError) as error:
        validator.validate(_plan(_free(), _basic()))
    assert "free_navigation must be the final stage" in str(error.value)


def test_terminal_kind_requires_a_registered_stage_validator():
    with pytest.raises(ValueError):
        MissionValidator(
            {TaskKind.BASIC_MOTION: BasicMotionStageValidator()},
            terminal_kinds={TaskKind.FREE_NAVIGATION},
        )
    with pytest.raises(ValueError):
        MissionValidator(
            {TaskKind.FREE_NAVIGATION: FreeNavigationStageValidator()},
            terminal_kinds={"free_navigation"},
        )


@pytest.mark.parametrize(
    "plan",
    [
        MissionPlan("v2", (_basic(),)),
        MissionPlan("mission_plan_v1", ()),
        _plan(_basic("dup"), _basic("dup")),
        _plan(MissionStage("", TaskKind.BASIC_MOTION, BasicMotionParameters(
            "drive", forward_m=1.0
        ))),
    ],
)
def test_validator_rejects_malformed_plans(plan):
    with pytest.raises(ProtocolError):
        _validator().validate(plan)


def test_validator_requires_explicit_support_and_parameter_types():
    basic_only = MissionValidator(
        {TaskKind.BASIC_MOTION: BasicMotionStageValidator()}
    )
    with pytest.raises(ProtocolError):
        basic_only.validate(_plan(_visual()))
    with pytest.raises(ProtocolError):
        _validator().validate(
            MissionPlan(
                "mission_plan_v1",
                (
                    MissionStage(
                        "stage_1",
                        TaskKind.BASIC_MOTION,
                        VisualGoalNavigationParameters("drive", "car"),
                    ),
                ),
            )
        )
    with pytest.raises(ValueError):
        MissionValidator({"basic_motion": BasicMotionStageValidator()})
    with pytest.raises(ValueError):
        MissionValidator({TaskKind.BASIC_MOTION: object()})


class FakeWorkflow:
    def __init__(self, results=()):
        self.results = list(results)
        self.prepared = 0
        self.closed = 0
        self.runs = []

    def prepare(self):
        self.prepared += 1

    def close(self):
        self.closed += 1

    def run(self, stage):
        self.runs.append(stage.stage_id)
        return self.results.pop(0)


def _router(basic, visual):
    return WorkflowRouter(
        {
            TaskKind.BASIC_MOTION: basic,
            TaskKind.VISUAL_GOAL_NAVIGATION: visual,
        }
    )


def test_router_dispatches_by_kind():
    basic = FakeWorkflow([_ok("stage_1")])
    visual = FakeWorkflow([_ok("stage_2")])
    router = _router(basic, visual)
    assert router.supported_kinds == {
        TaskKind.BASIC_MOTION,
        TaskKind.VISUAL_GOAL_NAVIGATION,
    }
    router.workflow_for(_basic()).run(_basic())
    router.workflow_for(_visual()).run(_visual())
    assert basic.runs == ["stage_1"]
    assert visual.runs == ["stage_2"]
    with pytest.raises(ProtocolError):
        WorkflowRouter({}).workflow_for(_basic())


def test_runner_executes_stages_in_order():
    basic = FakeWorkflow([_ok("stage_1")])
    visual = FakeWorkflow([_ok("stage_2")])
    runner = MissionRunner(_router(basic, visual))
    runner.prepare(_plan(_basic("stage_1"), _basic("stage_3"), _visual()))
    assert basic.prepared == 1
    assert visual.prepared == 1

    result = runner.run(_plan(_basic("stage_1"), _visual("stage_2")))
    assert result.success is True
    assert [stage.stage_id for stage in result.stages] == [
        "stage_1",
        "stage_2",
    ]
    assert basic.runs == ["stage_1"]
    assert visual.runs == ["stage_2"]


def test_runner_stops_at_the_first_failure():
    basic = FakeWorkflow([_failed("stage_1")])
    visual = FakeWorkflow([_ok("stage_2")])
    result = MissionRunner(_router(basic, visual)).run(
        _plan(_basic("stage_1"), _visual("stage_2"))
    )
    assert result.success is False
    assert result.error == "boom"
    assert visual.runs == []


def test_runner_requires_confirmed_completion_evidence():
    unconfirmed = StageResult(
        "stage_1",
        True,
        summary=_summary(confirmed=False),
    )
    basic = FakeWorkflow([unconfirmed])
    result = MissionRunner(_router(basic, FakeWorkflow())).run(
        _plan(_basic("stage_1"))
    )
    assert result.success is False
    assert "confirmed stage completion evidence" in result.error
    assert result.stages[0].summary.status == "invalid_result"


def test_runner_rejects_a_mismatched_stage_id():
    basic = FakeWorkflow([_ok("other")])
    result = MissionRunner(_router(basic, FakeWorkflow())).run(
        _plan(_basic("stage_1"))
    )
    assert result.success is False
    assert "mismatched stage ID" in result.error


class FakeParser:
    def __init__(self, plan):
        self.plan = plan
        self.instructions = []

    def parse(self, instruction):
        self.instructions.append(instruction)
        return self.plan


def _application(plan):
    basic = FakeWorkflow([_ok("stage_1")])
    visual = FakeWorkflow([_ok("stage_2")])
    parser = FakeParser(plan)
    application = PlannerApplication(
        parser,
        _validator(),
        MissionRunner(_router(basic, visual)),
    )
    return application, parser, basic, visual


def test_application_parses_validates_and_executes():
    plan = _plan(_basic("stage_1"), _visual("stage_2"))
    application, parser, basic, visual = _application(plan)
    assert application.prepare("go") is plan
    assert parser.instructions == ["go"]
    assert basic.prepared == 1 and visual.prepared == 1

    result = application.execute("go")
    assert result.success is True
    assert basic.runs == ["stage_1"]
    assert visual.runs == ["stage_2"]


def test_application_parse_call_overrides_the_parser():
    plan = _plan(_basic("stage_1"))
    basic = FakeWorkflow([_ok("stage_1")])

    class ExplodingParser:
        def parse(self, instruction):
            raise AssertionError("the parser must not be called")

    application = PlannerApplication(
        ExplodingParser(),
        _validator(),
        MissionRunner(_router(basic, FakeWorkflow())),
        parse_call=lambda instruction: plan,
    )
    assert application.execute("go").success is True


def test_application_rejects_an_invalid_plan_before_execution():
    application, _, basic, _ = _application(
        MissionPlan("wrong", (_basic("stage_1"),))
    )
    with pytest.raises(ProtocolError):
        application.prepare("go")
    assert basic.runs == []


def test_mission_report_lists_stage_evidence_and_exit_code():
    summary = StageSummary(
        "completed",
        "requested position reached",
        CompletionBasis.LOCAL_MEASUREMENT,
        True,
        measurements=(ResultMeasurement("distance", 5.0, "m"),),
    )
    result = MissionResult(
        True, (StageResult("stage_1", True, summary=summary),)
    )
    lines, code = mission_report(result)
    assert code == 0
    assert "Stage stage_1: completed; basis=local_measurement;" in lines[0]
    assert "distance=5.0m" in lines[0]
    assert lines[-1] == (
        "Mission completion confirmed by task-specific evidence: 1 stage(s)"
    )


def test_mission_report_flags_unconfirmed_stages():
    result = MissionResult(
        True,
        (
            StageResult(
                "stage_1", True, summary=_summary(confirmed=False)
            ),
        ),
    )
    lines, code = mission_report(result)
    assert code == 0
    assert lines[-1] == (
        "Stages executed: 1; task completion not verified"
    )


def test_mission_report_handles_failure_and_missing_summary():
    failed = MissionResult(False, (), error="parse failed")
    lines, code = mission_report(failed)
    assert code == 1
    assert lines == ("Mission failed: parse failed",)

    missing = MissionResult(
        False, (StageResult("stage_1", False, error="no evidence"),)
    )
    lines, code = mission_report(missing)
    assert code == 1
    assert "completion not verified" in lines[0]
    assert "reason=no evidence" in lines[0]