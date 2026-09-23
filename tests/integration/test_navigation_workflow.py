from PIL import Image
from unittest.mock import Mock
from planner.domain.progress import ProgressDecision, ProgressStatus

from planner.domain.mission import MissionStage, NavigationParameters, TaskKind
from planner.domain.motion import MotionLimits
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.services.execution import ExecutionService
from planner.services.motion import SynchronizedMotionPlanner
from planner.services.trajectory_validation import TrajectoryValidator
from planner.workflows import NavigationWorkflow
from planner.workflows.termination import NavigationExitPolicy
from tests.fakes.components import FakeVehicle


class ObservationSource:
    def capture(self):
        image = Image.new("RGB", (2, 2))
        return Observation(
            image, None, image, WorldPose(0, 0, -2, 0), "camera",
            CameraIntrinsics(2, 2, 1, 1, 1, 1, 90), (0, 0, -2),
            ((1, 0, 0), (0, 1, 0), (0, 0, 1)), 1,
        )


class Planner:
    def plan(self, observation, instruction):
        return RelativeTrajectory(tuple(RelativePoseDelta(1, 0, 0, 0) for _ in range(5)))


def test_navigation_workflow_runs_without_legacy_runtime():
    vehicle = FakeVehicle()
    workflow = NavigationWorkflow(
        ObservationSource(), Planner(), TrajectoryValidator(5, 2, 45),
        SynchronizedMotionPlanner(2, MotionLimits()), ExecutionService(vehicle),
    )
    result = workflow.run(MissionStage("stage_1", TaskKind.NAVIGATION, NavigationParameters("forward")))
    assert result.success
    assert len(vehicle.segments) == 5
    assert vehicle.pose.x == 5


def test_loop_reobserves_and_uses_remaining_instruction():
    source = Mock(wraps=ObservationSource())
    planner = Mock(wraps=Planner())
    reviewer = Mock()
    reviewer.review.side_effect = [
        ProgressDecision(ProgressStatus.CONTINUE, "not above roof", "move toward roof"),
        ProgressDecision(ProgressStatus.COMPLETE, "above roof", ""),
    ]
    vehicle = FakeVehicle()
    workflow = NavigationWorkflow(source, planner, TrajectoryValidator(5, 2, 45),
        SynchronizedMotionPlanner(2, MotionLimits()), ExecutionService(vehicle), reviewer=reviewer,
        exit_policy=NavigationExitPolicy(3))
    result = workflow.run(MissionStage("s", TaskKind.NAVIGATION, NavigationParameters("above house")))
    assert result.success
    assert result.value["completion"] == "model_review"
    assert source.capture.call_count == 3
    assert planner.plan.call_args.args[1] == "move toward roof"
    assert vehicle.hovered


def test_loop_limit_is_not_success():
    reviewer = Mock()
    reviewer.review.return_value = ProgressDecision(ProgressStatus.CONTINUE, "not there", "forward")
    vehicle = FakeVehicle()
    workflow = NavigationWorkflow(ObservationSource(), Planner(), TrajectoryValidator(5, 2, 45),
        SynchronizedMotionPlanner(2, MotionLimits()), ExecutionService(vehicle), reviewer=reviewer,
        exit_policy=NavigationExitPolicy(1))
    result = workflow.run(MissionStage("s", TaskKind.NAVIGATION, NavigationParameters("above house")))
    assert not result.success
    assert "limit_reached" in result.error
    assert vehicle.hovered
