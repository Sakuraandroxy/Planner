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
    def __init__(self, vehicle=None):
        self.vehicle = vehicle

    def capture(self):
        image = Image.new("RGB", (2, 2))
        pose = self.vehicle.pose if self.vehicle is not None else WorldPose(0, 0, -2, 0)
        return Observation(
            image, None, image, pose, "camera",
            CameraIntrinsics(2, 2, 1, 1, 1, 1, 90), (pose.x, pose.y, pose.z),
            ((1, 0, 0), (0, 1, 0), (0, 0, 1)), 1,
        )


class Planner:
    def plan(self, observation, instruction, **context):
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


def test_loop_reobserves_and_uses_current_guidance():
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
    first_call, second_call = planner.plan.call_args_list
    assert first_call.args[1] == "above house"
    assert first_call.kwargs["current_guidance"] is None
    assert first_call.kwargs["round_index"] == 1
    assert first_call.kwargs["previous_trajectory"] is None
    assert second_call.args[1] == "above house"
    assert second_call.kwargs["current_guidance"] == "move toward roof"
    assert second_call.kwargs["round_index"] == 2
    assert second_call.kwargs["previous_trajectory"] is not None
    assert second_call.kwargs["initial_pose"] == first_call.kwargs["initial_pose"]
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


def test_new_stage_resets_planning_context_at_current_pose():
    vehicle = FakeVehicle()
    source = ObservationSource(vehicle)
    planner = Mock(wraps=Planner())
    workflow = NavigationWorkflow(
        source, planner, TrajectoryValidator(5, 2, 45),
        SynchronizedMotionPlanner(2, MotionLimits()), ExecutionService(vehicle),
    )

    first = workflow.run(MissionStage("s1", TaskKind.NAVIGATION, NavigationParameters("forward")))
    second = workflow.run(MissionStage("s2", TaskKind.NAVIGATION, NavigationParameters("turn right")))

    assert first.success and second.success
    first_call, second_call = planner.plan.call_args_list
    assert first_call.args[1] == "forward"
    assert second_call.args[1] == "turn right"
    assert first_call.kwargs["round_index"] == second_call.kwargs["round_index"] == 1
    assert first_call.kwargs["current_guidance"] is None
    assert second_call.kwargs["current_guidance"] is None
    assert first_call.kwargs["previous_trajectory"] is None
    assert second_call.kwargs["previous_trajectory"] is None
    assert first_call.kwargs["initial_pose"] == WorldPose(0, 0, 0, 0)
    assert second_call.kwargs["initial_pose"] == WorldPose(5, 0, 0, 0)
