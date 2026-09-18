from PIL import Image

from planner.domain.mission import MissionStage, NavigationParameters, TaskKind
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.services.execution import ExecutionService
from planner.services.motion import PassThroughMotionPlanner
from planner.services.trajectory_validation import TrajectoryValidator
from planner.workflows import NavigationWorkflow
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
        PassThroughMotionPlanner(2), ExecutionService(vehicle),
    )
    result = workflow.run(MissionStage("stage_1", TaskKind.NAVIGATION, NavigationParameters("forward")))
    assert result.success
    assert len(vehicle.segments) == 5
    assert vehicle.pose.x == 5
