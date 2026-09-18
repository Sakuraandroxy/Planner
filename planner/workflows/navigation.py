from __future__ import annotations

from planner.domain.mission import MissionStage
from planner.domain.result import StageResult
from planner.ports.observation_source import ObservationSource
from planner.ports.trajectory_planner import TrajectoryPlanner
from planner.services.execution import ExecutionService
from planner.services.motion.base import MotionPlanner
from planner.services.trajectory_transform import relative_to_world
from planner.services.trajectory_validation import TrajectoryValidator


class NavigationWorkflow:
    """One perception-plan-execute cycle for a navigation stage.

    The base planner deliberately has no semantic completion detector. A
    successful result means that the validated trajectory was executed.
    """

    def __init__(
        self,
        observation_source: ObservationSource,
        trajectory_planner: TrajectoryPlanner,
        validator: TrajectoryValidator,
        motion_planner: MotionPlanner,
        execution: ExecutionService,
    ):
        self.observation_source = observation_source
        self.trajectory_planner = trajectory_planner
        self.validator = validator
        self.motion_planner = motion_planner
        self.execution = execution

    def run(self, stage: MissionStage) -> StageResult[object]:
        try:
            observation = self.observation_source.capture()
            relative = self.trajectory_planner.plan(observation, stage.parameters.instruction)
            self.validator.validate_relative(relative)
            world = relative_to_world(relative, observation.vehicle_pose)
            self.validator.validate_world(world)
            motion = self.motion_planner.create_motion(world)
            self.execution.execute(motion)
            return StageResult(stage_id=stage.stage_id, success=True, value=world)
        except Exception as exc:
            return StageResult(stage_id=stage.stage_id, success=False, error=str(exc))

