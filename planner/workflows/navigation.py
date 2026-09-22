from __future__ import annotations

import logging
from planner.domain.mission import MissionStage
from planner.domain.result import StageResult
from planner.ports.observation_source import ObservationSource
from planner.ports.progress_reviewer import ProgressReviewer
from planner.ports.trajectory_planner import TrajectoryPlanner
from planner.services.execution import ExecutionService
from planner.services.motion.base import MotionPlanner
from planner.services.trajectory_transform import relative_to_world
from planner.services.trajectory_validation import TrajectoryValidator
from planner.workflows.termination import ExitPolicy, NavigationExitContext, NavigationExitPolicy

logger = logging.getLogger(__name__)


class NavigationWorkflow:
    """Bounded observe-plan-execute loop with an optional progress reviewer."""

    def __init__(
        self,
        observation_source: ObservationSource,
        trajectory_planner: TrajectoryPlanner,
        validator: TrajectoryValidator,
        motion_planner: MotionPlanner,
        execution: ExecutionService,
        reviewer: ProgressReviewer | None = None,
        exit_policy: ExitPolicy[NavigationExitContext] | None = None,
    ):
        self.observation_source = observation_source
        self.trajectory_planner = trajectory_planner
        self.validator = validator
        self.motion_planner = motion_planner
        self.execution = execution
        self.reviewer = reviewer
        self.exit_policy = exit_policy or NavigationExitPolicy(max_rounds=1)

    def run(self, stage: MissionStage) -> StageResult[object]:
        try:
            logger.info("[Stage %s] start: %s", stage.stage_id, stage.parameters.instruction)
            observation = self.observation_source.capture()
            initial = observation
            remaining = stage.parameters.instruction
            self.exit_policy.begin()
            round_index = 0
            while True:
                round_index += 1
                logger.info("[Stage %s] round %s", stage.stage_id, round_index)
                relative = self.trajectory_planner.plan(observation, remaining)
                self.validator.validate_relative(relative)
                world = relative_to_world(relative, observation.vehicle_pose)
                self.validator.validate_world(world)
                logger.info("[Stage %s] validated world trajectory: %s", stage.stage_id, world)
                self.execution.execute(self.motion_planner.create_motion(world))
                if self.reviewer is None:
                    logger.info("Trajectory executed; task completion NOT verified")
                    return StageResult(stage_id=stage.stage_id, success=True, value=world)
                self.execution.hold()
                current = self.observation_source.capture()
                decision = self.reviewer.review(stage.parameters.instruction, initial, current, round_index)
                exit_decision = self.exit_policy.evaluate(NavigationExitContext(
                    round_index=round_index, previous=observation, current=current, progress=decision,
                ))
                logger.info("[Stage %s] review=%s exit=%s evidence=%s", stage.stage_id,
                            decision.status.value, exit_decision.status.value, exit_decision.reason)
                if exit_decision.completed:
                    return StageResult(stage_id=stage.stage_id, success=True, value={
                        "completion": "model_review", "reason": exit_decision.reason,
                    })
                if exit_decision.should_stop:
                    raise RuntimeError(f"{exit_decision.status.value}: {exit_decision.reason}")
                remaining = decision.next_instruction
                observation = current
        except Exception as exc:
            if self.reviewer is not None:
                try:
                    self.execution.hold()
                except Exception:
                    logger.exception("Failed to hover after navigation failure")
            logger.error("[Stage %s] failed: %s", stage.stage_id, exc)
            return StageResult(stage_id=stage.stage_id, success=False, error=str(exc))

