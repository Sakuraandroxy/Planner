"""One bounded explicit action; never replay a relative command after failure."""
import logging
import math
import time

from planner.capabilities.basic_motion_conversion import BasicMotionConverter
from planner.capabilities.basic_motion_safety import BasicMotionSafety
from planner.capabilities.completion.basic_motion import BasicMotionCompletion
from planner.domain.basic_motion import BasicMotionParameters, BasicMotionResult, BasicMotionStatus
from planner.domain.execution import ExecutionBudget, StopResult
from planner.domain.mission import MissionStage, TaskKind
from planner.domain.motion import MotionLimits
from planner.domain.pose import wrap_yaw_deg
from planner.domain.result import StageResult, StageSummary, CompletionBasis, ResultMeasurement
from planner.errors import BudgetExceeded, ExecutionError, ObservationError, ProtocolError
from planner.ports.observation_source import ObservationSource
from planner.ports.collision_check import CollisionCheck
from planner.services.execution import ExecutionService
from planner.services.motion.base import MotionPlanner
from planner.services.trajectory_transform import relative_to_world
from planner.services.trajectory_validation import TrajectoryValidator

logger = logging.getLogger(__name__)


class BasicMotionWorkflow:
    def __init__(self, observation_source: ObservationSource, converter: BasicMotionConverter,
                 safety: BasicMotionSafety, completion: BasicMotionCompletion,
                 validator: TrajectoryValidator, motion_planner: MotionPlanner,
                 execution: ExecutionService, limits: MotionLimits, timeout_s: float,
                 clock=time.monotonic, *, collision_check: CollisionCheck | None = None):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("basic motion timeout must be finite and positive")
        self.observation_source, self.converter = observation_source, converter
        self.safety, self.completion = safety, completion
        self.validator, self.motion_planner, self.execution = validator, motion_planner, execution
        self.limits, self.timeout_s, self.clock = limits, timeout_s, clock
        self._closed = False
        self.collision_check = collision_check

    def prepare(self) -> None:
        if self._closed:
            raise ExecutionError("basic motion workflow is closed")

    def close(self) -> None:
        self._closed = True

    def run(self, stage: MissionStage) -> StageResult[object]:
        began = self.clock()
        budget = ExecutionBudget(began+self.timeout_s)
        start = target = actual = None
        decision = None
        stop = StopResult(False)
        status, reason = BasicMotionStatus.INVALID, "invalid basic motion"
        leased = False
        try:
            self.prepare()
            self.execution.acquire_flight(self, collision_check=self.collision_check)
            leased = True
            self.execution.set_deadline(budget.deadline_monotonic)
            logger.info("[BasicMotion %s] collision_check_enabled=%s", stage.stage_id,
                        self.collision_check is not None)
            if stage.kind is not TaskKind.BASIC_MOTION or not isinstance(stage.parameters, BasicMotionParameters):
                raise ProtocolError("basic motion stage has invalid parameters")
            relative = self.converter.create_trajectory(stage.parameters)
            stage.parameters.validate_limits(self.validator.max_step_m, self.validator.max_yaw_step_deg)
            self.validator.validate_relative(relative)
            # Official source capture holds flight, verifies stabilization, and
            # returns measured vehicle pose. No VLM request is made here.
            initial = self.observation_source.capture(budget=budget)
            start = initial.vehicle_pose
            if not initial.stopped:
                raise ObservationError("basic motion requires a verified stopped observation")
            self._check_budget(budget)
            world = relative_to_world(relative, start)
            self.validator.validate_world(world)
            target = world.poses[-1]
            motion = self.motion_planner.create_motion(world)
            safety = self.safety.check(motion)
            if not safety.allowed:
                status = BasicMotionStatus.BLOCKED
                raise ProtocolError(safety.reason)
            actual = self.execution.current_pose()
            drift = math.dist((start.x, start.y, start.z), (actual.x, actual.y, actual.z))
            if (drift > self.limits.position_tolerance_m or
                    abs(wrap_yaw_deg(actual.yaw_deg-start.yaw_deg)) > self.limits.yaw_tolerance_deg):
                raise ObservationError("basic motion anchor changed before execution; command not replayed")
            if self.execution.collision_state():
                raise ExecutionError("collision before basic motion execution")
            logger.info("[BasicMotion %s] relative=%s target=%s", stage.stage_id, relative, target)
            receipt = self.execution.execute_with_receipt(motion, budget=budget)
            actual = receipt.final_pose
            if receipt.completed_segments != len(motion.segments):
                raise ExecutionError("basic motion did not execute every segment")
            current = self.observation_source.capture(budget=budget)
            self._check_budget(budget)
            actual = current.vehicle_pose
            decision = self.completion.evaluate(target, current, self.execution.collision_state())
            reason = decision.reason
            if not decision.completed:
                status = BasicMotionStatus.UNCONFIRMED
                raise ExecutionError(reason)
            status = BasicMotionStatus.COMPLETED
            stop = StopResult(True)
        except KeyboardInterrupt:
            self.execution.stop()
            raise
        except Exception as exc:
            if isinstance(exc, BudgetExceeded):
                status = BasicMotionStatus.TIMEOUT
            elif isinstance(exc, ObservationError):
                status = BasicMotionStatus.OBSERVATION_ERROR
            elif isinstance(exc, ExecutionError):
                if status is not BasicMotionStatus.UNCONFIRMED:
                    status = BasicMotionStatus.EXECUTION_ERROR
            elif not isinstance(exc, (ProtocolError, ValueError)):
                status = BasicMotionStatus.EXECUTION_ERROR
            reason = str(exc)
            receipt = getattr(exc, "receipt", None)
            if receipt is not None:
                actual = receipt.final_pose
            stop = self.execution.stop_if_owner(self)
            logger.error("[BasicMotion %s] %s: %s", stage.stage_id, status.value, reason)
        finally:
            if leased:
                self.execution.release_flight(self)
        value = BasicMotionResult(
            status, reason, start, target, actual, self.clock()-began, stop,
            decision.position_error_m if decision else None,
            decision.yaw_error_deg if decision else None,
            "measured_pose_and_stopped_observation" if status is BasicMotionStatus.COMPLETED else "not_verified",
        )
        success = status is BasicMotionStatus.COMPLETED
        summary = StageSummary(status.value, reason,
            CompletionBasis.LOCAL_MEASUREMENT if success else CompletionBasis.NOT_VERIFIED,
            success, measurements=(ResultMeasurement("position_error", value.position_error_m, "m"),
                ResultMeasurement("yaw_error", value.yaw_error_deg, "deg"),
                ResultMeasurement("elapsed", value.elapsed_s, "s"),
                ResultMeasurement("stop_confirmed", stop.confirmed),
                ResultMeasurement("collision_check_enabled", self.collision_check is not None)))
        return StageResult(stage.stage_id, success, value, None if success else reason, summary)

    def _check_budget(self, budget: ExecutionBudget) -> None:
        if budget.remaining(self.clock()) <= 0:
            raise BudgetExceeded("basic motion stage deadline exhausted")
