"""Execute numeric ground commands directly, without observation gates.

The reference AirSim vehicle exposes a native basic-motion primitive. A
kinematic CARLA car has no such call, so this workflow builds one synchronized
quintic ``MotionSegment`` and runs it through the shared ``ExecutionService``:
20 Hz control, stop checkpoints and settle verification stay identical. A
``MotionSegment`` accepts an explicit ``yaw_delta_deg``, so turns beyond 180
degrees (full turns) execute directly without ``WorldTrajectory``'s cap.
"""
import logging
import math
import time

from planner.capabilities.basic_motion_conversion import BasicMotionConverter
from planner.domain.basic_motion import BasicMotionResult, BasicMotionStatus
from planner.domain.execution import StopResult
from planner.domain.motion import MotionLimits
from planner.domain.pose import wrap_yaw_deg
from planner.domain.result import (
    CompletionBasis,
    ResultMeasurement,
    StageResult,
    StageSummary,
)
from planner.domain.trajectory import MotionSegment, MotionTrajectory
from planner.errors import ExecutionError
from planner.services.execution import ExecutionService
from planner.services.motion.synchronized import SMOOTH_PROFILE, duration_for

logger = logging.getLogger(__name__)


class BasicMotionWorkflow:
    def __init__(
        self,
        vehicle,
        motion,
        execution: ExecutionService,
        clock=time.monotonic,
        *,
        limits=MotionLimits(),
        telemetry=None,
    ):
        self.vehicle = vehicle
        self.motion = motion
        self.execution = execution
        self.clock = clock
        self.converter = BasicMotionConverter()
        self.limits = limits
        self.telemetry = telemetry

    def prepare(self) -> None:
        pass

    def close(self) -> None:
        pass

    def run(self, stage) -> StageResult[BasicMotionResult]:
        began = self.clock()
        start = target = actual = None
        leased = False
        stop = StopResult(False)
        status = BasicMotionStatus.EXECUTION_ERROR
        reason = "execution did not finish"
        try:
            # Coordinate shared ownership; this task has no visual admission gate.
            self.execution.acquire_flight(self)
            leased = True
            start = self.vehicle.current_pose()
            if self.telemetry is not None:
                self.telemetry.publish_no_queue(
                    instruction=stage.parameters.instruction,
                    stage_id=stage.stage_id,
                    pose=start,
                )
            target = self.converter.create_target(stage.parameters, start)
            yaw_delta = stage.parameters.yaw_right_deg
            logger.info(
                "[BasicMotion %s] direct target=%s yaw_delta_deg=%s",
                stage.stage_id,
                target,
                yaw_delta,
            )
            segment = MotionSegment(
                start=start,
                end=target,
                profile=SMOOTH_PROFILE,
                duration_s=duration_for(
                    start,
                    target,
                    self.motion.speed_mps,
                    self.motion.limits,
                    yaw_delta_deg=yaw_delta,
                ),
                target_speed_mps=self.motion.speed_mps,
                yaw_delta_deg=yaw_delta,
            )
            self.execution.execute(MotionTrajectory((segment,)))
            actual = self.vehicle.current_pose()
            if (
                math.dist(
                    (actual.x, actual.y, actual.z),
                    (target.x, target.y, target.z),
                )
                > self.limits.position_tolerance_m
                or abs(wrap_yaw_deg(actual.yaw_deg - target.yaw_deg))
                > self.limits.yaw_tolerance_deg
            ):
                raise ExecutionError(
                    "basic motion returned before the requested "
                    "position/yaw was reached"
                )
            status = BasicMotionStatus.COMPLETED
            reason = (
                "requested position/yaw reached and held; "
                "confirmed by local measurement"
            )
            stop = StopResult(True)
        except KeyboardInterrupt:
            self.execution.stop_if_owner(self)
            raise
        except Exception as exc:
            reason = str(exc)
            stop = self.execution.stop_if_owner(self)
            logger.error(
                "[BasicMotion %s] execution_error: %s",
                stage.stage_id,
                reason,
            )
        finally:
            if leased:
                self.execution.release_flight(self)
        success = status is BasicMotionStatus.COMPLETED
        basis = (
            CompletionBasis.LOCAL_MEASUREMENT
            if success
            else CompletionBasis.NOT_VERIFIED
        )
        value = BasicMotionResult(
            status,
            reason,
            start,
            target,
            actual,
            self.clock() - began,
            stop,
            basis.value,
        )
        summary = StageSummary(
            status.value,
            reason,
            basis,
            success,
            measurements=(
                ResultMeasurement("elapsed", value.elapsed_s, "s"),
                ResultMeasurement("stop_confirmed", stop.confirmed),
            ),
            final_pose=actual,
        )
        return StageResult(
            stage.stage_id, success, value, None if success else reason, summary
        )