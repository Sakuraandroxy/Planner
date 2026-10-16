"""Execute numeric commands directly, without observation or motion admission gates."""
import logging
import time

from planner.capabilities.basic_motion_conversion import BasicMotionConverter
from planner.domain.basic_motion import BasicMotionResult, BasicMotionStatus
from planner.domain.execution import StopResult
from planner.domain.result import StageResult, StageSummary, CompletionBasis, ResultMeasurement
from planner.ports.basic_motion_vehicle import BasicMotionVehicle
from planner.services.execution import ExecutionService

logger = logging.getLogger(__name__)


class BasicMotionWorkflow:
    def __init__(self, vehicle: BasicMotionVehicle, execution: ExecutionService, clock=time.monotonic):
        self.vehicle, self.execution, self.clock = vehicle, execution, clock
        self.converter = BasicMotionConverter()

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
            # Coordinate shared flight ownership; no task-space or arrival checks.
            self.execution.acquire_flight(self)
            leased = True
            start = self.vehicle.current_pose()
            target = self.converter.create_target(stage.parameters, start)
            logger.info("[BasicMotion %s] direct target=%s yaw_delta_deg=%s",
                        stage.stage_id, target, stage.parameters.yaw_right_deg)
            self.vehicle.execute_basic_motion(start, target, stage.parameters.yaw_right_deg)
            status = BasicMotionStatus.COMPLETED
            reason = "motion API execution finished; arrival not independently verified"
            try:
                actual = self.vehicle.current_pose()  # Diagnostic only; never gates success.
            except Exception:
                logger.info("[BasicMotion %s] final pose unavailable", stage.stage_id)
        except KeyboardInterrupt:
            self.execution.stop_if_owner(self)
            raise
        except Exception as exc:
            reason = str(exc)
            stop = self.execution.stop_if_owner(self)
            logger.error("[BasicMotion %s] execution_error: %s", stage.stage_id, reason)
        finally:
            if leased:
                self.execution.release_flight(self)
        success = status is BasicMotionStatus.COMPLETED
        basis = CompletionBasis.API_EXECUTION if success else CompletionBasis.NOT_VERIFIED
        value = BasicMotionResult(status, reason, start, target, actual, self.clock()-began,
                                  stop, basis.value)
        summary = StageSummary(status.value, reason, basis, success, measurements=(
            ResultMeasurement("elapsed", value.elapsed_s, "s"),
            ResultMeasurement("stop_confirmed", False),
        ))
        return StageResult(stage.stage_id, success, value, None if success else reason, summary)
