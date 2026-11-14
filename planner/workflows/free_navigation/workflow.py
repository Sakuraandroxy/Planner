"""Drive along the road indefinitely with target-free trajectory planning.

The stage has no numeric magnitude and no static target. Continuous mode (the
default when the sliding window is enabled) keeps a world-waypoint queue fed by
background planning and follows it at the control rate without stopping between
batches. The stopped-batch mode remains as a fallback when the sliding window is
disabled. The loop has no completion condition: it ends only when the user stops
the task (the shared hold checkpoint raises ``KeyboardInterrupt``) or a local
check fails.
"""
from __future__ import annotations

import logging
import math
import time
from functools import partial

from planner.domain.execution import StopResult
from planner.domain.free_navigation import (
    FreeNavigationParameters,
    FreeNavigationResult,
    FreeNavigationStatus,
)
from planner.domain.mission import MissionStage
from planner.domain.motion import ContinuousTrackingSettings, MotionLimits
from planner.domain.result import (
    CompletionBasis,
    ResultMeasurement,
    StageResult,
    StageSummary,
)
from planner.domain.sliding_window import SlidingWindowSettings
from planner.domain.trajectory_telemetry import MODE_STOPPED_BATCH
from planner.errors import (
    BudgetExceeded,
    ExecutionError,
    FreeNavigationStalled,
    ModelCallTimeout,
    ObservationError,
    ProtocolError,
)
from planner.services.execution import ExecutionService
from planner.services.model_calls import ModelCallService
from planner.services.observation_snapshot import freeze_observation
from planner.services.planning_window import PlanningWindow
from planner.services.resource_cleanup import close_resources
from planner.services.trajectory_transform import relative_to_world
from .continuous import FreeNavigationContinuous

logger = logging.getLogger(__name__)


class FreeNavigationWorkflow:
    def __init__(
        self,
        observation_source,
        planner,
        validator,
        motion,
        execution: ExecutionService,
        clock=time.monotonic,
        sleep=time.sleep,
        *,
        model_request_timeout_s: float = 30.0,
        max_consecutive_stalls: int = 2,
        sliding_window: SlidingWindowSettings | None = None,
        motion_limits: MotionLimits | None = None,
        moving_observation_source=None,
        stream_speed_mps: float = 2.0,
        stream_waypoint_timeout_s: float = 60.0,
        collision_backoff_m: float = 3.0,
        maximum_collision_recoveries: int = 3,
        collision_recovery_reset_m: float = 20.0,
        continuous_tracking: ContinuousTrackingSettings = ContinuousTrackingSettings(),
        lane_reference_provider=None,
        telemetry=None,
    ):
        if (
            isinstance(model_request_timeout_s, bool)
            or not isinstance(model_request_timeout_s, (int, float))
            or not math.isfinite(model_request_timeout_s)
            or model_request_timeout_s <= 0
        ):
            raise ValueError(
                "free navigation model timeout must be finite and positive"
            )
        if (
            type(max_consecutive_stalls) is not int
            or max_consecutive_stalls < 1
        ):
            raise ValueError(
                "free navigation stall limit must be a positive integer"
            )
        if (
            isinstance(stream_speed_mps, bool)
            or not isinstance(stream_speed_mps, (int, float))
            or not math.isfinite(stream_speed_mps)
            or stream_speed_mps <= 0
        ):
            raise ValueError(
                "free navigation stream speed must be finite and positive"
            )
        if (
            isinstance(stream_waypoint_timeout_s, bool)
            or not isinstance(stream_waypoint_timeout_s, (int, float))
            or not math.isfinite(stream_waypoint_timeout_s)
            or stream_waypoint_timeout_s <= 0
        ):
            raise ValueError(
                "free navigation stream waypoint timeout must be finite and positive"
            )
        self.source = observation_source
        self.planner = planner
        self.validator = validator
        self.motion = motion
        self.execution = execution
        self.clock = clock
        self.sleep = sleep
        self.model_request_timeout_s = float(model_request_timeout_s)
        self.max_consecutive_stalls = max_consecutive_stalls
        self.telemetry = telemetry
        self.lane_reference_provider = lane_reference_provider
        self.model_calls = ModelCallService(clock)
        self._closed = False
        self.window = None
        self.continuous = None
        if moving_observation_source is not None:
            if sliding_window is None or not sliding_window.enabled:
                raise ProtocolError(
                    "continuous execution requires the enabled planning window"
                )
            if motion_limits is None:
                raise ValueError(
                    "continuous execution requires motion limits"
                )
            self.window = PlanningWindow(
                planner,
                validator,
                sliding_window,
                motion_limits,
                clock,
            )
            self.continuous = FreeNavigationContinuous(
                execution,
                moving_observation_source,
                motion_limits,
                stream_speed_mps,
                clock,
                sleep,
                motion=motion,
                collision_backoff_m=collision_backoff_m,
                maximum_collision_recoveries=maximum_collision_recoveries,
                collision_recovery_reset_m=collision_recovery_reset_m,
                model_request_timeout_s=self.model_request_timeout_s,
                stall_timeout_s=float(stream_waypoint_timeout_s),
                waypoint_timeout_s=float(stream_waypoint_timeout_s),
                maximum_capture_failures=(
                    sliding_window.max_consecutive_plan_failures + 1
                ),
                tracking_settings=continuous_tracking,
                lane_reference_provider=lane_reference_provider,
                telemetry=telemetry,
            )

    def prepare(self) -> None:
        if self._closed:
            raise ExecutionError("free navigation workflow is closed")
        if self.continuous is None:
            self.model_calls.require_available()
            return
        self.window.worker.require_available()
        self.continuous.prepare()

    def has_pending_calls(self) -> bool:
        return (
            self.model_calls.has_pending_call
            or (
                self.window is not None
                and self.window.worker.has_pending_call
            )
            or (
                self.continuous is not None
                and self.continuous.has_pending_calls()
            )
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        callbacks = [self.model_calls.close]
        if self.continuous is not None:
            callbacks.append(self.continuous.close)
        if self.window is not None:
            callbacks.append(self.window.close)
        close_resources(tuple(callbacks))

    def run(self, stage: MissionStage) -> StageResult[FreeNavigationResult]:
        started = self.clock()
        status = FreeNavigationStatus.BLOCKED
        reason = "free navigation did not start"
        stop = StopResult(False)
        planning_calls = 0
        traveled_m = 0.0
        initial = None
        last = None
        final_pose = None
        leased = False
        try:
            self.prepare()
            parameters = stage.parameters
            if not isinstance(parameters, FreeNavigationParameters):
                raise ProtocolError(
                    "free navigation stage has invalid parameters"
                )
            self.execution.acquire_flight(self)
            leased = True
            self.execution.set_deadline(None)
            if self.continuous is not None:
                self.window.begin(
                    deadline_monotonic=None,
                    maximum_planning_rounds=None,
                )
                observation = self._capture_stopped()
                self.continuous.run(
                    parameters, observation, self.window,
                    stage_id=stage.stage_id,
                )
                raise AssertionError(
                    "continuous free navigation returned unexpectedly"
                )
            stalls = 0
            while True:
                observation = self._capture_stopped()
                if initial is None:
                    initial = observation.pose
                if self.telemetry is not None:
                    self.telemetry.publish_no_queue(
                        instruction=parameters.instruction,
                        stage_id=stage.stage_id,
                        pose=observation.pose,
                        mode=MODE_STOPPED_BATCH,
                    )
                planning_calls += 1
                operation = partial(
                    self.planner.plan,
                    freeze_observation(observation),
                    parameters.instruction,
                    round_index=planning_calls,
                    initial_pose=initial,
                    previous_trajectory=(
                        None if last is None else last.actions
                    ),
                    previous_trajectory_start=(
                        None if last is None else last.start
                    ),
                    current_guidance=None,
                    task_context=self._lane_context(observation.pose),
                    budget=None,
                    timeout_s=self.model_request_timeout_s,
                )
                trajectory = self.model_calls.call(
                    operation,
                    self.model_request_timeout_s,
                    self.execution.checkpoint,
                )
                self.execution.checkpoint()
                self.validator.validate_relative(trajectory)
                world = relative_to_world(trajectory, observation.pose)
                self.validator.validate_world(world)
                record = self.execution.execute(
                    self.motion.create_motion(world)
                )
                last = record
                traveled_m += record.translation_m
                if (
                    record.translation_m < 1e-6
                    and record.absolute_yaw_deg < 1e-6
                ):
                    stalls += 1
                    if stalls >= self.max_consecutive_stalls:
                        raise FreeNavigationStalled(
                            "free navigation made no measurable progress"
                        )
                else:
                    stalls = 0
        except KeyboardInterrupt as exc:
            stop = (
                getattr(exc, "stop_result", None)
                or self.execution.stop_if_owner(self)
            )
            raise
        except Exception as exc:
            receipt = getattr(exc, "receipt", None)
            if receipt is not None:
                final_pose = receipt.final_pose
            reason = str(exc) or type(exc).__name__
            if isinstance(exc, (ModelCallTimeout, BudgetExceeded)):
                status = FreeNavigationStatus.TIMEOUT
            elif isinstance(exc, ObservationError):
                status = FreeNavigationStatus.OBSERVATION_ERROR
            elif isinstance(exc, ProtocolError):
                status = FreeNavigationStatus.MODEL_ERROR
            elif isinstance(exc, FreeNavigationStalled):
                status = FreeNavigationStatus.STUCK
            elif isinstance(exc, ExecutionError):
                status = FreeNavigationStatus.EXECUTION_ERROR
            else:
                status = FreeNavigationStatus.BLOCKED
            stop = (
                getattr(exc, "stop_result", None)
                or self.execution.stop_if_owner(self)
            )
            logger.error(
                "[FreeNavigation %s] %s: %s",
                stage.stage_id,
                status.value,
                reason,
            )
        finally:
            if leased:
                if self.window is not None:
                    self.window.end()
                self.execution.release_flight(self, self.has_pending_calls)
        if self.continuous is not None:
            planning_calls = self.continuous.planning_calls
            traveled_m = self.continuous.path_length_m
            if final_pose is None:
                final_pose = self.continuous.final_pose
        elif final_pose is None and last is not None:
            final_pose = last.end
        elapsed_s = self.clock() - started
        value = FreeNavigationResult(
            status,
            reason,
            planning_calls,
            elapsed_s,
            traveled_m,
            stop,
            final_pose,
        )
        measurements = [
            ResultMeasurement("planning_calls", planning_calls),
            ResultMeasurement("elapsed", elapsed_s, "s"),
            ResultMeasurement("traveled", traveled_m, "m"),
            ResultMeasurement("stop_confirmed", stop.confirmed),
            ResultMeasurement(
                "execution_mode",
                (
                    "stopped_segments"
                    if self.continuous is None
                    else "continuous_world_queue"
                ),
            ),
        ]
        summary = StageSummary(
            status.value,
            reason,
            CompletionBasis.NOT_VERIFIED,
            False,
            measurements=tuple(measurements),
            final_pose=final_pose,
        )
        return StageResult(
            stage.stage_id,
            False,
            value,
            f"{status.value}: {reason}",
            summary,
        )

    def _lane_context(self, pose):
        """Measure the HD-map lane prior; never fail the task for it.

        The provider is optional and map projection is best-effort: an
        unavailable lane (junction, overlapping road, adapter error) becomes
        ``None`` and the prompt falls back to its free-space rules.
        """
        if self.lane_reference_provider is None:
            return None
        try:
            return self.lane_reference_provider.lane_reference(pose)
        except Exception as exc:
            logger.warning(
                "[FreeNavigation] lane reference unavailable: %s", exc
            )
            return None

    def _capture_stopped(self):
        try:
            observation = self.source.capture()
        except Exception as exc:
            raise ObservationError(
                f"free navigation observation failed: {exc}"
            ) from exc
        if not observation.stopped:
            raise ObservationError(
                "free navigation requires a stopped observation"
            )
        return observation