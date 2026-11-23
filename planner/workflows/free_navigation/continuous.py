"""Continuous target-free road following over the shared streaming stack.

This is the free-navigation counterpart of ``VisualGoalContinuous``: the same
20 Hz queue follower and background sliding-window planning, with the target,
pause-radius, search and completion branches removed. Collision backoff happens
in the control loop, which stops, reverses and clears the queue before the next
tick replans; the stage itself is unbounded, ending only on a user stop or local
failure.

取帧、poll_stream、补点门控与节拍由 ``workflows.common.streaming.StreamRunner``
编排，本模块只提供判定回调。
"""
from __future__ import annotations

import logging
import math

from planner.domain import FreeNavigationParameters
from planner.domain import ContinuousTrackingSettings
from planner.domain import PlanningWindowInput
from planner.domain import QueueObservationAnchor
from planner.runtime import MODE_CONTINUOUS
from planner.errors import (
    ExecutionError,
    FreeNavigationStalled,
    ObservationError,
    ProtocolError,
)
from planner.runtime import ControlLoop
from planner.runtime import QueueFollower
from planner.runtime import MovingObservations
from planner.workflows.common.streaming import StreamPolicy, StreamRunner

logger = logging.getLogger(__name__)


class FreeNavigationContinuous:
    def __init__(
        self,
        execution,
        loop: ControlLoop,
        moving_source,
        motion_limits,
        speed_mps,
        clock,
        sleep,
        *,
        model_request_timeout_s: float,
        stall_timeout_s: float,
        waypoint_timeout_s: float = 60.0,
        maximum_capture_failures: int = 3,
        tracking_settings: ContinuousTrackingSettings = ContinuousTrackingSettings(),
        telemetry=None,
        vertical_motion: bool = True,
    ):
        for name, value in (
            ("model request timeout", model_request_timeout_s),
            ("stall timeout", stall_timeout_s),
            ("waypoint timeout", waypoint_timeout_s),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"continuous {name} must be finite and positive")
        if (
            type(maximum_capture_failures) is not int
            or maximum_capture_failures < 1
        ):
            raise ValueError(
                "continuous capture failure budget must be a positive integer"
            )
        self.execution = execution
        self.loop = loop
        self.clock, self.sleep = clock, sleep
        self.telemetry = telemetry
        self.vertical_motion = bool(vertical_motion)
        self.source = moving_source
        self.model_request_timeout_s = float(model_request_timeout_s)
        self.stall_timeout_s = float(stall_timeout_s)
        self.follower = QueueFollower(
            execution.vehicle,
            motion_limits,
            speed_mps,
            clock,
            waypoint_timeout_s,
            settings=tracking_settings,
        )
        self.frames = MovingObservations(
            moving_source,
            clock,
            self.model_request_timeout_s,
            maximum_capture_failures,
        )
        self.initial_pose = None
        self.planning_calls = 0
        self.path_length_m = 0.0
        self.final_pose = None
        self.progress_m = 0.0
        self.progress_at = 0.0
        self._closed = False

    def prepare(self) -> None:
        if self._closed:
            raise ExecutionError("free navigation continuous mode is closed")
        if not all(
            callable(getattr(self.execution.vehicle, name, None))
            for name in ("stream_state", "stream_command")
        ):
            raise ExecutionError(
                "continuous execution requires streaming vehicle capability"
            )
        if self.frames.job.busy:
            raise ExecutionError(
                "previous capture request remains quarantined"
            )

    def has_pending_calls(self) -> bool:
        return self.frames.job.busy

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.frames.close()

    def run(
        self, parameters, initial_observation, window, stage_id: str | None = None
    ) -> None:
        if not isinstance(parameters, FreeNavigationParameters):
            raise ProtocolError(
                "free navigation stage has invalid parameters"
            )
        if not initial_observation.stopped:
            raise ObservationError(
                "free navigation requires a stopped initial observation"
            )
        self.initial_pose = initial_observation.pose
        self.planning_calls = 0
        self.path_length_m = 0.0
        self.final_pose = initial_observation.pose
        self.frames.begin()
        self.loop.recorder.begin(
            window.queue, window.background.stage_token
        )
        self.follower.begin(window.queue)
        self.progress_m = self.loop.recorder.path_length_m
        self.progress_at = self.clock()
        runner = StreamRunner(
            self.loop,
            self.follower,
            self.frames,
            self.execution,
            clock=self.clock,
            sleep=self.sleep,
            capture_timeout_s=self.model_request_timeout_s,
            policy=StreamPolicy(
                authorized=self._log_authorized,
                progress=self._progress,
                idle=self._idle,
                measure=lambda window: self._measure(
                    window, parameters.instruction, stage_id
                ),
                submit=lambda observation, window: self._submit(
                    observation, parameters, window
                ),
            ),
        )
        try:
            runner.run(window, initial_observation)
        finally:
            self.frames.discard()
            self._update_measurements()

    def _publish(self, window, instruction, stage_id) -> None:
        if self.telemetry is None:
            return
        pose = (
            self.loop.state.pose if self.loop.state is not None else self.final_pose
        )
        if pose is None:
            return
        self.telemetry.publish_queue(
            instruction=instruction,
            stage_id=stage_id,
            mode=MODE_CONTINUOUS,
            capacity=getattr(window.settings, "max_buffer_points", 0),
            pose=pose,
            items=window.queue.items,
            planning_calls=self.planning_calls,
            path_length_m=self.path_length_m,
            vertical_motion=self.vertical_motion,
        )

    def _log_authorized(self, result, window) -> None:
        logger.info(
            "[FreeNavigation] %s request=%s detail=%s queue=%d",
            result[0],
            result[1],
            result[2],
            len(window.queue.items),
        )

    def _progress(self, window) -> None:
        current = self.loop.recorder.path_length_m
        now = self.clock()
        if not window.queue.items:
            self.progress_m, self.progress_at = current, now
            return
        if current > self.progress_m + 1e-6:
            self.progress_m, self.progress_at = current, now
            return
        if now - self.progress_at >= self.stall_timeout_s:
            raise FreeNavigationStalled(
                "free navigation made no measurable progress"
            )

    def _idle(self, window) -> None:
        # CARLA is synchronous: with no command the world would not advance, so
        # a moving capture could never complete while the queue waits for
        # planning. Hovering keeps the car at its pose and advances one step.
        self.execution.vehicle.hover()

    def _measure(self, window, instruction, stage_id) -> None:
        self._update_measurements()
        self._publish(window, instruction, stage_id)

    def _update_measurements(self) -> None:
        self.path_length_m = self.loop.recorder.path_length_m
        if self.loop.recorder.record is not None:
            self.final_pose = self.loop.recorder.record.end

    def _submit(self, observation, parameters, window) -> bool:
        try:
            anchor = self.loop.recorder.timeline.at(
                observation.timestamp_ns,
                window.background.stage_token,
            )
        except ProtocolError:
            # The initial stopped frame predates the first streaming sample.
            if not observation.stopped or window.queue.items:
                return False
            window.queue.require_anchor(
                self.loop.state.pose, observation.pose
            )
            anchor = QueueObservationAnchor(
                window.background.stage_token,
                window.queue.revision,
                observation.timestamp_ns,
                (),
            )
        record = self.loop.recorder.record
        planning = PlanningWindowInput(
            self.initial_pose,
            window.settings.max_observation_age_s,
            self.model_request_timeout_s,
            None if record is None else record.actions,
            None if record is None else record.start,
        )

        def on_request():
            self.planning_calls += 1

        try:
            return window.submit_stream(
                observation,
                None,
                parameters.instruction,
                planning,
                anchor,
                on_request,
                lambda: self.execution.checkpoint(),
            )
        except ProtocolError:
            return False
