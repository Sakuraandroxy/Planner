"""Continuous target-free road following over the shared streaming stack.

This is the free-navigation counterpart of ``VisualGoalContinuous``: the same
20 Hz queue follower and background sliding-window planning, with the target,
pause-radius, search and completion branches removed. A native collision event
triggers a bounded reverse, clears the remaining queue and resumes replanning;
the stage itself is unbounded, ending only on a user stop or local failure.
"""
from __future__ import annotations

import logging
import math

from planner.domain.execution import ExecutionBudget
from planner.domain.free_navigation import FreeNavigationParameters
from planner.domain.motion import ContinuousTrackingSettings
from planner.domain.planning_window import PlanningWindowInput
from planner.domain.pose import RelativePoseDelta
from planner.domain.streaming import QueueObservationAnchor
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.trajectory_telemetry import MODE_CONTINUOUS
from planner.errors import (
    ExecutionError,
    FreeNavigationStalled,
    ObservationError,
    ProtocolError,
)
from planner.services.continuous_execution import ContinuousExecution
from planner.services.moving_observations import MovingObservations
from planner.services.trajectory_transform import relative_to_world

logger = logging.getLogger(__name__)


class FreeNavigationContinuous:
    def __init__(
        self,
        execution,
        moving_source,
        motion_limits,
        speed_mps,
        clock,
        sleep,
        *,
        motion=None,
        collision_backoff_m: float = 3.0,
        maximum_collision_recoveries: int = 3,
        collision_recovery_reset_m: float = 20.0,
        model_request_timeout_s: float,
        stall_timeout_s: float,
        waypoint_timeout_s: float = 60.0,
        maximum_capture_failures: int = 3,
        tracking_settings: ContinuousTrackingSettings = ContinuousTrackingSettings(),
        lane_reference_provider=None,
        telemetry=None,
    ):
        for name, value in (
            ("model request timeout", model_request_timeout_s),
            ("stall timeout", stall_timeout_s),
            ("waypoint timeout", waypoint_timeout_s),
            ("collision backoff", collision_backoff_m),
            ("collision recovery reset distance", collision_recovery_reset_m),
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
        if (
            type(maximum_collision_recoveries) is not int
            or maximum_collision_recoveries < 1
        ):
            raise ValueError(
                "continuous collision recovery budget must be a positive integer"
            )
        self.execution = execution
        self.clock, self.sleep = clock, sleep
        self.telemetry = telemetry
        self.motion = motion
        self.lane_reference_provider = lane_reference_provider
        self.collision_backoff_m = float(collision_backoff_m)
        self.maximum_collision_recoveries = maximum_collision_recoveries
        self.collision_recovery_reset_m = float(collision_recovery_reset_m)
        self.collision_recoveries = 0
        self.collision_reset_baseline_m = None
        self.source = moving_source
        self.model_request_timeout_s = float(model_request_timeout_s)
        self.stall_timeout_s = float(stall_timeout_s)
        self.stream = ContinuousExecution(
            execution,
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
        self._closed = False

    def prepare(self) -> None:
        if self._closed:
            raise ExecutionError("free navigation continuous mode is closed")
        if not all(
            callable(getattr(self.execution.vehicle, name, None))
            for name in ("stream_state", "stream_velocity")
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
        self.collision_recoveries = 0
        self.collision_reset_baseline_m = None
        self.final_pose = initial_observation.pose
        self._discard_pretask_contact()
        observation = initial_observation
        self.frames.begin()
        self.stream.begin(
            window.queue, window.background.stage_token
        )
        used_capture = None
        progress_m = self.stream.path_length_m
        progress_at = self.clock()
        try:
            while True:
                tick_started = self.clock()
                if self.execution.vehicle.collision_state():
                    observation = self._recover_from_collision(window)
                    used_capture = None
                    progress_m = self.stream.path_length_m
                    progress_at = self.clock()
                self.stream.sample()
                self._reset_recovery_budget_after_progress()
                window.record = self.stream.record
                frame = self.frames.poll()
                if frame is not None:
                    observation = frame
                result = window.poll_stream(
                    self.stream.state.pose, self.execution.checkpoint
                )
                if result is not None:
                    logger.info(
                        "[FreeNavigation] %s request=%s detail=%s queue=%d",
                        result[0],
                        result[1],
                        result[2],
                        len(window.queue.items),
                    )
                progress_m, progress_at = self._check_progress(
                    window, progress_m, progress_at
                )
                self.stream.tick(None, sample=False)
                if not window.queue.items:
                    # CARLA is synchronous: with no command the world would not
                    # advance, so a moving capture could never complete while
                    # the queue waits for planning. Hovering keeps the car at
                    # its pose and advances exactly one fixed step.
                    self.execution.vehicle.hover()
                self._update_measurements()
                self._publish(window, parameters.instruction, stage_id)
                due = window.stream_due()
                if due and used_capture != observation.capture_id:
                    self._submit(observation, parameters, window)
                    used_capture = observation.capture_id
                if (
                    due
                    and self.frames.job.future is None
                    and not self.frames.job.busy
                ):
                    self.frames.request(self._capture_budget())
                self.sleep(
                    max(
                        0.001,
                        1 / self.stream.limits.control_hz
                        - (self.clock() - tick_started),
                    )
                )
        finally:
            self.frames.discard()
            self._update_measurements()

    def _publish(self, window, instruction, stage_id) -> None:
        if self.telemetry is None:
            return
        pose = (
            self.stream.state.pose
            if self.stream.state is not None
            else self.final_pose
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
        )

    def _check_progress(self, window, progress_m, progress_at):
        current = self.stream.path_length_m
        now = self.clock()
        if not window.queue.items:
            return current, now
        if current > progress_m + 1e-6:
            return current, now
        if now - progress_at >= self.stall_timeout_s:
            raise FreeNavigationStalled(
                "free navigation made no measurable progress"
            )
        return progress_m, progress_at

    def _reset_recovery_budget_after_progress(self) -> None:
        """Clear the consecutive contact count after sustained forward progress.

        Free navigation is unbounded, so the recovery budget must protect
        against being stuck at one obstacle rather than cap the lifetime of the
        task. Once the car has driven ``collision_recovery_reset_m`` past the
        last recovery, a later contact starts a fresh budget.
        """
        if not self.collision_recoveries:
            return
        baseline = self.collision_reset_baseline_m
        if baseline is None:
            return
        progress_m = self.stream.path_length_m - baseline
        if progress_m < self.collision_recovery_reset_m:
            return
        logger.info(
            "[FreeNavigation] %.1f m of progress since the last contact; "
            "collision recovery budget reset",
            progress_m,
        )
        self.collision_recoveries = 0
        self.collision_reset_baseline_m = None

    def _discard_pretask_contact(self) -> None:
        """Drop a contact that was latched before this stage started.

        The reference adapter consumes contacts in ``reset``/``return_home``,
        so a contact cannot leak from one task into the next. ProjectAirSim
        publishes contacts asynchronously, so a teleport or settle contact can
        arrive after that read and still be pending when the stage begins; it
        would otherwise trigger a spurious reverse on the very first tick.
        """
        consume = getattr(self.execution.vehicle, "collision_state", None)
        if not callable(consume) or not consume():
            return
        last = getattr(self.execution.vehicle, "last_collision", None)
        logger.info(
            "[FreeNavigation] discarding a contact latched before the stage "
            "started (object=%s)",
            getattr(last, "object_name", "unknown"),
        )

    def _update_measurements(self) -> None:
        self.path_length_m = self.stream.path_length_m
        if self.stream.record is not None:
            self.final_pose = self.stream.record.end

    def _recover_from_collision(self, window):
        """Back off, drop the authorized queue and let the loop replan.

        Free navigation has no completion condition, so a contact always means
        "still unfinished": reverse along the current heading, clear every
        remaining waypoint and discard speculative planning/captures before the
        next tick requests a fresh observation. Repeated contacts back off
        farther, because a 3 m retreat leaves the camera too close to a static
        obstacle for the model to see an alternative.
        """
        self.collision_recoveries += 1
        if self.collision_recoveries > self.maximum_collision_recoveries:
            raise ExecutionError("collision recovery budget exhausted")
        if self.motion is None:
            raise ExecutionError("collision recovery requires a motion planner")
        backoff_m = self.collision_backoff_m * self.collision_recoveries
        last = getattr(self.execution.vehicle, "last_collision", None)
        logger.warning(
            "[FreeNavigation] collision detected against %s; backing off "
            "%.1f m and clearing the queue (recovery %d/%d)",
            getattr(last, "object_name", "unknown"),
            backoff_m,
            self.collision_recoveries,
            self.maximum_collision_recoveries,
        )
        self.execution.stop()
        window.background.discard()
        self.frames.discard()
        # A capture epoch ends with the collision; stale retry accounting must
        # not make the next planning observation fail immediately.
        self.frames.failures = 0
        self.frames.retry_at = 0.0
        pose = self.stream.state.pose
        backoff = RelativeTrajectory(
            (RelativePoseDelta(dx=-backoff_m),)
        )
        self.execution.execute(
            self.motion.create_motion(relative_to_world(backoff, pose))
        )
        window.queue.clear()
        # Contacts made while still overlapping or reversing out are already
        # handled by this recovery; do not start another one immediately.
        self.execution.vehicle.collision_state()
        self.stream.goal_yaw = None
        self.stream.head_id = None
        self.stream.paused = None
        self.stream.holding = True
        self.stream.follower.reset()
        self.stream.sample()
        # Start a fresh execution-history window at the recovered pose. Passing
        # the collided segment back as the model's previous trajectory anchors
        # replanning to the failed path. Cumulative path length is untouched so
        # telemetry and the stall watchdog keep their meaning.
        self.stream.initial = self.stream.state.pose
        self.stream.actions = []
        self.stream.record = None
        self.collision_reset_baseline_m = self.stream.path_length_m
        deadline = self.clock() + self.model_request_timeout_s
        while self.frames.job.busy:
            if self.clock() >= deadline:
                raise ObservationError(
                    "previous capture did not finish during collision recovery"
                )
            self.sleep(0.01)
        return self.source.capture()

    def _capture_budget(self):
        return ExecutionBudget(
            self.clock() + self.model_request_timeout_s
        )

    def _submit(self, observation, parameters, window) -> bool:
        try:
            anchor = self.stream.timeline.at(
                observation.timestamp_ns,
                window.background.stage_token,
            )
        except ProtocolError:
            # The initial stopped frame predates the first streaming sample.
            if not observation.stopped or window.queue.items:
                return False
            window.queue.require_anchor(
                self.stream.state.pose, observation.pose
            )
            anchor = QueueObservationAnchor(
                window.background.stage_token,
                window.queue.revision,
                observation.timestamp_ns,
                (),
            )
        record = self.stream.record
        planning = PlanningWindowInput(
            self.initial_pose,
            window.settings.max_observation_age_s,
            self.model_request_timeout_s,
            None if record is None else record.actions,
            None if record is None else record.start,
        )

        def on_request():
            self.planning_calls += 1

        # New actions are anchored where the pending queue ends, so the lane
        # prior must be measured at that pose. Measuring it at the older
        # observation pose makes the model apply the already-queued turn a
        # second time (observed as a heading error swinging from -50 to +50).
        pending_items = anchor.items
        anchor_pose = (
            pending_items[-1].end if pending_items else observation.pose
        )
        try:
            return window.submit_stream(
                observation,
                self._lane_context(anchor_pose),
                parameters.instruction,
                planning,
                anchor,
                on_request,
                lambda: self.execution.checkpoint(),
            )
        except ProtocolError:
            return False

    def _lane_context(self, pose):
        """Measure the HD-map lane prior; never fail the task for it."""
        if self.lane_reference_provider is None:
            return None
        try:
            return self.lane_reference_provider.lane_reference(pose)
        except Exception as exc:
            logger.warning(
                "[FreeNavigation] lane reference unavailable: %s", exc
            )
            return None