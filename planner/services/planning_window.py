"""Task-independent bridge to the owner queue and model-only prefetch.

The bridge receives measured observations/context and executable motion.
It never decides completion or calls the vehicle. The Workflow checks protocol,
request freshness and ownership before execution; no corridor audit is applied.
"""
from dataclasses import dataclass, replace
import math
import logging
import time
from typing import Callable, Generic, TypeVar

from planner.capabilities.sliding_window.planning import BackgroundContinuationPlanner
from planner.capabilities.sliding_window.scheduler import RefillScheduler
from planner.capabilities.sliding_window.worker import SlowPlannerWorker
from planner.domain.execution import ExecutionRecord
from planner.domain.sliding_window import PlanningSnapshot, SlidingWindowSettings
from planner.domain.trajectory import MotionTrajectory, RelativeTrajectory, WorldTrajectory
from planner.domain.planning_window import PlanningWindowInput
from planner.errors import BudgetExceeded, ExecutionError, ModelCallTimeout, ProtocolError
from planner.ports.trajectory_planner import TrajectoryPlanner
from planner.services.trajectory_queue import WorldWaypointQueue
from planner.services.trajectory_validation import TrajectoryValidator
from planner.domain.pose import resolve_yaw_delta_deg
from planner.services.trajectory_transform import world_to_relative

ContextT = TypeVar("ContextT")
logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class _BoundPlanner(Generic[ContextT]):
    planner: TrajectoryPlanner[ContextT]
    context: ContextT
    trace_images: Callable | None = None
    request_id: int = 0
    clock: Callable = time.monotonic

    def plan(self, observation, instruction, **execution_context):
        started = self.clock()
        if self.trace_images is not None:
            try:
                self.trace_images(self.request_id, observation)
            except Exception:
                logger.exception("Planning diagnostic image write failed")
        if execution_context.get('timeout_s') is not None:
            remaining = execution_context['timeout_s']-(self.clock()-started)
            if remaining <= 0:
                raise ModelCallTimeout("request deadline exhausted while recording planning images")
            execution_context['timeout_s'] = remaining
        return self.planner.plan(observation, instruction,
                                 task_context=self.context, **execution_context)


class PlanningWindow(Generic[ContextT]):
    def __init__(self, planner, validator: TrajectoryValidator, settings: SlidingWindowSettings,
                 motion_limits, clock):
        self.planner, self.settings, self.clock = planner, settings, clock
        self.queue = WorldWaypointQueue(motion_limits.position_tolerance_m, motion_limits.yaw_tolerance_deg)
        self.worker = SlowPlannerWorker()
        self.scheduler = RefillScheduler(settings.refill_pending_count, settings.latency_margin_s)
        self.background = BackgroundContinuationPlanner(planner, validator, self.worker, self.scheduler, clock)
        self.deadline = 0.0
        self.prefetched_revision = -1
        self.record: ExecutionRecord | None = None
        self.active = False
        self.last_stream_submit = -float("inf")
        self.stream_retry_at = 0.0

    def begin(self, *, deadline_monotonic: float, maximum_planning_rounds: int) -> None:
        self.queue.check_owner()
        try:
            finite_deadline = (not isinstance(deadline_monotonic, bool) and
                isinstance(deadline_monotonic, (int, float)) and math.isfinite(deadline_monotonic))
        except OverflowError:
            finite_deadline = False
        if not finite_deadline or type(maximum_planning_rounds) is not int or maximum_planning_rounds < 1:
            raise ValueError("window begin requires a finite deadline and positive planning count")
        if deadline_monotonic <= self.clock():
            raise BudgetExceeded("sliding-window stage deadline exhausted before begin")
        if self.active:
            raise ExecutionError("sliding-window stage is already active")
        self.background.begin(maximum_planning_rounds)
        self.queue.clear()
        self.deadline = min(deadline_monotonic, self.clock()+self.settings.max_task_duration_s)
        self.record, self.prefetched_revision, self.active = None, -1, True
        self.last_stream_submit, self.stream_retry_at = -float("inf"), 0.0

    def _check(self, checkpoint) -> None:
        checkpoint()
        if not self.active or self.clock() >= self.deadline:
            raise BudgetExceeded("sliding-window stage deadline exhausted")

    def _submit(self, observation, context: ContextT, instruction, planning: PlanningWindowInput,
                on_request: Callable[[], None], checkpoint, *, frame_anchor=None, trace_images=None) -> None:
        self._check(checkpoint)
        self.worker.require_available()
        items = self.queue.items if frame_anchor is None else frame_anchor.items
        if len(items) > 5:
            raise ProtocolError("planning pending exceeds five points")
        if frame_anchor is not None and (frame_anchor.stage_token != self.background.stage_token or
                                         frame_anchor.queue_revision != self.queue.revision):
            raise ProtocolError("image queue anchor was invalidated")
        pose = observation.vehicle_pose
        if items:
            correction = resolve_yaw_delta_deg(items[0].start.yaw_deg, pose.yaw_deg)
            deltas = (items[0].yaw_delta_deg-correction, *(p.yaw_delta_deg for p in items[1:]))
            pending = world_to_relative(WorldTrajectory(pose, tuple(p.end for p in items), deltas))
        else:
            pending = RelativeTrajectory(())
        raw = observation.depth_meters.copy()
        raw.setflags(write=False)
        frozen = replace(observation, rgb=observation.rgb.copy(),
                         depth_image=observation.depth_image.copy(), depth_meters=raw)
        now = self.clock()
        valid_until = min(self.deadline, observation.captured_monotonic + min(
            self.settings.max_observation_age_s, planning.maximum_observation_age_s))
        # A delayed image must not be paired with actions executed after its
        # exposure. Extend only the last measured prefix at/before that frame,
        # using the frame-time pose for the final fractional control interval.
        history = self.record if frame_anchor is None else frame_anchor.history
        if history is not None and frame_anchor is not None:
            final = world_to_relative(WorldTrajectory(history.end, (pose,),
                (resolve_yaw_delta_deg(history.end.yaw_deg, pose.yaw_deg),)))
            history = ExecutionRecord(history.start, pose,
                RelativeTrajectory(history.actions.points+final.points))
        snapshot = PlanningSnapshot(self.background.stage_token, self.background.request_count+1,
            frozen, planning.initial_pose, instruction, None, None, pending,
            tuple(item.waypoint_id for item in items), self.queue.revision,
            items[-1].end if items else observation.vehicle_pose,
            history.actions if history else (None if frame_anchor is not None else planning.previous_trajectory),
            history.start if history else (None if frame_anchor is not None else planning.previous_trajectory_start),
            now, min(valid_until, now+planning.model_request_timeout_s), valid_until)
        if snapshot.request_deadline <= now:
            raise ProtocolError("planning observation expired before window submission")
        on_request()
        # BackgroundContinuationPlanner captures this immutable model-only
        # wrapper at submit; later context updates cannot alter the request.
        self.background.planner = _BoundPlanner(self.planner, context, trace_images, snapshot.request_id, self.clock)
        self.background.submit(snapshot)

    def stream_due(self):
        self.queue.check_owner()
        return (self.active and self.background.job is None and self.background.candidate is None
                and not self.worker.has_pending_call and len(self.queue.items) <= 5
                and len(self.queue.items)+5 <= self.settings.max_buffer_points
                and self.clock() >= self.stream_retry_at
                and (not self.queue.items or self.clock()-self.last_stream_submit >= self.settings.planning_interval_s))

    def submit_stream(self, observation, context, instruction, planning, frame_anchor, on_request, checkpoint,
                      *, trace_images=None):
        if not self.stream_due():
            return False
        self._submit(observation, context, instruction, planning, on_request, checkpoint,
                     frame_anchor=frame_anchor, trace_images=trace_images)
        self.last_stream_submit = self.clock()
        return True

    def poll_stream(self, measured, checkpoint):
        self._check(checkpoint)
        if self.background.job is not None:
            snapshot, future = self.background.job
            if self.clock() >= snapshot.request_deadline and not future.done():
                self.background.discard()
                raise ModelCallTimeout("stream planning timed out; late result quarantined")
        before_failures = self.background.consecutive_failures
        self.background.poll()
        if self.background.consecutive_failures > before_failures:
            self.stream_retry_at = self.clock()+1.0
        if self.background.consecutive_failures >= self.settings.max_consecutive_plan_failures:
            raise ProtocolError("repeated stream planning failures")
        candidate = self.background.candidate
        if candidate is None:
            return None
        self.background.candidate = None
        snapshot = candidate.snapshot
        try:
            self.queue.append_continuation(candidate.world, snapshot.pending_ids, snapshot.queue_revision,
                                           measured, self.settings.max_buffer_points)
        except (ExecutionError, ProtocolError) as exc:
            self.stream_retry_at = self.clock()+1.0
            return ("planning_discarded", snapshot.request_id, str(exc))
        return ("queue_appended", snapshot.request_id, candidate.world)

    def _wait(self, checkpoint):
        snapshot = self.background.job[0] if self.background.job else None
        try:
            return self.background.wait(lambda: self._check(checkpoint))
        except ExecutionError as exc:
            if (snapshot is not None and self.clock() >= snapshot.request_deadline and
                    self.worker.has_pending_call):
                raise ModelCallTimeout("window planning timed out; late result quarantined") from exc
            raise

    def next_world(self, observation, context: ContextT, instruction, planning: PlanningWindowInput,
                   on_request: Callable[[], None], checkpoint):
        """Collect a continuation at an empty queue; fresh plan if it is stale."""
        if self.queue.items:
            raise ProtocolError("continuation cannot replace unfinished authorized motion")
        candidate = self._wait(checkpoint)
        if candidate is not None:
            snapshot = candidate.snapshot
            try:
                if snapshot.stage_token != self.background.stage_token or not self.queue.matches(snapshot):
                    raise ProtocolError("candidate queue/stage changed")
                if self.clock() >= snapshot.valid_until:
                    raise ProtocolError("continuation observation expired")
                self.queue.require_anchor(observation.vehicle_pose, snapshot.tail)
                self.background.candidate = None
                return candidate.world
            except (ExecutionError, ProtocolError):
                self.background.discard()
        for _ in range(self.settings.max_consecutive_plan_failures):
            self._submit(observation, context, instruction, planning, on_request, checkpoint)
            candidate = self._wait(checkpoint)
            if candidate is not None:
                self.queue.require_anchor(observation.vehicle_pose, candidate.snapshot.tail)
                self.background.candidate = None
                return candidate.world
            if self.background.consecutive_failures >= self.settings.max_consecutive_plan_failures:
                break
        raise ProtocolError("window planning failed within its retry budget")

    def load(self, motion: MotionTrajectory) -> None:
        """Load only an approved prefix; never overwrite a retained queue."""
        endpoints = tuple(segment.end for segment in motion.segments)
        if self.queue.items:
            if endpoints != tuple(item.end for item in self.queue.items):
                raise ProtocolError("approved motion does not match the retained window queue")
            return
        world = WorldTrajectory(motion.segments[0].start, endpoints,
                                tuple(segment.yaw_delta_deg for segment in motion.segments))
        self.queue.load_prefix(world)
        self.record = None

    def prefetch(self, observation, context: ContextT, instruction, planning: PlanningWindowInput,
                 on_request: Callable[[], None], motion, checkpoint) -> None:
        self._check(checkpoint)
        self.background.poll()
        if self.background.consecutive_failures >= self.settings.max_consecutive_plan_failures:
            raise ProtocolError("repeated window planning failures")
        if self.background.job is not None or self.background.candidate is not None:
            return
        if self.prefetched_revision == self.queue.revision:
            return
        duration = sum((segment.duration_s or 0.0) for segment in motion.segments)
        if not self.scheduler.should_refill(len(self.queue.items), duration):
            return
        try:
            self._submit(observation, context, instruction, planning, on_request, checkpoint)
        except ProtocolError:
            # A signed half-turn plus drift may be unrepresentable as pending;
            # keep its approved world goal, rather than flipping its direction.
            return
        self.prefetched_revision = self.queue.revision

    def should_refill(self, motion, checkpoint) -> bool:
        self._check(checkpoint)
        self.background.poll()
        if self.background.consecutive_failures >= self.settings.max_consecutive_plan_failures:
            raise ProtocolError("repeated window planning failures")
        return (self.background.job is None and self.background.candidate is None and
                self.prefetched_revision != self.queue.revision and
                self.scheduler.should_refill(len(self.queue.items),
                    sum((segment.duration_s or 0.0) for segment in motion.segments)))

    def executed(self, before, receipt, remaining: MotionTrajectory) -> None:
        actions = (self.record.actions.points if self.record else ()) + receipt.measured_actions
        start = self.record.start if self.record else before
        self.record = ExecutionRecord(start, receipt.final_pose, RelativeTrajectory(actions), receipt.paused)
        if receipt.paused:
            # Invalidate predictions anchored to the pre-pause remainder.
            # Drain/discard that one HTTP call while hovering, before reuse.
            self.queue.clear()
            if remaining.segments:
                world = WorldTrajectory(remaining.segments[0].start,
                    tuple(segment.end for segment in remaining.segments),
                    tuple(segment.yaw_delta_deg for segment in remaining.segments))
                self.queue.load_prefix(world)
        else:
            self.queue.consume(self.queue.items[0].waypoint_id, receipt.final_pose)

    def pause_for_completion(self, checkpoint) -> None:
        """Drain/discard speculative output while keeping the authorized queue."""
        self._wait(checkpoint)
        self.background.discard()

    def complete(self) -> None:
        self.background.end()
        self.queue.clear()

    def end(self) -> None:
        if self.active:
            self.background.end()
            self.active = False

    def close(self) -> None:
        self.queue.check_owner()
        try:
            self.end()
        finally:
            self.worker.close()
