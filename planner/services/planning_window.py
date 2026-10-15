"""Task-independent bridge to the owner queue and model-only prefetch.

The bridge receives measured observations/context and executable motion.
It never decides completion or calls the vehicle. The Workflow checks protocol,
request freshness and ownership before execution; no corridor audit is applied.
"""
from dataclasses import dataclass, replace
import math
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

ContextT = TypeVar("ContextT")


@dataclass(frozen=True)
class _BoundPlanner(Generic[ContextT]):
    planner: TrajectoryPlanner[ContextT]
    context: ContextT

    def plan(self, observation, instruction, **execution_context):
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

    def _check(self, checkpoint) -> None:
        checkpoint()
        if not self.active or self.clock() >= self.deadline:
            raise BudgetExceeded("sliding-window stage deadline exhausted")

    def _submit(self, observation, context: ContextT, instruction, planning: PlanningWindowInput,
                on_request: Callable[[], None], checkpoint) -> None:
        self._check(checkpoint)
        self.worker.require_available()
        pending = self.queue.pending(observation.vehicle_pose)
        items = self.queue.items
        raw = observation.depth_meters.copy()
        raw.setflags(write=False)
        frozen = replace(observation, rgb=observation.rgb.copy(),
                         depth_image=observation.depth_image.copy(), depth_meters=raw)
        now = self.clock()
        valid_until = min(self.deadline, observation.captured_monotonic + min(
            self.settings.max_observation_age_s, planning.maximum_observation_age_s))
        history = self.record
        snapshot = PlanningSnapshot(self.background.stage_token, self.background.request_count+1,
            frozen, planning.initial_pose, instruction, None, None, pending,
            tuple(item.waypoint_id for item in items), self.queue.revision,
            items[-1].end if items else observation.vehicle_pose,
            history.actions if history else planning.previous_trajectory,
            history.start if history else planning.previous_trajectory_start,
            now, min(valid_until, now+planning.model_request_timeout_s), valid_until)
        if snapshot.request_deadline <= now:
            raise ProtocolError("planning observation expired before window submission")
        on_request()
        # BackgroundContinuationPlanner captures this immutable model-only
        # wrapper at submit; later context updates cannot alter the request.
        self.background.planner = _BoundPlanner(self.planner, context)
        self.background.submit(snapshot)

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
