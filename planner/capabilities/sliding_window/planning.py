"""Owner-side collection of model-only futures; results are never auto-authorized."""
from concurrent.futures import Future
import logging
import threading
import time

from planner.domain.sliding_window import ContinuationCandidate, PlanningOutput, PlanningSnapshot
from planner.errors import ExecutionError, ProtocolError
from planner.ports.trajectory_planner import TrajectoryPlanner
from planner.services.trajectory_transform import relative_to_world
from planner.services.trajectory_validation import TrajectoryValidator
from .scheduler import RefillScheduler
from .worker import SlowPlannerWorker

logger = logging.getLogger(__name__)


def _model_call(planner: TrajectoryPlanner, snapshot: PlanningSnapshot, clock) -> PlanningOutput:
    started = clock()
    remaining = snapshot.request_deadline - started
    if remaining <= 0:
        raise ExecutionError("planning snapshot expired before request started")
    relative = planner.plan(
        snapshot.observation, snapshot.instruction,
        current_guidance=snapshot.guidance, guidance_start=snapshot.guidance_start,
        round_index=snapshot.request_id,
        initial_pose=snapshot.initial_pose, previous_trajectory=snapshot.history,
        previous_trajectory_start=snapshot.history_start,
        pending_trajectory=snapshot.pending, timeout_s=remaining,
    )
    completed_at = clock()
    return PlanningOutput(relative, completed_at - started, completed_at)


class BackgroundContinuationPlanner:
    def __init__(self, planner: TrajectoryPlanner, validator: TrajectoryValidator,
                 worker: SlowPlannerWorker, scheduler: RefillScheduler, clock=time.monotonic):
        self.planner, self.validator = planner, validator
        self.worker, self.scheduler, self.clock = worker, scheduler, clock
        self._owner = threading.get_ident()
        self.stage_token = 0
        self.request_count = 0
        self.request_limit = 0
        self.job: tuple[PlanningSnapshot, Future] | None = None
        self.candidate: ContinuationCandidate | None = None
        self.consecutive_failures = 0

    def check_owner(self) -> None:
        if threading.get_ident() != self._owner:
            raise ExecutionError("only the control owner may collect planning results")

    def begin(self, request_limit: int) -> None:
        self.check_owner()
        self.worker.require_available()
        self.stage_token += 1
        self.request_count, self.request_limit = 0, request_limit
        self.job = self.candidate = None
        self.consecutive_failures = 0

    def submit(self, snapshot: PlanningSnapshot) -> None:
        self.check_owner()
        if self.job is not None or self.candidate is not None:
            raise ExecutionError("a continuation request or candidate is already pending")
        if self.request_count >= self.request_limit:
            raise ExecutionError("sliding-window planning request budget exceeded")
        if snapshot.stage_token != self.stage_token or snapshot.request_id != self.request_count + 1:
            raise ProtocolError("planning snapshot belongs to another stage/request")
        planner, clock = self.planner, self.clock
        # The closure holds immutable request inputs and the HTTP adapter only,
        # never this session, a queue, an observation source or a vehicle.
        future = self.worker.submit(lambda: _model_call(planner, snapshot, clock))
        self.request_count += 1
        self.job = snapshot, future
        logger.info("[SlidingWindow] request=%s pending=%s tail=%s",
                    snapshot.request_id, len(snapshot.pending.points), snapshot.tail)

    def poll(self) -> None:
        self.check_owner()
        if self.job is None or not self.job[1].done():
            return
        snapshot, future = self.job
        self.job = None
        try:
            output = future.result()
            if snapshot.stage_token != self.stage_token:
                return
            if output.completed_at >= snapshot.request_deadline:
                raise ProtocolError("model response arrived after its request deadline")
            if self.clock() >= snapshot.valid_until:
                raise ProtocolError("model response observation expired")
            if len(output.relative.points) != 5:
                raise ProtocolError("continuation must contain exactly five NEW points")
            self.validator.validate_relative(output.relative)
            world = relative_to_world(output.relative, snapshot.tail)
            self.validator.validate_world(world)
            self.candidate = ContinuationCandidate(snapshot, world)
            self.scheduler.record_latency(output.latency_s)
            self.consecutive_failures = 0
            logger.info("[SlidingWindow] candidate=%s generated and validated; awaiting control-owner execution",
                        snapshot.request_id)
        except Exception as exc:
            self.consecutive_failures += 1
            logger.warning("[SlidingWindow] planning failed (%s): %s", self.consecutive_failures, exc)

    def wait(self, checkpoint) -> ContinuationCandidate | None:
        self.check_owner()
        while self.job is not None:
            checkpoint()
            snapshot, future = self.job
            if self.clock() >= snapshot.request_deadline and not future.done():
                self.discard()
                raise ExecutionError("slow request timed out; its late result is quarantined")
            self.poll()
            if self.job is not None:
                # Do not hold queue, RPC or observation locks while waiting.
                threading.Event().wait(0.02)
        checkpoint()
        return self.candidate

    def discard(self) -> None:
        self.check_owner()
        if self.job is not None:
            self.job[1].cancel()
        self.job = self.candidate = None

    def end(self) -> None:
        self.discard()
        self.stage_token += 1
