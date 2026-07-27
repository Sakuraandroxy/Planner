"""Fast-slow controller for asynchronous sliding-window planning.

The controller is deliberately independent from AirSim.  It owns queue state and
background planning futures; the runtime loop supplies images, pose, execution
callbacks and completion callbacks.  This keeps the old closed-loop scripts
safe while giving the new entry point a clean scheduling primitive.
"""

from __future__ import annotations

import math
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence, Any

from agent.functions.trajectory_queue.world_queue import WorldTrajectoryQueue


@dataclass
class PlanningJob:
    future: Future
    plan_pos: Sequence[float]
    plan_yaw_deg: float
    plan_rot_body_to_world: Any = None
    pending_world: list[list[float]] = field(default_factory=list)
    generation: int = 0
    submitted_at: float = field(default_factory=time.perf_counter)


@dataclass(frozen=True)
class ExecutionDecision:
    count: int
    submit_before: bool = False
    submit_after: bool = False
    queue_time_s: float = 0.0
    remaining_time_s: float = 0.0
    reason: str = ""


@dataclass(frozen=True)
class ContinuousPlanningDecision:
    submit: bool
    queue_time_s: float = 0.0
    after_next_time_s: float = 0.0
    reason: str = ""


class FastSlowController:
    """Queue-aware controller for a fast executor and slow Qwen planner."""

    def __init__(self, config=None, queue: Optional[WorldTrajectoryQueue] = None):
        cfg = config or {}
        self.enabled = bool(cfg.get("ENABLED", True))
        self.execute_count = int(cfg.get("EXECUTE_COUNT", 1))
        self.reserve_time_s = float(cfg.get("RESERVE_TIME_S", 4.5))
        self.planning_execute_count = max(1, int(cfg.get("PLANNING_EXECUTE_COUNT", 2)))
        self.completion_interval_s = float(cfg.get("COMPLETION_INTERVAL_S", 1.5))
        self.queue = queue or WorldTrajectoryQueue(
            max_pending=int(cfg.get("MAX_PENDING", 5)),
            execute_count=self.execute_count,
        )
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="fast_slow_planner")
        self._job: PlanningJob | None = None
        self._generation = 0
        self._last_completion_check = 0.0

    @property
    def planning(self) -> bool:
        return self._job is not None and not self._job.future.done()

    @property
    def has_plan_job(self) -> bool:
        """Whether a submitted plan still needs to be polled or discarded."""
        return self._job is not None

    def can_submit_plan(self) -> bool:
        return (
            self.enabled
            and self._job is None
            and len(self.queue.world_waypoints) <= self.queue.max_pending
        )

    def clear(self) -> None:
        self._generation += 1
        if self._job is not None:
            self._job.future.cancel()
        self.queue.clear()
        self._job = None
        self._last_completion_check = 0.0

    def discard_plan(self) -> None:
        """Drop any pending slow-planner result without touching executed history."""
        self._generation += 1
        if self._job is not None:
            self._job.future.cancel()
        self._job = None

    def should_check_completion(self) -> bool:
        now = time.perf_counter()
        if now - self._last_completion_check >= self.completion_interval_s:
            self._last_completion_check = now
            return True
        return False

    def maybe_submit_plan(
        self,
        *,
        current_pos: Sequence[float],
        current_yaw_deg: float,
        current_rot_body_to_world=None,
        plan_fn: Callable[[list[list[float]]], Any],
    ) -> bool:
        """Submit a slow planning job when the pending queue is valid."""
        if not self.can_submit_plan():
            return False

        pending_world = self.queue.pending_world()
        pending = self.queue.pending_for_model(
            current_pos,
            current_yaw_deg,
            current_rot_body_to_world=current_rot_body_to_world,
        )
        future = self.executor.submit(plan_fn, pending)
        self._job = PlanningJob(
            future=future,
            plan_pos=list(current_pos),
            plan_yaw_deg=float(current_yaw_deg),
            plan_rot_body_to_world=current_rot_body_to_world,
            pending_world=pending_world,
            generation=self._generation,
        )
        return True

    def estimate_segment_times(
        self,
        current_pos: Sequence[float],
        velocity: float,
    ) -> list[float]:
        """Estimate sequential waypoint travel times using commanded velocity."""
        speed = max(float(velocity), 1e-6)
        prev = [float(current_pos[0]), float(current_pos[1]), float(current_pos[2])]
        times: list[float] = []
        for waypoint in self.queue.world_waypoints:
            current = [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])]
            distance = math.sqrt(sum((current[i] - prev[i]) ** 2 for i in range(3)))
            times.append(0.0 if distance <= 0.01 else distance / speed)
            prev = current
        return times

    def execution_decision(
        self,
        current_pos: Sequence[float],
        velocity: float,
    ) -> ExecutionDecision:
        """Choose a maximal batch while preserving the configured time buffer."""
        queue_len = len(self.queue.world_waypoints)
        if queue_len <= 0:
            return ExecutionDecision(count=0, reason="queue_empty")

        segment_times = self.estimate_segment_times(current_pos, velocity)
        queue_time = sum(segment_times)

        if self.has_plan_job:
            count = min(self.planning_execute_count, queue_len)
            return ExecutionDecision(
                count=count,
                queue_time_s=queue_time,
                remaining_time_s=sum(segment_times[count:]),
                reason="planner_running",
            )

        max_pending = max(0, int(self.queue.max_pending))
        feasible_count = 0
        feasible_remaining = 0.0
        for count in range(1, queue_len + 1):
            remaining_count = queue_len - count
            remaining_time = sum(segment_times[count:])
            if remaining_count <= max_pending and remaining_time >= self.reserve_time_s:
                feasible_count = count
                feasible_remaining = remaining_time

        if feasible_count > 0:
            return ExecutionDecision(
                count=feasible_count,
                submit_after=True,
                queue_time_s=queue_time,
                remaining_time_s=feasible_remaining,
                reason="reserve_preserved",
            )

        if queue_len > max_pending:
            count = max(1, queue_len - max_pending)
            return ExecutionDecision(
                count=count,
                submit_after=True,
                queue_time_s=queue_time,
                remaining_time_s=sum(segment_times[count:]),
                reason="reduce_to_max_pending",
            )

        return ExecutionDecision(
            count=1,
            submit_before=True,
            queue_time_s=queue_time,
            remaining_time_s=sum(segment_times[1:]),
            reason="insufficient_reserve",
        )

    def continuous_planning_decision(
        self,
        current_pos: Sequence[float],
        velocity: float,
    ) -> ContinuousPlanningDecision:
        """Submit immediately whenever the active queue fits Qwen's pending window."""
        if not self.can_submit_plan():
            return ContinuousPlanningDecision(False, reason="planner_busy_or_pending_too_long")
        segment_times = self.estimate_segment_times(current_pos, velocity)
        queue_time = sum(segment_times)
        after_next = sum(segment_times[1:])
        return ContinuousPlanningDecision(
            True,
            queue_time_s=queue_time,
            after_next_time_s=after_next,
            reason="pending_window_ready",
        )

    def poll_plan(self, select_fn: Callable[[Any], Any] | None = None):
        if self._job is None or not self._job.future.done():
            return None
        job = self._job
        self._job = None
        if job.generation != self._generation:
            return None
        output = job.future.result()
        if output is not None:
            output._planning_wall_s = max(0.0, time.perf_counter() - job.submitted_at)
        waypoint_format = getattr(output, "waypoint_format", "incremental_body")
        if output and waypoint_format != "incremental_body":
            raise ValueError(
                "FastSlowController expects incremental_body planner output; "
                f"got {waypoint_format!r}"
            )
        if output and select_fn is not None:
            selected = select_fn(output)
            if selected is not None:
                output = selected
        self.queue.append_model_output(
            getattr(output, "waypoints", []) if output else [],
            job.plan_pos,
            job.plan_yaw_deg,
            anchor_world_waypoints=job.pending_world,
            anchor_rot_body_to_world=job.plan_rot_body_to_world,
        )
        return output

    def next_execution_waypoints(
        self,
        current_pos: Sequence[float],
        current_yaw_deg: float,
        count: int | None = None,
        current_rot_body_to_world=None,
    ) -> list[list[float]]:
        return self.queue.execution_slice(
            current_pos,
            current_yaw_deg,
            count=count or self.execute_count,
            current_rot_body_to_world=current_rot_body_to_world,
        )

    def mark_executed(self, count: int | None = None) -> None:
        self.queue.mark_executed(count or self.execute_count)

    def shutdown(self) -> None:
        self.executor.shutdown(wait=False, cancel_futures=True)
