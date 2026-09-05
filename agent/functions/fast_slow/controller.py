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
        self.planning_latency_initial_s = max(
            0.0,
            float(cfg.get("PLANNING_LATENCY_INITIAL_S", 5.5)),
        )
        self.planning_latency_margin_s = max(
            0.0,
            float(cfg.get("PLANNING_LATENCY_MARGIN_S", 1.5)),
        )
        self.planning_latency_ema_alpha = min(
            1.0,
            max(0.01, float(cfg.get("PLANNING_LATENCY_EMA_ALPHA", 0.25))),
        )
        self._planning_latency_ema_s = self.planning_latency_initial_s
        self.queue_replenish_trigger_s = max(
            0.0,
            float(cfg.get("QUEUE_REPLENISH_TRIGGER_S", 1.0)),
        )
        self.min_planning_horizon_m = max(
            0.0,
            float(cfg.get("MIN_PLANNING_HORIZON_M", 14.0)),
        )
        self.max_planning_horizon_m = max(
            self.min_planning_horizon_m,
            float(cfg.get("MAX_PLANNING_HORIZON_M", 24.0)),
        )
        self.stale_plan_rebase_distance_m = max(
            0.0,
            float(cfg.get("STALE_PLAN_REBASE_DISTANCE_M", 2.5)),
        )
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

    def required_planning_reserve_s(self) -> float:
        """Return the queue time that should remain while Qwen is running."""
        return max(
            self.reserve_time_s,
            self._planning_latency_ema_s + self.planning_latency_margin_s,
        )

    def required_planning_horizon_m(self, velocity_mps: float) -> float:
        """Convert the dynamic reserve into a continuous flight horizon."""
        return max(0.0, float(velocity_mps)) * self.required_planning_reserve_s()

    def planning_horizon_m(self, velocity_mps: float) -> float:
        """Return the path length requested from the slow planner.

        The reserve is driven by measured Qwen latency.  The explicit bounds
        prevent a very short first leg while also avoiding an unbounded path
        when the service has a transiently high latency.
        """
        required = self.required_planning_horizon_m(velocity_mps)
        return min(
            self.max_planning_horizon_m,
            max(self.min_planning_horizon_m, required),
        )

    def record_planning_latency(self, elapsed_s: float) -> None:
        elapsed = max(0.0, float(elapsed_s))
        alpha = self.planning_latency_ema_alpha
        self._planning_latency_ema_s = (
            (1.0 - alpha) * self._planning_latency_ema_s + alpha * elapsed
        )

    @property
    def planning(self) -> bool:
        return self._job is not None and not self._job.future.done()

    @property
    def has_plan_job(self) -> bool:
        """Whether a submitted plan still needs to be polled or discarded."""
        return self._job is not None

    def can_submit_plan(
        self,
        *,
        current_pos: Sequence[float] | None = None,
        velocity_mps: float | None = None,
    ) -> bool:
        if not self.enabled or self._job is not None:
            return False
        if len(self.queue.world_waypoints) == 0 or len(self.queue.world_waypoints) < self.queue.max_pending:
            return True
        # A full point-count window can still be too short for Qwen's measured
        # latency.  Permit an early replenishment when its time horizon is low.
        if current_pos is None or velocity_mps is None:
            return False
        queue_time = sum(self.estimate_segment_times(current_pos, velocity_mps))
        return queue_time <= self.required_planning_reserve_s() + self.queue_replenish_trigger_s

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
        velocity_mps: float | None = None,
        plan_fn: Callable[[list[list[float]]], Any],
    ) -> bool:
        """Submit a slow planning job when the pending queue is valid."""
        if not self.can_submit_plan(current_pos=current_pos, velocity_mps=velocity_mps):
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
            if remaining_count <= max_pending and remaining_time >= self.required_planning_reserve_s():
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
        if not self.can_submit_plan(current_pos=current_pos, velocity_mps=velocity):
            return ContinuousPlanningDecision(False, reason="planner_busy_or_pending_too_long")
        segment_times = self.estimate_segment_times(current_pos, velocity)
        queue_time = sum(segment_times)
        after_next = sum(segment_times[1:])
        reason = (
            "time_horizon_low"
            if queue_time <= self.required_planning_reserve_s() + self.queue_replenish_trigger_s
            else "pending_window_ready"
        )
        return ContinuousPlanningDecision(True, queue_time_s=queue_time, after_next_time_s=after_next, reason=reason)

    def poll_plan(
        self,
        select_fn: Callable[[Any], Any] | None = None,
        *,
        current_pos: Sequence[float] | None = None,
        current_yaw_deg: float | None = None,
    ):
        if self._job is None or not self._job.future.done():
            return None
        job = self._job
        self._job = None
        if job.generation != self._generation:
            return None
        elapsed = max(0.0, time.perf_counter() - job.submitted_at)
        self.record_planning_latency(elapsed)
        try:
            output = job.future.result()
        except Exception as exc:
            # A slow-planner/network failure must not terminate the AirSim
            # control loop.  The current queue remains executable and the
            # scheduler can submit a fresh request once the reserve is low.
            print(
                "  [PlanError] slow planner result unavailable; "
                f"queue kept error={type(exc).__name__}: {exc}"
            )
            return None
        if output is not None:
            output._planning_wall_s = elapsed
        waypoint_format = getattr(output, "waypoint_format", "incremental_body")
        if output and waypoint_format != "incremental_body":
            raise ValueError(
                "FastSlowController expects incremental_body planner output; "
                f"got {waypoint_format!r}"
            )
        if output and select_fn is not None:
            try:
                selected = select_fn(output)
            except Exception as exc:
                print(
                    "  [PlanError] planner output rejected by local guards; "
                    f"queue kept error={type(exc).__name__}: {exc}"
                )
                return None
            if selected is not None:
                output = selected
        if output and current_pos is not None and current_yaw_deg is not None:
            output = self._rebase_stale_output(output, job, current_pos, current_yaw_deg)
        rebased = bool(getattr(output, "_stale_plan_rebased", False)) if output else False
        if rebased:
            # The returned increments were generated after the frozen pending
            # prefix, but the UAV has already drifted past that anchor.  The
            # old queue suffix is no longer a valid predecessor for the new
            # live-pose suffix; retaining it would append a path that goes
            # forward and then back toward the UAV.  Replace the stale queue
            # atomically before appending the rebased output.
            self.queue.clear()
        self.queue.append_model_output(
            getattr(output, "waypoints", []) if output else [],
            list(current_pos) if rebased and current_pos is not None else job.plan_pos,
            float(current_yaw_deg) if rebased and current_yaw_deg is not None else job.plan_yaw_deg,
            anchor_world_waypoints=[] if rebased else job.pending_world,
            anchor_rot_body_to_world=None if rebased else job.plan_rot_body_to_world,
        )
        return output

    def _rebase_stale_output(
        self,
        output: Any,
        job: PlanningJob,
        current_pos: Sequence[float],
        current_yaw_deg: float,
    ) -> Any:
        """Rebase only the newly planned suffix when the old anchor was passed.

        Qwen describes its increments after the pending prefix seen at submit
        time.  If AirSim has already passed that prefix while Qwen was running,
        appending the suffix at the old anchor creates points behind the UAV.
        The suffix keeps its shape but starts at the live pose instead.
        """
        threshold = self.stale_plan_rebase_distance_m
        if threshold <= 0.0 or not getattr(output, "waypoints", None):
            return output
        moved = math.sqrt(
            sum((float(current_pos[i]) - float(job.plan_pos[i])) ** 2 for i in range(3))
        )
        if moved <= threshold:
            return output
        anchor = list(job.pending_world or [])
        if anchor:
            endpoint = anchor[-1]
            endpoint_gap = math.sqrt(
                sum(
                    (float(current_pos[index]) - float(endpoint[index])) ** 2
                    for index in range(3)
                )
            )
            if endpoint_gap <= threshold:
                previous = anchor[-2] if len(anchor) >= 2 else job.plan_pos
                segment = [
                    float(endpoint[index]) - float(previous[index])
                    for index in range(3)
                ]
                segment_sq = sum(value * value for value in segment)
                if segment_sq <= 1e-9:
                    return output
                from_previous = [
                    float(current_pos[index]) - float(previous[index])
                    for index in range(3)
                ]
                projection = sum(
                    from_previous[index] * segment[index]
                    for index in range(3)
                ) / segment_sq
                closest = [
                    float(previous[index]) + projection * segment[index]
                    for index in range(3)
                ]
                cross_track = math.sqrt(
                    sum(
                        (float(current_pos[index]) - closest[index]) ** 2
                        for index in range(3)
                    )
                )
                if projection <= 1.0 or cross_track > threshold:
                    return output
        output._stale_plan_rebased = True
        output._stale_plan_rebase_from = list(job.plan_pos)
        output._stale_plan_rebase_to = list(current_pos)
        output._stale_plan_rebase_yaw_deg = float(current_yaw_deg)
        output._stale_plan_rebase_reason = f"pose_drift_{moved:.1f}m"
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
