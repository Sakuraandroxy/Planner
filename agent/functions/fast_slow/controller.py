"""Fast-slow controller for asynchronous sliding-window planning.

The controller is deliberately independent from AirSim.  It owns queue state and
background planning futures; the runtime loop supplies images, pose, execution
callbacks and completion callbacks.  This keeps the old closed-loop scripts
safe while giving the new entry point a clean scheduling primitive.
"""

from __future__ import annotations

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
    submitted_at: float = field(default_factory=time.perf_counter)


class FastSlowController:
    """Queue-aware controller for a fast executor and slow Qwen planner."""

    def __init__(self, config=None, queue: Optional[WorldTrajectoryQueue] = None):
        cfg = config or {}
        self.enabled = bool(cfg.get("ENABLED", True))
        self.low_watermark = int(cfg.get("LOW_WATERMARK", 3))
        self.high_watermark = int(cfg.get("HIGH_WATERMARK", 8))
        self.execute_count = int(cfg.get("EXECUTE_COUNT", 1))
        self.completion_interval_s = float(cfg.get("COMPLETION_INTERVAL_S", 1.5))
        self.queue = queue or WorldTrajectoryQueue(
            max_pending=int(cfg.get("MAX_PENDING", 3)),
            execute_count=self.execute_count,
        )
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="fast_slow_planner")
        self._job: PlanningJob | None = None
        self._last_completion_check = 0.0

    @property
    def planning(self) -> bool:
        return self._job is not None and not self._job.future.done()

    def clear(self) -> None:
        self.queue.clear()
        self._job = None
        self._last_completion_check = 0.0

    def discard_plan(self) -> None:
        """Drop any pending slow-planner result without touching executed history."""
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
        """Submit a slow planning job when the queue is low."""
        if not self.enabled or self.planning:
            return False
        if len(self.queue.world_waypoints) > self.low_watermark:
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
        )
        return True

    def poll_plan(self, select_fn: Callable[[Any], Any] | None = None):
        if self._job is None or not self._job.future.done():
            return None
        job = self._job
        self._job = None
        output = job.future.result()
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
