"""Scheduling policies for task completion and trajectory planning."""

from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Any, Callable, Optional

from agent.completion.task_completion import CompletionResult


@dataclass
class CompletionPlanningResult:
    completion: CompletionResult
    plan_result: Optional[Any] = None
    completion_elapsed: float = 0.0
    planning_elapsed: float = 0.0
    planning_started: bool = False
    mode: str = "sync_preplan"


def run_completion_and_planning(
    mode: str,
    completion_fn: Callable[[], CompletionResult],
    planning_fn: Callable[[], Any],
) -> CompletionPlanningResult:
    """Run completion and planning with a configurable scheduling policy.

    The policy is intentionally independent from the completion implementation:
    API completion and detector/depth completion both work with either mode.
    """
    normalized = (mode or "sync_preplan").strip().lower()
    if normalized in {"sync", "serial", "sync_preplan", "serial_preplan"}:
        return _run_sync_preplan(completion_fn, planning_fn, normalized)
    if normalized in {"parallel", "parallel_preplan"}:
        return _run_parallel_preplan(completion_fn, planning_fn, normalized)
    raise ValueError(f"Unknown TASK_COMPLETION.CALL_MODE: {mode}")


def _run_sync_preplan(
    completion_fn: Callable[[], CompletionResult],
    planning_fn: Callable[[], Any],
    mode: str,
) -> CompletionPlanningResult:
    t0 = time.perf_counter()
    completion = completion_fn()
    completion_elapsed = time.perf_counter() - t0
    if completion.done:
        return CompletionPlanningResult(
            completion=completion,
            completion_elapsed=completion.elapsed or completion_elapsed,
            planning_started=False,
            mode=mode,
        )

    t1 = time.perf_counter()
    plan_result = planning_fn()
    planning_elapsed = time.perf_counter() - t1
    return CompletionPlanningResult(
        completion=completion,
        plan_result=plan_result,
        completion_elapsed=completion.elapsed or completion_elapsed,
        planning_elapsed=planning_elapsed,
        planning_started=True,
        mode=mode,
    )


def _run_parallel_preplan(
    completion_fn: Callable[[], CompletionResult],
    planning_fn: Callable[[], Any],
    mode: str,
) -> CompletionPlanningResult:
    def _timed_completion():
        t = time.perf_counter()
        out = completion_fn()
        return out, time.perf_counter() - t

    def _timed_planning():
        t = time.perf_counter()
        out = planning_fn()
        return out, time.perf_counter() - t

    with ThreadPoolExecutor(max_workers=2) as executor:
        completion_future = executor.submit(_timed_completion)
        planning_future = executor.submit(_timed_planning)
        completion, completion_elapsed = completion_future.result()
        plan_result, planning_elapsed = planning_future.result()

    return CompletionPlanningResult(
        completion=completion,
        plan_result=None if completion.done else plan_result,
        completion_elapsed=completion.elapsed or completion_elapsed,
        planning_elapsed=planning_elapsed,
        planning_started=True,
        mode=mode,
    )
