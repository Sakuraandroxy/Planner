from config import cfg
from agent.functions.common.config_access import function_section

from agent.functions.completion.api_completion import ApiTaskCompletionChecker
from agent.functions.completion.scheduler import CompletionPlanningResult, run_completion_and_planning
from agent.functions.completion.task_completion import CompletionResult, TaskCompletionChecker
from agent.functions.completion.distance_arrival import (
    DistanceArrivalCompletion,
    DistanceArrivalResult,
    build_distance_arrival_completion,
)
from agent.functions.completion.navigation_metrics import NavigationMetricsTracker


def build_task_completion_checker(detector=None, direction_estimator=None):
    """Build the configured task completion checker."""
    tc = {**(cfg.get("TASK_COMPLETION", {}) or {}), **function_section(cfg, "COMPLETION")}
    effective_cfg = dict(cfg)
    effective_cfg["TASK_COMPLETION"] = tc
    name = str(tc.get("NAME", "depth_detector")).strip().lower()
    if not bool(tc.get("ENABLED", True)):
        checker = ApiTaskCompletionChecker(effective_cfg)
        checker.enabled = False
        return checker
    if name in {"api_completion", "vlm_completion", "api"}:
        return ApiTaskCompletionChecker(effective_cfg)
    if name in {"depth_detector", "detector_depth"}:
        return TaskCompletionChecker(effective_cfg, detector=detector, direction_estimator=direction_estimator)
    raise ValueError(f"Unknown TASK_COMPLETION.NAME: {name}")


__all__ = [
    "ApiTaskCompletionChecker",
    "CompletionPlanningResult",
    "CompletionResult",
    "TaskCompletionChecker",
    "build_task_completion_checker",
    "run_completion_and_planning",
    "DistanceArrivalCompletion",
    "DistanceArrivalResult",
    "build_distance_arrival_completion",
    "NavigationMetricsTracker",
]
