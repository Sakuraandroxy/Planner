from config import cfg

from agent.completion.api_completion import ApiTaskCompletionChecker
from agent.completion.scheduler import CompletionPlanningResult, run_completion_and_planning
from agent.completion.task_completion import CompletionResult, TaskCompletionChecker


def build_task_completion_checker(detector=None, direction_estimator=None):
    """Build the configured task completion checker."""
    tc = cfg.get("TASK_COMPLETION", {})
    name = str(tc.get("NAME", "api_completion")).strip().lower()
    if not bool(tc.get("ENABLED", True)):
        # Keep a lightweight instance so the main loop can still ask for the
        # base capture profile without special cases.
        checker = ApiTaskCompletionChecker(cfg)
        checker.enabled = False
        return checker
    if name in {"api_completion", "vlm_completion", "api"}:
        return ApiTaskCompletionChecker(cfg)
    if name in {"depth_detector", "detector_depth", "legacy"}:
        return TaskCompletionChecker(cfg, detector=detector, direction_estimator=direction_estimator)
    raise ValueError(f"Unknown TASK_COMPLETION.NAME: {name}")


__all__ = [
    "ApiTaskCompletionChecker",
    "CompletionPlanningResult",
    "CompletionResult",
    "TaskCompletionChecker",
    "build_task_completion_checker",
    "run_completion_and_planning",
]
