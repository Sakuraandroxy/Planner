"""Semantic check over frozen local evidence; no simulator or HTTP implementation."""
from dataclasses import replace

from planner.errors import ProtocolError
from planner.ports.task_completion import TaskCompletionChecker
from planner.domain.visual_goal_completion import VisualGoalCompletionInput
from .visual_goal_request import require_visual_completion_evidence
from planner.services.model_calls import ModelCallService


class VisualGoalVerification:
    def __init__(self, checker: TaskCompletionChecker[VisualGoalCompletionInput], clock):
        self.checker = checker
        self.calls = ModelCallService(clock)

    def check(self, request, budget, timeout_s, retry_count, checkpoint, clock):
        require_visual_completion_evidence(request)
        # The worker sees copies, never a mutable control session or device.
        depth = request.observation.depth_meters.copy()
        depth.setflags(write=False)
        observed = replace(request.observation, rgb=request.observation.rgb.copy(),
                           depth_image=request.observation.depth_image.copy(), depth_meters=depth)
        frozen = replace(request, observation=observed,
            reference_rgb=request.reference_rgb.copy() if request.reference_rgb is not None else None)
        for attempt in range(retry_count+1):
            timeout = min(timeout_s, budget.remaining(clock()))
            try:
                value = self.calls.call(lambda: self.checker.check(frozen, timeout_s=timeout), timeout, checkpoint)
                if type(value) is not bool:
                    raise ProtocolError("completion checker must return a boolean")
                return value
            except ProtocolError:
                if attempt >= retry_count:
                    raise
        raise AssertionError("unreachable completion retry state")
