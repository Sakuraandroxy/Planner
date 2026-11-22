"""Semantic check over frozen local evidence; no simulator or HTTP implementation."""
from dataclasses import replace

from planner.domain.visual_goal_completion import VisualGoalCompletionInput
from planner.errors import ProtocolError
from planner.ports.task_completion import TaskCompletionChecker
from planner.runtime.model_calls import ModelCallService

from .visual_goal_request import require_visual_completion_evidence


class VisualGoalVerification:
    def __init__(self, checker: TaskCompletionChecker[VisualGoalCompletionInput], clock):
        self.checker = checker
        self.calls = ModelCallService(clock)

    def check(self, request, budget, timeout_s, retry_count, checkpoint, clock):
        require_visual_completion_evidence(request)
        # The worker sees copies, never a mutable control session or device.
        depth = request.observation.depth_m.copy()
        depth.setflags(write=False)
        depth_image = request.observation.depth_image
        if depth_image is not None:
            depth_image = depth_image.copy()
        observed = replace(request.observation, rgb=request.observation.rgb.copy(),
                           depth_image=depth_image, depth_m=depth)
        frozen = replace(request, observation=observed,
            reference_rgb=request.reference_rgb.copy() if request.reference_rgb is not None else None)
        for attempt in range(retry_count+1):
            timeout = min(timeout_s, budget.remaining(clock()))
            try:
                value = self.calls.call(lambda timeout=timeout: self.checker.check(frozen, timeout_s=timeout), timeout, checkpoint)
                if type(value) is not bool:
                    raise ProtocolError("completion checker must return a boolean")
                return value
            except ProtocolError:
                if attempt >= retry_count:
                    raise
        raise AssertionError("unreachable completion retry state")