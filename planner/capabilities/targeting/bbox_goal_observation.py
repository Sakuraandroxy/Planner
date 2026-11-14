"""Optional grounding observer; surface pixels are selected by the model."""
import time

from planner.domain.target_detection import TargetDetectionInput
from planner.domain.visual_goal import GoalObservation, GoalVisibility, GoalReference
from planner.domain.observation import Observation
from planner.domain.execution import ExecutionBudget
from planner.ports.target_detector import TargetDetector
from planner.errors import BudgetExceeded


class BBoxGoalObserver:
    def __init__(self, detector: TargetDetector, timeout_s: float = 20.0, clock=time.monotonic):
        self.detector, self.timeout_s, self.clock = detector, timeout_s, clock

    def observe(self, observation: Observation, target_description: str,
                reference: GoalReference | None = None, *, budget: ExecutionBudget | None = None,
                initial_pose=None, instruction: str | None = None) -> GoalObservation:
        timeout = self.timeout_s if budget is None else min(self.timeout_s, budget.remaining(self.clock()))
        if timeout <= 0:
            raise BudgetExceeded("deadline before target detection")
        request = TargetDetectionInput(observation, instruction or target_description,
            target_description, reference.crop if reference is not None else None)
        detection = self.detector.detect(request, timeout_s=timeout)
        if budget is not None and budget.remaining(self.clock()) <= 0:
            raise BudgetExceeded("late target detection discarded")
        source = "model_reference_surface_points" if reference is not None else "model_surface_points"
        if not detection.found:
            return GoalObservation(GoalVisibility.NOT_VISIBLE, None, None, target_description,
                "detector returned no certain bbox", False, source, surface_points=())
        box = detection.bbox.coordinates
        return GoalObservation(GoalVisibility.VISIBLE, box, None, target_description,
            "model target association and visible surface samples; not persistent physical landmarks",
            True, source, surface_points=detection.surface_points)