"""Task-specific stopped bbox-center completion candidate conditions."""
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.capabilities.approach import TargetApproach
from planner.domain.visual_goal import GoalVisibility


class VisualGoalCompletion:
    def __init__(self, limits: VisualGoalLimits, approach: TargetApproach | None = None):
        self.limits = limits
        self.approach = approach if approach is not None else TargetApproach()

    def candidate(self, parameters, observation, goal, geometry, initial_pose, *, radius_m: float):
        limits = self.limits
        if (goal.status is not GoalVisibility.VISIBLE or not observation.stopped or
                not geometry.reliable or not goal.same_target):
            return False
        if (limits.maximum_vertical_offset_m is not None and
                abs(observation.vehicle_pose.z-initial_pose.z) > limits.maximum_vertical_offset_m):
            return False
        if not self.approach.distance_within_radius(geometry.distance_m, limits.approach_settings(radius_m)):
            return False
        if parameters.keep_target_in_view:
            a, b, c, d = goal.bbox
            margin = limits.target_center_margin
            if not (margin <= (a+c)/2 <= 1-margin and margin <= (b+d)/2 <= 1-margin):
                return False
        return True
