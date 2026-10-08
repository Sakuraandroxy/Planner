from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.domain.visual_goal import GoalVisibility


class VisualGoalCompletion:
    def __init__(self, limits: VisualGoalLimits):
        self.limits = limits

    def candidate(self, parameters, observation, goal, geometry, initial_pose, clearance_ok):
        limits = self.limits
        if (goal.status is not GoalVisibility.VISIBLE or not observation.stopped or not clearance_ok
                or not geometry.reliable or not goal.same_target):
            return False
        if abs(observation.vehicle_pose.z-initial_pose.z) > limits.maximum_vertical_offset_m:
            return False
        distance = parameters.standoff_m if parameters.standoff_m is not None else limits.standoff_m
        if not (distance-limits.distance_tolerance_m <= geometry.distance_m-limits.distance_uncertainty_margin_m
                and geometry.distance_m+limits.distance_uncertainty_margin_m <= distance+limits.distance_tolerance_m):
            return False
        if parameters.keep_target_in_view:
            a, b, c, d = goal.bbox
            margin = limits.target_center_margin
            if not (margin <= (a+c)/2 <= 1-margin and margin <= (b+d)/2 <= 1-margin):
                return False
        return True
