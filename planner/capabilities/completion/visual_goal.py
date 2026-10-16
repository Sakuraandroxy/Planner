"""Task-specific stopped distance admission; visibility is judged by the model."""
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.capabilities.approach import TargetApproach
from planner.domain.visual_goal import GoalVisibility


class VisualGoalCompletion:
    def __init__(self, limits: VisualGoalLimits, approach: TargetApproach | None = None):
        self.limits = limits
        self.approach = approach if approach is not None else TargetApproach()

    def candidate(self, parameters, observation, goal, geometry, initial_pose, *, radius_m: float, target_memory=None):
        limits = self.limits
        if not observation.stopped or not geometry.reliable:
            return False
        if target_memory is None:
            if goal.status is not GoalVisibility.VISIBLE or not goal.same_target:
                return False
        elif geometry.world_point != target_memory.world_point:
            return False
        if (limits.maximum_vertical_offset_m is not None and
                abs(observation.vehicle_pose.z-initial_pose.z) > limits.maximum_vertical_offset_m):
            return False
        if not self.approach.distance_within_radius(geometry.distance_m, limits.approach_settings(radius_m)):
            return False
        return True
