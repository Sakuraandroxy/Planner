"""Task-specific stopped distance admission; visibility is judged by the model."""
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.capabilities.approach import TargetApproach
from planner.domain.visual_goal import GoalVisibility
from planner.services.target_coordinates import horizontal_distance


class VisualGoalCompletion:
    def __init__(self, limits: VisualGoalLimits, approach: TargetApproach | None = None):
        self.limits = limits
        self.approach = approach if approach is not None else TargetApproach()

    def candidate(self, parameters, observation, goal, geometry, initial_pose, *, radius_m: float, target_state=None):
        limits = self.limits
        if not observation.stopped or not geometry.reliable:
            return False
        if target_state is None:
            if goal.status is not GoalVisibility.VISIBLE or not goal.same_target:
                return False
        elif geometry.world_point not in target_state.known_surfaces:
            return False
        if (limits.maximum_vertical_offset_m is not None and
                abs(observation.pose.z-initial_pose.z) > limits.maximum_vertical_offset_m):
            return False
        if not self.approach.distance_within_radius(horizontal_distance(geometry.world_point,observation.pose), limits.approach_settings(radius_m)):
            return False
        return True