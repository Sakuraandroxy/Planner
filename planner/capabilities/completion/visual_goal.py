from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.domain.visual_goal import GoalVisibility, VisualGoalNavigationParameters
from planner.domain.target_surface import SurfaceSelection


class VisualGoalCompletion:
    def __init__(self, limits: VisualGoalLimits):
        self.limits = limits

    def surface_clearance(self, parameters: VisualGoalNavigationParameters, selection: SurfaceSelection) -> bool:
        """All reliable target samples constrain proximity, including unselected regions."""
        desired = parameters.standoff_m if parameters.standoff_m is not None else self.limits.standoff_m
        lower = desired-self.limits.distance_tolerance_m+self.limits.distance_uncertainty_margin_m
        return all(item.geometry.distance_m >= lower for item in selection.estimates
                   if item.geometry.reliable and item.geometry.distance_m is not None)

    def candidate(self, parameters, observation, goal, geometry, initial_pose, clearance_ok, *,
                  surface_selection: SurfaceSelection | None = None):
        limits = self.limits
        if (goal.status is not GoalVisibility.VISIBLE or not observation.stopped or not clearance_ok
                or not geometry.reliable or not goal.same_target):
            return False
        if abs(observation.vehicle_pose.z-initial_pose.z) > limits.maximum_vertical_offset_m:
            return False
        distance = parameters.standoff_m if parameters.standoff_m is not None else limits.standoff_m
        if surface_selection is not None:
            if surface_selection.selected is None or not self.surface_clearance(parameters, surface_selection):
                return False
            # An anchor cannot override fresh depth disagreeing with the arrival band.
            measured = surface_selection.selected.geometry.distance_m
            if not (distance-limits.distance_tolerance_m <= measured-limits.distance_uncertainty_margin_m
                    and measured+limits.distance_uncertainty_margin_m <= distance+limits.distance_tolerance_m):
                return False
        if not (distance-limits.distance_tolerance_m <= geometry.distance_m-limits.distance_uncertainty_margin_m
                and geometry.distance_m+limits.distance_uncertainty_margin_m <= distance+limits.distance_tolerance_m):
            return False
        if parameters.keep_target_in_view:
            a, b, c, d = goal.bbox
            margin = limits.target_center_margin
            if not (margin <= (a+c)/2 <= 1-margin and margin <= (b+d)/2 <= 1-margin):
                return False
        return True
