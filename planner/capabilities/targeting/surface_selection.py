"""Stateless region selection; correspondence and budgets belong to the caller.

Inputs are typed per-frame estimates and an optional stage-local reference.
Unusable evidence returns a failed selection, not an alternate facade or a flight
command. No requests, threads or independent retry loop are started here.
"""
from dataclasses import replace
import math

from planner.domain.observation import Observation
from planner.domain.pose import WorldPose, wrap_yaw_deg
from planner.domain.target_surface import GoalSurfaceObservation, SurfaceEstimate, SurfaceSelection
from planner.domain.visual_goal import GoalGeometry, GoalReference, GoalVisibility
from planner.domain.visual_goal_limits import VisualGoalLimits
from .association import associate


class SurfaceSelector:
    def __init__(self, limits: VisualGoalLimits):
        self.limits = limits

    def select(self, observation: Observation, goal: GoalSurfaceObservation,
               estimates: tuple[SurfaceEstimate, ...], reference: GoalReference | None,
               initial_pose: WorldPose, standoff_m: float) -> SurfaceSelection:
        """First select by reliability/height/heading, later match the fixed region.

        Camera pixels, frame-local IDs and distance-to-vehicle are not persistent
        identities. Accepted subsequent geometry uses the fixed anchor for progress;
        fresh raw estimates remain available for completion and proximity checks.
        """
        if goal.status is not GoalVisibility.VISIBLE or not goal.same_target:
            return self._failed(estimates, "target object identity is not established")
        if not 1 <= len(estimates) <= self.limits.maximum_surface_candidates:
            return self._failed(estimates, "surface candidate count exceeds configured contract")
        if reference is None and any(item.candidate.matches_reference_region for item in estimates):
            return self._failed(estimates, "region match claimed before a region reference exists")
        pose = observation.vehicle_pose
        def rank(item):
            if item.geometry.world_point is None:
                return (math.inf, math.inf, math.inf, math.inf, item.candidate.point)
            x, y, z = item.geometry.world_point
            heading = math.degrees(math.atan2(y-pose.y, x-pose.x))
            return (-item.geometry.valid_ratio, item.geometry.mad_m or 0.0,
                    abs(z-initial_pose.z), abs(wrap_yaw_deg(heading-pose.yaw_deg)),
                    item.candidate.point)
        evaluated, eligible = [], []
        # Order-independent deduplication: keep the better supported nearby sample.
        for item in sorted(estimates, key=rank):
            geometry, point = item.geometry, item.candidate.point
            reason = geometry.reason
            accepted = geometry.reliable and geometry.world_point is not None
            if accepted and any(math.dist(point, old.candidate.point) <= self.limits.depth_window_fraction/2
                                for old in eligible):
                accepted, reason = False, "candidate duplicates an accepted depth sampling region"
            if accepted:
                # Surface points need not have UAV height, but an extreme roof point
                # cannot support the requested near relation inside this height envelope.
                height_gap = max(0.0, abs(geometry.world_point[2]-initial_pose.z)
                                 - self.limits.maximum_vertical_offset_m)
                if height_gap >= standoff_m-self.limits.distance_tolerance_m:
                    accepted, reason = False, "surface is unsuitable for a level-height approach"
            if accepted:
                accepted, reason = associate(reference, goal.at(item.candidate), geometry,
                    self.limits.association_gate_m,
                    matches_reference_region=item.candidate.matches_reference_region)
            evaluated_item = replace(item, eligible=accepted, reason=reason)
            evaluated.append(evaluated_item)
            if accepted:
                eligible.append(evaluated_item)
        if not eligible:
            return self._failed(tuple(evaluated), "no reliable candidate matches the approach region")
        if reference is not None and len(eligible) != 1:
            return self._failed(tuple(evaluated), "selected region correspondence is ambiguous")
        chosen = min(eligible, key=rank)
        geometry = chosen.geometry
        if reference is not None:
            geometry = replace(geometry, world_point=reference.world_point,
                distance_m=math.dist(reference.world_point, (pose.x, pose.y, pose.z)),
                reason="fresh depth supports selected region; distance uses fixed stage anchor")
        return SurfaceSelection(tuple(evaluated), chosen, goal.at(chosen.candidate), geometry,
                                "selected region matched" if reference else "initial region selected")

    @staticmethod
    def motion_evidence(goal: GoalSurfaceObservation, selection: SurfaceSelection) -> SurfaceEstimate | None:
        """Only an explicit, unique same-region claim can support motion suspicion."""
        matches = [item for item in selection.estimates if item.geometry.reliable
                   and item.candidate.matches_reference_region]
        return matches[0] if goal.same_target and len(matches) == 1 else None

    @staticmethod
    def _failed(estimates: tuple[SurfaceEstimate, ...], reason: str) -> SurfaceSelection:
        return SurfaceSelection(estimates, None, None, GoalGeometry(False, reason), reason)
