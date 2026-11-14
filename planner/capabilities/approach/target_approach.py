"""Reusable measured-distance arrival evaluation, without route authorization."""
import math

from planner.domain.approach import ApproachAssessment, ApproachSettings, ApproachStatus, ApproachTarget
from planner.domain.pose import WorldPose


class TargetApproach:
    @staticmethod
    def distance_within_radius(distance_m: float | None, settings: ApproachSettings) -> bool:
        try:
            valid = (not isinstance(distance_m, bool) and isinstance(distance_m, (int, float))
                     and math.isfinite(distance_m))
        except OverflowError:
            valid = False
        return bool(valid and distance_m >= 0 and distance_m+settings.uncertainty_margin_m <= settings.arrival_radius_m)

    def assess(self, pose: WorldPose, target: ApproachTarget, settings: ApproachSettings) -> ApproachAssessment:
        if not target.reliable or target.position_world is None:
            return ApproachAssessment(ApproachStatus.UNCONFIRMED, None, "target position is unavailable")
        distance = math.dist((pose.x, pose.y, pose.z), target.position_world)
        inside = self.distance_within_radius(distance, settings)
        return ApproachAssessment(ApproachStatus.WITHIN_RADIUS if inside else ApproachStatus.APPROACHING,
            distance, "inside arrival radius; completion is separate" if inside else "outside arrival radius")
