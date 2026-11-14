"""Stateless semantic-axis conversion for a ground vehicle."""
import math

from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.pose import WorldPose, wrap_yaw_deg


class BasicMotionConverter:
    def create_target(
        self, parameters: BasicMotionParameters, start: WorldPose
    ) -> WorldPose:
        angle = math.radians(start.yaw_deg)
        return WorldPose(
            start.x + math.cos(angle) * parameters.forward_m,
            start.y + math.sin(angle) * parameters.forward_m,
            start.z,
            wrap_yaw_deg(start.yaw_deg + parameters.yaw_right_deg),
        )