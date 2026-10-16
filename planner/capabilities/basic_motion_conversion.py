"""Stateless semantic-axis conversion, shared execution protocol stays NED."""
import math

from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.pose import WorldPose, wrap_yaw_deg


class BasicMotionConverter:
    def create_target(self, parameters: BasicMotionParameters, start: WorldPose) -> WorldPose:
        angle = math.radians(start.yaw_deg)
        return WorldPose(
            start.x + math.cos(angle)*parameters.forward_m - math.sin(angle)*parameters.right_m,
            start.y + math.sin(angle)*parameters.forward_m + math.cos(angle)*parameters.right_m,
            start.z - parameters.up_m,
            wrap_yaw_deg(start.yaw_deg + parameters.yaw_right_deg),
        )
