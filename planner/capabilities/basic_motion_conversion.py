"""Stateless semantic-axis conversion, shared execution protocol stays NED."""
from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.pose import RelativePoseDelta
from planner.domain.trajectory import RelativeTrajectory


class BasicMotionConverter:
    def create_trajectory(self, parameters: BasicMotionParameters) -> RelativeTrajectory:
        return RelativeTrajectory((RelativePoseDelta(
            parameters.forward_m, parameters.right_m, -parameters.up_m, parameters.yaw_right_deg,
        ),))
