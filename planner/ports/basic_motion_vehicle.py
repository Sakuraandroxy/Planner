"""Direct numeric motion; API completion does not certify measured arrival."""
from typing import Protocol
from planner.domain.pose import WorldPose


class BasicMotionVehicle(Protocol):
    def current_pose(self) -> WorldPose: ...
    def execute_basic_motion(self, start: WorldPose, target: WorldPose, yaw_delta_deg: float) -> None: ...
