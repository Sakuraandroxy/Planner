"""Direct numeric motion. Return after measured endpoint hold.

Execution failures raise ExecutionError; the workflow independently checks the
final position/yaw. No image, collision or free-space preflight is required.
An unconfirmed native translation may finish through local fixed-goal control.
Signed/full rotations still require native execution confirmation.
"""
from typing import Protocol
from planner.domain.pose import WorldPose


class BasicMotionVehicle(Protocol):
    def current_pose(self) -> WorldPose: ...
    def execute_basic_motion(self, start: WorldPose, target: WorldPose, yaw_delta_deg: float) -> None: ...
