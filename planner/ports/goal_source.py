"""连续路径的目标源：每一拍给出一条期望命令，或表示没有目标。"""

from typing import Protocol

from planner.domain import MotionCommand, TimedVehicleState


class StreamGoalSource(Protocol):
    def next(
        self, state: TimedVehicleState, unwrapped_yaw_deg: float
    ) -> MotionCommand | None: ...
