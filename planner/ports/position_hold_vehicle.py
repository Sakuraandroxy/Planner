"""Measured velocity hold. Only the control owner calls this port."""
from typing import Protocol

from planner.domain.streaming import TimedVehicleState


class PositionHoldVehicle(Protocol):
    def stream_state(self) -> TimedVehicleState: ...
    def cancel(self) -> None: ...
    def hold_velocity(self, velocity_world: tuple[float, float, float],
                      yaw_rate_deg_s: float, duration_s: float) -> None: ...
