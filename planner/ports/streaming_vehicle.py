"""Optional measured velocity-control port; called only by the flight owner."""
from typing import Protocol

from planner.domain.streaming import TimedVehicleState


class StreamingVehicle(Protocol):
    def stream_state(self) -> TimedVehicleState: ...
    def stream_velocity(self, velocity_world: tuple[float, float, float],
                        yaw_rate_deg_s: float, duration_s: float) -> None: ...

