"""Optional measured velocity-control port; called only by the flight owner."""
from typing import Protocol

from planner.domain import MotionCommand, TimedVehicleState


class StreamingVehicle(Protocol):
    def stream_state(self) -> TimedVehicleState: ...
    def stream_command(self, command: MotionCommand, duration_s: float) -> None: ...
