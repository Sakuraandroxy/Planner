from __future__ import annotations

from planner.domain.trajectory import MotionTrajectory
from planner.errors import ExecutionError
from planner.ports.vehicle import Vehicle


class ExecutionService:
    def __init__(self, vehicle: Vehicle):
        self.vehicle = vehicle

    def execute(self, trajectory: MotionTrajectory) -> None:
        try:
            for segment in trajectory.segments:
                self.vehicle.execute_segment(segment)
                if self.vehicle.collision_state():
                    raise ExecutionError("AirSim reported a collision")
        except (Exception, KeyboardInterrupt) as exc:
            self.vehicle.cancel()
            self.vehicle.hover()
            if isinstance(exc, (ExecutionError, KeyboardInterrupt)):
                raise
            raise ExecutionError(str(exc)) from exc

    def hold(self) -> None:
        self.vehicle.hover()

