from __future__ import annotations
import time
import logging
import math

from planner.domain.execution import ExecutionBudget, ExecutionReceipt, StopResult

from planner.domain.trajectory import MotionTrajectory
from planner.errors import BudgetExceeded, ExecutionError
from planner.ports.vehicle import Vehicle


class ExecutionService:
    def __init__(self, vehicle: Vehicle, clock=time.monotonic):
        self.vehicle = vehicle
        self.clock = clock

    def execute(self, trajectory: MotionTrajectory) -> None:
        self.execute_with_receipt(trajectory)

    def execute_with_receipt(self, trajectory: MotionTrajectory, *, budget: ExecutionBudget | None = None):
        began = self.clock()
        completed = 0
        measured_chords = 0.0
        try:
            previous = self.vehicle.current_pose()
            for segment in trajectory.segments:
                if budget is not None and budget.remaining(self.clock()) <= 0:
                    raise BudgetExceeded("deadline before segment")
                if budget is None:
                    self.vehicle.execute_segment(segment)
                else:
                    self.vehicle.execute_segment(segment, budget=budget)
                completed += 1
                current = self.vehicle.current_pose()
                measured_chords += math.dist(
                    (previous.x, previous.y, previous.z), (current.x, current.y, current.z)
                )
                previous = current
                if self.vehicle.collision_state():
                    raise ExecutionError("AirSim reported a collision")
                if budget is not None and budget.remaining(self.clock()) <= 0:
                    raise BudgetExceeded("deadline after segment")
            return ExecutionReceipt(completed, self.vehicle.current_pose(), self.clock()-began, measured_chords)
        except (Exception, KeyboardInterrupt) as exc:
            stop = self.stop()
            try:
                receipt = ExecutionReceipt(completed, self.vehicle.current_pose(), self.clock()-began, measured_chords)
            except Exception:
                receipt = None
            if isinstance(exc, (ExecutionError, KeyboardInterrupt)):
                exc.receipt, exc.stop_result = receipt, stop
                raise
            wrapped = ExecutionError(str(exc))
            wrapped.receipt, wrapped.stop_result = receipt, stop
            raise wrapped from exc

    def hold(self) -> None:
        self.vehicle.hover()

    def current_pose(self):
        return self.vehicle.current_pose()

    def collision_state(self):
        return self.vehicle.collision_state()

    def stop(self) -> StopResult:
        errors = []
        for action in (self.vehicle.cancel, self.vehicle.hover):
            try:
                action()
            except Exception as exc:
                errors.append(f"{action.__name__}: {exc}")
                logging.getLogger(__name__).error("Stop action failed: %s", exc)
        # Successful API calls alone do not verify physical stabilization.
        return StopResult(False, tuple(errors))

