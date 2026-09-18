from typing import Protocol

from planner.domain.trajectory import MotionTrajectory, WorldTrajectory


class MotionPlanner(Protocol):
    def create_motion(self, trajectory: WorldTrajectory) -> MotionTrajectory: ...

