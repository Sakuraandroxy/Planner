from __future__ import annotations

import math

from planner.domain.trajectory import RelativeTrajectory, WorldTrajectory
from planner.errors import ProtocolError


class TrajectoryValidator:
    def __init__(self, expected_points: int, max_step_m: float, max_yaw_step_deg: float):
        self.expected_points = expected_points
        self.max_step_m = max_step_m
        self.max_yaw_step_deg = max_yaw_step_deg

    def validate_relative(self, trajectory: RelativeTrajectory) -> None:
        if len(trajectory.points) != self.expected_points:
            raise ProtocolError(f"expected {self.expected_points} trajectory points, got {len(trajectory.points)}")
        for index, point in enumerate(trajectory.points):
            distance = math.sqrt(point.dx ** 2 + point.dy ** 2 + point.dz ** 2)
            if distance > self.max_step_m:
                raise ProtocolError(f"relative point {index} exceeds max step distance")
            if abs(point.dyaw_deg) > self.max_yaw_step_deg:
                raise ProtocolError(f"relative point {index} exceeds max yaw step")

    def validate_world(self, trajectory: WorldTrajectory) -> None:
        previous = trajectory.start
        for index, pose in enumerate(trajectory.poses):
            distance = math.sqrt((pose.x - previous.x) ** 2 + (pose.y - previous.y) ** 2 + (pose.z - previous.z) ** 2)
            if distance > self.max_step_m + 1e-6:
                raise ProtocolError(f"world segment {index} exceeds max step distance")
            previous = pose

