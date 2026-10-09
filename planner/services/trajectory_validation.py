from __future__ import annotations

import math

from planner.domain.trajectory import RelativeTrajectory, WorldTrajectory
from planner.domain.pose import resolve_yaw_delta_deg
from planner.errors import ProtocolError
'''
轨迹安全与协议校验器，负责在执行前拒绝明显不合法或过大的轨迹
合法轨迹为:模型输出1～5个航点；每个航点必须恰好包含4个有限数值；每一步三维位移不超过30米；每一步相对 yaw 不超过配置上限（当前为±180°）。
'''
class TrajectoryValidator:
    def __init__(self, expected_points: int, max_step_m: float, max_yaw_step_deg: float):
        self.expected_points = expected_points
        self.max_step_m = max_step_m
        self.max_yaw_step_deg = max_yaw_step_deg

    def validate_relative(self, trajectory: RelativeTrajectory) -> None:
        if not 1 <= len(trajectory.points) <= self.expected_points:
            raise ProtocolError(f"expected 1 to {self.expected_points} trajectory points, got {len(trajectory.points)}")
        for index, point in enumerate(trajectory.points):
            distance = math.sqrt(point.dx ** 2 + point.dy ** 2 + point.dz ** 2)
            if distance > self.max_step_m:
                raise ProtocolError(f"relative point {index} exceeds max step distance")
            if abs(point.dyaw_deg) > self.max_yaw_step_deg:
                raise ProtocolError(f"relative point {index} exceeds max yaw step")

    def validate_world(self, trajectory: WorldTrajectory) -> None:
        if not 1 <= len(trajectory.poses) <= self.expected_points:
            raise ProtocolError("world trajectory must contain 1 to the configured maximum points")
        previous = trajectory.start
        for index, pose in enumerate(trajectory.poses):
            distance = math.sqrt((pose.x - previous.x) ** 2 + (pose.y - previous.y) ** 2 + (pose.z - previous.z) ** 2)
            if distance > self.max_step_m + 1e-6:
                raise ProtocolError(f"world segment {index} exceeds max step distance")
            explicit_yaw = None if trajectory.yaw_deltas_deg is None else trajectory.yaw_deltas_deg[index]
            angle = resolve_yaw_delta_deg(previous.yaw_deg, pose.yaw_deg, explicit_yaw)
            if abs(angle) > self.max_yaw_step_deg + 1e-6:
                raise ProtocolError(f"world segment {index} exceeds max yaw step")
            previous = pose

