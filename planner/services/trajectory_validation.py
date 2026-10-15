from __future__ import annotations

import math

from planner.domain.trajectory import RelativeTrajectory, WorldTrajectory
from planner.domain.pose import WorldPose, resolve_yaw_delta_deg, wrap_yaw_deg
from planner.errors import ExecutionError, ProtocolError


def require_execution_anchor(measured: WorldPose, expected: WorldPose,
                             position_tolerance_m: float, yaw_tolerance_deg: float) -> None:
    distance = math.dist((measured.x, measured.y, measured.z), (expected.x, expected.y, expected.z))
    angle = abs(wrap_yaw_deg(measured.yaw_deg - expected.yaw_deg))
    if distance > position_tolerance_m + 1e-6 or angle > yaw_tolerance_deg + 1e-6:
        raise ExecutionError("continuation anchor drifted outside position/yaw tolerance")
'''
轨迹安全与协议校验器，负责在执行前拒绝明显不合法或过大的轨迹
执行轨迹接受1～配置上限个航点；模型客户端另行要求恰好5个新增点。每个航点包含4个有限数值；位移和相对 yaw 不超过各自配置上限。
'''
class TrajectoryValidator:
    def __init__(self, expected_points: int, max_step_m: float, max_yaw_step_deg: float):
        if type(expected_points) is not int or not 1 <= expected_points <= 5:
            raise ValueError("expected_points must be an integer between 1 and 5")
        for name, value in (("max_step_m", max_step_m), ("max_yaw_step_deg", max_yaw_step_deg)):
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be finite and positive")
        if max_yaw_step_deg > 180:
            raise ValueError("max_yaw_step_deg must not exceed 180")
        self.expected_points = expected_points
        self.max_step_m = max_step_m
        self.max_yaw_step_deg = max_yaw_step_deg

    def validate_relative(self, trajectory: RelativeTrajectory) -> None:
        if not 1 <= len(trajectory.points) <= self.expected_points:
            raise ProtocolError(f"expected 1 to {self.expected_points} trajectory points, got {len(trajectory.points)}")
        for index, point in enumerate(trajectory.points):
            distance = math.hypot(point.dx, point.dy, point.dz)
            if distance > self.max_step_m:
                raise ProtocolError(f"relative point {index} exceeds max step distance")
            if abs(point.dyaw_deg) > self.max_yaw_step_deg:
                raise ProtocolError(f"relative point {index} exceeds max yaw step")

    def validate_world(self, trajectory: WorldTrajectory) -> None:
        if not 1 <= len(trajectory.poses) <= self.expected_points:
            raise ProtocolError("world trajectory must contain 1 to the configured maximum points")
        previous = trajectory.start
        for index, pose in enumerate(trajectory.poses):
            distance = math.dist((pose.x, pose.y, pose.z), (previous.x, previous.y, previous.z))
            if distance > self.max_step_m + 1e-6:
                raise ProtocolError(f"world segment {index} exceeds max step distance")
            explicit_yaw = None if trajectory.yaw_deltas_deg is None else trajectory.yaw_deltas_deg[index]
            angle = resolve_yaw_delta_deg(previous.yaw_deg, pose.yaw_deg, explicit_yaw)
            if abs(angle) > self.max_yaw_step_deg + 1e-6:
                raise ProtocolError(f"world segment {index} exceeds max yaw step")
            previous = pose

