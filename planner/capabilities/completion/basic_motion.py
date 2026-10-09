"""Stateless measured-pose completion; API success is not sufficient."""
import math

from planner.domain.basic_motion import BasicMotionCompletionDecision
from planner.domain.motion import MotionLimits
from planner.domain.observation import Observation
from planner.domain.pose import WorldPose, wrap_yaw_deg


class BasicMotionCompletion:
    def __init__(self, limits: MotionLimits):
        self.limits = limits

    def evaluate(self, target: WorldPose, current: Observation, collision: bool) -> BasicMotionCompletionDecision:
        actual = current.vehicle_pose
        distance = math.dist((target.x, target.y, target.z), (actual.x, actual.y, actual.z))
        angle = abs(wrap_yaw_deg(target.yaw_deg-actual.yaw_deg))
        completed = (not collision and current.stopped and
                     distance <= self.limits.position_tolerance_m and angle <= self.limits.yaw_tolerance_deg)
        reason = ("measured target pose reached and stopped observation confirmed" if completed else
                  "collision" if collision else "stopped state not confirmed" if not current.stopped else
                  "measured position/yaw is outside completion tolerance")
        return BasicMotionCompletionDecision(completed, reason, distance, angle)
