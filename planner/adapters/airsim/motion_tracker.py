"""Track a timed world-frame pose profile without coupling heading to velocity."""
import logging
import math
import time
from dataclasses import replace
from typing import Callable

from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose, wrap_yaw_deg
from planner.domain.trajectory import MotionSegment
from planner.errors import ExecutionError
from planner.services.motion.synchronized import duration_for, sample_segment

logger = logging.getLogger(__name__)


def limit_vector(values, maximum):
    norm = math.sqrt(sum(value*value for value in values))
    scale = min(1.0, maximum / norm) if norm else 1.0
    return tuple(value*scale for value in values)


class AirSimMotionTracker:
    def __init__(self, connection, current_pose: Callable[[], WorldPose], limits: MotionLimits,
                 timeout_s: float, clock=time.monotonic, sleep=time.sleep):
        self.connection = connection
        self.current_pose = current_pose
        self.limits = limits
        self.timeout_s = timeout_s
        self.clock = clock
        self.sleep = sleep

    def execute(self, segment: MotionSegment) -> None:
        import airsim

        limits = self.limits
        speed = segment.target_speed_mps
        if speed is None:
            raise ExecutionError("smooth motion requires a speed limit")
        start = self.current_pose()
        duration = max(segment.duration_s or 0, duration_for(start, segment.end, speed, limits))
        if duration + limits.settle_time_s >= self.timeout_s:
            raise ExecutionError(f"smooth duration {duration:.2f}s exceeds available execution timeout")
        # Re-anchor interpolation to measured position, but keep the original world goal.
        segment = replace(segment, start=start, duration_s=duration)
        logger.info("[Motion] synchronized flight/rotation: duration=%.2fs target=%s", duration, segment.end)
        period = 1 / limits.control_hz
        began = self.clock()
        previous_time = began
        previous_pose = start
        last_velocity = (0.0, 0.0, 0.0)
        last_yaw_rate = 0.0
        settled_since = None
        while True:
            tick = self.clock()
            elapsed = tick - began
            if elapsed >= self.timeout_s:
                raise ExecutionError("synchronized motion timed out before position/yaw settled")
            actual = self.current_pose()
            if self.connection.call("simGetCollisionInfo").has_collided:
                raise ExecutionError("AirSim reported a collision during synchronized motion")
            dt = tick - previous_time
            moved = math.dist((actual.x, actual.y, actual.z), (previous_pose.x, previous_pose.y, previous_pose.z))
            measured_speed = moved/dt if dt > 0 else 0.0
            measured_yaw_rate = abs(wrap_yaw_deg(actual.yaw_deg-previous_pose.yaw_deg))/dt if dt > 0 else 0.0
            goal = segment.end
            distance = math.dist((actual.x, actual.y, actual.z), (goal.x, goal.y, goal.z))
            yaw_error = abs(wrap_yaw_deg(goal.yaw_deg-actual.yaw_deg))
            settled = (elapsed >= duration and distance <= limits.position_tolerance_m
                       and yaw_error <= limits.yaw_tolerance_deg
                       and measured_speed <= limits.stopped_speed_mps
                       and measured_yaw_rate <= limits.stopped_yaw_rate_deg_s)
            settled_since = (tick if settled_since is None else settled_since) if settled else None
            if settled_since is not None and tick-settled_since >= limits.settle_time_s:
                self.connection.call_async_and_wait("hoverAsync")
                final = self.current_pose()
                final_distance = math.dist((final.x, final.y, final.z), (goal.x, goal.y, goal.z))
                final_yaw = abs(wrap_yaw_deg(goal.yaw_deg-final.yaw_deg))
                if final_distance > limits.position_tolerance_m or final_yaw > limits.yaw_tolerance_deg:
                    raise ExecutionError("position/yaw drifted outside tolerance while hovering")
                logger.info("[Motion] reached: position_error=%.3fm yaw_error=%.2fdeg elapsed=%.2fs",
                            final_distance, final_yaw, self.clock()-began)
                return
            reference = sample_segment(segment, elapsed)
            error = (reference.pose.x-actual.x, reference.pose.y-actual.y, reference.pose.z-actual.z)
            desired = limit_vector(tuple(v+limits.position_gain*e for v, e in zip(reference.velocity, error)), speed)
            change = limit_vector(tuple(v-p for v, p in zip(desired, last_velocity)),
                                  limits.max_acceleration_mps2 * period)
            velocity = tuple(p+d for p, d in zip(last_velocity, change))
            yaw_rate = reference.yaw_rate_deg_s + limits.yaw_gain*wrap_yaw_deg(reference.pose.yaw_deg-actual.yaw_deg)
            yaw_rate = max(-limits.max_yaw_rate_deg_s, min(limits.max_yaw_rate_deg_s, yaw_rate))
            yaw_change = limits.max_yaw_acceleration_deg_s2 * period
            yaw_rate = max(last_yaw_rate-yaw_change, min(last_yaw_rate+yaw_change, yaw_rate))
            # World NED velocity + yaw RATE, not body-frame velocity or ForwardOnly.
            # Serialize and drain each short RPC future; no cross-thread pending calls.
            self.connection.call_async_and_wait(
                "moveByVelocityAsync", *velocity, period,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
            )
            last_velocity, last_yaw_rate = velocity, yaw_rate
            previous_time, previous_pose = tick, actual
            self.sleep(max(0.0, period-(self.clock()-tick)))
