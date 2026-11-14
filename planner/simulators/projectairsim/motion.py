"""Track a timed world-frame pose profile with physical vehicle commands.

Mirrors ``CarlaMotionTracker`` step for step: the same reference sampling,
speed/yaw-rate feedback, acceleration limits, budget checks, pose checkpoints
and settle verification. The only difference is actuation: instead of
integrating the control velocity into a kinematic ``set_transform``, each
control period sends the vehicle's native command (``SetRoverControls`` for
the rover, ``MoveByVelocity`` for the drone) and advances the paused simulator
by exactly one period, so physics (including contacts) stays in the loop.
"""

from __future__ import annotations

import math
import time
from dataclasses import replace
from typing import Callable

from ...domain.execution import ExecutionBudget, PoseCheckpoint
from ...domain.motion import MotionLimits
from ...domain.pose import WorldPose, resolve_yaw_delta_deg, wrap_yaw_deg
from ...domain.trajectory import MotionSegment
from ...errors import BudgetExceeded, ExecutionError, MotionPaused
from ...services.motion.synchronized import duration_for, sample_segment

#: ``drive(velocity_world, yaw_rate_deg_s, measured_velocity_world,
#: yaw_deg, period_s)``
DriveCallback = Callable[
    [tuple[float, float, float], float, tuple[float, float, float], float, float],
    None,
]


def limit_vector(values, maximum):
    if not math.isfinite(maximum) or maximum <= 0 or not all(
        math.isfinite(v) for v in values
    ):
        raise ExecutionError("non-finite motion vector or invalid limit")
    norm = math.hypot(*values)
    if not math.isfinite(norm):
        raise ExecutionError("motion vector magnitude overflowed")
    scale = min(1.0, maximum / norm) if norm else 1.0
    return tuple(value * scale for value in values)


class SteppedMotionTracker:
    def __init__(
        self,
        drive: DriveCallback,
        current_pose: Callable[[], WorldPose],
        limits: MotionLimits,
        timeout_s: float,
        clock=time.monotonic,
        cancelled: Callable[[], bool] | None = None,
    ):
        self.drive = drive
        self.current_pose = current_pose
        self.limits = limits
        self.timeout_s = timeout_s
        self.clock = clock
        self.cancelled = cancelled or (lambda: False)
        self.period = 1 / limits.control_hz

    def execute(
        self,
        segment: MotionSegment,
        *,
        budget: ExecutionBudget | None = None,
        pose_checkpoint: PoseCheckpoint | None = None,
    ) -> None:
        limits = self.limits
        speed = segment.target_speed_mps
        if speed is None:
            raise ExecutionError("smooth motion requires a speed limit")
        start = self.current_pose()
        goal_yaw = segment.start.yaw_deg + resolve_yaw_delta_deg(
            segment.start.yaw_deg, segment.end.yaw_deg, segment.yaw_delta_deg
        )
        measured_yaw = segment.start.yaw_deg + resolve_yaw_delta_deg(
            segment.start.yaw_deg, start.yaw_deg
        )
        remaining_yaw = goal_yaw - measured_yaw
        duration = max(
            segment.duration_s or 0,
            duration_for(
                start,
                segment.end,
                speed,
                limits,
                yaw_delta_deg=remaining_yaw,
            ),
        )
        self._check_budget(budget, duration + limits.settle_time_s)
        if duration + limits.settle_time_s >= self.timeout_s:
            raise ExecutionError(
                f"smooth duration {duration:.2f}s exceeds available execution timeout"
            )
        # Re-anchor interpolation to measured position, but keep the world goal.
        segment = replace(
            segment, start=start, duration_s=duration, yaw_delta_deg=remaining_yaw
        )
        period = self.period
        steps = 0
        previous_pose = start
        last_velocity = (0.0, 0.0, 0.0)
        last_yaw_rate = 0.0
        settled_since = None
        while True:
            self._check_budget(budget)
            if self.cancelled():
                raise ExecutionError("vehicle motion cancelled")
            elapsed = steps * period
            if elapsed >= self.timeout_s:
                raise ExecutionError(
                    "synchronized motion timed out before position/yaw settled"
                )
            actual = self.current_pose()
            if steps:
                measured_yaw += resolve_yaw_delta_deg(
                    previous_pose.yaw_deg, actual.yaw_deg
                )
            measured_velocity = (
                (actual.x - previous_pose.x) / period,
                (actual.y - previous_pose.y) / period,
                (actual.z - previous_pose.z) / period,
            )
            moved = math.dist(
                (actual.x, actual.y, actual.z),
                (previous_pose.x, previous_pose.y, previous_pose.z),
            )
            measured_speed = moved / period if period > 0 else 0.0
            measured_yaw_rate = (
                abs(wrap_yaw_deg(actual.yaw_deg - previous_pose.yaw_deg)) / period
                if period > 0
                else 0.0
            )
            goal = segment.end
            distance = math.dist((actual.x, actual.y, actual.z), (goal.x, goal.y, goal.z))
            yaw_error = abs(goal_yaw - measured_yaw)
            settled = (
                elapsed >= duration
                and distance <= limits.position_tolerance_m
                and yaw_error <= limits.yaw_tolerance_deg
                and measured_speed <= limits.stopped_speed_mps
                and measured_yaw_rate <= limits.stopped_yaw_rate_deg_s
            )
            settled_since = (
                (elapsed if settled_since is None else settled_since)
                if settled
                else None
            )
            if settled_since is not None and elapsed - settled_since >= limits.settle_time_s:
                final = self.current_pose()
                final_distance = math.dist(
                    (final.x, final.y, final.z), (goal.x, goal.y, goal.z)
                )
                final_yaw = abs(
                    goal_yaw
                    - (measured_yaw + wrap_yaw_deg(final.yaw_deg - actual.yaw_deg))
                )
                if (
                    final_distance > limits.position_tolerance_m
                    or final_yaw > limits.yaw_tolerance_deg
                ):
                    raise ExecutionError(
                        "position/yaw drifted outside tolerance while hovering"
                    )
                return
            if pose_checkpoint is not None and pose_checkpoint(actual, measured_yaw):
                raise MotionPaused(actual, measured_yaw)
            reference = sample_segment(segment, elapsed)
            error = (
                reference.pose.x - actual.x,
                reference.pose.y - actual.y,
                reference.pose.z - actual.z,
            )
            desired = limit_vector(
                tuple(
                    v + limits.position_gain * e
                    for v, e in zip(reference.velocity, error)
                ),
                speed,
            )
            change = limit_vector(
                tuple(v - p for v, p in zip(desired, last_velocity)),
                limits.max_acceleration_mps2 * period,
            )
            velocity = tuple(p + d for p, d in zip(last_velocity, change))
            yaw_rate = reference.yaw_rate_deg_s + limits.yaw_gain * wrap_yaw_deg(
                reference.pose.yaw_deg - actual.yaw_deg
            )
            if not math.isfinite(yaw_rate):
                raise ExecutionError("non-finite yaw control command")
            yaw_rate = max(
                -limits.max_yaw_rate_deg_s,
                min(limits.max_yaw_rate_deg_s, yaw_rate),
            )
            yaw_change = limits.max_yaw_acceleration_deg_s2 * period
            yaw_rate = max(
                last_yaw_rate - yaw_change, min(last_yaw_rate + yaw_change, yaw_rate)
            )
            if not all(
                math.isfinite(v) for v in (*velocity, yaw_rate, period)
            ):
                raise ExecutionError("non-finite control velocity or yaw rate")
            # A fresh tick with no advance reports zero displacement; the
            # drive model clamps the steering ratio at low speed.
            self.drive(
                velocity, yaw_rate, measured_velocity, actual.yaw_deg, period
            )
            last_velocity, last_yaw_rate = velocity, yaw_rate
            previous_pose = actual
            steps += 1

    def _check_budget(self, budget, required_s=0.0):
        if budget is not None and budget.remaining(self.clock()) <= required_s:
            raise BudgetExceeded("motion/task deadline exhausted")


def tracking_limits(motion: MotionLimits) -> MotionLimits:
    """Convert the configured envelope into the domain tracking limits."""
    from ...domain.motion import MotionLimits as TrackingLimits

    return TrackingLimits(
        max_acceleration_mps2=motion.max_accel_mps2,
        max_yaw_rate_deg_s=motion.max_yaw_rate_dps,
        max_yaw_acceleration_deg_s2=motion.max_yaw_accel_dps2,
        control_hz=motion.control_hz,
        min_duration_s=max(motion.min_duration_s, 1e-3),
    )