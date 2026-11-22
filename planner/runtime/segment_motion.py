"""Quintic-segment motion planning shared by all vehicle adapters."""

from __future__ import annotations

import math
from dataclasses import dataclass

from planner.domain import MotionLimits
from planner.domain import RelativePoseDelta, WorldPose, relative_to_world, wrap_deg

# Peak values of the normalized quintic 10t^3 - 15t^4 + 6t^5.
_QUINTIC_PEAK_VELOCITY = 1.875
_QUINTIC_PEAK_ACCELERATION = 5.773502691896258


@dataclass(frozen=True)
class SegmentSetpoint:
    t: float
    pose: WorldPose


def quintic(tau: float) -> float:
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def duration_for(delta: RelativePoseDelta, limits: MotionLimits) -> float:
    """Minimum duration that keeps speed/acceleration within limits."""
    distance = math.sqrt(delta.dx**2 + delta.dy**2 + delta.dz**2)
    yaw = abs(delta.dyaw_deg)
    durations = [limits.min_duration_s]
    if distance > 0.0:
        durations.append(
            _QUINTIC_PEAK_VELOCITY * distance / limits.max_speed_mps
        )
        durations.append(
            math.sqrt(
                _QUINTIC_PEAK_ACCELERATION * distance / limits.max_acceleration_mps2
            )
        )
    if yaw > 0.0:
        durations.append(
            _QUINTIC_PEAK_VELOCITY * yaw / limits.max_yaw_rate_deg_s
        )
        durations.append(
            math.sqrt(
                _QUINTIC_PEAK_ACCELERATION * yaw / limits.max_yaw_acceleration_deg_s2
            )
        )
    return max(durations)


def plan_segment(
    start: WorldPose, delta: RelativePoseDelta, limits: MotionLimits
) -> list[SegmentSetpoint]:
    """Sample a quintic segment at the control rate."""
    duration = duration_for(delta, limits)
    steps = max(2, math.ceil(duration * limits.control_hz) + 1)
    world_dx, world_dy, world_dz = relative_to_world(
        start.yaw_deg, delta.dx, delta.dy, delta.dz
    )
    setpoints: list[SegmentSetpoint] = []
    for index in range(steps):
        tau = index / (steps - 1)
        s = quintic(tau)
        setpoints.append(
            SegmentSetpoint(
                t=tau * duration,
                pose=WorldPose(
                    start.x + s * world_dx,
                    start.y + s * world_dy,
                    start.z + s * world_dz,
                    wrap_deg(start.yaw_deg + s * delta.dyaw_deg),
                ),
            )
        )
    return setpoints