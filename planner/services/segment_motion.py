"""Quintic-segment motion planning shared by all vehicle adapters."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..domain.pose import (
    RelativePoseDelta,
    WorldPose,
    relative_to_world,
    wrap_deg,
)

# Peak values of the normalized quintic 10t^3 - 15t^4 + 6t^5.
_QUINTIC_PEAK_VELOCITY = 1.875
_QUINTIC_PEAK_ACCELERATION = 5.773502691896258


@dataclass(frozen=True)
class MotionLimits:
    max_speed_mps: float = 3.0
    max_accel_mps2: float = 1.5
    max_yaw_rate_dps: float = 30.0
    max_yaw_accel_dps2: float = 60.0
    control_hz: float = 20.0
    min_duration_s: float = 1.0
    #: 航向伺服的四个参数，含义见 planner/services/motion/heading_servo.py。
    heading_hold_gain: float = 0.4
    heading_rate_damping: float = 0.0
    heading_deadband_deg: float = 0.5
    heading_rate_deadband_dps: float = 1.5

    #: 允许取零的参数：零表示关掉该项。
    OPTIONAL = frozenset(
        {"heading_rate_damping", "heading_deadband_deg", "heading_rate_deadband_dps"}
    )
    #: 必须为正的参数，单独列出以便校验时区分。
    POSITIVE = frozenset({"heading_hold_gain"})

    def __post_init__(self) -> None:
        for name in (
            "max_speed_mps",
            "max_accel_mps2",
            "max_yaw_rate_dps",
            "max_yaw_accel_dps2",
            "control_hz",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.min_duration_s < 0:
            raise ValueError("min_duration_s must be non-negative")
        for name in self.OPTIONAL:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be numeric")
            if value < 0:
                raise ValueError(f"{name} must be non-negative")
        for name in self.POSITIVE:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be numeric")
            if value <= 0:
                raise ValueError(f"{name} must be positive")

    def to_tracking_limits(self):
        """Convert this configured envelope into the domain tracking limits.

        Two classes are named ``MotionLimits`` here: this one carries what
        ``config/core/<profile>/base.yaml`` states (``max_accel_mps2``, ``max_yaw_rate_dps``), and
        ``planner/domain/motion.py`` carries what the motion trackers and the
        position hold read (``max_acceleration_mps2``, ``max_yaw_rate_deg_s``, plus
        the tolerances). Every simulator needs the second, so the mapping lives with
        the first instead of being re-derived per simulator package.
        """
        from ..domain.motion import MotionLimits as TrackingLimits

        return TrackingLimits(
            max_acceleration_mps2=self.max_accel_mps2,
            max_yaw_rate_deg_s=self.max_yaw_rate_dps,
            max_yaw_acceleration_deg_s2=self.max_yaw_accel_dps2,
            control_hz=self.control_hz,
            min_duration_s=max(self.min_duration_s, 1e-3),
            heading_hold_gain=self.heading_hold_gain,
            heading_rate_damping=self.heading_rate_damping,
            heading_deadband_deg=self.heading_deadband_deg,
            heading_rate_deadband_dps=self.heading_rate_deadband_dps,
        )


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
                _QUINTIC_PEAK_ACCELERATION * distance / limits.max_accel_mps2
            )
        )
    if yaw > 0.0:
        durations.append(
            _QUINTIC_PEAK_VELOCITY * yaw / limits.max_yaw_rate_dps
        )
        durations.append(
            math.sqrt(
                _QUINTIC_PEAK_ACCELERATION * yaw / limits.max_yaw_accel_dps2
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