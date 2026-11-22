"""Control-loop telemetry sampled during streaming; the diagnostic trace reads it."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ControlTelemetry:
    timestamp_ns: int
    observed_monotonic: float
    dt_s: float
    waypoint_id: int
    queue_length: int
    commanded_velocity: tuple[float, float, float]
    measured_velocity: tuple[float, float, float]
    commanded_acceleration: tuple[float, float, float]
    yaw_rate_deg_s: float
    yaw_acceleration_deg_s2: float
    yaw_error_deg: float
    position_error_m: float
    aligning: bool
    pitch_deg: float | None
    roll_deg: float | None
