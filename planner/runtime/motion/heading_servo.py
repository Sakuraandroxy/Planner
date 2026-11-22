"""Heading rate shaping shared by the follower, the trackers and the hold.

One law for every place that turns a heading error into a rate command:

    rate = clamp(sign(e) * min(gain * |e|, braking curve) - damping * measured)

The proportional term sets the slope near the target. ``braking_speed`` can
only lower it, so a large error cannot overshoot the way a bare proportional
term does. The damping term uses the measured yaw rate, never a pose
difference: differencing amplifies pose jitter into the command and the
feedback then holds an oscillation instead of removing one.

``tracked_heading_rate_deg_s`` adds what the two segment trackers need on top
of that law: a feed-forward rate from the reference trajectory, the rate clamp
and the slew limit. The trackers differ in transport only, so the arithmetic
lives here once.
"""
import math

from ...errors import ExecutionError
from .continuous_speed import braking_speed


def heading_rate_deg_s(
    error_deg: float,
    measured_rate_dps: float | None = None,
    *,
    gain: float,
    maximum_rate_dps: float,
    acceleration_dps2: float,
    jerk_dps3: float,
    damping: float = 0.0,
    deadband_deg: float = 0.0,
    rate_deadband_dps: float = 0.0,
    current_acceleration_dps2: float = 0.0,
    response_time_s: float = 0.0,
) -> float:
    """Rate command in degrees per second for a heading error.

    ``measured_rate_dps`` is the simulator's ground-truth yaw rate. ``None``
    turns the damping term off, which is the only option on a call site that
    has no rate measurement at all.
    """
    _require_finite("heading error", error_deg)
    if measured_rate_dps is not None:
        _require_finite("measured yaw rate", measured_rate_dps)
    measured = 0.0 if measured_rate_dps is None else float(measured_rate_dps)
    if abs(error_deg) <= deadband_deg and abs(measured) <= rate_deadband_dps:
        return 0.0
    proportional = gain * abs(error_deg)
    limited = braking_speed(
        abs(error_deg),
        0.0,
        maximum_rate_dps,
        acceleration_dps2,
        jerk_dps3,
        current_acceleration_dps2,
        response_time_s,
    )
    shaped = math.copysign(min(proportional, limited), error_deg)
    command = shaped - damping * measured if measured_rate_dps is not None else shaped
    return max(-maximum_rate_dps, min(maximum_rate_dps, command))


def tracked_heading_rate_deg_s(
    reference_rate_dps: float,
    error_deg: float,
    *,
    last_rate_dps: float,
    period_s: float,
    gain: float,
    maximum_rate_dps: float,
    acceleration_dps2: float,
    jerk_dps3: float,
) -> float:
    """Feed-forward plus the shared law, clamped to the rate and its slew limit."""
    command = float(reference_rate_dps) + heading_rate_deg_s(
        error_deg,
        gain=gain,
        maximum_rate_dps=maximum_rate_dps,
        acceleration_dps2=acceleration_dps2,
        jerk_dps3=jerk_dps3,
    )
    if not math.isfinite(command):
        raise ExecutionError("non-finite yaw control command")
    command = max(-maximum_rate_dps, min(maximum_rate_dps, command))
    step = acceleration_dps2 * period_s
    return max(last_rate_dps - step, min(last_rate_dps + step, command))


def _require_finite(name: str, value) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be a finite number")