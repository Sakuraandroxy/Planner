"""One heading law for every control path: shaping, damping, deadband."""
import math

import pytest

from planner.runtime import braking_speed
from planner.runtime import heading_rate_deg_s

LIMITS = dict(
    gain=2.0,
    maximum_rate_dps=30.0,
    acceleration_dps2=60.0,
    jerk_dps3=60.0,
)


def test_proportional_term_sets_the_small_error_slope():
    assert heading_rate_deg_s(1.0, 0.0, **LIMITS) == pytest.approx(2.0)
    assert heading_rate_deg_s(-1.0, 0.0, **LIMITS) == pytest.approx(-2.0)


def test_braking_curve_caps_a_large_error_below_the_proportional_slope():
    command = heading_rate_deg_s(90.0, 0.0, **LIMITS)
    assert command == pytest.approx(LIMITS["maximum_rate_dps"])
    assert command < LIMITS["gain"] * 90.0


def test_damping_subtracts_a_rate_that_follows_the_error():
    undamped = heading_rate_deg_s(10.0, 0.0, **LIMITS)
    damped = heading_rate_deg_s(10.0, 4.0, damping=0.6, **LIMITS)
    assert damped == pytest.approx(undamped - 0.6 * 4.0)


def test_damping_adds_when_the_vehicle_turns_against_the_error():
    undamped = heading_rate_deg_s(10.0, 0.0, **LIMITS)
    damped = heading_rate_deg_s(10.0, -4.0, damping=0.6, **LIMITS)
    assert damped == pytest.approx(undamped + 0.6 * 4.0)


def test_deadband_is_zero_inside_and_a_command_outside():
    assert heading_rate_deg_s(0.2, 0.4, deadband_deg=0.5, rate_deadband_dps=1.5, **LIMITS) == 0.0
    assert heading_rate_deg_s(0.2, 4.0, deadband_deg=0.5, rate_deadband_dps=1.5, **LIMITS) != 0.0
    assert heading_rate_deg_s(3.0, 0.4, deadband_deg=0.5, rate_deadband_dps=1.5, **LIMITS) != 0.0


def test_missing_measured_rate_disables_the_damping_term():
    assert heading_rate_deg_s(10.0, None, damping=0.6, **LIMITS) == pytest.approx(
        heading_rate_deg_s(10.0, 0.0, **LIMITS)
    )


def test_command_never_exceeds_the_maximum_rate():
    strong = dict(LIMITS, gain=100.0)
    assert heading_rate_deg_s(90.0, -30.0, damping=1.0, **strong) == pytest.approx(
        LIMITS["maximum_rate_dps"]
    )
    assert heading_rate_deg_s(-90.0, 30.0, damping=1.0, **strong) == pytest.approx(
        -LIMITS["maximum_rate_dps"]
    )


@pytest.mark.parametrize("error_deg", [-90.0, -12.0, -0.7, 0.0, 0.7, 12.0, 90.0])
def test_damping_off_and_deadband_off_gives_the_proportional_term_capped_by_braking(error_deg):
    """The follower's own numbers come out of the shared law unchanged."""
    bandwidth = 2.0
    expected = math.copysign(
        min(
            LIMITS["gain"] * abs(error_deg),
            braking_speed(
                abs(error_deg),
                0.0,
                LIMITS["maximum_rate_dps"],
                LIMITS["acceleration_dps2"],
                LIMITS["jerk_dps3"],
                0.0,
                2 / bandwidth,
            ),
        ),
        error_deg,
    )
    assert heading_rate_deg_s(
        error_deg, None, response_time_s=2 / bandwidth, **LIMITS
    ) == pytest.approx(expected)


def _track_step(response, damping, *, seconds=12.0, dt=0.05, target_deg=90.0, tau=0.5):
    """Yaw plant: commanded rate reaches the vehicle through a first-order lag."""
    yaw, rate, peak = 0.0, 0.0, 0.0
    for _ in range(int(seconds / dt)):
        command = heading_rate_deg_s(
            target_deg - yaw, rate, damping=damping, **LIMITS
        )
        rate += (command - rate) * dt / tau
        yaw += rate * dt
        peak = max(peak, yaw)
    return peak, yaw


def test_damping_removes_the_overshoot_of_a_lagged_plant():
    peak_without, final_without = _track_step(0.0, 0.0)
    peak_with, final_with = _track_step(0.0, 0.6)
    assert peak_without - 90.0 > 2.0
    assert abs(peak_with - 90.0) < 0.5
    assert final_with == pytest.approx(90.0, abs=0.05)
    assert final_without == pytest.approx(90.0, abs=0.05)


def _sign_changes(damping, deadband, *, steps=400, error_deg=0.1, jitter_deg=0.6):
    """A settled vehicle whose measured heading jitters: does the command chatter?"""
    changes, previous = 0, 0.0
    for index in range(steps):
        measured = jitter_deg * math.sin(index * 1.1)
        command = heading_rate_deg_s(
            error_deg + measured, measured, damping=damping,
            deadband_deg=deadband, rate_deadband_dps=deadband * 3, **LIMITS
        )
        if command * previous < 0:
            changes += 1
        previous = command
    return changes


def test_deadband_stops_the_command_from_following_measurement_jitter():
    assert _sign_changes(0.6, 0.0) > 20
    assert _sign_changes(0.6, 0.5) == 0


def test_rejects_non_finite_error_and_measured_rate():
    with pytest.raises(ValueError, match="heading error"):
        heading_rate_deg_s(float("nan"), 0.0, **LIMITS)
    with pytest.raises(ValueError, match="measured yaw rate"):
        heading_rate_deg_s(10.0, float("inf"), **LIMITS)