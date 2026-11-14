import math

import pytest

from planner.adapters.projectairsim.drive import (
    RoverDriveModel,
    SpeedPid,
    controls_from_reference,
    heading_vector,
)

MODEL = RoverDriveModel()


def controls(
    velocity_world,
    *,
    yaw_rate=0.0,
    measured=(0.0, 0.0, 0.0),
    yaw=0.0,
    dt=0.05,
    model=MODEL,
    pid=None,
):
    return controls_from_reference(
        model,
        pid if pid is not None else SpeedPid(model),
        velocity_world,
        yaw_rate,
        measured,
        yaw,
        dt,
    )


# -- RoverDriveModel -------------------------------------------------------
def test_heading_vector_is_ned_forward():
    assert heading_vector(0.0) == pytest.approx((1.0, 0.0))
    assert heading_vector(90.0) == pytest.approx((0.0, 1.0), abs=1e-12)
    assert heading_vector(180.0) == pytest.approx((-1.0, 0.0), abs=1e-12)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"speed_per_engine_mps": 0.0},
        {"yaw_rate_per_steer_mps": -1.0},
        {"max_steering": 0.0},
        {"max_steering": 1.5},
        {"speed_kp": float("nan")},
        {"speed_ki": True},
    ],
)
def test_drive_model_rejects_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        RoverDriveModel(**kwargs)


def test_steering_is_proportional_to_yaw_rate_over_signed_speed():
    assert MODEL.steering_for(17.9, 1.0) == pytest.approx(1.0)
    assert MODEL.steering_for(8.95, 2.0) == pytest.approx(0.25)
    # Reversing needs the opposite steering sign for the same yaw rate.
    assert MODEL.steering_for(17.9, -1.0) == pytest.approx(-1.0)
    assert MODEL.steering_for(-17.9, 1.0) == pytest.approx(-1.0)


def test_steering_ratio_is_frozen_below_the_minimum_speed():
    assert MODEL.steering_for(4.475, 0.0) == pytest.approx(0.5)
    assert MODEL.steering_for(-4.475, 0.0) == pytest.approx(-0.5)
    assert MODEL.steering_for(4.475, 0.1) == pytest.approx(0.5)
    assert MODEL.steering_for(1000.0, 0.1) == pytest.approx(1.0)


# -- SpeedPid --------------------------------------------------------------
def test_speed_pid_integrates_with_clamp_and_resets():
    pid = SpeedPid(
        RoverDriveModel(speed_kp=0.0, speed_ki=1.0, speed_kd=0.0, integral_limit=0.2)
    )
    assert pid.step(1.0, 0.1) == pytest.approx(0.1)
    assert pid.step(1.0, 0.1) == pytest.approx(0.2)
    assert pid.step(1.0, 0.1) == pytest.approx(0.2)  # clamped
    pid.reset()
    assert pid.step(1.0, 0.1) == pytest.approx(0.1)


def test_speed_pid_derivative_uses_the_previous_error():
    pid = SpeedPid(
        RoverDriveModel(speed_kp=0.0, speed_ki=0.0, speed_kd=1.0, integral_limit=1.0)
    )
    assert pid.step(1.0, 0.5) == 0.0
    assert pid.step(2.0, 0.5) == pytest.approx(2.0)


def test_speed_pid_rejects_non_finite_inputs():
    pid = SpeedPid(RoverDriveModel())
    with pytest.raises(ValueError):
        pid.step(float("nan"), 0.05)
    with pytest.raises(ValueError):
        pid.step(1.0, 0.0)


# -- controls_from_reference ----------------------------------------------
def test_forward_command_combines_feedforward_and_speed_feedback():
    result = controls((5.0, 0.0, 0.0))
    # error=5 m/s -> integral 5*0.05=0.25 -> correction 0.004*5 + 0.1*0.25.
    assert result.engine == pytest.approx(5.0 / 23.3 + 0.045)
    assert result.steering == 0.0
    assert result.brake == 0.0


def test_lateral_world_velocity_is_projected_onto_the_heading():
    result = controls((2.0, 3.0, 0.0), yaw=90.0)
    command = 3.0  # north*cos90 + east*sin90
    assert result.engine == pytest.approx(
        command / 23.3 + 0.004 * command + 0.1 * (command * 0.05)
    )


def test_reverse_command_drives_the_engine_backwards_from_rest():
    result = controls((-2.0, 0.0, 0.0))
    # error=-2 -> integral=-0.1 -> correction=-0.008-0.01.
    assert result.engine < 0
    assert result.engine == pytest.approx(-2.0 / 23.3 - 0.018)
    # Rest is not an overspeed: the brake must stay off so reverse can start.
    assert result.brake == 0.0


def test_sustained_reverse_stays_off_the_brake():
    pid = SpeedPid(MODEL)
    result = controls((-2.0, 0.0, 0.0), measured=(-1.0, 0.0, 0.0), pid=pid)
    assert result.brake == 0.0
    assert result.engine < 0


def test_brake_overrides_the_engine_while_rolling_forward_too_fast():
    result = controls((0.0, 0.0, 0.0), measured=(4.0, 0.0, 0.0))
    assert result.engine == 0.0
    assert result.brake == pytest.approx(1.0)


def test_brake_gain_scales_with_the_forward_speed_overrun():
    result = controls((2.0, 0.0, 0.0), measured=(2.5, 0.0, 0.0))
    assert result.engine == 0.0
    assert result.brake == pytest.approx(0.4)


def test_yaw_rate_command_produces_steering_from_measured_speed():
    result = controls((5.0, 0.0, 0.0), yaw_rate=8.95, measured=(2.0, 0.0, 0.0))
    assert result.steering == pytest.approx(8.95 / (17.9 * 2.0))


@pytest.mark.parametrize(
    "kwargs",
    [
        {"velocity_world": (float("nan"), 0.0, 0.0)},
        {"velocity_world": (1.0, 0.0)},
        {"measured": (float("inf"), 0.0, 0.0)},
        {"yaw_rate": float("nan")},
        {"dt": 0.0},
    ],
)
def test_controls_reject_non_finite_requests(kwargs):
    with pytest.raises(ValueError):
        controls(**{"velocity_world": (1.0, 0.0, 0.0), **kwargs})


def test_engine_saturates_at_full_scale():
    assert controls((1000.0, 0.0, 0.0)).engine == pytest.approx(1.0)
    assert controls((-1000.0, 0.0, 0.0), measured=(-1000.0, 0.0, 0.0)).engine < 0