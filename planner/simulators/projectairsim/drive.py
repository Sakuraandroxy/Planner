"""Reference velocity/yaw stream -> ProjectAirSim rover controls.

``ContinuousExecution`` and ``SynchronizedMotionPlanner`` emit a world-frame
velocity and a yaw rate; the rover only accepts ``engine``, ``steering_angle``
and ``brake``. This module is that actuator mapping (approach A in
``docs/PLAN.md``), measured on CityEnviron's ``fast-physics`` OffroadCar:

- ``engine`` maps linearly to signed ground speed at about ``23.3 m/s`` per
  unit in both directions (0.05 -> 1.16 m/s, 0.5 -> 11.65 m/s, -0.2 -> -4.66).
- ``steering_angle`` maps to yaw rate proportionally to signed forward speed
  at about ``17.9 deg/s per (m/s * unit)`` for ``|steering| <= 0.5`` and
  saturates gently above it (1.0 behaves like 0.62 rad of effective steer).
- ``brake=1`` stops the rover from ~8 m/s within one control step. The brake
  is only applied while the rover is still rolling forward faster than
  commanded, so a reverse command from rest reaches the engine instead.

Speed is held by a small speed PID whose feed-forward term already matches the
static plant; the feedback only removes model error (grades, contact drag).
Steering stays a feed-forward of the reference yaw rate, which is the
pure-pursuit curvature command computed by the unchanged reference stack.
Reverse is handled by the signed forward speed: the same yaw-rate command
needs the opposite steering sign while backing up.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class RoverDriveModel:
    """Calibrated actuator model for the CityEnviron rover."""

    speed_per_engine_mps: float = 23.3
    yaw_rate_per_steer_mps: float = 17.9
    max_steering: float = 1.0
    #: Below this speed the steering ratio is frozen so a stopped rover that
    #: receives a heading command still turns as soon as it creeps forward.
    min_steering_speed_mps: float = 0.5
    #: Commanded speed overrun that starts applying the brake, in m/s.
    brake_deadband_mps: float = 0.15
    #: Brake fraction per m/s of overrun.
    brake_gain: float = 0.8
    speed_kp: float = 0.004
    speed_ki: float = 0.1
    speed_kd: float = 0.0
    integral_limit: float = 0.3

    def __post_init__(self) -> None:
        for name in (
            "speed_per_engine_mps",
            "yaw_rate_per_steer_mps",
            "max_steering",
            "min_steering_speed_mps",
            "brake_deadband_mps",
            "brake_gain",
            "speed_kp",
            "speed_ki",
            "speed_kd",
            "integral_limit",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
            ):
                raise ValueError(f"drive model {name} must be finite and non-negative")
        if self.speed_per_engine_mps <= 0 or self.yaw_rate_per_steer_mps <= 0:
            raise ValueError("drive model gains must be positive")
        if not 0 < self.max_steering <= 1:
            raise ValueError("drive model max_steering must be in (0, 1]")

    def steering_for(self, yaw_rate_deg_s: float, forward_speed_mps: float) -> float:
        """Steering angle that produces ``yaw_rate_deg_s`` at this speed."""
        speed = forward_speed_mps
        if abs(speed) < self.min_steering_speed_mps:
            speed = math.copysign(self.min_steering_speed_mps, speed if speed else 1.0)
        raw = yaw_rate_deg_s / (self.yaw_rate_per_steer_mps * speed)
        return _clamp(raw, -self.max_steering, self.max_steering)


class SpeedPid:
    """Speed feedback around the model's feed-forward engine term."""

    def __init__(self, model: RoverDriveModel):
        self.model = model
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error: float | None = None

    def step(self, error: float, dt_s: float) -> float:
        if (
            not math.isfinite(error)
            or not math.isfinite(dt_s)
            or dt_s <= 0
        ):
            raise ValueError("speed PID requires a finite error and positive dt")
        model = self.model
        self._integral = _clamp(
            self._integral + error * dt_s,
            -model.integral_limit,
            model.integral_limit,
        )
        derivative = 0.0
        if self._previous_error is not None:
            derivative = (error - self._previous_error) / dt_s
        self._previous_error = error
        return (
            model.speed_kp * error
            + model.speed_ki * self._integral
            + model.speed_kd * derivative
        )


def heading_vector(yaw_deg: float) -> tuple[float, float]:
    """NED forward axis (north, east) for a yaw angle."""
    angle = math.radians(yaw_deg)
    return math.cos(angle), math.sin(angle)


@dataclass(frozen=True)
class RoverControls:
    engine: float
    steering: float
    brake: float


def controls_from_reference(
    model: RoverDriveModel,
    pid: SpeedPid,
    velocity_world: tuple[float, float, float],
    yaw_rate_deg_s: float,
    measured_velocity_world: tuple[float, float, float],
    yaw_deg: float,
    dt_s: float,
) -> RoverControls:
    """Convert the reference stream command into actuator values."""
    if (
        len(velocity_world) != 3
        or len(measured_velocity_world) != 3
        or not all(math.isfinite(value) for value in (*velocity_world, *measured_velocity_world))
        or not math.isfinite(yaw_rate_deg_s)
        or not math.isfinite(dt_s)
        or dt_s <= 0
    ):
        raise ValueError("drive conversion requires finite values and positive dt")
    north, east = heading_vector(yaw_deg)
    command = velocity_world[0] * north + velocity_world[1] * east
    measured = measured_velocity_world[0] * north + measured_velocity_world[1] * east
    error = command - measured
    correction = pid.step(error, dt_s)
    engine = _clamp(command / model.speed_per_engine_mps + correction, -1.0, 1.0)
    brake = 0.0
    # Only brake a rover that is still rolling forward faster than commanded.
    # At rest the same negative error means "start reversing", and the engine
    # must be allowed to spin the wheels backwards instead.
    if measured > model.brake_deadband_mps and error < -model.brake_deadband_mps:
        engine = 0.0
        brake = _clamp(-error * model.brake_gain, 0.0, 1.0)
    steering = model.steering_for(yaw_rate_deg_s, measured)
    return RoverControls(engine=engine, steering=steering, brake=brake)