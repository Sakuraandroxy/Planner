"""Weather parameters mapped onto the loaded world's visual effects.

A weather state is the complete eight-field set from ``planner.domain``; effects
are enabled and each nonzero field is pushed by name. An all-zero state disables
the effects instead of setting eight parameters to zero, which is the state a
freshly loaded scene is in.
"""

from __future__ import annotations

from projectairsim.types import WeatherParameter

from planner.domain import WEATHER_FIELDS, validate_weather_values
from ...errors import ProtocolError


def apply_weather(connection, fields, log) -> dict[str, float]:
    """Push one validated weather state to the world; return what was applied.

    The caller owns the remembered state: this writes effects and reports, and
    raises before touching the world when the field set is incomplete.
    """
    values = validate_weather_values(fields)
    if connection is None or connection.world is None:
        raise RuntimeError("the ProjectAirSim session is closed")
    connection.reset_weather_effects()
    if all(value == 0.0 for value in values.values()):
        if not connection.disable_weather_effects():
            raise ProtocolError("simulator refused to disable weather effects")
        log("Weather cleared; all visual effects disabled.")
        return values
    if not connection.enable_weather_effects():
        raise ProtocolError("simulator refused to enable weather effects")
    for name in WEATHER_FIELDS:
        value = values[name]
        if value == 0.0:
            continue
        parameter = int(getattr(WeatherParameter, name.upper()))
        if not connection.set_weather_parameter(parameter, value):
            raise ProtocolError(
                f"simulator refused weather.{name}={value:.2f}"
            )
    log(
        "Weather applied: "
        + ", ".join(
            f"{name}={values[name]:.2f}"
            for name in WEATHER_FIELDS
            if values[name] > 0.0
        )
    )
    return values
