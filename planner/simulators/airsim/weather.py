"""AirSim weather: the eight parameters the engine's RPC accepts.

Ported from ``airsim-flight-console/weather_service.py``, which applies these
against real UE4 builds. The canonical-name to engine-spelling map is not duplicated
here: ``runtime.WEATHER_FIELDS`` holds it, and this module adds only what is AirSim's
own -- the ``WeatherParameter`` enum values.
"""

from __future__ import annotations

from typing import Callable

#: ``airsim.WeatherParameter`` values, keyed by the engine's spelling. Note
#: ``roadwetness``: the engine writes it without the underscore the planner uses,
#: and that difference is exactly why the spelling map lives in the adapter.
PARAMETER_IDS: dict[str, int] = {
    "rain": 0,
    "roadwetness": 1,
    "snow": 2,
    "road_snow": 3,
    "maple_leaf": 4,
    "road_leaf": 5,
    "dust": 6,
    "fog": 7,
}


class AirSimWeather:
    """``WeatherControl`` over the AirSim msgpack RPC."""

    def __init__(self, connection, fields: dict[str, str], presets: tuple[str, ...]):
        self._connection = connection
        self._fields = dict(fields)
        self._presets = tuple(presets)

    def fields(self) -> dict[str, str]:
        return dict(self._fields)

    def presets(self) -> tuple[str, ...]:
        return self._presets

    def apply(self, values: dict[str, float]) -> dict[str, float]:
        """Send every parameter the caller named, in the engine's own spelling.

        AirSim leaves its weather effects switched off, so
        ``simEnableWeather`` must run before ``simSetWeatherParameter`` has any
        effect. An all-zero state switches them back off.
        """
        self._connection.call(
            "simEnableWeather", any(value > 0.0 for value in values.values())
        )
        applied: dict[str, float] = {}
        for name, value in values.items():
            spelling = self._fields.get(name)
            if spelling is None:
                raise KeyError(
                    f"weather field {name!r} has no AirSim spelling; this simulator "
                    f"knows {sorted(self._fields)}"
                )
            parameter = PARAMETER_IDS[spelling]
            self._connection.call("simSetWeatherParameter", parameter, float(value))
            applied[name] = float(value)
        return applied
