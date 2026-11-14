"""ProjectAirSim weather-effect catalog and value validation.

Domain-only module: no simulator SDK types. The eight field ids map to
``projectairsim.types.WeatherParameter`` values 1..8 in this exact order;
the session adapter performs the enum conversion.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


# WeatherParameter order: RAIN=1, ROAD_WETNESS=2, SNOW=3, ROAD_SNOW=4,
# MAPLE_LEAF=5, ROAD_LEAF=6, DUST=7, FOG=8.
WEATHER_FIELDS: tuple[str, ...] = (
    "rain",
    "road_wetness",
    "snow",
    "road_snow",
    "maple_leaf",
    "road_leaf",
    "dust",
    "fog",
)

WEATHER_LABELS: dict[str, str] = {
    "rain": "降雨",
    "road_wetness": "路面湿度",
    "snow": "降雪",
    "road_snow": "路面积雪",
    "maple_leaf": "落叶（枫叶）",
    "road_leaf": "路面落叶",
    "dust": "扬尘",
    "fog": "雾",
}


@dataclass(frozen=True)
class WeatherField:
    id: str
    label: str


@dataclass(frozen=True)
class WeatherPreset:
    """A named, complete eight-field weather state (values in [0, 1])."""

    id: str
    label: str
    values: dict[str, float]


@dataclass(frozen=True)
class WeatherCatalog:
    fields: tuple[WeatherField, ...]
    presets: tuple[WeatherPreset, ...]

    def preset(self, preset_id: str) -> WeatherPreset | None:
        for preset in self.presets:
            if preset.id == preset_id:
                return preset
        return None


def _validate_value(name: str, value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or not 0.0 <= float(value) <= 1.0
    ):
        raise ValueError(f"weather.{name} must be a finite number in [0, 1]")
    return float(value)


def validate_weather_values(values: object) -> dict[str, float]:
    """Validate and return a complete eight-field weather mapping."""
    if not isinstance(values, dict):
        raise ValueError("weather values must be a mapping")
    unknown = set(values) - set(WEATHER_FIELDS)
    if unknown:
        raise ValueError(f"unknown weather fields: {sorted(unknown)}")
    missing = [name for name in WEATHER_FIELDS if name not in values]
    if missing:
        raise ValueError(f"missing weather fields: {', '.join(missing)}")
    return {name: _validate_value(name, values[name]) for name in WEATHER_FIELDS}


def merge_weather(
    current: dict[str, float] | None, updates: object
) -> dict[str, float]:
    """Overlay a partial update on the current state and return all fields."""
    if not isinstance(updates, dict) or not updates:
        raise ValueError("weather update must be a non-empty mapping")
    unknown = set(updates) - set(WEATHER_FIELDS)
    if unknown:
        raise ValueError(f"unknown weather fields: {sorted(unknown)}")
    base = validate_weather_values(current) if current is not None else (
        default_weather()
    )
    for name, value in updates.items():
        base[name] = _validate_value(name, value)
    return base


def default_weather() -> dict[str, float]:
    return {name: 0.0 for name in WEATHER_FIELDS}


def _complete(values: dict[str, float]) -> dict[str, float]:
    merged = default_weather()
    merged.update(values)
    return validate_weather_values(merged)


WEATHER_PRESETS: tuple[WeatherPreset, ...] = (
    WeatherPreset("clear", "晴朗", _complete({})),
    WeatherPreset(
        "overcast",
        "阴天",
        _complete({"road_wetness": 0.1, "road_leaf": 0.2, "fog": 0.35}),
    ),
    WeatherPreset(
        "light_rain",
        "小雨",
        _complete({"rain": 0.35, "road_wetness": 0.5, "fog": 0.1}),
    ),
    WeatherPreset(
        "heavy_rain",
        "大雨",
        _complete({"rain": 0.85, "road_wetness": 0.9, "fog": 0.25}),
    ),
    WeatherPreset(
        "snow",
        "降雪",
        _complete({"snow": 0.7, "road_snow": 0.6, "fog": 0.15}),
    ),
    WeatherPreset("fog", "浓雾", _complete({"fog": 0.8})),
    WeatherPreset("dust", "扬尘", _complete({"dust": 0.75, "fog": 0.1})),
)

WEATHER_CATALOG = WeatherCatalog(
    fields=tuple(WeatherField(name, WEATHER_LABELS[name]) for name in WEATHER_FIELDS),
    presets=WEATHER_PRESETS,
)
