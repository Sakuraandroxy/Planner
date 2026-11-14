import math

import pytest

from planner.domain.weather import (
    WEATHER_CATALOG,
    WEATHER_FIELDS,
    WEATHER_LABELS,
    default_weather,
    merge_weather,
    validate_weather_values,
)


def _complete(**overrides):
    values = default_weather()
    values.update(overrides)
    return values


def test_fields_follow_the_weather_parameter_order():
    assert WEATHER_FIELDS == (
        "rain",
        "road_wetness",
        "snow",
        "road_snow",
        "maple_leaf",
        "road_leaf",
        "dust",
        "fog",
    )
    assert set(WEATHER_LABELS) == set(WEATHER_FIELDS)


def test_every_preset_is_complete_and_in_range():
    assert WEATHER_CATALOG.presets
    for preset in WEATHER_CATALOG.presets:
        assert preset.id
        assert preset.label
        assert set(preset.values) == set(WEATHER_FIELDS)
        for value in preset.values.values():
            assert 0.0 <= value <= 1.0
    clear = WEATHER_CATALOG.preset("clear")
    assert clear is not None
    assert all(value == 0.0 for value in clear.values.values())


def test_catalog_lists_fields_with_labels():
    assert [field.id for field in WEATHER_CATALOG.fields] == list(WEATHER_FIELDS)
    for field in WEATHER_CATALOG.fields:
        assert field.label == WEATHER_LABELS[field.id]


def test_catalog_lookup_returns_none_for_unknown_presets():
    assert WEATHER_CATALOG.preset("moon") is None


@pytest.mark.parametrize(
    "values",
    [
        {"rain": 0.5},
        {"rain": "0.5"},
        {"rain": True},
        {"rain": math.nan},
        {"rain": math.inf},
        {"rain": 1.1},
        {"rain": -0.1},
        {"unknown": 0.5},
        [],
        None,
    ],
    ids=[
        "incomplete",
        "string",
        "bool",
        "nan",
        "inf",
        "above-range",
        "below-range",
        "unknown-field",
        "list",
        "none",
    ],
)
def test_validate_weather_values_rejects_bad_input(values):
    with pytest.raises(ValueError):
        validate_weather_values(values)


def test_validate_weather_values_accepts_a_full_mapping():
    values = _complete(rain=0.25, fog=1.0)
    assert validate_weather_values(values) == values


def test_merge_weather_uses_zeroes_as_the_base():
    merged = merge_weather(None, {"rain": 0.5})
    assert merged == _complete(rain=0.5)


def test_merge_weather_overlays_partial_updates():
    current = _complete(rain=0.5, fog=0.2)
    merged = merge_weather(current, {"fog": 0.9})
    assert merged["rain"] == 0.5
    assert merged["fog"] == 0.9
    assert current["fog"] == 0.2


def test_merge_weather_rejects_empty_and_unknown_updates():
    with pytest.raises(ValueError, match="non-empty"):
        merge_weather(None, {})
    with pytest.raises(ValueError, match="unknown weather fields"):
        merge_weather(None, {"moon": 0.5})


def test_merge_weather_validates_the_current_state():
    with pytest.raises(ValueError):
        merge_weather({"rain": 0.5}, {"fog": 0.2})
