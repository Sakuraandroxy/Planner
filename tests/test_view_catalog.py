import pytest

from planner.domain.view import (
    INITIAL_STREAM_INDEX,
    VEHICLE_DEFAULT_VIEW,
    VEHICLE_FRONT_VIEW,
    VIEW_LABELS,
    VIEW_MODES,
    cycle_delta,
    default_view_for,
    front_view_for,
    validate_view_mode,
    view_index,
)


def test_view_modes_are_unique_and_labelled():
    assert len(VIEW_MODES) == 4
    assert len(set(VIEW_MODES)) == len(VIEW_MODES)
    assert set(VIEW_LABELS) == set(VIEW_MODES)
    assert INITIAL_STREAM_INDEX == 0
    assert VIEW_MODES[INITIAL_STREAM_INDEX] == "rover_front"


def test_validate_view_mode_accepts_known_modes():
    for mode in VIEW_MODES:
        assert validate_view_mode(mode) == mode


@pytest.mark.parametrize("mode", ["moon", "", None, 3])
def test_validate_view_mode_rejects_unknown_modes(mode):
    with pytest.raises(ValueError, match="unknown view mode"):
        validate_view_mode(mode)


def test_view_index_matches_the_cycle_order():
    assert [view_index(mode) for mode in VIEW_MODES] == [0, 1, 2, 3]


def test_front_view_for_maps_both_vehicles():
    assert VEHICLE_FRONT_VIEW == {
        "rover": "rover_front",
        "drone": "drone_front",
    }
    assert front_view_for("rover") == "rover_front"
    assert front_view_for("drone") == "drone_front"
    assert set(VEHICLE_FRONT_VIEW.values()) <= set(VIEW_MODES)


@pytest.mark.parametrize("vehicle", ["boat", "", None, 1])
def test_front_view_for_rejects_unknown_vehicles(vehicle):
    with pytest.raises(ValueError, match="unknown vehicle"):
        front_view_for(vehicle)


def test_default_view_for_maps_both_vehicles_to_the_chase_camera():
    assert VEHICLE_DEFAULT_VIEW == {
        "rover": "rover_chase",
        "drone": "drone_chase",
    }
    assert default_view_for("rover") == "rover_chase"
    assert default_view_for("drone") == "drone_chase"
    assert set(VEHICLE_DEFAULT_VIEW.values()) <= set(VIEW_MODES)


@pytest.mark.parametrize("vehicle", ["boat", "", None, 1])
def test_default_view_for_rejects_unknown_vehicles(vehicle):
    with pytest.raises(ValueError, match="unknown vehicle"):
        default_view_for(vehicle)


def test_cycle_delta_steps_forward_around_the_ring():
    assert cycle_delta(0, 0) == 0
    assert cycle_delta(0, 2) == 2
    assert cycle_delta(2, 0) == 2
    assert cycle_delta(3, 0) == 1


def test_cycle_delta_rejects_out_of_range_indices():
    with pytest.raises(ValueError, match="out of range"):
        cycle_delta(0, 4)
    with pytest.raises(ValueError, match="out of range"):
        cycle_delta(-1, 0)
    with pytest.raises(ValueError):
        cycle_delta(0.0, 1)
