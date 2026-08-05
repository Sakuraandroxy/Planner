"""Tests for reachable evaluation goal selection."""

from eval.goal_geometry import object_target_position, success_goal_position


def test_end_is_default_success_goal_and_object_center_is_retained():
    mark = {
        "end": [10.0, 2.0, -4.0],
        "target": {"position": [20.0, 2.0, 8.0]},
    }

    assert success_goal_position(mark) == [10.0, 2.0, -4.0]
    assert object_target_position(mark) == [20.0, 2.0, 8.0]
    assert success_goal_position(mark, "target") == [20.0, 2.0, 8.0]


def test_missing_end_falls_back_to_object_target():
    mark = {"target": {"position": [3.0, 4.0, 5.0]}}

    assert success_goal_position(mark, "end") == [3.0, 4.0, 5.0]
