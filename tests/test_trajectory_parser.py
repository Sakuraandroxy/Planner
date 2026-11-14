import pytest

from planner.adapters.trajectory_planner import parse_trajectory
from planner.errors import ProtocolError


def test_parse_five_waypoints():
    trajectory = parse_trajectory(
        "[[0.5,0.1,-0.2,5.0],[0.6,0.1,-0.1,4.0],[0.5,0.0,0.0,2.0],"
        "[0.3,-0.1,0.0,-1.0],[0.1,0.0,0.0,0.0]]"
    )
    assert len(trajectory.points) == 5
    assert trajectory.points[0].dx == pytest.approx(0.5)
    assert trajectory.points[0].dyaw_deg == pytest.approx(5.0)


def test_parse_extracts_array_from_prose_and_fences():
    text = (
        "Here is the plan:\n```json\n"
        "[[1,0,0,0],[1,0,0,0],[1,0,0,0],[1,0,0,0],[1,0,0,0]]\n```"
    )
    assert len(parse_trajectory(text).points) == 5


def test_parse_accepts_fewer_points():
    assert len(parse_trajectory("[[1,0,0,0]]").points) == 1


def test_parse_rejects_too_many_points():
    text = "[" + ",".join(["[1,0,0,0]"] * 6) + "]"
    with pytest.raises(ProtocolError):
        parse_trajectory(text)


def test_parse_rejects_wrong_waypoint_width():
    with pytest.raises(ProtocolError):
        parse_trajectory("[[1,0,0]]")


def test_parse_rejects_strings_and_booleans():
    with pytest.raises(ProtocolError):
        parse_trajectory('[[1,"0",0,0]]')
    with pytest.raises(ProtocolError):
        parse_trajectory("[[1,true,0,0]]")


def test_parse_rejects_non_finite_numbers():
    with pytest.raises(ProtocolError):
        parse_trajectory("[[1,0,0,NaN]]")
    with pytest.raises(ProtocolError):
        parse_trajectory("[[1,0,0,Infinity]]")


def test_parse_rejects_missing_array():
    with pytest.raises(ProtocolError):
        parse_trajectory("no trajectory here")