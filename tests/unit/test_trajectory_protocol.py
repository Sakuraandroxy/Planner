import pytest

from planner.adapters.trajectory_planner.response_parser import parse_trajectory
from planner.domain.pose import WorldPose
from planner.services.trajectory_transform import relative_to_world


def test_parser_requires_five_four_dimensional_points():
    trajectory = parse_trajectory("[[1,0,0,90],[1,0,0,0],[1,0,0,0],[1,0,0,0],[1,0,0,0]]")
    assert len(trajectory.points) == 5
    with pytest.raises(Exception):
        parse_trajectory("[[1,0,0],[1,0,0],[1,0,0],[1,0,0],[1,0,0]]")


def test_world_transform_accumulates_yaw():
    trajectory = parse_trajectory("[[1,0,0,90],[1,0,0,90],[1,0,0,0],[1,0,0,0],[1,0,0,0]]")
    world = relative_to_world(trajectory, WorldPose(10, 20, -5, 0))
    assert world.poses[0] == WorldPose(11, 20, -5, 90)
    assert world.poses[1].x == pytest.approx(11)
    assert world.poses[1].y == pytest.approx(21)
    assert world.poses[2].x == pytest.approx(10)

