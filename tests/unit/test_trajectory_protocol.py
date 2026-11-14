import pytest

from planner.adapters.trajectory_planner.response_parser import parse_trajectory
from planner.domain.pose import RelativePoseDelta
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.errors import ProtocolError
from planner.services.trajectory_transform import relative_to_world
from planner.services.trajectory_validation import TrajectoryValidator


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


def test_validator_allows_a_single_180_degree_turn():
    validator = TrajectoryValidator(expected_points=5, max_step_m=30, max_yaw_step_deg=180)

    validator.validate_relative(RelativeTrajectory((RelativePoseDelta(0, 0, 0, 180),)))
    validator.validate_relative(RelativeTrajectory((RelativePoseDelta(0, 0, 0, -180),)))

    with pytest.raises(ProtocolError, match="exceeds max yaw step"):
        validator.validate_relative(RelativeTrajectory((RelativePoseDelta(0, 0, 0, 180.001),)))

