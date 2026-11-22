"""轨迹变换与校验的直接单测：相对/世界互转、带符号偏航元数据、1e-6 容差带。

覆盖 planner/algorithms/trajectory_transform.py 的三个函数与
planner/algorithms/trajectory_validation.py 的两个校验入口。
"""

import math

import pytest

from planner.algorithms import (
    TrajectoryValidator,
    relative_to_world,
    world_pose_delta_in_reference_frame,
    world_to_relative,
)
from planner.domain import (
    RelativePoseDelta,
    RelativeTrajectory,
    WorldPose,
    WorldTrajectory,
)
from planner.errors import ProtocolError


def _pose(x=0.0, y=0.0, z=0.0, yaw=0.0) -> WorldPose:
    return WorldPose(x, y, z, yaw)


def test_reference_frame_delta_uses_start_heading():
    delta = world_pose_delta_in_reference_frame(
        _pose(1.0, 2.0, 0.0, 90.0), _pose(1.0, 3.0, 0.0, 90.0)
    )
    assert delta.dx == pytest.approx(1.0)
    assert delta.dy == pytest.approx(0.0)
    assert delta.dz == pytest.approx(0.0)
    assert delta.dyaw_deg == pytest.approx(0.0)


def test_reference_frame_delta_keeps_vertical_and_wraps_yaw():
    delta = world_pose_delta_in_reference_frame(
        _pose(0.0, 0.0, 0.0, 179.0), _pose(0.0, 0.0, 1.5, -179.0)
    )
    assert delta.dz == pytest.approx(1.5)
    assert delta.dyaw_deg == pytest.approx(2.0)


def test_relative_to_world_accumulates_headings():
    world = relative_to_world(
        RelativeTrajectory(
            (RelativePoseDelta(1.0, 0.0, 0.0, 90.0), RelativePoseDelta(1.0, 0.0, 0.0, 0.0))
        ),
        _pose(),
    )
    assert [tuple(round(v, 9) for v in p.as_tuple()) for p in world.poses] == [
        (1.0, 0.0, 0.0, 90.0),
        (1.0, 1.0, 0.0, 90.0),
    ]
    assert world.start == _pose()


def test_relative_to_world_keeps_signed_yaw_metadata():
    world = relative_to_world(
        RelativeTrajectory(
            (
                RelativePoseDelta(0.0, 0.0, 0.0, 0.0),
                RelativePoseDelta(0.0, 0.0, 0.0, -180.0),
                RelativePoseDelta(0.0, 0.0, 0.0, 0.0),
            )
        ),
        _pose(),
    )
    # 元数据保留原样的 0.0 与 -180.0，不被 180 度回绕或小角吸附改写。
    assert world.yaw_deltas_deg == (0.0, -180.0, 0.0)
    assert [p.yaw_deg for p in world.poses] == [0.0, -180.0, -180.0]


def test_relative_to_world_with_no_points_returns_the_anchor():
    world = relative_to_world(RelativeTrajectory(()), _pose(3.0, -2.0, 1.0, 45.0))
    assert world.poses == ()
    assert world.yaw_deltas_deg == ()
    assert world.start == _pose(3.0, -2.0, 1.0, 45.0)


def test_world_to_relative_inverts_relative_to_world():
    start = _pose(2.0, -1.0, 0.5, 30.0)
    points = (
        RelativePoseDelta(1.0, 0.5, 0.2, 30.0),
        RelativePoseDelta(0.3, -0.4, 0.0, 15.0),
        RelativePoseDelta(0.0, 0.0, -0.1, -10.0),
    )
    round_trip = world_to_relative(relative_to_world(RelativeTrajectory(points), start))
    assert len(round_trip.points) == len(points)
    for got, want in zip(round_trip.points, points):
        assert got.dx == pytest.approx(want.dx, abs=1e-9)
        assert got.dy == pytest.approx(want.dy, abs=1e-9)
        assert got.dz == pytest.approx(want.dz, abs=1e-9)
        assert got.dyaw_deg == pytest.approx(want.dyaw_deg, abs=1e-9)


def test_world_to_relative_prefers_explicit_signed_half_turn():
    world = WorldTrajectory(_pose(), (_pose(0.0, 0.0, 0.0, 180.0),), (180.0,))
    relative = world_to_relative(world)
    assert relative.points[0].dyaw_deg == pytest.approx(180.0)


def test_world_to_relative_measures_yaw_without_metadata():
    world = WorldTrajectory(_pose(), (_pose(0.0, 0.0, 0.0, 170.0),))
    relative = world_to_relative(world)
    assert relative.points[0].dyaw_deg == pytest.approx(170.0)


def test_validator_rejects_empty_and_oversized_batches():
    validator = TrajectoryValidator(2, 5.0, 45.0)
    with pytest.raises(ProtocolError):
        validator.validate_relative(RelativeTrajectory(()))
    with pytest.raises(ProtocolError):
        validator.validate_relative(
            RelativeTrajectory(tuple(RelativePoseDelta(0.1) for _ in range(3)))
        )
    with pytest.raises(ProtocolError):
        validator.validate_world(WorldTrajectory(_pose(), ()))


def test_validator_accepts_exact_relative_limits():
    validator = TrajectoryValidator(1, 2.0, 45.0)
    validator.validate_relative(RelativeTrajectory((RelativePoseDelta(2.0, 0.0, 0.0, 45.0),)))


def test_validator_rejects_relative_step_just_over_limit():
    validator = TrajectoryValidator(1, 2.0, 45.0)
    with pytest.raises(ProtocolError):
        validator.validate_relative(
            RelativeTrajectory((RelativePoseDelta(2.0 + 1e-9, 0.0, 0.0, 0.0),))
        )
    with pytest.raises(ProtocolError):
        validator.validate_relative(
            RelativeTrajectory((RelativePoseDelta(0.0, 0.0, 0.0, 45.0 + 1e-9),))
        )


def test_validator_world_distance_absorbs_one_micrometre():
    validator = TrajectoryValidator(1, 2.0, 45.0)
    inside = WorldTrajectory(_pose(), (_pose(2.0 + 5e-7),))
    validator.validate_world(inside)
    outside = WorldTrajectory(_pose(), (_pose(2.0 + 2e-6),))
    with pytest.raises(ProtocolError):
        validator.validate_world(outside)


def test_validator_world_yaw_absorbs_one_micrometre_of_degrees():
    validator = TrajectoryValidator(1, 5.0, 1.0)
    validator.validate_world(WorldTrajectory(_pose(), (_pose(0.0, 0.0, 0.0, 1.0),)))
    with pytest.raises(ProtocolError):
        validator.validate_world(WorldTrajectory(_pose(), (_pose(0.0, 0.0, 0.0, 1.000002),)))


def test_validator_world_honours_explicit_signed_half_turn():
    pose = _pose(0.0, 0.0, 0.0, 180.0)
    TrajectoryValidator(1, 5.0, 180.0).validate_world(WorldTrajectory(_pose(), (pose,), (180.0,)))
    with pytest.raises(ProtocolError):
        TrajectoryValidator(1, 5.0, 90.0).validate_world(
            WorldTrajectory(_pose(), (pose,), (180.0,))
        )


@pytest.mark.parametrize(
    "arguments",
    [(0, 1.0, 10.0), (6, 1.0, 10.0), (1.0, 1.0, 10.0), (3, 0.0, 10.0), (3, -1.0, 10.0),
     (3, math.inf, 10.0), (3, 1.0, 181.0), (3, True, 10.0)],
)
def test_validator_constructor_rejects_unusable_limits(arguments):
    with pytest.raises(ValueError):
        TrajectoryValidator(*arguments)
