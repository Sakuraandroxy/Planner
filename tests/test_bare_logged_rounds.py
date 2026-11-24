"""Offline replay of the three planner replies logged on 4090 (planner-1124-1432.log).

The three replies are copied verbatim from the log. This test asserts what the
shipped parser and ``offsets_to_world`` make of them, with fake observations: no
engine and no model call. The log lines carry the arrays only, so the start poses
below are made up; the arrays are the recorded ones.
"""

import math

import numpy as np
import pytest

from planner.adapters.trajectory_planner.response_parser import (
    parse_translation_trajectory,
)
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.services.trajectory_transform import offsets_to_world
from planner.workflows.visual_goal_navigation.bare import _BATCH_POINTS

_IDENTITY_POSE = WorldPose(0.0, 0.0, 0.0, 0.0)

# The spacing the bare prompt now asks for: successive waypoints of one array.
SPACING_MIN_M = 0.5
SPACING_MAX_M = 2.0

# (round id, logged reply, fake observation pose, expected world points)
_ROUNDS = (
    (
        "round-1",
        "[[3.0,-1.2,-0.2],[3.0,-1.0,0.0],[2.5,-0.8,0.0]]",
        _IDENTITY_POSE,
        ((3.0, -1.2, -0.2), (3.0, -1.0, 0.0), (2.5, -0.8, 0.0)),
    ),
    (
        "round-2",
        "[[6.0,-1.5,-0.2],[6.0,-1.5,0.0],[5.0,-1.0,0.0]]",
        WorldPose(4.0, 0.5, -0.2, 90.0),
        ((5.5, 6.5, -0.4), (5.5, 6.5, -0.2), (5.0, 5.5, -0.2)),
    ),
    (
        "round-3",
        "[[2.0,-0.8,-0.2],[2.5,-1.2,-0.2],[2.5,-1.5,-0.2]]",
        WorldPose(5.5, 6.5, -0.2, 0.0),
        ((7.5, 5.7, -0.4), (8.0, 5.3, -0.4), (8.0, 5.0, -0.4)),
    ),
)


def _observation(pose: WorldPose, capture_id: str = "logged-capture") -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(640, 480, 90.0)
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=pose,
        rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        depth_m=np.ones((480, 640), dtype=np.float32),
        depth_image=np.zeros((480, 640), dtype=np.uint8),
        intrinsics=intrinsics,
        camera_matrix=np.eye(4),
        camera_id="front",
        timestamp_ns=1_000_000,
        capture_id=capture_id,
        captured_monotonic=1.0,
    )


def _parsed(reply: str) -> RelativeTrajectory:
    return parse_translation_trajectory(reply)


@pytest.mark.parametrize(
    ("reply", "pose", "expected"), [case[1:] for case in _ROUNDS], ids=[case[0] for case in _ROUNDS]
)
def test_logged_reply_becomes_the_expected_world_points(reply, pose, expected):
    trajectory = _parsed(reply)
    assert len(trajectory.points) == 3
    batch = trajectory.points[:_BATCH_POINTS]
    assert len(batch) == 3
    poses = offsets_to_world(RelativeTrajectory(batch), pose).poses
    assert [(round(item.x, 6), round(item.y, 6), round(item.z, 6)) for item in poses] == [
        (round(x, 6), round(y, 6), round(z, 6)) for x, y, z in expected
    ]


@pytest.mark.parametrize("reply", [case[1] for case in _ROUNDS], ids=[case[0] for case in _ROUNDS])
def test_logged_reply_carries_no_heading(reply):
    trajectory = _parsed(reply)
    assert all(point.dyaw_deg == 0.0 for point in trajectory.points)
    world = offsets_to_world(trajectory, WorldPose(1.0, 2.0, 3.0, 37.5))
    assert [item.yaw_deg for item in world.poses] == [37.5] * 3


@pytest.mark.parametrize(
    ("reply", "pose", "expected"), [case[1:] for case in _ROUNDS], ids=[case[0] for case in _ROUNDS]
)
def test_logged_reply_is_measured_from_the_observation_pose(reply, pose, expected):
    """Every point is an offset from the same pose, so points may coincide.

    This is what makes the recorded replies degenerate rather than a chain: the
    three points of one reply stay within 1.3 m of each other while the reply's
    forward component is 2.5 m to 6.0 m.
    """
    observation = _observation(pose)
    poses = offsets_to_world(_parsed(reply), observation.pose).poses
    spread = max(
        math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
        for index, a in enumerate(poses)
        for b in poses[index + 1:]
    )
    forward = max(abs(point[0]) for point in expected)
    assert forward >= 2.5
    assert spread <= 1.3


@pytest.mark.parametrize(
    ("reply", "pose", "expected"), [case[1:] for case in _ROUNDS], ids=[case[0] for case in _ROUNDS]
)
def test_logged_reply_is_tighter_than_the_spacing_the_prompt_asks_for(reply, pose, expected):
    """Legs between the offset points are how the spacing rule is measured.

    The rule is pose-independent: a rigid transform keeps the distances between
    offsets unchanged, so the recorded replies can be checked without the poses.
    """
    legs = [math.dist(expected[index], expected[index + 1]) for index in range(len(expected) - 1)]
    assert min(legs) < SPACING_MIN_M
    assert max(legs) <= SPACING_MAX_M
