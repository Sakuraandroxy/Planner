import pytest

from planner.domain import RelativePoseDelta, WorldPose


def test_delta_without_vertical_drops_dz():
    delta = RelativePoseDelta(dx=1.0, dy=2.0, dz=-3.0, dyaw_deg=4.0)
    flat = delta.without_vertical()
    assert (flat.dx, flat.dy, flat.dz, flat.dyaw_deg) == (1.0, 2.0, 0.0, 4.0)


def test_pose_at_height_clamps_z():
    pose = WorldPose(3.0, 4.0, -5.0, 90.0)
    clamped = pose.at_height(0.25)
    assert clamped == WorldPose(3.0, 4.0, 0.25, 90.0)