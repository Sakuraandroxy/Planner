import pytest

from planner.domain.pose import WorldPose
from planner.services.trajectory_transform import world_pose_delta_in_reference_frame


@pytest.mark.parametrize(
    ("start", "end", "expected"),
    [
        (WorldPose(0, 0, 0, 0), WorldPose(3, 2, 1, 10), (3, 2, 1, 10)),
        (WorldPose(0, 0, 0, 90), WorldPose(0, 3, -2, 90), (3, 0, -2, 0)),
        (WorldPose(0, 0, 0, 90), WorldPose(-4, 0, 2, 90), (0, 4, 2, 0)),
        (WorldPose(0, 0, 0, 170), WorldPose(0, 0, 0, -170), (0, 0, 0, 20)),
        (WorldPose(1, 2, 3, 45), WorldPose(1, 2, 3, 45), (0, 0, 0, 0)),
    ],
)
def test_world_pose_delta_in_reference_frame(start, end, expected):
    delta = world_pose_delta_in_reference_frame(start, end)
    assert (delta.dx, delta.dy, delta.dz, delta.dyaw_deg) == pytest.approx(expected)
