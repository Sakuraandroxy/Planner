import math

import pytest

from planner.adapters.airsim.camera_model import intrinsics_from_horizontal_fov, quaternion_to_rotation


def test_camera_model_preserves_arbitrary_pitch():
    pitch = math.radians(-45)
    rotation = quaternion_to_rotation(0, math.sin(pitch / 2), 0, math.cos(pitch / 2))
    optical_axis = (rotation[0][0], rotation[1][0], rotation[2][0])
    assert optical_axis[0] == pytest.approx(math.sqrt(0.5))
    assert abs(optical_axis[2]) == pytest.approx(math.sqrt(0.5))
    assert intrinsics_from_horizontal_fov(1920, 1080, 90).fx == pytest.approx(960)

