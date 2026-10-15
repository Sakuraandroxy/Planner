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


@pytest.mark.parametrize("fov", [0, -1, 180, 181, float("nan"), float("inf"), True, None])
def test_invalid_fov_is_rejected_instead_of_clamped(fov):
    with pytest.raises(ValueError, match="FOV"):
        intrinsics_from_horizontal_fov(128, 128, fov)


@pytest.mark.parametrize("size", [(0, 128), (128, -1), (1.5, 128), (True, 128)])
def test_invalid_camera_dimensions_are_rejected(size):
    with pytest.raises(ValueError, match="dimensions"):
        intrinsics_from_horizontal_fov(*size, 90)


@pytest.mark.parametrize("q", [(0, 0, 0, 0), (0, 0, 0, 2), (float("nan"), 0, 0, 1), (None, 0, 0, 1)])
def test_invalid_camera_quaternions_are_rejected(q):
    with pytest.raises(ValueError, match="quaternion"):
        quaternion_to_rotation(*q)


def test_small_unit_quaternion_roundoff_is_normalized():
    assert quaternion_to_rotation(0, 0, 0, 1.00001) == ((1, 0, 0), (0, 1, 0), (0, 0, 1))

