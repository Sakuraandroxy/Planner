import numpy as np
import pytest

from planner.adapters.model_visual_inputs import aligned_depth_image
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import WorldPose
from planner.errors import ObservationError


def _observation(*, depth_intrinsics=None, depth_image=None, depth_m=None,
                 with_depth_image=True) -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(8, 6, 90.0)
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        rgb=np.zeros((6, 8, 3), dtype=np.uint8),
        depth_m=np.full((6, 8), 5.0, dtype=np.float32) if depth_m is None else depth_m,
        depth_image=(
            (np.full((6, 8, 3), 128, dtype=np.uint8) if depth_image is None else depth_image)
            if with_depth_image else None
        ),
        intrinsics=intrinsics,
        camera_matrix=np.eye(4),
        depth_intrinsics=depth_intrinsics,
    )


def test_aligned_depth_image_reuses_the_registered_visualization():
    observation = _observation()
    assert aligned_depth_image(observation) is observation.depth_image


def test_aligned_depth_image_requires_a_visualization():
    with pytest.raises(ObservationError):
        aligned_depth_image(_observation(with_depth_image=False))


def test_aligned_depth_image_requires_matching_dimensions():
    smaller = CameraIntrinsics.from_horizontal_fov(4, 4, 90.0)
    with pytest.raises(ObservationError):
        aligned_depth_image(_observation(depth_intrinsics=smaller))


def test_aligned_depth_image_rejects_invalid_depth_calibration():
    bad = CameraIntrinsics(8, 6, 10.0, 10.0, 3.5, 2.5, 0.0)
    with pytest.raises(ObservationError):
        aligned_depth_image(_observation(depth_intrinsics=bad))