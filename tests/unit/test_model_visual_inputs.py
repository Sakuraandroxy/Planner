from dataclasses import replace

import numpy as np
import pytest
from PIL import Image

from planner.adapters.airsim.camera_model import intrinsics_from_horizontal_fov
from planner.adapters.model_visual_inputs import aligned_depth_image
from planner.errors import ObservationError
from tests.fakes.visual_goal import wall_observation


def asymmetric_observation():
    observation = wall_observation(size=(192, 108), depth_size=(64, 64))
    pixels = np.full((64, 64, 3), 20, dtype=np.uint8)
    # This near patch projects to RGB (0.5, 0.25), not depth (0.5, 0.25).
    pixels[22:25, 30:35] = 240
    return replace(observation, depth_image=Image.fromarray(pixels))


def test_different_aspect_aligns_asymmetric_scene_without_changing_raw_observation():
    observation = asymmetric_observation()
    original_image = observation.depth_image.tobytes()
    original_depth = observation.depth_meters.copy()
    aligned = aligned_depth_image(observation)
    assert aligned.size == (64, 36)
    assert aligned.getpixel((32, 9)) == (240, 240, 240)
    assert aligned.getpixel((32, 27)) == (20, 20, 20)
    assert observation.depth_image.getpixel((32, 16)) == (20, 20, 20)
    assert observation.depth_image.tobytes() == original_image
    np.testing.assert_array_equal(observation.depth_meters, original_depth)


def test_matching_view_reuses_original_depth_image_even_with_different_resolution():
    observation = wall_observation(size=(192, 108), depth_size=(64, 36))
    assert aligned_depth_image(observation) is observation.depth_image


def test_narrower_depth_fov_leaves_unsupported_rgb_rays_black():
    observation = wall_observation(size=(128, 128), depth_size=(64, 64))
    observation = replace(observation, depth_intrinsics=intrinsics_from_horizontal_fov(64, 64, 60),
                          depth_image=Image.new("RGB", (64, 64), (77, 77, 77)))
    aligned = aligned_depth_image(observation)
    assert aligned.getpixel((32, 32)) == (77, 77, 77)
    assert aligned.getpixel((0, 32)) == (0, 0, 0)
    assert aligned.getpixel((63, 32)) == (0, 0, 0)


def test_nearest_neighbor_preserves_depth_edges_and_invalid_pixels():
    observation = wall_observation(size=(128, 128), depth_size=(64, 64))
    pixels = np.zeros((64, 64, 3), dtype=np.uint8)
    pixels[:, 32:] = 255
    observation = replace(observation, depth_intrinsics=intrinsics_from_horizontal_fov(64, 64, 60),
                          depth_image=Image.fromarray(pixels))
    values = np.asarray(aligned_depth_image(observation))
    assert set(np.unique(values)) == {0, 255}
    assert values[32, 16, 0] == 0 and values[32, 40, 0] == 255


@pytest.mark.parametrize("change", ["missing", "focal", "nonnumeric", "principal", "dimensions", "image", "float_grid"])
def test_invalid_alignment_metadata_fails_explicitly(change):
    observation = wall_observation()
    k = observation.depth_intrinsics
    if change == "missing":
        observation = replace(observation, depth_intrinsics=None)
    elif change == "focal":
        observation = replace(observation, depth_intrinsics=replace(k, fx=float("nan")))
    elif change == "nonnumeric":
        observation = replace(observation, depth_intrinsics=replace(k, fy=None))
    elif change == "principal":
        observation = replace(observation, depth_intrinsics=replace(k, cy=float("inf")))
    elif change == "dimensions":
        observation = replace(observation, depth_intrinsics=replace(k, width=0))
    elif change == "image":
        observation = replace(observation, depth_image=Image.new("RGB", (64, 64)))
    else:
        observation = replace(observation, depth_meters=np.ones((64, 64)))
    with pytest.raises(ObservationError):
        aligned_depth_image(observation)
