import numpy as np
import pytest

from planner.domain.depth import FAR_VALID_PIXEL_VALUE, SHARED_DEPTH_MAPPING
from planner.domain.depth_visualization import render_depth


def test_legacy_mapping_is_near_bright_and_black_out_of_range():
    depth = np.array([[1.0, 10.0, 0.2, 300.0]], dtype=np.float32)
    image = render_depth(depth, 0.5, 200.0)
    assert image.shape == (1, 4, 3)
    assert image.dtype == np.uint8
    gray = image[..., 0]
    assert gray[0, 0] > gray[0, 1] > 0
    assert gray[0, 2] == 0 and gray[0, 3] == 0
    assert np.array_equal(image[..., 0], image[..., 1])
    assert np.array_equal(image[..., 0], image[..., 2])
    # The metric source is never modified by the display mapping.
    assert depth[0, 0] == pytest.approx(1.0)


def test_legacy_mapping_keeps_invalid_depth_black():
    depth = np.array([[np.nan, np.inf]], dtype=np.float32)
    assert np.all(render_depth(depth, 0.5, 200.0) == 0)


def test_shared_mapping_uses_log1p_and_invalid_sentinel():
    depth = np.array([[0.0, 5.0, 200.0, 70000.0]], dtype=np.float32)
    image = render_depth(depth, 0.0, 200.0, SHARED_DEPTH_MAPPING)
    gray = image[..., 0]
    assert gray[0, 0] == 0
    assert gray[0, 1] > gray[0, 2]
    assert gray[0, 2] == FAR_VALID_PIXEL_VALUE
    assert gray[0, 3] == 0


def test_unknown_mapping_is_rejected():
    with pytest.raises(ValueError):
        render_depth(np.ones((2, 2)), 0.5, 200.0, "unknown_mapping")