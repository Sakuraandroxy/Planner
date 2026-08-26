"""Focused tests for detector-independent down-depth roof geometry."""

from __future__ import annotations

import numpy as np

from agent.functions.perception.roof_plane import estimate_down_roof_plane


def _horizontal_plane_depth(clearance_m: float, size: int = 256, fov_deg: float = 90.0):
    """Build radial DepthPerspective values for a horizontal nadir plane."""
    rows, cols = np.mgrid[0:size, 0:size]
    focal = size / (2.0 * np.tan(np.deg2rad(fov_deg) * 0.5))
    u = (cols + 0.5 - size * 0.5) / focal
    v = (rows + 0.5 - size * 0.5) / focal
    return float(clearance_m) * np.sqrt(1.0 + u * u + v * v)


def _multi_level_complex_roof_depth(size: int = 256, fov_deg: float = 90.0):
    """Three centre height levels, with one stable patch at exact nadir."""

    rows, cols = np.mgrid[0:size, 0:size]
    focal = size / (2.0 * np.tan(np.deg2rad(fov_deg) * 0.5))
    u = (cols + 0.5 - size * 0.5) / focal
    v = (rows + 0.5 - size * 0.5) / focal
    normalized_x = (cols + 0.5) / size - 0.5
    normalized_y = (rows + 0.5) / size - 0.5
    broad_center = (np.abs(normalized_x) <= 0.22) & (np.abs(normalized_y) <= 0.22)
    clearance = np.full((size, size), 20.0, dtype=float)
    clearance[broad_center & (normalized_x < -0.08)] = 15.0
    clearance[broad_center & (normalized_x > 0.08)] = 25.0
    return clearance * np.sqrt(1.0 + u * u + v * v)


def test_full_frame_roof_is_recovered_without_bbox():
    depth = _horizontal_plane_depth(20.0)

    estimate = estimate_down_roof_plane(
        depth,
        observer_world=[10.0, 20.0, -50.0],
        observer_yaw_deg=35.0,
        config={"ABOVE_ROOF_DEPTH_STRIDE": 8},
        sim_config={"DOWN_FOV": 90.0},
    )

    assert estimate.valid
    assert estimate.full_frame
    assert abs(float(estimate.roof_z_world) - (-30.0)) < 0.05
    assert abs(float(estimate.center_depth_m) - 20.0) < 0.1
    assert estimate.coverage_ratio > 0.95
    assert estimate.center_support_ratio > 0.95
    assert estimate.confidence > 0.9
    assert estimate.bounds_world is not None


def test_sparse_invalid_down_depth_is_rejected():
    depth = np.full((256, 256), np.nan, dtype=float)
    depth[120:136, 120:136] = 15.0

    estimate = estimate_down_roof_plane(
        depth,
        observer_world=[0.0, 0.0, -30.0],
        observer_yaw_deg=0.0,
        config={"ABOVE_ROOF_DEPTH_STRIDE": 4, "ABOVE_ROOF_MIN_VALID_RATIO": 0.35},
        sim_config={"DOWN_FOV": 90.0},
    )

    assert not estimate.valid
    assert estimate.reason == "insufficient_valid_down_depth"


def test_complex_multilevel_roof_uses_stable_local_nadir_patch():
    estimate = estimate_down_roof_plane(
        _multi_level_complex_roof_depth(),
        observer_world=[10.0, 20.0, -50.0],
        observer_yaw_deg=15.0,
        config={
            "ABOVE_ROOF_DEPTH_STRIDE": 4,
            "ABOVE_ROOF_MIN_CENTER_SUPPORT_RATIO": 0.65,
            "ABOVE_ROOF_LOCAL_PATCH_ENABLED": True,
            "ABOVE_ROOF_LOCAL_CENTER_FRACTION": 0.08,
            "ABOVE_ROOF_LOCAL_MIN_SUPPORT_RATIO": 0.55,
        },
        sim_config={"DOWN_FOV": 90.0},
    )

    assert estimate.valid
    assert estimate.reason == "local_horizontal_patch_under_drone"
    assert not estimate.full_frame
    assert abs(float(estimate.roof_z_world) - (-30.0)) < 0.05
    assert estimate.center_support_ratio > 0.95
    assert estimate.confidence >= 0.45


def test_complex_multilevel_roof_without_local_fallback_is_rejected():
    estimate = estimate_down_roof_plane(
        _multi_level_complex_roof_depth(),
        observer_world=[0.0, 0.0, -50.0],
        observer_yaw_deg=0.0,
        config={
            "ABOVE_ROOF_DEPTH_STRIDE": 4,
            "ABOVE_ROOF_MIN_CENTER_SUPPORT_RATIO": 0.65,
            "ABOVE_ROOF_LOCAL_PATCH_ENABLED": False,
        },
        sim_config={"DOWN_FOV": 90.0},
    )

    assert not estimate.valid
    assert estimate.reason == "down_center_not_supported_by_one_plane"


def test_non_horizontal_depth_surface_is_rejected():
    depth = _horizontal_plane_depth(20.0)
    # A strong left-to-right range ramp is not a horizontal roof in world NED.
    ramp = np.linspace(0.65, 1.35, depth.shape[1], dtype=float)
    depth = depth * ramp[None, :]

    estimate = estimate_down_roof_plane(
        depth,
        observer_world=[0.0, 0.0, -50.0],
        observer_yaw_deg=0.0,
        config={
            "ABOVE_ROOF_DEPTH_STRIDE": 4,
            "ABOVE_ROOF_MAX_Z_MAD_M": 0.45,
            "ABOVE_ROOF_MAX_NORMAL_ERROR_DEG": 10.0,
        },
        sim_config={"DOWN_FOV": 90.0},
    )

    assert not estimate.valid
    assert estimate.reason in {
        "down_center_not_supported_by_one_plane",
        "roof_plane_z_too_noisy",
        "dominant_down_surface_not_horizontal",
    }
