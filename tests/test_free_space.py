import math

import numpy as np
import pytest

from planner.adapters.trajectory_planner.prompts.free_navigation import (
    FREE_NAVIGATION_SYSTEM_PROMPT,
    FreeNavigationTrajectoryPrompt,
)
from planner.domain.depth import INVALID_DEPTH_SENTINEL_M
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import WorldPose
from planner.services.free_space import (
    FreeSpaceProfileSettings,
    measure_free_space,
)

K = CameraIntrinsics.from_horizontal_fov(640, 480, 90.0)
COLUMNS = np.arange(K.width, dtype=np.float64)[None, :]
ROWS = np.arange(K.height, dtype=np.float64)[:, None]
RIGHT_RATIO = (COLUMNS - K.cx) / K.fx
DOWN_RATIO = (ROWS - K.cy) / K.fy


def _synthetic_depth(
    *,
    camera_height_m: float = 1.5,
    slope: float = 0.0,
    walls: tuple[tuple[float, float, float, float], ...] = (),
    far_m: float = 1000.0,
) -> np.ndarray:
    """Optical-axis depth for flat or sloped ground plus vertical walls.

    Each wall is (bearing_center_deg, half_width_deg, distance_m, height_m),
    perpendicular to the forward axis and standing on the local ground.
    """
    with np.errstate(divide="ignore", invalid="ignore"):
        forward = camera_height_m / (DOWN_RATIO + slope)
    forward = np.where(np.isfinite(forward) & (forward > 0), forward, far_m)
    forward = np.broadcast_to(forward, (K.height, K.width)).copy()
    bearing = np.degrees(np.arctan2(RIGHT_RATIO, 1.0))
    for center, half_width, distance, height in walls:
        ground_down = camera_height_m - slope * distance
        down = distance * DOWN_RATIO
        hit = (
            (np.abs(bearing - center) <= half_width)
            & (forward > distance)
            & (down <= ground_down)
            & (down >= ground_down - height)
        )
        forward = np.where(hit, distance, forward)
    return forward


def _observation(
    depth: np.ndarray, *, depth_kind: str = "optical_depth"
) -> Observation:
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        rgb=np.zeros((K.height, K.width, 3), dtype=np.uint8),
        depth_m=depth.astype(np.float32),
        intrinsics=K,
        camera_matrix=np.eye(4),
        depth_kind=depth_kind,
    )


def _ahead(profile):
    return min(profile.sectors, key=lambda sector: abs(sector.bearing_deg))


def _sector(profile, bearing):
    return next(
        sector
        for sector in profile.sectors
        if math.isclose(sector.bearing_deg, bearing)
    )


def test_flat_ground_without_obstacles_is_clear():
    profile = measure_free_space(_observation(_synthetic_depth()))
    assert all(sector.ground_available for sector in profile.sectors)
    assert all(sector.distance_m is None for sector in profile.sectors)
    assert profile.nearest_ahead_m is None
    assert profile.clear_window_deg == (-40.0, 40.0)


def test_wall_ahead_reports_the_measured_distance():
    depth = _synthetic_depth(walls=((0.0, 6.0, 10.0, 2.0),))
    profile = measure_free_space(_observation(depth))
    ahead = _ahead(profile)
    assert ahead.distance_m == pytest.approx(10.0, abs=0.2)
    assert ahead.forward_m == pytest.approx(10.0, abs=0.2)
    assert abs(ahead.right_m) < 0.5
    assert profile.nearest_ahead_m == pytest.approx(10.0, abs=0.2)
    assert _sector(profile, 30.0).distance_m is None


def test_right_side_guardrail_reports_positive_bearing():
    depth = _synthetic_depth(walls=((20.0, 4.0, 12.0, 2.0),))
    profile = measure_free_space(_observation(depth))
    right = _sector(profile, 20.0)
    assert right.distance_m == pytest.approx(12.5, abs=0.3)
    assert right.forward_m == pytest.approx(12.0, abs=0.2)
    assert right.right_m > 3.0
    assert _sector(profile, -20.0).distance_m is None
    assert profile.clear_window_deg is not None


def test_sloped_ground_is_self_calibrating():
    flat_slope = measure_free_space(_observation(_synthetic_depth(slope=0.04)))
    assert flat_slope.nearest_ahead_m is None

    wall = _synthetic_depth(
        slope=0.04, walls=((0.0, 6.0, 15.0, 2.0),)
    )
    profile = measure_free_space(_observation(wall))
    assert _ahead(profile).distance_m == pytest.approx(15.0, abs=0.2)


def test_invalid_depth_pixels_are_ignored():
    depth = _synthetic_depth(walls=((0.0, 6.0, 10.0, 2.0),))
    depth[::7, ::5] = np.nan
    depth[1::11, 3::13] = 0.0
    depth[2::13, 5::17] = INVALID_DEPTH_SENTINEL_M
    profile = measure_free_space(_observation(depth))
    assert _ahead(profile).distance_m == pytest.approx(10.0, abs=0.2)


def test_all_invalid_depth_is_unknown_not_clear():
    depth = np.zeros((K.height, K.width), dtype=np.float32)
    profile = measure_free_space(_observation(depth))
    assert all(not sector.ground_available for sector in profile.sectors)
    assert all(sector.distance_m is None for sector in profile.sectors)
    assert profile.nearest_ahead_m is None
    assert profile.clear_window_deg is None


def test_ray_range_depth_matches_optical_geometry():
    optical = _synthetic_depth(walls=((0.0, 6.0, 10.0, 2.0),))
    norm = np.sqrt(1.0 + RIGHT_RATIO ** 2 + DOWN_RATIO ** 2)
    profile = measure_free_space(
        _observation(optical * norm, depth_kind="ray_range")
    )
    assert _ahead(profile).distance_m == pytest.approx(10.0, abs=0.2)


def test_free_navigation_prompt_carries_measured_profile():
    depth = _synthetic_depth(walls=((20.0, 4.0, 12.0, 2.0),))
    observation = _observation(depth)
    prompt = FreeNavigationTrajectoryPrompt().build_user_prompt(
        "follow the road", observation, 5, initial_pose=observation.pose
    )
    assert "Metric free-space profile" in prompt
    assert "+20.0 deg: 12.5 m (forward 12.0 m, right +3.5 m)" in prompt
    assert "Longest clear bearing window" in prompt
    assert "Use these measured distances and bearings" in prompt
    assert prompt.index("Metric free-space profile") > prompt.index(
        "Generate the next short trajectory."
    )


def test_free_navigation_system_prompt_requires_measured_clearance():
    assert "A measured metric free-space profile" in FREE_NAVIGATION_SYSTEM_PROMPT
    assert "Treat its distances and bearings as measured values" in (
        FREE_NAVIGATION_SYSTEM_PROMPT
    )
    assert "turns\naccumulate across the five actions" in (
        FREE_NAVIGATION_SYSTEM_PROMPT
    )
    assert "five actions with -8 degrees" in FREE_NAVIGATION_SYSTEM_PROMPT


@pytest.mark.parametrize(
    "kwargs",
    [
        {"sector_width_deg": 0.0},
        {"max_range_m": -1.0},
        {"distance_bin_m": float("nan")},
        {"min_points_per_bin": 0},
        {"min_obstacle_pixels": 0},
        {"ground_height_percentile": 101.0},
        {"obstacle_distance_percentile": -1.0},
        {"min_height_above_ground_m": 0.0},
        {"min_height_above_ground_m": 3.0, "max_height_above_ground_m": 1.0},
    ],
)
def test_profile_settings_are_validated(kwargs):
    with pytest.raises(ValueError):
        FreeSpaceProfileSettings(**kwargs)