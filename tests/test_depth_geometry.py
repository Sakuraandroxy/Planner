import math

import numpy as np
import pytest

from tests.support import ue_camera_matrix
from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import WorldPose
from planner.domain.visual_goal import GoalObservation, GoalVisibility

BOX = (0.4, 0.4, 0.6, 0.6)


def _observation(
    pose: WorldPose = WorldPose(0.0, 0.0, 0.0, 0.0),
    depth_value: float = 10.0,
    *,
    depth_kind: str = "optical_depth",
) -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(640, 480, 90.0)
    depth = np.full((480, 640), depth_value, dtype=np.float32)
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=pose,
        rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        depth_m=depth,
        intrinsics=intrinsics,
        # UE camera pose: world z down -> UE z up.
        camera_matrix=ue_camera_matrix(pose.x, pose.y, -pose.z, pose.yaw_deg),
        depth_kind=depth_kind,
    )


def _goal(
    points: tuple[tuple[float, float], ...] | None = None,
    *,
    bbox: tuple[float, float, float, float] | None = BOX,
) -> GoalObservation:
    return GoalObservation(
        GoalVisibility.VISIBLE,
        bbox,
        None,
        "target",
        "manual",
        surface_points=points,
    )


def _geometry(**kwargs) -> DepthGeometry:
    return DepthGeometry(0.5, True, **kwargs)


def test_center_sample_back_projects_onto_optical_axis():
    goal = _geometry().estimate(_observation(), _goal(((0.5, 0.5),)))
    assert goal.reliable
    # (0.5, 0.5) samples pixel (320, 240), half a pixel from the principal point.
    assert goal.world_point == pytest.approx(
        (10.0, 0.015625, 0.015625), abs=1e-6
    )
    assert goal.distance_m == pytest.approx(
        math.dist((10.0, 0.015625, 0.015625), (0.0, 0.0, 0.0)), abs=1e-6
    )


def test_off_axis_sample_expands_lateral_offset():
    goal = _geometry().estimate(_observation(), _goal(((0.6, 0.5),)))
    assert goal.reliable
    # 0.6 across the image is xn = 0.2015625, so +2.0156 m lateral at 10 m depth.
    assert goal.world_point[0] == pytest.approx(10.0, abs=1e-6)
    assert goal.world_point[1] == pytest.approx(2.015625, abs=1e-6)
    # Distance is three-dimensional from the vehicle, including the 0.015625 m z offset.
    assert goal.distance_m == pytest.approx(
        math.dist((10.0, 2.015625, 0.015625), (0.0, 0.0, 0.0)), abs=1e-6
    )


def test_yaw_rotates_back_projected_point():
    goal = _geometry().estimate(
        _observation(WorldPose(0.0, 0.0, 0.0, 90.0)), _goal(((0.5, 0.5),))
    )
    # Yaw 90 rotates the body-frame point (10, 0.015625, -0.015625).
    assert goal.world_point == pytest.approx(
        (-0.015625, 10.0, 0.015625), abs=1e-6
    )


def test_invalid_depth_is_unreliable():
    goal = _geometry().estimate(
        _observation(depth_value=0.0), _goal(((0.5, 0.5),))
    )
    assert not goal.reliable
    assert "depth is invalid" in goal.reason


def test_too_near_depth_is_unreliable():
    geometry = DepthGeometry(1.0, True)
    goal = geometry.estimate(
        _observation(depth_value=0.5), _goal(((0.5, 0.5),))
    )
    assert not goal.reliable


def test_nearest_valid_sample_is_selected():
    goal = _geometry().estimate(
        _observation(), _goal(((0.5, 0.5), (0.6, 0.5)))
    )
    assert goal.reliable
    assert len(goal.surface_samples) == 2
    # Uniform 10 m optical depth: the centre ray is shorter than the off-axis ray.
    assert goal.distance_m == pytest.approx(10.0, abs=1e-3)


def test_empty_surface_points_is_unreliable():
    # Explicit empty model points never fall back to the bbox centre.
    goal = _geometry().estimate(_observation(), _goal(()))
    assert not goal.reliable
    assert "no visible surface points" in goal.reason


def test_missing_bbox_returns_unreliable():
    goal = _geometry().estimate(_observation(), _goal(((0.5, 0.5),), bbox=None))
    assert not goal.reliable


def test_missing_detection_returns_unreliable():
    detection = GoalObservation(
        GoalVisibility.NOT_VISIBLE, None, None, "target", "manual"
    )
    goal = _geometry().estimate(_observation(), detection)
    assert not goal.reliable


def test_calibration_must_be_verified():
    geometry = DepthGeometry(0.5, False)
    goal = geometry.estimate(_observation(), _goal(((0.5, 0.5),)))
    assert not goal.reliable
    assert "calibration" in goal.reason


def test_declared_ray_range_requires_matching_depth_kind():
    geometry = _geometry(depth_is_range=True)
    goal = geometry.estimate(_observation(), _goal(((0.5, 0.5),)))
    assert not goal.reliable
    assert "ray_range" in goal.reason


def test_ray_range_depth_produces_radial_distance():
    geometry = _geometry(depth_is_range=True)
    goal = geometry.estimate(
        _observation(depth_value=10.0, depth_kind="ray_range"),
        _goal(((0.5, 0.5),)),
    )
    assert goal.reliable
    assert goal.distance_m == pytest.approx(10.0, abs=1e-6)