import numpy as np
import pytest

from tests.support import ue_camera_matrix
from planner.capabilities.targeting.association import (
    accept_position,
    bound_target_geometry,
    make_reference,
)
from planner.capabilities.targeting.bbox_goal_observation import BBoxGoalObserver
from planner.capabilities.targeting.depth_foreground import DepthForegroundFilter
from planner.capabilities.targeting.depth_surface_sampling import sample_surface_depth
from planner.capabilities.targeting.visual_goal_refresh import VisualGoalRefresh
from planner.domain.depth_foreground import DepthForegroundSettings
from planner.domain.execution import ExecutionBudget
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import WorldPose
from planner.domain.target_detection import NormalizedBoundingBox, TargetDetection
from planner.domain.target_state import TargetPositionState
from planner.domain.visual_goal import (
    GoalGeometry,
    GoalObservation,
    GoalReference,
    GoalVisibility,
    SurfacePointEstimate,
)
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.errors import BudgetExceeded


def _observation(
    pose: WorldPose = WorldPose(0.0, 0.0, 0.0, 0.0),
    *,
    capture_id: str = "capture-1",
    captured_monotonic: float = 1.0,
    timestamp_ns: int = 10,
    width: int = 8,
    height: int = 6,
) -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(width, height, 90.0)
    rgb = np.arange(height * width * 3, dtype=np.uint8).reshape(height, width, 3)
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=pose,
        rgb=rgb,
        depth_m=np.full((height, width), 10.0, dtype=np.float32),
        intrinsics=intrinsics,
        camera_matrix=ue_camera_matrix(pose.x, pose.y, -pose.z, pose.yaw_deg),
        capture_id=capture_id,
        captured_monotonic=captured_monotonic,
        timestamp_ns=timestamp_ns,
        stopped=True,
    )


def _goal(status=GoalVisibility.VISIBLE, same_target=True, bbox=(0.4, 0.4, 0.6, 0.6)) -> GoalObservation:
    return GoalObservation(status, bbox, None, "the red car", "visible", same_target,
                           "model_surface_points", surface_points=())


def _geometry(points, world_point=None, reliable=True) -> GoalGeometry:
    samples = tuple(
        SurfacePointEstimate((0.5, 0.5), True, "sampled", 5.0, point, 5.0)
        for point in points
    )
    return GoalGeometry(reliable, "sampled", 5.0, world_point or (points[0] if points else None),
                        surface_samples=samples, position_source="surface_point_depth")


def _state(timestamp_ns=10) -> TargetPositionState:
    return TargetPositionState("stage-1", (5.0, 0.0, 0.0), (5.0, 0.0, 0.0),
                               "capture-0", 1.0, timestamp_ns,
                               anchor_surface_points_world=((5.0, 0.0, 0.0),),
                               surface_points_world=((5.0, 0.0, 0.0),))


def test_accept_position_binds_first_observation_to_its_surfaces():
    observation = _observation()
    state, reason = accept_position(None, "stage-1", observation, _goal(),
                                    _geometry([(5.0, 0.0, 0.0), (6.0, 0.0, 0.0)]), 0.5)
    assert reason is None
    assert state.anchor_surface_points_world == ((5.0, 0.0, 0.0), (6.0, 0.0, 0.0))
    assert state.world_point == (5.0, 0.0, 0.0)
    assert state.capture_id == "capture-1"
    assert state.position_source == "surface_point_depth"


def test_accept_position_rejects_stale_or_unmatched_observations():
    state = _state()
    stale = _observation(timestamp_ns=10)
    kept, reason = accept_position(state, "stage-1", stale, _goal(),
                                   _geometry([(5.0, 0.0, 0.0)]), 0.5)
    assert kept is state
    assert reason == "stale_target_observation"

    fresh = _observation(timestamp_ns=20, captured_monotonic=2.0)
    kept, reason = accept_position(state, "stage-1", fresh, _goal(),
                                   _geometry([(20.0, 0.0, 0.0)]), 0.5)
    assert kept is state
    assert reason == "new_surface_not_added_to_fixed_target"


def test_accept_position_refines_only_matching_surfaces():
    state = _state()
    fresh = _observation(timestamp_ns=20, captured_monotonic=2.0)
    updated, reason = accept_position(state, "stage-1", fresh, _goal(),
                                      _geometry([(5.2, 0.0, 0.0)]), 0.5)
    assert reason is None
    assert updated.world_point == (5.2, 0.0, 0.0)
    assert updated.anchor_surface_points_world == ((5.0, 0.0, 0.0),)
    assert updated.capture_id == "capture-1"


def test_accept_position_rejects_a_different_stage():
    with pytest.raises(ValueError):
        accept_position(_state(), "stage-2", _observation(), _goal(),
                        _geometry([(5.0, 0.0, 0.0)]), 0.5)


def test_bound_target_geometry_uses_the_nearest_bound_surface():
    state = _state()
    geometry = bound_target_geometry(state, WorldPose(6.0, 0.0, 0.0, 0.0))
    assert geometry.reliable is True
    assert geometry.world_point == (5.0, 0.0, 0.0)
    assert geometry.distance_m == pytest.approx(1.0)


def test_make_reference_crops_numpy_rgb_from_the_goal_bbox():
    observation = _observation()
    reference = make_reference(observation, _goal(), _geometry([(5.0, 0.0, 0.0)]))
    assert isinstance(reference, GoalReference)
    expected = observation.rgb[2:3, 3:4]
    assert np.array_equal(reference.crop, expected)
    assert reference.crop.shape == (1, 1, 3)


class _Detector:
    def __init__(self, detection):
        self.detection = detection
        self.requests = []

    def detect(self, request, *, timeout_s):
        self.requests.append((request, timeout_s))
        return self.detection


def test_bbox_goal_observer_reports_visible_and_hidden_targets():
    bbox = NormalizedBoundingBox(0.4, 0.4, 0.6, 0.6)
    detector = _Detector(TargetDetection(found=True, bbox=bbox, surface_points=((0.5, 0.5),)))
    observer = BBoxGoalObserver(detector, timeout_s=7.0, clock=lambda: 0.0)
    goal = observer.observe(_observation(), "the red car")
    assert goal.status is GoalVisibility.VISIBLE
    assert goal.bbox == bbox.coordinates
    assert goal.surface_points == ((0.5, 0.5),)
    assert goal.association_source == "model_surface_points"
    assert detector.requests[0][1] == 7.0
    assert detector.requests[0][0].reference_rgb is None

    hidden = BBoxGoalObserver(_Detector(TargetDetection.missing()), clock=lambda: 0.0)
    result = hidden.observe(_observation(), "the red car", GoalReference("the red car", object(), (1, 0, 0), None))
    assert result.status is GoalVisibility.NOT_VISIBLE
    assert result.same_target is False


def test_bbox_goal_observer_passes_reference_crop_and_rejects_exhausted_budget():
    crop = np.zeros((2, 2, 3), dtype=np.uint8)
    detector = _Detector(TargetDetection(found=True, bbox=NormalizedBoundingBox(0.4, 0.4, 0.6, 0.6)))
    observer = BBoxGoalObserver(detector, clock=lambda: 0.0)
    observer.observe(_observation(), "the red car",
                     GoalReference("the red car", crop, (1.0, 0.0, 0.0), (0.4, 0.4, 0.6, 0.6)))
    assert detector.requests[0][0].reference_rgb is crop
    exhausted = BBoxGoalObserver(_Detector(TargetDetection.missing()), clock=lambda: 5.0)
    with pytest.raises(BudgetExceeded):
        exhausted.observe(_observation(), "the red car",
                          budget=ExecutionBudget(5.0))


def test_sample_surface_depth_reads_exactly_one_pixel():
    intrinsics = CameraIntrinsics(10, 10, 10.0, 10.0, 4.5, 4.5, 90.0)
    depth = np.full((10, 10), 12.0, dtype=np.float32)
    assert sample_surface_depth(depth, (5.0, 5.0), (0.0, 0.0, 1.0, 1.0),
                                intrinsics, intrinsics, 0.5).valid is True
    outside = sample_surface_depth(depth, (5.0, 5.0), (0.6, 0.6, 1.0, 1.0),
                                   intrinsics, intrinsics, 0.5)
    assert outside.valid is False
    too_near = sample_surface_depth(np.full((10, 10), 0.2, dtype=np.float32), (5.0, 5.0),
                                    (0.0, 0.0, 1.0, 1.0), intrinsics, intrinsics, 0.5)
    assert too_near.valid is False
    uncovered = sample_surface_depth(depth, (10.0, 5.0), (0.0, 0.0, 1.0, 1.0),
                                     intrinsics, intrinsics, 0.5)
    assert uncovered.valid is False


def test_depth_foreground_filter_disabled_and_no_separation():
    settings = DepthForegroundSettings(enabled=False)
    samples = (SurfacePointEstimate((0.5, 0.5), True, "sampled", 5.0, (5.0, 0.0, 0.0), 5.0),)
    kept, result = DepthForegroundFilter(settings).filter(
        _observation(), (0.0, 0.0, 1.0, 1.0), np.full((6, 8), 5.0), 0.5, samples, ((4, 3),))
    assert kept == samples
    assert result.applied is False and result.reason == "disabled"

    uniform = DepthForegroundFilter(DepthForegroundSettings(pixel_radius=1))
    depth = np.full((6, 8), 5.0)
    kept, result = uniform.filter(_observation(), (0.0, 0.0, 1.0, 1.0), depth, 0.5,
                                  samples, ((4, 3),))
    assert result.applied is False
    assert result.reason == "no_clear_local_depth_separation"


class _Session:
    def __init__(self):
        self.stage_id = "stage-1"
        self.target_state = None
        self.target_observation = None
        self.reference = None
        self.geometry = None
        self.search = _Search()
        self.trigger = _Trigger()
        self.initial = WorldPose(0.0, 0.0, 0.0, 0.0)
        self.budget = ExecutionBudget(100.0)

    def require_time(self, clock):
        return None


class _Search:
    def __init__(self):
        self.found_calls = 0

    def found(self):
        self.found_calls += 1


class _Trigger:
    def __init__(self):
        self.snapshots = []

    def update(self, snapshot):
        self.snapshots.append(snapshot)


class _Geometry:
    def __init__(self, geometry):
        self.geometry = geometry

    def estimate(self, observation, goal):
        return self.geometry


def _refresh(geometry):
    events = []
    refresh = VisualGoalRefresh(
        observer=None,
        geometry=_Geometry(geometry),
        limits=VisualGoalLimits(),
        clock=lambda: 0.0,
        event=lambda trace, name, **payload: events.append((name, payload)),
    )
    return refresh, events


def test_visual_goal_refresh_accepts_a_first_target_and_updates_session():
    geometry = _geometry([(5.0, 0.0, 0.0)])
    refresh, events = _refresh(geometry)
    session = _Session()
    accepted = refresh.accept(_observation(), _goal(), session, trace=None)
    assert accepted is True
    assert session.target_state.world_point == (5.0, 0.0, 0.0)
    assert session.geometry.world_point == (5.0, 0.0, 0.0)
    assert session.reference.crop is not None
    assert session.search.found_calls == 1
    assert events[0][0] == "target_updated"


def test_visual_goal_refresh_rejects_an_unmatched_second_surface():
    refresh, events = _refresh(_geometry([(20.0, 0.0, 0.0)]))
    session = _Session()
    session.target_state = _state()
    accepted = refresh.accept(_observation(timestamp_ns=20, captured_monotonic=2.0),
                              _goal(), session, trace=None)
    assert accepted is False
    assert session.target_state.world_point == (5.0, 0.0, 0.0)
    assert events[0][0] == "target_update_rejected"
    assert events[0][1]["reason"] == "new_surface_not_added_to_fixed_target"