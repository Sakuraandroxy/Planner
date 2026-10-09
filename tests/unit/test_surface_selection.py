from dataclasses import replace

import numpy as np
import pytest

from planner.capabilities.targeting.association import make_reference, coherent_surface_motion
from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.capabilities.targeting.surface_selection import SurfaceSelector
from planner.capabilities.completion.visual_goal import VisualGoalCompletion
from planner.domain.pose import WorldPose
from planner.domain.target_surface import SurfaceCandidate, SurfaceEstimate
from planner.domain.visual_goal import GoalGeometry, VisualGoalNavigationParameters
from planner.domain.visual_goal_limits import VisualGoalLimits
from tests.fakes.visual_goal import VISIBLE, surface_observation, wall_observation


def evaluate(obs, candidates, *, reference=None, estimates=None, limits=None):
    limits = limits or VisualGoalLimits()
    raw = surface_observation(candidates=candidates)
    estimates = estimates or DepthGeometry(0.5, 200, True).estimate_candidates(obs, raw, limits)
    return SurfaceSelector(limits).select(obs, raw, estimates, reference,
                                         WorldPose(0, 0, -4, 0), 4)


def point(name, u=0.5, v=0.5, match=False):
    return SurfaceCandidate(name, (u, v), "solid facade", match)


def test_bad_depth_candidate_does_not_hide_a_good_candidate():
    obs = wall_observation(6)
    obs.depth_meters[62:67, 49:54] = np.nan  # point u=.4
    selected = evaluate(obs, [point("bad", .4), point("good", .6)])
    assert selected.selected.candidate.candidate_id == "good"
    bad = next(item for item in selected.estimates if item.candidate.candidate_id == "bad")
    assert not bad.geometry.reliable and not bad.eligible


def test_all_invalid_depth_has_no_selection():
    obs = wall_observation()
    obs.depth_meters[:] = np.nan
    selected = evaluate(obs, [point("a", .4), point("b", .6)])
    assert selected.selected is None and not selected.geometry.reliable


def test_nearest_is_not_selected_over_better_depth_evidence():
    obs = wall_observation()
    candidates = [point("nearest", .4), point("reliable", .6)]
    estimates = tuple(SurfaceEstimate(p, GoalGeometry(True, "test", distance, (distance, 0, -4), ratio, mad), True, "test")
                      for p, distance, ratio, mad in [(candidates[0], 3.5, .71, .4),
                                                    (candidates[1], 6, 1, .01)])
    selected = evaluate(obs, candidates, estimates=estimates)
    assert selected.selected.candidate.candidate_id == "reliable"
    assert selected.geometry.distance_m == 6  # No averaging of surfaces.


def test_roof_candidate_is_not_a_level_height_approach_region():
    obs = wall_observation()
    roof = point("roof")
    evidence = SurfaceEstimate(roof, GoalGeometry(True, "test", 10, (6, 0, -12), 1, 0), True, "test")
    selected = evaluate(obs, [roof], estimates=(evidence,))
    assert selected.selected is None
    assert "height" in selected.estimates[0].reason
    assert selected.reliable_world_points == ((6, 0, -12),)  # Still retained as evidence.


def test_initial_order_and_near_duplicate_deduplication_are_deterministic():
    candidates = [point("a", .5), point("b", .505), point("c", .65)]
    forward = evaluate(wall_observation(), candidates)
    reverse = evaluate(wall_observation(), list(reversed(candidates)))
    assert forward.selected.candidate == reverse.selected.candidate
    assert [item.candidate for item in forward.estimates if item.eligible] == [
        item.candidate for item in reverse.estimates if item.eligible]
    assert sum("duplicates" in item.reason for item in forward.estimates) == 1


def test_frame_ids_and_order_do_not_override_selected_region():
    obs = wall_observation(6)
    first = evaluate(obs, [point("first")])
    reference = make_reference(obs, first.goal, first.geometry, first.selected.candidate.description)
    candidates = [point("first", .65), point("different-id", .5, match=True)]
    for order in (candidates, list(reversed(candidates))):
        selected = evaluate(obs, order, reference=reference)
        assert selected.selected.candidate.candidate_id == "different-id"
        assert selected.geometry.world_point == reference.world_point


def test_unmatched_facade_never_switches_or_claims_target_motion():
    obs = wall_observation(6)
    first = evaluate(obs, [point("first")])
    reference = make_reference(obs, first.goal, first.geometry)
    selected = evaluate(obs, [point("same-id", .7)], reference=reference)
    assert selected.selected is None
    raw = surface_observation(candidates=[point("same-id", .7)])
    assert SurfaceSelector.motion_evidence(raw, selected) is None
    assert not coherent_surface_motion(reference, VISIBLE,
        GoalGeometry(True, "new facade", 10, (10, 0, -4)), (9, 0, -4), 1.5, .3)


def test_small_resampling_change_cannot_fake_distance_progress():
    obs = wall_observation(6)
    first = evaluate(obs, [point("a")])
    reference = make_reference(obs, first.goal, first.geometry)
    new = evaluate(obs, [point("b", .55, match=True)], reference=reference)
    assert new.selected.geometry.distance_m > first.geometry.distance_m
    assert new.geometry.distance_m == first.geometry.distance_m
    assert new.geometry.world_point == reference.world_point


def test_anchor_arrival_requires_fresh_depth_agreement():
    obs = wall_observation(4)
    first = evaluate(obs, [point("a")])
    reference = make_reference(obs, first.goal, first.geometry)
    current = wall_observation(5)
    selected = evaluate(current, [point("b", match=True)], reference=reference)
    assert selected.geometry.distance_m == pytest.approx(4, abs=.01)
    check = VisualGoalCompletion(VisualGoalLimits())
    params = VisualGoalNavigationParameters("approach", "house")
    assert not check.candidate(params, current, selected.goal, selected.geometry, obs.vehicle_pose,
                               True, surface_selection=selected)


def test_unselected_reliable_near_surface_prevents_completion():
    obs = wall_observation(4)
    first = evaluate(obs, [point("anchor")])
    near = SurfaceEstimate(point("near", .6), GoalGeometry(True, "near wing", 2.8, (2.8, 0, -4), 1, 0),
                           False, "unselected")
    selected = replace(first, estimates=first.estimates+(near,))
    check = VisualGoalCompletion(VisualGoalLimits())
    params = VisualGoalNavigationParameters("approach", "house")
    assert not check.surface_clearance(params, selected)
    assert not check.candidate(params, obs, selected.goal, selected.geometry, obs.vehicle_pose, True,
                               surface_selection=selected)


@pytest.mark.parametrize("depth_size", [(64, 64), (256, 144)])
def test_multi_candidate_depth_uses_calibrated_resolution_and_yaw(depth_size):
    obs = wall_observation(6, WorldPose(2, 3, -4, 90), size=(192, 108), depth_size=depth_size)
    raw = surface_observation(candidates=[point("left", .4), point("right", .6)])
    estimates = DepthGeometry(.5, 200, True).estimate_candidates(obs, raw, VisualGoalLimits())
    for item, sign in zip(estimates, (1, -1)):
        assert item.geometry.reliable
        assert item.geometry.world_point == pytest.approx((2+sign*1.2, 9, -4), abs=.05)


@pytest.mark.parametrize("value", [0, -1, True, 2.5, float("inf")])
def test_candidate_limit_validation(value):
    with pytest.raises(ValueError):
        VisualGoalLimits(maximum_surface_candidates=value)
