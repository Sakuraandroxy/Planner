from dataclasses import replace

import pytest

from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.capabilities.targeting.surface_selection import SurfaceSelector
from planner.capabilities.completion.visual_goal import VisualGoalCompletion
from planner.domain.mission import MissionStage, TaskKind
from planner.domain.motion import MotionLimits
from planner.domain.visual_goal import GoalVisibility, VisualGoalStatus, VisualGoalNavigationParameters
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.errors import ObservationError
from planner.domain.pose import RelativePoseDelta
from planner.domain.target_surface import SurfaceCandidate
from planner.adapters.diagnostics.visual_goal_trace import VisualGoalTrace
from planner.services.execution import ExecutionService
from planner.services.motion import SynchronizedMotionPlanner
from planner.services.trajectory_safety import TrajectorySafetyChecker
from planner.services.trajectory_validation import TrajectoryValidator
from planner.workflows.visual_goal_navigation import VisualGoalNavigationWorkflow
from tests.fakes.components import FakeVehicle
from tests.fakes.visual_goal import Clock, WallSource, StaticObserver, ForwardPlanner, VISIBLE, INVISIBLE, surface_observation


def setup(target_x=6, goal=VISIBLE, limits=None):
    vehicle = FakeVehicle()
    clock = Clock()
    limits = limits or VisualGoalLimits()
    source = WallSource(vehicle, clock, target_x)
    observer, planner = StaticObserver(goal), ForwardPlanner()
    geometry, motion_limits = DepthGeometry(0.5, 200, True), MotionLimits()
    workflow = VisualGoalNavigationWorkflow(source, observer, planner, TrajectoryValidator(5, 30, 180),
        SynchronizedMotionPlanner(2, motion_limits), ExecutionService(vehicle, clock), geometry,
        VisualGoalCompletion(limits), TrajectorySafetyChecker(geometry, limits, motion_limits, True), limits,
        SurfaceSelector(limits),
        clock=clock, sleep=clock.sleep)
    stage = MissionStage("goal", TaskKind.VISUAL_GOAL_NAVIGATION,
                         VisualGoalNavigationParameters("approach red house", "red house"))
    return workflow, stage, source, observer, planner, vehicle, clock


def test_initial_arrival_requires_two_fresh_observations_without_motion():
    workflow, stage, source, observer, planner, vehicle, clock = setup(target_x=4)
    result = workflow.run(stage)
    assert result.success, result.error
    assert result.value.status is VisualGoalStatus.REACHED
    assert source.captures == 2 and not planner.calls and not vehicle.segments
    assert result.value.confirmation_count == 2 and result.value.stop.confirmed
    assert clock() >= 100.5  # Static images have identical contents and remain valid.


def test_approach_executes_only_safe_prefix_then_confirms_arrival():
    workflow, stage, source, observer, planner, vehicle, clock = setup()
    result = workflow.run(stage)
    assert result.success, result.error
    assert len(planner.calls) == 1 and len(vehicle.segments) == 1
    assert planner.calls[0][2]["round_index"] == 1
    assert vehicle.pose.x == 2
    assert result.value.final_distance_m == pytest.approx(4, abs=0.01)
    assert observer.calls[1][2] is not None


def test_replayed_timestamp_never_confirms_success():
    workflow, stage, source, observer, planner, vehicle, clock = setup(target_x=4)
    source.replay = True
    result = workflow.run(stage)
    assert not result.success and result.value.status is VisualGoalStatus.PERCEPTION_UNRELIABLE
    assert not vehicle.segments


def test_unknown_target_finite_search_without_translation():
    limits = replace(VisualGoalLimits(), search_clearance_confirmed=True)
    workflow, stage, source, observer, planner, vehicle, clock = setup(goal=INVISIBLE, target_x=30, limits=limits)
    result = workflow.run(stage)
    assert not result.success and result.value.status is VisualGoalStatus.TARGET_NOT_FOUND
    assert len(vehicle.segments) == 9
    assert all(s.start.x == s.end.x and s.start.y == s.end.y for s in vehicle.segments)
    assert not planner.calls


def test_unconfirmed_rotation_clearance_blocks_search():
    workflow, stage, source, observer, planner, vehicle, clock = setup(goal=INVISIBLE)
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.BLOCKED and not vehicle.segments


def test_ambiguous_and_identity_change_do_not_switch_target():
    workflow, stage, source, observer, planner, vehicle, clock = setup(
        goal=replace(VISIBLE, status=GoalVisibility.AMBIGUOUS, bbox=None, surface_point=None))
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.TARGET_AMBIGUOUS and not vehicle.segments
    observer.goal = replace(VISIBLE, same_target=False)
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.PERCEPTION_UNRELIABLE


def test_missing_depth_and_capture_failure_never_move():
    workflow, stage, source, observer, planner, vehicle, clock = setup()
    capture = source.capture
    source.capture = lambda **kwargs: replace(capture(**kwargs), depth_kind="unknown")
    assert workflow.run(stage).value.status is VisualGoalStatus.BLOCKED
    def fail(**kwargs):
        raise ObservationError("camera RPC failed")
    source.capture = fail
    assert workflow.run(stage).value.status is VisualGoalStatus.OBSERVATION_ERROR
    assert not vehicle.segments


def test_new_stage_has_no_previous_target_reference():
    workflow, stage, source, observer, planner, vehicle, clock = setup(target_x=4)
    assert workflow.run(stage).success
    observer.calls.clear()
    assert workflow.run(replace(stage, stage_id="goal2")).success
    assert observer.calls[0][2] is None


def test_failure_to_stop_preserves_model_failure():
    workflow, stage, source, observer, planner, vehicle, clock = setup()
    def fail(*args, **kwargs):
        raise RuntimeError("model unavailable")
    observer.observe = fail
    vehicle.hover = lambda: (_ for _ in ()).throw(RuntimeError("hover unavailable"))
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.MODEL_ERROR
    assert "model unavailable" in result.error and not result.value.stop.confirmed
    assert result.value.stop.errors


def test_slow_goal_reply_expires_task_without_motion():
    workflow, stage, source, observer, planner, vehicle, clock = setup(
        limits=replace(VisualGoalLimits(), maximum_task_duration_s=1))
    def slow(*args, **kwargs):
        clock.sleep(2)
        return VISIBLE
    observer.observe = slow
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.TIMEOUT and not vehicle.segments


def test_drift_during_planning_discards_trajectory_and_refresh_is_finite():
    workflow, stage, source, observer, planner, vehicle, clock = setup(target_x=20)
    plan = planner.plan
    def drifting(*args, **kwargs):
        relative = plan(*args, **kwargs)
        vehicle.pose = replace(vehicle.pose, y=vehicle.pose.y+0.5)
        return relative
    planner.plan = drifting
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.PERCEPTION_UNRELIABLE
    assert not vehicle.segments
    assert len(planner.calls) == 2


def test_same_surface_coherent_motion_rejects_static_target_premise():
    workflow, stage, source, observer, planner, vehicle, clock = setup()
    capture = source.capture
    def moved(**kwargs):
        if source.captures > 0:
            source.target_x = 8+source.captures
        return capture(**kwargs)
    source.capture = moved
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.TARGET_MOVED
    assert not result.success and len(vehicle.segments) == 1


def test_first_segment_too_long_only_replans_once_and_never_clips():
    workflow, stage, source, observer, planner, vehicle, clock = setup(target_x=20)
    planner.points = (RelativePoseDelta(10, 0, 0, 0),)
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.BLOCKED
    assert len(planner.calls) == 2 and not vehicle.segments


def test_zero_motion_repeatedly_requested_is_stuck_not_success():
    workflow, stage, source, observer, planner, vehicle, clock = setup()
    planner.points = (RelativePoseDelta(0, 0, 0, 0),)
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.STUCK and not result.success


def test_diagnostics_save_evidence_without_api_payloads(tmp_path):
    import json
    workflow, stage, source, observer, planner, vehicle, clock = setup()
    workflow.trace_factory = lambda: VisualGoalTrace(tmp_path)
    result = workflow.run(stage)
    assert result.success, result.error
    from pathlib import Path
    output = Path(result.value.run_directory)
    assert list(output.glob("*_target.png")) and list(output.glob("*_depth_meters.npy"))
    records = [json.loads(line) for line in (output/"events.jsonl").read_text(encoding="utf-8").splitlines()]
    assert records[-1]["result"]["status"] == "reached"
    assert "base64" not in (output/"events.jsonl").read_text(encoding="utf-8")


def test_multi_surface_rollout_keeps_region_despite_id_and_order_changes(tmp_path):
    workflow, stage, source, observer, planner, vehicle, clock = setup()
    workflow.trace_factory = lambda: VisualGoalTrace(tmp_path)
    def observe(observation, description, reference=None, **kwargs):
        observer.calls.append((observation, description, reference, kwargs))
        count = len(observer.calls)
        candidates = [SurfaceCandidate(f"left-{count}", (.4, .5), "left facade"),
                      SurfaceCandidate(f"middle-{count}", (.5, .5), "central facade", reference is not None),
                      SurfaceCandidate(f"right-{count}", (.6, .5), "right facade")]
        return surface_observation(candidates=candidates if count % 2 else reversed(candidates))
    observer.observe = observe
    result = workflow.run(stage)
    assert result.success, result.error
    assert len(vehicle.segments) == 1 and result.value.confirmation_count == 2
    assert len(observer.calls) == 3 and observer.calls[-1][2] is observer.calls[1][2]
    from pathlib import Path
    import json
    path = Path(result.value.run_directory)
    assert len(list(path.glob("*_surface_selection.png"))) == 3
    records = [json.loads(line) for line in (path/"events.jsonl").read_text(encoding="utf-8").splitlines()]
    selections = [record["selection"] for record in records if record["event"] == "surface_selection"]
    assert all(len(item["estimates"]) == 3 for item in selections)
    anchors = [item["geometry"]["world_point"] for item in selections]
    assert anchors[1] == anchors[2] == anchors[0]


def test_all_candidate_depth_invalid_stops_without_motion():
    workflow, stage, source, observer, planner, vehicle, clock = setup()
    capture = source.capture
    def bad(**kwargs):
        observation = capture(**kwargs)
        observation.depth_meters[:] = float("nan")
        return observation
    source.capture = bad
    observer.goal = surface_observation(candidates=[SurfaceCandidate("a", (.4, .5), "left"),
                                                   SurfaceCandidate("b", (.6, .5), "right")])
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.PERCEPTION_UNRELIABLE
    assert len(observer.calls) == workflow.limits.maximum_unreliable_observations
    assert not vehicle.segments and not planner.calls


def test_visible_object_without_selected_region_does_not_switch_or_complete():
    workflow, stage, source, observer, planner, vehicle, clock = setup()
    def observe(observation, description, reference=None, **kwargs):
        observer.calls.append((observation, description, reference, kwargs))
        point = (.5, .5) if reference is None else (.7, .5)
        # Same object, but another visible facade is NOT the selected region.
        return surface_observation(candidates=[SurfaceCandidate("a", point, "another visible region", False)])
    observer.observe = observe
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.PERCEPTION_UNRELIABLE
    assert result.value.confirmation_count == 0 and len(vehicle.segments) == 1


def test_multi_candidates_with_one_bad_window_can_still_reach():
    workflow, stage, source, observer, planner, vehicle, clock = setup()
    capture = source.capture
    def partly_bad(**kwargs):
        observation = capture(**kwargs)
        observation.depth_meters[62:67, 49:54] = float("nan")
        return observation
    source.capture = partly_bad
    def observe(observation, description, reference=None, **kwargs):
        return surface_observation(candidates=[SurfaceCandidate("bad", (.4, .5), "bad depth"),
            SurfaceCandidate("center", (.5, .5), "central wall", reference is not None)])
    observer.observe = observe
    result = workflow.run(stage)
    assert result.success, result.error
    assert len(vehicle.segments) == 1


def test_other_near_surface_cancels_an_existing_arrival_confirmation():
    workflow, stage, source, observer, planner, vehicle, clock = setup(target_x=4)
    capture = source.capture
    def near_wing(**kwargs):
        obs = capture(**kwargs)
        if source.captures > 1:
            obs.depth_meters[62:67, 49:54] = 2.8
        return obs
    source.capture = near_wing
    def observe(observation, description, reference=None, **kwargs):
        candidates = [SurfaceCandidate("anchor", (.5, .5), "central wall", reference is not None)]
        if reference is not None:
            candidates.append(SurfaceCandidate("wing", (.4, .5), "closer wing"))
        return surface_observation(candidates=candidates)
    observer.observe = observe
    result = workflow.run(stage)
    assert result.value.status is VisualGoalStatus.SAFETY_STOP and not result.success
    assert result.value.confirmation_count == 0
    assert not vehicle.segments and not planner.calls
