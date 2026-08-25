"""Regression tests for high-altitude large-building ``above`` navigation."""

from __future__ import annotations

import time
from types import SimpleNamespace

import numpy as np
from PIL import Image

import agent.functions.fast_slow.runtime as runtime_module
from agent.functions.common.task_manager import TaskManager
from agent.functions.fast_slow.runtime import (
    _apply_above_altitude_path_guard,
    _apply_above_roof_acquisition_path_guard,
    _above_stage_state,
    _handle_above_completion_trigger,
    _queue_reaches_memory_arrival,
    _record_synchronized_roof_plane,
    _should_trigger_completion_vlm,
    _should_trigger_idle_memory_completion,
)
from agent.functions.memory import MissionMemory
from agent.functions.memory.mission_memory import stage_key
from agent.functions.memory.schemas import TargetInstanceBelief, TargetMemory


def _above_stage():
    return SimpleNamespace(
        index=0,
        instruction="Fly above the first building",
        mode="target",
        target="building",
        relation="above",
        ordinal=1,
        selection_rule="ordinal",
        completion_condition="above the building",
    )


def _locked_memory(stage, *, target_x: float = 30.0):
    memory = MissionMemory(
        config={
            "ENABLED": True,
            "ABOVE_ALTITUDE_GUARD_ENABLED": True,
            "ABOVE_PRE_ROOF_MAX_DESCENT_M": 1.0,
            "ABOVE_MAX_VERTICAL_LEG_M": 3.0,
            "ABOVE_MAX_CLEARANCE_M": 18.0,
            "ABOVE_MIN_CLEARANCE_M": 0.3,
            "ABOVE_MAX_ALTITUDE_M": 60.0,
            "ABOVE_REQUIRE_ROOF_GEOMETRY": True,
            "ABOVE_ROOF_MEMORY_MIN_OBSERVATIONS": 2,
            "ABOVE_ROOF_MEMORY_MIN_CONFIDENCE": 0.60,
            "ABOVE_ROOF_MAX_UNCERTAINTY_M": 2.0,
            "ABOVE_ROOF_MAX_AGE_S": 20.0,
            "MAX_COMPLETION_UNCERTAINTY_M": 5.0,
            "MEMORY_ONLY_MIN_CONFIDENCE": 0.88,
            "MEMORY_ONLY_MIN_OBSERVATIONS": 2,
            "METRIC_LOCK_MAX_DEPTH_M": 120.0,
            "LOCKED_LARGE_STRUCTURE_CONTINUITY_M": 8.0,
            "VIEW_RELATIVE_LOCKED_SURFACE_CONTINUITY_M": 8.0,
            "LOCKED_BEARING_MIN_TOLERANCE_DEG": 8.0,
            "LOCKED_BEARING_MAX_TOLERANCE_DEG": 32.0,
        },
        sim_config={"FRONT_FOV": 90.0, "DOWN_FOV": 90.0},
    )
    instance = TargetInstanceBelief(
        instance_id="building:1",
        encounter_order=1,
        target_world=[target_x, 0.0, -30.0],
        confidence=0.90,
        uncertainty_m=1.0,
        observation_count=1,
        footprint_radius_m=5.0,
        surface_points_world=[[target_x, 0.0, -50.0]],
        surface_bounds_world=[[target_x, -2.0, -55.0], [target_x, 2.0, -25.0]],
        surface_observation_count=2,
        geometry_kind="large_surface",
        is_large_structure=True,
    )
    target = TargetMemory(
        target_key="building",
        target_name="building",
        primary_instance_id=instance.instance_id,
    )
    target.instances[instance.instance_id] = instance
    memory.target_memories["building"] = target
    memory.stage_locks[stage_key(stage)] = instance.instance_id
    return memory, instance


def _runtime_objects(memory):
    return SimpleNamespace(mission_memory=memory, above_stage_states={})


def _install_roof(instance, *, roof_z: float = -20.0):
    instance.roof_points_world = [[27.0, -3.0, roof_z], [33.0, 3.0, roof_z]]
    instance.roof_bounds_world = [[27.0, -3.0, roof_z], [33.0, 3.0, roof_z]]
    instance.roof_z_median = roof_z
    instance.roof_confidence = 0.82
    instance.roof_uncertainty_m = 0.4
    instance.roof_observation_count = 2
    instance.last_roof_seen_s = time.perf_counter()


def _horizontal_plane_depth(clearance_m: float, size: int = 256):
    rows, cols = np.mgrid[0:size, 0:size]
    focal = size / 2.0
    u = (cols + 0.5 - size * 0.5) / focal
    v = (rows + 0.5 - size * 0.5) / focal
    return float(clearance_m) * np.sqrt(1.0 + u * u + v * v)


def test_unknown_roof_clamps_descent_but_preserves_climb():
    stage = _above_stage()
    memory, _instance = _locked_memory(stage)
    objects = _runtime_objects(memory)
    _above_stage_state(objects, stage, [0.0, 0.0, -50.0])

    guarded, reason = _apply_above_altitude_path_guard(
        objects,
        stage,
        [[4.0, 0.0, -2.0], [8.0, 0.0, 14.0]],
        selection_pos=[0.0, 0.0, -50.0],
    )

    assert guarded[0][2] == -2.0
    assert guarded[1][2] == 1.0
    assert "roof_unknown_hold_entry_altitude" in reason


def test_trusted_roof_allows_only_segmented_descent_then_holds_clearance():
    stage = _above_stage()
    memory, instance = _locked_memory(stage)
    _install_roof(instance, roof_z=-20.0)
    objects = _runtime_objects(memory)

    high_path, high_reason = _apply_above_altitude_path_guard(
        objects,
        stage,
        [[5.0, 0.0, 12.0]],
        selection_pos=[30.0, 0.0, -50.0],
    )
    safe_path, safe_reason = _apply_above_altitude_path_guard(
        objects,
        stage,
        [[5.0, 0.0, 4.0]],
        selection_pos=[30.0, 0.0, -35.0],
    )

    assert high_path[0][2] == 3.0
    assert "segmented_descent" in high_reason
    assert safe_path[0][2] == 0.0
    assert "hold_clearance" in safe_reason


def test_large_building_facade_point_cannot_complete_above_but_roof_can():
    stage = _above_stage()
    memory, instance = _locked_memory(stage)

    facade_only = memory.evaluate_completion(stage=stage, current_world=[30.0, 0.0, -50.0])
    _install_roof(instance, roof_z=-20.0)
    roof_complete = memory.evaluate_completion(stage=stage, current_world=[30.0, 0.0, -50.0])

    assert not facade_only.done
    assert facade_only.status == "HOLD_CONFIRM"
    assert "no verified down-view roof geometry" in facade_only.reason
    assert roof_complete.done
    assert roof_complete.details["geometry"] == "roof"
    assert roof_complete.target_world == [30.0, 0.0, -20.0]


def test_facade_point_cannot_trigger_runtime_watchdog_or_idle_completion():
    stage = _above_stage()
    memory, _instance = _locked_memory(stage)
    controller = SimpleNamespace(
        planning=False,
        has_plan_job=False,
        queue=SimpleNamespace(world_waypoints=[]),
    )
    objects = SimpleNamespace(
        mission_memory=memory,
        above_stage_states={},
        distance_estimator=SimpleNamespace(use_for_completion=False),
        controller=controller,
    )
    estimate = SimpleNamespace(
        source="mission_memory",
        distance_m=1.0,
        current_world=[29.0, 0.0, -50.0],
        trigger_radius_m=None,
    )

    assert not _should_trigger_completion_vlm(objects, stage, estimate, 4.0)
    assert not _should_trigger_idle_memory_completion(objects, stage, estimate, 4.0)

    above_state = _above_stage_state(objects, stage, estimate.current_world)
    above_state.require_roof_before_next_completion = True
    assert not _should_trigger_completion_vlm(objects, stage, estimate, 4.0)


def test_facade_arrival_does_not_suppress_planning_before_roof_is_found():
    stage = _above_stage()
    memory, instance = _locked_memory(stage)
    controller = SimpleNamespace(queue=SimpleNamespace(world_waypoints=[[30.0, 0.0, -50.0]]))
    objects = SimpleNamespace(
        mission_memory=memory,
        above_stage_states={},
        controller=controller,
    )

    assert not _queue_reaches_memory_arrival(objects, stage, 4.0)

    _install_roof(instance, roof_z=-20.0)
    assert _queue_reaches_memory_arrival(objects, stage, 4.0)


def test_failed_single_roof_confirmation_resumes_motion_until_roof_is_trusted():
    stage = _above_stage()
    memory, instance = _locked_memory(stage)
    instance.roof_points_world = [[28.0, -2.0, -20.0], [32.0, 2.0, -20.0]]
    instance.roof_bounds_world = [[28.0, -2.0, -20.0], [32.0, 2.0, -20.0]]
    instance.roof_z_median = -20.0
    instance.roof_confidence = 0.52
    instance.roof_uncertainty_m = 0.8
    instance.roof_observation_count = 1
    instance.last_roof_seen_s = time.perf_counter()
    controller = SimpleNamespace(queue=SimpleNamespace(world_waypoints=[[30.0, 0.0, -50.0]]))
    objects = SimpleNamespace(
        mission_memory=memory,
        above_stage_states={},
        controller=controller,
        distance_estimator=SimpleNamespace(use_for_completion=False),
    )
    above_state = _above_stage_state(objects, stage, [30.0, 0.0, -50.0])
    above_state.require_roof_before_next_completion = True
    estimate = SimpleNamespace(**memory.estimate_distance(stage, [30.0, 0.0, -50.0]))

    guarded, reason = _apply_above_roof_acquisition_path_guard(
        objects,
        stage,
        [[1.0, 0.0, 0.0]],
        selection_pos=[30.0, 0.0, -50.0],
        selection_yaw=0.0,
        planning_wall_s=5.2,
    )

    assert not _should_trigger_completion_vlm(objects, stage, estimate, 4.0)
    assert not _queue_reaches_memory_arrival(objects, stage, 4.0)
    assert "roof_acquire_continuous_horizon" in reason
    assert guarded[-1][0] > 1.0


def test_short_above_plan_is_extended_to_cover_planner_latency():
    stage = _above_stage()
    memory, _instance = _locked_memory(stage, target_x=30.0)
    memory.config.update({
        "ABOVE_MIN_CONTINUOUS_HORIZON_M": 12.0,
        "ABOVE_CONTINUOUS_FALLBACK_PLANNING_S": 5.5,
        "ABOVE_CONTINUOUS_HORIZON_MARGIN_S": 1.5,
        "PATH_MAX_GUIDED_LEG_M": 18.0,
    })
    objects = _runtime_objects(memory)

    guarded, reason = _apply_above_roof_acquisition_path_guard(
        objects,
        stage,
        [[2.0, 0.0, 0.0], [4.6, 0.0, 0.0]],
        selection_pos=[0.0, 0.0, -50.0],
        selection_yaw=0.0,
        planning_wall_s=5.2,
    )

    assert "approach_continuous_horizon" in reason
    assert guarded[-1][0] >= 12.0
    assert guarded[-1][0] <= 18.0
    assert all(point[2] == 0.0 for point in guarded)


def test_roof_acquisition_crosses_facade_and_keeps_original_probe_direction():
    stage = _above_stage()
    memory, _instance = _locked_memory(stage, target_x=30.0)
    memory.config.update({
        "ABOVE_MIN_CONTINUOUS_HORIZON_M": 12.0,
        "ABOVE_ROOF_PROBE_BEYOND_ANCHOR_M": 8.0,
        "ABOVE_ROOF_PROBE_MAX_TRAVEL_M": 24.0,
        "PATH_MAX_GUIDED_LEG_M": 18.0,
    })
    objects = _runtime_objects(memory)

    first, first_reason = _apply_above_roof_acquisition_path_guard(
        objects,
        stage,
        [[2.0, 0.0, 0.0]],
        selection_pos=[25.0, 0.0, -50.0],
        selection_yaw=0.0,
        planning_wall_s=5.2,
    )
    second, second_reason = _apply_above_roof_acquisition_path_guard(
        objects,
        stage,
        [[1.0, 0.0, 0.0]],
        selection_pos=[31.0, 0.0, -50.0],
        selection_yaw=0.0,
        planning_wall_s=5.2,
    )

    assert "roof_acquire_continuous_horizon" in first_reason
    assert first[-1][0] > 5.0
    assert "roof_acquire_continuous_horizon" in second_reason
    assert second[-1][0] > 0.0


def test_trusted_roof_reenables_normal_completion_trigger():
    stage = _above_stage()
    memory, instance = _locked_memory(stage)
    _install_roof(instance, roof_z=-20.0)
    objects = SimpleNamespace(
        mission_memory=memory,
        above_stage_states={},
        distance_estimator=SimpleNamespace(use_for_completion=False),
    )
    estimate = SimpleNamespace(**memory.estimate_distance(stage, [30.0, 0.0, -50.0]))

    assert _should_trigger_completion_vlm(objects, stage, estimate, 4.0)


def test_roof_plane_rejection_reason_is_visible_without_debug_logs(capsys):
    stage = _above_stage()
    memory, _instance = _locked_memory(stage)
    memory.config.update({
        "ABOVE_ROOF_DEPTH_STRIDE": 8,
        "ABOVE_ROOF_MIN_VALID_RATIO": 0.35,
        "ABOVE_ROOF_MIN_COVERAGE_RATIO": 0.20,
        "ABOVE_ROOF_MIN_CENTER_SUPPORT_RATIO": 0.65,
    })
    objects = _runtime_objects(memory)

    _record_synchronized_roof_plane(
        objects,
        stage,
        np.full((256, 256), np.nan, dtype=float),
        [30.0, 0.0, -50.0],
        0.0,
        source="above_completion",
    )
    invalid_output = capsys.readouterr().out
    _record_synchronized_roof_plane(
        objects,
        stage,
        _horizontal_plane_depth(80.0),
        [30.0, 0.0, -50.0],
        0.0,
        source="planning_snapshot",
    )
    rejected_output = capsys.readouterr().out

    assert "reason=insufficient_valid_down_depth" in invalid_output
    assert "center=" in invalid_output
    assert "reason=reject_roof_plane_below_anchor" in rejected_output


def test_locked_identity_rejects_far_same_class_metric_and_wrong_rgb_bearing():
    stage = _above_stage()
    memory, _instance = _locked_memory(stage)
    image = Image.new("RGB", (640, 480), (100, 100, 100))
    far_metric = SimpleNamespace(
        visible=True,
        bbox=[300, 200, 340, 260],
        score=0.9,
        label="building",
        camera="front",
        depth_median=90.0,
        depth_valid_ratio=0.9,
        depth_mad_m=0.2,
    )
    wrong_bearing = SimpleNamespace(
        visible=True,
        bbox=[550, 190, 620, 270],
        score=0.9,
        label="building",
        camera="front",
        depth_median=None,
    )

    metric_gate = memory.evaluate_locked_detection_identity(
        stage,
        far_metric,
        image,
        observer_world=[0.0, 0.0, -50.0],
        observer_yaw_deg=0.0,
        view="front",
    )
    bearing_gate = memory.evaluate_locked_detection_identity(
        stage,
        wrong_bearing,
        image,
        observer_world=[0.0, 0.0, -50.0],
        observer_yaw_deg=0.0,
        view="front",
    )

    assert not metric_gate["accepted"]
    assert metric_gate["reason"] == "metric_locked_geometry_mismatch"
    assert not bearing_gate["accepted"]
    assert bearing_gate["reason"] == "rgb_locked_bearing_mismatch"


def test_above_completion_uses_synchronized_full_frame_roof_without_bbox(monkeypatch):
    stage = _above_stage()
    memory, instance = _locked_memory(stage)
    memory.config.update({
        "ABOVE_ROOF_DEPTH_STRIDE": 8,
        "ABOVE_ROOF_MIN_VALID_RATIO": 0.35,
        "ABOVE_ROOF_MIN_COVERAGE_RATIO": 0.20,
        "ABOVE_ROOF_MIN_CENTER_SUPPORT_RATIO": 0.65,
        "ABOVE_ROOF_MAX_Z_MAD_M": 0.45,
        "ABOVE_ROOF_MAX_NORMAL_ERROR_DEG": 15.0,
        "ABOVE_ROOF_MIN_CONFIDENCE": 0.45,
        "ABOVE_ROOF_ASSOCIATION_MARGIN_M": 6.0,
        "ABOVE_ROOF_Z_CONTINUITY_M": 2.5,
    })
    # One earlier bbox-free planning snapshot; the synchronized completion
    # capture below supplies the second consistent plane.
    instance.roof_points_world = [[28.0, -2.0, -30.0], [32.0, 2.0, -30.0]]
    instance.roof_bounds_world = [[28.0, -2.0, -30.0], [32.0, 2.0, -30.0]]
    instance.roof_z_median = -30.0
    instance.roof_confidence = 0.52
    instance.roof_uncertainty_m = 0.8
    instance.roof_observation_count = 1
    instance.last_roof_seen_s = time.perf_counter()

    front = Image.new("RGB", (640, 480), (100, 100, 100))
    down = Image.new("RGB", (640, 480), (80, 80, 80))
    depth = _horizontal_plane_depth(20.0)
    monkeypatch.setattr(
        runtime_module.web_helpers,
        "capture_profile_isolated_with_pose",
        lambda _client, _profile: (
            front,
            down,
            None,
            depth,
            {"total_s": 0.01},
            [30.0, 0.0, -50.0],
            0.0,
        ),
    )
    invisible_front = SimpleNamespace(visible=False, camera="front", score=0.0)
    invisible_down = SimpleNamespace(visible=False, camera="down", score=0.0)
    monkeypatch.setattr(
        runtime_module,
        "_detect_dual_view",
        lambda *_args, **_kwargs: (
            None,
            invisible_front,
            invisible_down,
            0.0,
            [],
            [],
        ),
    )

    manager = TaskManager()
    manager.start_with_stages("above building", [stage])
    objects = SimpleNamespace(
        mission_memory=memory,
        above_stage_states={},
        completion_checker=SimpleNamespace(depth_profile="front_down_both_depth"),
        distance_estimator=SimpleNamespace(enabled=False, clear=lambda: None),
        navigation_metrics=SimpleNamespace(
            update_target=lambda *_args, **_kwargs: None,
            record_distance=lambda *_args, **_kwargs: None,
            task_completed=False,
        ),
        controller=SimpleNamespace(clear=lambda: None),
        completion_pipeline=None,
        task_manager=manager,
        completion_attempts={},
        completion_retry_after={},
    )
    state_values = {}
    state = SimpleNamespace(update=lambda **values: state_values.update(values))
    path_stream = SimpleNamespace(stop=lambda: None)

    task_done = _handle_above_completion_trigger(
        objects,
        client=SimpleNamespace(),
        path_stream=path_stream,
        state=state,
        stage=stage,
        task_text="above building",
        trigger_radius_m=4.0,
    )

    assert task_done is True
    assert instance.roof_observation_count == 2
    assert memory.last_completion_decision is not None
    assert memory.last_completion_decision.done
    assert memory.last_completion_decision.details["geometry"] == "roof"
    assert state_values["task_done"] is True
