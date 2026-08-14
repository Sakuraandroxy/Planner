"""Regression tests for stage failure and locked-facade path safety."""

from __future__ import annotations

from types import SimpleNamespace

import agent.functions.fast_slow.runtime as runtime_module
from agent.functions.common.task_manager import TaskManager
from agent.functions.fast_slow.runtime import (
    _CompletionRadiusWatchdog,
    _active_queue_overshoots_locked_surface,
    _continuous_path_velocity,
    _fail_current_stage_after_relocalization,
    _handle_distance_completion_trigger,
    _limit_cumulative_path_length,
    _record_locked_target_collision,
    _truncate_queue_at_completion_radius,
)
from agent.functions.memory import MissionMemory
from agent.functions.memory.mission_memory import stage_key
from agent.functions.memory.schemas import TargetInstanceBelief, TargetMemory
from agent.functions.task_parser.base import TaskStage


def _building_stage() -> TaskStage:
    return TaskStage(
        index=0,
        instruction="Fly to the first building in the right front",
        mode="target",
        target="building",
        relation="near",
        ordinal=1,
        view_relative=True,
    )


def _locked_building_memory(stage: TaskStage, surface_x: float = 10.0) -> MissionMemory:
    memory = MissionMemory(
        config={
            "ENABLED": True,
            "INSTANCE_LOCK_ENABLED": True,
            "NEAR_APPROACH_RADIUS_M": 4.5,
            "LARGE_STRUCTURE_COLLISION_CONTACT_MAX_DISTANCE_M": 2.5,
            "LARGE_STRUCTURE_COLLISION_CONTACT_MAX_LATERAL_M": 4.0,
            "LARGE_STRUCTURE_COLLISION_CONTACT_FORWARD_M": 0.8,
            "LARGE_STRUCTURE_DEPTH_CONTACT_MAX_RANGE_M": 8.0,
            "LARGE_STRUCTURE_DEPTH_CONTACT_MIN_CONFIDENCE": 0.38,
            "LARGE_STRUCTURE_DEPTH_CONTACT_UNCERTAINTY_M": 1.5,
        }
    )
    instance = TargetInstanceBelief(
        instance_id="building:1",
        encounter_order=1,
        target_world=[surface_x + 20.0, 0.0, 0.0],
        confidence=0.36,
        uncertainty_m=1.5,
        surface_points_world=[[surface_x, -5.0, -5.0], [surface_x, 5.0, 5.0]],
        surface_patches_world=[[[surface_x, -5.0, -5.0], [surface_x, 5.0, 5.0]]],
        surface_bounds_world=[[surface_x, -5.0, -5.0], [surface_x, 5.0, 5.0]],
        surface_observation_count=1,
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
    memory.stage_local_instances[stage_key(stage)] = [instance.instance_id]
    memory.stage_locks[stage_key(stage)] = instance.instance_id
    return memory


def test_failed_stage_does_not_advance_to_future_action():
    target = _building_stage()
    turn = TaskStage(index=1, instruction="Turn right", mode="action", action="right", value=90.0)
    manager = TaskManager()
    manager.start_with_stages("building then turn", [target, turn])

    failed = manager.fail_current("target was never bound")

    assert failed is target
    assert manager.current_stage() is target
    assert manager.current_index == 0
    assert manager.is_stage_failed(0)
    assert not manager.is_stage_completed(0)
    assert "failed1" in manager.summary()
    assert "pending2" in manager.summary()


def test_runtime_terminal_failure_never_executes_future_action():
    target = _building_stage()
    turn = TaskStage(index=1, instruction="Turn right", mode="action", action="right", value=90.0)
    manager = TaskManager()
    manager.start_with_stages("building then turn", [target, turn])
    state_values = {}
    objects = SimpleNamespace(
        controller=SimpleNamespace(clear=lambda: None),
        completion_pipeline=None,
        distance_estimator=SimpleNamespace(clear=lambda: None),
        navigation_metrics=SimpleNamespace(invalidate_target=lambda _key: None),
        task_failed=False,
        task_manager=manager,
        completion_retry_after={(0, target.instruction): 1.0},
    )
    path_stream = SimpleNamespace(stop=lambda: None)
    state = SimpleNamespace(update=lambda **values: state_values.update(values))

    should_break = _fail_current_stage_after_relocalization(
        objects,
        path_stream,
        state,
        (0, target.instruction),
        "target was never bound",
    )

    assert should_break
    assert objects.task_failed
    assert manager.current_stage() is target
    assert manager.current_index == 0
    assert state_values["status"] == "failed"
    assert not state_values["task_done"]


def test_delayed_binding_keeps_activation_view_coordinates():
    stage = _building_stage()
    memory = MissionMemory(
        config={
            "ENABLED": True,
            "VIEW_RELATIVE_MIN_FORWARD_M": 0.5,
            "VIEW_RELATIVE_LATERAL_MARGIN_M": 0.5,
        }
    )
    memory.begin_view_relative_binding(stage, [0.0, 0.0, 0.0], 0.0)

    allowed = memory._view_relative_observation_allowed(
        stage,
        {
            "view": "front",
            "world": [10.0, 4.0, 0.0],
            # These current-view projections deliberately claim the opposite
            # side. The frozen activation frame must win.
            "forward_projection": 4.0,
            "lateral_projection": -10.0,
        },
    )

    assert allowed


def test_stale_active_queue_crossing_locked_facade_is_rejected():
    stage = _building_stage()
    memory = _locked_building_memory(stage, surface_x=10.0)
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=SimpleNamespace(
            queue=SimpleNamespace(world_waypoints=[[5.0, 0.0, 0.0], [20.0, 0.0, 0.0]])
        ),
    )

    assert _active_queue_overshoots_locked_surface(
        objects,
        stage,
        current_world=[0.0, 0.0, 0.0],
        trigger_radius_m=4.0,
    )


def test_active_queue_that_enters_and_stays_near_facade_is_kept():
    stage = _building_stage()
    memory = _locked_building_memory(stage, surface_x=10.0)
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=SimpleNamespace(
            queue=SimpleNamespace(
                world_waypoints=[
                    [5.0, 0.0, 0.0],
                    [7.0, 0.0, 0.0],
                    [8.0, 1.0, 0.0],
                    [9.0, 2.0, 0.0],
                ]
            )
        ),
    )

    assert not _active_queue_overshoots_locked_surface(
        objects,
        stage,
        current_world=[0.0, 0.0, 0.0],
        trigger_radius_m=4.0,
    )


def test_continuous_path_velocity_never_drops_below_configured_floor():
    objects = SimpleNamespace(
        controller=SimpleNamespace(
            planning=True,
            reserve_time_s=4.5,
            queue=SimpleNamespace(world_waypoints=[[0.2, 0.0, 0.0]]),
        )
    )

    assert _continuous_path_velocity(objects, [0.0, 0.0, 0.0]) >= 1.0


def test_memory_guided_leg_is_bounded_before_queueing():
    limited, changed = _limit_cumulative_path_length(
        [[80.0, 0.0, 0.0], [160.0, 0.0, 0.0]],
        {"PATH_MAX_GUIDED_LEG_M": 12.0},
    )

    assert changed
    assert limited == [[12.0, 0.0, 0.0]]


def test_queue_is_truncated_at_first_xy_arrival_radius_entry():
    stage = _building_stage()
    memory = _locked_building_memory(stage, surface_x=10.0)
    memory.primary_instance(stage).surface_points_world = [[10.0, 0.0, 0.0]]
    memory.primary_instance(stage).surface_patches_world = [[[10.0, 0.0, 0.0]]]
    # The path entry is inset by 0.25m, so it reaches x=5.75 for a surface at
    # x=10 and completion radius=4.5. The later point must never reach AirSim.
    queue = SimpleNamespace(world_waypoints=[[6.0, 0.0, 0.0], [20.0, 0.0, 0.0]])
    stopped = []
    discarded = []
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=SimpleNamespace(
            queue=queue,
            discard_plan=lambda: discarded.append(True),
        ),
    )
    path_stream = SimpleNamespace(stop=lambda: stopped.append(True))
    state_values = {}
    state = SimpleNamespace(update=lambda **values: state_values.update(values))

    result = _truncate_queue_at_completion_radius(
        objects,
        path_stream,
        state,
        stage,
        [0.0, 0.0, 0.0],
        {"source": "mission_memory", "target_world": [10.0, 0.0, 0.0], "trigger_radius_m": 4.5},
        4.5,
    )

    assert result == "truncated"
    assert len(queue.world_waypoints) == 1
    assert queue.world_waypoints[0][0] == 5.75
    assert stopped == [True]
    assert discarded == [True]
    assert len(state_values["trajectory_queue"]) == 1


def test_queue_guard_reports_arrived_without_reissuing_path():
    stage = _building_stage()
    memory = _locked_building_memory(stage, surface_x=10.0)
    memory.primary_instance(stage).surface_points_world = [[10.0, 0.0, 0.0]]
    memory.primary_instance(stage).surface_patches_world = [[[10.0, 0.0, 0.0]]]
    queue = SimpleNamespace(world_waypoints=[[8.0, 0.0, 0.0]])
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=SimpleNamespace(queue=queue, discard_plan=lambda: None),
    )
    path_stream = SimpleNamespace(stop=lambda: None)
    state = SimpleNamespace(update=lambda **_values: None)

    result = _truncate_queue_at_completion_radius(
        objects,
        path_stream,
        state,
        stage,
        [6.0, 0.0, 0.0],
        {"source": "mission_memory", "target_world": [10.0, 0.0, 0.0], "trigger_radius_m": 4.5},
        4.5,
    )

    assert result == "arrived"


def test_queue_guard_uses_earliest_entry_across_all_surface_samples():
    stage = _building_stage()
    memory = _locked_building_memory(stage, surface_x=10.0)
    instance = memory.primary_instance(stage)
    instance.surface_points_world = [[10.0, 0.0, 0.0], [20.0, 0.0, 0.0]]
    instance.surface_patches_world = [[[10.0, 0.0, 0.0]], [[20.0, 0.0, 0.0]]]
    queue = SimpleNamespace(world_waypoints=[[30.0, 0.0, 0.0]])
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=SimpleNamespace(queue=queue, discard_plan=lambda: None),
    )

    result = _truncate_queue_at_completion_radius(
        objects,
        SimpleNamespace(stop=lambda: None),
        SimpleNamespace(update=lambda **_values: None),
        stage,
        [0.0, 0.0, 0.0],
        {"source": "mission_memory", "target_world": [10.0, 0.0, 0.0], "trigger_radius_m": 4.5},
        4.5,
    )

    assert result == "truncated"
    assert queue.world_waypoints == [[5.75, 0.0, 0.0]]


def test_completion_watchdog_emergency_stops_and_clears_queue(monkeypatch):
    stage = _building_stage()
    queue = SimpleNamespace(world_waypoints=[[20.0, 0.0, 0.0]])
    cleared = []
    stopped = []
    objects = SimpleNamespace(
        task_manager=SimpleNamespace(current_stage=lambda: stage),
        controller=SimpleNamespace(
            queue=queue,
            planning=False,
            has_plan_job=False,
            clear=lambda: (cleared.append(True), queue.world_waypoints.clear()),
        ),
    )
    client = SimpleNamespace(get_pose=lambda: ([6.0, 0.0, 0.0], 0.0))
    path_stream = SimpleNamespace(
        active=True,
        emergency_stop=lambda: stopped.append(True),
    )
    monkeypatch.setattr(
        runtime_module,
        "_cached_target_distance",
        lambda *_args, **_kwargs: SimpleNamespace(
            source="mission_memory",
            distance_m=4.0,
            target_world=[10.0, 0.0, 0.0],
        ),
    )
    monkeypatch.setattr(runtime_module, "_should_trigger_completion_vlm", lambda *_args, **_kwargs: True)

    watchdog = _CompletionRadiusWatchdog(objects, client, path_stream, stage, 4.5, 0.02)
    watchdog.start()
    watchdog._thread.join(timeout=0.5)
    watchdog.stop()

    assert stopped == [True]
    assert cleared == [True]
    assert queue.world_waypoints == []


def test_very_near_collision_refreshes_locked_facade_contact():
    stage = _building_stage()
    memory = _locked_building_memory(stage, surface_x=1.0)
    objects = SimpleNamespace(mission_memory=memory)

    contact = _record_locked_target_collision(
        objects,
        stage,
        collision_world=[0.0, 0.0, 0.0],
        collision_yaw_deg=0.0,
    )

    recent = memory.recent_locked_large_surface_contact(stage)
    decision = memory.evaluate_completion(
        stage=stage,
        current_world=[-2.0, 0.0, 0.0],
        fresh_visual_support=False,
        stop_radius_m=4.0,
    )
    assert contact is not None
    assert recent is not None
    assert recent["kind"] == "collision"
    assert memory.primary_instance(stage).confidence >= 0.38
    assert decision.done


def test_unrelated_collision_does_not_refresh_facade_contact():
    stage = _building_stage()
    memory = _locked_building_memory(stage, surface_x=10.0)
    objects = SimpleNamespace(mission_memory=memory)

    contact = _record_locked_target_collision(
        objects,
        stage,
        collision_world=[0.0, 0.0, 0.0],
        collision_yaw_deg=0.0,
    )

    assert contact is None
    assert memory.recent_locked_large_surface_contact(stage) is None


def test_recent_facade_contact_completes_before_far_detector_can_veto():
    stage = _building_stage()
    turn = TaskStage(index=1, instruction="Turn right", mode="action", action="right", value=90.0)
    manager = TaskManager()
    manager.start_with_stages("building then turn", [stage, turn])
    memory = _locked_building_memory(stage, surface_x=1.0)
    collision_objects = SimpleNamespace(mission_memory=memory)
    assert _record_locked_target_collision(
        collision_objects,
        stage,
        collision_world=[0.0, 0.0, 0.0],
        collision_yaw_deg=0.0,
    ) is not None

    state_values = {}
    metrics = SimpleNamespace(
        update_target=lambda *_args, **_kwargs: None,
        record_distance=lambda *_args, **_kwargs: None,
        task_completed=False,
    )
    objects = SimpleNamespace(
        mission_memory=memory,
        task_manager=manager,
        controller=SimpleNamespace(clear=lambda: None),
        completion_pipeline=None,
        navigation_metrics=metrics,
        completion_attempts={},
        completion_retry_after={},
    )
    client = SimpleNamespace(get_pose=lambda: ([-2.0, 0.0, 0.0], 0.0))
    path_stream = SimpleNamespace(stop=lambda: None)
    state = SimpleNamespace(update=lambda **values: state_values.update(values))

    task_done = _handle_distance_completion_trigger(
        objects,
        client,
        path_stream,
        state,
        stage,
        "Fly to the first building in the right front",
        "front",
        {
            "distance_m": 2.8,
            "source": "mission_memory",
            "distance_kind": "surface",
            "trigger_radius_m": 4.5,
        },
        4.0,
    )

    assert not task_done
    assert manager.is_stage_completed(0)
    assert manager.current_stage() is turn


def test_large_surface_arrival_skips_fresh_rgb_and_vlm(monkeypatch):
    stage = _building_stage()
    manager = TaskManager()
    manager.start_with_stages("building", [stage])
    memory = _locked_building_memory(stage, surface_x=1.0)
    memory.primary_instance(stage).surface_points_world = [[1.0, 0.0, -20.0]]
    memory.primary_instance(stage).surface_bounds_world = [[1.0, 0.0, -20.0], [1.0, 0.0, -20.0]]
    state_values = {}
    objects = SimpleNamespace(
        mission_memory=memory,
        task_manager=manager,
        controller=SimpleNamespace(clear=lambda: None),
        completion_pipeline=None,
        navigation_metrics=SimpleNamespace(
            update_target=lambda *_args, **_kwargs: None,
            record_distance=lambda *_args, **_kwargs: None,
            task_completed=False,
        ),
        completion_attempts={},
        completion_retry_after={},
    )
    client = SimpleNamespace(get_pose=lambda: ([4.0, 0.0, 100.0], 0.0))
    path_stream = SimpleNamespace(stop=lambda: None)
    state = SimpleNamespace(update=lambda **values: state_values.update(values))

    def _unexpected_visual_call(*_args, **_kwargs):
        raise AssertionError("large surface completion must not capture RGB or call VLM")

    monkeypatch.setattr(runtime_module, "_capture_fresh_rgb_frames", _unexpected_visual_call)
    monkeypatch.setattr(runtime_module, "_detect_dual_view", _unexpected_visual_call)

    task_done = _handle_distance_completion_trigger(
        objects,
        client,
        path_stream,
        state,
        stage,
        stage.instruction,
        "front",
        {"distance_m": 3.0, "source": "mission_memory", "distance_kind": "surface"},
        4.0,
    )

    assert task_done
    assert manager.is_done()
