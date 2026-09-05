"""Focused regressions for smooth small-target ``above`` control."""

from __future__ import annotations

from concurrent.futures import Future
from types import SimpleNamespace

from PIL import Image

from agent.functions.fast_slow.completion_pipeline import DetectionDepthBundle
from agent.functions.fast_slow.controller import FastSlowController, PlanningJob
from agent.functions.fast_slow.runtime import (
    _apply_small_above_target_guidance,
    _handle_distance_completion_trigger,
    _handle_small_above_down_center_observation,
    _should_trigger_completion_vlm,
    _truncate_queue_at_completion_radius,
)
from agent.models.detection.base import DetectionResult


def _stage():
    return SimpleNamespace(
        index=0,
        instruction="Fly above the fountain",
        mode="target",
        target="fountain",
        relation="above",
        completion_condition="",
    )


class _SmallMemory:
    def __init__(self):
        self.instance = SimpleNamespace(
            instance_id="fountain:1",
            is_large_structure=False,
            target_world=[7.161, -6.217, 0.0],
            confidence=0.9,
            uncertainty_m=0.5,
        )
        self.config = {
            "ABOVE_TARGET_GUIDANCE_ENABLED": True,
            "ABOVE_PATH_CENTER_RADIUS_M": 1.0,
            "ABOVE_TARGET_GUIDANCE_MIN_HORIZON_M": 12.0,
            "ABOVE_TARGET_GUIDANCE_SPACING_M": 3.0,
            "ABOVE_TARGET_GUIDANCE_MAX_WAYPOINTS": 6,
            "ABOVE_TARGET_GUIDANCE_MAX_DEVIATION_DEG": 15.0,
            "ABOVE_TARGET_GUIDANCE_MIN_PROGRESS_RATIO": 0.15,
            "ABOVE_CONTINUOUS_FALLBACK_PLANNING_S": 5.5,
            "ABOVE_CONTINUOUS_HORIZON_MARGIN_S": 1.5,
            "PATH_MAX_GUIDED_LEG_M": 18.0,
            "MEMORY_DISTANCE_MIN_CONFIDENCE": 0.45,
            "BEARING_MEMORY_MAX_UNCERTAINTY_M": 5.0,
            "BEARING_MEMORY_MAX_AGE_S": 30.0,
            "ABOVE_DOWN_CENTER_TOLERANCE_RATIO": 0.22,
            "ABOVE_DOWN_CENTER_CONFIRMATIONS": 2,
        }

    def primary_instance(self, _stage):
        return self.instance

    @staticmethod
    def is_primary_locked(_stage):
        return True

    @staticmethod
    def estimate_distance(_stage, _current_world):
        return {
            "confidence": 0.9,
            "uncertainty_m": 0.5,
            "observation_age_s": 1.0,
        }

    @staticmethod
    def previous_entity_exclusions(_stage, _world, _yaw):
        return []

    @staticmethod
    def evaluate_locked_detection_identity(*_args, **_kwargs):
        return {"accepted": True, "reason": "locked_identity_match"}

    @staticmethod
    def archive_stage(_stage, _reason):
        return None

    @staticmethod
    def summary(_stage):
        return {}


def test_locked_small_above_curves_toward_lateral_target_instead_of_flying_past():
    memory = _SmallMemory()
    controller = SimpleNamespace(required_planning_horizon_m=lambda _speed: 14.0)
    objects = SimpleNamespace(mission_memory=memory, controller=controller)
    context = {
        "enabled": True,
        "target_body": [7.161, -6.217, 0.0],
        "confidence": 0.9,
        "uncertainty_m": 0.5,
    }

    guided, reason = _apply_small_above_target_guidance(
        objects,
        _stage(),
        [[5.16, -0.01, 0.0], [7.96, -0.01, 0.0], [18.0, -0.01, 0.0]],
        context,
        selection_pos=[0.0, 0.0, -20.0],
        planning_wall_s=5.3,
    )

    assert reason.startswith("smooth_locked_target_")
    assert len(guided) >= 3
    assert guided[0][0] > 0.0
    assert guided[0][1] > guided[-1][1]
    assert abs(guided[-1][0] - 7.161) < 0.05
    assert abs(guided[-1][1] + 6.217) < 0.05
    assert all(point[2] == 0.0 for point in guided)


def test_small_above_queue_is_not_cut_by_generic_distance_radius():
    memory = _SmallMemory()
    controller = SimpleNamespace(
        queue=SimpleNamespace(
            world_waypoints=[
                [3.0, -2.0, -20.0],
                [7.161, -6.217, -20.0],
                [14.0, -10.0, -20.0],
            ]
        ),
        discard_plan=lambda: None,
    )
    objects = SimpleNamespace(mission_memory=memory, controller=controller)
    path_stream = SimpleNamespace(emergency_stop=lambda: None)
    state = SimpleNamespace(update=lambda **_kwargs: None)
    cached = SimpleNamespace(
        source="mission_memory",
        target_world=[7.161, -6.217, -20.0],
        trigger_radius_m=1.0,
    )

    original_queue = [list(point) for point in controller.queue.world_waypoints]
    result = _truncate_queue_at_completion_radius(
        objects,
        path_stream,
        state,
        _stage(),
        [0.0, 0.0, -20.0],
        cached,
        5.0,
    )

    assert result == "none"
    assert controller.queue.world_waypoints == original_queue


def test_small_above_distance_trigger_is_ignored_until_down_center_confirmation():
    memory = _SmallMemory()
    objects = SimpleNamespace(mission_memory=memory)
    estimated = SimpleNamespace(
        source="mission_memory",
        distance_m=2.0,
        target_world=[7.161, -6.217, 0.0],
        current_world=[6.0, -5.0, -20.0],
    )

    assert not _should_trigger_completion_vlm(objects, _stage(), estimated, 4.0)


def test_stale_small_above_distance_trigger_keeps_active_queue():
    memory = _SmallMemory()
    queue = SimpleNamespace(world_waypoints=[[5.0, -4.0, -20.0]])
    stop_calls = []
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=SimpleNamespace(queue=queue, clear=lambda: queue.world_waypoints.clear()),
    )

    result = _handle_distance_completion_trigger(
        objects,
        client=SimpleNamespace(get_pose=lambda: ([5.0, -4.0, -20.0], 0.0)),
        path_stream=SimpleNamespace(emergency_stop=lambda: stop_calls.append(True)),
        state=SimpleNamespace(update=lambda **_kwargs: None),
        stage=_stage(),
        task_text="Fly above the fountain",
        capture_mode="front",
        cached_distance={"distance_m": 2.0, "target_world": [7.161, -6.217, 0.0]},
        trigger_radius_m=4.0,
    )

    assert not result
    assert queue.world_waypoints == [[5.0, -4.0, -20.0]]
    assert stop_calls == []


def test_planning_horizon_uses_latency_ema_and_margin():
    controller = FastSlowController({
        "PLANNING_LATENCY_INITIAL_S": 5.5,
        "PLANNING_LATENCY_MARGIN_S": 1.5,
        "RESERVE_TIME_S": 4.5,
    })
    try:
        assert controller.required_planning_reserve_s() == 7.0
        assert controller.required_planning_horizon_m(2.0) == 14.0
    finally:
        controller.shutdown()


def test_full_queue_replenishes_when_time_horizon_is_shorter_than_latency_reserve():
    controller = FastSlowController(
        {
            "MAX_PENDING": 5,
            "RESERVE_TIME_S": 4.5,
            "PLANNING_LATENCY_INITIAL_S": 5.5,
            "PLANNING_LATENCY_MARGIN_S": 1.5,
            "QUEUE_REPLENISH_TRIGGER_S": 1.0,
        }
    )
    try:
        controller.queue.world_waypoints = [[3.0 * index, 0.0, 0.0] for index in range(1, 6)]
        assert not controller.can_submit_plan(current_pos=[0.0, 0.0, 0.0], velocity_mps=1.0)
        controller.queue.world_waypoints = [[1.0 * index, 0.0, 0.0] for index in range(1, 6)]
        assert controller.can_submit_plan(current_pos=[0.0, 0.0, 0.0], velocity_mps=1.0)
    finally:
        controller.shutdown()


def test_stale_plan_rebase_replaces_old_queue_suffix():
    controller = FastSlowController(
        {
            "MAX_PENDING": 5,
            "STALE_PLAN_REBASE_DISTANCE_M": 2.5,
        }
    )
    try:
        controller.queue.world_waypoints = [[5.0, 0.0, 0.0], [10.0, 0.0, 0.0]]
        future = Future()
        future.set_result(
            SimpleNamespace(
                waypoints=[[2.0, 0.0, 0.0]],
                waypoint_format="incremental_body",
            )
        )
        controller._job = PlanningJob(
            future=future,
            plan_pos=[0.0, 0.0, 0.0],
            plan_yaw_deg=0.0,
            pending_world=[[5.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
            generation=controller._generation,
        )

        result = controller.poll_plan(current_pos=[4.0, 0.0, 0.0], current_yaw_deg=0.0)

        assert result is not None
        assert getattr(result, "_stale_plan_rebased", False)
        assert controller.queue.world_waypoints == [[6.0, 0.0, 0.0]]
    finally:
        controller.shutdown()


def test_plan_rebases_after_vehicle_passes_old_anchor_endpoint():
    controller = FastSlowController({"STALE_PLAN_REBASE_DISTANCE_M": 0.5})
    try:
        controller.queue.world_waypoints = [[5.0, 0.0, 0.0], [10.0, 0.0, 0.0]]
        future = Future()
        future.set_result(
            SimpleNamespace(
                waypoints=[[2.0, 0.0, 0.0]],
                waypoint_format="incremental_body",
            )
        )
        controller._job = PlanningJob(
            future=future,
            plan_pos=[0.0, 0.0, 0.0],
            plan_yaw_deg=0.0,
            pending_world=[[5.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
            generation=controller._generation,
        )

        controller.poll_plan(current_pos=[10.4, 0.0, 0.0], current_yaw_deg=0.0)

        assert controller.queue.world_waypoints == [[12.4, 0.0, 0.0]]
    finally:
        controller.shutdown()


def test_two_fresh_down_center_hits_complete_plain_above_and_first_hit_stops():
    memory = _SmallMemory()
    image = Image.new("RGB", (100, 80), "white")
    image.camera_frame = SimpleNamespace(optical_axis_world=[0.0, 0.0, 1.0])
    stop_calls = []

    class _TaskManager:
        completed = False

        def complete_current(self, _reason):
            self.completed = True

        @staticmethod
        def summary():
            return "1/1 stages completed"

        def is_done(self):
            return self.completed

    navigation_metrics = SimpleNamespace(
        task_completed=False,
        update_target=lambda *_args, **_kwargs: None,
        record_distance=lambda *_args, **_kwargs: None,
    )
    objects = SimpleNamespace(
        mission_memory=memory,
        above_stage_states={},
        completion_checker=SimpleNamespace(_camera_points_downward=lambda _image: True),
        completion_pipeline=None,
        controller=SimpleNamespace(clear=lambda: stop_calls.append("clear")),
        task_manager=_TaskManager(),
        navigation_metrics=navigation_metrics,
        completion_attempts={},
        completion_retry_after={},
    )
    path_stream = SimpleNamespace(
        emergency_stop=lambda: stop_calls.append("stop"),
        stop=lambda: stop_calls.append("stop"),
    )
    state = SimpleNamespace(update=lambda **_kwargs: None)
    stage = _stage()

    def bundle(capture_id):
        detection = DetectionResult(
            visible=True,
            bbox=[35, 25, 65, 55],
            score=0.9,
            label="fountain",
            camera="down",
            capture_id=capture_id,
        )
        return DetectionDepthBundle(
            best_detection=detection,
            front_detection=None,
            down_detection=detection,
            down_detections=[detection],
            down_image=image,
            observer_world=[7.1, -6.2, -20.0],
            observer_yaw_deg=0.0,
            capture_id=capture_id,
        )

    handled, done = _handle_small_above_down_center_observation(
        objects, path_stream, state, stage, bundle("capture-1")
    )
    assert handled and not done
    assert stop_calls

    handled, done = _handle_small_above_down_center_observation(
        objects, path_stream, state, stage, bundle("capture-2")
    )
    assert handled and done
    assert objects.task_manager.completed
    assert navigation_metrics.task_completed
