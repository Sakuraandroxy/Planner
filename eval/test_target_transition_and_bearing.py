"""Focused tests for bearing-only navigation and entity transitions."""

from types import SimpleNamespace

from PIL import Image

from agent.functions.common.web_runtime_helpers import target_depth_text
from agent.functions.fast_slow.runtime import (
    _above_queue_violation_reason,
    _apply_above_altitude_path_guard,
)
from agent.functions.memory import MissionMemory
from agent.functions.obstacle_avoidance.local_depth_avoider import DepthObstacleAvoider
from agent.functions.obstacle_avoidance.schemas import ObstacleCell
from agent.functions.perception import TargetBearingTracker, metric_depth_usable
from agent.functions.task_parser.base import TaskStage


def _detection(depth, bbox=(360, 180, 500, 340)):
    return SimpleNamespace(
        visible=True,
        bbox=list(bbox),
        score=0.9,
        label="building",
        depth_median=depth,
        depth_valid_ratio=None,
        depth_mad_m=None,
        surface_depth_samples=None,
        camera="front",
    )


def _mixed_surface_detection(*, label="building facade", bbox=(360, 80, 620, 450)):
    samples = [
        [u, v, 24.0 + offset]
        for v, offset in ((0.20, -0.4), (0.45, 0.0), (0.75, 0.4))
        for u in (0.62, 0.75, 0.88)
    ]
    return SimpleNamespace(
        visible=True,
        bbox=list(bbox),
        score=0.62,
        label=label,
        depth_median=24.0,
        depth_bbox_median=55.0,
        depth_p10_m=22.0,
        depth_p90_m=82.0,
        depth_valid_ratio=0.92,
        depth_mad_m=0.5,
        depth_sample_count=100,
        surface_depth_samples=samples,
        camera="front",
    )


def _memory():
    return MissionMemory(
        {
            "ENABLED": True,
            "MIN_DETECTION_SCORE": 0.1,
            "LOCK_MIN_CONFIDENCE": 0.1,
            "VIEW_RELATIVE_MIN_FORWARD_M": 0.5,
            "VIEW_RELATIVE_LATERAL_MARGIN_M": 0.5,
            "VIEW_RELATIVE_MAX_BEHIND_M": 1.0,
            "PREVIOUS_ENTITY_EXCLUSION_RADIUS_M": 8.0,
            "PREVIOUS_ENTITY_EXCLUSION_MAX_RADIUS_M": 10.0,
        },
        {"FRONT_FOV": 90.0},
    )


def test_far_depth_keeps_rgb_bearing_without_metric_lock():
    tracker = TargetBearingTracker(
        {
            "BEARING_MIN_CONFIDENCE": 0.4,
            "METRIC_LOCK_MAX_DEPTH_M": 120.0,
        },
        {"FRONT_FOV": 90.0},
    )
    detection = _detection(164.7)
    assert metric_depth_usable(detection, tracker.config) == (False, "too_far")
    observation = tracker.record(
        stage_key=(0,),
        detection=detection,
        image=Image.new("RGB", (640, 480)),
        observer_yaw_deg=10.0,
    )
    assert observation is not None
    assert observation.depth_state == "too_far"
    assert observation.relative_angle_deg > 0.0


def test_mixed_foreground_background_bbox_depth_cannot_create_metric_lock():
    detection = _detection(100.0)
    detection.depth_valid_ratio = 0.9
    detection.depth_mad_m = 0.5
    detection.depth_bbox_median = 76.0
    detection.depth_p10_m = 74.0
    detection.depth_p90_m = 103.0

    usable, reason = metric_depth_usable(
        detection,
        {
            "METRIC_LOCK_MAX_DEPTH_M": 120.0,
            "METRIC_DEPTH_MAX_CENTER_BBOX_DELTA_RATIO": 0.10,
            "METRIC_DEPTH_MAX_P10_P90_RATIO": 0.18,
        },
    )

    assert not usable
    assert reason == "mixed_bbox_depth"


def test_mixed_depth_building_uses_coherent_facade_to_lock_and_climb_early():
    memory = _memory()
    memory.config.update({
        "ABOVE_ALTITUDE_GUARD_ENABLED": True,
        "ABOVE_PRE_ROOF_FACADE_CLIMB_ENABLED": True,
        "ABOVE_PRE_ROOF_FACADE_CLEARANCE_M": 3.0,
        "ABOVE_PRE_ROOF_FACADE_UNCERTAINTY_MARGIN_M": 1.0,
        "ABOVE_PRE_ROOF_MAX_CLIMB_LEG_M": 10.0,
        "AIRSIM_MIN_VERTICAL_COMMAND_M": 5.5,
        "AIRSIM_MIN_CLIMB_COMMAND_M": 10.0,
        "ABOVE_REQUIRE_ROOF_GEOMETRY": True,
        "METRIC_LOCK_MAX_DEPTH_M": 120.0,
        "METRIC_DEPTH_MAX_CENTER_BBOX_DELTA_RATIO": 0.10,
        "METRIC_DEPTH_MAX_P10_P90_RATIO": 0.18,
        "LARGE_STRUCTURE_SURFACE_FALLBACK_ENABLED": True,
        "LARGE_STRUCTURE_SURFACE_FALLBACK_MIN_SAMPLES": 6,
    })
    stage = TaskStage(
        index=0,
        instruction="Fly above the first building ahead on the right",
        target="building",
        relation="above",
        ordinal=1,
        selection_rule="ordinal",
        view_relative=True,
    )
    memory.begin_view_relative_binding(stage, [0.0, 0.0, -6.0], 0.0)
    events = memory.update_from_detections(
        stage=stage,
        detections_by_view={"front": [_mixed_surface_detection()]},
        images_by_view={"front": Image.new("RGB", (640, 480), (120, 120, 120))},
        observer_world=[0.0, 0.0, -6.0],
        observer_yaw_deg=0.0,
    )

    instance = memory.primary_instance(stage)
    assert events
    assert instance is not None
    assert instance.surface_observation_count == 1
    assert getattr(events[0].detection, "surface_lock_fallback", False)
    assert not instance.roof_points_world

    objects = SimpleNamespace(mission_memory=memory, above_stage_states={})
    guarded, reason = _apply_above_altitude_path_guard(
        objects,
        stage,
        [[12.0, 0.0, 0.0]],
        selection_pos=[0.0, 0.0, -6.0],
    )

    assert guarded == [[0.0, 0.0, -10.0]]
    assert "roof_unknown_climb_above_observed_facade" in reason
    assert "vertical_first" in reason


def test_real_depth_attachment_with_window_holes_still_locks_facade():
    import numpy as np

    image = Image.new("RGB", (640, 480), (120, 120, 120))
    depth = np.full((48, 64), 80.0, dtype=np.float32)
    depth[8:46, 36:63] = 24.0
    depth[8:46, 42:45] = 80.0
    depth[8:46, 50:53] = 80.0
    depth[8:46, 58:60] = 80.0
    detection = _mixed_surface_detection()
    detection.depth_median = None
    detection.surface_depth_samples = None
    target_depth_text("front", detection, image, depth)

    usable, reason = metric_depth_usable(detection, {"METRIC_LOCK_MAX_DEPTH_M": 120.0})
    assert not usable
    assert reason == "mixed_center_depth"

    memory = _memory()
    stage = TaskStage(
        index=0,
        instruction="Fly above the first building ahead on the right",
        target="building",
        relation="above",
        ordinal=1,
        selection_rule="ordinal",
        view_relative=True,
    )
    memory.begin_view_relative_binding(stage, [0.0, 0.0, -6.0], 0.0)
    events = memory.update_from_detections(
        stage=stage,
        detections_by_view={"front": [detection]},
        images_by_view={"front": image},
        observer_world=[0.0, 0.0, -6.0],
        observer_yaw_deg=0.0,
    )

    assert events
    assert memory.primary_instance(stage) is not None
    assert getattr(events[0].detection, "surface_lock_fallback", False)


def test_new_facade_lock_invalidates_forward_queue_before_wall_contact():
    memory = _memory()
    memory.config.update({
        "ABOVE_ALTITUDE_GUARD_ENABLED": True,
        "ABOVE_PRE_ROOF_FACADE_CLIMB_ENABLED": True,
        "AIRSIM_MIN_VERTICAL_COMMAND_M": 5.5,
        "METRIC_LOCK_MAX_DEPTH_M": 120.0,
        "LARGE_STRUCTURE_SURFACE_FALLBACK_ENABLED": True,
    })
    stage = TaskStage(
        index=0,
        instruction="Fly above the first building ahead on the right",
        target="building",
        relation="above",
        ordinal=1,
        selection_rule="ordinal",
        view_relative=True,
    )
    current = [0.0, 0.0, -6.0]
    memory.begin_view_relative_binding(stage, current, 0.0)
    memory.update_from_detections(
        stage=stage,
        detections_by_view={"front": [_mixed_surface_detection()]},
        images_by_view={"front": Image.new("RGB", (640, 480), (120, 120, 120))},
        observer_world=current,
        observer_yaw_deg=0.0,
    )
    objects = SimpleNamespace(
        mission_memory=memory,
        above_stage_states={},
        controller=SimpleNamespace(queue=SimpleNamespace(world_waypoints=[[15.0, 8.0, -6.0]])),
    )

    assert _above_queue_violation_reason(objects, stage, current) == (
        "queued_xy_motion_before_above_facade_clearance"
    )


def test_full_frame_building_facade_can_still_create_surface_lock():
    memory = _memory()
    memory.config.update({
        "METRIC_LOCK_MAX_DEPTH_M": 120.0,
        "LARGE_STRUCTURE_SURFACE_FALLBACK_ENABLED": True,
    })
    stage = TaskStage(
        index=0,
        instruction="Fly above the building",
        target="building",
        relation="above",
    )
    detection = _mixed_surface_detection(bbox=(0, 0, 640, 480))
    events = memory.update_from_detections(
        stage=stage,
        detections_by_view={"front": [detection]},
        images_by_view={"front": Image.new("RGB", (640, 480), (120, 120, 120))},
        observer_world=[0.0, 0.0, -6.0],
        observer_yaw_deg=0.0,
    )

    assert events
    assert memory.primary_instance(stage) is not None


def test_mixed_depth_small_target_cannot_use_facade_fallback():
    memory = _memory()
    stage = TaskStage(
        index=0,
        instruction="Fly above the car",
        target="car",
        relation="above",
    )
    events = memory.update_from_detections(
        stage=stage,
        detections_by_view={"front": [_mixed_surface_detection(label="car")]},
        images_by_view={"front": Image.new("RGB", (640, 480), (120, 120, 120))},
        observer_world=[0.0, 0.0, -6.0],
        observer_yaw_deg=0.0,
    )

    assert events == []
    assert memory.primary_instance(stage) is None


def test_new_view_rejects_previous_building_but_return_selects_it():
    memory = _memory()
    image = Image.new("RGB", (640, 480), (100, 100, 100))
    first = TaskStage(
        index=0,
        instruction="first building on the right",
        target="building",
        relation="near",
        ordinal=1,
        selection_rule="ordinal",
        view_relative=True,
    )
    memory.begin_view_relative_binding(first, [0.0, 0.0, 0.0], 0.0)
    memory.update_from_detections(
        stage=first,
        detections_by_view={"front": [_detection(10.0)]},
        images_by_view={"front": image},
        observer_world=[0.0, 0.0, 0.0],
        observer_yaw_deg=0.0,
    )
    first_id = memory.primary_instance(first).instance_id
    memory.archive_stage(first, "done")

    second = TaskStage(
        index=1,
        instruction="first building on the right after the turn",
        target="building",
        relation="near",
        ordinal=1,
        selection_rule="ordinal",
        view_relative=True,
    )
    memory.begin_view_relative_binding(second, [0.0, 0.0, 0.0], 0.0)
    rejected = memory.update_from_detections(
        stage=second,
        detections_by_view={"front": [_detection(10.0)]},
        images_by_view={"front": image},
        observer_world=[0.0, 0.0, 0.0],
        observer_yaw_deg=0.0,
    )
    assert rejected == []

    accepted = memory.update_from_detections(
        stage=second,
        detections_by_view={"front": [_detection(22.0)]},
        images_by_view={"front": image},
        observer_world=[0.0, 0.0, 0.0],
        observer_yaw_deg=0.0,
    )
    second_id = memory.primary_instance(second).instance_id
    assert accepted and second_id != first_id

    returned = TaskStage(
        index=2,
        instruction="fly back to the first building",
        target="building",
        relation="near",
        ordinal=1,
        return_target=True,
    )
    assert memory.primary_instance(returned).instance_id == first_id


def test_goal_near_obstacle_does_not_rejoin_original_suffix():
    avoider = DepthObstacleAvoider(
        {
            "ENABLED": True,
            "TARGET_KEEP_OUT_ENABLED": False,
            "BYPASS_ENABLED": True,
            "SAFETY_RADIUS_M": 1.0,
            "VERTICAL_CLEARANCE_M": 1.0,
            "BYPASS_LATERAL_M": 3.0,
            "STOP_BUFFER_M": 1.0,
            "OBSTACLE_TTL_S": 100.0,
            "MIN_CELL_CONFIDENCE": 0.0,
        },
        {},
    )
    avoider.obstacle_cells["wall"] = ObstacleCell(
        key="wall",
        center_world=[6.0, 0.0, 0.0],
        confidence=0.9,
    )
    result = avoider.filter_cumulative_waypoints(
        [[15.0, 0.0, 0.0], [25.0, 0.0, 0.0]],
        current_world=[0.0, 0.0, 0.0],
        yaw_deg=0.0,
        memory_context={
            "enabled": True,
            "target_body": [12.0, 0.0, 0.0],
            "footprint_radius_m": 1.0,
            "uncertainty_m": 0.5,
            "completion_radius_m": 4.5,
        },
    )
    assert result.reason == "depth_goal_near_bypass"
    assert result.details["resume_policy"] == "stop_and_reobserve"
    assert result.waypoints[-1][0] < 10.0
