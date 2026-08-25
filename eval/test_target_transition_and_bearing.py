"""Focused tests for bearing-only navigation and entity transitions."""

from types import SimpleNamespace

from PIL import Image

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
