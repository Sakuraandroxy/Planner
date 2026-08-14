"""Focused tests for activation-time target binding."""

from __future__ import annotations

from types import SimpleNamespace

from PIL import Image

from agent.functions.completion.task_completion import TaskCompletionChecker
from agent.functions.fast_slow.completion_pipeline import CompletionPipeline
from agent.functions.fast_slow.runtime import (
    _detection_reliability,
    _fresh_visual_support_matches_locked_memory,
    _memory_observation_stages,
)
from agent.functions.memory import MissionMemory, is_view_relative_stage
from agent.functions.memory.schemas import TargetInstanceBelief, TargetMemory
from agent.functions.task_parser.base import TaskStage
from agent.functions.task_parser.vlm_task_parser import parse_task_parser_to_stages


def _memory() -> MissionMemory:
    return MissionMemory(
        config={
            "ENABLED": True,
            "MIN_DETECTION_SCORE": 0.1,
            "LOCK_MIN_CONFIDENCE": 0.1,
            "MIN_ASSOCIATION_RADIUS_M": 2.0,
            "LARGE_STRUCTURE_ASSOCIATION_RADIUS_M": 25.0,
            "VIEW_RELATIVE_MIN_FORWARD_M": 0.5,
            "VIEW_RELATIVE_MAX_BEHIND_M": 1.0,
            "VIEW_RELATIVE_LATERAL_MARGIN_M": 0.5,
            "FRONT_CAMERA_OFFSET": [0.0, 0.0, 0.0],
        },
        sim_config={"FRONT_FOV": 90.0, "DOWN_FOV": 90.0},
    )


def _detection(depth: float, bbox) -> SimpleNamespace:
    return SimpleNamespace(
        visible=True,
        bbox=list(bbox),
        score=0.9,
        label="building",
        depth_median=float(depth),
        depth_bbox=None,
        surface_depth_samples=None,
        camera="front",
    )


def test_parser_distinguishes_view_relative_binding_from_return_target():
    stages = parse_task_parser_to_stages(
        """
        {
          "stages": [
            {
              "instruction": "Fly to the second building visible ahead in the new view",
              "mode": "target",
              "target": "building",
              "ordinal": 2,
              "selection_rule": "ordinal",
              "view_relative": false
            },
            {
              "instruction": "Fly back to the previously visited building",
              "mode": "target",
              "target": "building",
              "view_relative": true,
              "return_target": false
            },
            {
              "instruction": "Fly to the second red car",
              "mode": "target",
              "target": "red car",
              "ordinal": 2
            }
          ]
        }
        """
    )

    assert stages[0].view_relative is True
    assert stages[0].return_target is False
    assert stages[1].return_target is True
    assert stages[1].view_relative is False
    assert stages[2].view_relative is False


def test_bootstrap_and_future_scan_skip_view_relative_targets():
    current = TaskStage(index=0, instruction="Fly to the red car", target="red car")
    future_stable = TaskStage(index=1, instruction="Fly to the tower", target="tower")
    future_relative = TaskStage(
        index=2,
        instruction="Fly to the first building in the new view",
        target="building",
        ordinal=1,
        view_relative=True,
    )
    task_manager = SimpleNamespace(
        stages=[current, future_stable, future_relative],
        current_stage=lambda: current,
    )
    objects = SimpleNamespace(mission_memory=object(), task_manager=task_manager)

    bootstrap = _memory_observation_stages(objects, current_stage=None, future_only=False)
    future = _memory_observation_stages(objects, current_stage=current, future_only=True)

    assert future_relative not in bootstrap
    assert future_stable in bootstrap
    assert future == [future_stable]


def test_view_relative_ordinal_uses_only_activation_view_candidates():
    memory = _memory()
    image = Image.new("RGB", (640, 480), (120, 120, 120))
    old = TargetInstanceBelief(
        instance_id="building:1",
        encounter_order=1,
        target_world=[5.0, 0.0, -5.0],
        confidence=0.99,
        observation_count=3,
        status="completed",
    )
    target_memory = TargetMemory(
        target_key="building",
        target_name="building",
        primary_instance_id=old.instance_id,
        next_encounter_order=2,
    )
    target_memory.instances[old.instance_id] = old
    memory.target_memories["building"] = target_memory
    stage = TaskStage(
        index=3,
        instruction="Fly to the second building visible ahead in the new view",
        target="building",
        relation="near",
        ordinal=2,
        selection_rule="ordinal",
        view_relative=True,
    )

    memory.update_from_detections(
        stage=stage,
        detections_by_view={
            "front": [
                _detection(10.0, [270, 210, 310, 260]),
                _detection(40.0, [330, 210, 370, 260]),
            ]
        },
        images_by_view={"front": image},
        observer_world=[0.0, 0.0, -5.0],
        observer_yaw_deg=0.0,
    )

    primary = memory.primary_instance(stage)
    assert memory.local_instance_ids(stage) == ["building:2", "building:3"]
    assert primary is not None
    assert primary.instance_id == "building:3"
    assert "building:1" in memory.target_memories["building"].instances

    memory.archive_stage(stage, "done")
    return_stage = TaskStage(
        index=4,
        instruction="Fly back to the previously visited building",
        target="building",
        relation="near",
        return_target=True,
    )
    assert is_view_relative_stage(return_stage) is False
    assert memory.primary_instance(return_stage).instance_id == "building:3"


def test_reset_view_relative_binding_preserves_global_instances():
    memory = _memory()
    stage = TaskStage(
        index=2,
        instruction="Fly to the first building in the current view",
        target="building",
        ordinal=1,
        view_relative=True,
    )
    target_memory = TargetMemory(
        target_key="building",
        target_name="building",
        primary_instance_id="building:7",
    )
    target_memory.instances["building:7"] = TargetInstanceBelief(
        instance_id="building:7",
        encounter_order=7,
        target_world=[20.0, 0.0, -5.0],
        confidence=0.8,
    )
    memory.target_memories["building"] = target_memory
    key = "2|Fly to the first building in the current view|target"
    memory.stage_local_instances[key] = ["building:7"]
    memory.stage_locks[key] = "building:7"

    cleared = memory.reset_view_relative_binding(stage)

    assert cleared == ["building:7"]
    assert "building:7" in target_memory.instances
    assert key not in memory.stage_local_instances
    assert key not in memory.stage_locks
    assert target_memory.primary_instance_id == ""


def test_left_front_binding_rejects_wrong_side_and_behind():
    memory = _memory()
    stage = TaskStage(
        index=1,
        instruction="Fly to the first building ahead on the left in the current view",
        target="building",
        ordinal=1,
        view_relative=True,
    )

    assert memory._view_relative_observation_allowed(
        stage,
        {"view": "front", "forward_projection": 12.0, "lateral_projection": -4.0},
    )
    assert not memory._view_relative_observation_allowed(
        stage,
        {"view": "front", "forward_projection": -8.0, "lateral_projection": -4.0},
    )
    assert not memory._view_relative_observation_allowed(
        stage,
        {"view": "front", "forward_projection": 12.0, "lateral_projection": 4.0},
    )
    assert not memory._view_relative_observation_allowed(
        stage,
        {"view": "front", "forward_projection": 0.2, "lateral_projection": -4.0},
    )


def test_locked_view_relative_target_rejects_nearby_same_class_update_across_gap():
    memory = _memory()
    image = Image.new("RGB", (640, 480), (120, 120, 120))
    stage = TaskStage(
        index=0,
        instruction="Fly to the first building in the right front",
        target="building",
        relation="near",
        ordinal=1,
        view_relative=True,
    )
    memory.update_from_detections(
        stage=stage,
        detections_by_view={"front": [_detection(12.0, [360, 180, 500, 340])]},
        images_by_view={"front": image},
        observer_world=[0.0, 0.0, -5.0],
        observer_yaw_deg=0.0,
    )
    primary = memory.primary_instance(stage)
    assert primary is not None
    original_world = list(primary.target_world)
    original_count = primary.observation_count

    events = memory.update_from_detections(
        stage=stage,
        # 12m farther than the locked facade: inside the old 25m building
        # association radius, but outside the new surface-continuity gate.
        detections_by_view={"front": [_detection(24.0, [360, 180, 500, 340])]},
        images_by_view={"front": image},
        observer_world=[0.0, 0.0, -5.0],
        observer_yaw_deg=0.0,
    )

    target_memory = memory.target_memories["building"]
    assert events == []
    assert memory.local_instance_ids(stage) == [primary.instance_id]
    assert list(target_memory.instances) == [primary.instance_id]
    assert primary.target_world == original_world
    assert primary.observation_count == original_count


def test_full_height_building_facade_remains_valid_but_car_box_is_rejected():
    image = Image.new("RGB", (640, 480), (120, 120, 120))
    facade = _detection(12.0, [80, 0, 400, 480])
    building_stage = TaskStage(
        index=0,
        instruction="Fly to the first building in the left front from the new view",
        target="building",
        relation="near",
        view_relative=True,
    )
    car_stage = TaskStage(
        index=1,
        instruction="Fly to the white car",
        target="white car",
        relation="near",
    )

    assert _detection_reliability(building_stage, facade, image) > 0.0
    assert _detection_reliability(car_stage, facade, image) == 0.0

    pipeline_facade = _detection(12.0, [80, 0, 400, 480])
    CompletionPipeline._suppress_giant_bbox(building_stage, pipeline_facade, image)
    assert pipeline_facade.visible
    assert pipeline_facade.score > 0.0

    completion_facade = _detection(12.0, [80, 0, 400, 480])
    TaskCompletionChecker._zero_giant_bbox_score(building_stage, completion_facade, image)
    assert completion_facade.score > 0.0

    car_box = _detection(12.0, [80, 0, 400, 480])
    CompletionPipeline._suppress_giant_bbox(car_stage, car_box, image)
    assert not car_box.visible


def test_non_view_relative_target_still_creates_a_far_instance():
    memory = _memory()
    image = Image.new("RGB", (640, 480), (120, 120, 120))
    stage = TaskStage(
        index=0,
        instruction="Fly to a building",
        target="building",
        relation="near",
    )
    memory.update_from_detections(
        stage=stage,
        detections_by_view={"front": [_detection(12.0, [260, 180, 380, 340])]},
        images_by_view={"front": image},
        observer_world=[0.0, 0.0, -5.0],
        observer_yaw_deg=0.0,
    )
    memory.update_from_detections(
        stage=stage,
        detections_by_view={"front": [_detection(120.0, [260, 180, 380, 340])]},
        images_by_view={"front": image},
        observer_world=[0.0, 0.0, -5.0],
        observer_yaw_deg=0.0,
    )

    assert len(memory.target_memories["building"].instances) == 2


def test_outside_radius_does_not_override_locked_building_surface_memory():
    stage = TaskStage(
        index=0,
        instruction="Fly to the first building in the right front",
        target="building",
        relation="near",
        ordinal=1,
        view_relative=True,
    )
    instance = TargetInstanceBelief(
        instance_id="building:1",
        encounter_order=1,
        target_world=[10.0, 3.0, -5.0],
        confidence=0.61,
        observation_count=1,
        surface_points_world=[[8.0, 2.0, -5.0]],
        surface_observation_count=1,
        geometry_kind="large_surface",
        is_large_structure=True,
        uncertainty_m=2.0,
    )
    memory = SimpleNamespace(primary_instance=lambda _stage: instance)
    objects = SimpleNamespace(mission_memory=memory)
    outside = SimpleNamespace(
        target_detected=True,
        accepted_view="front",
        reason="outside_radius",
    )
    complete = SimpleNamespace(
        target_detected=True,
        accepted_view="front",
        reason="complete_near_target",
    )

    assert not _fresh_visual_support_matches_locked_memory(objects, stage, outside)
    assert _fresh_visual_support_matches_locked_memory(objects, stage, complete)

    stable_stage = TaskStage(
        index=1,
        instruction="Fly to a building",
        target="building",
        relation="near",
    )
    assert _fresh_visual_support_matches_locked_memory(objects, stable_stage, outside)

    actual_memory = _memory()
    target_memory = TargetMemory(
        target_key="building",
        target_name="building",
        primary_instance_id=instance.instance_id,
    )
    target_memory.instances[instance.instance_id] = instance
    actual_memory.target_memories["building"] = target_memory
    stage_key = "0|Fly to the first building in the right front|target"
    actual_memory.stage_local_instances[stage_key] = [instance.instance_id]
    actual_memory.stage_locks[stage_key] = instance.instance_id

    with_wrong_visual = actual_memory.evaluate_completion(
        stage=stage,
        current_world=[4.0, 2.0, -5.0],
        fresh_visual_support=True,
        visual_score=0.2,
        stop_radius_m=4.0,
    )
    from_locked_surface = actual_memory.evaluate_completion(
        stage=stage,
        current_world=[4.0, 2.0, -5.0],
        fresh_visual_support=False,
        stop_radius_m=4.0,
    )

    assert not with_wrong_visual.done
    assert with_wrong_visual.reason == "memory confidence below completion threshold"
    assert from_locked_surface.done
    assert from_locked_surface.reason == "memory_surface_near_complete"
