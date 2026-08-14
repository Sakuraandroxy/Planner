"""Focused tests for the lightweight MissionMemory behavior."""

from __future__ import annotations

from types import SimpleNamespace

from PIL import Image

from agent.functions.candidate.base import CandidateTrajectory
from agent.functions.candidate.scorer import score_candidates
from agent.functions.common.web_runtime_helpers import target_depth_text
from agent.functions.completion.navigation_metrics import NavigationMetricsTracker
from agent.functions.fast_slow.runtime import (
    _apply_memory_path_guard,
    _handle_background_target_lost,
    _memory_distance_trigger_radius,
    _sync_path_if_ready,
    _update_target_pose_from_bundle,
)
from agent.functions.memory import MissionMemory
from agent.functions.memory.appearance_signature import build_appearance_signature
from agent.functions.memory.geometry import distance_to_instance_geometry, nearest_instance_surface_point
from agent.functions.memory.mission_memory import stage_key
from agent.functions.memory.schemas import TargetInstanceBelief, TargetMemory
from agent.functions.memory.spatial_reasoning import relation_kind
from agent.functions.obstacle_avoidance import DepthObstacleAvoider
from agent.functions.obstacle_avoidance.schemas import ObstacleCell
from agent.functions.planning.direction_hint import direction_hint_from_locked_body_target
from agent.functions.task_parser.vlm_task_parser import parse_task_parser_to_stages


def _stage(**kwargs):
    data = {
        "index": 0,
        "instruction": "Fly to the red car",
        "mode": "target",
        "target": "red car",
        "relation": "near",
        "ordinal": None,
        "selection_rule": "",
        "completion_condition": "",
    }
    data.update(kwargs)
    return SimpleNamespace(**data)


def _det(bbox, depth, score=0.8, camera="front", label="red car"):
    return SimpleNamespace(
        visible=True,
        bbox=list(bbox),
        score=score,
        label=label,
        depth_median=depth,
        depth_bbox=None,
        camera=camera,
    )


def _memory():
    return MissionMemory(
        config={
            "ENABLED": True,
            "MIN_DETECTION_SCORE": 0.1,
            "MEMORY_ONLY_COMPLETION_ENABLED": True,
            "MEMORY_ONLY_MIN_CONFIDENCE": 0.80,
            "MEMORY_ONLY_MIN_OBSERVATIONS": 2,
            "MAX_COMPLETION_UNCERTAINTY_M": 5.0,
            "NEAR_MAX_ALTITUDE_M": 8.0,
            "ABOVE_HORIZONTAL_RADIUS_M": 2.0,
            "ABOVE_MIN_CLEARANCE_M": 0.1,
            "ABOVE_MAX_ALTITUDE_M": 60.0,
            "FRONT_CAMERA_OFFSET": [0.0, 0.0, 0.0],
        },
        sim_config={"FRONT_FOV": 90.0, "DOWN_FOV": 90.0},
    )


def test_ordinal_lock_survives_after_first_instance_leaves_view():
    img = Image.new("RGB", (640, 480), (120, 120, 120))
    memory = _memory()
    stage = _stage(
        instruction="Fly to the second red car",
        ordinal=2,
        selection_rule="ordinal",
    )

    memory.update_from_detections(
        stage=stage,
        detections_by_view={
            "front": [
                _det([290, 210, 330, 250], 10.0),
                _det([290, 210, 330, 250], 20.0),
                _det([290, 210, 330, 250], 30.0),
            ],
        },
        images_by_view={"front": img},
        observer_world=[0.0, 0.0, -5.0],
        observer_yaw_deg=0.0,
    )
    primary_before = memory.primary_instance(stage)

    memory.update_from_detections(
        stage=stage,
        detections_by_view={
            "front": [
                _det([290, 210, 330, 250], 10.0),
                _det([290, 210, 330, 250], 20.0),
            ],
        },
        images_by_view={"front": img},
        observer_world=[10.0, 0.0, -5.0],
        observer_yaw_deg=0.0,
    )
    primary_after = memory.primary_instance(stage)

    assert primary_before is not None
    assert primary_after is not None
    assert primary_before.instance_id == primary_after.instance_id
    assert primary_after.encounter_order == 2


def test_completed_instance_remains_available_for_later_return_stage():
    memory = _memory()
    target = TargetMemory(
        target_key="white car",
        target_name="white car",
        primary_instance_id="white car:1",
    )
    original = TargetInstanceBelief(
        instance_id="white car:1",
        encounter_order=1,
        target_world=[20.0, -3.0, -2.0],
        confidence=0.99,
        observation_count=3,
    )
    target.instances[original.instance_id] = original
    memory.target_memories["white car"] = target
    first_stage = _stage(
        index=0,
        instruction="Fly to the first white car",
        target="white car",
        relation="near",
        ordinal=1,
        selection_rule="ordinal",
    )
    return_stage = _stage(
        index=2,
        instruction="Fly back to the first white car passed",
        target="white car",
        relation="near",
        ordinal=1,
        selection_rule="ordinal",
    )

    memory.archive_stage(first_stage, "complete_near_target")
    recalled = memory.primary_instance(return_stage)

    assert original.status == "completed"
    assert recalled is original
    assert memory.stage_summaries[-1].primary_instance_id == "white car:1"


def test_near_completion_rejects_large_height_difference():
    memory = _memory()
    stage = _stage(relation="near")
    target = TargetMemory(target_key="red car", target_name="red car", primary_instance_id="red_car:1")
    target.instances["red_car:1"] = TargetInstanceBelief(
        instance_id="red_car:1",
        encounter_order=1,
        target_world=[0.0, 0.0, 0.0],
        confidence=0.95,
        observation_count=3,
        uncertainty_m=1.0,
    )
    memory.target_memories["red car"] = target
    memory.stage_locks["0|Fly to the red car|target"] = "red_car:1"

    decision = memory.evaluate_completion(stage=stage, current_world=[1.0, 1.0, -20.0], stop_radius_m=4.0)

    assert not decision.done
    assert decision.status in {"HOLD_CONFIRM", "NOT_COMPLETE"}
    assert "height" in decision.reason


def test_above_completion_uses_horizontal_geometry_and_uncertainty():
    memory = _memory()
    stage = _stage(instruction="Fly above the red car", relation="above")
    target = TargetMemory(target_key="red car", target_name="red car", primary_instance_id="red_car:1")
    target.instances["red_car:1"] = TargetInstanceBelief(
        instance_id="red_car:1",
        encounter_order=1,
        target_world=[0.0, 0.0, 0.0],
        confidence=0.96,
        observation_count=3,
        uncertainty_m=0.8,
        footprint_radius_m=1.5,
    )
    memory.target_memories["red car"] = target
    memory.stage_locks["0|Fly above the red car|target"] = "red_car:1"

    decision = memory.evaluate_completion(stage=stage, current_world=[0.8, 0.4, -3.0], stop_radius_m=4.0)

    assert decision.done
    assert decision.status == "COMPLETE"


def test_low_saturation_appearance_is_less_reliable_than_red_crop():
    gray = Image.new("RGB", (80, 80), (120, 120, 120))
    red = Image.new("RGB", (80, 80), (180, 40, 35))

    gray_sig = build_appearance_signature(gray, [5, 5, 75, 75])
    red_sig = build_appearance_signature(red, [5, 5, 75, 75])

    assert gray_sig is not None
    assert red_sig is not None
    assert red_sig.reliability > gray_sig.reliability


def test_candidate_memory_score_prefers_locked_primary():
    primary = CandidateTrajectory(waypoints=[[4.0, 0.0, 0.0], [9.5, 0.2, 0.0]], source="primary")
    wrong = CandidateTrajectory(waypoints=[[4.0, 2.0, 0.0], [9.5, 5.0, 0.0]], source="wrong")

    scored = score_candidates(
        [primary, wrong],
        memory_context={
            "enabled": True,
            "target_body": [10.0, 0.0, 0.0],
            "non_primary_bodies": [[10.0, 5.0, 0.0]],
            "relation": "near",
            "confidence": 0.95,
            "uncertainty_m": 1.0,
            "footprint_radius_m": 1.5,
        },
    )

    assert scored[0].pre_score > scored[1].pre_score
    assert scored[0].score_breakdown["memory"] > scored[1].score_breakdown["memory"]


def test_parser_extracts_bush_anchor_for_red_car_stage():
    stages = parse_task_parser_to_stages(
        """
        {"stages": [
          {"instruction": "Fly to the white car", "mode": "target", "target": "white car", "relation": "beside"},
          {"instruction": "Fly to the red car near the bushes", "mode": "target", "target": "red car near bushes", "relation": "beside"}
        ]}
        """,
        original_instruction="首先飞行到白车旁边，然后飞行到灌木丛旁边的红车旁边",
    )

    assert stages[0].target == "white car"
    assert stages[0].auxiliary_targets == []
    assert stages[1].target == "red car"
    assert "bushes" in stages[1].auxiliary_targets
    assert stages[1].selection_rule == "anchored"


def test_anchor_memory_selects_red_car_near_bushes():
    memory = _memory()
    stage = _stage(
        instruction="Fly to the red car near the bushes",
        target="red car",
        selection_rule="anchored",
    )
    stage.auxiliary_targets = ["bushes"]
    red = TargetMemory(target_key="red car", target_name="red car")
    red.instances["red_car:1"] = TargetInstanceBelief(
        instance_id="red_car:1",
        encounter_order=1,
        target_world=[0.0, 0.0, 0.0],
        confidence=0.95,
        observation_count=2,
    )
    red.instances["red_car:2"] = TargetInstanceBelief(
        instance_id="red_car:2",
        encounter_order=2,
        target_world=[20.0, 0.0, 0.0],
        confidence=0.90,
        observation_count=2,
    )
    bushes = TargetMemory(target_key="bushes", target_name="bushes", primary_instance_id="bushes:1")
    bushes.instances["bushes:1"] = TargetInstanceBelief(
        instance_id="bushes:1",
        encounter_order=1,
        target_world=[21.0, 0.5, 0.0],
        confidence=0.90,
        observation_count=2,
    )
    memory.target_memories["red car"] = red
    memory.target_memories["bushes"] = bushes

    primary = memory.primary_instance(stage)

    assert primary is not None
    assert primary.instance_id == "red_car:2"


def test_memory_path_guard_clips_qwen_overshoot():
    objects = SimpleNamespace(
        mission_memory=SimpleNamespace(config={"PATH_CLIP_ENABLED": True, "PATH_CLIP_RADIUS_M": 4.0}),
        completion_checker=SimpleNamespace(stop_depth=4.0),
    )
    original = [
        [16.3, -5.0, 0.0],
        [30.6, -9.4, 0.0],
        [43.0, -13.2, 0.0],
        [63.1, -19.3, 0.0],
    ]

    clipped, reason = _apply_memory_path_guard(
        objects,
        _stage(),
        original,
        {
            "enabled": True,
            "target_body": [21.2, -4.9, 0.8],
            "relation": "near",
            "footprint_radius_m": 1.5,
        },
    )

    assert reason
    assert len(clipped) < len(original)
    last = clipped[-1]
    horizontal = ((last[0] - 21.2) ** 2 + (last[1] + 4.9) ** 2) ** 0.5
    assert horizontal <= 4.05
    assert last[2] <= 0.2


def test_memory_path_guard_replaces_when_target_is_behind():
    objects = SimpleNamespace(
        mission_memory=SimpleNamespace(config={"PATH_CLIP_ENABLED": True, "PATH_CLIP_RADIUS_M": 4.0}),
        completion_checker=SimpleNamespace(stop_depth=4.0),
    )

    guarded, reason = _apply_memory_path_guard(
        objects,
        _stage(),
        [[5.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
        {
            "enabled": True,
            "target_body": [-20.0, 3.0, 0.0],
            "relation": "near",
            "footprint_radius_m": 1.5,
        },
    )

    assert "target_behind" in reason
    assert guarded and guarded[0][0] < 0.0


def test_return_to_previously_passed_target_is_near_not_pass_relation():
    stages = parse_task_parser_to_stages(
        """
        {"stages": [
          {"instruction": "Fly back to the first white car passed", "mode": "target", "target": "white car", "relation": "pass", "ordinal": 1}
        ]}
        """,
        original_instruction="飞回第一次经过的白车旁边",
    )

    assert len(stages) == 1
    assert stages[0].relation == "near"
    assert stages[0].ordinal == 1
    assert relation_kind(stages[0]) == "near"
    assert relation_kind(_stage(instruction="Pass the red car", relation="pass")) == "pass"


def test_locked_memory_direction_reports_target_behind():
    hint = direction_hint_from_locked_body_target([-18.9, -2.4, 1.0], confidence=0.99)

    assert "behind" in hint.text.lower()
    assert hint.reason == "locked memory anchor"
    assert abs(abs(hint.angle_deg) - 180.0) < 10.0
    assert hint.score == 0.99


def test_path_sync_turns_before_preserving_intentional_return_waypoint():
    class FakeMemory:
        config = {
            "RETURN_TARGET_REORIENT_ENABLED": True,
            "RETURN_TARGET_REORIENT_TRIGGER_DEG": 90.0,
            "RETURN_TARGET_REORIENT_TIMEOUT_S": 8.0,
        }

        @staticmethod
        def has_primary(stage):
            return True

        @staticmethod
        def preferred_yaw_deg(stage, current_world):
            return 180.0

        @staticmethod
        def record_pose(stage, position, yaw_deg):
            return None

        @staticmethod
        def summary(stage):
            return {"target": "white car:1"}

    class FakeController:
        def __init__(self):
            self.queue = SimpleNamespace(world_waypoints=[[-10.0, 0.0, -2.0]])
            self.planning = False
            self.has_plan_job = False

        @staticmethod
        def mark_executed(count):
            return None

    class FakeClient:
        def __init__(self):
            self.yaw = 0.0
            self.rotations = []

        def get_pose(self):
            return [0.0, 0.0, -2.0], self.yaw

        def rotate_to_yaw(self, yaw_deg, timeout=5.0):
            self.rotations.append((float(yaw_deg), float(timeout)))
            self.yaw = float(yaw_deg)

    class FakePathStream:
        active = False

        def __init__(self):
            self.synced = []

        @staticmethod
        def poll(queue_waypoints, current_pos):
            return SimpleNamespace(consumed=0, collided=False)

        def stop(self):
            self.active = False

        def sync(self, waypoints, current_pos, velocity):
            self.synced = [list(point) for point in waypoints]
            self.active = True
            return True

    class FakeState:
        def __init__(self):
            self.updates = []

        def update(self, **kwargs):
            self.updates.append(kwargs)

    client = FakeClient()
    path_stream = FakePathStream()
    controller = FakeController()
    objects = SimpleNamespace(
        mission_memory=FakeMemory(),
        controller=controller,
    )

    consumed = _sync_path_if_ready(
        objects,
        path_stream,
        client,
        FakeState(),
        stage=_stage(
            instruction="Fly back to the first white car passed",
            target="white car",
            relation="near",
            ordinal=1,
        ),
    )

    assert consumed == 0
    assert client.rotations == [(180.0, 8.0)]
    assert controller.queue.world_waypoints == [[-10.0, 0.0, -2.0]]
    assert path_stream.synced == [[-10.0, 0.0, -2.0]]


def test_memory_path_guard_uses_inner_approach_not_outer_circle():
    objects = SimpleNamespace(
        mission_memory=SimpleNamespace(
            config={
                "PATH_CLIP_ENABLED": True,
                "PATH_CLIP_RADIUS_M": 6.0,
                "NEAR_STANDOFF_M": 6.0,
                "LOW_ALTITUDE_EXTRA_STANDOFF_M": 1.0,
                "NEAR_APPROACH_RADIUS_M": 4.5,
                "NEAR_APPROACH_TARGET_CLEARANCE_M": 2.2,
                "NEAR_APPROACH_UNCERTAINTY_CAP_M": 1.5,
            }
        ),
        completion_checker=SimpleNamespace(stop_depth=4.0),
    )

    guarded, reason = _apply_memory_path_guard(
        objects,
        _stage(),
        [[16.8, -4.0, 0.0], [31.5, -7.5, 0.0]],
        {
            "enabled": True,
            "target_body": [21.2, -4.9, 0.8],
            "relation": "near",
            "footprint_radius_m": 1.5,
            "uncertainty_m": 2.3,
        },
    )

    assert "approach" in reason
    last = guarded[-1]
    horizontal = ((last[0] - 21.2) ** 2 + (last[1] + 4.9) ** 2) ** 0.5
    assert 4.4 <= horizontal <= 4.6


def test_memory_path_guard_allows_refinement_inside_above_radius():
    objects = SimpleNamespace(
        mission_memory=SimpleNamespace(config={"PATH_CLIP_ENABLED": True, "PATH_CLIP_RADIUS_M": 6.0}),
        completion_checker=SimpleNamespace(stop_depth=4.0),
    )
    original = [[3.7, -1.3, -0.5], [7.5, -2.6, -1.0]]

    guarded, reason = _apply_memory_path_guard(
        objects,
        _stage(instruction="Fly over the building", target="building", relation="above"),
        original,
        {
            "enabled": True,
            "target_body": [4.8, -2.6, 0.2],
            "relation": "above",
            "footprint_radius_m": 4.3,
            "uncertainty_m": 0.6,
        },
    )

    assert reason == ""
    assert guarded == original


def test_parser_does_not_treat_direction_words_as_auxiliary_targets():
    stages = parse_task_parser_to_stages(
        """
        {"stages": [
          {"instruction": "Fly to the first white car on the left front", "mode": "target", "target": "white car", "relation": "beside", "ordinal": 1, "auxiliary_targets": ["left front"]},
          {"instruction": "Fly to the first red car on the right front", "mode": "target", "target": "red car", "relation": "beside", "ordinal": 1, "auxiliary_targets": ["right front"]}
        ]}
        """,
        original_instruction="首先飞到左前方第一辆白车旁边，然后再飞到右前方第一辆红车旁边",
    )

    assert stages[0].target == "white car"
    assert stages[0].auxiliary_targets == []
    assert stages[1].target == "red car"
    assert stages[1].auxiliary_targets == []


def test_depth_obstacle_avoider_stops_before_front_obstacle():
    import numpy as np

    depth = np.full((32, 32), 50.0, dtype=np.float32)
    depth[15:17, 15:17] = 5.0
    avoider = DepthObstacleAvoider(
        config={
            "ENABLED": True,
            "FRONT_DEPTH_STRIDE": 1,
            "FRONT_MIN_DEPTH_M": 0.5,
            "FRONT_MAX_DEPTH_M": 10.0,
            "LOOKAHEAD_M": 10.0,
            "SIDE_RANGE_M": 4.0,
            "VERTICAL_RANGE_M": 3.0,
            "SAFETY_RADIUS_M": 0.8,
            "VERTICAL_CLEARANCE_M": 0.8,
            "STOP_BUFFER_M": 2.0,
            "BYPASS_ENABLED": False,
            "TARGET_KEEP_OUT_ENABLED": False,
            "FRONT_CAMERA_OFFSET": [0.0, 0.0, 0.0],
        },
        sim_config={"FRONT_FOV": 90.0},
    )

    updates = avoider.update_from_depth(
        front_depth_meters=depth,
        observer_world=[0.0, 0.0, 0.0],
        observer_yaw_deg=0.0,
    )
    result = avoider.filter_cumulative_waypoints(
        [[8.0, 0.0, 0.0]],
        current_world=[0.0, 0.0, 0.0],
        yaw_deg=0.0,
    )

    assert updates > 0
    assert result.changed
    assert result.waypoints
    assert result.waypoints[-1][0] < 5.0


def test_large_building_completes_from_surface_memory_after_detection_is_lost():
    memory = _memory()
    stage = _stage(
        instruction="Fly near the high-rise building",
        target="high-rise building",
        relation="near",
    )
    target = TargetMemory(
        target_key="high rise building",
        target_name="high-rise building",
        primary_instance_id="high-rise building:1",
    )
    target.instances["high-rise building:1"] = TargetInstanceBelief(
        instance_id="high-rise building:1",
        encounter_order=1,
        # Deliberately place the identity anchor deep inside the building.
        target_world=[30.0, 0.0, 5.0],
        confidence=0.72,
        observation_count=1,
        uncertainty_m=1.2,
        footprint_radius_m=8.0,
        surface_points_world=[
            [0.0, -15.0, -20.0],
            [0.0, 15.0, -20.0],
            [0.0, -15.0, 30.0],
            [0.0, 15.0, 30.0],
        ],
        surface_bounds_world=[[0.0, -15.0, -20.0], [0.0, 15.0, 30.0]],
        surface_observation_count=1,
        geometry_kind="large_surface",
        is_large_structure=True,
    )
    memory.target_memories["high rise building"] = target
    memory.stage_locks[stage_key(stage)] = "high-rise building:1"

    estimate = memory.estimate_distance(stage, current_world=[-4.5, 6.0, 0.0])
    decision = memory.evaluate_completion(
        stage=stage,
        current_world=[-4.5, 6.0, 0.0],
        fresh_visual_support=False,
        stop_radius_m=4.0,
    )

    assert estimate is not None
    assert estimate["distance_kind"] == "surface"
    assert abs(estimate["distance_m"] - 10.0623058987) < 1e-6
    assert estimate["nearest_surface_world"] == [0.0, 6.0, 0.0]
    # Completion uses the finite retained samples, not a filled AABB.  The
    # query is therefore moved onto a retained facade sample for the direct
    # large-structure rule below.
    decision = memory.evaluate_completion(
        stage=stage,
        current_world=[4.0, -15.0, 100.0],
        fresh_visual_support=False,
        stop_radius_m=4.0,
    )
    assert decision.done
    assert decision.reason == "large_surface_xy_radius_complete"


def test_large_surface_arrival_uses_xy_only_and_fixed_radius():
    memory = _memory()
    stage = _stage(target="warehouse", instruction="Fly near the warehouse")
    target = TargetMemory(
        target_key="warehouse",
        target_name="warehouse",
        primary_instance_id="warehouse:1",
    )
    target.instances["warehouse:1"] = TargetInstanceBelief(
        instance_id="warehouse:1",
        encounter_order=1,
        target_world=[20.0, 0.0, 0.0],
        confidence=0.40,
        observation_count=1,
        uncertainty_m=7.5,
        surface_points_world=[[0.0, 0.0, -20.0]],
        surface_observation_count=1,
        geometry_kind="large_surface",
        is_large_structure=True,
    )
    memory.target_memories["warehouse"] = target
    memory.stage_locks[stage_key(stage)] = "warehouse:1"

    decision = memory.evaluate_large_surface_arrival(
        stage,
        current_world=[4.5, 0.0, 100.0],
        radius_m=4.5,
    )
    assert decision is not None
    assert decision.done
    assert decision.distance_m == 4.5
    assert decision.vertical_delta_m == 120.0
    assert decision.reason == "large_surface_xy_radius_complete"

    assert memory.evaluate_large_surface_arrival(
        stage,
        current_world=[4.51, 0.0, 100.0],
        radius_m=4.5,
    ) is None


def test_small_surface_target_does_not_use_large_structure_shortcut():
    memory = _memory()
    stage = _stage(target="red car", instruction="Fly near the red car")
    target = TargetMemory(
        target_key="red car",
        target_name="red car",
        primary_instance_id="red car:1",
    )
    target.instances["red car:1"] = TargetInstanceBelief(
        instance_id="red car:1",
        encounter_order=1,
        target_world=[0.0, 0.0, 0.0],
        confidence=0.99,
        observation_count=3,
        surface_points_world=[[0.0, 0.0, 0.0]],
        surface_observation_count=1,
        geometry_kind="surface_bounds",
        is_large_structure=False,
    )
    memory.target_memories["red car"] = target
    memory.stage_locks[stage_key(stage)] = "red car:1"

    assert memory.evaluate_large_surface_arrival(
        stage,
        current_world=[0.0, 0.0, 100.0],
        radius_m=4.5,
    ) is None


def test_car_and_fountain_semantics_override_large_footprint_heuristic():
    image = Image.new("RGB", (640, 480), (120, 120, 120))
    for target_name in ("red car", "fountain"):
        memory = _memory()
        memory.config["LARGE_STRUCTURE_MIN_FOOTPRINT_M"] = 0.1
        stage = _stage(
            target=target_name,
            instruction=f"Fly near the {target_name}",
        )
        memory.update_from_detections(
            stage=stage,
            detections_by_view={
                "front": [_det([80, 80, 560, 440], 40.0, label=target_name)],
            },
            images_by_view={"front": image},
            observer_world=[0.0, 0.0, 0.0],
            observer_yaw_deg=0.0,
        )

        instance = memory.primary_instance(stage)
        assert instance is not None
        assert not instance.is_large_structure


def test_sparse_depth_samples_build_a_bounded_surface_instead_of_one_point():
    memory = _memory()
    stage = _stage(target="warehouse", instruction="Fly near the warehouse")
    image = Image.new("RGB", (640, 480), (120, 120, 120))
    detection = _det([160, 100, 480, 380], 20.0, score=0.85, label="warehouse")
    detection.surface_depth_samples = [
        [0.30, 0.30, 20.0],
        [0.70, 0.30, 20.0],
        [0.30, 0.70, 20.0],
        [0.70, 0.70, 20.0],
    ]

    memory.update_from_detections(
        stage=stage,
        detections_by_view={"front": [detection]},
        images_by_view={"front": image},
        observer_world=[0.0, 0.0, 0.0],
        observer_yaw_deg=0.0,
    )

    instance = memory.primary_instance(stage)
    assert instance is not None
    assert instance.geometry_kind == "large_surface"
    assert instance.surface_observation_count == 1
    assert len(instance.surface_points_world) == 4
    assert len(instance.surface_patches_world) == 1
    assert instance.surface_bounds_world is not None
    assert instance.surface_bounds_world[1][1] > instance.surface_bounds_world[0][1]
    assert instance.surface_bounds_world[1][2] > instance.surface_bounds_world[0][2]


def test_independent_surface_patches_do_not_fill_empty_aabb_space():
    left_patch = [
        [0.0, -4.0, -5.0],
        [0.0, 4.0, -5.0],
        [0.0, -4.0, 5.0],
        [0.0, 4.0, 5.0],
    ]
    right_patch = [
        [20.0, -4.0, -5.0],
        [20.0, 4.0, -5.0],
        [20.0, -4.0, 5.0],
        [20.0, 4.0, 5.0],
    ]
    instance = TargetInstanceBelief(
        instance_id="building:1",
        encounter_order=1,
        target_world=[0.0, 0.0, 0.0],
        confidence=0.8,
        surface_points_world=left_patch + right_patch,
        surface_patches_world=[left_patch, right_patch],
        surface_bounds_world=[[0.0, -4.0, -5.0], [20.0, 4.0, 5.0]],
        surface_observation_count=2,
        geometry_kind="large_surface",
        is_large_structure=True,
    )

    query = [10.0, 0.0, 0.0]
    nearest = nearest_instance_surface_point(query, instance)
    assert abs(distance_to_instance_geometry(query, instance) - 10.0) < 1e-6
    assert nearest[0] in {0.0, 20.0}


def test_navigation_metrics_replace_point_anchor_with_surface_geometry():
    metrics = NavigationMetricsTracker(success_radius_m=4.0)
    stage_key = (0, "Fly near the building", "target")
    metrics.record_pose([0.0, 0.0, 0.0])
    metrics.update_target(stage_key, [100.0, 0.0, 0.0], confidence=0.8)
    assert metrics.straight_line_distance_m == 100.0

    metrics.record_pose([10.0, 0.0, 0.0])
    metrics.update_target(
        stage_key,
        [20.0, 0.0, 0.0],
        confidence=0.8,
        replace_stage_reference=True,
    )

    assert metrics.straight_line_distance_m == 20.0
    assert metrics.ne_m == 10.0


def test_depth_attachment_extracts_bounded_normalized_surface_samples():
    import numpy as np

    image = Image.new("RGB", (640, 480), (120, 120, 120))
    depth = np.full((48, 64), 20.0, dtype=np.float32)
    detection = _det([160, 120, 480, 360], 20.0, score=0.8, label="building")

    target_depth_text("front", detection, image, depth)

    assert detection.depth_median == 20.0
    assert 4 <= len(detection.surface_depth_samples) <= 36
    assert all(0.0 <= sample[0] <= 1.0 for sample in detection.surface_depth_samples)
    assert all(0.0 <= sample[1] <= 1.0 for sample in detection.surface_depth_samples)
    assert all(sample[2] == 20.0 for sample in detection.surface_depth_samples)


def test_surface_trigger_does_not_add_large_building_footprint_twice():
    instance = TargetInstanceBelief(
        instance_id="building:1",
        encounter_order=1,
        target_world=[20.0, 0.0, 0.0],
        confidence=0.8,
        footprint_radius_m=8.0,
        surface_points_world=[[10.0, -5.0, -5.0], [10.0, 5.0, 5.0]],
        surface_bounds_world=[[10.0, -5.0, -5.0], [10.0, 5.0, 5.0]],
        surface_observation_count=1,
        geometry_kind="large_surface",
        is_large_structure=True,
    )
    memory = SimpleNamespace(
        config={"SURFACE_APPROACH_RADIUS_M": 4.5, "NEAR_APPROACH_RADIUS_M": 4.5},
        primary_instance=lambda stage: instance,
    )
    objects = SimpleNamespace(mission_memory=memory)

    assert _memory_distance_trigger_radius(objects, _stage(target="building"), 4.0) == 4.5


def test_background_detector_loss_keeps_flying_with_fresh_building_surface_memory():
    memory = _memory()
    stage = _stage(target="building", instruction="Fly near the building")
    target = TargetMemory(target_key="building", target_name="building", primary_instance_id="building:1")
    target.instances["building:1"] = TargetInstanceBelief(
        instance_id="building:1",
        encounter_order=1,
        target_world=[20.0, 0.0, 0.0],
        confidence=0.72,
        observation_count=1,
        uncertainty_m=1.0,
        surface_points_world=[[10.0, -5.0, -5.0], [10.0, 5.0, 5.0]],
        surface_bounds_world=[[10.0, -5.0, -5.0], [10.0, 5.0, 5.0]],
        surface_observation_count=1,
        geometry_kind="large_surface",
        is_large_structure=True,
    )
    memory.target_memories["building"] = target
    objects = SimpleNamespace(mission_memory=memory)
    client = SimpleNamespace(get_pose=lambda: ([0.0, 0.0, 0.0], 0.0))
    state = SimpleNamespace(update=lambda **kwargs: None)

    should_stop_stage = _handle_background_target_lost(
        objects,
        client,
        path_stream=None,
        state=state,
        stage=stage,
        capture_mode="batch",
        reason="target fills view and detector returned no bbox",
    )

    assert not should_stop_stage


def _locked_low_confidence_building_memory(stage):
    memory = _memory()
    target = TargetMemory(target_key="building", target_name="building", primary_instance_id="building:1")
    target.instances["building:1"] = TargetInstanceBelief(
        instance_id="building:1",
        encounter_order=1,
        target_world=[20.0, 0.0, 0.0],
        confidence=0.38,
        observation_count=3,
        uncertainty_m=1.8,
        surface_points_world=[[10.0, -5.0, -5.0], [10.0, 5.0, 5.0]],
        surface_bounds_world=[[10.0, -5.0, -5.0], [10.0, 5.0, 5.0]],
        surface_observation_count=3,
        geometry_kind="large_surface",
        is_large_structure=True,
    )
    memory.target_memories["building"] = target
    memory.stage_locks[stage_key(stage)] = "building:1"
    return memory


def test_near_locked_building_ignores_detector_loss_without_changing_yaw():
    stage = _stage(
        target="building",
        instruction="Fly to the first building in the right front",
        ordinal=1,
        selection_rule="view_relative",
    )
    memory = _locked_low_confidence_building_memory(stage)
    rotations = []
    client = SimpleNamespace(
        get_pose=lambda: ([0.0, 0.0, 0.0], 47.0),
        rotate_to_yaw=lambda yaw: rotations.append(yaw),
        rotate_yaw=lambda yaw: rotations.append(yaw),
    )
    objects = SimpleNamespace(mission_memory=memory)
    state = SimpleNamespace(update=lambda **kwargs: None)

    should_stop_stage = _handle_background_target_lost(
        objects,
        client,
        path_stream=None,
        state=state,
        stage=stage,
        capture_mode="batch",
        reason="partial facade was not detected as a building",
    )

    assert not should_stop_stage
    assert rotations == []
    trusted = memory.trusted_near_large_surface_estimate(stage, [0.0, 0.0, 0.0])
    assert trusted is not None
    assert abs(trusted["distance_m"] - 11.1803398875) < 1e-6


def test_low_confidence_locked_building_completes_from_near_surface_memory():
    stage = _stage(
        target="building",
        instruction="Fly to the first building in the right front",
        ordinal=1,
        selection_rule="view_relative",
    )
    memory = _locked_low_confidence_building_memory(stage)

    decision = memory.evaluate_completion(
        stage=stage,
        current_world=[4.5, 0.0, 0.0],
        fresh_visual_support=False,
        stop_radius_m=5.0,
    )

    assert not decision.done
    assert decision.reason == "outside_large_surface_xy_radius"
    assert decision.required_radius_m == 4.5
    assert memory.trusted_near_large_surface_estimate(stage, [-3.0, 0.0, 0.0]) is None


def test_near_locked_building_rejects_far_detector_point_update():
    stage = _stage(
        target="building",
        instruction="Fly to the first building in the right front",
        ordinal=1,
        selection_rule="view_relative",
    )
    memory = _locked_low_confidence_building_memory(stage)
    image = Image.new("RGB", (640, 480), (120, 120, 120))
    far_detection = _det([280, 180, 360, 300], 50.0, score=0.8, label="building")
    estimator_calls = []
    metric_updates = []
    distance_estimator = SimpleNamespace(
        enabled=True,
        clear=lambda: estimator_calls.append("clear"),
        update_from_detection=lambda **kwargs: estimator_calls.append("update"),
    )
    navigation_metrics = SimpleNamespace(
        update_target=lambda *args, **kwargs: metric_updates.append((args, kwargs)),
        record_distance=lambda distance: metric_updates.append(("distance", distance)),
        invalidate_target=lambda *args, **kwargs: None,
    )
    objects = SimpleNamespace(
        mission_memory=memory,
        distance_estimator=distance_estimator,
        navigation_metrics=navigation_metrics,
    )
    bundle = SimpleNamespace(
        observer_world=[0.0, 0.0, 0.0],
        observer_yaw_deg=0.0,
        front_detection=far_detection,
        down_detection=SimpleNamespace(visible=False, camera="down"),
        front_detections=[far_detection],
        down_detections=[],
        front_image=image,
        down_image=None,
        distance_view="front",
    )

    estimate = _update_target_pose_from_bundle(objects, stage, bundle)

    assert estimate is not None
    assert estimate.source == "mission_memory"
    assert "clear" in estimator_calls
    assert "update" not in estimator_calls
    assert metric_updates


def _weak_large_surface_memory(stage):
    memory = _memory()
    target = TargetMemory(target_key="building", target_name="building", primary_instance_id="building:1")
    target.instances["building:1"] = TargetInstanceBelief(
        instance_id="building:1",
        encounter_order=1,
        target_world=[95.0, 20.0, -8.0],
        confidence=0.31,
        observation_count=1,
        uncertainty_m=7.9,
        surface_points_world=[[10.0, -5.0, -5.0], [10.0, 5.0, 5.0]],
        surface_bounds_world=[[10.0, -5.0, -5.0], [10.0, 5.0, 5.0]],
        surface_observation_count=1,
        geometry_kind="large_surface",
        is_large_structure=True,
    )
    memory.target_memories["building"] = target
    memory.stage_local_instances[stage_key(stage)] = ["building:1"]
    return memory, target


def test_unique_weak_building_surface_gets_stage_lock():
    stage = _stage(
        target="building",
        instruction="Fly to the first building in the right front",
        ordinal=1,
        selection_rule="view_relative",
    )
    memory, target = _weak_large_surface_memory(stage)

    memory._ensure_stage_lock(stage, target, [0.0, 0.0, 0.0], 0.0)

    assert memory.is_primary_locked(stage)


def test_ordinal_weak_building_surface_gets_lock_with_other_local_candidates():
    stage = _stage(
        target="building",
        instruction="Fly to the first building in the left front",
        ordinal=1,
        selection_rule="view_relative",
        view_relative=True,
    )
    memory = _memory()
    memory.config.update(
        {
            "LOCK_MIN_CONFIDENCE": 0.35,
            "LARGE_STRUCTURE_SURFACE_LOCK_MIN_CONFIDENCE": 0.15,
        }
    )
    target = TargetMemory(target_key="building", target_name="building")
    for index, confidence in ((1, 0.32), (2, 0.34), (3, 0.41)):
        instance = TargetInstanceBelief(
            instance_id=f"building:{index}",
            encounter_order=index,
            target_world=[10.0 + index, float(index), -20.0],
            confidence=confidence,
            observation_count=1,
            uncertainty_m=2.0,
            surface_points_world=[[10.0 + index, float(index), -20.0]],
            surface_observation_count=1,
            geometry_kind="large_surface",
            is_large_structure=True,
        )
        target.instances[instance.instance_id] = instance
    memory.target_memories["building"] = target
    memory.stage_local_instances[stage_key(stage)] = [
        "building:1",
        "building:2",
        "building:3",
    ]

    memory._ensure_stage_lock(stage, target, [0.0, 0.0, -20.0], 0.0)

    assert memory.stage_locks[stage_key(stage)] == "building:1"
    assert memory.is_primary_locked(stage)
    decision = memory.evaluate_large_surface_arrival(
        stage,
        [10.0 + 1.0 - 3.14, 1.0, 100.0],
        radius_m=4.5,
    )
    assert decision is not None
    assert decision.done
    assert decision.reason == "large_surface_xy_radius_complete"


def test_active_path_keeps_weak_locked_building_memory_without_relocalizing():
    stage = _stage(
        target="building",
        instruction="Fly to the first building in the right front",
        ordinal=1,
        selection_rule="view_relative",
    )
    memory, target = _weak_large_surface_memory(stage)
    memory._ensure_stage_lock(stage, target, [0.0, 0.0, 0.0], 0.0)
    rotations = []
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=SimpleNamespace(
            queue=SimpleNamespace(world_waypoints=[[20.0, 0.0, 0.0]]),
            planning=False,
        ),
    )
    client = SimpleNamespace(
        get_pose=lambda: ([0.0, 0.0, 0.0], 49.0),
        rotate_to_yaw=lambda yaw: rotations.append(yaw),
        rotate_yaw=lambda yaw: rotations.append(yaw),
    )
    state = SimpleNamespace(update=lambda **kwargs: None)

    should_stop_stage = _handle_background_target_lost(
        objects,
        client,
        path_stream=SimpleNamespace(active=True),
        state=state,
        stage=stage,
        capture_mode="batch",
        reason="intermittent building detector miss",
    )

    assert not should_stop_stage
    assert rotations == []


def test_large_facade_matches_near_depth_barrier_despite_lateral_point_error():
    avoider = DepthObstacleAvoider(
        config={
            "LARGE_STRUCTURE_TARGET_MATCH_RADIUS_M": 6.0,
            "LARGE_STRUCTURE_TARGET_MATCH_RANGE_TOLERANCE_M": 3.0,
            "LARGE_STRUCTURE_TARGET_MATCH_VERTICAL_M": 3.0,
        }
    )
    context = {
        "target_body": [7.5, 3.0, 0.8],
        "is_large_structure": True,
        "uses_surface_geometry": True,
    }

    assert avoider._obstacle_matches_locked_target([5.79, -0.67, -0.56], context)
    assert not avoider._obstacle_matches_locked_target([2.0, -8.0, -0.56], context)
    avoider.obstacle_cells["wall"] = ObstacleCell(
        key="wall",
        center_world=[5.79, -0.67, -0.56],
        confidence=0.9,
    )
    result = avoider.filter_cumulative_waypoints(
        [[4.97, -0.80, 0.01]],
        current_world=[0.0, 0.0, 0.0],
        yaw_deg=0.0,
        memory_context={"enabled": True, "relation": "near", "footprint_radius_m": 0.5, **context},
    )
    assert result.reason == "stop_before_target_depth_obstacle"
    assert result.waypoints == [[2.798, -0.45, 0.006]]


def test_near_depth_contact_corrects_weak_facade_and_enables_completion():
    stage = _stage(
        target="building",
        instruction="Fly to the first building in the right front",
        ordinal=1,
        selection_rule="view_relative",
    )
    memory, target = _weak_large_surface_memory(stage)
    memory._ensure_stage_lock(stage, target, [0.0, 0.0, 0.0], 0.0)

    contact = memory.record_locked_large_surface_contact(
        stage,
        observer_world=[0.0, 0.0, 0.0],
        observer_yaw_deg=0.0,
        obstacle_body=[5.79, -0.67, -0.56],
    )
    decision = memory.evaluate_completion(
        stage=stage,
        current_world=[0.0, 0.0, 0.0],
        fresh_visual_support=False,
        stop_radius_m=5.0,
    )

    instance = memory.primary_instance(stage)
    assert contact is not None
    assert abs(contact["distance_m"] - 5.8286362041) < 0.01
    assert instance.confidence >= 0.38
    assert instance.uncertainty_m <= 1.5
    assert not decision.done
    assert decision.reason == "outside_large_surface_xy_radius"
    assert decision.required_radius_m == 4.5
