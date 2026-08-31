"""Regression tests for semantic yaw transitions and encounter binding."""

from __future__ import annotations

import math
from types import SimpleNamespace

from PIL import Image

from agent.functions.common.task_manager import TaskManager
from agent.functions.fast_slow.runtime import (
    _camera_frame_view_yaw_deg,
    _execute_action_stage,
    _record_stage_activation_heading,
    _record_stage_semantic_completion,
    _reorient_to_locked_target_if_behind,
)
from agent.functions.memory import MissionMemory
from agent.functions.task_parser.base import TaskStage
from agent.functions.task_parser.vlm_task_parser import parse_task_parser_to_stages


class _YawClient:
    def __init__(self, yaw_deg: float, *, camera_yaw_offset_deg: float = 0.0):
        self.position = [0.0, 0.0, -8.0]
        self.yaw_deg = float(yaw_deg)
        self.commanded_yaws = []
        self.primary_camera_id = "front"
        self._camera_yaw_offset_deg = float(camera_yaw_offset_deg)

    def get_pose(self):
        return list(self.position), float(self.yaw_deg)

    def rotate_to_yaw(self, yaw_deg):
        self.commanded_yaws.append(float(yaw_deg) % 360.0)
        self.yaw_deg = float(yaw_deg) % 360.0

    def latest_capture_snapshot(self):
        view_yaw = math.radians(20.0 + self._camera_yaw_offset_deg)
        frame = SimpleNamespace(
            navigation_yaw_deg=20.0,
            rotation_camera_to_world=[
                [math.cos(view_yaw), -math.sin(view_yaw), 0.0],
                [math.sin(view_yaw), math.cos(view_yaw), 0.0],
                [0.0, 0.0, 1.0],
            ],
        )
        return SimpleNamespace(
            frame=lambda camera_id: frame if camera_id == "front" else None,
            frames={"front": frame},
        )


def _completed_target_then_turn(*, reference: str = "previous_stage_semantic"):
    target = TaskStage(
        index=0,
        instruction="Fly above the white car",
        mode="target",
        target="white car",
        relation="above",
    )
    turn = TaskStage(
        index=1,
        instruction="Turn the view right by 15 degrees",
        mode="action",
        action="right",
        value=15.0,
        heading_reference=reference,
    )
    next_target = TaskStage(
        index=2,
        instruction="Fly above the first manhole cover encountered",
        mode="target",
        target="manhole cover",
        relation="above",
        ordinal=1,
        selection_rule="encounter",
        view_relative=True,
    )
    manager = TaskManager()
    manager.start_with_stages("mission", [target, turn, next_target])
    manager.update_stage_context(
        0,
        semantic_view_yaw_deg=20.0,
        semantic_heading_source="activation_to_locked_target",
    )
    manager.complete_current("white car reached")
    return manager, turn, next_target


def _memory() -> MissionMemory:
    return MissionMemory(
        config={
            "ENABLED": True,
            "MIN_DETECTION_SCORE": 0.1,
            "LOCK_MIN_CONFIDENCE": 0.1,
            "VIEW_RELATIVE_MIN_FORWARD_M": 0.5,
            "ENCOUNTER_CORRIDOR_MIN_HALF_WIDTH_M": 2.0,
            "ENCOUNTER_CORRIDOR_MAX_HALF_WIDTH_M": 12.0,
            "ENCOUNTER_CORRIDOR_HALF_ANGLE_DEG": 35.0,
            "FRONT_CAMERA_OFFSET": [0.0, 0.0, 0.0],
        },
        sim_config={"FRONT_FOV": 90.0, "DOWN_FOV": 90.0},
    )


def _detection(depth_m: float, score: float):
    return SimpleNamespace(
        visible=True,
        bbox=[300, 210, 340, 270],
        score=float(score),
        label="manhole cover",
        depth_median=float(depth_m),
        depth_bbox=None,
        surface_depth_samples=None,
        camera="front",
    )


def test_parser_marks_semantic_turn_and_post_turn_encounter_target():
    stages = parse_task_parser_to_stages(
        """
        {
          "task_type": "multi",
          "stages": [
            {
              "instruction": "Fly above the white car",
              "mode": "target",
              "target": "white car",
              "relation": "above"
            },
            {
              "instruction": "Turn the view right by 15 degrees",
              "mode": "action",
              "action": "right",
              "value": 15
            },
            {
              "instruction": "Fly above the first manhole cover encountered",
              "mode": "target",
              "target": "manhole cover",
              "relation": "above"
            }
          ]
        }
        """
    )

    assert stages[1].heading_reference == "previous_stage_semantic"
    assert stages[2].view_relative is True
    assert stages[2].selection_rule == "encounter"
    assert stages[2].ordinal == 1


def test_turn_uses_previous_semantic_heading_after_overshoot_return():
    manager, turn, _next_target = _completed_target_then_turn()
    client = _YawClient(200.0)

    context = _execute_action_stage(client, turn, task_manager=manager)

    assert client.commanded_yaws == [35.0]
    assert context["semantic_view_yaw_deg"] == 35.0
    assert context["semantic_heading_source"] == "previous_stage_semantic"


def test_navigation_completion_freezes_activation_to_target_heading():
    target = TaskStage(
        index=0,
        instruction="Fly above the white car",
        mode="target",
        target="white car",
        relation="above",
    )
    manager = TaskManager()
    manager.start_with_stages("mission", [target])
    manager.activate_current(
        activation_position=[0.0, 0.0, -8.0],
        activation_view_yaw_deg=0.0,
        last_actual_body_yaw_deg=200.0,
    )
    instance = SimpleNamespace(
        instance_id="white car:1",
        identity_world=[10.0, 10.0, -8.0],
        target_world=[10.0, 10.0, -8.0],
        confidence=0.93,
    )
    objects = SimpleNamespace(
        task_manager=manager,
        mission_memory=SimpleNamespace(primary_instance=lambda _stage: instance),
    )

    context = _record_stage_semantic_completion(objects, target)

    assert context["semantic_view_yaw_deg"] == 45.0
    assert context["semantic_heading_source"] == "activation_to_locked_target"


def test_explicit_actual_current_turn_uses_physical_exit_heading():
    manager, turn, _next_target = _completed_target_then_turn(reference="actual_current")
    client = _YawClient(200.0)

    context = _execute_action_stage(client, turn, task_manager=manager)

    assert client.commanded_yaws == [215.0]
    assert context["semantic_view_yaw_deg"] == 215.0
    assert context["semantic_heading_source"] == "actual_current"


def test_camera_yaw_extrinsic_converts_view_target_to_body_command():
    manager, turn, next_target = _completed_target_then_turn()
    client = _YawClient(200.0, camera_yaw_offset_deg=30.0)

    context = _execute_action_stage(client, turn, task_manager=manager)

    assert client.commanded_yaws == [5.0]
    assert context["semantic_view_yaw_deg"] == 35.0
    manager.update_stage_context(turn.index, **context)
    manager.complete_current("action executed")
    activation = _record_stage_activation_heading(
        SimpleNamespace(task_manager=manager),
        client,
        next_target,
        client.position,
        client.yaw_deg,
    )
    assert activation["activation_body_yaw_deg"] == 5.0
    assert activation["activation_view_yaw_deg"] == 35.0


def test_vertical_camera_uses_image_up_as_world_view_heading():
    frame = SimpleNamespace(
        # Camera X points down; negative camera Z (image up) points world +Y.
        rotation_camera_to_world=[
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ]
    )

    assert _camera_frame_view_yaw_deg(frame) == 90.0


def test_locked_target_reorientation_cancels_planning_and_ignores_semantic_heading():
    stage = TaskStage(
        index=3,
        instruction="Fly above the red car",
        mode="target",
        target="red car",
        relation="above",
    )

    class Client:
        yaw = 0.0
        commanded = []

        def get_pose(self):
            return [32.0, -1.0, -14.0], float(self.yaw)

        def rotate_to_yaw(self, yaw_deg, timeout=0.0):
            self.commanded.append((float(yaw_deg), float(timeout)))
            self.yaw = float(yaw_deg)

    class Controller:
        planning = True
        has_plan_job = True

        def __init__(self):
            self.queue = SimpleNamespace(world_waypoints=[[27.0, -4.0, -14.0]])
            self.cleared = 0

        def clear(self):
            self.cleared += 1
            self.queue.world_waypoints.clear()
            self.planning = False
            self.has_plan_job = False

    class Watchdog:
        stopped = 0

        def stop(self):
            self.stopped += 1
            return True

    path_stream = SimpleNamespace(stopped=0)
    path_stream.stop = lambda: setattr(path_stream, "stopped", path_stream.stopped + 1)
    pipeline = SimpleNamespace(cleared=0)
    pipeline.clear = lambda: setattr(pipeline, "cleared", pipeline.cleared + 1)
    state = SimpleNamespace(values={})
    state.update = lambda **values: state.values.update(values)
    memory = SimpleNamespace(
        config={
            "RETURN_TARGET_REORIENT_ENABLED": True,
            "RETURN_TARGET_REORIENT_TRIGGER_DEG": 90.0,
            "RETURN_TARGET_REORIENT_TIMEOUT_S": 8.0,
        },
        has_primary=lambda _stage: True,
        above_overhead_context=lambda _stage, _position: {
            "active": True,
            "phase": "verify",
            "trusted": False,
        },
        preferred_yaw_deg=lambda _stage, _position: 160.0,
        record_pose=lambda *_args: None,
        summary=lambda _stage: {},
    )
    controller = Controller()
    watchdog = Watchdog()
    client = Client()
    # A deliberately different semantic heading proves that active geometric
    # recovery does not consume the previous stage transition reference.
    task_manager = TaskManager()
    task_manager.start_with_stages("mission", [stage])
    task_manager.update_stage_context(stage.index, semantic_view_yaw_deg=45.0)
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=controller,
        completion_pipeline=pipeline,
        task_manager=task_manager,
    )

    outcome = _reorient_to_locked_target_if_behind(
        objects,
        client,
        path_stream,
        state,
        stage,
        completion_watchdog=watchdog,
    )

    assert outcome == "completed"
    assert watchdog.stopped == 1
    assert controller.cleared == 1
    assert controller.queue.world_waypoints == []
    assert pipeline.cleared == 1
    assert path_stream.stopped == 1
    assert client.commanded == [(160.0, 8.0)]


def test_locked_target_reorientation_waits_for_watchdog_rpc_exit():
    stage = TaskStage(
        index=0,
        instruction="Fly to the red car",
        mode="target",
        target="red car",
        relation="near",
    )
    client = SimpleNamespace(
        get_pose=lambda: ([0.0, 0.0, -5.0], 0.0),
        rotate_to_yaw=lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("rotation must wait for watchdog")
        ),
    )
    controller = SimpleNamespace(
        queue=SimpleNamespace(world_waypoints=[[-5.0, 0.0, -5.0]]),
        clear=lambda: (_ for _ in ()).throw(AssertionError("queue must remain until RPC exits")),
    )
    memory = SimpleNamespace(
        config={"RETURN_TARGET_REORIENT_ENABLED": True},
        has_primary=lambda _stage: True,
        preferred_yaw_deg=lambda _stage, _position: 170.0,
    )
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=controller,
        completion_pipeline=None,
    )
    path_stream = SimpleNamespace(
        stop=lambda: (_ for _ in ()).throw(AssertionError("path must remain until RPC exits"))
    )
    watchdog = SimpleNamespace(stop=lambda: False)
    state = SimpleNamespace(update=lambda **_values: None)

    outcome = _reorient_to_locked_target_if_behind(
        objects,
        client,
        path_stream,
        state,
        stage,
        completion_watchdog=watchdog,
    )

    assert outcome == "deferred"


def test_encounter_binding_uses_forward_distance_not_detector_score():
    memory = _memory()
    image = Image.new("RGB", (640, 480), (120, 120, 120))
    stage = TaskStage(
        index=2,
        instruction="Fly above the first manhole cover encountered",
        mode="target",
        target="manhole cover",
        relation="above",
        ordinal=1,
        selection_rule="encounter",
        view_relative=True,
    )
    memory.begin_view_relative_binding(stage, [0.0, 0.0, -8.0], 0.0)
    far_high_score = _detection(15.0, 0.99)
    near_lower_score = _detection(8.0, 0.55)

    events = memory.update_from_detections(
        stage=stage,
        detections_by_view={"front": [far_high_score, near_lower_score]},
        images_by_view={"front": image},
        observer_world=[0.0, 0.0, -8.0],
        observer_yaw_deg=0.0,
    )

    primary = memory.primary_instance(stage)
    assert primary is not None
    assert primary.depth_median == 8.0
    assert events[0].detection is near_lower_score


def test_encounter_corridor_rejects_behind_and_far_lateral_candidates():
    memory = _memory()
    stage = TaskStage(
        index=2,
        instruction="Fly above the first manhole cover encountered",
        mode="target",
        target="manhole cover",
        ordinal=1,
        selection_rule="encounter",
        view_relative=True,
    )
    memory.begin_view_relative_binding(stage, [0.0, 0.0, -8.0], 0.0)

    assert memory._view_relative_observation_allowed(
        stage,
        {"activation_forward_projection": 8.0, "activation_lateral_projection": 1.0},
    )
    assert not memory._view_relative_observation_allowed(
        stage,
        {"activation_forward_projection": -2.0, "activation_lateral_projection": 0.0},
    )
    assert not memory._view_relative_observation_allowed(
        stage,
        {"activation_forward_projection": 8.0, "activation_lateral_projection": 8.0},
    )
