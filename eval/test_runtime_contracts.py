"""Focused tests for camera, relation and Qwen runtime contracts."""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
from PIL import Image

from agent.functions.common import web_runtime_helpers
from agent.functions.completion.task_completion import TaskCompletionChecker
from agent.functions.fast_slow import runtime
from agent.functions.fast_slow.runtime import _planning_geometry_context
from agent.functions.memory.schemas import TargetInstanceBelief
from agent.functions.memory.spatial_reasoning import (
    evaluate_memory_completion,
    requested_above_clearance_m,
)
from agent.functions.task_parser.vlm_task_parser import parse_task_parser_to_stages
from agent.models.planner.api_atomic_planner import ApiAtomicPlanner
from agent.models.planner.prompt_contract import build_sliding_window_prompt
from lora_example.extract_training_samples_sliding_window import (
    build_prompt as build_training_prompt,
    compute_incremental_body_waypoints,
)
from sim.camera_frames import CameraFrame, CameraIntrinsics
from web.shared_state import SharedState


def test_task_run_result_exposes_reliable_terminal_status():
    success = runtime.TaskRunResult("success", "all stages completed", 8, 2, 2)
    failed = runtime.TaskRunResult("failed", "detect stage exhausted", 3, 0, 1)
    assert success.success
    assert not failed.success
    assert failed.reason == "detect stage exhausted"

    state = SharedState()
    state.update(
        task_result_status=failed.status,
        task_result_reason=failed.reason,
        task_result_steps=failed.steps,
        task_result_completed_stages=failed.completed_stages,
        task_result_total_stages=failed.total_stages,
    )
    public = state.get_state()
    assert public["task_result_status"] == "failed"
    assert public["task_result_steps"] == 3
    assert public["task_result_total_stages"] == 1


def test_detect_stage_completes_from_rgb_without_metric_depth(monkeypatch):
    frame = Image.new("RGB", (100, 80))
    detection = SimpleNamespace(visible=True, score=0.91)
    stage = SimpleNamespace(
        index=1,
        instruction="Detect the fountain",
        mode="detect",
        target="fountain",
    )

    class TaskManager:
        completed_reason = ""

        def complete_current(self, reason):
            self.completed_reason = reason

        def summary(self):
            return "1/1 stages completed"

        def is_done(self):
            return True

    class State:
        values = {}

        def update(self, **kwargs):
            self.values.update(kwargs)

    objects = SimpleNamespace(
        detect_stage_attempts={(1, "Detect the fountain", "detect"): 1},
        mission_memory=None,
        task_manager=TaskManager(),
        navigation_metrics=SimpleNamespace(task_completed=False),
    )
    monkeypatch.setattr(runtime, "_clear_stopped_queue", lambda *_args: None)
    monkeypatch.setattr(runtime, "_prebind_target_from_bundle", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(runtime, "_record_target_bearing", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(
        runtime.web_helpers,
        "capture_profile_isolated_with_pose",
        lambda *_args: (frame, None, None, None, {"total_s": 0.01}, [0.0, 0.0, 0.0], 0.0),
    )
    monkeypatch.setattr(
        runtime,
        "_detect_dual_view",
        lambda *_args: (detection, detection, None, 0.02, [detection], []),
    )
    state = State()
    outcome = runtime._handle_detect_stage(
        objects,
        client=object(),
        path_stream=object(),
        state=state,
        stage=stage,
        task_text=stage.instruction,
    )
    assert outcome == "complete"
    assert "RGB identity lock" in objects.task_manager.completed_reason
    assert objects.navigation_metrics.task_completed
    assert state.values["task_done"] is True


def test_detect_stage_has_bounded_failure_without_planner(monkeypatch):
    stage = SimpleNamespace(index=1, instruction="Detect the fountain", mode="detect")

    class TaskManager:
        failed_reason = ""

        def fail_current(self, reason):
            self.failed_reason = reason

    class State:
        values = {}

        def update(self, **kwargs):
            self.values.update(kwargs)

    objects = SimpleNamespace(
        detect_stage_attempts={},
        task_failed=False,
        task_manager=TaskManager(),
    )
    state = State()
    monkeypatch.setattr(runtime, "_clear_stopped_queue", lambda *_args: None)
    first = runtime._detect_stage_retry_or_fail(
        objects, object(), state, stage, "target not found"
    )
    second = runtime._detect_stage_retry_or_fail(
        objects, object(), state, stage, "target not found"
    )
    third = runtime._detect_stage_retry_or_fail(
        objects, object(), state, stage, "target not found"
    )
    assert first == "retry"
    assert second == "retry"
    assert third == "failed"
    assert objects.task_failed
    assert objects.task_manager.failed_reason == "target not found"
    assert state.values["status"] == "failed"


def _camera_frame(camera_id="tilted", optical_axis=(0.707, 0.0, 0.707)):
    axis_x, axis_y, axis_z = optical_axis
    return CameraFrame(
        camera_id=camera_id,
        capture_id="capture-1",
        timestamp_ns=1,
        rgb_intrinsics=CameraIntrinsics.from_horizontal_fov(100, 80, 90.0),
        depth_intrinsics=CameraIntrinsics.from_horizontal_fov(20, 16, 90.0),
        camera_position_world=[1.0, 2.0, -3.0],
        rotation_camera_to_world=[
            [axis_x, 0.0, 0.0],
            [axis_y, 1.0, 0.0],
            [axis_z, 0.0, 1.0],
        ],
    )


def test_isolated_capture_transfers_rgb_and_numpy_depth_camera_frames(monkeypatch):
    rgb = Image.new("RGB", (100, 80))
    depth = np.ones((16, 20), dtype=np.float32)
    frame = _camera_frame()

    class IsolatedClient:
        def __init__(self, **_kwargs):
            self.values = {id(rgb): frame, id(depth): frame}

        def get_pose(self):
            return [1.0, 2.0, -3.0], 0.0

        def capture_views(self, **_kwargs):
            return rgb, None, depth, None, {"total_s": 0.01}

        def camera_frame_for_image(self, value):
            return self.values.get(id(value))

    class MainClient:
        _ip = ""
        _port = 41451

        def __init__(self):
            self.values = {}

        def associate_camera_frame(self, value, camera_frame):
            self.values[id(value)] = camera_frame

        def camera_frame_for_image(self, value):
            return self.values.get(id(value))

    monkeypatch.setattr(web_runtime_helpers, "AirSimClient", IsolatedClient)
    main = MainClient()
    captured = web_runtime_helpers.capture_profile_isolated_with_pose(main, "front_depth")
    assert main.camera_frame_for_image(captured[0]) is frame
    assert main.camera_frame_for_image(captured[2]) is frame


def test_planning_geometry_falls_back_to_image_camera_frame():
    image = Image.new("RGB", (100, 80))
    image.camera_frame = _camera_frame()
    client = SimpleNamespace(camera_frame_for_image=lambda _image: None)
    objects = SimpleNamespace(mission_memory=None, obstacle_avoider=None)
    images, context = _planning_geometry_context(
        objects,
        client,
        SimpleNamespace(),
        [0.0, 0.0, 0.0],
        0.0,
        [image],
    )
    assert images == [image]
    assert context["camera_views"][0]["camera_id"] == "tilted"
    assert context["camera_views"][0]["optical_axis_world"][2] == 0.707


def _instance():
    return TargetInstanceBelief(
        instance_id="tower:1",
        encounter_order=1,
        target_world=[0.0, 0.0, 0.0],
        confidence=0.9,
        observation_count=3,
        sigma_xy=0.2,
        sigma_z=0.2,
        uncertainty_m=0.3,
    )


def _pass_stage(instruction="Fly past the tower", relation="pass"):
    return SimpleNamespace(
        instruction=instruction,
        completion_condition="continue beyond it",
        relation=relation,
    )


def test_pass_completion_requires_crossing_and_exit():
    decision = evaluate_memory_completion(
        stage=_pass_stage(),
        instance=_instance(),
        current_world=[10.0, 0.0, 0.0],
        config={"PASS_MIN_CONFIDENCE": 0.5},
        stop_radius_m=4.0,
        pose_history=[[-10.0, 0.0, 0.0], [-3.0, 0.0, 0.0], [0.5, 0.0, 0.0], [10.0, 0.0, 0.0]],
    )
    assert decision.done
    assert decision.reason == "memory_pass_crossing_complete"


def test_pass_completion_rejects_same_side_retreat():
    decision = evaluate_memory_completion(
        stage=_pass_stage(),
        instance=_instance(),
        current_world=[-10.0, 0.0, 0.0],
        config={"PASS_MIN_CONFIDENCE": 0.5},
        stop_radius_m=4.0,
        pose_history=[[-10.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [-10.0, 0.0, 0.0]],
    )
    assert not decision.done


def _above_stage(instruction, distance=None):
    return SimpleNamespace(
        instruction=instruction,
        completion_condition="",
        relation="above",
        relation_distance_m=distance,
    )


def test_plain_above_has_no_maximum_clearance_but_explicit_height_does():
    config = {
        "ABOVE_REQUIRE_ROOF_GEOMETRY": False,
        "MEMORY_ONLY_MIN_OBSERVATIONS": 1,
        "MEMORY_ONLY_MIN_CONFIDENCE": 0.5,
        "MAX_COMPLETION_UNCERTAINTY_M": 5.0,
    }
    plain = evaluate_memory_completion(
        stage=_above_stage("Fly above the fountain"),
        instance=_instance(),
        current_world=[0.0, 0.0, -50.0],
        config=config,
    )
    explicit_far = evaluate_memory_completion(
        stage=_above_stage("Fly 10 meters above the fountain", 10.0),
        instance=_instance(),
        current_world=[0.0, 0.0, -50.0],
        config=config,
    )
    explicit_ok = evaluate_memory_completion(
        stage=_above_stage("Fly 10 meters above the fountain", 10.0),
        instance=_instance(),
        current_world=[0.0, 0.0, -10.0],
        config=config,
    )
    assert plain.done
    assert not explicit_far.done
    assert explicit_ok.done


def test_completion_depth_gate_ignores_plain_above_depth_only():
    checker = TaskCompletionChecker({"FUNCTIONS": {"COMPLETION": {}}})
    detection = SimpleNamespace(visible=True, depth_median=40.0)
    assert checker._is_depth_inside_arrival_radius(
        _above_stage("Fly above the fountain"), detection
    )
    assert not checker._is_depth_inside_arrival_radius(
        _above_stage("Fly 10 meters above the fountain", 10.0), detection
    )


def test_parser_retains_explicit_above_clearance():
    stages = parse_task_parser_to_stages(
        '{"task_type":"single","stages":[{"index":1,"instruction":"Fly 12 meters above the fountain",'
        '"mode":"target","target":"fountain","relation":"above","relation_distance_m":12}]}',
        original_instruction="飞到喷泉上方12米",
    )
    assert stages[0].relation_distance_m == 12.0
    assert requested_above_clearance_m(stages[0]) == 12.0


def test_training_and_runtime_share_prompt_and_yaw_only_coordinates():
    geometry = {"camera_views": [{"camera_id": "front_center"}]}
    expected = build_sliding_window_prompt(
        "Fly near the tower",
        [[1.0, 2.0, 3.0]],
        max_additional=5,
        geometry_context=geometry,
    )
    assert build_training_prompt(
        "Fly near the tower",
        [[1.0, 2.0, 3.0]],
        5,
        geometry_context=geometry,
    ) == expected
    api_atomic = ApiAtomicPlanner.__new__(ApiAtomicPlanner)
    assert api_atomic._build_prompt(
        "Fly near the tower",
        [[1.0, 2.0, 3.0]],
        geometry_context=geometry,
    ) == expected
    pitch_45 = [0.0, math.sin(math.radians(22.5)), 0.0, math.cos(math.radians(22.5))]
    frames = [
        {"sensors": {"state": {"position": [0.0, 0.0, 0.0]}, "imu": {"orientation": pitch_45}}},
        {"sensors": {"state": {"position": [10.0, 0.0, 0.0]}, "imu": {"orientation": pitch_45}}},
    ]
    waypoint = compute_incremental_body_waypoints(frames, 0, [1])[0]
    assert waypoint == [10.0, 0.0, 0.0]
