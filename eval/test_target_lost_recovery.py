"""Focused regression tests for scale-aware target-loss recovery."""

from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import numpy as np
from PIL import Image

from agent.functions.fast_slow.completion_pipeline import CompletionPipeline, CompletionPipelineEvent
from agent.functions.fast_slow.runtime import (
    _build_locked_relocalization_validator,
    _handle_background_target_lost,
    _runtime_stage_key,
)
from agent.functions.memory import MissionMemory
from agent.functions.memory.mission_memory import stage_key
from agent.functions.memory.schemas import TargetInstanceBelief, TargetMemory
from agent.functions.perception import TargetBearingTracker
from agent.functions.relocalization import (
    RelocalizationResult,
    TargetLostRecoveryState,
    TargetRelocalizer,
)
from agent.models.detection.base import DetectionResult


def _stage(*, target="building", relation="near", instruction=None):
    return SimpleNamespace(
        index=0,
        instruction=instruction or f"Fly to the {target}",
        mode="target",
        target=target,
        target_query=target,
        relation=relation,
        ordinal=None,
        selection_rule="",
        completion_condition="",
    )


def _detection(*, label="person", bbox=None, depth=10.0, score=0.8, camera="front"):
    depth_value = None if depth is None else float(depth)
    return DetectionResult(
        visible=True,
        bbox=list(bbox or [280, 180, 360, 300]),
        score=float(score),
        label=label,
        depth_median=depth_value,
        depth_valid_ratio=None if depth is None else 0.9,
        depth_mad_m=None if depth is None else 0.1,
        depth_bbox_median=depth_value,
        depth_p10_m=depth_value,
        depth_p90_m=depth_value,
        camera=camera,
    )


def _locked_memory(stage, *, large: bool, target_world=None):
    memory = MissionMemory(
        config={
            "ENABLED": True,
            "MIN_DETECTION_SCORE": 0.1,
            "FRONT_CAMERA_OFFSET": [0.0, 0.0, 0.0],
            "BEARING_LOST_CONFIRMATIONS": 1,
            "METRIC_LOCK_MAX_DEPTH_M": 120.0,
            "METRIC_DEPTH_MIN_VALID_RATIO": 0.2,
            "METRIC_DEPTH_MAX_MAD_RATIO": 0.12,
            "RELOCALIZATION_MEMORY_MIN_CONFIDENCE": 0.1,
            "RELOCALIZATION_MEMORY_MAX_UNCERTAINTY_M": 10.0,
            "RELOCALIZATION_MEMORY_MAX_AGE_S": 120.0,
        },
        sim_config={"FRONT_FOV": 90.0, "DOWN_FOV": 90.0},
    )
    target_name = str(stage.target)
    target = TargetMemory(
        target_key=target_name,
        target_name=target_name,
        primary_instance_id=f"{target_name}:1",
    )
    point = list(target_world or [10.0, 0.0, 0.0])
    instance = TargetInstanceBelief(
        instance_id=f"{target_name}:1",
        encounter_order=1,
        target_world=point,
        confidence=0.8,
        observation_count=3,
        uncertainty_m=1.0,
        surface_points_world=[point] if large else [],
        surface_patches_world=[[point]] if large else [],
        surface_observation_count=3 if large else 0,
        geometry_kind="large_surface" if large else "point",
        is_large_structure=large,
    )
    target.instances[instance.instance_id] = instance
    memory.target_memories[target_name] = target
    memory.stage_locks[stage_key(stage)] = instance.instance_id
    return memory, instance


class _State:
    def __init__(self):
        self.updates = []

    def update(self, **kwargs):
        self.updates.append(kwargs)


class _Controller:
    def __init__(self, *, has_plan_job=False, waypoints=None):
        self.has_plan_job = bool(has_plan_job)
        self.planning = False
        self.queue = SimpleNamespace(world_waypoints=list(waypoints or []))
        self.clear_count = 0

    def clear(self):
        self.clear_count += 1
        self.queue.world_waypoints.clear()


class _Path:
    def __init__(self, *, active=False):
        self.active = bool(active)
        self.stop_count = 0

    def stop(self):
        self.stop_count += 1
        self.active = False

    def emergency_stop(self):
        self.stop()


def test_locked_building_loss_keeps_completed_plan_job_and_never_rotates():
    stage = _stage(target="building")
    memory, _instance = _locked_memory(stage, large=True)
    rotations = []
    client = SimpleNamespace(
        get_pose=lambda: ([0.0, 0.0, -20.0], 37.0),
        rotate_yaw=lambda value: rotations.append(value),
        rotate_to_yaw=lambda value: rotations.append(value),
    )
    controller = _Controller(has_plan_job=True)
    path = _Path(active=False)
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=controller,
        target_bearing_tracker=TargetBearingTracker(memory.config, memory.sim_config),
        target_lost_recovery=TargetLostRecoveryState({}),
    )

    assert not _handle_background_target_lost(
        objects,
        client,
        path,
        _State(),
        stage,
        "batch",
        "building dropped below the front camera",
    )
    assert rotations == []
    assert controller.clear_count == 0
    assert controller.has_plan_job
    assert path.stop_count == 0


def test_above_building_front_loss_switches_to_down_view_without_rotation():
    stage = _stage(target="building", relation="above", instruction="Fly above the building")
    memory, _instance = _locked_memory(stage, large=True)
    rotations = []
    client = SimpleNamespace(
        get_pose=lambda: ([8.0, 0.0, -25.0], 12.0),
        rotate_yaw=lambda value: rotations.append(value),
        rotate_to_yaw=lambda value: rotations.append(value),
    )
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=_Controller(waypoints=[[12.0, 0.0, -25.0]]),
        target_bearing_tracker=TargetBearingTracker(memory.config, memory.sim_config),
        target_lost_recovery=TargetLostRecoveryState({}),
    )

    assert not _handle_background_target_lost(
        objects,
        client,
        _Path(active=True),
        _State(),
        stage,
        "batch",
        "facade not visible from above",
    )
    assert rotations == []
    assert objects.controller.queue.world_waypoints == [[12.0, 0.0, -25.0]]


def test_compact_relocalizer_uses_finite_fan_and_validator_skips_wrong_same_class():
    stage = _stage(target="person")
    image = Image.new("RGB", (640, 480), (100, 110, 120))
    depth = np.full((48, 64), 10.0, dtype=np.float32)

    class Client:
        def __init__(self):
            self.yaw = 0.0
            self.rotations = []

        def get_pose(self):
            return [0.0, 0.0, 0.0], self.yaw

        def rotate_to_yaw(self, yaw):
            self.rotations.append(float(yaw))
            self.yaw = float(yaw)

        def rotate_yaw(self, delta):
            self.rotations.append(self.yaw + float(delta))
            self.yaw += float(delta)

        def capture_views(self, **_kwargs):
            return image, image, depth, depth, {}

    client = Client()

    class Detector:
        @staticmethod
        def detect_all(_image, _caption, depth_meters=None, camera_name="front"):
            if camera_name != "front" or abs(client.yaw - 15.0) > 0.1:
                return []
            return [
                _detection(label="wrong person", score=0.95),
                _detection(label="locked person", score=0.70),
            ]

    relocalizer = TargetRelocalizer(
        {"RELOCALIZATION": {"CAPTURE_PROFILE": "front_down_both_depth", "CENTER_ON_TARGET": False}},
        detector=Detector(),
    )
    result = relocalizer.search(
        client,
        stage,
        capture_mode="batch",
        skip_initial_frame=True,
        yaw_offsets_deg=[0.0, -15.0, 15.0, -30.0, 30.0],
        base_yaw_deg=0.0,
        max_total_rotation_deg=300.0,
        validator=lambda _s, _f, _d, _fd, _dd, candidate: (
            candidate if candidate.label == "locked person" else None
        ),
        session_id="fan-1",
    )

    assert result.found
    assert result.detection.label == "locked person"
    assert result.searched_yaws_deg == [0.0, 345.0, 15.0]
    assert len(result.searched_yaws_deg) < 6
    assert result.total_rotation_deg <= 300.0


def test_relocalizer_itself_refuses_large_structure_yaw_search():
    stage = _stage(target="building")

    class Client:
        rotations = []
        captures = 0

        @staticmethod
        def get_pose():
            return [0.0, 0.0, 0.0], 0.0

        @classmethod
        def rotate_yaw(cls, value):
            cls.rotations.append(value)

        @classmethod
        def capture_views(cls, **_kwargs):
            cls.captures += 1
            return None, None, None, None, {}

    result = TargetRelocalizer(
        {"RELOCALIZATION": {"ENABLED": True}},
        detector=SimpleNamespace(),
    ).search(Client(), stage, skip_initial_frame=True)

    assert not result.found
    assert "yaw search forbidden" in result.reason
    assert Client.rotations == []
    assert Client.captures == 0


def test_failed_global_search_including_heading_restore_stays_inside_hard_rotation_budget():
    stage = _stage(target="person")
    image = Image.new("RGB", (640, 480), (100, 100, 100))

    class Client:
        def __init__(self):
            self.yaw = 0.0

        def get_pose(self):
            return [0.0, 0.0, 0.0], self.yaw

        def rotate_to_yaw(self, yaw):
            self.yaw = float(yaw) % 360.0

        def rotate_yaw(self, delta):
            self.yaw = (self.yaw + float(delta)) % 360.0

        @staticmethod
        def capture_views(**_kwargs):
            return image, image, None, None, {}

    class Detector:
        @staticmethod
        def detect_all(*_args, **_kwargs):
            return []

    recovery = TargetLostRecoveryState({})
    session, _reason = recovery.begin_small_session(
        stage_key=_runtime_stage_key(stage),
        instance_id="",
        current_position=[0.0, 0.0, 0.0],
        current_yaw_deg=0.0,
        preferred_yaw_deg=None,
        reliable_memory=False,
    )
    result = TargetRelocalizer(
        {"RELOCALIZATION": {"CENTER_ON_TARGET": False}},
        detector=Detector(),
    ).search(
        Client(),
        stage,
        skip_initial_frame=True,
        yaw_offsets_deg=recovery.search_offsets(session),
        base_yaw_deg=0.0,
        max_total_rotation_deg=420.0,
    )

    assert not result.found
    assert result.total_rotation_deg <= 420.0


def test_locked_identity_validator_rejects_wrong_metric_instance():
    stage = _stage(target="person")
    memory, instance = _locked_memory(stage, large=False, target_world=[10.0, 0.0, 0.0])
    client = SimpleNamespace(get_pose=lambda: ([0.0, 0.0, 0.0], 0.0))
    objects = SimpleNamespace(mission_memory=memory)
    validator = _build_locked_relocalization_validator(objects, client, stage)
    image = Image.new("RGB", (640, 480), (120, 120, 120))

    wrong = _detection(label="person", depth=35.0)
    correct = _detection(label="person", depth=10.0)
    assert validator(stage, image, image, wrong, wrong, wrong) is None
    assert validator(stage, image, image, correct, correct, correct) is correct
    assert memory.primary_instance(stage).instance_id == instance.instance_id


def test_failed_global_search_cannot_repeat_without_moving_three_meters():
    recovery = TargetLostRecoveryState({"SMALL_RETRY_MIN_MOVE_M": 3.0})
    key = (0, "find person", "target")
    session, _reason = recovery.begin_small_session(
        stage_key=key,
        instance_id="",
        current_position=[0.0, 0.0, 0.0],
        current_yaw_deg=0.0,
        preferred_yaw_deg=None,
        reliable_memory=False,
    )
    assert session is not None and session.global_search
    recovery.finish_small_session(
        session,
        found=False,
        searched_yaws_deg=[0.0, 90.0, 180.0, 270.0],
        total_rotation_deg=360.0,
    )

    repeated, reason = recovery.begin_small_session(
        stage_key=key,
        instance_id="",
        current_position=[1.0, 0.0, 0.0],
        current_yaw_deg=0.0,
        preferred_yaw_deg=None,
        reliable_memory=False,
    )
    assert repeated is None
    assert "move 3.0m" in reason
    moved, _reason = recovery.begin_small_session(
        stage_key=key,
        instance_id="",
        current_position=[3.1, 0.0, 0.0],
        current_yaw_deg=0.0,
        preferred_yaw_deg=None,
        reliable_memory=False,
    )
    assert moved is not None


def test_successful_compact_relocalization_writes_memory_and_resets_lost_count():
    stage = _stage(target="person")
    memory, instance = _locked_memory(stage, large=False, target_world=[10.0, 0.0, 0.0])
    image = Image.new("RGB", (640, 480), (120, 120, 120))
    depth = np.full((48, 64), 10.0, dtype=np.float32)
    detection = _detection(label="person", depth=10.0)
    tracker = TargetBearingTracker(memory.config, memory.sim_config)
    tracker.mark_lost(_runtime_stage_key(stage))

    class Relocalizer:
        enabled = True

        @staticmethod
        def search(*_args, **kwargs):
            return RelocalizationResult(
                found=True,
                detection=detection,
                front_image=image,
                down_image=image,
                front_depth=depth,
                down_depth=depth,
                front_detections=[detection],
                down_detections=[],
                observer_world=[0.0, 0.0, 0.0],
                observer_yaw_deg=0.0,
                capture_timestamp_s=1.0,
                session_id=str(kwargs.get("session_id", "")),
                searched_yaws_deg=[0.0],
                reason="validated target detected in front view",
            )

    controller = _Controller(waypoints=[[3.0, 0.0, 0.0]])
    pipeline = SimpleNamespace(clear_count=0)
    pipeline.clear = lambda: setattr(pipeline, "clear_count", pipeline.clear_count + 1)
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=controller,
        relocalizer=Relocalizer(),
        target_bearing_tracker=tracker,
        target_lost_recovery=TargetLostRecoveryState({}),
        completion_pipeline=pipeline,
        distance_estimator=SimpleNamespace(enabled=False, clear=lambda: None),
        navigation_metrics=SimpleNamespace(invalidate_target=lambda _key: None),
    )
    before = instance.observation_count

    assert not _handle_background_target_lost(
        objects,
        SimpleNamespace(get_pose=lambda: ([0.0, 0.0, 0.0], 0.0)),
        _Path(active=True),
        _State(),
        stage,
        "batch",
        "person temporarily occluded",
    )
    assert tracker.lost_count(_runtime_stage_key(stage)) == 0
    assert memory.primary_instance(stage).instance_id == instance.instance_id
    assert memory.primary_instance(stage).observation_count > before
    assert pipeline.clear_count >= 1


def test_stale_target_lost_generation_is_discarded_before_touching_queue():
    stage = _stage(target="building")
    memory, instance = _locked_memory(stage, large=True)
    controller = _Controller(has_plan_job=True)
    path = _Path(active=True)
    objects = SimpleNamespace(
        mission_memory=memory,
        controller=controller,
        stage_generations={_runtime_stage_key(stage): 2},
        lock_generations={_runtime_stage_key(stage): 1},
        target_lost_recovery=TargetLostRecoveryState({}),
    )
    event = CompletionPipelineEvent(
        kind="target_lost",
        stage_key=_runtime_stage_key(stage),
        stage_generation=1,
        lock_generation=1,
        locked_instance_id=instance.instance_id,
    )

    assert not _handle_background_target_lost(
        objects,
        SimpleNamespace(get_pose=lambda: ([0.0, 0.0, 0.0], 0.0)),
        path,
        _State(),
        stage,
        "batch",
        "late detector miss",
        event=event,
    )
    assert controller.clear_count == 0
    assert path.stop_count == 0


def test_detector_connection_error_becomes_nonfatal_event_with_retry_backoff():
    stage = _stage(target="person")
    checker = SimpleNamespace(
        uses_detector=True,
        should_check_stage=lambda _stage: True,
        is_detector_enabled=lambda: True,
    )
    detect_executor = ThreadPoolExecutor(max_workers=1)
    slow_executor = ThreadPoolExecutor(max_workers=1)
    pipeline = CompletionPipeline(
        detector=SimpleNamespace(),
        checker=checker,
        client=SimpleNamespace(),
        capture_mode="batch",
        detect_executor=detect_executor,
        slow_executor=slow_executor,
        stop_radius_m=-1.0,
        slow_radius_m=5.0,
        error_retry_backoff_s=2.0,
        error_retry_max_backoff_s=10.0,
    )

    def fail_detection(_stage, _task_text):
        raise TimeoutError("GroundingDINO connection timed out")

    pipeline._capture_detect_depth_bundle = fail_detection
    try:
        assert pipeline.submit(
            stage,
            stage.instruction,
            None,
            None,
            stage_generation=3,
            lock_generation=2,
            session_id="session-1",
            locked_instance_id="person:1",
        )
        event = None
        deadline = time.perf_counter() + 2.0
        while event is None and time.perf_counter() < deadline:
            event = pipeline.poll()
            time.sleep(0.005)

        assert event is not None
        assert event.kind == "perception_error"
        assert "TimeoutError" in event.error
        assert event.retry_after_s == 2.0
        assert event.stage_generation == 3
        assert event.lock_generation == 2
        assert not pipeline.submit(stage, stage.instruction, None, None)
    finally:
        detect_executor.shutdown(wait=True, cancel_futures=True)
        slow_executor.shutdown(wait=True, cancel_futures=True)
