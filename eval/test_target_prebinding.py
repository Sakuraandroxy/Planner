"""Focused regression tests for RGB target prebinding."""

from __future__ import annotations

from datetime import datetime
from types import SimpleNamespace

from PIL import Image

from agent.functions.debug import TargetSnapshotRecorder
from agent.functions.fast_slow.completion_pipeline import DetectionDepthBundle
from agent.functions.fast_slow.runtime import (
    _apply_bearing_path_guard,
    _bearing_only_active,
    _poll_plan,
    _prebind_target_from_bundle,
    _select_prebind_front_detection,
)
from agent.functions.perception import TargetBearingTracker
from agent.models.detection.base import DetectionResult


def _stage(*, view_relative: bool = False):
    return SimpleNamespace(
        index=0,
        instruction=(
            "fly to the first building on the right"
            if view_relative
            else "fly to the building"
        ),
        mode="target",
        target="building",
        target_query="building",
        relation="near",
        view_relative=view_relative,
        ordinal=1 if view_relative else None,
        selection_rule="ordinal" if view_relative else "stable",
        completion_condition="",
    )


def _detection(bbox, *, score=0.9, depth=18.0):
    return DetectionResult(
        visible=True,
        bbox=list(bbox),
        score=float(score),
        label="building",
        depth_median=depth,
        camera="front",
    )


class _Memory:
    sim_config = {"FRONT_FOV": 90.0}

    def __init__(self, *, exclusions=None):
        self.config = {
            "PREBIND_ENABLED": True,
            "BEARING_ONLY_ENABLED": True,
            "BEARING_MIN_CONFIDENCE": 0.40,
            "PREBIND_MAX_AGE_S": 30.0,
            "PREBIND_MAX_LEG_M": 10.0,
            "PREBIND_MAX_PATH_DEVIATION_DEG": 18.0,
            "METRIC_LOCK_MAX_DEPTH_M": 120.0,
        }
        self._exclusions = list(exclusions or [])

    def previous_entity_exclusions(self, _stage, _world, _yaw):
        return list(self._exclusions)

    @staticmethod
    def evaluate_locked_detection_identity(_stage, _detection, _image, **_kwargs):
        return {"accepted": True, "reason": "stage_not_locked"}

    @staticmethod
    def estimate_distance(_stage, _current_world):
        # A trustworthy old metric estimate must not override the newer
        # activation-view RGB direction on the very first leg.
        return {
            "confidence": 0.95,
            "uncertainty_m": 0.5,
            "observation_age_s": 1.0,
        }


def _bundle(image, detections, *, preferred=None):
    return DetectionDepthBundle(
        best_detection=preferred,
        front_detection=preferred,
        down_detection=None,
        front_detections=list(detections),
        down_detections=[],
        front_image=image,
        down_image=None,
        observer_world=[0.0, 0.0, -8.0],
        observer_yaw_deg=0.0,
    )


def test_prebind_is_rgb_only_and_saves_first_boxed_frame(tmp_path):
    stage = _stage()
    image = Image.new("RGB", (100, 80), "white")
    detection = _detection((62, 15, 88, 62), depth=24.0)
    memory = _Memory()
    recorder = TargetSnapshotRecorder(
        enabled=True,
        output_root=tmp_path / "output",
        started_at=datetime(2026, 8, 26, 9, 7),
    )
    recorder.begin_task(stage.instruction)
    tracker = TargetBearingTracker(memory.config, memory.sim_config)
    objects = SimpleNamespace(
        mission_memory=memory,
        target_bearing_tracker=tracker,
        target_snapshot_recorder=recorder,
    )

    assert _prebind_target_from_bundle(
        objects,
        stage,
        _bundle(image, [detection], preferred=detection),
        source="stage_activation_prebind",
    )

    observation = tracker.current((0, stage.instruction, "target"))
    assert observation is not None
    assert observation.source == "stage_activation_prebind"
    assert observation.depth_state == "unavailable"
    assert observation.range_hint_m is None
    assert detection.depth_median == 24.0
    saved = list(recorder.run_directory.glob("*.jpg"))
    assert [path.name for path in saved] == [
        "task_01_stage_01_building_prebind_front.jpg"
    ]


def test_view_relative_prebind_uses_requested_sector_and_previous_entity_exclusion():
    stage = _stage(view_relative=True)
    image = Image.new("RGB", (100, 80), "white")
    wrong_left = _detection((8, 18, 32, 60), score=0.99)
    right = _detection((68, 18, 92, 60), score=0.75)
    bundle = _bundle(image, [wrong_left, right], preferred=wrong_left)
    objects = SimpleNamespace(mission_memory=_Memory())

    assert _select_prebind_front_detection(objects, stage, bundle) is right

    right_center_bearing = ((80.0 / 100.0) - 0.5) * 90.0
    objects.mission_memory = _Memory(
        exclusions=[{"bearing_deg": right_center_bearing, "tolerance_deg": 5.0}]
    )
    assert _select_prebind_front_detection(objects, stage, bundle) is None


def test_view_relative_prebind_prefers_perspective_near_candidate_over_dino_score():
    stage = _stage(view_relative=True)
    image = Image.new("RGB", (100, 80), "white")
    far_high_score = _detection((58, 8, 78, 40), score=0.98, depth=None)
    near_lower_score = _detection((62, 25, 96, 76), score=0.72, depth=None)
    bundle = _bundle(
        image,
        [far_high_score, near_lower_score],
        preferred=far_high_score,
    )

    selected = _select_prebind_front_detection(
        SimpleNamespace(mission_memory=_Memory()),
        stage,
        bundle,
    )

    assert selected is near_lower_score


def test_prebind_overrides_trustworthy_old_memory_and_guards_first_leg():
    stage = _stage()
    image = Image.new("RGB", (100, 80), "white")
    detection = _detection((70, 20, 90, 60), depth=None)
    memory = _Memory()
    tracker = TargetBearingTracker(memory.config, memory.sim_config)
    tracker.record(
        stage_key=(0, stage.instruction, "target"),
        detection=detection,
        image=image,
        observer_yaw_deg=0.0,
        source="stage_activation_prebind",
    )
    objects = SimpleNamespace(mission_memory=memory, target_bearing_tracker=tracker)

    assert _bearing_only_active(objects, stage, [0.0, 0.0, -8.0])
    guarded, reason = _apply_bearing_path_guard(
        objects,
        stage,
        [[20.0, 0.0, 0.0]],
        selection_pos=[0.0, 0.0, -8.0],
        selection_yaw=0.0,
    )

    assert reason.startswith("prebind_replace_bearing_deviation_")
    assert len(guarded) == 1
    assert 8.0 < guarded[0][0] < 10.0
    assert guarded[0][1] > 3.0
    assert guarded[0][2] == 0.0


def test_metric_lock_does_not_erase_unconsumed_activation_bearing():
    stage = _stage()
    image = Image.new("RGB", (100, 80), "white")
    memory = _Memory()
    tracker = TargetBearingTracker(memory.config, memory.sim_config)
    stage_key = (0, stage.instruction, "target")

    tracker.record(
        stage_key=stage_key,
        detection=_detection((8, 20, 32, 60), depth=None),
        image=image,
        observer_yaw_deg=0.0,
        source="stage_activation_prebind",
    )
    tracker.record(
        stage_key=stage_key,
        detection=_detection((8, 20, 32, 60), depth=114.7),
        image=image,
        observer_yaw_deg=0.0,
        source="active_observation",
    )
    tracker.clear(stage_key, preserve_activation_guard=True)
    tracker.record(
        stage_key=stage_key,
        detection=_detection((8, 20, 32, 60), depth=114.7),
        image=image,
        observer_yaw_deg=0.0,
        source="active_observation",
    )

    assert tracker.current(stage_key).depth_state == "metric"
    assert tracker.activation_guard(stage_key) is not None
    assert _bearing_only_active(objects := SimpleNamespace(
        mission_memory=memory,
        target_bearing_tracker=tracker,
    ), stage, [0.0, 0.0, -8.0])

    guarded, reason = _apply_bearing_path_guard(
        objects,
        stage,
        [[18.0, 0.0, 0.0]],
        selection_pos=[0.0, 0.0, -8.0],
        selection_yaw=0.0,
    )

    assert reason.startswith("prebind_replace_bearing_deviation_")
    assert guarded[0][1] < -3.0
    assert tracker.consume_activation_guard(stage_key) is not None
    assert not _bearing_only_active(objects, stage, [0.0, 0.0, -8.0])


def test_activation_bearing_is_consumed_only_after_nonempty_path_enqueue():
    stage = _stage()
    image = Image.new("RGB", (100, 80), "white")
    memory = _Memory()
    tracker = TargetBearingTracker(memory.config, memory.sim_config)
    stage_key = (0, stage.instruction, "target")
    tracker.record(
        stage_key=stage_key,
        detection=_detection((8, 20, 32, 60), depth=None),
        image=image,
        observer_yaw_deg=0.0,
        source="stage_activation_prebind",
    )

    class _Controller:
        def __init__(self):
            self.queue = SimpleNamespace(world_waypoints=[])
            self.output = SimpleNamespace(
                waypoints=[],
                rejection_reason="",
                _consume_activation_bearing_guard=True,
            )

        def poll_plan(self, select_fn=None):
            if self.output.waypoints:
                self.queue.world_waypoints.append([1.0, 0.0, -8.0])
            return self.output

    controller = _Controller()
    objects = SimpleNamespace(
        mission_memory=memory,
        target_bearing_tracker=tracker,
        controller=controller,
    )

    _poll_plan(objects, stage=stage)
    assert tracker.activation_guard(stage_key) is not None

    controller.output = SimpleNamespace(
        waypoints=[[1.0, 0.0, 0.0]],
        rejection_reason="",
        _consume_activation_bearing_guard=True,
    )
    _poll_plan(objects, stage=stage)
    assert tracker.activation_guard(stage_key) is None
