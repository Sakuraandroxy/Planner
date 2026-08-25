"""Focused tests for optional first-locked-target snapshots."""

from __future__ import annotations

import json
from datetime import datetime
from types import SimpleNamespace

from PIL import Image

from agent.functions.debug import TargetSnapshotRecorder
from agent.functions.fast_slow.completion_pipeline import DetectionDepthBundle
from agent.functions.fast_slow.runtime import _record_locked_target_snapshot
from agent.models.detection.base import DetectionResult


def _detection(bbox=(10, 8, 42, 35), *, score=0.9, camera="front") -> DetectionResult:
    return DetectionResult(
        visible=True,
        bbox=list(bbox),
        score=float(score),
        label="building",
        depth_median=12.5,
        camera=camera,
    )


def test_disabled_recorder_has_no_filesystem_side_effect(tmp_path):
    output_root = tmp_path / "output"
    recorder = TargetSnapshotRecorder(
        output_root=output_root,
        started_at=datetime(2026, 8, 25, 14, 32),
    )
    recorder.begin_task("Fly to the building")

    saved = recorder.record_first(
        stage_key=(0, "Fly to the building", "target"),
        stage_index=0,
        instance_id="building:1",
        target_name="building",
        view="front",
        image=Image.new("RGB", (64, 48), "white"),
        detection=_detection(),
    )

    assert saved is None
    assert recorder.run_directory is None
    assert not output_root.exists()


def test_enabled_recorder_draws_box_writes_manifest_and_deduplicates(tmp_path):
    recorder = TargetSnapshotRecorder(
        enabled=True,
        output_root=tmp_path / "output",
        started_at=datetime(2026, 8, 25, 14, 32),
    )
    recorder.begin_task("Fly to the building")
    image = Image.new("RGB", (80, 60), "white")
    kwargs = dict(
        stage_key=(0, "Fly to the building", "target"),
        stage_index=0,
        instance_id="building:1",
        target_name="building",
        view="front",
        image=image,
        detection=_detection((20, 15, 60, 45)),
        observer_world=[1.0, 2.0, -8.0],
        observer_yaw_deg=35.0,
        source="active_observation",
    )

    saved = recorder.record_first(**kwargs)
    duplicate = recorder.record_first(**kwargs)

    assert recorder.run_directory == tmp_path / "output" / "20260825_1432"
    assert saved is not None and saved.exists()
    assert saved.name == "task_01_stage_01_building_building_1_front.jpg"
    assert duplicate is None
    with Image.open(saved) as boxed:
        red, green, blue = boxed.convert("RGB").getpixel((20, 45))
        assert red > 170 and red > green * 1.4 and red > blue * 1.4

    manifest_lines = (recorder.run_directory / "manifest.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(manifest_lines) == 1
    manifest = json.loads(manifest_lines[0])
    assert manifest["instance_id"] == "building:1"
    assert manifest["bbox"] == [20, 15, 60, 45]
    assert manifest["observer_world"] == [1.0, 2.0, -8.0]
    assert manifest["source"] == "active_observation"


def test_same_stage_and_instance_can_be_recorded_again_in_a_new_task(tmp_path):
    recorder = TargetSnapshotRecorder(
        enabled=True,
        output_root=tmp_path / "output",
        started_at=datetime(2026, 8, 25, 14, 32),
    )
    common = dict(
        stage_key=(0, "Fly to the building", "target"),
        stage_index=0,
        instance_id="building:1",
        target_name="building",
        view="front",
        image=Image.new("RGB", (64, 48), "white"),
        detection=_detection(),
    )

    recorder.begin_task("first task")
    first = recorder.record_first(**common)
    recorder.begin_task("second task")
    second = recorder.record_first(**common)

    assert first is not None and second is not None
    assert first.name.startswith("task_01_")
    assert second.name.startswith("task_02_")
    assert len(list(recorder.run_directory.glob("*.jpg"))) == 2


def test_invalid_bbox_is_skipped_without_consuming_first_snapshot(tmp_path):
    recorder = TargetSnapshotRecorder(enabled=True, output_root=tmp_path / "output")
    recorder.begin_task("task")
    common = dict(
        stage_key=(0, "task", "target"),
        stage_index=0,
        instance_id="car:1",
        target_name="car",
        view="front",
        image=Image.new("RGB", (64, 48), "white"),
    )

    assert recorder.record_first(detection=_detection((10, 10, 10, 20)), **common) is None
    assert recorder.record_first(detection=_detection((5, 5, 30, 30)), **common) is not None


def test_runtime_snapshot_uses_identity_approved_locked_candidate(tmp_path):
    recorder = TargetSnapshotRecorder(enabled=True, output_root=tmp_path / "output")
    recorder.begin_task("Fly to the building")
    stage = SimpleNamespace(
        index=0,
        instruction="Fly to the building",
        mode="target",
        target="building",
        target_query="building",
        relation="near",
    )
    wrong_high_score = _detection((5, 8, 28, 36), score=0.99)
    locked_candidate = _detection((45, 18, 85, 58), score=0.80)

    class LockedMemory:
        sim_config = {"FRONT_FOV": 90.0}

        @staticmethod
        def is_primary_locked(_stage):
            return True

        @staticmethod
        def primary_instance(_stage):
            return SimpleNamespace(instance_id="building:7")

        @staticmethod
        def previous_entity_exclusions(_stage, _world, _yaw):
            return []

        @staticmethod
        def evaluate_locked_detection_identity(_stage, detection, _image, **_kwargs):
            accepted = detection is locked_candidate
            return {"accepted": accepted, "reason": "match" if accepted else "wrong_instance"}

    image = Image.new("RGB", (100, 75), "white")
    bundle = DetectionDepthBundle(
        best_detection=wrong_high_score,
        front_detection=wrong_high_score,
        down_detection=None,
        front_detections=[wrong_high_score, locked_candidate],
        front_image=image,
        observer_world=[0.0, 0.0, -5.0],
        observer_yaw_deg=0.0,
    )
    objects = SimpleNamespace(
        target_snapshot_recorder=recorder,
        mission_memory=LockedMemory(),
    )

    _record_locked_target_snapshot(objects, stage, bundle, source="test")

    saved_files = list(recorder.run_directory.glob("*.jpg"))
    assert len(saved_files) == 1
    with Image.open(saved_files[0]) as boxed:
        boxed = boxed.convert("RGB")
        locked_pixel = boxed.getpixel((45, 18))
        wrong_pixel = boxed.getpixel((5, 8))
        assert locked_pixel[0] > 170 and locked_pixel[0] > locked_pixel[1] * 1.4
        assert min(wrong_pixel) > 210
