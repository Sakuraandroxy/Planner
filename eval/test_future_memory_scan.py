"""Focused tests for non-blocking opportunistic MissionMemory scans."""

from __future__ import annotations

import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from types import SimpleNamespace

import numpy as np
from PIL import Image

from agent.functions.fast_slow.runtime import (
    FutureMemoryObservation,
    FutureMemoryScanJob,
    MemoryScanSnapshot,
    _maybe_submit_future_memory_scan,
    _poll_future_memory_scan,
)


def _stage(index: int, target: str):
    return SimpleNamespace(
        index=index,
        instruction=f"Fly to the {target}",
        mode="target",
        target=target,
        target_query=target,
        relation="near",
        auxiliary_targets=[],
    )


def _snapshot():
    return MemoryScanSnapshot(
        frame=Image.new("RGB", (32, 24), "white"),
        down_frame=Image.new("RGB", (32, 24), "gray"),
        front_depth=np.full((24, 32), 12.0, dtype=np.float32),
        down_depth=np.full((24, 32), 8.0, dtype=np.float32),
        observer_world=(1.0, 2.0, -3.0),
        observer_yaw_deg=15.0,
    )


def test_future_scan_submission_returns_without_waiting_and_applies_backpressure():
    gate = threading.Event()

    class SlowDetector:
        def detect_all(self, image, caption, depth_meters=None, camera_name="front"):
            gate.wait(timeout=2.0)
            return []

    current = _stage(0, "white car")
    future_stage = _stage(1, "red car")
    memory = SimpleNamespace(
        enabled=True,
        root_instruction="long task",
        config={
            "OPPORTUNISTIC_SCAN_ENABLED": True,
            "OPPORTUNISTIC_SCAN_INTERVAL_S": 0.0,
            "OPPORTUNISTIC_TARGETS_PER_SCAN": 1,
        },
    )
    scan_executor = ThreadPoolExecutor(max_workers=1)
    detect_executor = ThreadPoolExecutor(max_workers=1)
    objects = SimpleNamespace(
        mission_memory=memory,
        task_manager=SimpleNamespace(stages=[current, future_stage]),
        detector=SlowDetector(),
        future_scan_executor=scan_executor,
        future_detect_executor=detect_executor,
        future_scan_job=None,
        future_scan_index=0,
        last_future_scan_s=0.0,
    )
    try:
        started = time.perf_counter()
        assert _maybe_submit_future_memory_scan(objects, current, "long task", _snapshot())
        assert time.perf_counter() - started < 0.5
        assert objects.future_scan_job is not None
        assert not _maybe_submit_future_memory_scan(objects, current, "long task", _snapshot())
    finally:
        gate.set()
        if objects.future_scan_job is not None:
            objects.future_scan_job.future.result(timeout=3.0)
        scan_executor.shutdown(wait=True, cancel_futures=True)
        detect_executor.shutdown(wait=True, cancel_futures=True)


def test_future_scan_result_is_committed_on_main_poll_with_snapshot_pose():
    query_stage = _stage(2, "red car")
    current_stage = _stage(1, "white car")
    snapshot = _snapshot()
    completed = Future()
    completed.set_result(
        [
            FutureMemoryObservation(
                stage=query_stage,
                front_detections=[],
                down_detections=[],
                detect_elapsed=0.4,
            )
        ]
    )

    calls = []

    class RecordingMemory:
        enabled = True
        root_instruction = "long task"
        config = {}

        def update_from_detections(self, **kwargs):
            calls.append(kwargs)
            return []

        def summary(self, stage=None):
            return {"enabled": True}

    memory = RecordingMemory()
    objects = SimpleNamespace(
        mission_memory=memory,
        future_scan_job=FutureMemoryScanJob(
            future=completed,
            snapshot=snapshot,
            root_instruction="long task",
            source_stage_key=(0, "Fly to the white car", "target"),
            target_names=("red car",),
            submitted_at=time.perf_counter(),
        ),
    )
    state = SimpleNamespace(update=lambda **kwargs: None)

    _poll_future_memory_scan(objects, state, current_stage)

    assert objects.future_scan_job is None
    assert len(calls) == 1
    assert calls[0]["observer_world"] == snapshot.observer_world
    assert calls[0]["observer_yaw_deg"] == snapshot.observer_yaw_deg


def test_future_scan_discards_results_for_already_completed_stages():
    old_stage = _stage(1, "red car")
    current_stage = _stage(2, "house")
    completed = Future()
    completed.set_result(
        [FutureMemoryObservation(old_stage, [], [], 0.1)]
    )

    calls = []
    memory = SimpleNamespace(
        root_instruction="long task",
        config={},
        update_from_detections=lambda **kwargs: calls.append(kwargs) or [],
        summary=lambda stage=None: {},
    )
    objects = SimpleNamespace(
        mission_memory=memory,
        future_scan_job=FutureMemoryScanJob(
            future=completed,
            snapshot=_snapshot(),
            root_instruction="long task",
            source_stage_key=(0, "", "target"),
            target_names=("red car",),
            submitted_at=time.perf_counter(),
        ),
    )

    _poll_future_memory_scan(objects, SimpleNamespace(update=lambda **kwargs: None), current_stage)

    assert calls == []
