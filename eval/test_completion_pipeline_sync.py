"""Regression tests for timestamp-aligned completion observations."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
from PIL import Image

from agent.functions.fast_slow.completion_pipeline import CompletionPipeline
from agent.functions.fast_slow import completion_pipeline as pipeline_module


def test_completion_capture_uses_one_rgb_depth_pose_batch(monkeypatch):
    frame = Image.new("RGB", (16, 12), "white")
    down_frame = Image.new("RGB", (16, 12), "gray")
    front_depth = np.full((12, 16), 18.0, dtype=np.float32)
    down_depth = np.full((12, 16), 9.0, dtype=np.float32)
    calls = []

    def capture_once(_client, profile):
        calls.append(profile)
        return (
            frame,
            down_frame,
            front_depth,
            down_depth,
            {"total_s": 0.1},
            [4.0, 5.0, -8.0],
            15.0,
        )

    monkeypatch.setattr(pipeline_module.web_helpers, "capture_profile_isolated_with_pose", capture_once)
    monkeypatch.setattr(
        pipeline_module.web_helpers,
        "capture_profile_isolated",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("second depth capture is forbidden")),
    )
    pipeline = CompletionPipeline(
        detector=SimpleNamespace(),
        checker=SimpleNamespace(),
        client=SimpleNamespace(),
        capture_mode="batch",
        detect_executor=None,
        slow_executor=None,
        stop_radius_m=5.0,
        slow_radius_m=10.0,
    )
    pipeline._detect_dual_view = lambda *_args: (None, None, [], [], 0.2)

    bundle = pipeline._capture_detect_depth_bundle(
        SimpleNamespace(index=0, instruction="Fly above the white car", mode="target", target="white car"),
        "Fly above the white car",
    )

    assert calls == ["front_down_both_depth"]
    assert bundle.front_depth is front_depth
    assert bundle.down_depth is down_depth
    assert bundle.observer_world == [4.0, 5.0, -8.0]
    assert bundle.observer_yaw_deg == 15.0
