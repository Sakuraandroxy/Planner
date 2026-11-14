import csv
import json

import numpy as np
import pytest

from planner.adapters.diagnostics import VisualGoalTrace
from planner.domain.depth_foreground import DepthForegroundResult
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import WorldPose
from planner.domain.streaming import ControlTelemetry
from planner.domain.visual_goal import (
    GoalGeometry,
    GoalObservation,
    GoalVisibility,
    SurfacePointEstimate,
)


def _observation() -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(8, 6, 90.0)
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=WorldPose(1.0, 2.0, 0.0, 30.0),
        rgb=np.arange(6 * 8 * 3, dtype=np.uint8).reshape(6, 8, 3),
        depth_m=np.full((6, 8), 10.0, dtype=np.float32),
        depth_image=np.full((6, 8), 128, dtype=np.uint8),
        intrinsics=intrinsics,
        camera_matrix=np.eye(4),
        capture_id="capture-1",
        captured_monotonic=1.0,
        timestamp_ns=10,
        stopped=True,
    )


def _goal() -> GoalObservation:
    return GoalObservation(
        GoalVisibility.VISIBLE, (0.25, 0.25, 0.75, 0.75), None,
        "the red car", "visible", True, "model_surface_points",
        surface_points=((0.5, 0.5),),
    )


def _events(trace: VisualGoalTrace) -> list[dict]:
    return [
        json.loads(line)
        for line in (trace.path / "events.jsonl").read_text().splitlines()
    ]


def test_trace_creates_a_unique_directory(tmp_path):
    first = VisualGoalTrace(tmp_path)
    second = VisualGoalTrace(tmp_path)
    assert first.path.is_dir()
    assert first.path != second.path
    assert first.path.parent == tmp_path


def test_event_serialises_dataclasses_enums_and_numpy(tmp_path):
    trace = VisualGoalTrace(tmp_path)
    trace.event(
        "task_start",
        pose=WorldPose(1.0, 2.0, 0.0, 30.0),
        visibility=GoalVisibility.VISIBLE,
        rotation=np.eye(3),
        scalar=np.float32(1.5),
    )
    entry = _events(trace)[0]
    assert entry["event"] == "task_start"
    assert entry["pose"] == {"x": 1.0, "y": 2.0, "z": 0.0, "yaw_deg": 30.0}
    assert entry["visibility"] == "visible"
    assert entry["rotation"] == [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    assert entry["scalar"] == pytest.approx(1.5)
    assert "logged_wall_time" in entry


def test_observation_writes_images_depth_and_event(tmp_path):
    trace = VisualGoalTrace(tmp_path)
    trace.observation(_observation(), _goal())
    assert (trace.path / "0001_rgb.png").is_file()
    assert (trace.path / "0001_depth.png").is_file()
    assert (trace.path / "0001_depth_meters.npy").is_file()
    assert (trace.path / "0001_target.png").is_file()
    saved = np.load(trace.path / "0001_depth_meters.npy")
    assert saved.shape == (6, 8)
    event = _events(trace)[0]
    assert event["event"] == "observation"
    assert event["capture_id"] == "capture-1"
    assert event["pose"]["x"] == pytest.approx(1.0)
    assert event["goal"]["surface_points"] == [[0.5, 0.5]]


def test_target_geometry_draws_samples_and_reports_depth_filter(tmp_path):
    trace = VisualGoalTrace(tmp_path)
    trace.counter = 3
    geometry = GoalGeometry(
        True, "sampled", 5.0, (5.0, 0.0, 0.0),
        surface_samples=(
            SurfacePointEstimate((0.5, 0.5), True, "sampled", 5.0, (5.0, 0.0, 0.0), 5.0),
            SurfacePointEstimate((0.2, 0.2), False, "filtered_background", 12.0),
        ),
        depth_filter=DepthForegroundResult(True, "local_layers", 1, ()),
    )
    trace.target_geometry(_observation(), _goal(), geometry)
    assert (trace.path / "0003_target_geometry.png").is_file()
    event = _events(trace)[0]
    assert event["event"] == "surface_depth_samples"
    assert event["selected_world_point"] == [5.0, 0.0, 0.0]
    assert event["depth_filter"]["reason"] == "local_layers"


def test_planning_observation_writes_rgb_and_aligned_depth(tmp_path):
    trace = VisualGoalTrace(tmp_path)
    trace.planning_observation(7, _observation())
    assert (trace.path / "plan_0007_rgb.jpg").is_file()
    assert (trace.path / "plan_0007_depth.png").is_file()
    event = _events(trace)[0]
    assert event["event"] == "planning_images"
    assert event["request_id"] == 7


def test_control_samples_expand_vectors_and_append(tmp_path):
    trace = VisualGoalTrace(tmp_path)
    sample = ControlTelemetry(
        timestamp_ns=1, observed_monotonic=0.1, dt_s=0.05, waypoint_id=2,
        queue_length=3, commanded_velocity=(0.3, 0.4, 0.0),
        measured_velocity=(0.3, 0.4, 0.0), commanded_acceleration=(0.0, 0.0, 0.0),
        yaw_rate_deg_s=0.0, yaw_acceleration_deg_s2=0.0, yaw_error_deg=0.0,
        position_error_m=0.0, aligning=False, pitch_deg=None, roll_deg=None,
    )
    trace.control_samples((sample,), dropped=2)
    with (trace.path / "control_samples.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert rows[0]["commanded_velocity_x"] == "0.3"
    assert float(rows[0]["commanded_velocity_norm"]) == pytest.approx(0.5)
    event = _events(trace)[0]
    assert event == {
        "event": "control_telemetry",
        "logged_wall_time": event["logged_wall_time"],
        "file": "control_samples.csv",
        "samples": 1,
        "dropped": 2,
    }