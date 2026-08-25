"""Regression tests for timestamped background depth observations."""

import math
import threading
from types import SimpleNamespace

from sim.frame_capturer import FrameCapturer


def _capturer_without_airsim() -> FrameCapturer:
    capturer = FrameCapturer.__new__(FrameCapturer)
    capturer._lock = threading.Lock()
    capturer.interval = 0.1
    capturer._latest_rgb = b"rgb"
    capturer._latest_depth = object()
    capturer._latest_rgb_png = b"png"
    capturer._latest_captured_at = 12.5
    capturer._latest_observer_world = (1.0, 2.0, -3.0)
    capturer._latest_observer_yaw_deg = 45.0
    capturer._capture_count = 4
    capturer._capture_period_ema_s = 0.2
    capturer._last_capture_duration_s = 0.05
    capturer._front_camera_offset = (1.0, 0.0, 0.0)
    return capturer


def test_timestamp_interface_exposes_the_same_atomic_depth_observation():
    capturer = _capturer_without_airsim()

    frame, depth, captured_at = capturer.get_latest_frame_with_timestamp()
    observation = capturer.get_latest_observation()

    assert frame == b"rgb"
    assert depth is capturer._latest_depth
    assert captured_at == 12.5
    assert observation["depth"] is depth
    assert observation["observer_world"] == (1.0, 2.0, -3.0)
    assert observation["observer_yaw_deg"] == 45.0
    assert capturer.get_capture_status()["capture_count"] == 4


def test_camera_pose_is_converted_back_to_vehicle_origin():
    capturer = _capturer_without_airsim()
    half_turn = math.radians(90.0) * 0.5
    response = SimpleNamespace(
        camera_position=SimpleNamespace(x_val=10.0, y_val=21.0, z_val=-3.0),
        camera_orientation=SimpleNamespace(
            x_val=0.0,
            y_val=0.0,
            z_val=math.sin(half_turn),
            w_val=math.cos(half_turn),
        ),
    )

    observer_world, yaw_deg = capturer._observer_pose_from_response(response)

    assert math.isclose(yaw_deg, 90.0, abs_tol=1e-9)
    assert all(
        math.isclose(actual, expected, abs_tol=1e-9)
        for actual, expected in zip(observer_world, (10.0, 20.0, -3.0))
    )
