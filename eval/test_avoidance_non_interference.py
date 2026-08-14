"""Regression tests for the obstacle layer's NORMAL-state identity contract."""

import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from agent.functions.fast_slow import runtime as runtime_module
from agent.functions.fast_slow.path_stream import ContinuousPathStream
from agent.functions.fast_slow.runtime import (
    RealtimeDepthSafetyMonitor,
    _backtrack_avoidance_route,
    _select_avoidance_scan,
)
from agent.functions.obstacle_avoidance.local_depth_avoider import DepthObstacleAvoider


class _State:
    def update(self, **_kwargs):
        pass


def test_safe_fresh_depth_never_stops_or_mutates_the_route():
    class Capturer:
        def get_latest_frame_with_timestamp(self):
            return None, np.full((32, 32), 20.0, dtype=np.float32), time.perf_counter()

    class Client:
        @staticmethod
        def get_speed_mps():
            return 2.0

        @staticmethod
        def get_pose():
            return [0.0, 0.0, 0.0], 0.0

    class PathStream:
        active = True

        def __init__(self):
            self.stop_count = 0

        def emergency_stop(self):
            self.stop_count += 1

    queue = [[5.0, 0.0, 0.0], [10.0, 0.0, 0.0]]
    avoider = DepthObstacleAvoider({
        "ENABLED": True,
        "EMERGENCY_BRAKE_DISTANCE_M": 3.0,
        "RAW_DEPTH_CORRIDOR_MIN_SAMPLES": 4,
        "REALTIME_UNSAFE_CONFIRMATIONS": 2,
    })
    objects = SimpleNamespace(
        obstacle_avoider=avoider,
        controller=SimpleNamespace(queue=SimpleNamespace(world_waypoints=queue)),
        avoidance_hold=False,
        avoidance_recovery_active=False,
    )
    stream = PathStream()
    monitor = RealtimeDepthSafetyMonitor(objects, Client(), stream, Capturer(), _State())

    monitor._check_once()
    monitor._check_once()

    assert not monitor.blocked
    assert stream.stop_count == 0
    assert objects.controller.queue.world_waypoints == queue
    assert not objects.avoidance_hold
    assert not objects.avoidance_recovery_active


def test_normal_path_stream_uses_the_original_command_signature():
    class Client:
        def __init__(self):
            self.commands = []

        def start_waypoint_path(self, waypoints, velocity=None, **options):
            self.commands.append((waypoints, velocity, options))

        @staticmethod
        def collision_marker():
            return (False, 0, 0)

        @staticmethod
        def has_collision_since(_marker):
            return False

        @staticmethod
        def stop_waypoint_path():
            pass

    client = Client()
    stream = ContinuousPathStream(client)
    route = [[5.0, 0.0, 0.0], [10.0, 0.0, 0.0]]

    assert stream.sync(route, [0.0, 0.0, 0.0], 2.0)

    assert client.commands == [(route, 2.0, {})]


def test_default_emergency_brake_depth_is_three_meters():
    with open("config/default.yaml", "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    obstacle = config["FUNCTIONS"]["OBSTACLE_AVOIDANCE"]

    assert obstacle["EMERGENCY_BRAKE_DISTANCE_M"] == 3.0


def test_open_range_depth_is_clear_but_invalid_depth_stays_unknown():
    avoider = DepthObstacleAvoider({
        "FRONT_MAX_DEPTH_M": 28.0,
        "RAW_DEPTH_CORRIDOR_MIN_SAMPLES": 4,
    })

    assert avoider.front_corridor_clearance_m(
        np.full((16, 16), np.inf, dtype=np.float32)
    ) == 28.0
    assert avoider.front_corridor_clearance_m(
        np.zeros((16, 16), dtype=np.float32)
    ) is None


def test_retry_scan_forces_the_opposite_untried_side():
    scans = [
        {"offset_deg": -60.0, "clearance_m": 12.0, "score": 11.4},
        {"offset_deg": 60.0, "clearance_m": 8.0, "score": 7.4},
    ]

    first = _select_avoidance_scan(scans, 3.8)
    second = _select_avoidance_scan(scans, 3.8, tried_sides=["left"])

    assert first["offset_deg"] == -60.0
    assert second["offset_deg"] == 60.0


def test_failed_side_backtracks_over_completed_route(monkeypatch):
    class Client:
        def __init__(self):
            self.position = [1.0, -6.0, 0.0]

        def get_pose(self):
            return list(self.position), 0.0

    client = Client()
    flown = []

    def capture(_objects, _client, _capturer, yaw_deg=None):
        return {"captured_at": time.perf_counter(), "clearance_m": 30.0, "yaw": yaw_deg}

    def fly(_objects, _client, _stream, _state, _monitor, waypoints, **_kwargs):
        target = list(waypoints[-1])
        flown.append(target)
        client.position = target
        return True

    monkeypatch.setattr(runtime_module, "_capture_avoidance_depth", capture)
    monkeypatch.setattr(runtime_module, "_fly_avoidance_path", fly)
    objects = SimpleNamespace(
        obstacle_avoider=SimpleNamespace(
            config={"SCAN_MOVE_MARGIN_M": 0.8, "FRONT_MAX_DEPTH_M": 28.0}
        ),
        avoidance_origin_world=[0.0, 0.0, 0.0],
        avoidance_last_error="",
    )

    assert _backtrack_avoidance_route(
        objects,
        client,
        object(),
        None,
        _State(),
        None,
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, -4.0, 0.0]],
        original_yaw_deg=0.0,
    )
    assert flown == [
        [1.0, -4.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]


def test_runtime_cleanup_runs_when_loop_raises(monkeypatch):
    events = []

    class Cleanup:
        def close(self):
            events.append("closed")

    def fail(*_args, **_kwargs):
        raise RuntimeError("boom")

    monkeypatch.setattr(runtime_module, "_RuntimeLoopCleanup", Cleanup)
    monkeypatch.setattr(runtime_module, "_run_fast_slow_loop_impl", fail)

    with pytest.raises(RuntimeError, match="boom"):
        runtime_module.run_fast_slow_loop(_State(), "task", 1, object())

    assert events == ["closed"]
