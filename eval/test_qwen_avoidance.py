"""Focused tests for sliding-window suffix rewrites and Qwen parsing."""

import time
from types import SimpleNamespace

from agent.core.types import PlanOutput
from agent.functions.fast_slow.controller import FastSlowController
from agent.functions.fast_slow.runtime import (
    RealtimeDepthSafetyMonitor,
    _RuntimeLoopCleanup,
    _continuous_path_velocity,
    _depth_freshness_limit_s,
    _heuristic_fallback_allowed,
    _schedule_avoidance_retry,
    _sync_path_if_ready,
)
from agent.functions.obstacle_avoidance.local_depth_avoider import DepthObstacleAvoider
from agent.functions.trajectory_queue.world_queue import WorldTrajectoryQueue
from agent.functions.obstacle_avoidance.qwen_avoidance_planner import QwenAvoidancePlanner
from agent.functions.obstacle_avoidance.schemas import AvoidancePlan


def _wait_for_plan(controller, timeout_s=1.0):
    deadline = time.perf_counter() + timeout_s
    while not controller.has_plan_job or controller.planning:
        if time.perf_counter() >= deadline:
            raise AssertionError("planner future did not finish")
        time.sleep(0.005)


def test_route_splice_preserves_prefix_and_rejoins_old_tail():
    queue = WorldTrajectoryQueue(max_pending=5)
    queue.world_waypoints.extend([
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [3.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
    ])
    revision = queue.route_revision

    committed = queue.splice_world_waypoints(
        [[2.0, -2.0, 0.0], [4.0, -2.0, 0.0]],
        preserve_prefix_count=1,
        rejoin_index=3,
        base_revision=revision,
    )

    assert committed
    assert queue.route_revision == revision + 1
    assert queue.world_waypoints == [
        [1.0, 0.0, 0.0],
        [2.0, -2.0, 0.0],
        [4.0, -2.0, 0.0],
        [4.0, 0.0, 0.0],
    ]


def test_dynamic_stop_buffer_accounts_for_depth_age():
    avoider = DepthObstacleAvoider({
        "DYNAMIC_STOP_ENABLED": True,
        "STOP_BUFFER_M": 0.0,
        "REACTION_TIME_S": 0.4,
        "BRAKING_DECEL_MPS2": 2.5,
        "EXTRA_STOP_MARGIN_M": 0.5,
        "MAX_ACCOUNTED_DEPTH_AGE_S": 1.0,
    })

    fresh = avoider.dynamic_stop_buffer_m(2.0, depth_age_s=0.0)
    stale = avoider.dynamic_stop_buffer_m(2.0, depth_age_s=0.75)

    assert stale > fresh
    assert abs((stale - fresh) - 1.5) < 1e-6


def test_route_splice_rejects_late_result_from_old_revision():
    queue = WorldTrajectoryQueue(max_pending=5)
    queue.world_waypoints.extend([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    old_revision = queue.route_revision
    assert queue.replace_world_waypoints([[0.0, 2.0, 0.0]], base_revision=old_revision)

    assert not queue.splice_world_waypoints(
        [[9.0, 9.0, 9.0]],
        base_revision=old_revision,
    )
    assert queue.world_waypoints == [[0.0, 2.0, 0.0]]


def test_normal_prefix_consumption_keeps_inflight_sliding_plan_valid():
    queue = WorldTrajectoryQueue(max_pending=5)
    queue.world_waypoints.extend([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    controller = FastSlowController({"MAX_PENDING": 5}, queue=queue)
    try:
        assert controller.maybe_submit_plan(
            current_pos=[0.0, 0.0, 0.0],
            current_yaw_deg=0.0,
            plan_fn=lambda _pending: PlanOutput(
                waypoints=[[1.0, 0.0, 0.0]],
                waypoint_format="incremental_body",
            ),
        )
        _wait_for_plan(controller)
        controller.mark_executed(1)

        result = controller.poll_plan()

        assert result is not None
        assert queue.world_waypoints == [[2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
    finally:
        controller.shutdown()


def test_structural_splice_discards_inflight_global_plan():
    queue = WorldTrajectoryQueue(max_pending=5)
    queue.world_waypoints.extend([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    controller = FastSlowController({"MAX_PENDING": 5}, queue=queue)
    try:
        assert controller.maybe_submit_plan(
            current_pos=[0.0, 0.0, 0.0],
            current_yaw_deg=0.0,
            plan_fn=lambda _pending: PlanOutput(
                waypoints=[[5.0, 0.0, 0.0]],
                waypoint_format="incremental_body",
            ),
        )
        _wait_for_plan(controller)
        assert queue.splice_world_waypoints(
            [[1.0, -2.0, 0.0]],
            preserve_prefix_count=0,
            rejoin_index=1,
        )

        assert controller.poll_plan() is None
        assert queue.world_waypoints == [[1.0, -2.0, 0.0], [2.0, 0.0, 0.0]]
    finally:
        controller.shutdown()


def test_qwen_avoidance_parser_accepts_json_and_bare_list():
    planner = QwenAvoidancePlanner({"URL": "http://example.invalid", "MAX_WAYPOINTS": 5})

    structured = planner._normalize_plan(
        '```json\n{"action":"detour","waypoints":[[1,2,0],[3,2,0]],"rejoin":true}\n```',
        elapsed_s=0.1,
    )
    bare = planner._normalize_plan("[[0, 2, 0], [2, 2, 0]]", elapsed_s=0.1)

    assert structured.action == "detour"
    assert structured.waypoints == [[1.0, 2.0, 0.0], [3.0, 2.0, 0.0]]
    assert bare.action == "detour"
    assert bare.waypoints == [[0.0, 2.0, 0.0], [2.0, 2.0, 0.0]]


def test_qwen_avoidance_parser_fails_closed():
    planner = QwenAvoidancePlanner({"URL": "http://example.invalid"})

    result = planner._normalize_plan("fly left around the tree", elapsed_s=0.1)

    assert result.action == "hold"
    assert result.waypoints == []
    assert result.error


def test_qwen_hold_discards_contradictory_waypoints():
    planner = QwenAvoidancePlanner({"URL": "http://example.invalid"})

    result = planner._normalize_plan(
        '{"action":"hold","waypoints":[[2,3,0]],"confidence":0.9}',
        elapsed_s=0.1,
    )

    assert result.action == "hold"
    assert result.waypoints == []


def test_qwen_hold_can_never_commit_mechanical_fallback():
    job = type("Job", (), {
        "fallback_world": [[1.0, 2.0, 0.0]],
        "fallback_reason": "depth_lateral_bypass",
    })()

    assert not _heuristic_fallback_allowed(
        AvoidancePlan(action="hold"),
        job,
        {"QWEN_AVOIDANCE_FALLBACK_TO_HEURISTIC": True},
    )


def test_qwen_exhausted_view_scans_and_retains_mission():
    class Client:
        def __init__(self):
            self.rotations = []

        def get_pose(self):
            return [0.0, 0.0, 0.0], 60.0

        def rotate_to_yaw(self, yaw, timeout=0.0):
            self.rotations.append((yaw, timeout))

    class State:
        def __init__(self):
            self.values = {}

        def update(self, **kwargs):
            self.values.update(kwargs)

    objects = SimpleNamespace(
        obstacle_avoider=SimpleNamespace(config={
            "QWEN_AVOIDANCE_MAX_RETRIES": 2,
            "QWEN_AVOIDANCE_CONTINUE_UNTIL_SAFE": True,
            "QWEN_AVOIDANCE_SCAN_ENABLED": True,
            "QWEN_AVOIDANCE_SCAN_YAW_OFFSETS_DEG": [-45.0, 45.0],
            "QWEN_AVOIDANCE_SCAN_SETTLE_S": 0.5,
        }),
        controller=SimpleNamespace(queue=SimpleNamespace(route_revision=0)),
        avoidance_attempt=2,
        avoidance_job=None,
        avoidance_reference_world=[],
        avoidance_safe_prefix_world=[],
        avoidance_preserve_count=0,
        avoidance_base_revision=0,
        avoidance_stage_key=None,
        avoidance_started_s=0.0,
        avoidance_retry_after_s=0.0,
        avoidance_hold=False,
        avoidance_last_error="",
        avoidance_recovery_active=True,
        avoidance_trigger_result=None,
        avoidance_scan_anchor_yaw_deg=None,
        avoidance_scan_index=0,
        avoidance_scan_cycles=0,
    )
    client = Client()
    state = State()

    _schedule_avoidance_retry(
        objects,
        client,
        path_stream=None,
        state=state,
        plan=AvoidancePlan(action="hold"),
        error="no safe route",
    )

    assert client.rotations == [(15.0, 6.0)]
    assert not objects.avoidance_hold
    assert objects.avoidance_recovery_active
    assert objects.avoidance_retry_after_s > time.perf_counter()
    assert state.values["avoidance_state"] == "scan_wait"
    assert state.values["safety_hold"] is True


def test_independent_monitor_brakes_while_main_loop_is_blocked():
    class Capturer:
        def get_latest_frame_with_timestamp(self):
            import numpy as np

            return None, np.full((32, 32), 2.5, dtype=np.float32), time.perf_counter()

    class Client:
        def get_pose(self):
            return [0.0, 0.0, 0.0], 0.0

        def get_speed_mps(self):
            return 2.0

    class PathStream:
        def __init__(self):
            self.active = True
            self.remaining_waypoints = [[10.0, 0.0, 0.0]]
            self.stop_count = 0

        def emergency_stop(self):
            self.stop_count += 1
            self.active = False
            self.remaining_waypoints = []

    class State:
        def update(self, **_kwargs):
            pass

    avoider = DepthObstacleAvoider({
        "ENABLED": True,
        "RAW_DEPTH_EMERGENCY_ENABLED": True,
        "RAW_DEPTH_CORRIDOR_MIN_SAMPLES": 4,
        "RAW_DEPTH_EMERGENCY_MARGIN_M": 0.8,
        "DYNAMIC_STOP_ENABLED": True,
        "STOP_BUFFER_M": 2.2,
        "REACTION_TIME_S": 0.45,
        "BRAKING_DECEL_MPS2": 2.5,
        "EXTRA_STOP_MARGIN_M": 0.8,
        "REALTIME_MONITOR_INTERVAL_S": 0.02,
    })
    path_stream = PathStream()
    objects = SimpleNamespace(
        obstacle_avoider=avoider,
        controller=SimpleNamespace(queue=SimpleNamespace(world_waypoints=[])),
    )
    monitor = RealtimeDepthSafetyMonitor(
        objects,
        Client(),
        path_stream,
        Capturer(),
        State(),
    )
    monitor.start()
    deadline = time.perf_counter() + 1.0
    while not monitor.blocked and time.perf_counter() < deadline:
        time.sleep(0.01)
    monitor.stop()

    assert monitor.blocked
    assert path_stream.stop_count == 1
    assert monitor.snapshot()["reason"] == "raw_depth_corridor"


def test_stale_depth_does_not_cancel_a_continuous_path_by_default():
    class Capturer:
        def get_latest_frame_with_timestamp(self):
            return None, None, time.perf_counter() - 2.0

    class PathStream:
        active = True

        def __init__(self):
            self.stop_count = 0

        def emergency_stop(self):
            self.stop_count += 1

    class State:
        def update(self, **_kwargs):
            pass

    path_stream = PathStream()
    monitor = RealtimeDepthSafetyMonitor(
        SimpleNamespace(obstacle_avoider=SimpleNamespace(config={
            "REALTIME_DEPTH_MAX_AGE_S": 0.1,
            "REALTIME_STALE_BRAKE_ENABLED": False,
        })),
        SimpleNamespace(),
        path_stream,
        Capturer(),
        State(),
    )

    monitor._check_once()
    monitor._check_once()

    assert not monitor.blocked
    assert path_stream.stop_count == 0


def test_monitor_requires_a_new_depth_frame_to_release_brake():
    class PathStream:
        def __init__(self):
            self.stop_count = 0

        def emergency_stop(self):
            self.stop_count += 1

    class State:
        def update(self, **_kwargs):
            pass

    path_stream = PathStream()
    monitor = RealtimeDepthSafetyMonitor(
        SimpleNamespace(),
        SimpleNamespace(),
        path_stream,
        SimpleNamespace(),
        State(),
    )

    monitor._trigger("depth_stale", captured_at=10.0)
    monitor._trigger("depth_stale", captured_at=10.0)

    assert monitor.blocked
    assert path_stream.stop_count == 1
    assert monitor.clear(captured_at=10.0) is False
    assert monitor.blocked
    assert monitor.clear(captured_at=10.1) is True
    assert not monitor.blocked


def test_monitor_debounces_unsafe_and_safe_depth_frames():
    class PathStream:
        def __init__(self):
            self.stop_count = 0

        def emergency_stop(self):
            self.stop_count += 1

    class State:
        def update(self, **_kwargs):
            pass

    path_stream = PathStream()
    objects = SimpleNamespace(obstacle_avoider=SimpleNamespace(config={
        "REALTIME_UNSAFE_CONFIRMATIONS": 2,
        "REALTIME_SAFE_CONFIRMATIONS": 3,
    }))
    monitor = RealtimeDepthSafetyMonitor(
        objects,
        SimpleNamespace(),
        path_stream,
        SimpleNamespace(),
        State(),
    )

    monitor._observe_unsafe(
        "raw_depth_corridor",
        captured_at=1.0,
        age_s=0.1,
        clearance_m=2.0,
        required_m=3.0,
    )
    assert not monitor.blocked
    monitor._observe_unsafe(
        "raw_depth_corridor",
        captured_at=2.0,
        age_s=0.1,
        clearance_m=2.0,
        required_m=3.0,
    )
    assert monitor.blocked
    assert path_stream.stop_count == 1

    assert not monitor._observe_safe(captured_at=3.0, age_s=0.1, clearance_m=5.0, required_m=3.75)
    assert not monitor._observe_safe(captured_at=4.0, age_s=0.1, clearance_m=5.0, required_m=3.75)
    assert monitor._observe_safe(captured_at=5.0, age_s=0.1, clearance_m=5.0, required_m=3.75)
    assert not monitor.blocked
    assert path_stream.stop_count == 1


def test_planning_velocity_is_stable_for_one_background_job():
    job = SimpleNamespace()
    controller = SimpleNamespace(
        planning=True,
        _job=job,
        reserve_time_s=4.5,
        queue=SimpleNamespace(world_waypoints=[[6.0, 0.0, 0.0]]),
    )
    objects = SimpleNamespace(controller=controller)

    first = _continuous_path_velocity(objects, [0.0, 0.0, 0.0])
    later = _continuous_path_velocity(objects, [3.0, 0.0, 0.0])

    assert first == later
    assert 1.0 < first < 2.0


def test_runtime_cleanup_stops_monitor_before_path_and_is_idempotent():
    events = []

    class Monitor:
        def stop(self):
            events.append("monitor")

    class Controller:
        def shutdown(self, *, wait):
            assert wait
            events.append("controller")

    class Executor:
        def shutdown(self, *, wait, cancel_futures):
            assert wait and cancel_futures
            events.append("executor")

    class PathStream:
        def stop(self):
            events.append("path")

    cleanup = _RuntimeLoopCleanup()
    cleanup.bind(
        objects=SimpleNamespace(
            realtime_safety_monitor=Monitor(),
            controller=Controller(),
            detect_executor=Executor(),
        ),
        path_stream=PathStream(),
    )

    cleanup.close()
    cleanup.close()

    assert events == ["monitor", "controller", "executor", "path"]


def test_depth_freshness_limit_adapts_to_capture_period_but_keeps_hard_cap():
    class Capturer:
        def get_capture_status(self):
            return {
                "capture_period_ema_s": 1.0,
                "last_capture_duration_s": 0.4,
                "interval_s": 0.1,
            }

    limit_s = _depth_freshness_limit_s(
        {
            "REALTIME_DEPTH_MAX_AGE_S": 0.75,
            "REALTIME_DEPTH_ADAPTIVE_MAX_AGE_S": 1.5,
            "REALTIME_DEPTH_PERIOD_FACTOR": 2.0,
            "REALTIME_DEPTH_AGE_GRACE_S": 0.2,
        },
        Capturer(),
    )

    assert limit_s == 1.5


def test_path_start_waits_for_fresh_depth_before_issuing_command():
    import numpy as np

    class Capturer:
        def __init__(self):
            self.captured_at = time.perf_counter() - 10.0

        def get_latest_frame_with_timestamp(self):
            return None, np.full((8, 8), 20.0, dtype=np.float32), self.captured_at

    class Client:
        def get_pose(self):
            return [0.0, 0.0, 0.0], 0.0

    class PathStream:
        def __init__(self):
            self.active = False
            self.sync_count = 0

        @staticmethod
        def poll(_queue_waypoints, _current_pos):
            return SimpleNamespace(consumed=0, collided=False)

        def sync(self, _waypoints, _current_pos, _velocity):
            self.sync_count += 1
            self.active = True
            return True

    class State:
        def update(self, **_kwargs):
            pass

    capturer = Capturer()
    path_stream = PathStream()
    objects = SimpleNamespace(
        realtime_safety_monitor=None,
        avoidance_recovery_active=False,
        avoidance_job=None,
        avoidance_hold=False,
        avoidance_retry_after_s=0.0,
        mission_memory=None,
        obstacle_avoider=SimpleNamespace(config={
            "REALTIME_DEPTH_MAX_AGE_S": 0.75,
            "REQUIRE_FRESH_DEPTH_BEFORE_PATH_START": True,
        }),
        controller=SimpleNamespace(
            planning=False,
            queue=SimpleNamespace(world_waypoints=[[10.0, 0.0, 0.0]]),
        ),
        path_start_depth_wait_capture_at=-1.0,
    )

    # Continuous mode preserves the original sliding-window behavior unless
    # a deployment explicitly opts into the fresh-frame start gate.
    objects.obstacle_avoider.config["REQUIRE_FRESH_DEPTH_BEFORE_PATH_START"] = False
    _sync_path_if_ready(
        objects,
        path_stream,
        Client(),
        State(),
        capturer=capturer,
    )
    assert path_stream.sync_count == 1

    path_stream.active = False
    path_stream.sync_count = 0
    objects.obstacle_avoider.config["REQUIRE_FRESH_DEPTH_BEFORE_PATH_START"] = True
    _sync_path_if_ready(
        objects,
        path_stream,
        Client(),
        State(),
        capturer=capturer,
    )
    assert path_stream.sync_count == 0
    assert objects.safety_depth_hold

    capturer.captured_at = time.perf_counter()
    _sync_path_if_ready(
        objects,
        path_stream,
        Client(),
        State(),
        capturer=capturer,
    )
    assert path_stream.sync_count == 1


def test_blocked_reference_route_cannot_be_reissued_during_recovery():
    class Client:
        def get_pose(self):
            raise AssertionError("blocked recovery must not reach path issue logic")

    objects = SimpleNamespace(
        realtime_safety_monitor=None,
        avoidance_recovery_active=True,
    )

    consumed = _sync_path_if_ready(
        objects,
        path_stream=SimpleNamespace(),
        client=Client(),
        state=SimpleNamespace(),
    )

    assert consumed == 0


def test_qwen_avoidance_parser_rejects_nonfinite_and_nonlocal_points():
    planner = QwenAvoidancePlanner({
        "URL": "http://example.invalid",
        "MAX_COORDINATE_M": 20.0,
    })

    result = planner._normalize_plan(
        '{"action":"detour","waypoints":[[NaN,0,0],[1000,0,0]]}',
        elapsed_s=0.1,
    )

    assert result.action == "hold"
    assert result.waypoints == []
