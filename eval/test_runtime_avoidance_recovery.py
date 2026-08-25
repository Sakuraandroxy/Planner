"""Regression tests for recovery-command de-duplication."""

import time
from types import SimpleNamespace

from agent.functions.fast_slow.runtime import (
    _arm_preplan_avoidance,
    _commit_local_avoidance_route,
    _service_avoidance_recovery,
)


def test_hard_retreat_is_not_rechecked_as_a_forward_hazard():
    calls = []

    class Avoider:
        enabled = True
        config = {"ACTIVE_ROUTE_CHECK_INTERVAL_S": 0.0}

        def filter_cumulative_waypoints(self, *_args, **_kwargs):
            calls.append(True)
            raise AssertionError("retreat route must not be filtered by forward depth")

    objects = SimpleNamespace(
        avoidance_mode="hard_retreat",
        obstacle_avoider=Avoider(),
        controller=SimpleNamespace(
            queue=SimpleNamespace(world_waypoints=[[1.0, 0.0, 0.0]])
        ),
    )

    assert _service_avoidance_recovery(
        objects,
        client=object(),
        path_stream=object(),
        capturer=object(),
        state=object(),
        stage=None,
    )
    assert calls == []


def test_soft_stop_prefix_is_not_reissued_before_reaching_stop_point():
    calls = []

    class Avoider:
        enabled = True
        config = {"ACTIVE_ROUTE_CHECK_INTERVAL_S": 0.0}

        def filter_cumulative_waypoints(self, *_args, **_kwargs):
            calls.append(True)
            raise AssertionError("soft-stop prefix must be allowed to finish")

    objects = SimpleNamespace(
        avoidance_mode="soft_stop",
        obstacle_avoider=Avoider(),
        controller=SimpleNamespace(
            queue=SimpleNamespace(world_waypoints=[[1.0, 0.0, 0.0]])
        ),
    )

    assert _service_avoidance_recovery(
        objects,
        client=object(),
        path_stream=object(),
        capturer=object(),
        state=object(),
        stage=None,
    )
    assert calls == []


def test_identical_recovery_route_is_not_reissued_immediately():
    class Queue:
        route_revision = 0

        def replace_world_waypoints(self, route, *, base_revision=None):
            if base_revision is not None and base_revision != self.route_revision:
                return False
            self.route_revision += 1
            self.world_waypoints = [list(point) for point in route]
            return True

    queue = Queue()
    queue.world_waypoints = []
    objects = SimpleNamespace(
        controller=SimpleNamespace(
            queue=queue,
            discard_plan=lambda: None,
        ),
        obstacle_avoider=SimpleNamespace(
            config={"AVOIDANCE_REISSUE_MIN_INTERVAL_S": 10.0}
        ),
        avoidance_mode="normal",
    )
    state = SimpleNamespace(update=lambda **_values: None)
    route = [[1.0, 2.0, 0.0], [3.0, 2.0, 0.0]]

    assert _commit_local_avoidance_route(
        objects,
        path_stream=object(),
        state=state,
        route_world=route,
        mode="hard_retreat",
        hold_yaw_deg=0.0,
        bypass_point_count=2,
        base_revision=0,
    )
    assert not _commit_local_avoidance_route(
        objects,
        path_stream=object(),
        state=state,
        route_world=route,
        mode="hard_retreat",
        hold_yaw_deg=0.0,
        bypass_point_count=2,
        base_revision=1,
    )


def test_single_reference_point_cannot_be_committed_as_a_bypass():
    replacements = []
    queue = SimpleNamespace(
        route_revision=0,
        replace_world_waypoints=lambda route, **_kwargs: replacements.append(route) or True,
    )
    objects = SimpleNamespace(
        controller=SimpleNamespace(queue=queue, discard_plan=lambda: None),
        obstacle_avoider=SimpleNamespace(config={}),
    )

    assert not _commit_local_avoidance_route(
        objects,
        path_stream=object(),
        state=SimpleNamespace(update=lambda **_values: None),
        route_world=[[10.0, 0.0, 0.0]],
        mode="hard_bypass",
        hold_yaw_deg=0.0,
        bypass_point_count=0,
        base_revision=0,
    )
    assert replacements == []


def test_empty_preplan_route_arms_recovery_instead_of_retrying_qwen():
    queue = SimpleNamespace(world_waypoints=[])
    state_values = {}
    objects = SimpleNamespace(
        controller=SimpleNamespace(queue=queue),
        obstacle_avoider=SimpleNamespace(config={}),
    )

    assert _arm_preplan_avoidance(
        objects,
        reference_world=[[10.0, 0.0, 0.0]],
        obstacle_result=SimpleNamespace(
            reason="stop_before_continuous_structure",
            obstacle_body=[2.8, 0.0, 1.7],
        ),
        hold_yaw_deg=0.0,
        state=SimpleNamespace(update=lambda **values: state_values.update(values)),
    )

    assert objects.avoidance_mode == "hard_stopped"
    assert objects.avoidance_reference_world == [[10.0, 0.0, 0.0]]
    assert state_values["avoidance_state"] == "preplan_hard_stopped"


def test_clear_reference_route_resumes_with_path_tangent_heading():
    class Queue:
        route_revision = 0

        def __init__(self):
            self.world_waypoints = []

        def replace_world_waypoints(self, route, *, base_revision=None):
            if base_revision is not None and base_revision != self.route_revision:
                return False
            self.world_waypoints = [list(point) for point in route]
            self.route_revision += 1
            return True

    class Capturer:
        def get_latest_frame_with_timestamp(self):
            return None, object(), time.perf_counter()

    class Avoider:
        enabled = True
        config = {
            "REALTIME_DEPTH_MAX_AGE_S": 1.0,
            "EMERGENCY_BRAKE_DISTANCE_M": 3.0,
            "RECOVERY_ROUTE_CLEAR_MARGIN_M": 1.0,
            "RECOVERY_ROUTE_CLEAR_CONFIRMATIONS": 3,
        }

        @staticmethod
        def update_from_depth(**_kwargs):
            pass

        @staticmethod
        def front_corridor_clearance_m(_depth):
            return 10.0

        @staticmethod
        def filter_cumulative_waypoints(*_args, **_kwargs):
            return SimpleNamespace(changed=False)

    queue = Queue()
    clears = []
    objects = SimpleNamespace(
        avoidance_mode="hard_stopped",
        avoidance_hold_yaw_deg=-30.0,
        avoidance_reference_world=[[10.0, 0.0, 0.0]],
        avoidance_retreat_done=True,
        avoidance_last_depth_at=0.0,
        avoidance_reference_clear_count=0,
        avoidance_reference_clear_depth_at=0.0,
        obstacle_avoider=Avoider(),
        mission_memory=None,
        controller=SimpleNamespace(
            queue=queue,
            discard_plan=lambda: None,
        ),
        realtime_safety_monitor=SimpleNamespace(
            clear=lambda **kwargs: clears.append(kwargs)
        ),
        _avoidance_client=SimpleNamespace(get_speed_mps=lambda: 0.0),
    )
    client = SimpleNamespace(get_pose=lambda: ([0.0, 0.0, 0.0], 0.0))
    state_values = {}
    state = SimpleNamespace(update=lambda **values: state_values.update(values))

    for _ in range(2):
        assert _service_avoidance_recovery(
            objects,
            client,
            path_stream=object(),
            capturer=Capturer(),
            state=state,
            stage=None,
        )
        assert queue.world_waypoints == []
        assert objects.avoidance_mode == "hard_stopped"

    assert _service_avoidance_recovery(
        objects,
        client,
        path_stream=object(),
        capturer=Capturer(),
        state=state,
        stage=None,
    )

    assert queue.world_waypoints == [[10.0, 0.0, 0.0]]
    assert objects.avoidance_mode == "normal"
    assert objects.avoidance_hold_yaw_deg is None
    assert state_values["avoidance_state"] == "reference_route_resumed"
    assert len(clears) == 1
