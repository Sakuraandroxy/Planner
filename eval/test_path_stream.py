"""Regression tests for heading-preserving local obstacle bypasses."""

from types import SimpleNamespace

from agent.functions.fast_slow.runtime import (
    _drop_path_points_behind_vehicle,
    _sync_path_if_ready,
)
from agent.functions.fast_slow.path_stream import ContinuousPathStream


class _FakeClient:
    def __init__(self):
        self.commands = []
        self.stop_count = 0
        self.pose = [32.29, 0.53, -7.45]
        self.yaw = 40.4

    def start_waypoint_path(self, waypoints, velocity=None, **kwargs):
        self.commands.append((waypoints, velocity, kwargs))

    def collision_marker(self):
        return (False, 0, 0)

    def get_pose(self):
        return list(self.pose), float(self.yaw)

    def has_collision_since(self, _marker):
        return False

    def stop_waypoint_path(self):
        self.stop_count += 1


def test_path_stream_passes_fixed_heading_for_bypass():
    client = _FakeClient()
    stream = ContinuousPathStream(client)

    issued = stream.sync(
        [[1.0, -3.0, 0.0], [5.0, -3.0, 0.0], [10.0, 0.0, 0.0]],
        [0.0, 0.0, 0.0],
        2.0,
        hold_heading=True,
        heading_yaw_deg=65.0,
    )

    assert issued
    assert len(client.commands) == 1
    _waypoints, velocity, options = client.commands[0]
    assert velocity == 2.0
    assert options == {"hold_heading": True, "heading_yaw_deg": 65.0}


def test_path_stream_reissues_when_bypass_heading_changes():
    client = _FakeClient()
    stream = ContinuousPathStream(client)
    waypoints = [[1.0, -3.0, 0.0], [5.0, -3.0, 0.0]]

    assert stream.sync(waypoints, [0.0, 0.0, 0.0], 2.0, hold_heading=True, heading_yaw_deg=65.0)
    assert not stream.sync(waypoints, [0.0, 0.0, 0.0], 2.0, hold_heading=True, heading_yaw_deg=65.0)
    assert stream.sync(waypoints, [0.0, 0.0, 0.0], 2.0, hold_heading=True, heading_yaw_deg=70.0)
    assert len(client.commands) == 2


def test_pure_vertical_path_automatically_holds_entry_heading():
    client = _FakeClient()
    stream = ContinuousPathStream(client)
    waypoint = [[32.29, 0.53, -13.45]]

    assert stream.sync(waypoint, client.pose, 1.3)
    assert not stream.sync(waypoint, client.pose, 1.3)

    _waypoints, velocity, options = client.commands[0]
    assert velocity == 1.3
    assert options == {"hold_heading": True, "heading_yaw_deg": 40.4}


def test_horizontal_path_keeps_normal_path_heading_mode():
    client = _FakeClient()
    stream = ContinuousPathStream(client)

    assert stream.sync([[40.0, 2.0, -7.45]], client.pose, 2.0)
    assert client.commands[0][2] == {}


def test_vertical_endpoint_residual_inside_climb_tolerance_is_consumed():
    client = _FakeClient()
    stream = ContinuousPathStream(client, {"PATH_VERTICAL_FINAL_REACH_TOLERANCE_M": 0.5})
    waypoint = [[32.29, 0.53, -13.45]]
    assert stream.sync(waypoint, client.pose, 1.3)

    client.pose = [32.29, 0.53, -13.09]
    progress = stream.poll(waypoint, client.pose)

    assert progress.consumed == 1
    assert not stream.active


def test_vertical_waypoint_is_not_dropped_as_behind_and_reaches_path_stream():
    client = _FakeClient()
    # The rounded runtime log prints the same XY for pose and waypoint, but
    # this sub-centimetre drift gives the old heading filter a negative dot
    # product and used to delete the climb.
    client.pose = [32.2904, 0.5303, -1.48]
    client.yaw = 26.2
    waypoint = [32.29, 0.53, -11.48]
    queue = SimpleNamespace(world_waypoints=[list(waypoint)])
    controller = SimpleNamespace(
        queue=queue,
        planning=False,
        reserve_time_s=1.0,
        mark_executed=lambda _count: None,
        clear=lambda: None,
    )
    objects = SimpleNamespace(controller=controller)

    dropped = _drop_path_points_behind_vehicle(objects, client.pose, client.yaw)

    assert dropped == 0
    assert queue.world_waypoints == [waypoint]

    class _State:
        def update(self, **_kwargs):
            pass

    stream = ContinuousPathStream(client)
    consumed = _sync_path_if_ready(objects, stream, client, _State())

    assert consumed == 0
    assert queue.world_waypoints == [waypoint]
    assert client.commands == [
        ([waypoint], 2.0, {"hold_heading": True, "heading_yaw_deg": 26.2})
    ]


def test_true_horizontal_waypoint_behind_vehicle_is_still_dropped():
    queue = SimpleNamespace(world_waypoints=[[-2.0, 0.0, -1.0]])
    objects = SimpleNamespace(controller=SimpleNamespace(queue=queue))

    dropped = _drop_path_points_behind_vehicle(objects, [0.0, 0.0, -1.0], 0.0)

    assert dropped == 1
    assert queue.world_waypoints == []


def test_emergency_stop_always_reaches_airsim_after_local_reset():
    client = _FakeClient()
    stream = ContinuousPathStream(client)
    stream.reset()

    stream.emergency_stop()

    assert client.stop_count == 1
    assert not stream.active
