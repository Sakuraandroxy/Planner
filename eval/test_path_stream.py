"""Regression tests for heading-preserving local obstacle bypasses."""

from agent.functions.fast_slow.path_stream import ContinuousPathStream


class _FakeClient:
    def __init__(self):
        self.commands = []
        self.stop_count = 0

    def start_waypoint_path(self, waypoints, velocity=None, **kwargs):
        self.commands.append((waypoints, velocity, kwargs))

    def collision_marker(self):
        return (False, 0, 0)

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


def test_emergency_stop_always_reaches_airsim_after_local_reset():
    client = _FakeClient()
    stream = ContinuousPathStream(client)
    stream.reset()

    stream.emergency_stop()

    assert client.stop_count == 1
    assert not stream.active
