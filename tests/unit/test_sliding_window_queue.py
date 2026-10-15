import threading
from types import SimpleNamespace

import pytest

from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.errors import ExecutionError, ProtocolError
from planner.services.trajectory_queue import WorldWaypointQueue
from planner.services.trajectory_transform import relative_to_world, world_to_relative


def world(angle=0):
    points = RelativeTrajectory(tuple(RelativePoseDelta(1, 0.5, -0.2, angle) for _ in range(5)))
    return relative_to_world(points, WorldPose(3, 2, -5, 170))


@pytest.mark.parametrize("angle", [0, 15, -30, 180, -180])
def test_world_pending_round_trip_preserves_preceding_heading_and_turn_direction(angle):
    value = world(angle)
    restored = relative_to_world(world_to_relative(value), value.start)
    assert restored.yaw_deltas_deg == value.yaw_deltas_deg
    for a, b in zip(restored.poses, value.poses):
        assert (a.x, a.y, a.z, a.yaw_deg) == pytest.approx((b.x, b.y, b.z, b.yaw_deg))


def test_prefix_consumption_preserves_snapshot_revision_and_tail():
    queue = WorldWaypointQueue(0.3, 2)
    value = world(15)
    queue.load_batch(value)
    snapshot = SimpleNamespace(queue_revision=queue.revision,
                               pending_ids=tuple(p.waypoint_id for p in queue.items))
    for item in queue.items[:2]:
        queue.consume(item.waypoint_id, item.end)
    assert queue.matches(snapshot)
    pending = queue.pending(value.poses[1])
    rebuilt = relative_to_world(pending, value.poses[1])
    assert len(pending.points) == 3
    assert rebuilt.poses[-1] == value.poses[-1]
    for item in queue.items:
        queue.consume(item.waypoint_id, item.end)
    assert queue.matches(snapshot)
    queue.clear()
    assert not queue.matches(snapshot)


def test_unfinished_rotation_cannot_be_consumed_by_position_alone():
    queue = WorldWaypointQueue(0.3, 2)
    points = RelativeTrajectory(tuple(RelativePoseDelta(0, 0, 0, 30) for _ in range(5)))
    value = relative_to_world(points, WorldPose(0, 0, -5, 0))
    queue.load_batch(value)
    with pytest.raises(ExecutionError, match="yaw tolerance"):
        queue.consume(queue.items[0].waypoint_id, value.start)
    assert len(queue.items) == 5


def test_owner_guard_rejects_foreign_queue_mutation():
    queue = WorldWaypointQueue(0.3, 2)
    errors = []
    def mutate():
        try:
            queue.clear()
        except Exception as exc:
            errors.append(exc)
    thread = threading.Thread(target=mutate)
    thread.start()
    thread.join(1)
    assert not thread.is_alive()
    assert isinstance(errors[0], ExecutionError)


@pytest.mark.parametrize("angle,drift", [(180, -0.1), (-180, 0.1)])
def test_half_turn_is_not_flipped_to_hide_anchor_error(angle, drift):
    queue = WorldWaypointQueue(0.3, 2)
    value = world(angle)
    queue.load_batch(value)
    anchor = WorldPose(value.start.x, value.start.y, value.start.z, value.start.yaw_deg + drift)
    with pytest.raises(ProtocolError, match="signed yaw"):
        queue.pending(anchor)


def test_queue_rejects_loading_another_batch_over_unfinished_pending():
    queue = WorldWaypointQueue(0.3, 2)
    queue.load_batch(world())
    with pytest.raises(ProtocolError):
        queue.load_batch(world())
