import json
import math
from dataclasses import asdict

import pytest

from planner.domain import WorldPose
from planner.domain import QueuedWaypoint
from planner.domain import (
    MODE_CONTINUOUS,
    MODE_IDLE,
    MODE_NO_QUEUE,
    MODE_PENDING,
    MODE_STOPPED_BATCH,
    QueueSnapshot,
    TrajectorySnapshot,
    snapshot_from_payload,
    snapshot_to_payload,
)
from planner.runtime.trajectory_hub import TrajectoryHub


def _items():
    start = WorldPose(0.0, 0.0, 0.0, 0.0)
    return (
        QueuedWaypoint(4, start, WorldPose(3.0, 4.0, 0.0, 90.0), 90.0),
        QueuedWaypoint(
            5, WorldPose(3.0, 4.0, 0.0, 90.0), WorldPose(6.0, 4.0, 0.0, 90.0), 0.0
        ),
    )


def test_publish_queue_reports_whether_the_vehicle_can_change_height():
    hub = TrajectoryHub(clock=lambda: 1.0)
    ground = hub.publish_queue(
        instruction="drive to the car",
        stage_id="stage_1",
        mode=MODE_CONTINUOUS,
        capacity=10,
        pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        items=_items(),
        planning_calls=1,
        path_length_m=5.0,
        vertical_motion=False,
    )
    assert ground.vertical_motion is False
    assert json.loads(json.dumps(asdict(ground)))["vertical_motion"] is False
    airborne = hub.publish_queue(
        instruction="fly to the car",
        stage_id="stage_1",
        mode=MODE_CONTINUOUS,
        capacity=10,
        pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        items=_items(),
        planning_calls=2,
        path_length_m=5.0,
    )
    assert airborne.vertical_motion is True


def test_snapshot_payload_carries_the_height_flag_across_the_worker_channel():
    hub = TrajectoryHub(clock=lambda: 2.0)
    snapshot = hub.publish_queue(
        instruction="drive to the car",
        stage_id="stage_1",
        mode=MODE_CONTINUOUS,
        capacity=10,
        pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        items=_items(),
        planning_calls=1,
        path_length_m=5.0,
        vertical_motion=False,
    )
    assert snapshot_from_payload(snapshot_to_payload(snapshot)).vertical_motion is False


def test_publish_queue_describes_remaining_points_without_owner_access():
    hub = TrajectoryHub(clock=lambda: 12.5)
    snapshot = hub.publish_queue(
        instruction="follow the road",
        stage_id="stage_1",
        mode=MODE_CONTINUOUS,
        capacity=10,
        pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        items=_items(),
        planning_calls=3,
        path_length_m=8.25,
    )

    assert snapshot.revision == 1
    assert snapshot.active is True
    assert snapshot.instruction == "follow the road"
    assert snapshot.stage_id == "stage_1"
    assert snapshot.mode == MODE_CONTINUOUS
    assert snapshot.queue == QueueSnapshot(2, 10)
    assert snapshot.vehicle.x == 0.0
    assert snapshot.planning_calls == 3
    assert snapshot.path_length_m == 8.25
    assert snapshot.updated_monotonic == 12.5
    first, second = snapshot.points
    assert (first.index, first.waypoint_id) == (1, 4)
    assert (first.x, first.y) == (3.0, 4.0)
    assert first.distance_m == math.hypot(3.0, 4.0)
    assert first.leg_m == math.hypot(3.0, 4.0)
    assert first.yaw_delta_deg == 90.0
    assert (second.index, second.waypoint_id) == (2, 5)
    assert second.distance_m == math.hypot(6.0, 4.0)
    assert second.leg_m == 3.0


def test_revisions_increase_and_reads_return_the_latest_snapshot():
    hub = TrajectoryHub(clock=lambda: 1.0)
    assert hub.read().active is False
    started = hub.start("drive")
    assert started.revision == 1
    assert started.mode == MODE_PENDING
    no_queue = hub.publish_no_queue(
        instruction="drive", stage_id="stage_1"
    )
    assert no_queue.revision == 2
    assert no_queue.mode == MODE_NO_QUEUE
    cleared = hub.clear()
    assert cleared.revision == 3
    assert cleared.active is False
    assert cleared.mode == MODE_IDLE
    assert hub.read().revision == 3


def test_adopt_stamps_the_parent_clock_and_bumps_revision():
    """A snapshot relayed from a mission child is published like a local one.

    The child's ``updated_monotonic`` is another process's clock; keeping it would
    make the console's staleness checks compare numbers that never meant the same
    instant.
    """
    ticks = iter([100.0, 200.0])
    hub = TrajectoryHub(clock=lambda: next(ticks))
    hub.clear()
    adopted = hub.adopt(
        TrajectorySnapshot(active=True, mode=MODE_CONTINUOUS, revision=7)
    )
    assert adopted.updated_monotonic == 200.0
    assert adopted.revision == 2


def test_snapshot_payload_round_trips_every_field():
    """The wire form belongs to the domain type, not to the pipe that carries it."""
    from planner.domain import (
        QueueSnapshot,
        TrajectoryPointSnapshot,
        VehicleSnapshot,
        snapshot_from_payload,
        snapshot_to_payload,
    )

    snapshot = TrajectorySnapshot(
        active=True,
        instruction="Fly to the red car. \u98de\u5230\u7ea2\u8272\u6c7d\u8f66\u3002",
        stage_id="stage_1",
        mode=MODE_STOPPED_BATCH,
        queue=QueueSnapshot(2, 4),
        vehicle=VehicleSnapshot(1.0, 2.0, -3.0, 10.0),
        planning_calls=3,
        path_length_m=12.5,
        points=(
            TrajectoryPointSnapshot(0, 1, 1.0, 2.0, -3.0, 10.0, 5.0, 1.2, 0.9),
            TrajectoryPointSnapshot(1, 2, 2.0, 3.0, -3.0, 15.0, 5.0, 1.1, 1.4),
        ),
        revision=9,
        updated_monotonic=44.0,
    )
    payload = snapshot_to_payload(snapshot)
    assert json.loads(json.dumps(payload, ensure_ascii=False)) == payload
    # ``updated_monotonic`` deliberately does not cross the pipe: it is another
    # process's clock, and TrajectoryHub.adopt stamps the parent's own.
    from dataclasses import replace

    assert snapshot_from_payload(payload) == replace(snapshot, updated_monotonic=0.0)


def test_an_unusable_snapshot_payload_is_refused():
    from planner.domain import snapshot_from_payload

    with pytest.raises(ValueError, match="unusable trajectory snapshot"):
        snapshot_from_payload({"active": True})
