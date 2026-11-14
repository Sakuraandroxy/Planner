import math

from planner.domain.pose import WorldPose
from planner.domain.sliding_window import QueuedWaypoint
from planner.domain.trajectory_telemetry import (
    MODE_CONTINUOUS,
    MODE_IDLE,
    MODE_NO_QUEUE,
    MODE_PENDING,
    QueueSnapshot,
)
from planner.services.trajectory_hub import TrajectoryHub


def _items():
    start = WorldPose(0.0, 0.0, 0.0, 0.0)
    return (
        QueuedWaypoint(4, start, WorldPose(3.0, 4.0, 0.0, 90.0), 90.0),
        QueuedWaypoint(
            5, WorldPose(3.0, 4.0, 0.0, 90.0), WorldPose(6.0, 4.0, 0.0, 90.0), 0.0
        ),
    )


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