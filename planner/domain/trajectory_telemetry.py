"""JSON-safe snapshots of the remaining world-waypoint queue.

The console reads these snapshots over HTTP. They are built on the workflow
thread while it still owns the queue; the HTTP thread never touches
``WorldWaypointQueue`` directly.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .pose import WorldPose
from .sliding_window import QueuedWaypoint

MODE_IDLE = "idle"
MODE_PENDING = "pending"
MODE_CONTINUOUS = "continuous_world_queue"
MODE_STOPPED_BATCH = "stopped_batch"
MODE_NO_QUEUE = "no_queue"


@dataclass(frozen=True)
class VehicleSnapshot:
    x: float
    y: float
    z: float
    yaw_deg: float


@dataclass(frozen=True)
class TrajectoryPointSnapshot:
    index: int
    waypoint_id: int
    x: float
    y: float
    z: float
    yaw_deg: float
    yaw_delta_deg: float
    distance_m: float
    leg_m: float


@dataclass(frozen=True)
class QueueSnapshot:
    count: int
    capacity: int


@dataclass(frozen=True)
class TrajectorySnapshot:
    active: bool = False
    instruction: str = ""
    stage_id: str | None = None
    mode: str = MODE_IDLE
    queue: QueueSnapshot = QueueSnapshot(0, 0)
    vehicle: VehicleSnapshot | None = None
    planning_calls: int = 0
    path_length_m: float = 0.0
    points: tuple[TrajectoryPointSnapshot, ...] = ()
    revision: int = 0
    updated_monotonic: float = 0.0


def vehicle_snapshot(pose: WorldPose) -> VehicleSnapshot:
    return VehicleSnapshot(pose.x, pose.y, pose.z, pose.yaw_deg)


def build_queue_snapshot(
    *,
    instruction: str,
    stage_id: str | None,
    mode: str,
    capacity: int,
    pose: WorldPose,
    items: tuple[QueuedWaypoint, ...],
    planning_calls: int,
    path_length_m: float,
    clock,
) -> TrajectorySnapshot:
    """Describe the remaining queue in world coordinates.

    ``distance_m`` is the straight-line horizontal distance from the vehicle
    to that waypoint; ``leg_m`` is the length of the preceding leg (vehicle to
    the first point, then point to point).
    """
    points = []
    previous_x, previous_y = pose.x, pose.y
    for index, item in enumerate(tuple(items), start=1):
        end = item.end
        points.append(
            TrajectoryPointSnapshot(
                index=index,
                waypoint_id=item.waypoint_id,
                x=end.x,
                y=end.y,
                z=end.z,
                yaw_deg=end.yaw_deg,
                yaw_delta_deg=item.yaw_delta_deg,
                distance_m=math.hypot(end.x - pose.x, end.y - pose.y),
                leg_m=math.hypot(end.x - previous_x, end.y - previous_y),
            )
        )
        previous_x, previous_y = end.x, end.y
    return TrajectorySnapshot(
        active=True,
        instruction=instruction,
        stage_id=stage_id,
        mode=mode,
        queue=QueueSnapshot(len(points), int(capacity)),
        vehicle=vehicle_snapshot(pose),
        planning_calls=int(planning_calls),
        path_length_m=float(path_length_m),
        points=tuple(points),
        updated_monotonic=float(clock()),
    )