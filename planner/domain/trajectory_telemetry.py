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
    #: 载具能否改变高度。前端据此隐藏 dz：车只能在地面上走。
    vertical_motion: bool = True
    queue: QueueSnapshot = QueueSnapshot(0, 0)
    vehicle: VehicleSnapshot | None = None
    planning_calls: int = 0
    path_length_m: float = 0.0
    points: tuple[TrajectoryPointSnapshot, ...] = ()
    revision: int = 0
    updated_monotonic: float = 0.0


def vehicle_snapshot(pose: WorldPose) -> VehicleSnapshot:
    return VehicleSnapshot(pose.x, pose.y, pose.z, pose.yaw_deg)


def snapshot_to_payload(snapshot: TrajectorySnapshot) -> dict:
    """Turn a snapshot into JSON-safe data for the mission-worker channel.

    The UE4 task loop runs in a child process, so the queue it publishes has to
    cross a pipe before it reaches the hub the HTTP thread reads. The wire format
    lives here, next to the objects it describes, for the same reason the weather
    catalog's does: a consumer should not have to guess a nested dict's shape.
    """
    return {
        "active": bool(snapshot.active),
        "instruction": snapshot.instruction,
        "stage_id": snapshot.stage_id,
        "mode": snapshot.mode,
        "vertical_motion": bool(snapshot.vertical_motion),
        "queue": {"count": snapshot.queue.count, "capacity": snapshot.queue.capacity},
        "vehicle": (
            None
            if snapshot.vehicle is None
            else {
                "x": snapshot.vehicle.x,
                "y": snapshot.vehicle.y,
                "z": snapshot.vehicle.z,
                "yaw_deg": snapshot.vehicle.yaw_deg,
            }
        ),
        "planning_calls": int(snapshot.planning_calls),
        "path_length_m": float(snapshot.path_length_m),
        "points": [
            {
                "index": point.index,
                "waypoint_id": point.waypoint_id,
                "x": point.x,
                "y": point.y,
                "z": point.z,
                "yaw_deg": point.yaw_deg,
                "yaw_delta_deg": point.yaw_delta_deg,
                "distance_m": point.distance_m,
                "leg_m": point.leg_m,
            }
            for point in snapshot.points
        ],
        "revision": int(snapshot.revision),
    }


def snapshot_from_payload(payload: dict) -> TrajectorySnapshot:
    """Rebuild a snapshot from :func:`snapshot_to_payload`.

    Anything malformed raises: a half-read snapshot would show the console a
    shorter remaining path than the vehicle is actually following.
    """
    try:
        vehicle = payload.get("vehicle")
        return TrajectorySnapshot(
            active=bool(payload["active"]),
            instruction=str(payload["instruction"]),
            stage_id=payload.get("stage_id"),
            mode=str(payload["mode"]),
            vertical_motion=bool(payload.get("vertical_motion", True)),
            queue=QueueSnapshot(
                int(payload["queue"]["count"]), int(payload["queue"]["capacity"])
            ),
            vehicle=(
                None
                if vehicle is None
                else VehicleSnapshot(
                    float(vehicle["x"]),
                    float(vehicle["y"]),
                    float(vehicle["z"]),
                    float(vehicle["yaw_deg"]),
                )
            ),
            planning_calls=int(payload["planning_calls"]),
            path_length_m=float(payload["path_length_m"]),
            points=tuple(
                TrajectoryPointSnapshot(
                    index=int(point["index"]),
                    waypoint_id=int(point["waypoint_id"]),
                    x=float(point["x"]),
                    y=float(point["y"]),
                    z=float(point["z"]),
                    yaw_deg=float(point["yaw_deg"]),
                    yaw_delta_deg=float(point["yaw_delta_deg"]),
                    distance_m=float(point["distance_m"]),
                    leg_m=float(point["leg_m"]),
                )
                for point in payload["points"]
            ),
            revision=int(payload["revision"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"unusable trajectory snapshot: {error}") from error


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
    vertical_motion: bool = True,
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
        vertical_motion=vertical_motion,
        queue=QueueSnapshot(len(points), int(capacity)),
        vehicle=vehicle_snapshot(pose),
        planning_calls=int(planning_calls),
        path_length_m=float(path_length_m),
        points=tuple(points),
        updated_monotonic=float(clock()),
    )