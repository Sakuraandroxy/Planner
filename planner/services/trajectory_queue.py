"""Owner-thread world queue; consuming a prefix does not change its revision."""
import threading

from planner.domain.pose import WorldPose, resolve_yaw_delta_deg
from planner.domain.sliding_window import PlanningSnapshot, QueuedWaypoint
from planner.domain.trajectory import RelativeTrajectory, WorldTrajectory
from planner.errors import ExecutionError, ProtocolError
from planner.services.trajectory_transform import world_to_relative
from planner.services.trajectory_validation import require_execution_anchor
from planner.domain.streaming import QueuePauseSnapshot
from planner.services.motion.continuous import reached


class WorldWaypointQueue:
    def __init__(self, position_tolerance_m: float, yaw_tolerance_deg: float):
        self.position_tolerance_m = position_tolerance_m
        self.yaw_tolerance_deg = yaw_tolerance_deg
        self._owner = threading.get_ident()
        self._items: list[QueuedWaypoint] = []
        self._next_id = 0
        self._revision = 0

    def check_owner(self) -> None:
        if threading.get_ident() != self._owner:
            raise ExecutionError("world queue may only be accessed by its execution owner")

    @property
    def items(self) -> tuple[QueuedWaypoint, ...]:
        self.check_owner()
        return tuple(self._items)

    @property
    def revision(self) -> int:
        self.check_owner()
        return self._revision

    def clear(self) -> None:
        self.check_owner()
        self._items.clear()
        self._revision += 1

    def load_batch(self, world: WorldTrajectory) -> None:
        self.check_owner()
        if self._items or len(world.poses) != 5:
            raise ProtocolError("authorize exactly five new points only at an empty batch boundary")
        previous = world.start
        for index, end in enumerate(world.poses):
            delta = None if world.yaw_deltas_deg is None else world.yaw_deltas_deg[index]
            yaw = resolve_yaw_delta_deg(previous.yaw_deg, end.yaw_deg, delta)
            self._next_id += 1
            self._items.append(QueuedWaypoint(self._next_id, previous, end, yaw))
            previous = end
        self._revision += 1

    def pending(self, anchor: WorldPose) -> RelativeTrajectory:
        self.check_owner()
        if not self._items:
            return RelativeTrajectory(())
        first = self._items[0]
        # Small measured heading error must not flip a signed half-turn.
        correction = resolve_yaw_delta_deg(first.start.yaw_deg, anchor.yaw_deg)
        deltas = (first.yaw_delta_deg - correction,) + tuple(p.yaw_delta_deg for p in self._items[1:])
        if any(abs(delta) > 180 for delta in deltas):
            raise ProtocolError("remaining signed yaw cannot be represented in the pending protocol")
        world = WorldTrajectory(anchor, tuple(p.end for p in self._items), deltas)
        relative = world_to_relative(world)
        return relative

    def load_prefix(self, world: WorldTrajectory) -> None:
        """Authorize a locally checked 1..5-point prefix; load_batch stays five-only."""
        self.check_owner()
        if self._items or not 1 <= len(world.poses) <= 5:
            raise ProtocolError("authorize a nonempty safe prefix only at an empty batch boundary")
        previous = world.start
        for index, end in enumerate(world.poses):
            delta = None if world.yaw_deltas_deg is None else world.yaw_deltas_deg[index]
            yaw = resolve_yaw_delta_deg(previous.yaw_deg, end.yaw_deg, delta)
            self._next_id += 1
            self._items.append(QueuedWaypoint(self._next_id, previous, end, yaw))
            previous = end
        self._revision += 1

    def consume(self, waypoint_id: int, measured: WorldPose) -> None:
        self.check_owner()
        if not self._items or self._items[0].waypoint_id != waypoint_id:
            raise ProtocolError("only the confirmed queue head may be consumed")
        self.require_anchor(measured, self._items[0].end)
        self._items.pop(0)

    def matches(self, snapshot: PlanningSnapshot) -> bool:
        self.check_owner()
        ids = tuple(p.waypoint_id for p in self._items)
        expected = snapshot.pending_ids
        return (self._revision == snapshot.queue_revision and len(ids) <= len(expected)
                and (not ids or ids == expected[-len(ids):]))

    def require_anchor(self, measured: WorldPose, expected: WorldPose) -> None:
        require_execution_anchor(measured, expected, self.position_tolerance_m, self.yaw_tolerance_deg)

    def append_continuation(self, world, pending_ids, revision, measured, maximum_points=10):
        """Owner-only append after frozen-pending reconciliation; no pose rebase."""
        self.check_owner()
        if revision != self.revision or len(world.poses) != 5:
            raise ProtocolError("continuation belongs to an invalidated queue or batch")
        ids = tuple(p.waypoint_id for p in self._items)
        if ids and (len(ids) > len(pending_ids) or ids != pending_ids[-len(ids):]):
            raise ProtocolError("live queue is not the frozen pending suffix")
        if self._items:
            if self._items[-1].end != world.start:
                raise ProtocolError("continuation does not meet the authorized queue tail")
        else:
            self.require_anchor(measured, world.start)
        if len(self._items)+5 > maximum_points:
            raise ProtocolError("continuation exceeds local buffer capacity")
        previous = world.start
        for i, end in enumerate(world.poses):
            explicit = None if world.yaw_deltas_deg is None else world.yaw_deltas_deg[i]
            delta = resolve_yaw_delta_deg(previous.yaw_deg, end.yaw_deg, explicit)
            self._next_id += 1
            self._items.append(QueuedWaypoint(self._next_id, previous, end, delta))
            previous = end
        self._revision += 1

    def consume_stream(self, measured, unwrapped_yaw, goal_yaw):
        self.check_owner()
        if not self._items or not reached(self._items[0], measured, unwrapped_yaw, goal_yaw,
                                         self.position_tolerance_m, self.yaw_tolerance_deg):
            return False
        self._items.pop(0)
        return True

    def pause_stream(self, stage_token, state, yaw, first_goal, reason):
        self.check_owner()
        self._revision += 1  # invalidate speculative output; retain every authorized endpoint
        return QueuePauseSnapshot(stage_token, self.revision, state, yaw, self.items, first_goal, reason)

    def restore_stream(self, snapshot, stage_token):
        self.check_owner()
        if snapshot.stage_token != stage_token or snapshot.queue_revision != self.revision:
            raise ProtocolError("pause snapshot belongs to another stage or queue")
        if self.items != snapshot.remaining:
            raise ProtocolError("authorized endpoints changed while paused")
        return snapshot.first_goal_yaw_deg
