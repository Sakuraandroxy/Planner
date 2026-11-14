"""Native ProjectAirSim collision contacts with ground noise removed.

``{robot}/collision_info`` is published for every contact of the robot's
collision components. A rover resting on or driving over the map reports
hundreds of contacts per second with the Unreal landscape actor (measured:
397 events in 3 s of driving, all ``Landscape_1``), so ground contacts are
filtered by object name before latching. The city map's road surface is part
of the landscape, but the neighborhood map lays it out as separate
``Road_82``/``Road_88``/``Road_89`` actors, so those are ground too and must
be ignored the same way. Spawn and teleport also publish a contact with the
robot's own actor, which cannot be an obstacle. Everything else is a real
contact that the execution layer must treat as a collision.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import threading

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class CollisionEvent:
    object_name: str
    timestamp_ns: int
    position: tuple[float, float, float] | None = None
    normal: tuple[float, float, float] | None = None
    penetration_depth: float = 0.0


def _point(value: object) -> tuple[float, float, float] | None:
    if not isinstance(value, dict):
        return None
    try:
        return (float(value["x"]), float(value["y"]), float(value["z"]))
    except (KeyError, TypeError, ValueError):
        return None


class CollisionMonitor:
    def __init__(
        self,
        client,
        topic: str,
        *,
        ignored_prefixes: tuple[str, ...] = ("Landscape", "Road_"),
        ignored_names: tuple[str, ...] = (),
    ):
        self._client = client
        self._topic = str(topic)
        self._ignored_prefixes = tuple(
            prefix.lower() for prefix in ignored_prefixes
        )
        self._ignored_names = frozenset(name.lower() for name in ignored_names)
        self._lock = threading.Lock()
        self._pending: CollisionEvent | None = None
        self._closed = False

    def start(self) -> None:
        self._client.subscribe(self._topic, self._on_collision)

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._pending = None
        try:
            self._client.unsubscribe(self._topic)
        except Exception:
            # The client may already be disconnected; topics die with it.
            pass

    def _is_ignored(self, object_name: str) -> bool:
        lowered = object_name.lower()
        if lowered in self._ignored_names:
            return True
        return any(lowered.startswith(prefix) for prefix in self._ignored_prefixes)

    def _on_collision(self, _topic, message: dict) -> None:
        # Runs on the client's topic receive thread: keep it cheap and safe.
        try:
            object_name = str(message.get("object_name") or "")
            if not object_name or self._is_ignored(object_name):
                return
            event = CollisionEvent(
                object_name=object_name,
                timestamp_ns=int(message.get("time_stamp") or 0),
                position=_point(message.get("position")),
                normal=_point(message.get("normal")),
                penetration_depth=float(message.get("penetration_depth") or 0.0),
            )
        except Exception:
            return
        with self._lock:
            if self._closed or self._pending is not None:
                return
            self._pending = event
        # Name the contact in the service log: ProjectAirSim reports transient
        # spawn/settle and attachment contacts on the same topic as real
        # obstacles, and only the object name tells one from the other.
        logger.info(
            "collision contact latched: object=%s penetration=%.3f position=%s",
            event.object_name,
            event.penetration_depth,
            event.position,
        )

    def consume(self) -> CollisionEvent | None:
        """Return the latched contact, if any, and clear it."""
        with self._lock:
            event = self._pending
            self._pending = None
            return event

    def clear(self) -> None:
        with self._lock:
            self._pending = None