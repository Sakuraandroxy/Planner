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

import logging
import threading
from dataclasses import dataclass

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
    # Contacts against these prefixes are ground ONLY when the contact normal
    # is vertical. The JiaoLiuCenter map merges its whole static world —
    # drivable ground AND boundary walls — into one ``LCCActor_0`` collision
    # actor: resting on or driving over the ground latches contacts with zero
    # penetration at every traveled position (all with a vertical normal), but
    # the same actor also publishes the road-edge wall hit (horizontal normal).
    # Unconditionally ignoring the prefix wedges the car silently against the
    # boundary; a purely vertical-normal rule keeps the wall contact as a real
    # collision.
    GROUND_PREFIXES = ("Landscape", "Road_", "LCCActor")
    VERTICAL_NORMAL_Z = 0.7

    def __init__(
        self,
        client,
        topic: str,
        *,
        ignored_prefixes: tuple[str, ...] = GROUND_PREFIXES,
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

    def _on_collision(self, _topic, message: dict) -> None:
        # Runs on the client's topic receive thread: keep it cheap and safe.
        try:
            object_name = str(message.get("object_name") or "")
            if not object_name:
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
        if self._is_ignored(event):
            return
        with self._lock:
            if self._closed or self._pending is not None:
                return
            self._pending = event
        # Name the contact in the service log: ProjectAirSim reports transient
        # spawn/settle and attachment contacts on the same topic as real
        # obstacles, and only the object name tells one from the other.
        logger.info(
            "collision contact latched: object=%s penetration=%.3f normal=%s position=%s",
            event.object_name,
            event.penetration_depth,
            event.normal,
            event.position,
        )

    def _is_ignored(self, event: CollisionEvent) -> bool:
        lowered = event.object_name.lower()
        if lowered in self._ignored_names:
            return True
        if not any(lowered.startswith(prefix) for prefix in self._ignored_prefixes):
            return False
        # A ground prefix only suppresses ground-like contacts: a vertical
        # normal means a wheel or the body touched a surface from above. The
        # same merged actor can also present a wall face, whose horizontal
        # normal must stay a real collision.
        normal = event.normal
        if normal is None:
            # Without a normal the contact cannot be classified as ground;
            # report it rather than silently wedge the car against an
            # unreported wall.
            return False
        return abs(normal[2]) >= self.VERTICAL_NORMAL_Z

    def consume(self) -> CollisionEvent | None:
        """Return the latched contact, if any, and clear it."""
        with self._lock:
            event = self._pending
            self._pending = None
            return event

    def clear(self) -> None:
        with self._lock:
            self._pending = None