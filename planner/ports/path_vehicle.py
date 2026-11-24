"""Optional engine path-following port: one whole world-frame path per call.

The engine owns the tracking. The shared layer hands over an ordered polyline and
a speed, then waits. It does not steer point by point, stop at each waypoint, send
a yaw command or validate the points -- that is the difference between this port
and ``Vehicle.execute_segment``, which flies one stopped segment at a time.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

from ..domain.pose import WorldPose


class PathVehicle(Protocol):
    def follow_path(
        self,
        points: Sequence[WorldPose],
        speed_mps: float,
        *,
        timeout_s: float,
    ) -> None:
        """Follow ``points`` as one path and return when the last one is reached.

        ``points`` are world-frame poses in travel order. The whole path is handed
        to the engine in one request: the engine turns the vehicle to follow it, so
        no yaw angle is passed in. The call blocks until the path is flown; it
        raises ``ExecutionError`` when the engine refuses or aborts the request and
        when the path is not finished within ``timeout_s`` seconds.
        """
