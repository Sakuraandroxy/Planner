"""Non-owning moving observation proxy for shared vehicle adapters.

The reference planner opens a dedicated read-only transport for moving
captures, so closing it cannot affect the command channel. ProjectAirSim's
topic socket is a single-consumer link and the vehicle adapter already owns
the only camera subscription, so workflows borrow the vehicle's capture
instead. This proxy delegates reads and keeps ``close`` a no-op: the session
owns the vehicle lifecycle (landing, hold and the async bridge).
"""

from __future__ import annotations


class BorrowedMovingSource:
    def __init__(self, vehicle):
        self._vehicle = vehicle

    def capture(self, *, budget=None):
        # Collision recovery replans from a fresh stopped frame; the reference
        # planner hands the vehicle itself to this slot, so the borrowed proxy
        # must expose the same stopped capture as well as the moving one.
        return self._vehicle.capture(budget=budget)

    def capture_moving(self, *, budget=None):
        return self._vehicle.capture_moving(budget=budget)

    def close(self) -> None:
        pass