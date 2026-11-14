"""Run ProjectAirSim's asyncio commands from the synchronous control thread.

The planner's control loop is a single synchronous owner thread, while the
Drone API is coroutine based: ``move_by_velocity_async`` sends the request and
returns an ``asyncio.Task`` whose response arrives after the simulator advances
by the command duration. A dedicated event loop runs those coroutines so the
control thread can advance the clock and then collect the result.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import threading
from typing import Awaitable, TypeVar

T = TypeVar("T")


class AsyncBridge:
    def __init__(self) -> None:
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._serve, name="projectairsim-async", daemon=True
        )
        self._thread.start()

    def _serve(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def run(self, awaitable: Awaitable[T], *, timeout_s: float) -> T:
        """Await ``awaitable`` on the loop; raise ``TimeoutError`` on expiry."""
        if not self._thread.is_alive():
            raise RuntimeError("ProjectAirSim async bridge is closed")
        future = asyncio.run_coroutine_threadsafe(_await(awaitable), self._loop)
        try:
            return future.result(timeout=timeout_s)
        except concurrent.futures.TimeoutError:
            future.cancel()
            raise TimeoutError(
                "ProjectAirSim async command did not complete"
            ) from None

    @property
    def alive(self) -> bool:
        return self._thread.is_alive() and not self._loop.is_closed()

    def close(self) -> None:
        if self._thread.is_alive():
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            raise RuntimeError("ProjectAirSim async bridge did not stop")
        if self._loop.is_closed():
            return
        pending = asyncio.all_tasks(self._loop)
        for task in pending:
            task.cancel()
        if pending:
            self._loop.run_until_complete(
                asyncio.gather(*pending, return_exceptions=True)
            )
        self._loop.close()


async def _await(awaitable: Awaitable[T]) -> T:
    return await awaitable