"""Thread-safe latest camera frame shared by the worker and HTTP viewers.

The CARLA sensor callback owns the writer side; MJPEG handlers read the newest
frame by sequence number. Publishing is throttled so a 20 Hz control loop does
not force a 20 fps network stream.
"""

from __future__ import annotations

import threading
import time

import numpy as np


class FrameHub:
    def __init__(self, *, max_fps: float = 10.0, clock=time.monotonic):
        if max_fps <= 0:
            raise ValueError("max_fps must be positive")
        self._min_interval_s = 1.0 / float(max_fps)
        self._clock = clock
        self._cond = threading.Condition()
        self._sequence = 0
        self._frame: np.ndarray | None = None
        self._timestamp_s: float | None = None
        self._last_publish = float("-inf")
        self._closed = False

    def publish(self, bgra: np.ndarray, timestamp_s: float | None = None) -> None:
        """Offer a BGRA frame; frames above ``max_fps`` are dropped."""
        with self._cond:
            if self._closed:
                return
            now = self._clock()
            if now - self._last_publish < self._min_interval_s:
                return
            self._last_publish = now
            self._sequence += 1
            self._frame = np.asarray(bgra)
            self._timestamp_s = (
                None if timestamp_s is None else float(timestamp_s)
            )
            self._cond.notify_all()

    def wait_next(
        self, after: int, *, timeout_s: float = 0.5
    ) -> tuple[int, np.ndarray] | None:
        """Return the newest frame newer than ``after``, or ``None`` on timeout."""
        with self._cond:
            if self._sequence <= after:
                self._cond.wait(timeout_s)
            if self._sequence <= after or self._frame is None:
                return None
            return self._sequence, self._frame

    @property
    def sequence(self) -> int:
        with self._cond:
            return self._sequence

    @property
    def closed(self) -> bool:
        with self._cond:
            return self._closed

    def close(self) -> None:
        """Stop publishing; streaming viewers notice on their next poll."""
        with self._cond:
            self._closed = True
            self._cond.notify_all()