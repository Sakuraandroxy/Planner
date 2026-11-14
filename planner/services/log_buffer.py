"""Bounded diagnostic text with cursors shared by all HTTP readers."""

from __future__ import annotations

from collections import deque
from datetime import datetime
import threading

from ..domain.server import LogBatch, ServerLog


class LogBuffer:
    def __init__(self, capacity: int = 500):
        if capacity <= 0:
            raise ValueError("log buffer capacity must be positive")
        self._items: deque[ServerLog] = deque(maxlen=int(capacity))
        self._sequence = 0
        self._lock = threading.Lock()

    def append(self, message: str, level: str = "INFO") -> None:
        with self._lock:
            self._sequence += 1
            self._items.append(
                ServerLog(
                    self._sequence,
                    datetime.now().strftime("%H:%M:%S"),
                    level,
                    str(message),
                )
            )

    def read(self, cursor: int = 0) -> LogBatch:
        if cursor < 0:
            raise ValueError("cursor must be non-negative")
        with self._lock:
            items = tuple(item for item in self._items if item.id > cursor)
            truncated = bool(
                self._items and cursor and cursor < self._items[0].id - 1
            )
            return LogBatch(
                items,
                items[-1].id if items else self._sequence,
                truncated,
            )