"""Thread-safe fan-out for Camera API snapshots."""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable, Optional

from sim.camera_frames import CameraFrame, MultiCameraSnapshot


class CameraFrameHub:
    """Publish each Camera API snapshot once to runtime, preview and recorder."""

    def __init__(self, *, history_seconds: float = 3.0, max_history: int = 180):
        self.history_seconds = max(0.1, float(history_seconds))
        self.max_history = max(1, int(max_history))
        self._lock = threading.RLock()
        self._latest_snapshot: Optional[MultiCameraSnapshot] = None
        self._latest_frames: dict[str, CameraFrame] = {}
        self._history: deque[MultiCameraSnapshot] = deque(maxlen=self.max_history)
        self._subscribers: list[Callable[[MultiCameraSnapshot], None]] = []
        self._published_keys: deque[tuple[str, tuple[tuple[str, int], ...]]] = deque(maxlen=512)
        self._published_key_set: set[tuple[str, tuple[tuple[str, int], ...]]] = set()

    @staticmethod
    def _snapshot_key(snapshot: MultiCameraSnapshot):
        return (
            str(snapshot.capture_id),
            tuple(sorted((camera_id, int(frame.timestamp_ns)) for camera_id, frame in snapshot.frames.items())),
        )

    def publish(self, snapshot: MultiCameraSnapshot) -> bool:
        if snapshot is None or not snapshot.frames:
            return False
        key = self._snapshot_key(snapshot)
        with self._lock:
            if key in self._published_key_set:
                return False
            if len(self._published_keys) == self._published_keys.maxlen:
                oldest = self._published_keys.popleft()
                self._published_key_set.discard(oldest)
            self._published_keys.append(key)
            self._published_key_set.add(key)
            self._latest_snapshot = snapshot
            self._latest_frames.update(snapshot.frames)
            self._history.append(snapshot)
            self._prune_history_locked()
            subscribers = list(self._subscribers)
        for callback in subscribers:
            try:
                callback(snapshot)
            except Exception as exc:
                print(f"  [CameraFrameHub] subscriber error: {exc}")
        return True

    def subscribe(self, callback: Callable[[MultiCameraSnapshot], None]) -> None:
        with self._lock:
            if callback not in self._subscribers:
                self._subscribers.append(callback)

    def unsubscribe(self, callback: Callable[[MultiCameraSnapshot], None]) -> None:
        with self._lock:
            if callback in self._subscribers:
                self._subscribers.remove(callback)

    def latest_snapshot(self) -> Optional[MultiCameraSnapshot]:
        with self._lock:
            return self._latest_snapshot

    def latest_frame(self, camera_id: str) -> Optional[CameraFrame]:
        with self._lock:
            return self._latest_frames.get(str(camera_id))

    def camera_ids(self) -> list[str]:
        with self._lock:
            return sorted(self._latest_frames)

    def history(self, seconds: float | None = None) -> list[MultiCameraSnapshot]:
        horizon = self.history_seconds if seconds is None else max(0.0, float(seconds))
        cutoff = time.perf_counter() - horizon
        with self._lock:
            return [snapshot for snapshot in self._history if float(snapshot.captured_at) >= cutoff]

    def _prune_history_locked(self) -> None:
        cutoff = time.perf_counter() - self.history_seconds
        while self._history and float(self._history[0].captured_at) < cutoff:
            self._history.popleft()

