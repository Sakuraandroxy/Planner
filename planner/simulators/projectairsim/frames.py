"""Synchronized RGB/depth topic stream for one ProjectAirSim camera.

ProjectAirSim publishes scene and depth images on separate topics. Every
message carries the capture timestamp (nanoseconds, simulator time) and the
camera NED pose in the same fields. Empirically both topics publish the same
``time_stamp`` for a capture, so frames are paired by exact timestamp.

Callbacks run on the client's topic receive thread; they never raise (that
would stop every subscription), and decoding errors are re-raised on the next
poll. The optional ``frame_sink`` receives a BGRA frame for the console
preview, matching the existing frame hub contract.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Callable

import cv2
import numpy as np
from projectairsim.utils import unpack_image

from ...errors import ObservationError

#: Depth values at or above this are sky/no-hit sentinels (fp16 max is 65504).
SKY_DEPTH_M = 60000.0

_SCENE_ENCODING = "BGR"
_DEPTH_ENCODING = "16FC1"


@dataclass(frozen=True)
class FramePair:
    """One scene/depth capture sharing a simulator timestamp."""

    timestamp_ns: int
    rgb_message: dict
    depth_message: dict
    #: Decoded scene image in BGR order when the frame sink needed it.
    bgr: np.ndarray | None = None


class CameraTopicStream:
    def __init__(
        self,
        client,
        *,
        rgb_topic: str,
        depth_topic: str,
        width: int,
        height: int,
        frame_sink: Callable[[np.ndarray, float], None] | None = None,
        queue_limit: int = 8,
    ):
        if width <= 0 or height <= 0:
            raise ValueError("camera dimensions must be positive")
        self._client = client
        self._rgb_topic = str(rgb_topic)
        self._depth_topic = str(depth_topic)
        self._width = int(width)
        self._height = int(height)
        self._frame_sink = frame_sink
        self._queue_limit = max(2, int(queue_limit))
        self._lock = threading.Lock()
        self._rgb: dict[int, tuple[dict, np.ndarray | None]] = {}
        self._depth: dict[int, dict] = {}
        self._error: Exception | None = None
        self._closed = False
        self._last_delivered_ns = 0

    # -- lifecycle ---------------------------------------------------------
    def start(self) -> None:
        self._client.subscribe(self._rgb_topic, self._on_rgb)
        self._client.subscribe(self._depth_topic, self._on_depth)

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._rgb.clear()
            self._depth.clear()
        try:
            self._client.unsubscribe([self._rgb_topic, self._depth_topic])
        except Exception:
            # The client may already be disconnected; topics die with it.
            pass

    @property
    def last_delivered_ns(self) -> int:
        with self._lock:
            return self._last_delivered_ns

    # -- callbacks ---------------------------------------------------------
    def _on_rgb(self, _topic, message: dict) -> None:
        try:
            stamp = self._validate_message(message, _SCENE_ENCODING)
            bgr = self.decode_scene(message) if self._frame_sink is not None else None
            with self._lock:
                if self._closed:
                    return
                self._rgb[stamp] = (message, bgr)
                self._prune_locked()
        except Exception as exc:  # never stop the client receive thread
            self._fail(exc)
            return
        if self._frame_sink is not None:
            try:
                self._frame_sink(
                    cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA), stamp / 1e9
                )
            except Exception:
                pass  # a preview failure must not drop sensor frames

    def _on_depth(self, _topic, message: dict) -> None:
        try:
            stamp = self._validate_message(message, _DEPTH_ENCODING)
            with self._lock:
                if self._closed:
                    return
                self._depth[stamp] = message
                self._prune_locked()
        except Exception as exc:
            self._fail(exc)

    def _validate_message(self, message: dict, encoding: str) -> int:
        if message.get("encoding") != encoding:
            raise ValueError(
                f"expected {encoding!r} image, got {message.get('encoding')!r}"
            )
        if (
            int(message.get("width", -1)) != self._width
            or int(message.get("height", -1)) != self._height
        ):
            raise ValueError("camera frame size does not match the configured camera")
        stamp = message.get("time_stamp")
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
            raise ValueError("camera frame has no valid capture timestamp")
        return stamp

    def _fail(self, error: Exception) -> None:
        with self._lock:
            if self._error is None:
                self._error = error

    def _prune_locked(self) -> None:
        for buffer in (self._rgb, self._depth):
            while len(buffer) > self._queue_limit:
                del buffer[min(buffer)]

    # -- decoding ----------------------------------------------------------
    def decode_scene(self, message: dict) -> np.ndarray:
        image = unpack_image(message)
        if image.dtype != np.uint8 or image.shape != (self._height, self._width, 3):
            raise ValueError(
                f"scene image shape {image.shape} {image.dtype} does not match "
                f"{self._height}x{self._width} BGR"
            )
        return image

    def decode_depth(self, message: dict) -> np.ndarray:
        image = np.asarray(unpack_image(message), dtype=np.float32)
        if image.shape != (self._height, self._width):
            raise ValueError(
                f"depth image shape {image.shape} does not match "
                f"{self._height}x{self._width}"
            )
        invalid = ~np.isfinite(image) | (image >= SKY_DEPTH_M)
        if invalid.any():
            image = image.copy()
            image[invalid] = np.nan
        return image

    # -- delivery ----------------------------------------------------------
    def poll_next(self, after_ns: int = 0) -> FramePair | None:
        """Oldest paired frame captured after ``after_ns``, without blocking."""
        with self._lock:
            self._raise_error_locked()
            common = sorted(self._rgb.keys() & self._depth.keys())
            for stamp in common:
                if stamp <= after_ns:
                    continue
                rgb_message, bgr = self._rgb.pop(stamp)
                depth_message = self._depth.pop(stamp)
                self._drop_older_locked(stamp)
                self._last_delivered_ns = stamp
                return FramePair(stamp, rgb_message, depth_message, bgr)
            return None

    def latest_pair(self) -> FramePair | None:
        """Newest paired frame, discarding any older unread frames."""
        with self._lock:
            self._raise_error_locked()
            common = self._rgb.keys() & self._depth.keys()
            if not common:
                return None
            stamp = max(common)
            rgb_message, bgr = self._rgb.pop(stamp)
            depth_message = self._depth.pop(stamp)
            self._drop_older_locked(stamp)
            self._last_delivered_ns = max(self._last_delivered_ns, stamp)
            return FramePair(stamp, rgb_message, depth_message, bgr)

    def _drop_older_locked(self, stamp: int) -> None:
        for buffer in (self._rgb, self._depth):
            for older in [value for value in buffer if value < stamp]:
                del buffer[older]

    def _raise_error_locked(self) -> None:
        if self._closed:
            raise ObservationError("camera topic stream is closed")
        if self._error is not None:
            raise ObservationError(f"camera topic stream failed: {self._error}")