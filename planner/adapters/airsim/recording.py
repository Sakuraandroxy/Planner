from __future__ import annotations

import json
import math
import threading
import time
from concurrent.futures import Future
from datetime import datetime
from pathlib import Path

from planner.domain.observation import Observation
from planner.ports.observation_source import ObservationSource


class CameraApiRecorder:
    """Continuously persist Camera API observations on a background thread."""

    def __init__(self, source: ObservationSource, output_root: str | Path, fps: float):
        if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not math.isfinite(fps) or fps <= 0:
            raise ValueError("recording fps must be finite and positive")
        self.source = source
        self.output_root = Path(output_root)
        self.fps = float(fps)
        self.run_dir: Path | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._frame_index = 0
        self._last_error: str | None = None
        self._state_lock = threading.RLock()
        self._capture_lock = threading.Lock()
        self._running = False
        self._closed = False
        self._on_stopped = None
        self._started: Future | None = None

    @property
    def last_error(self) -> str | None:
        return self._last_error

    def start(self, timeout_s: float = 10.0) -> Path:
        if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("start timeout must be finite and positive")
        with self._state_lock:
            if self._closed:
                raise RuntimeError("recorder is closed")
            if self._thread and self._thread.is_alive():
                if self._stop_event.is_set():
                    raise RuntimeError("recorder is still stopping; restart is prohibited")
            else:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                self.run_dir = self.output_root / timestamp
                for name in ("rgb", "depth", "metadata"):
                    (self.run_dir / name).mkdir(parents=True, exist_ok=False)
                self._stop_event = threading.Event()  # Never revive an earlier run's event.
                self._frame_index = 0
                self._last_error = None
                self._started = Future()
                self._running = True
                self._thread = threading.Thread(target=self._run, name="camera-api-recorder", daemon=True)
                try:
                    self._thread.start()
                except Exception:
                    self._running = False
                    self._stop_event.set()
                    self._thread = None
                    raise
            started = self._started
        # Do not hold lifecycle locks during RPC / first-frame wait.
        try:
            return started.result(timeout=timeout_s)
        except TimeoutError:
            self.stop(timeout_s=0)
            raise

    def stop(self, timeout_s: float | None = None) -> bool:
        timeout = max(5.0, 2.0 / self.fps) if timeout_s is None else timeout_s
        if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout < 0:
            raise ValueError("stop timeout must be finite and nonnegative")
        with self._state_lock:
            self._stop_event.set()
            thread = self._thread
        if thread:
            if thread is threading.current_thread():
                return False
            thread.join(timeout=timeout)
            # Keep the reference even on timeout; start() cannot create a rival.
            return not thread.is_alive()
        return True

    def close(self, on_stopped=None, timeout_s: float | None = None) -> bool:
        """Quarantine slow capture; close its transport only after capture has ended."""
        callback = None
        with self._state_lock:
            if not self._closed:
                self._closed = True
                if self._running:
                    self._on_stopped = on_stopped
                else:
                    callback = on_stopped
        if callback is not None:
            callback()
        return self.stop(timeout_s)

    def capture_once(self) -> Path:
        with self._capture_lock:
            if self.run_dir is None:
                raise RuntimeError("recorder has not been started")
            if threading.current_thread() is not self._thread:
                raise RuntimeError("only the recording worker may capture frames")
            if self._stop_event.is_set():
                raise RuntimeError("recorder is stopping")
            observation = self.source.capture()
            self._save(observation, self._frame_index)
            self._frame_index += 1
            return self.run_dir

    def _run(self) -> None:
        try:
            self._started.set_result(self.capture_once())
            self._record_loop()
        except Exception as exc:
            self._last_error = str(exc)
            self._stop_event.set()
            if not self._started.done():
                self._started.set_exception(exc)
        finally:
            with self._state_lock:
                self._running = False
                callback, self._on_stopped = self._on_stopped, None
            if callback is not None:
                try:
                    callback()
                except Exception as exc:
                    self._last_error = f"recording transport cleanup failed: {type(exc).__name__}"

    def _record_loop(self) -> None:
        interval = 1.0 / self.fps
        deadline = time.monotonic() + interval
        while not self._stop_event.is_set():
            wait_s = max(0.0, deadline - time.monotonic())
            if self._stop_event.wait(wait_s):
                break
            try:
                self.capture_once()
                self._last_error = None
            except Exception as exc:
                if self._stop_event.is_set():
                    break
                self._last_error = str(exc)
            deadline += interval
            if deadline < time.monotonic():
                deadline = time.monotonic()

    def _save(self, observation: Observation, index: int) -> None:
        if self.run_dir is None:
            raise RuntimeError("recorder has no output directory")
        stem = f"{index:06d}"
        observation.rgb.save(self.run_dir / "rgb" / f"{stem}.jpg", format="JPEG", quality=92)
        observation.depth_image.save(self.run_dir / "depth" / f"{stem}.png", format="PNG")
        metadata = _metadata(observation, index)
        (self.run_dir / "metadata" / f"{stem}.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )


def _metadata(observation: Observation, index: int) -> dict:
    pose = observation.vehicle_pose
    intrinsics = observation.intrinsics
    return {
        "frame_index": index,
        "timestamp_ns": observation.timestamp_ns,
        "depth_timestamp_ns": observation.depth_timestamp_ns,
        "depth_mapping": observation.depth_mapping,
        "depth_display_range_m": [observation.depth_min_m, observation.depth_max_m],
        "camera_id": observation.camera_id,
        "vehicle_pose": {
            "x": pose.x, "y": pose.y, "z": pose.z, "yaw_deg": pose.yaw_deg,
        },
        "camera_position_world": list(observation.camera_position_world),
        "rotation_camera_to_world": [list(row) for row in observation.rotation_camera_to_world],
        "intrinsics": {
            "width": intrinsics.width, "height": intrinsics.height,
            "fx": intrinsics.fx, "fy": intrinsics.fy,
            "cx": intrinsics.cx, "cy": intrinsics.cy,
            "horizontal_fov_deg": intrinsics.horizontal_fov_deg,
        },
    }
