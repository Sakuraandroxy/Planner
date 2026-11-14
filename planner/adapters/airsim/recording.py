from __future__ import annotations

import json
import threading
import time
from datetime import datetime
from pathlib import Path

from planner.domain.observation import Observation
from planner.ports.observation_source import ObservationSource


class CameraApiRecorder:
    """Continuously persist Camera API observations on a background thread."""

    def __init__(self, source: ObservationSource, output_root: str | Path, fps: float):
        if fps <= 0:
            raise ValueError("recording fps must be positive")
        self.source = source
        self.output_root = Path(output_root)
        self.fps = float(fps)
        self.run_dir: Path | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._frame_index = 0
        self._last_error: str | None = None

    @property
    def last_error(self) -> str | None:
        return self._last_error

    def start(self) -> Path:
        if self._thread and self._thread.is_alive():
            if self.run_dir is None:
                raise RuntimeError("recorder is running without an output directory")
            return self.run_dir
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = self.output_root / timestamp
        for name in ("rgb", "depth", "metadata"):
            (self.run_dir / name).mkdir(parents=True, exist_ok=True)
        self._stop_event.clear()
        self.capture_once()
        self._thread = threading.Thread(target=self._run, name="camera-api-recorder", daemon=True)
        self._thread.start()
        return self.run_dir

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=max(5.0, 2.0 / self.fps))
        self._thread = None

    def capture_once(self) -> Path:
        if self.run_dir is None:
            raise RuntimeError("recorder has not been started")
        observation = self.source.capture()
        self._save(observation, self._frame_index)
        self._frame_index += 1
        return self.run_dir

    def _run(self) -> None:
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
