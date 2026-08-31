"""Non-blocking recorder for frames already captured through the AirSim Camera API."""

from __future__ import annotations

import json
import queue
import shutil
import threading
import time
from dataclasses import replace
from io import BytesIO
from pathlib import Path
from typing import Optional

from sim.camera_frame_hub import CameraFrameHub
from sim.camera_frames import CameraFrame, MultiCameraSnapshot
from sim.camera_recording_options import CameraRecordingOptions


class CameraApiRecorder:
    """Record CameraFrameHub snapshots without adding AirSim RPC traffic."""

    _STOP = object()

    def __init__(
        self,
        frame_hub: CameraFrameHub,
        options: CameraRecordingOptions | None = None,
        *,
        state=None,
    ):
        self.frame_hub = frame_hub
        self.options = options or CameraRecordingOptions()
        self.state = state
        self._lock = threading.RLock()
        self._queue: queue.Queue = queue.Queue(maxsize=max(1, int(self.options.async_queue_size)))
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._started_at = 0.0
        self._run_directory: Optional[Path] = None
        self._manifest_handles: dict[str, object] = {}
        self._video_writers: dict[str, object] = {}
        self._video_sizes: dict[str, tuple[int, int]] = {}
        self._last_enqueued_at: dict[str, float] = {}
        self._seen_keys: set[tuple[str, str, int, str]] = set()
        self._seen_order: list[tuple[str, str, int, str]] = []
        self._event_name: Optional[str] = None
        self._event_until = 0.0
        self._written_frames = 0
        self._written_bytes = 0
        self._dropped_frames = 0
        self._last_error = ""
        self._stopped_reason = ""

    @property
    def run_directory(self) -> Optional[Path]:
        with self._lock:
            return self._run_directory

    def start(
        self,
        options: CameraRecordingOptions | None = None,
        *,
        mode: str | None = None,
        camera_ids=None,
    ) -> dict:
        """Start recording; the output directory is created on the first frame."""
        with self._lock:
            if self._running:
                return self.status()
            selected = options or self.options
            selected_mode = str(mode or selected.mode or "video").strip().lower()
            if selected_mode == "off":
                selected_mode = "video"
            if selected_mode not in {"frames", "video", "events"}:
                raise ValueError(f"unsupported Camera API recording mode: {selected_mode}")
            ids = tuple(str(item).strip() for item in (camera_ids or selected.camera_ids) if str(item).strip())
            self.options = replace(selected, enabled=True, mode=selected_mode, camera_ids=ids)
            self._queue = queue.Queue(maxsize=max(1, int(self.options.async_queue_size)))
            self._running = True
            self._started_at = time.perf_counter()
            self._run_directory = None
            self._last_enqueued_at.clear()
            self._seen_keys.clear()
            self._seen_order.clear()
            self._event_name = None
            self._event_until = 0.0
            self._written_frames = 0
            self._written_bytes = 0
            self._dropped_frames = 0
            self._last_error = ""
            self._stopped_reason = ""
            self.frame_hub.subscribe(self._on_snapshot)
            self._thread = threading.Thread(target=self._worker, name="camera-api-recorder", daemon=True)
            self._thread.start()
        self._publish_status()
        return self.status()

    def stop(self, *, reason: str = "requested", drain: bool = True) -> dict:
        with self._lock:
            if not self._running and self._thread is None:
                return self.status()
            self._running = False
            self._stopped_reason = str(reason or "requested")
            self.frame_hub.unsubscribe(self._on_snapshot)
            if not drain:
                self._discard_pending_locked()
            thread = self._thread
            self._put_control(self._STOP)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=5.0)
        with self._lock:
            self._thread = None
            self._close_outputs_locked()
            self.options = self.options.with_enabled(False)
        self._publish_status()
        return self.status()

    def trigger_event(self, name: str = "event") -> dict:
        with self._lock:
            if not self._running:
                self.start(mode="events")
            elif self.options.mode != "events":
                return {**self.status(), "triggered": False, "reason": "mode_is_not_events"}
            event_stamp = time.strftime("%Y%m%d_%H%M%S")
            safe_name = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(name or "event"))
            self._event_name = f"{event_stamp}_{safe_name}"
            self._event_until = time.perf_counter() + max(0.0, float(self.options.event_post_seconds))
            event_name = self._event_name
        for snapshot in self.frame_hub.history(self.options.event_pre_seconds):
            self._enqueue_snapshot(snapshot, event_name=event_name, bypass_rate=True)
        result = self.status()
        result.update({"triggered": True, "event_name": event_name})
        return result

    def status(self) -> dict:
        with self._lock:
            elapsed = time.perf_counter() - self._started_at if self._started_at else 0.0
            return {
                "enabled": bool(self._running),
                "mode": self.options.mode if self._running else "off",
                "camera_ids": list(self.options.camera_ids),
                "record_fps": float(self.options.record_fps),
                "output_directory": str(self._run_directory) if self._run_directory else "",
                "queue_size": int(self._queue.qsize()),
                "queue_capacity": int(self._queue.maxsize),
                "written_frames": int(self._written_frames),
                "written_bytes": int(self._written_bytes),
                "dropped_frames": int(self._dropped_frames),
                "elapsed_s": round(max(0.0, elapsed), 3),
                "active_event": self._event_name or "",
                "last_error": self._last_error,
                "stopped_reason": self._stopped_reason,
            }

    def _publish_status(self) -> None:
        if self.state is not None and hasattr(self.state, "set_camera_recording_status"):
            self.state.set_camera_recording_status(self.status())

    def _on_snapshot(self, snapshot: MultiCameraSnapshot) -> None:
        with self._lock:
            if not self._running:
                return
            event_name = None
            if self.options.mode == "events":
                if self._event_name is None or time.perf_counter() > self._event_until:
                    self._event_name = None
                    return
                event_name = self._event_name
        self._enqueue_snapshot(snapshot, event_name=event_name)

    def _enqueue_snapshot(
        self,
        snapshot: MultiCameraSnapshot,
        *,
        event_name: str | None = None,
        bypass_rate: bool = False,
    ) -> None:
        now = time.perf_counter()
        allowed = set(self.options.camera_ids)
        for camera_id, frame in snapshot.frames.items():
            if allowed and camera_id not in allowed:
                continue
            key = (event_name or "continuous", camera_id, int(frame.timestamp_ns), frame.capture_id)
            with self._lock:
                if key in self._seen_keys:
                    continue
                minimum_period = 1.0 / max(0.1, float(self.options.record_fps))
                last = self._last_enqueued_at.get(camera_id, -1e9)
                if not bypass_rate and now - last < minimum_period:
                    continue
                self._remember_key_locked(key)
                self._last_enqueued_at[camera_id] = now
            self._put_frame((frame, event_name))

    def _remember_key_locked(self, key) -> None:
        self._seen_keys.add(key)
        self._seen_order.append(key)
        if len(self._seen_order) > 4096:
            oldest = self._seen_order.pop(0)
            self._seen_keys.discard(oldest)

    def _put_frame(self, item) -> None:
        try:
            self._queue.put_nowait(item)
            return
        except queue.Full:
            pass
        with self._lock:
            self._dropped_frames += 1
        if not self.options.drop_oldest_when_full:
            return
        try:
            self._queue.get_nowait()
            self._queue.task_done()
            self._queue.put_nowait(item)
        except (queue.Empty, queue.Full):
            return

    def _put_control(self, item) -> None:
        while True:
            try:
                self._queue.put_nowait(item)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                    self._queue.task_done()
                except queue.Empty:
                    return

    def _discard_pending_locked(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                return

    def _worker(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is self._STOP:
                    return
                frame, event_name = item
                if self._limits_exceeded():
                    with self._lock:
                        self._running = False
                        self.frame_hub.unsubscribe(self._on_snapshot)
                    continue
                self._write_frame(frame, event_name)
            except Exception as exc:
                with self._lock:
                    self._last_error = f"{type(exc).__name__}: {exc}"
                print(f"[CameraApiRecorder] {self._last_error}")
            finally:
                self._queue.task_done()

    def _limits_exceeded(self) -> bool:
        with self._lock:
            if self.options.max_duration_minutes > 0:
                elapsed = time.perf_counter() - self._started_at
                if elapsed >= self.options.max_duration_minutes * 60.0:
                    self._stopped_reason = "max_duration"
                    return True
            if self.options.max_disk_gb > 0 and self._written_bytes >= self.options.max_disk_gb * 1024**3:
                self._stopped_reason = "max_disk"
                return True
            root = self._run_directory or self._base_output_directory()
            probe = root if root.exists() else root.parent
            try:
                if self.options.min_free_disk_gb > 0 and shutil.disk_usage(probe).free < self.options.min_free_disk_gb * 1024**3:
                    self._stopped_reason = "min_free_disk"
                    return True
            except OSError:
                pass
        return False

    def _base_output_directory(self) -> Path:
        if self.options.output_root is not None:
            return Path(self.options.output_root)
        project_root = Path(__file__).resolve().parents[1]
        return project_root / "output" / time.strftime("%Y%m%d_%H%M") / "camera_api"

    def _ensure_run_directory(self) -> Path:
        with self._lock:
            if self._run_directory is None:
                self._run_directory = self._base_output_directory()
                self._run_directory.mkdir(parents=True, exist_ok=True)
            return self._run_directory

    @staticmethod
    def _safe_camera_id(camera_id: str) -> str:
        return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(camera_id)) or "camera"

    def _camera_directory(self, frame: CameraFrame, event_name: str | None) -> Path:
        root = self._ensure_run_directory()
        if event_name:
            root = root / "events" / event_name
        path = root / self._safe_camera_id(frame.camera_id)
        path.mkdir(parents=True, exist_ok=True)
        return path

    def _frame_stem(self, frame: CameraFrame) -> str:
        stamp = int(frame.timestamp_ns or time.time_ns())
        capture = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in frame.capture_id)
        return f"{stamp}_{capture}"

    def _encode_rgb(self, frame: CameraFrame, image_format: str) -> bytes:
        fmt = str(image_format or "jpg").lower()
        if fmt in {"jpg", "jpeg"} and frame.rgb_jpeg:
            return frame.rgb_jpeg
        if fmt == "png" and frame.rgb_png:
            return frame.rgb_png
        if frame.rgb is None:
            return b""
        buffer = BytesIO()
        if fmt in {"jpg", "jpeg"}:
            frame.rgb.save(buffer, format="JPEG", quality=int(self.options.jpeg_quality))
        else:
            frame.rgb.save(buffer, format="PNG")
        return buffer.getvalue()

    def _write_frame(self, frame: CameraFrame, event_name: str | None) -> None:
        if frame.rgb is None and not frame.rgb_jpeg and not frame.rgb_png:
            return
        camera_dir = self._camera_directory(frame, event_name)
        if self.options.mode == "video" and not event_name:
            self._write_video_frame(frame, camera_dir)
        else:
            extension = "jpg" if self.options.frame_format in {"jpg", "jpeg"} else "png"
            payload = self._encode_rgb(frame, extension)
            if payload:
                path = camera_dir / f"{self._frame_stem(frame)}.{extension}"
                path.write_bytes(payload)
                self._count_write(len(payload))
        if self.options.save_depth_preview and frame.depth is not None:
            self._write_depth_preview(frame, camera_dir)
        if self.options.save_raw_depth and frame.depth is not None:
            import numpy as np

            path = camera_dir / f"{self._frame_stem(frame)}_depth.npy"
            np.save(path, frame.depth)
            self._count_write(path.stat().st_size)
        if self.options.save_metadata:
            self._write_manifest(frame, camera_dir, event_name)
        with self._lock:
            self._written_frames += 1
        self._publish_status()

    def _write_video_frame(self, frame: CameraFrame, camera_dir: Path) -> None:
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("video mode requires opencv-python; use frames mode if OpenCV is unavailable") from exc
        if frame.rgb is None:
            from PIL import Image

            frame.rgb = Image.open(BytesIO(self._encode_rgb(frame, "jpg"))).convert("RGB")
        rgb = np.asarray(frame.rgb.convert("RGB"))
        height, width = rgb.shape[:2]
        camera_id = frame.camera_id
        writer = self._video_writers.get(camera_id)
        if writer is None:
            fourcc = cv2.VideoWriter_fourcc(*str(self.options.video_codec or "mp4v")[:4])
            video_path = camera_dir / "camera_api.mp4"
            writer = cv2.VideoWriter(str(video_path), fourcc, float(self.options.record_fps), (width, height))
            if not writer.isOpened():
                raise RuntimeError(f"failed to open video writer: {video_path}")
            self._video_writers[camera_id] = writer
            self._video_sizes[camera_id] = (width, height)
        target_size = self._video_sizes[camera_id]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if (width, height) != target_size:
            bgr = cv2.resize(bgr, target_size)
        writer.write(bgr)
        self._count_write(max(1, width * height // 8))

    def _write_depth_preview(self, frame: CameraFrame, camera_dir: Path) -> None:
        import numpy as np
        from PIL import Image

        depth = np.asarray(frame.depth, dtype=np.float32)
        valid = np.isfinite(depth) & (depth > 0)
        if not valid.any():
            return
        lo, hi = np.percentile(depth[valid], [2, 98])
        if hi <= lo:
            hi = lo + 1.0
        normalized = np.zeros(depth.shape, dtype=np.uint8)
        normalized[valid] = np.clip((depth[valid] - lo) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)
        buffer = BytesIO()
        Image.fromarray(normalized, mode="L").save(buffer, format="PNG")
        payload = buffer.getvalue()
        path = camera_dir / f"{self._frame_stem(frame)}_depth.png"
        path.write_bytes(payload)
        self._count_write(len(payload))

    def _write_manifest(self, frame: CameraFrame, camera_dir: Path, event_name: str | None) -> None:
        key = str(camera_dir.resolve())
        handle = self._manifest_handles.get(key)
        if handle is None:
            handle = (camera_dir / "manifest.jsonl").open("a", encoding="utf-8")
            self._manifest_handles[key] = handle
        metadata = frame.to_metadata_dict()
        metadata["event_name"] = event_name or ""
        line = json.dumps(metadata, ensure_ascii=False, separators=(",", ":")) + "\n"
        handle.write(line)
        handle.flush()
        self._count_write(len(line.encode("utf-8")))

    def _count_write(self, byte_count: int) -> None:
        with self._lock:
            self._written_bytes += max(0, int(byte_count))

    def _close_outputs_locked(self) -> None:
        for writer in self._video_writers.values():
            try:
                writer.release()
            except Exception:
                pass
        self._video_writers.clear()
        self._video_sizes.clear()
        for handle in self._manifest_handles.values():
            try:
                handle.close()
            except Exception:
                pass
        self._manifest_handles.clear()

