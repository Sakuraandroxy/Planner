"""Save the first accepted target view for offline identity inspection."""

from __future__ import annotations

import io
import json
import re
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Sequence

from PIL import Image, ImageDraw


class TargetSnapshotRecorder:
    """Write one boxed image per stage/locked-instance pair when enabled.

    The output directory is created lazily, so constructing the recorder with
    its default ``enabled=False`` has no filesystem side effects.
    """

    def __init__(
        self,
        *,
        enabled: bool = False,
        output_root: str | Path | None = None,
        started_at: datetime | None = None,
    ) -> None:
        project_root = Path(__file__).resolve().parents[3]
        self.output_root = Path(output_root) if output_root is not None else project_root / "output"
        self.started_at = started_at or datetime.now()
        self._enabled = False
        self._run_directory: Path | None = None
        self._recorded_keys: set[tuple[int, str, str]] = set()
        self._task_sequence = 0
        self._task_text = ""
        self._lock = threading.RLock()
        if enabled:
            self.set_enabled(True)

    @property
    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    @property
    def run_directory(self) -> Path | None:
        with self._lock:
            return self._run_directory

    def set_enabled(self, enabled: bool) -> Path | None:
        """Enable/disable future writes and return the active run directory."""

        with self._lock:
            self._enabled = bool(enabled)
            if self._enabled:
                return self._ensure_run_directory()
            return self._run_directory

    def begin_task(self, task_text: str) -> int:
        """Start a new task namespace without creating output while disabled."""

        with self._lock:
            self._task_sequence += 1
            self._task_text = str(task_text or "").strip()
            return self._task_sequence

    def record_first(
        self,
        *,
        stage_key: Sequence[Any] | Any,
        stage_index: int | None,
        instance_id: str,
        target_name: str,
        view: str,
        image: Any,
        detection: Any,
        observer_world: Sequence[float] | None = None,
        observer_yaw_deg: float | None = None,
        source: str = "runtime",
    ) -> Path | None:
        """Save a boxed image once for this stage and locked target instance."""

        with self._lock:
            if not self._enabled or image is None or detection is None:
                return None
            if not bool(getattr(detection, "visible", False)):
                return None

            normalized_stage_key = self._stage_key_text(stage_key)
            normalized_instance_id = str(instance_id or "").strip()
            if not normalized_instance_id:
                return None
            record_key = (self._task_sequence, normalized_stage_key, normalized_instance_id)
            if record_key in self._recorded_keys:
                return None

            try:
                canvas = self._to_pil_image(image)
                bbox = self._validated_bbox(getattr(detection, "bbox", None), canvas.size)
                if bbox is None:
                    return None
                view_name = str(view or getattr(detection, "camera", "front") or "front").lower()
                self._draw_overlay(canvas, bbox, target_name, normalized_instance_id, view_name, detection)

                run_directory = self._ensure_run_directory()
                stage_number = int(stage_index) + 1 if stage_index is not None else 0
                filename = (
                    f"task_{self._task_sequence:02d}_stage_{stage_number:02d}_"
                    f"{self._safe_component(target_name, 'target')}_"
                    f"{self._safe_component(normalized_instance_id, 'instance')}_"
                    f"{self._safe_component(view_name, 'view')}.jpg"
                )
                destination = self._unique_destination(run_directory / filename)
                canvas.save(destination, format="JPEG", quality=92, subsampling=0)

                manifest_entry = {
                    "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                    "file": destination.name,
                    "stage_key": list(stage_key) if isinstance(stage_key, (list, tuple)) else stage_key,
                    "stage_index": stage_index,
                    "task_sequence": self._task_sequence,
                    "task": self._task_text,
                    "target": str(target_name or ""),
                    "instance_id": normalized_instance_id,
                    "view": view_name,
                    "bbox": bbox,
                    "score": float(getattr(detection, "score", 0.0) or 0.0),
                    "label": str(getattr(detection, "label", "") or ""),
                    "depth_median_m": self._optional_float(getattr(detection, "depth_median", None)),
                    "observer_world": (
                        [float(value) for value in observer_world[:3]]
                        if observer_world is not None
                        else None
                    ),
                    "observer_yaw_deg": self._optional_float(observer_yaw_deg),
                    "source": str(source or "runtime"),
                }
                with (run_directory / "manifest.jsonl").open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(manifest_entry, ensure_ascii=False) + "\n")
                self._recorded_keys.add(record_key)
                return destination
            except Exception as exc:
                # Diagnostics must never interrupt flight or change target state.
                print(f"  [TargetSnapshot] save_failed error={exc}")
                return None

    def _ensure_run_directory(self) -> Path:
        if self._run_directory is not None:
            return self._run_directory
        self.output_root.mkdir(parents=True, exist_ok=True)
        stem = self.started_at.strftime("%Y%m%d_%H%M")
        candidate = self.output_root / stem
        suffix = 2
        while candidate.exists():
            candidate = self.output_root / f"{stem}_{suffix:02d}"
            suffix += 1
        candidate.mkdir(parents=False, exist_ok=False)
        self._run_directory = candidate
        return candidate

    @staticmethod
    def _to_pil_image(image: Any) -> Image.Image:
        if isinstance(image, Image.Image):
            return image.convert("RGB").copy()
        if isinstance(image, (bytes, bytearray, memoryview)):
            with Image.open(io.BytesIO(bytes(image))) as decoded:
                return decoded.convert("RGB").copy()
        # Keep NumPy optional at import time; Image.fromarray accepts ordinary
        # uint8 arrays used by image pipelines.
        return Image.fromarray(image).convert("RGB")

    @staticmethod
    def _validated_bbox(bbox: Any, size: tuple[int, int]) -> list[int] | None:
        if bbox is None or len(bbox) < 4:
            return None
        try:
            raw = [float(value) for value in bbox[:4]]
        except (TypeError, ValueError):
            return None
        if not all(value == value and abs(value) != float("inf") for value in raw):
            return None
        width, height = size
        if width <= 1 or height <= 1:
            return None
        x1, x2 = sorted((int(round(raw[0])), int(round(raw[2]))))
        y1, y2 = sorted((int(round(raw[1])), int(round(raw[3]))))
        x1 = max(0, min(width - 1, x1))
        x2 = max(0, min(width - 1, x2))
        y1 = max(0, min(height - 1, y1))
        y2 = max(0, min(height - 1, y2))
        if x2 <= x1 or y2 <= y1:
            return None
        return [x1, y1, x2, y2]

    @staticmethod
    def _draw_overlay(
        canvas: Image.Image,
        bbox: list[int],
        target_name: str,
        instance_id: str,
        view: str,
        detection: Any,
    ) -> None:
        draw = ImageDraw.Draw(canvas)
        x1, y1, x2, y2 = bbox
        color = (255, 48, 48) if view == "front" else (40, 190, 255)
        line_width = max(3, int(round(min(canvas.size) / 180.0)))
        for offset in range(line_width):
            draw.rectangle(
                [
                    max(0, x1 - offset),
                    max(0, y1 - offset),
                    min(canvas.size[0] - 1, x2 + offset),
                    min(canvas.size[1] - 1, y2 + offset),
                ],
                outline=color,
            )
        label = (
            f"{target_name} | {instance_id} | {view} | "
            f"score={float(getattr(detection, 'score', 0.0) or 0.0):.2f}"
        )
        depth = TargetSnapshotRecorder._optional_float(getattr(detection, "depth_median", None))
        if depth is not None:
            label += f" | depth={depth:.1f}m"
        # Pillow's built-in fallback font is ASCII-only on some installations.
        label = label.encode("ascii", "replace").decode("ascii")
        text_left = max(0, min(x1, canvas.size[0] - 1))
        text_top = max(0, y1 - 18)
        text_right = min(canvas.size[0] - 1, text_left + max(120, 7 * len(label)))
        draw.rectangle([text_left, text_top, text_right, min(canvas.size[1] - 1, text_top + 16)], fill=(0, 0, 0))
        draw.text((text_left + 3, text_top + 2), label, fill=color)

    @staticmethod
    def _stage_key_text(stage_key: Sequence[Any] | Any) -> str:
        if isinstance(stage_key, (list, tuple)):
            return "|".join(str(value) for value in stage_key)
        return str(stage_key)

    @staticmethod
    def _safe_component(value: Any, fallback: str) -> str:
        text = re.sub(r"[^A-Za-z0-9._-]+", "_", str(value or "").strip()).strip("._-")
        return (text or fallback)[:80]

    @staticmethod
    def _unique_destination(destination: Path) -> Path:
        if not destination.exists():
            return destination
        counter = 2
        while True:
            candidate = destination.with_name(f"{destination.stem}_{counter:02d}{destination.suffix}")
            if not candidate.exists():
                return candidate
            counter += 1

    @staticmethod
    def _optional_float(value: Any) -> float | None:
        if value is None:
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if number == number and abs(number) != float("inf") else None
