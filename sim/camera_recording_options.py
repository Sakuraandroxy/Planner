"""Shared CLI/config handling for Camera API recording."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Optional


@dataclass(frozen=True)
class CameraRecordingOptions:
    enabled: bool = False
    mode: str = "off"
    camera_ids: tuple[str, ...] = ()
    record_fps: float = 5.0
    output_root: Optional[Path] = None
    frame_format: str = "jpg"
    jpeg_quality: int = 82
    video_codec: str = "mp4v"
    async_queue_size: int = 64
    drop_oldest_when_full: bool = True
    event_pre_seconds: float = 2.0
    event_post_seconds: float = 3.0
    max_disk_gb: float = 5.0
    min_free_disk_gb: float = 2.0
    max_duration_minutes: float = 60.0
    save_metadata: bool = True
    save_depth_preview: bool = False
    save_raw_depth: bool = False

    def with_enabled(self, enabled: bool, *, mode: str | None = None):
        selected_mode = str(mode or self.mode or "video").lower()
        if enabled and selected_mode == "off":
            selected_mode = "video"
        return replace(self, enabled=bool(enabled), mode=selected_mode if enabled else "off")


def add_camera_recording_arguments(parser: argparse.ArgumentParser) -> None:
    boolean_action = getattr(argparse, "BooleanOptionalAction", None)
    if boolean_action is not None:
        parser.add_argument(
            "--record-camera-api",
            action=boolean_action,
            default=None,
            help="录制Camera API真实画面；可用--no-record-camera-api显式关闭",
        )
    else:
        group = parser.add_mutually_exclusive_group()
        group.add_argument("--record-camera-api", dest="record_camera_api", action="store_true")
        group.add_argument("--no-record-camera-api", dest="record_camera_api", action="store_false")
        parser.set_defaults(record_camera_api=None)
    parser.add_argument("--camera-record-mode", choices=("frames", "video", "events"), default=None)
    parser.add_argument("--camera-record-ids", default=None, help="逗号分隔的AirSim camera id")
    parser.add_argument("--camera-record-fps", type=float, default=None)
    parser.add_argument("--camera-record-output", default=None)
    parser.add_argument("--camera-record-max-gb", type=float, default=None)
    parser.add_argument("--camera-record-jpeg-quality", type=int, default=None)
    parser.add_argument("--save-camera-depth-preview", action="store_true", default=None)
    parser.add_argument("--save-camera-raw-depth", action="store_true", default=None)


def _camera_observer_config(config: dict) -> dict:
    functions = dict(config.get("FUNCTIONS", {}) or {})
    return dict(
        functions.get("CAMERA_API_OBSERVER")
        or config.get("CAMERA_API_OBSERVER")
        or {}
    )


def _camera_ids(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        values = value.split(",")
    else:
        values = list(value or [])
    return tuple(dict.fromkeys(str(item).strip() for item in values if str(item).strip()))


def resolve_camera_recording_options(args, config: dict) -> CameraRecordingOptions:
    rcfg = _camera_observer_config(config)
    cli_enabled = getattr(args, "record_camera_api", None)
    enabled = bool(rcfg.get("RECORDING_ENABLED", False)) if cli_enabled is None else bool(cli_enabled)
    cli_mode = getattr(args, "camera_record_mode", None)
    mode = str(cli_mode or rcfg.get("RECORDING_MODE", "off")).strip().lower()
    if enabled and mode == "off":
        mode = "video"
    if not enabled:
        mode = "off"
    cli_ids = getattr(args, "camera_record_ids", None)
    camera_ids = _camera_ids(cli_ids if cli_ids is not None else rcfg.get("CAMERA_IDS", []))
    output_value = getattr(args, "camera_record_output", None) or rcfg.get("OUTPUT_ROOT")
    return CameraRecordingOptions(
        enabled=enabled,
        mode=mode,
        camera_ids=camera_ids,
        record_fps=max(0.1, float(getattr(args, "camera_record_fps", None) or rcfg.get("RECORD_FPS", 5.0))),
        output_root=Path(output_value) if output_value else None,
        frame_format=str(rcfg.get("FRAME_FORMAT", "jpg") or "jpg").lower(),
        jpeg_quality=max(1, min(100, int(getattr(args, "camera_record_jpeg_quality", None) or rcfg.get("JPEG_QUALITY", 82)))),
        video_codec=str(rcfg.get("VIDEO_CODEC", "mp4v") or "mp4v"),
        async_queue_size=max(1, int(rcfg.get("ASYNC_QUEUE_SIZE", 64))),
        drop_oldest_when_full=bool(rcfg.get("DROP_OLDEST_WHEN_FULL", True)),
        event_pre_seconds=max(0.0, float(rcfg.get("EVENT_PRE_SECONDS", 2.0))),
        event_post_seconds=max(0.0, float(rcfg.get("EVENT_POST_SECONDS", 3.0))),
        max_disk_gb=max(0.0, float(getattr(args, "camera_record_max_gb", None) or rcfg.get("MAX_DISK_GB", 5.0))),
        min_free_disk_gb=max(0.0, float(rcfg.get("MIN_FREE_DISK_GB", 2.0))),
        max_duration_minutes=max(0.0, float(rcfg.get("MAX_DURATION_MINUTES", 60.0))),
        save_metadata=bool(rcfg.get("SAVE_METADATA", True)),
        save_depth_preview=bool(
            rcfg.get("SAVE_DEPTH_PREVIEW", False)
            if getattr(args, "save_camera_depth_preview", None) is None
            else getattr(args, "save_camera_depth_preview")
        ),
        save_raw_depth=bool(
            rcfg.get("SAVE_RAW_DEPTH", False)
            if getattr(args, "save_camera_raw_depth", None) is None
            else getattr(args, "save_camera_raw_depth")
        ),
    )

