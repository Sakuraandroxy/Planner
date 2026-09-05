"""Completion observation and detection helpers for the fast-slow runtime.

This module owns camera capture, dual-view detector coordination, and fresh
completion observations. Stage transitions, queues, and flight execution remain
in runtime.py.
"""

from __future__ import annotations

import io
import math
import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any

from config import cfg

from agent.functions.common import web_runtime_helpers as web_helpers
from agent.functions.common.config_access import function_section
from agent.functions.common.detection_policy import (
    detection_caption_for_stage,
    detection_reliability as shared_detection_reliability,
)
from agent.functions.completion import CompletionResult
from agent.functions.fast_slow.completion_pipeline import DetectionDepthBundle
from agent.functions.fast_slow.target_runtime import (
    _detection_matches_view_relative_sector,
    _identity_approved_detections,
    _record_locked_target_snapshot,
)
from agent.functions.memory import is_view_relative_stage
from agent.functions.perception import attach_detection_camera_context

if TYPE_CHECKING:
    from agent.functions.fast_slow.runtime_context import RuntimeObjects
else:
    RuntimeObjects = Any


def _debug_logs_enabled() -> bool:
    fast_slow_cfg = {**(cfg.get("FAST_SLOW", {}) or {}), **function_section(cfg, "FAST_SLOW")}
    return bool(fast_slow_cfg.get("DEBUG_LOGS", False))


def _debug_print(message: str) -> None:
    if _debug_logs_enabled():
        print(message)

def _capture_rgb_for_stage(client, completion_checker, current_stage, capture_mode: str):
    """Capture only RGB frames for the fast loop.

    Depth is requested only when a completion check is actually due.  This is
    the key difference from the old per-step completion capture.
    """
    profile = getattr(completion_checker, "rgb_profile", None) or "front_down"
    if current_stage and getattr(current_stage, "mode", "") == "detect":
        profile = getattr(completion_checker, "rgb_profile", profile)
    t0 = time.perf_counter()
    frame, down_frame, front_depth, down_depth, timing = client.capture_views(
        profile=profile,
        mode=capture_mode,
        verbose=False,
    )
    elapsed = time.perf_counter() - t0
    timing = dict(timing or {})
    timing.setdefault("total_s", elapsed)
    _debug_print(
        f"  [CaptureRGB] profile={profile} time={timing.get('total_s', elapsed):.2f}s  "
        f"front={web_helpers.shape_text(frame)}  down={web_helpers.shape_text(down_frame)}"
    )
    return frame, down_frame, front_depth, down_depth, timing


def _capture_completion_depth(client, completion_checker, capture_mode: str):
    depth_profile = web_helpers.depth_only_profile(getattr(completion_checker, "depth_profile", "front_down_both_depth"))
    t0 = time.perf_counter()
    _front, _down, front_depth, down_depth, timing = web_helpers.capture_profile_isolated(client, depth_profile)
    elapsed = time.perf_counter() - t0
    timing = dict(timing or {})
    timing.setdefault("total_s", elapsed)
    _debug_print(
        f"  [Depth] profile={depth_profile} time={timing.get('total_s', elapsed):.2f}s  "
        f"front_depth={web_helpers.shape_text(front_depth)}  down_depth={web_helpers.shape_text(down_depth)}"
    )
    return front_depth, down_depth, timing


def _publish_frames(state, client, frame, down_frame, depth_meters) -> None:
    web_helpers.push_pil_png_to_frontend(state, frame, view="front")
    if down_frame is not None:
        web_helpers.push_pil_png_to_frontend(state, down_frame, view="down")
    else:
        state.set_down_frame(b"")
    if depth_meters is not None:
        preview = client.depth_meters_to_image(depth_meters)
        if preview:
            buf = io.BytesIO()
            preview.save(buf, format="PNG")
            state.set_depth_frame(buf.getvalue())


def _evaluate_completion(objects: RuntimeObjects, stage, task_text, frame, down_frame, front_depth, down_depth):
    return objects.completion_checker.evaluate(
        stage,
        task_text,
        frame,
        down_frame,
        front_depth_meters=front_depth,
        down_depth_meters=down_depth,
    )


def _caption_for_stage(stage, fallback_instruction: str) -> str:
    return detection_caption_for_stage(stage, fallback_instruction)


































def _detection_reliability(stage: Any, detection: Any, image: Any = None) -> float:
    return shared_detection_reliability(stage, detection, image)


def _suppress_unreliable_detection(stage: Any, detection: Any, image: Any = None) -> None:
    """Normalize detector output before it can drive distance, VLM, or target-loss state."""
    if detection is None or not getattr(detection, "visible", False):
        return
    if _detection_reliability(stage, detection, image) > 0.0:
        return
    try:
        detection.score = 0.0
        detection.visible = False
    except Exception:
        pass


def _select_reliable_detection(stage: Any, front_det: Any, down_det: Any, frame: Any, down_frame: Any):
    candidates = []
    for det, image in ((front_det, frame), (down_det, down_frame)):
        reliability = _detection_reliability(stage, det, image)
        if reliability > 0.0:
            candidates.append((reliability, det))
    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0])[1]


def _detect_dual_view(
    objects: RuntimeObjects,
    stage,
    task_text,
    frame,
    down_frame,
    *,
    detect_executor: ThreadPoolExecutor | None = None,
):
    caption = _caption_for_stage(stage, task_text)

    def detect_one(image, camera_name: str):
        if image is None:
            return []
        if hasattr(objects.detector, "detect_all"):
            return list(objects.detector.detect_all(image, caption, depth_meters=None, camera_name=camera_name) or [])
        result = objects.detector.detect(image, caption, depth_meters=None, camera_name=camera_name)
        return [result] if result and result.visible else []

    t0 = time.perf_counter()
    executor = detect_executor or objects.detect_executor
    front_future = executor.submit(detect_one, frame, "front")
    down_future = executor.submit(detect_one, down_frame, "down")
    try:
        front_all = front_future.result()
        down_all = down_future.result()
    except Exception:
        # Avoid leaving a queued second-view request behind after the first
        # request fails, especially in the single-worker future-scan pool.
        front_future.cancel()
        down_future.cancel()
        raise
    for det in front_all:
        attach_detection_camera_context(det, frame)
        _suppress_unreliable_detection(stage, det, frame)
    for det in down_all:
        attach_detection_camera_context(det, down_frame)
        _suppress_unreliable_detection(stage, det, down_frame)
    enforce_view_relative_sector = bool(
        is_view_relative_stage(stage)
        and getattr(objects, "mission_memory", None) is not None
        and objects.mission_memory.primary_instance(stage) is None
    )
    front_det = _best_detection_from_list(
        stage,
        front_all,
        frame,
        camera_name="front",
        enforce_view_relative_sector=enforce_view_relative_sector,
    )
    down_det = _best_detection_from_list(
        stage,
        down_all,
        down_frame,
        camera_name="down",
        enforce_view_relative_sector=enforce_view_relative_sector,
    )
    elapsed = time.perf_counter() - t0
    visible = [d for d in (front_det, down_det) if d and d.visible]
    best = _select_reliable_detection(stage, front_det, down_det, frame, down_frame) if visible else None

    def describe(name, det):
        if det and det.visible:
            return f"{name}:bbox={det.bbox} score={float(det.score or 0.0):.2f}"
        if det and det.score:
            return f"{name}:not_visible score={float(det.score or 0.0):.2f}"
        return f"{name}:not_visible"

    front_rel = _detection_reliability(stage, front_det, frame)
    down_rel = _detection_reliability(stage, down_det, down_frame)
    _debug_print(
        f"  [DetectRGB] {describe('front', front_det)}  {describe('down', down_det)}  "
        f"best={(best.camera if best else 'none')} "
        f"front_rel={front_rel:.3f} down_rel={down_rel:.3f} time={elapsed:.2f}s"
    )
    return best, front_det, down_det, elapsed, front_all, down_all


def _best_detection_from_list(
    stage: Any,
    detections: list,
    image: Any,
    *,
    camera_name: str,
    enforce_view_relative_sector: bool = False,
):
    from agent.models.detection.base import DetectionResult

    visible = [
        detection for detection in list(detections or [])
        if detection and getattr(detection, "visible", False)
        and _detection_reliability(stage, detection, image) > 0.0
        and (
            not enforce_view_relative_sector
            or _detection_matches_view_relative_sector(stage, detection, image, camera_name=camera_name)
        )
    ]
    if not visible:
        return DetectionResult(visible=False, camera=camera_name)
    return max(visible, key=lambda d: float(getattr(d, "score", 0.0) or 0.0))


def _evaluate_completion_fast_slow(
    objects: RuntimeObjects,
    client,
    stage,
    task_text,
    frame,
    down_frame,
    capture_mode: str,
):
    checker = objects.completion_checker
    started = time.perf_counter()

    if not checker.should_check_stage(stage):
        return None

    if getattr(checker, "uses_detector", False) and checker.is_detector_enabled():
        best_det, front_det, down_det, detect_elapsed, front_all, down_all = _detect_dual_view(
            objects,
            stage,
            task_text,
            frame,
            down_frame,
        )
        if best_det is None or not getattr(best_det, "visible", False):
            return CompletionResult(
                checked=True,
                done=False,
                target_detected=False,
                accepted_view="none",
                reason="fresh target not detected",
                elapsed=time.perf_counter() - started,
            )
        front_depth, down_depth, depth_timing = _capture_completion_depth(
            client,
            checker,
            capture_mode,
        )
        if front_depth is not None or down_depth is not None:
            _debug_print(
                "  [TargetDepth] "
                + web_helpers.target_depth_text("front", front_det, frame, front_depth)
                + "  "
                + web_helpers.target_depth_text("down", down_det, down_frame, down_depth)
            )
            _attach_depth_to_detection_lists(front_all, down_all, frame, down_frame, front_depth, down_depth)
            _update_memory_from_fresh_detection(
                objects,
                client,
                stage,
                frame=frame,
                down_frame=down_frame,
                front_all=front_all,
                down_all=down_all,
            )
        completion = checker.evaluate_with_detection(
            stage,
            task_text,
            frame,
            down_frame,
            best_det,
            front_detection=front_det,
            down_detection=down_det,
            front_depth_meters=front_depth,
            down_depth_meters=down_depth,
        )
        completion.elapsed = time.perf_counter() - started
        return completion

    if getattr(checker, "name", "") == "api_completion" and hasattr(checker, "analyze_rgb"):
        analysis = checker.analyze_rgb(stage, task_text, frame, down_frame)
        if not analysis or analysis.get("target_detected", False):
            front_depth, down_depth, _depth_timing = _capture_completion_depth(
                client,
                checker,
                capture_mode,
            )
        else:
            front_depth = down_depth = None
        completion = checker.finalize_analysis(
            stage,
            analysis,
            frame,
            down_frame,
            front_depth_meters=front_depth,
            down_depth_meters=down_depth,
        )
        completion.elapsed = time.perf_counter() - started
        return completion

    completion = _evaluate_completion(objects, stage, task_text, frame, down_frame, None, None)
    completion.elapsed = time.perf_counter() - started
    return completion


def _capture_fresh_completion_frames(client, completion_checker, capture_mode: str):
    profile = getattr(completion_checker, "depth_profile", "front_down_both_depth")
    t0 = time.perf_counter()
    frame, down_frame, front_depth, down_depth, timing = client.capture_views(
        profile=profile,
        mode=capture_mode,
        verbose=False,
    )
    elapsed = time.perf_counter() - t0
    timing = dict(timing or {})
    timing.setdefault("total_s", elapsed)
    _debug_print(
        f"  [ConfirmCapture] profile={profile} time={timing.get('total_s', elapsed):.2f}s  "
        f"front={web_helpers.shape_text(frame)} down={web_helpers.shape_text(down_frame)} "
        f"front_depth={web_helpers.shape_text(front_depth)} down_depth={web_helpers.shape_text(down_depth)}"
    )
    return frame, down_frame, front_depth, down_depth, float(timing.get("total_s", elapsed))


def _capture_fresh_rgb_frames(client, completion_checker, capture_mode: str):
    profile = getattr(completion_checker, "rgb_profile", None) or "front_down"
    t0 = time.perf_counter()
    frame, down_frame, _front_depth, _down_depth, timing = client.capture_views(
        profile=profile,
        mode=capture_mode,
        verbose=False,
    )
    elapsed = time.perf_counter() - t0
    timing = dict(timing or {})
    timing.setdefault("total_s", elapsed)
    _debug_print(
        f"  [ConfirmRGB] profile={profile} time={timing.get('total_s', elapsed):.2f}s  "
        f"front={web_helpers.shape_text(frame)} down={web_helpers.shape_text(down_frame)}"
    )
    return frame, down_frame, float(timing.get("total_s", elapsed))


def _wait_completion_depth(depth_future):
    try:
        front_depth, down_depth, timing = depth_future.result()
        return front_depth, down_depth, float((timing or {}).get("total_s", 0.0) or 0.0)
    except Exception as exc:
        print(f"  [Depth] completion depth capture failed: {exc}")
        return None, None, 0.0


def _attach_depth_to_detection_lists(front_all, down_all, frame, down_frame, front_depth, down_depth) -> None:
    # memory需要每个候选实例的depth_median来投影世界坐标；这里只写入检测对象的轻量字段。
    for index, detection in enumerate(list(front_all or [])):
        web_helpers.target_depth_text(f"front#{index}", detection, frame, front_depth)
    for index, detection in enumerate(list(down_all or [])):
        web_helpers.target_depth_text(f"down#{index}", detection, down_frame, down_depth)


def _update_memory_from_fresh_detection(
    objects: RuntimeObjects,
    client,
    stage,
    *,
    frame,
    down_frame,
    front_all,
    down_all,
) -> None:
    if getattr(objects, "mission_memory", None) is None:
        return
    try:
        observer_world, observer_yaw = client.get_pose()
        identity_bundle = SimpleNamespace(
            front_detections=list(front_all or []),
            down_detections=list(down_all or []),
            front_detection=None,
            down_detection=None,
            front_image=frame,
            down_image=down_frame,
            observer_world=list(observer_world),
            observer_yaw_deg=float(observer_yaw),
        )
        approved_front = _identity_approved_detections(objects, stage, identity_bundle, "front")
        approved_down = _identity_approved_detections(objects, stage, identity_bundle, "down")
        events = objects.mission_memory.update_from_detections(
            stage=stage,
            detections_by_view={"front": approved_front, "down": approved_down},
            images_by_view={"front": frame, "down": down_frame},
            observer_world=observer_world,
            observer_yaw_deg=observer_yaw,
        )
        if events:
            primary = objects.mission_memory.primary_instance(stage)
            facade_fallbacks = sum(
                bool(getattr(event.detection, "surface_lock_fallback", False))
                for event in events
            )
            fallback_text = (
                f" facade_surface_locks={facade_fallbacks}"
                if facade_fallbacks
                else ""
            )
            print(
                f"  [Memory] fresh_updates={len(events)} "
                f"primary={(primary.instance_id if primary else 'none')}"
                f"{fallback_text}"
            )
            _record_locked_target_snapshot(
                objects,
                stage,
                identity_bundle,
                source="fresh_completion_observation",
                events=events,
            )
    except Exception as exc:
        print(f"  [Memory] update skipped: {exc}")


def _estimated_pose_xy_distance(estimated: Any, current_world=None) -> float | None:
    """Compute distance from the same live pose used by a completion trigger.

    Memory and distance-estimator records can outlive the capture that created
    them.  Completion code uses this helper to reject a stale estimate whose
    stored distance says "arrived" while the current AirSim pose is still far
    away.
    """
    if estimated is None:
        return None
    pose = current_world or getattr(estimated, "current_world", None)
    target = getattr(estimated, "target_world", None)
    if isinstance(estimated, dict):
        pose = current_world or estimated.get("current_world")
        target = estimated.get("target_world")
    if not pose or not target or len(pose) < 2 or len(target) < 2:
        return None
    try:
        return math.hypot(float(pose[0]) - float(target[0]), float(pose[1]) - float(target[1]))
    except (TypeError, ValueError):
        return None




__all__ = [
    "_capture_rgb_for_stage",
    "_capture_completion_depth",
    "_publish_frames",
    "_evaluate_completion",
    "_caption_for_stage",
    "_detection_reliability",
    "_suppress_unreliable_detection",
    "_select_reliable_detection",
    "_detect_dual_view",
    "_best_detection_from_list",
    "_evaluate_completion_fast_slow",
    "_capture_fresh_completion_frames",
    "_capture_fresh_rgb_frames",
    "_wait_completion_depth",
    "_attach_depth_to_detection_lists",
    "_update_memory_from_fresh_detection",
    "_estimated_pose_xy_distance",
]
