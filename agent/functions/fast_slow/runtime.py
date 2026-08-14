"""Fast-slow AirSim runtime built on decoupled agent functions."""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any
from types import SimpleNamespace
from concurrent.futures import Future, ThreadPoolExecutor

from config import cfg, get_cfg

from agent.functions.candidate.pipeline import prepare_candidates_for_world_model, select_best_candidate
from agent.models.planner.sliding_window_planner import cumulative_to_incremental, incremental_to_cumulative
from agent.functions.common.config_access import function_section
from agent.functions.common.detection_policy import (
    detection_caption_for_stage,
    detection_reliability as shared_detection_reliability,
    is_large_structure_stage,
)
from agent.functions.common.task_manager import TaskManager
from agent.functions.common.warmup import warmup_from_config
from agent.functions.common import web_runtime_helpers as web_helpers
from agent.functions.completion import (
    NavigationMetricsTracker,
    build_distance_arrival_completion,
    build_task_completion_checker,
)
from agent.functions.direction import build_direction_estimator
from agent.functions.distance_estimation import build_distance_estimator
from agent.functions.fast_slow.completion_pipeline import CompletionPipeline
from agent.functions.fast_slow.completion_pipeline import DetectionDepthBundle
from agent.functions.fast_slow.controller import FastSlowController
from agent.functions.fast_slow.path_stream import ContinuousPathStream
from agent.functions.memory import (
    MissionMemory,
    build_mission_memory,
    is_view_relative_stage,
    view_relative_direction,
)
from agent.functions.memory.geometry import (
    distance_to_instance_geometry,
    has_surface_geometry,
    has_surface_samples,
    instance_surface_sample_points,
    world_to_body,
)
from agent.functions.memory.spatial_reasoning import relation_kind
from agent.functions.obstacle_avoidance import DepthObstacleAvoider, build_depth_obstacle_avoider
from agent.functions.planning.direction_hint import (
    direction_hint_from_front_detection,
    direction_hint_from_locked_body_target,
)
from agent.functions.planning.sliding_window_planning import SlidingWindowPlanningFunction
from agent.functions.recovery.collision_recovery import CollisionRecovery
from agent.functions.relocalization import TargetRelocalizer
from agent.functions.task_parser import build_task_parser
from agent.models.detection import build_detector
from agent.models.world_model import build_world_model
from sim.frame_capturer import FrameCapturer
from web.app import create_app
from web.shared_state import SharedState


_LAST_NAVIGATION_METRICS: NavigationMetricsTracker | None = None


@dataclass
class RuntimeObjects:
    detector: Any
    direction_estimator: Any
    completion_checker: Any
    planner: SlidingWindowPlanningFunction
    controller: FastSlowController
    task_manager: TaskManager
    relocalizer: TargetRelocalizer
    collision_recovery: CollisionRecovery
    detect_executor: ThreadPoolExecutor
    slow_executor: ThreadPoolExecutor
    future_detect_executor: ThreadPoolExecutor
    future_scan_executor: ThreadPoolExecutor
    distance_estimator: Any
    arrival_completion: Any
    navigation_metrics: NavigationMetricsTracker
    mission_memory: MissionMemory
    obstacle_avoider: DepthObstacleAvoider
    world_model: Any = None
    completion_pipeline: CompletionPipeline | None = None
    display_step: int = 0
    display_max_steps: int = 0
    task_failed: bool = False
    completion_attempts: dict[tuple, int] = field(default_factory=dict)
    completion_retry_after: dict[tuple, float] = field(default_factory=dict)
    # future_scan_index/last_future_scan_s 控制低频轮询未来目标，避免每帧都检测所有目标。
    future_scan_index: int = 0
    last_future_scan_s: float = 0.0
    future_scan_job: Any = None


@dataclass(frozen=True)
class MemoryScanSnapshot:
    """One timestamp-aligned RGB/depth/pose snapshot for background memory scans."""

    frame: Any
    down_frame: Any
    front_depth: Any
    down_depth: Any
    observer_world: tuple[float, float, float]
    observer_yaw_deg: float


@dataclass(frozen=True)
class FutureMemoryObservation:
    stage: Any
    front_detections: list
    down_detections: list
    detect_elapsed: float


@dataclass
class FutureMemoryScanJob:
    future: Future
    snapshot: MemoryScanSnapshot
    root_instruction: str
    source_stage_key: tuple
    target_names: tuple[str, ...]
    submitted_at: float
    warned_slow: bool = False


def _debug_logs_enabled() -> bool:
    fast_slow_cfg = {**(cfg.get("FAST_SLOW", {}) or {}), **function_section(cfg, "FAST_SLOW")}
    return bool(fast_slow_cfg.get("DEBUG_LOGS", False))


def _debug_print(message: str) -> None:
    if _debug_logs_enabled():
        print(message)


def _build_runtime_objects() -> RuntimeObjects:
    detector = build_detector()
    direction_estimator = build_direction_estimator()
    completion_checker = build_task_completion_checker(
        detector=detector,
        direction_estimator=direction_estimator,
    )
    planner_cfg = {**(cfg.get("SLIDING_WINDOW", {}) or {}), **function_section(cfg, "PLANNING")}
    fast_slow_cfg = {**(cfg.get("FAST_SLOW", {}) or {}), **function_section(cfg, "FAST_SLOW")}
    relocalization_cfg = {**cfg, "RELOCALIZATION": {**(cfg.get("RELOCALIZATION", {}) or {}), **function_section(cfg, "RELOCALIZATION")}}
    collision_cfg = {**cfg, "COLLISION_RECOVERY": {**(cfg.get("COLLISION_RECOVERY", {}) or {}), **function_section(cfg, "COLLISION_RECOVERY")}}
    planner = SlidingWindowPlanningFunction(planner_cfg)
    controller = FastSlowController(fast_slow_cfg)
    detect_executor = ThreadPoolExecutor(
        max_workers=int(fast_slow_cfg.get("DETECT_WORKERS", 2)),
        thread_name_prefix="fast_slow_detect",
    )
    slow_executor = ThreadPoolExecutor(
        max_workers=int(fast_slow_cfg.get("SLOW_WORKERS", 2)),
        thread_name_prefix="fast_slow_slow",
    )
    # Future-target work must never occupy the current-stage detector workers.
    # One scan worker plus a small dedicated detector pool also provides hard
    # backpressure: at most one opportunistic scan can be active at a time.
    future_detect_executor = ThreadPoolExecutor(
        max_workers=max(1, int(fast_slow_cfg.get("FUTURE_DETECT_WORKERS", 1))),
        thread_name_prefix="future_memory_detect",
    )
    future_scan_executor = ThreadPoolExecutor(
        max_workers=1,
        thread_name_prefix="future_memory_scan",
    )
    arrival_completion = build_distance_arrival_completion(cfg)
    return RuntimeObjects(
        detector=detector,
        direction_estimator=direction_estimator,
        completion_checker=completion_checker,
        planner=planner,
        controller=controller,
        task_manager=TaskManager(enabled=True),
        relocalizer=TargetRelocalizer(relocalization_cfg, detector=detector),
        collision_recovery=CollisionRecovery.from_config(collision_cfg),
        detect_executor=detect_executor,
        slow_executor=slow_executor,
        future_detect_executor=future_detect_executor,
        future_scan_executor=future_scan_executor,
        distance_estimator=build_distance_estimator(),
        arrival_completion=arrival_completion,
        navigation_metrics=NavigationMetricsTracker(arrival_completion.arrival_radius_m),
        mission_memory=build_mission_memory(),
        obstacle_avoider=build_depth_obstacle_avoider(),
        world_model=build_world_model(),
    )


def _update_target_pose_from_bundle(objects: RuntimeObjects, stage, bundle: DetectionDepthBundle | None):
    if bundle is None:
        return None
    if (
        getattr(objects, "mission_memory", None) is not None
        and bundle.observer_world is not None
        and bundle.observer_yaw_deg is not None
    ):
        events = objects.mission_memory.update_from_detections(
            stage=stage,
            detections_by_view={
                "front": getattr(bundle, "front_detections", None) or [bundle.front_detection],
                "down": getattr(bundle, "down_detections", None) or [bundle.down_detection],
            },
            images_by_view={"front": bundle.front_image, "down": bundle.down_image},
            observer_world=bundle.observer_world,
            observer_yaw_deg=bundle.observer_yaw_deg,
        )
        if events:
            primary = objects.mission_memory.primary_instance(stage)
            print(
                f"  [Memory] updates={len(events)} "
                f"primary={(primary.instance_id if primary else 'none')}"
            )
        if is_view_relative_stage(stage) and not objects.mission_memory.has_primary(stage):
            # A detector hit is not a navigation identity until the requested
            # activation-view ordinal has been resolved and locked.
            objects.distance_estimator.clear()
            objects.navigation_metrics.invalidate_target(CompletionPipeline.stage_key(stage))
            return None
        trusted_surface_fn = getattr(
            objects.mission_memory,
            "trusted_near_large_surface_estimate",
            None,
        )
        trusted_surface = (
            trusted_surface_fn(stage, bundle.observer_world)
            if callable(trusted_surface_fn)
            else None
        )
        if trusted_surface is not None:
            # Near a locked facade, a generic detector may prefer a complete
            # building tens of metres away. Keep both control and metrics on
            # the locked surface instead of publishing that unrelated point.
            objects.distance_estimator.clear()
            objects.navigation_metrics.update_target(
                CompletionPipeline.stage_key(stage),
                trusted_surface["target_world"],
                confidence=float(trusted_surface.get("confidence", 0.0) or 0.0),
                replace_stage_reference=True,
            )
            objects.navigation_metrics.record_distance(trusted_surface["distance_m"])
            _debug_print(
                "  [DistanceEstimate] detector point ignored; "
                f"locked near-surface memory distance={trusted_surface['distance_m']:.1f}m"
            )
            return SimpleNamespace(**trusted_surface)
    if not objects.distance_estimator.enabled:
        return None
    view = str(bundle.distance_view or "none").strip().lower()
    if view == "front":
        detection, image = bundle.front_detection, bundle.front_image
    elif view == "down":
        detection, image = bundle.down_detection, bundle.down_image
    else:
        objects.distance_estimator.clear()
        objects.navigation_metrics.invalidate_target(CompletionPipeline.stage_key(stage))
        _debug_print("  [DistanceEstimate] cleared: target not detected")
        return None
    if bundle.observer_world is None or bundle.observer_yaw_deg is None:
        return None
    estimate = objects.distance_estimator.update_from_detection(
        stage_key=CompletionPipeline.stage_key(stage),
        detection=detection,
        image=image,
        observer_world=bundle.observer_world,
        observer_yaw_deg=bundle.observer_yaw_deg,
    )
    if estimate is None:
        objects.distance_estimator.clear()
        objects.navigation_metrics.invalidate_target(CompletionPipeline.stage_key(stage))
        _debug_print("  [DistanceEstimate] cleared: reliable target depth unavailable")
    if estimate is not None:
        target = estimate.target_world
        objects.navigation_metrics.update_target(
            CompletionPipeline.stage_key(stage),
            target,
            confidence=estimate.score,
        )
        _debug_print(
            f"  [DistanceEstimate] updated target=({target[0]:.2f},{target[1]:.2f},{target[2]:.2f}) "
            f"source={estimate.camera} measured={estimate.measured_distance_m:.1f}m"
        )
    return estimate


def _execute_action_stage(client, stage) -> None:
    act = stage.action
    val = stage.value or 0
    print(f"  [Action] direct execution: {act} {val}")
    pos_before, yaw_before = client.get_pose()
    try:
        if act in ("left", "right"):
            sign = 1 if act == "right" else -1
            client.rotate_to_yaw(yaw_before + sign * val)
        elif act == "forward":
            rad = math.radians(yaw_before)
            client.move_to_position(
                pos_before[0] + val * math.cos(rad),
                pos_before[1] + val * math.sin(rad),
                pos_before[2],
            )
        elif act == "backward":
            rad = math.radians(yaw_before)
            client.move_to_position(
                pos_before[0] - val * math.cos(rad),
                pos_before[1] - val * math.sin(rad),
                pos_before[2],
            )
        elif act == "up":
            client.move_to_position(pos_before[0], pos_before[1], pos_before[2] - val)
        elif act == "down":
            client.move_to_position(pos_before[0], pos_before[1], pos_before[2] + val)
        elif act == "land":
            # 降落是物理动作，不能由memory完成判定替代，必须真正调用飞控执行。
            client.land()
    except Exception as exc:
        print(f"  [Action] error: {exc}")
    pos_after, yaw_after = client.get_pose()
    print(
        f"  [Action] from: ({pos_before[0]:.1f}, {pos_before[1]:.1f}, {pos_before[2]:.1f}) "
        f"yaw={yaw_before:.1f}deg"
    )
    print(
        f"           to:   ({pos_after[0]:.1f}, {pos_after[1]:.1f}, {pos_after[2]:.1f}) "
        f"yaw={yaw_after:.1f}deg"
    )


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


def _is_above_stage(stage: Any) -> bool:
    relation = str(getattr(stage, "relation", "") or "").strip().lower()
    instruction = str(getattr(stage, "instruction", "") or "").strip().lower()
    return (
        relation in {"above", "over", "on top", "on top of"}
        or "above" in instruction
        or "on top" in instruction
    )


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
    from agent.models.detection.base import DetectionResult

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
        _suppress_unreliable_detection(stage, det, frame)
    for det in down_all:
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


def _detection_matches_view_relative_sector(
    stage: Any,
    detection: Any,
    image: Any,
    *,
    camera_name: str,
) -> bool:
    if str(camera_name or "").lower() != "front":
        return False
    bbox = list(getattr(detection, "bbox", []) or [])
    if len(bbox) < 4 or image is None or not hasattr(image, "size"):
        return False
    width = float(image.size[0])
    if width <= 1.0:
        return False
    center_ratio = (float(bbox[0]) + float(bbox[2])) / (2.0 * width)
    text = " ".join((
        str(getattr(stage, "instruction", "") or ""),
        str(getattr(stage, "completion_condition", "") or ""),
    )).lower().replace("-", " ")
    if any(token in text for token in ("front right", "right front", "on the right", "right side", "右前方", "右侧", "右边")):
        return center_ratio > 0.5
    if any(token in text for token in ("front left", "left front", "on the left", "left side", "左前方", "左侧", "左边")):
        return center_ratio < 0.5
    return True


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
        events = objects.mission_memory.update_from_detections(
            stage=stage,
            detections_by_view={"front": front_all or [], "down": down_all or []},
            images_by_view={"front": frame, "down": down_frame},
            observer_world=observer_world,
            observer_yaw_deg=observer_yaw,
        )
        if events:
            primary = objects.mission_memory.primary_instance(stage)
            print(
                f"  [Memory] fresh_updates={len(events)} "
                f"primary={(primary.instance_id if primary else 'none')}"
            )
    except Exception as exc:
        print(f"  [Memory] update skipped: {exc}")


def _memory_auxiliary_stages(stage) -> list:
    stages = []
    for idx, target in enumerate(list(getattr(stage, "auxiliary_targets", []) or [])):
        target = str(target or "").strip()
        if not target:
            continue
        # 辅助目标是锚点/限定物，例如 bush；它单独进memory，但不参与任务完成。
        stages.append(SimpleNamespace(
            index=getattr(stage, "index", 0),
            instruction=f"Memory anchor for stage {getattr(stage, 'index', 0) + 1}: {target}",
            mode="target",
            target=target,
            relation="",
            action="",
            value=None,
            unit="",
            requires_target=True,
            allow_relocalize=False,
            completion_condition="anchor landmark only",
            ordinal=None,
            selection_rule="stable",
            stage_kind="memory_anchor",
            auxiliary_targets=[],
        ))
    return stages


def _memory_observation_stages(objects: RuntimeObjects, current_stage=None, *, future_only: bool = False) -> list:
    if getattr(objects, "mission_memory", None) is None:
        return []
    stages = []
    current_index = getattr(current_stage, "index", -1) if current_stage is not None else -1
    seen = set()
    for stage in list(objects.task_manager.stages or []):
        if getattr(stage, "mode", "") not in {"target", "detect"}:
            continue
        if future_only and int(getattr(stage, "index", 0)) <= int(current_index):
            continue
        if is_view_relative_stage(stage):
            # Mission bootstrap and future scans must not define what "the
            # second building in the new view" means. Only the active stage
            # may observe candidates for this target.
            if current_stage is None or int(getattr(stage, "index", -1)) != int(current_index):
                continue
        for query_stage in [stage] + _memory_auxiliary_stages(stage):
            target = _caption_for_stage(query_stage, "")
            if not target:
                continue
            key = (getattr(query_stage, "index", None), target.lower(), getattr(query_stage, "stage_kind", ""))
            if key in seen:
                continue
            seen.add(key)
            stages.append(query_stage)
    return stages


def _run_memory_observation_scan(
    objects: RuntimeObjects,
    state,
    *,
    query_stages: list,
    frame,
    down_frame,
    front_depth,
    down_depth,
    observer_world,
    observer_yaw_deg,
    task_text: str,
    reason: str,
    max_queries: int,
) -> int:
    if getattr(objects, "mission_memory", None) is None or not objects.mission_memory.enabled:
        return 0
    if frame is None or not query_stages:
        return 0
    updated_total = 0
    for query_stage in list(query_stages)[: max(0, int(max_queries))]:
        best_det, front_det, down_det, detect_elapsed, front_all, down_all = _detect_dual_view(
            objects,
            query_stage,
            task_text,
            frame,
            down_frame,
        )
        _attach_depth_to_detection_lists(front_all, down_all, frame, down_frame, front_depth, down_depth)
        events = objects.mission_memory.update_from_detections(
            stage=query_stage,
            detections_by_view={"front": front_all or [], "down": down_all or []},
            images_by_view={"front": frame, "down": down_frame},
            observer_world=observer_world,
            observer_yaw_deg=observer_yaw_deg,
        )
        if events:
            updated_total += len(events)
            primary = objects.mission_memory.primary_instance(query_stage)
            print(
                f"  [MemoryScan] reason={reason} target={_caption_for_stage(query_stage, task_text)!r} "
                f"updates={len(events)} primary={(primary.instance_id if primary else 'none')} "
                f"detect={detect_elapsed:.2f}s"
            )
    if updated_total:
        state.update(memory_summary=objects.mission_memory.summary())
    return updated_total


def _bootstrap_mission_memory(objects: RuntimeObjects, client, state, task_text: str, capture_mode: str) -> None:
    memory = getattr(objects, "mission_memory", None)
    if memory is None or not memory.enabled or not bool(memory.config.get("BOOTSTRAP_ENABLED", True)):
        return
    query_stages = _memory_observation_stages(objects, current_stage=None, future_only=False)
    if not query_stages:
        return
    max_queries = int(memory.config.get("BOOTSTRAP_MAX_TARGETS", 8))
    profile = str(memory.config.get("BOOTSTRAP_CAPTURE_PROFILE", "front_down_both_depth") or "front_down_both_depth")
    try:
        (
            frame,
            down_frame,
            front_depth,
            down_depth,
            timing,
            observer_world,
            observer_yaw_deg,
        ) = web_helpers.capture_profile_isolated_with_pose(client, profile)
    except Exception as exc:
        print(f"  [MemoryBootstrap] skipped: {exc}")
        return
    target_text = ", ".join(_caption_for_stage(stage, task_text) for stage in query_stages[:max_queries])
    print(
        f"  [MemoryBootstrap] profile={profile} targets=[{target_text}] "
        f"time={float((timing or {}).get('total_s', 0.0) or 0.0):.2f}s"
    )
    _run_memory_observation_scan(
        objects,
        state,
        query_stages=query_stages,
        frame=frame,
        down_frame=down_frame,
        front_depth=front_depth,
        down_depth=down_depth,
        observer_world=observer_world,
        observer_yaw_deg=observer_yaw_deg,
        task_text=task_text,
        reason="bootstrap",
        max_queries=max_queries,
    )


def _bind_view_relative_stage(objects: RuntimeObjects, client, state, stage, task_text: str) -> bool:
    memory = getattr(objects, "mission_memory", None)
    if (
        memory is None
        or not memory.enabled
        or not is_view_relative_stage(stage)
        or not bool(memory.config.get("VIEW_RELATIVE_STAGE_BIND_ENABLED", True))
    ):
        return False

    memory.reset_view_relative_binding(stage)
    profile = str(memory.config.get("VIEW_RELATIVE_BIND_CAPTURE_PROFILE", "front_depth") or "front_depth")
    try:
        (
            frame,
            down_frame,
            front_depth,
            down_depth,
            timing,
            observer_world,
            observer_yaw_deg,
        ) = web_helpers.capture_profile_isolated_with_pose(client, profile)
    except Exception as exc:
        print(f"  [MemoryBind] skipped target={_caption_for_stage(stage, task_text)!r}: {exc}")
        try:
            fallback_world, fallback_yaw = client.get_pose()
            memory.begin_view_relative_binding(stage, fallback_world, fallback_yaw)
        except Exception:
            pass
        return False

    memory.begin_view_relative_binding(stage, observer_world, observer_yaw_deg)

    query_stages = [stage] + _memory_auxiliary_stages(stage)
    _run_memory_observation_scan(
        objects,
        state,
        query_stages=query_stages,
        frame=frame,
        down_frame=down_frame,
        front_depth=front_depth,
        down_depth=down_depth,
        observer_world=observer_world,
        observer_yaw_deg=observer_yaw_deg,
        task_text=task_text,
        reason="view_relative_activation",
        max_queries=len(query_stages),
    )
    primary = memory.primary_instance(stage)
    capture_total_s = float((timing or {}).get("total_s", 0.0) or 0.0)
    if (
        primary is None
        and is_large_structure_stage(stage)
        and bool(memory.config.get("VIEW_RELATIVE_LOCAL_RETRY_ENABLED", True))
    ):
        direction = view_relative_direction(stage)
        retry_yaw = float(memory.config.get("VIEW_RELATIVE_LOCAL_RETRY_YAW_DEG", 18.0))
        signed_offset = -retry_yaw if direction in {"left", "front_left"} else retry_yaw
        if direction == "front":
            signed_offset = 0.0
        if abs(signed_offset) > 1e-3:
            try:
                client.rotate_to_yaw(float(observer_yaw_deg) + signed_offset)
                (
                    retry_frame,
                    retry_down_frame,
                    retry_front_depth,
                    retry_down_depth,
                    retry_timing,
                    retry_world,
                    retry_yaw_deg,
                ) = web_helpers.capture_profile_isolated_with_pose(client, profile)
                capture_total_s += float((retry_timing or {}).get("total_s", 0.0) or 0.0)
                _run_memory_observation_scan(
                    objects,
                    state,
                    query_stages=query_stages,
                    frame=retry_frame,
                    down_frame=retry_down_frame,
                    front_depth=retry_front_depth,
                    down_depth=retry_down_depth,
                    observer_world=retry_world,
                    observer_yaw_deg=retry_yaw_deg,
                    task_text=task_text,
                    reason="view_relative_local_retry",
                    max_queries=len(query_stages),
                )
                primary = memory.primary_instance(stage)
                if primary is None:
                    client.rotate_to_yaw(float(observer_yaw_deg))
            except Exception as exc:
                print(f"  [MemoryBind] local retry skipped: {exc}")
                try:
                    client.rotate_to_yaw(float(observer_yaw_deg))
                except Exception:
                    pass

    local_ids = memory.local_instance_ids(stage)
    print(
        f"  [MemoryBind] view_relative target={_caption_for_stage(stage, task_text)!r} "
        f"ordinal={getattr(stage, 'ordinal', None) or 1} local_instances={local_ids} "
        f"primary={(primary.instance_id if primary else 'none')} "
        f"capture={capture_total_s:.2f}s"
    )
    state.update(memory_summary=memory.summary(stage))
    return primary is not None


def _detect_future_memory_snapshot(
    objects: RuntimeObjects,
    query_stages: list,
    snapshot: MemoryScanSnapshot,
    task_text: str,
) -> list[FutureMemoryObservation]:
    """Detect future targets without touching AirSim or MissionMemory."""

    observations: list[FutureMemoryObservation] = []
    for query_stage in query_stages:
        _best_det, _front_det, _down_det, detect_elapsed, front_all, down_all = _detect_dual_view(
            objects,
            query_stage,
            task_text,
            snapshot.frame,
            snapshot.down_frame,
            detect_executor=objects.future_detect_executor,
        )
        _attach_depth_to_detection_lists(
            front_all,
            down_all,
            snapshot.frame,
            snapshot.down_frame,
            snapshot.front_depth,
            snapshot.down_depth,
        )
        observations.append(
            FutureMemoryObservation(
                stage=query_stage,
                front_detections=front_all,
                down_detections=down_all,
                detect_elapsed=detect_elapsed,
            )
        )
    return observations


def _maybe_submit_future_memory_scan(
    objects: RuntimeObjects,
    stage,
    task_text: str,
    snapshot: MemoryScanSnapshot | None,
) -> bool:
    """Submit one bounded future-target scan and return immediately."""

    memory = getattr(objects, "mission_memory", None)
    if memory is None or not memory.enabled or not bool(memory.config.get("OPPORTUNISTIC_SCAN_ENABLED", True)):
        return False
    if snapshot is None or snapshot.frame is None:
        return False
    # Depth must belong to the same capture batch as RGB. Never restore the old
    # behavior of recapturing depth later while the vehicle keeps moving.
    if snapshot.front_depth is None and snapshot.down_depth is None:
        _debug_print("  [FutureScan] skipped: same-frame depth unavailable")
        return False
    if objects.future_scan_job is not None:
        return False
    now = time.perf_counter()
    interval = float(memory.config.get("OPPORTUNISTIC_SCAN_INTERVAL_S", 4.0))
    if now - float(objects.last_future_scan_s or 0.0) < interval:
        return False
    query_stages = _memory_observation_stages(objects, current_stage=stage, future_only=True)
    if not query_stages:
        return False
    per_scan = max(1, int(memory.config.get("OPPORTUNISTIC_TARGETS_PER_SCAN", 1)))
    start = int(objects.future_scan_index) % len(query_stages)
    ordered = query_stages[start:] + query_stages[:start]
    selected = ordered[:per_scan]
    objects.future_scan_index = (start + len(selected)) % len(query_stages)
    objects.last_future_scan_s = now
    target_names = tuple(_caption_for_stage(query_stage, task_text) for query_stage in selected)
    future = objects.future_scan_executor.submit(
        _detect_future_memory_snapshot,
        objects,
        list(selected),
        snapshot,
        task_text,
    )
    objects.future_scan_job = FutureMemoryScanJob(
        future=future,
        snapshot=snapshot,
        root_instruction=str(memory.root_instruction or ""),
        source_stage_key=CompletionPipeline.stage_key(stage),
        target_names=target_names,
        submitted_at=now,
    )
    print(f"  [FutureScan] submitted targets={list(target_names)}")
    return True


def _poll_future_memory_scan(objects: RuntimeObjects, state, current_stage) -> None:
    """Commit a completed scan on the main loop; never wait for it."""

    job = objects.future_scan_job
    if job is None:
        return
    elapsed = max(0.0, time.perf_counter() - float(job.submitted_at))
    if not job.future.done():
        memory = getattr(objects, "mission_memory", None)
        warn_after = float((getattr(memory, "config", {}) or {}).get("OPPORTUNISTIC_SCAN_WARN_AFTER_S", 10.0))
        if not job.warned_slow and elapsed >= warn_after:
            job.warned_slow = True
            print(f"  [FutureScan] still_running elapsed={elapsed:.1f}s; current flight continues")
        return

    objects.future_scan_job = None
    try:
        observations = list(job.future.result() or [])
    except Exception as exc:
        print(f"  [FutureScan] failed elapsed={elapsed:.1f}s error={exc}")
        return

    memory = getattr(objects, "mission_memory", None)
    if (
        memory is None
        or current_stage is None
        or str(memory.root_instruction or "") != job.root_instruction
    ):
        print(f"  [FutureScan] discarded stale_task elapsed={elapsed:.1f}s")
        return

    current_index = int(getattr(current_stage, "index", 0))
    updated_total = 0
    discarded = 0
    for observation in observations:
        query_stage = observation.stage
        if int(getattr(query_stage, "index", -1)) < current_index:
            discarded += 1
            continue
        events = memory.update_from_detections(
            stage=query_stage,
            detections_by_view={
                "front": observation.front_detections,
                "down": observation.down_detections,
            },
            images_by_view={
                "front": job.snapshot.frame,
                "down": job.snapshot.down_frame,
            },
            observer_world=job.snapshot.observer_world,
            observer_yaw_deg=job.snapshot.observer_yaw_deg,
        )
        updated_total += len(events)
        if events:
            primary = memory.primary_instance(query_stage)
            print(
                f"  [MemoryScan] reason=future target={_caption_for_stage(query_stage, '')!r} "
                f"updates={len(events)} primary={(primary.instance_id if primary else 'none')} "
                f"detect={observation.detect_elapsed:.2f}s"
            )
    if updated_total:
        state.update(memory_summary=memory.summary(current_stage))
    print(
        f"  [FutureScan] completed elapsed={elapsed:.1f}s "
        f"updates={updated_total} discarded={discarded}"
    )


def _print_completion_evidence(stage, front_det, down_det, frame, down_frame):
    def evidence_text(name, detection, image):
        reliable = _detection_reliability(stage, detection, image)
        if detection is None or not detection.visible:
            return f"{name}=not_visible"
        depth = "N/A" if detection.depth_median is None else f"{detection.depth_median:.2f}m"
        return (
            f"{name}=bbox:{detection.bbox} score:{float(detection.score or 0.0):.2f} "
            f"reliability:{reliable:.3f} depth:{depth}"
        )

    print(
        "  [CompletionEvidence] "
        + evidence_text("front", front_det, frame)
        + " | "
        + evidence_text("down", down_det, down_frame)
    )


def _confirm_completion_now(
    objects: RuntimeObjects,
    client,
    stage,
    task_text,
    capture_mode: str,
    *,
    estimated_distance_m: float | None = None,
):
    """Pause-time final confirmation using fresh RGB and depth frames."""
    started = time.perf_counter()
    frame, down_frame, front_depth, down_depth, capture_elapsed = _capture_fresh_completion_frames(
        client,
        objects.completion_checker,
        capture_mode,
    )
    if frame is None:
        return None

    if getattr(objects.completion_checker, "uses_detector", False) and objects.completion_checker.is_detector_enabled():
        best_det, front_det, down_det, detect_elapsed, front_all, down_all = _detect_dual_view(
            objects,
            stage,
            task_text,
            frame,
            down_frame,
        )
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
        _print_completion_evidence(stage, front_det, down_det, frame, down_frame)
        judge_started = time.perf_counter()
        completion = objects.completion_checker.evaluate_with_detection(
            stage,
            task_text,
            frame,
            down_frame,
            best_det,
            front_detection=front_det,
            down_detection=down_det,
            front_depth_meters=front_depth,
            down_depth_meters=down_depth,
            estimated_distance_m=estimated_distance_m,
        )
        judge_elapsed = time.perf_counter() - judge_started
    else:
        detect_elapsed = 0.0
        judge_started = time.perf_counter()
        completion = objects.completion_checker.evaluate(
            stage,
            task_text,
            frame,
            down_frame,
            front_depth_meters=front_depth,
            down_depth_meters=down_depth,
        )
        judge_elapsed = time.perf_counter() - judge_started
    completion.elapsed = time.perf_counter() - started
    completion.capture_elapsed = capture_elapsed
    completion.detect_elapsed = detect_elapsed
    completion.judge_elapsed = judge_elapsed
    return completion


def _submit_plan_if_needed(
    objects: RuntimeObjects,
    client,
    stage,
    instruction,
    frame,
    down_frame,
    *,
    front_depth=None,
    down_depth=None,
    plan_pos=None,
    plan_yaw=None,
    plan_rot=None,
) -> bool:
    if frame is None:
        return False

    if plan_pos is None or plan_yaw is None:
        pos_now, yaw_now = client.get_pose()
    else:
        pos_now, yaw_now = list(plan_pos), float(plan_yaw)

    avoider = getattr(objects, "obstacle_avoider", None)
    if avoider is not None and getattr(avoider, "enabled", False):
        updated = avoider.update_from_depth(
            front_depth_meters=front_depth,
            down_depth_meters=down_depth,
            observer_world=pos_now,
            observer_yaw_deg=yaw_now,
        )
        if updated:
            # 这里只输出稀疏cell数量，不打印点云；避障memory本身不保存原始深度图。
            print(f"  [ObstacleMemory] depth_updates={updated} cells={len(avoider.obstacle_cells)}")

    # Drop waypoints that have fallen behind the drone before they become
    # negative-dx pending entries that confuse Qwen.
    _drop_path_points_behind_vehicle(objects, pos_now, yaw_now)

    pending_for_model = objects.controller.queue.pending_for_model(
        pos_now,
        yaw_now,
        current_rot_body_to_world=plan_rot,
    )

    inst = instruction
    direction_hint = direction_hint_from_front_detection(
        objects.detector,
        frame,
        _caption_for_stage(stage, instruction),
        stage=stage,
    )
    memory_hint = ""
    if getattr(objects, "mission_memory", None) is not None:
        memory_hint = objects.mission_memory.planner_hint(
            stage=stage,
            current_world=pos_now,
            yaw_deg=yaw_now,
        )
        locked_estimate = objects.mission_memory.estimate_distance(stage, pos_now)
        if locked_estimate is not None:
            locked_confidence = float(locked_estimate.get("confidence", 0.0) or 0.0)
            lock_min_confidence = float(
                objects.mission_memory.config.get("LOCK_MIN_CONFIDENCE", 0.35)
            )
            if locked_confidence >= lock_min_confidence:
                locked_direction = direction_hint_from_locked_body_target(
                    world_to_body(
                        locked_estimate.get("target_world", []),
                        pos_now,
                        yaw_now,
                    ),
                    confidence=locked_confidence,
                )
                if locked_direction.text:
                    direction_hint = locked_direction
    direction_text = direction_hint.text
    print(
        f"  [PlanInput] instruction={inst!r} pending={len(pending_for_model)} "
        f"pending_wp={_format_waypoints(pending_for_model)} "
        f"direction={direction_text!r} direction_reason={direction_hint.reason} "
        f"memory_hint={'yes' if memory_hint else 'no'} "
        f"front={web_helpers.shape_text(frame)} down={web_helpers.shape_text(down_frame)} "
        f"pose=({pos_now[0]:.2f},{pos_now[1]:.2f},{pos_now[2]:.2f}) yaw={yaw_now:.1f}"
    )

    def plan_fn(pending_waypoints):
        output = objects.planner.plan(
            frame,
            down_frame,
            inst,
            pending_waypoints=pending_waypoints,
            direction=direction_text,
            relation=getattr(stage, "relation", "") if stage else "",
            target=getattr(stage, "target_query", "") if stage else "",
            memory_hint=memory_hint,
        )
        output._selection_front_frame = frame
        output._selection_down_frame = down_frame
        output._selection_front_depth = front_depth
        output._selection_down_depth = down_depth
        output._selection_pos = list(pos_now)
        output._selection_yaw = float(yaw_now)
        return output

    submitted = objects.controller.maybe_submit_plan(
        current_pos=pos_now,
        current_yaw_deg=yaw_now,
        current_rot_body_to_world=plan_rot,
        plan_fn=plan_fn,
    )
    if submitted:
        pending = len(pending_for_model)
        _debug_print(
            f"  [Plan] submitted pending={pending} "
            f"remaining={len(objects.controller.queue.world_waypoints)}"
        )
    return submitted


def _format_waypoints(waypoints, limit: int = 5) -> str:
    points = []
    for wp in list(waypoints or [])[:limit]:
        if not isinstance(wp, (list, tuple)) or len(wp) < 3:
            continue
        points.append(f"[{float(wp[0]):.2f}, {float(wp[1]):.2f}, {float(wp[2]):.2f}]")
    suffix = " ..." if len(list(waypoints or [])) > limit else ""
    return "[" + ", ".join(points) + "]" + suffix if points else "[]"


def _candidate_trace_text(cand) -> str:
    if cand is None:
        return "none"
    if isinstance(cand, dict):
        getter = cand.get
    else:
        getter = lambda key, default=None: getattr(cand, key, default)
    breakdown = getter("score_breakdown", {}) or {}
    parts = [
        f"score={float(getter('pre_score', 0.0) or 0.0):.4f}",
        f"conf={float(getter('confidence', 0.0) or 0.0):.2f}",
        f"source={getter('source', '')}",
        f"wp={_format_waypoints(getter('waypoints', []))}",
    ]
    if breakdown:
        parts.append("breakdown=" + ",".join(f"{k}:{float(v):.3f}" for k, v in breakdown.items()))
    return " ".join(parts)


def _memory_path_clip_radius(objects: RuntimeObjects) -> float:
    memory_cfg = getattr(getattr(objects, "mission_memory", None), "config", {}) or {}
    distance_cfg = function_section(cfg, "DISTANCE_ESTIMATION")
    return float(memory_cfg.get(
        "PATH_CLIP_RADIUS_M",
        distance_cfg.get(
            "TRIGGER_RADIUS_M",
            getattr(objects.completion_checker, "stop_depth", cfg.get("AGENT", {}).get("STOP_DEPTH_THRESHOLD", 4.0)),
        ),
    ))


def _memory_near_standoff_radius(objects: RuntimeObjects, memory_context: dict, base_radius: float) -> float:
    memory_cfg = getattr(getattr(objects, "mission_memory", None), "config", {}) or {}
    target = memory_context.get("target_body") or []
    radius = max(float(base_radius), float(memory_cfg.get("NEAR_STANDOFF_M", base_radius)))
    footprint = float(memory_context.get("footprint_radius_m", 1.5) or 1.5)
    uncertainty = float(memory_context.get("uncertainty_m", 0.0) or 0.0)
    radius = max(
        radius,
        footprint
        + float(memory_cfg.get("NEAR_TARGET_EXTRA_STANDOFF_M", 0.0))
        + min(max(uncertainty, 0.0), float(memory_cfg.get("STANDOFF_UNCERTAINTY_CAP_M", 0.0))) * 0.35,
    )
    # 低空贴近车辆/箱子时不要按目标中心飞，额外留一点外圈距离，避免擦到碰撞盒。
    if len(target) >= 3 and abs(float(target[2])) <= float(memory_cfg.get("LOW_ALTITUDE_Z_DELTA_M", 3.0)):
        radius += float(memory_cfg.get("LOW_ALTITUDE_EXTRA_STANDOFF_M", 0.0))
    return max(0.8, radius)


def _memory_near_approach_radius_from_values(
    memory_cfg: dict,
    *,
    footprint: float,
    uncertainty: float,
    outer_radius: float,
) -> float:
    explicit = memory_cfg.get("NEAR_APPROACH_RADIUS_M", None)
    if explicit is None:
        return max(0.8, float(outer_radius))
    radius = float(explicit)
    radius = max(
        radius,
        float(footprint)
        + float(memory_cfg.get("NEAR_APPROACH_TARGET_CLEARANCE_M", 2.2))
        + min(max(float(uncertainty), 0.0), float(memory_cfg.get("NEAR_APPROACH_UNCERTAINTY_CAP_M", 1.5))) * 0.25,
    )
    min_gap = float(memory_cfg.get("NEAR_APPROACH_OUTER_GAP_M", 1.0))
    if float(outer_radius) > min_gap + 0.8:
        radius = min(radius, float(outer_radius) - min_gap)
    return max(0.8, radius)


def _memory_near_approach_radius(objects: RuntimeObjects, memory_context: dict, outer_radius: float) -> float:
    memory_cfg = getattr(getattr(objects, "mission_memory", None), "config", {}) or {}
    return _memory_near_approach_radius_from_values(
        memory_cfg,
        footprint=float(memory_context.get("footprint_radius_m", 1.5) or 1.5),
        uncertainty=float(memory_context.get("uncertainty_m", 0.0) or 0.0),
        outer_radius=float(outer_radius),
    )


def _apply_memory_path_guard(objects: RuntimeObjects, stage, cumulative_waypoints: list, memory_context: dict) -> tuple[list, str]:
    """Clip or replace a Qwen path so it cannot fly far past the locked memory target."""
    memory_cfg = getattr(getattr(objects, "mission_memory", None), "config", {}) or {}
    if not bool(memory_cfg.get("PATH_CLIP_ENABLED", True)):
        return cumulative_waypoints, ""
    if not memory_context or not bool(memory_context.get("enabled", False)):
        return cumulative_waypoints, ""
    target = memory_context.get("target_body") or []
    if len(target) < 3 or not cumulative_waypoints:
        return cumulative_waypoints, ""
    target = [float(target[0]), float(target[1]), float(target[2])]
    target_dist = _norm3(target)
    if target_dist <= 1e-6:
        return [], "already_at_memory_target"

    relation = str(memory_context.get("relation", "near") or "near").lower()
    base_radius = _memory_path_clip_radius(objects)
    if relation == "above":
        outer_radius = max(
            base_radius,
            float(memory_context.get("footprint_radius_m", 1.5) or 1.5)
            + float(memory_cfg.get("ABOVE_HORIZONTAL_RADIUS_M", 3.5)),
        )
        # 已经进入“上方”的水平范围时，不要把所有轨迹清空；还需要允许Qwen继续做高度/位置微调。
        if _guard_distance([0.0, 0.0, 0.0], target, relation) <= outer_radius:
            return cumulative_waypoints, ""
        radius = float(memory_cfg.get("ABOVE_APPROACH_RADIUS_M", max(2.0, min(outer_radius * 0.65, outer_radius - 1.0))))
    else:
        outer_radius = _memory_near_standoff_radius(objects, memory_context, base_radius)
        radius = _memory_near_approach_radius(objects, memory_context, outer_radius)
    radius = max(0.8, float(radius))
    current_dist = _guard_distance([0.0, 0.0, 0.0], target, relation)
    start_inside_outer = current_dist <= float(outer_radius)

    clipped = _clip_path_at_target_radius(
        cumulative_waypoints,
        target,
        radius,
        relation=relation,
        near_max_descent_m=float(memory_cfg.get("PATH_NEAR_MAX_DESCENT_M", 0.2)),
        near_max_climb_m=float(memory_cfg.get("PATH_NEAR_MAX_CLIMB_M", 0.5)),
        empty_if_start_inside=not start_inside_outer,
    )
    if clipped is not None:
        clipped, leg_limited = _limit_cumulative_path_length(clipped, memory_cfg)
        leg_reason = f"_leg_{float(memory_cfg.get('PATH_MAX_GUIDED_LEG_M', 0.0)):.1f}m" if leg_limited else ""
        return clipped, f"clip_enter_approach_{radius:.1f}m{leg_reason}"

    endpoint = [float(v) for v in cumulative_waypoints[-1][:3]]
    endpoint_dist = _distance3_body(endpoint, target)
    closest_dist = _closest_path_distance_to_target(cumulative_waypoints, target, relation=relation)
    target_xy_norm = math.sqrt(target[0] * target[0] + target[1] * target[1])
    endpoint_projection = _project_xy(endpoint, target)
    overshoot = target_xy_norm > 1e-6 and endpoint_projection > target_xy_norm + radius
    if start_inside_outer:
        # 已在完成圆内部时，只拦截明显飞离目标的路径，允许继续向圆内部微调。
        diverging = endpoint_dist > max(
            float(outer_radius) * float(memory_cfg.get("PATH_EXIT_RADIUS_RATIO", 1.20)),
            current_dist + float(memory_cfg.get("PATH_EXIT_MARGIN_M", 2.0)),
        )
    else:
        diverging = endpoint_dist > target_dist * float(memory_cfg.get("PATH_DIVERGE_RATIO", 0.90))
    missed_close = closest_dist <= radius * float(memory_cfg.get("PATH_CLOSE_MISS_RATIO", 1.35))
    target_behind = (
        not start_inside_outer
        and target[0] < -float(memory_cfg.get("PATH_TARGET_BEHIND_X_M", 2.0))
    )
    if overshoot or diverging or missed_close or target_behind:
        direct = _direct_memory_waypoint(
            target,
            radius,
            relation=relation,
            near_max_descent_m=float(memory_cfg.get("PATH_NEAR_MAX_DESCENT_M", 0.2)),
            near_max_climb_m=float(memory_cfg.get("PATH_NEAR_MAX_CLIMB_M", 0.5)),
        )
        reason_bits = []
        if overshoot:
            reason_bits.append("overshoot")
        if diverging:
            reason_bits.append("diverging")
        if missed_close:
            reason_bits.append("near_miss")
        if target_behind:
            reason_bits.append("target_behind")
        direct, leg_limited = _limit_cumulative_path_length(direct, memory_cfg)
        if leg_limited:
            reason_bits.append(f"leg_{float(memory_cfg.get('PATH_MAX_GUIDED_LEG_M', 0.0)):.1f}m")
        return direct, "replace_" + "_".join(reason_bits)
    limited, leg_limited = _limit_cumulative_path_length(cumulative_waypoints, memory_cfg)
    if leg_limited:
        return limited, f"clip_active_leg_{float(memory_cfg.get('PATH_MAX_GUIDED_LEG_M', 0.0)):.1f}m"
    return cumulative_waypoints, ""


def _limit_cumulative_path_length(cumulative_waypoints: list, memory_cfg: dict) -> tuple[list, bool]:
    """Bound one newly appended memory-guided leg while preserving its shape."""
    max_length = float(memory_cfg.get("PATH_MAX_GUIDED_LEG_M", 0.0) or 0.0)
    if max_length <= 0.0 or not cumulative_waypoints:
        return cumulative_waypoints, False
    prev = [0.0, 0.0, 0.0]
    traveled = 0.0
    limited = []
    for waypoint in cumulative_waypoints:
        cur = [float(v) for v in waypoint[:3]]
        segment = [cur[i] - prev[i] for i in range(3)]
        segment_length = _norm3(segment)
        if traveled + segment_length <= max_length + 1e-6:
            limited.append([round(v, 3) for v in cur])
            traveled += segment_length
            prev = cur
            continue
        remaining = max(0.0, max_length - traveled)
        if remaining > 1e-3 and segment_length > 1e-6:
            scale = remaining / segment_length
            limited.append([
                round(prev[i] + segment[i] * scale, 3)
                for i in range(3)
            ])
        return limited, True
    return limited, False


def _clip_path_at_target_radius(
    cumulative_waypoints: list,
    target: list[float],
    radius: float,
    *,
    relation: str = "near",
    near_max_descent_m: float = 0.2,
    near_max_climb_m: float = 0.5,
    empty_if_start_inside: bool = True,
):
    prev = [0.0, 0.0, 0.0]
    prev_dist = _guard_distance(prev, target, relation)
    if prev_dist <= radius:
        return [] if empty_if_start_inside else None
    out = []
    for waypoint in cumulative_waypoints:
        cur = [float(v) for v in waypoint[:3]]
        cur_dist = _guard_distance(cur, target, relation)
        if cur_dist <= radius:
            if relation == "above":
                hit = _segment_circle_entry_xy(prev, cur, target, radius) or cur
            else:
                hit = _segment_circle_entry_xy(prev, cur, target, radius) or cur
                # “旁边/附近”不应该把高度也插值到目标中心；低空靠近车辆时尤其容易撞。
                hit[2] = _clamp(hit[2], -abs(float(near_max_climb_m)), abs(float(near_max_descent_m)))
            out.append([round(hit[0], 3), round(hit[1], 3), round(hit[2], 3)])
            return out
        out.append([round(cur[0], 3), round(cur[1], 3), round(cur[2], 3)])
        prev = cur
    return None


def _segment_sphere_entry(a: list[float], b: list[float], center: list[float], radius: float):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    ax = a[0] - center[0]
    ay = a[1] - center[1]
    az = a[2] - center[2]
    qa = dx * dx + dy * dy + dz * dz
    if qa <= 1e-9:
        return None
    qb = 2.0 * (ax * dx + ay * dy + az * dz)
    qc = ax * ax + ay * ay + az * az - radius * radius
    disc = qb * qb - 4.0 * qa * qc
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    candidates = [(-qb - root) / (2.0 * qa), (-qb + root) / (2.0 * qa)]
    valid = [t for t in candidates if 0.0 <= t <= 1.0]
    if not valid:
        return None
    t = min(valid)
    return [a[0] + t * dx, a[1] + t * dy, a[2] + t * dz]


def _segment_circle_entry_xy(a: list[float], b: list[float], center: list[float], radius: float):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    ax = a[0] - center[0]
    ay = a[1] - center[1]
    qa = dx * dx + dy * dy
    if qa <= 1e-9:
        return None
    qb = 2.0 * (ax * dx + ay * dy)
    qc = ax * ax + ay * ay - radius * radius
    disc = qb * qb - 4.0 * qa * qc
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    candidates = [(-qb - root) / (2.0 * qa), (-qb + root) / (2.0 * qa)]
    valid = [t for t in candidates if 0.0 <= t <= 1.0]
    if not valid:
        return None
    t = min(valid)
    return [a[0] + t * dx, a[1] + t * dy, a[2] + t * (b[2] - a[2])]


def _direct_memory_waypoint(
    target: list[float],
    radius: float,
    *,
    relation: str = "near",
    near_max_descent_m: float = 0.2,
    near_max_climb_m: float = 0.5,
) -> list:
    dist = _norm3(target) if relation == "above" else math.sqrt(target[0] * target[0] + target[1] * target[1])
    if dist <= max(radius, 1e-6):
        return []
    scale = max(0.0, (dist - radius) / dist)
    z = target[2] * scale if relation == "above" else _clamp(0.0, -abs(float(near_max_climb_m)), abs(float(near_max_descent_m)))
    return [[
        round(target[0] * scale, 3),
        round(target[1] * scale, 3),
        round(z, 3),
    ]]


def _closest_path_distance_to_target(cumulative_waypoints: list, target: list[float], *, relation: str = "near") -> float:
    prev = [0.0, 0.0, 0.0]
    best = _guard_distance(prev, target, relation)
    for waypoint in cumulative_waypoints:
        cur = [float(v) for v in waypoint[:3]]
        if relation == "above":
            best = min(best, _segment_point_distance_xy(prev, cur, target))
        else:
            best = min(best, _segment_point_distance_xy(prev, cur, target))
        prev = cur
    return best


def _segment_point_distance(a: list[float], b: list[float], point: list[float]) -> float:
    vx = b[0] - a[0]
    vy = b[1] - a[1]
    vz = b[2] - a[2]
    wx = point[0] - a[0]
    wy = point[1] - a[1]
    wz = point[2] - a[2]
    denom = vx * vx + vy * vy + vz * vz
    if denom <= 1e-9:
        return _distance3_body(a, point)
    t = max(0.0, min(1.0, (wx * vx + wy * vy + wz * vz) / denom))
    closest = [a[0] + t * vx, a[1] + t * vy, a[2] + t * vz]
    return _distance3_body(closest, point)


def _segment_point_distance_xy(a: list[float], b: list[float], point: list[float]) -> float:
    vx = b[0] - a[0]
    vy = b[1] - a[1]
    wx = point[0] - a[0]
    wy = point[1] - a[1]
    denom = vx * vx + vy * vy
    if denom <= 1e-9:
        return math.sqrt((a[0] - point[0]) ** 2 + (a[1] - point[1]) ** 2)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    closest_x = a[0] + t * vx
    closest_y = a[1] + t * vy
    return math.sqrt((closest_x - point[0]) ** 2 + (closest_y - point[1]) ** 2)


def _guard_distance(point: list[float], target: list[float], relation: str = "near") -> float:
    if str(relation or "near").lower() == "above":
        return math.sqrt((float(point[0]) - float(target[0])) ** 2 + (float(point[1]) - float(target[1])) ** 2)
    return math.sqrt((float(point[0]) - float(target[0])) ** 2 + (float(point[1]) - float(target[1])) ** 2)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def _project_xy(point: list[float], direction: list[float]) -> float:
    denom = math.sqrt(direction[0] * direction[0] + direction[1] * direction[1])
    if denom <= 1e-9:
        return 0.0
    return (point[0] * direction[0] + point[1] * direction[1]) / denom


def _norm3(point: list[float]) -> float:
    return math.sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2])


def _distance3_body(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _apply_obstacle_path_guard(
    objects: RuntimeObjects,
    cumulative_waypoints: list,
    *,
    selection_pos,
    selection_yaw: float,
    memory_context: dict,
) -> tuple[list, str, Any]:
    avoider = getattr(objects, "obstacle_avoider", None)
    if avoider is None or not getattr(avoider, "enabled", False):
        return cumulative_waypoints, "", None
    result = avoider.filter_cumulative_waypoints(
        cumulative_waypoints,
        current_world=selection_pos,
        yaw_deg=float(selection_yaw),
        memory_context=memory_context,
    )
    if not getattr(result, "changed", False):
        return cumulative_waypoints, "", result
    obstacle = getattr(result, "obstacle_body", None)
    obstacle_text = "" if obstacle is None else f" obstacle_body={obstacle}"
    return (
        list(getattr(result, "waypoints", []) or []),
        f"{result.reason}{obstacle_text}",
        result,
    )


def _select_and_transform_plan(objects: RuntimeObjects, stage, result, frame, down_frame):
    if result is None:
        return None
    cand_cfg = cfg.get("CANDIDATE", {}) or {}
    if not bool(cand_cfg.get("ENABLED", False)):
        return result

    qwen_incremental = [list(wp) for wp in (getattr(result, "waypoints", []) or [])]
    qwen_cumulative = incremental_to_cumulative(qwen_incremental)
    prepared = SimpleNamespace(
        waypoints=qwen_cumulative,
        candidates=getattr(result, "candidates", []),
        reasoning=getattr(result, "reasoning", ""),
    )
    memory_context = {}
    if getattr(objects, "mission_memory", None) is not None:
        memory_context = objects.mission_memory.candidate_context(
            stage=stage,
            current_world=getattr(result, "_selection_pos", [0.0, 0.0, 0.0]),
            yaw_deg=float(getattr(result, "_selection_yaw", 0.0) or 0.0),
        )
    selection = prepare_candidates_for_world_model(
        prepared,
        detection=None,
        direction=getattr(stage, "instruction", "") if stage else "",
        stop_threshold=float(cfg.get("AGENT", {}).get("STOP_DEPTH_THRESHOLD", 8.0)),
        memory_context=memory_context,
    )
    all_candidates = selection.all_candidates or []
    selection_result = select_best_candidate(
        selection,
        world_model=objects.world_model,
        front_image=frame or getattr(result, "_selection_front_frame", None),
        down_image=down_frame or getattr(result, "_selection_down_frame", None),
        instruction=getattr(stage, "instruction", "") if stage else "",
    )
    chosen = selection_result.chosen
    if chosen is None:
        return result

    objects.display_step += 1
    plan_raw = getattr(result, "raw", None)
    qwen_elapsed = float(
        getattr(plan_raw, "elapsed_s", getattr(result, "_planning_wall_s", 0.0)) or 0.0
    )
    print(f"\n[Step {objects.display_step}/{objects.display_max_steps}]")
    if stage is not None:
        print(
            f"  stage {stage.index + 1}/{len(objects.task_manager.stages)} "
            f"mode={getattr(stage, 'mode', 'target')}: {getattr(stage, 'instruction', '')}"
        )
    print(
        f"  [Qwen] {qwen_elapsed:.2f}s -> {len(qwen_incremental)} waypoints "
        f"({len(qwen_incremental)} raw)"
    )
    wm_scores = selection_result.world_model_scores
    print(f"  [TopK Trajectory] showing {len(all_candidates)}/{len(all_candidates)}")
    for i, cand in enumerate(all_candidates):
        wm_text = ""
        if i < len(wm_scores) and math.isfinite(wm_scores[i]):
            wm_text = f" wm={wm_scores[i]:.4f}"
        marker = "[x]" if i == selection_result.selected_index else "[ ]"
        print(f"    {marker} #{i + 1} {_candidate_trace_text(cand)}{wm_text}")
    selected_index = selection_result.selected_index
    selection_source = "world_model" if selection_result.used_world_model else "pre_score"
    print(
        f"  [Candidate] selected={selected_index} by={selection_source} "
        f"score={float(getattr(chosen, 'pre_score', 0.0) or 0.0):.4f} "
        f"reason={selection_result.reasoning}"
    )

    chosen_cumulative = [list(wp) for wp in getattr(chosen, "waypoints", []) or []]
    guarded_cumulative, guard_reason = _apply_memory_path_guard(
        objects,
        stage,
        chosen_cumulative,
        memory_context,
    )
    if guard_reason:
        print(
            f"  [MemoryPathGuard] {guard_reason} "
            f"from={_format_waypoints(chosen_cumulative)} to={_format_waypoints(guarded_cumulative)}"
        )
        chosen_cumulative = guarded_cumulative
    selection_pos = getattr(result, "_selection_pos", [0.0, 0.0, 0.0])
    selection_yaw = float(getattr(result, "_selection_yaw", 0.0) or 0.0)
    obstacle_guarded, obstacle_reason, obstacle_result = _apply_obstacle_path_guard(
        objects,
        chosen_cumulative,
        selection_pos=selection_pos,
        selection_yaw=selection_yaw,
        memory_context=memory_context,
    )
    if obstacle_reason:
        print(
            f"  [DepthObstacleGuard] {obstacle_reason} "
            f"from={_format_waypoints(chosen_cumulative)} to={_format_waypoints(obstacle_guarded)}"
        )
        chosen_cumulative = obstacle_guarded
    if (
        obstacle_result is not None
        and str(getattr(obstacle_result, "reason", "") or "") == "stop_before_target_depth_obstacle"
        and getattr(obstacle_result, "obstacle_body", None) is not None
        and getattr(objects, "mission_memory", None) is not None
    ):
        contact = objects.mission_memory.record_locked_large_surface_contact(
            stage,
            observer_world=selection_pos,
            observer_yaw_deg=selection_yaw,
            obstacle_body=obstacle_result.obstacle_body,
        )
        if contact is not None:
            print(
                "  [TargetSurfaceDepth] fused locked facade contact "
                f"range={contact['contact_range_m']:.2f}m "
                f"memory_distance={contact['distance_m']:.2f}m "
                f"uncertainty={contact['uncertainty_m']:.1f}m; "
                "completion will be checked before the next plan"
            )
    chosen_incremental = cumulative_to_incremental(chosen_cumulative)
    print(
        f"  [Trajectory] wp={len(chosen_cumulative)} non_zero={len(chosen_incremental)} "
        f"body_wp={_format_waypoints(chosen_cumulative)}"
    )
    objects.navigation_metrics.print_step()
    transformed = result
    transformed.waypoints = chosen_incremental
    transformed.reasoning = (
        f"{getattr(result, 'reasoning', '')} | chosen={getattr(chosen, 'source', 'planner')}"
        f" pre={float(getattr(chosen, 'pre_score', 0.0) or 0.0):.4f}"
    ).strip(" |")
    transformed.raw = {
        "qwen_incremental": qwen_incremental,
        "qwen_cumulative": qwen_cumulative,
        "selected_cumulative": chosen_cumulative,
        "selected_incremental": chosen_incremental,
        "memory_path_guard": guard_reason,
        "obstacle_path_guard": obstacle_reason,
        "all_candidates": [cand.to_dict() for cand in all_candidates],
        "wm_candidates": [cand.to_dict() for cand in (selection.wm_candidates or [])],
        "selected_candidate": chosen.to_dict(),
    }
    transformed.candidates = [cand.to_dict() for cand in all_candidates]
    transformed.selected_index = selected_index
    transformed.selected_candidate = chosen.to_dict()
    return transformed


def _poll_plan(objects: RuntimeObjects, stage=None, frame=None, down_frame=None, state=None) -> None:
    select_fn = lambda r: _select_and_transform_plan(objects, stage, r, frame, down_frame)
    result = objects.controller.poll_plan(select_fn=select_fn)
    if result is None:
        return
    qwen_inc = getattr(result, "waypoints", []) or []
    rejection_reason = getattr(result, "rejection_reason", "")
    if rejection_reason:
        print(f"  [PlanRejected] {rejection_reason}; existing queue kept")
    _debug_print(f"  [QwenSliding] raw={_format_waypoints(qwen_inc)}")
    print(f"  [Queue] added={len(qwen_inc)} remaining={len(objects.controller.queue.world_waypoints)}")
    print(
        f"  [WorldQueue] remaining={len(objects.controller.queue.world_waypoints)} "
        f"world_wp={_format_waypoints(objects.controller.queue.world_waypoints, limit=20)}"
    )
    if state is not None:
        state.update(
            qwen_waypoints=qwen_inc,
            trajectory_queue=[list(wp) for wp in objects.controller.queue.world_waypoints],
        )


def _execute_one_from_queue(objects: RuntimeObjects, client, state, count: int | None = None) -> bool:
    queue_len = len(objects.controller.queue.world_waypoints)
    requested = objects.controller.execute_count if count is None else int(count)
    n = min(max(1, requested), queue_len)
    n = max(1, n)
    exec_waypoints = [list(wp) for wp in objects.controller.queue.world_waypoints[:n]]
    if not exec_waypoints:
        return False
    if all(all(abs(v) < 1e-6 for v in wp) for wp in exec_waypoints):
        objects.controller.mark_executed(len(exec_waypoints))
        return False

    print(
        f"  [FastExec] execute={len(exec_waypoints)} "
        f"queue_before={len(objects.controller.queue.world_waypoints)} "
        f"world_wp={exec_waypoints}"
    )
    state.update(status="executing")
    t0 = time.perf_counter()
    pos_final, yaw_final, collided = client.execute_waypoints(exec_waypoints)
    elapsed = time.perf_counter() - t0
    state.update(pose=pos_final, yaw=yaw_final, collided=collided)
    if collided:
        objects.controller.queue.clear()
        recovery = objects.collision_recovery.recover(client)
        print(f"  [Collision] collided=True recovery_attempted={recovery.attempted}")
    else:
        objects.controller.mark_executed(len(exec_waypoints))
        settle = float(cfg.get("FUNCTIONS", {}).get("FAST_SLOW", {}).get("EXECUTION_SETTLE_S", 0.0))
        if settle > 0:
            time.sleep(settle)
    print(
        f"  [FastExec] done time={elapsed:.2f}s "
        f"queue_after={len(objects.controller.queue.world_waypoints)}"
    )
    return True


def _needs_planning_snapshot(objects: RuntimeObjects) -> bool:
    return (
        len(objects.controller.queue.world_waypoints) == 0
        and objects.controller.can_submit_plan()
    )


def _capture_and_submit_plan(
    objects: RuntimeObjects,
    client,
    stage,
    instruction,
    capture_mode: str,
    *,
    isolated_capture: bool = False,
):
    planning_cfg = function_section(cfg, "PLANNING")
    planning_capture_mode = str(planning_cfg.get("CAPTURE_MODE", "batch"))
    avoider = getattr(objects, "obstacle_avoider", None)
    memory = getattr(objects, "mission_memory", None)
    future_scan_enabled = bool(
        memory is not None
        and memory.enabled
        and bool(memory.config.get("OPPORTUNISTIC_SCAN_ENABLED", True))
    )
    need_depth = bool(
        (avoider is not None and getattr(avoider, "needs_plan_depth", lambda: False)())
        or future_scan_enabled
    )
    front_depth = down_depth = None
    snapshot_pos = snapshot_yaw = None
    snapshot_depth_aligned = False
    if isolated_capture and planning_capture_mode == "batch":
        profile = str(planning_cfg.get("CAPTURE_PROFILE", "front_down") or "front_down")
        if need_depth and profile == "front_down":
            profile = "front_down_both_depth"
        (
            frame,
            down_frame,
            front_depth,
            down_depth,
            _timing,
            capture_pos,
            capture_yaw,
        ) = web_helpers.capture_profile_isolated_with_pose(client, profile)
        snapshot_pos = list(capture_pos)
        snapshot_yaw = float(capture_yaw)
        snapshot_depth_aligned = front_depth is not None or down_depth is not None
        if hasattr(client, "get_pose_full"):
            _current_pos, _current_yaw, capture_rot = client.get_pose_full()
        else:
            print("  [Capture] full body rotation unavailable for Qwen plan, retry later")
            return None, None, False, None
    elif (
        planning_capture_mode == "batch"
        and need_depth
        and hasattr(client, "capture_planning_views_depth_with_pose")
    ):
        camera_offset = planning_cfg.get("FRONT_CAMERA_OFFSET", [1.0, 0.0, 0.0])
        frame, down_frame, front_depth, down_depth, capture_pos, capture_yaw, capture_rot, _timing = (
            client.capture_planning_views_depth_with_pose(camera_offset=camera_offset)
        )
        snapshot_pos = list(capture_pos)
        snapshot_yaw = float(capture_yaw)
        snapshot_depth_aligned = front_depth is not None or down_depth is not None
    elif planning_capture_mode == "batch" and hasattr(client, "capture_planning_views_with_pose"):
        camera_offset = planning_cfg.get("FRONT_CAMERA_OFFSET", [1.0, 0.0, 0.0])
        frame, down_frame, capture_pos, capture_yaw, capture_rot, _timing = (
            client.capture_planning_views_with_pose(camera_offset=camera_offset)
        )
        snapshot_pos = list(capture_pos)
        snapshot_yaw = float(capture_yaw)
    else:
        if need_depth:
            frame, down_frame, front_depth, down_depth, _timing = client.capture_views(
                profile="front_down_both_depth",
                mode=planning_capture_mode,
                verbose=False,
            )
        else:
            frame, down_frame, front_depth, down_depth, _timing = _capture_rgb_for_stage(
                client,
                objects.completion_checker,
                stage,
                planning_capture_mode,
            )
        if hasattr(client, "get_pose_full"):
            capture_pos, capture_yaw, capture_rot = client.get_pose_full()
        else:
            capture_pos, capture_yaw = client.get_pose()
            capture_rot = None
        snapshot_pos = list(capture_pos)
        snapshot_yaw = float(capture_yaw)
        snapshot_depth_aligned = front_depth is not None or down_depth is not None
    snapshot_front_depth = front_depth if snapshot_depth_aligned else None
    snapshot_down_depth = down_depth if snapshot_depth_aligned else None
    if need_depth and front_depth is None:
        try:
            front_depth, down_depth, _depth_timing = _capture_completion_depth(
                client,
                objects.completion_checker,
                capture_mode,
            )
        except Exception as exc:
            print(f"  [ObstacleMemory] depth capture skipped: {exc}")
    if frame is None:
        print("  [Capture] no frame for Qwen plan, retry later")
        return None, None, False, None
    if capture_rot is None:
        print("  [Capture] full body rotation unavailable for Qwen plan, retry later")
        return None, None, False, None
    submitted = _submit_plan_if_needed(
        objects,
        client,
        stage,
        instruction,
        frame,
        down_frame,
        front_depth=front_depth,
        down_depth=down_depth,
        plan_pos=capture_pos,
        plan_yaw=capture_yaw,
        plan_rot=capture_rot,
    )
    memory_snapshot = None
    if (
        snapshot_pos is not None
        and snapshot_yaw is not None
        and snapshot_depth_aligned
        and (snapshot_front_depth is not None or snapshot_down_depth is not None)
    ):
        memory_snapshot = MemoryScanSnapshot(
            frame=frame,
            down_frame=down_frame,
            front_depth=snapshot_front_depth,
            down_depth=snapshot_down_depth,
            observer_world=tuple(float(v) for v in snapshot_pos[:3]),
            observer_yaw_deg=float(snapshot_yaw),
        )
    return frame, down_frame, submitted, memory_snapshot


def _complete_stage_from_vlm(objects, path_stream, state, completion, distance_m: float) -> bool:
    """Finish a stage only after the stopped, fresh dual-view VLM judge accepts it."""
    print(
        f"  [Completion] done=True source=vlm accepted={completion.accepted_view} "
        f"distance={distance_m:.2f}m reason={completion.reason}"
    )
    path_stream.stop()
    objects.controller.clear()
    if objects.completion_pipeline is not None:
        objects.completion_pipeline.clear()
    state.update(
        trajectory_queue=[],
        qwen_waypoints=[],
        trajectory_candidates=[],
        selected_trajectory={},
    )
    current_stage = objects.task_manager.current_stage()
    if getattr(objects, "mission_memory", None) is not None and current_stage is not None:
        objects.mission_memory.archive_stage(current_stage, completion.reason)
        state.update(memory_summary=objects.mission_memory.summary(current_stage))
    objects.task_manager.complete_current(completion.reason)
    completed_key = CompletionPipeline.stage_key(current_stage) if current_stage is not None else None
    if completed_key is not None:
        objects.completion_attempts.pop(completed_key, None)
        objects.completion_retry_after.pop(completed_key, None)
    print(f"  [TASK] {objects.task_manager.summary()}")
    task_done = objects.task_manager.is_done()
    if task_done:
        objects.navigation_metrics.task_completed = True
        state.update(status="done", task_done=True, step=0)
    return task_done


def _complete_stage_from_memory(objects, path_stream, state, stage, decision) -> bool:
    """Finish a stage when memory geometry reaches the strict completion threshold."""
    distance_text = "N/A" if decision.distance_m is None else f"{decision.distance_m:.2f}m"
    required_text = (
        "N/A" if decision.required_radius_m is None
        else f"{decision.required_radius_m:.2f}m"
    )
    geometry = str((decision.details or {}).get("geometry", "point") or "point")
    print(
        f"  [Completion] done=True source=mission_memory instance={decision.instance_id} "
        f"geometry={geometry} distance={distance_text} required<={required_text} "
        f"confidence={decision.confidence:.2f} reason={decision.reason}"
    )
    if decision.target_world is not None:
        objects.navigation_metrics.update_target(
            CompletionPipeline.stage_key(stage),
            decision.target_world,
            confidence=decision.confidence,
            replace_stage_reference=True,
        )
    if decision.distance_m is not None:
        objects.navigation_metrics.record_distance(decision.distance_m)
    path_stream.stop()
    objects.controller.clear()
    if objects.completion_pipeline is not None:
        objects.completion_pipeline.clear()
    state.update(
        trajectory_queue=[],
        qwen_waypoints=[],
        trajectory_candidates=[],
        selected_trajectory={},
    )
    objects.mission_memory.archive_stage(stage, decision.reason)
    state.update(memory_summary=objects.mission_memory.summary(stage))
    objects.task_manager.complete_current(decision.reason)
    completed_key = CompletionPipeline.stage_key(stage)
    objects.completion_attempts.pop(completed_key, None)
    objects.completion_retry_after.pop(completed_key, None)
    print(f"  [TASK] {objects.task_manager.summary()}")
    task_done = objects.task_manager.is_done()
    if task_done:
        objects.navigation_metrics.task_completed = True
        state.update(status="done", task_done=True, step=0)
    return task_done


def _clear_stopped_queue(objects, path_stream, state) -> None:
    emergency_stop = getattr(path_stream, "emergency_stop", None)
    if callable(emergency_stop):
        emergency_stop()
    else:
        path_stream.stop()
    objects.controller.clear()
    if objects.completion_pipeline is not None:
        objects.completion_pipeline.clear()
    state.update(
        trajectory_queue=[],
        qwen_waypoints=[],
        trajectory_candidates=[],
        selected_trajectory={},
    )


def _fail_current_stage_after_relocalization(objects, path_stream, state, stage_key, reason: str) -> bool:
    _clear_stopped_queue(objects, path_stream, state)
    objects.distance_estimator.clear()
    objects.navigation_metrics.invalidate_target(stage_key)
    objects.task_failed = True
    objects.task_manager.fail_current(reason)
    objects.completion_retry_after.pop(stage_key, None)
    print(f"  [Completion] done=False source=relocalization_failed reason={reason}")
    print(f"  [TASK] {objects.task_manager.summary()}")
    state.update(status="failed", task_done=False, step=0, error=reason)
    return True


def _search_with_memory_guidance(objects, client, stage, capture_mode: str, *, skip_initial_frame: bool):
    if (
        getattr(objects, "mission_memory", None) is not None
        and bool(objects.mission_memory.config.get("DIRECTED_RELOCALIZATION_ENABLED", True))
        and objects.mission_memory.has_primary(stage)
    ):
        pos_now, _yaw_now = client.get_pose()
        preferred_yaw = objects.mission_memory.preferred_yaw_deg(stage, pos_now)
        if preferred_yaw is not None:
            print(f"  [Relocalize] memory_guided_yaw={preferred_yaw:.1f}deg")
            try:
                client.rotate_to_yaw(preferred_yaw)
                skip_initial_frame = False
            except Exception as exc:
                print(f"  [Relocalize] memory-guided yaw failed: {exc}")
    return objects.relocalizer.search(
        client,
        stage,
        capture_mode=capture_mode,
        skip_initial_frame=skip_initial_frame,
    )


def _evaluate_memory_completion_now(
    objects,
    client,
    stage,
    *,
    fresh_visual_support: bool = False,
    visual_score: float = 0.0,
    stop_radius_m: float = 4.0,
):
    if getattr(objects, "mission_memory", None) is None:
        return None
    pos_now, _yaw_now = client.get_pose()
    return objects.mission_memory.evaluate_completion(
        stage=stage,
        current_world=pos_now,
        fresh_visual_support=fresh_visual_support,
        visual_score=visual_score,
        stop_radius_m=stop_radius_m,
    )


def _fresh_visual_support_matches_locked_memory(objects, stage, completion) -> bool:
    """Return whether completion vision supports the already locked target."""
    if not bool(getattr(completion, "target_detected", False)):
        return False
    if str(getattr(completion, "accepted_view", "none") or "none").lower() not in {"front", "down"}:
        return False

    reason = str(getattr(completion, "reason", "") or "").strip().lower()
    if reason == "target_mismatch":
        return False
    if reason != "outside_radius" or not is_view_relative_stage(stage):
        return True

    memory = getattr(objects, "mission_memory", None)
    instance = memory.primary_instance(stage) if memory is not None else None
    return not bool(
        instance is not None
        and getattr(instance, "is_large_structure", False)
        and has_surface_geometry(instance)
    )


def _handle_background_target_lost(
    objects: RuntimeObjects,
    client,
    path_stream,
    state,
    stage,
    capture_mode: str,
    reason: str,
) -> bool:
    """Stop immediately when the slow observation loop loses the target."""
    current_stage_key = CompletionPipeline.stage_key(stage)
    if (
        is_view_relative_stage(stage)
        and getattr(objects, "mission_memory", None) is not None
        and not objects.mission_memory.has_primary(stage)
    ):
        pos_now, _yaw_now = client.get_pose()
        expired_fn = getattr(objects.mission_memory, "view_relative_binding_expired", None)
        expired, wait_reason = (
            expired_fn(stage, pos_now)
            if callable(expired_fn)
            else (False, "waiting for delayed binding")
        )
        if expired:
            return _fail_current_stage_after_relocalization(
                objects,
                path_stream,
                state,
                current_stage_key,
                wait_reason,
            )
        print(f"  [TargetLost] delayed_binding_pending; {wait_reason}; current stage kept active")
        state.update(memory_summary=objects.mission_memory.summary(stage))
        return False
    if getattr(objects, "mission_memory", None) is not None and objects.mission_memory.has_primary(stage):
        pos_now, _yaw_now = client.get_pose()
        trusted_surface_fn = getattr(
            objects.mission_memory,
            "trusted_near_large_surface_estimate",
            None,
        )
        trusted_surface = (
            trusted_surface_fn(stage, pos_now)
            if callable(trusted_surface_fn)
            else None
        )
        if trusted_surface is not None:
            print(
                "  [TargetLost] ignored_by_locked_near_surface_memory "
                f"distance={trusted_surface['distance_m']:.2f}m "
                f"confidence={trusted_surface['confidence']:.2f} "
                f"uncertainty={trusted_surface['uncertainty_m']:.1f}m; "
                "continuing without yaw change"
            )
            state.update(memory_summary=objects.mission_memory.summary(stage))
            return False
        mem_distance = objects.mission_memory.estimate_distance(stage, pos_now)
        instance = objects.mission_memory.primary_instance(stage)
        memory_cfg = getattr(objects.mission_memory, "config", {}) or {}
        confidence = float((mem_distance or {}).get("confidence", 0.0) or 0.0)
        uncertainty = float((mem_distance or {}).get("uncertainty_m", 999.0) or 999.0)
        uses_surface = str((mem_distance or {}).get("distance_kind", "point")) == "surface"
        min_confidence = 0.65
        if uses_surface:
            min_confidence = float(memory_cfg.get("SURFACE_MEMORY_ONLY_MIN_CONFIDENCE", 0.65))
            if bool(getattr(instance, "is_large_structure", False)):
                min_confidence = float(memory_cfg.get("LARGE_STRUCTURE_MEMORY_ONLY_MIN_CONFIDENCE", 0.55))
        surface_fresh = not uses_surface or (
            instance is not None
            and instance.age_s() <= float(memory_cfg.get("SURFACE_COMPLETION_MAX_AGE_S", 120.0))
        )
        controller = getattr(objects, "controller", None)
        queue = getattr(controller, "queue", None)
        pending_world = list(getattr(queue, "world_waypoints", []) or [])
        navigation_active = bool(
            getattr(path_stream, "active", False)
            or pending_world
            or getattr(controller, "planning", False)
        )
        locked_primary_fn = getattr(objects.mission_memory, "is_primary_locked", None)
        locked_primary = bool(locked_primary_fn(stage)) if callable(locked_primary_fn) else False
        if (
            navigation_active
            and locked_primary
            and uses_surface
            and bool(getattr(instance, "is_large_structure", False))
            and surface_fresh
            and confidence >= float(
                memory_cfg.get("LARGE_STRUCTURE_SURFACE_LOCK_MIN_CONFIDENCE", 0.15)
            )
        ):
            print(
                "  [TargetLost] ignored_by_locked_surface_navigation "
                f"distance={float((mem_distance or {}).get('distance_m', 0.0)):.2f}m "
                f"confidence={confidence:.2f} uncertainty={uncertainty:.1f}m; "
                "active path kept without yaw change"
            )
            state.update(memory_summary=objects.mission_memory.summary(stage))
            return False
        if (
            confidence >= min_confidence
            and uncertainty <= float(memory_cfg.get("MAX_COMPLETION_UNCERTAINTY_M", 5.0))
            and surface_fresh
        ):
            # 有稳定锁定实例时，单帧/单轮检测丢失不立刻停车清空；继续用memory引导规划。
            print(
                f"  [TargetLost] ignored_by_memory geometry={'surface' if uses_surface else 'point'} "
                f"confidence={confidence:.2f} "
                f"uncertainty={uncertainty:.1f}m reason={reason}"
            )
            state.update(memory_summary=objects.mission_memory.summary(stage))
            return False
    _clear_stopped_queue(objects, path_stream, state)
    objects.distance_estimator.clear()
    objects.navigation_metrics.invalidate_target(current_stage_key)
    print(f"  [TargetLost] stopped, cleared queue, relocalizing reason={reason}")
    if not objects.relocalizer.enabled:
        return _fail_current_stage_after_relocalization(
            objects,
            path_stream,
            state,
            current_stage_key,
            "target lost and relocalization disabled",
        )

    relocalized = _search_with_memory_guidance(
        objects,
        client,
        stage,
        capture_mode,
        skip_initial_frame=True,
    )
    print(
        f"  [Relocalize] found={relocalized.found} "
        f"view={(relocalized.detection.camera if relocalized.detection else 'none')} "
        f"yaw_delta={relocalized.yaw_delta_deg:.1f}deg "
        f"elapsed={relocalized.elapsed:.2f}s reason={relocalized.reason}"
    )
    if not relocalized.found:
        return _fail_current_stage_after_relocalization(
            objects,
            path_stream,
            state,
            current_stage_key,
            relocalized.reason or "target not found after relocalization",
        )

    # Relocalization only turns the vehicle back toward the target.  The next
    # loop starts fresh planning and fresh detector/depth observations.
    _clear_stopped_queue(objects, path_stream, state)
    objects.distance_estimator.clear()
    objects.navigation_metrics.invalidate_target(current_stage_key)
    return False


def _handle_distance_completion_trigger(
    objects: RuntimeObjects,
    client,
    path_stream,
    state,
    stage,
    task_text,
    capture_mode: str,
    cached_distance,
    trigger_radius_m: float,
) -> bool:
    current_stage_key = CompletionPipeline.stage_key(stage)
    cached_distance_m = float(
        cached_distance.get("distance_m") if isinstance(cached_distance, dict)
        else getattr(cached_distance, "distance_m", 0.0)
    )
    trigger_display_m = float(
        cached_distance.get("trigger_radius_m") if isinstance(cached_distance, dict) and cached_distance.get("trigger_radius_m") is not None
        else getattr(cached_distance, "trigger_radius_m", trigger_radius_m)
    )
    _clear_stopped_queue(objects, path_stream, state)
    source = str(
        cached_distance.get("source", "distance_estimator")
        if isinstance(cached_distance, dict)
        else getattr(cached_distance, "source", "distance_estimator")
    )
    distance_kind = str(
        cached_distance.get("distance_kind", "point")
        if isinstance(cached_distance, dict)
        else getattr(cached_distance, "distance_kind", "point")
    )
    print(
        f"\n  [CompletionTrigger] source={source} geometry={distance_kind} "
        f"distance={cached_distance_m:.2f}m "
        f"<= {trigger_display_m:.2f}m; stopped, cleared queue, checking fresh evidence"
    )

    memory = getattr(objects, "mission_memory", None)
    # Large structures are intentionally completed from the locked surface
    # memory once the XY arrival radius is reached.  A close facade is often
    # only a partial detector view, so capturing fresh RGB here would let the
    # VLM reject a geometrically valid arrival.  Small targets fall through to
    # the existing fresh RGB + VLM confirmation path below.
    large_surface_arrival_fn = getattr(memory, "evaluate_large_surface_arrival", None)
    if callable(large_surface_arrival_fn):
        pos_now, _yaw_now = client.get_pose()
        primary = memory.primary_instance(stage) if memory is not None else None
        locked_fn = getattr(memory, "is_primary_locked", None) if memory is not None else None
        locked = bool(locked_fn(stage)) if callable(locked_fn) else False
        surface_count = len(instance_surface_sample_points(primary)) if primary is not None else 0
        print(
            "  [CompletionMemoryCheck] "
            f"primary={(primary.instance_id if primary is not None else 'none')} "
            f"locked={locked} "
            f"large={bool(getattr(primary, 'is_large_structure', False)) if primary is not None else False} "
            f"surface_points={surface_count} "
            f"relation={relation_kind(stage)}"
        )
        large_surface_decision = large_surface_arrival_fn(
            stage,
            pos_now,
            radius_m=float(memory.config.get("SURFACE_APPROACH_RADIUS_M", 4.5)),
        )
        if large_surface_decision is not None:
            print(
                "  [MemoryCompletion] large surface XY arrival; "
                "skipping fresh RGB/VLM "
                f"distance={large_surface_decision.distance_m:.2f}m "
                f"reason={large_surface_decision.reason}"
            )
            state.update(memory_summary=memory.summary(stage))
            return _complete_stage_from_memory(
                objects,
                path_stream,
                state,
                stage,
                large_surface_decision,
            )
        if (
            primary is not None
            and callable(locked_fn)
            and locked_fn(stage)
            and bool(getattr(primary, "is_large_structure", False))
            and has_surface_samples(primary)
            and relation_kind(stage) == "near"
        ):
            # A large target that is not yet inside the fixed 4.5m circle
            # resumes navigation; it must never fall through to fresh VLM.
            print("  [MemoryCompletion] large surface outside 4.5m XY radius; resuming navigation")
            state.update(memory_summary=memory.summary(stage))
            return False

    recent_contact_fn = getattr(memory, "recent_locked_large_surface_contact", None)
    recent_contact = recent_contact_fn(stage) if callable(recent_contact_fn) else None
    if recent_contact is not None:
        decision = _evaluate_memory_completion_now(
            objects,
            client,
            stage,
            fresh_visual_support=False,
            stop_radius_m=float(trigger_radius_m),
        )
        if decision is not None:
            print(
                f"  [TargetSurfaceContact] kind={recent_contact['kind']} "
                f"age={recent_contact['age_s']:.1f}s status={decision.status} "
                f"confidence={decision.confidence:.2f} reason={decision.reason}"
            )
            if decision.done:
                return _complete_stage_from_memory(objects, path_stream, state, stage, decision)

    frame, down_frame, rgb_elapsed = _capture_fresh_rgb_frames(
        client,
        objects.completion_checker,
        capture_mode,
    )
    if frame is None:
        return _fail_current_stage_after_relocalization(
            objects,
            path_stream,
            state,
            current_stage_key,
            "fresh RGB capture failed",
        )

    best_det, front_det, down_det, detect_elapsed, front_all, down_all = _detect_dual_view(
        objects,
        stage,
        task_text,
        frame,
        down_frame,
    )
    if best_det is None or not getattr(best_det, "visible", False):
        decision = _evaluate_memory_completion_now(
            objects,
            client,
            stage,
            fresh_visual_support=False,
            stop_radius_m=float(trigger_radius_m),
        )
        if decision is not None:
            print(
                f"  [MemoryCompletion] status={decision.status} "
                f"confidence={decision.confidence:.2f} reason={decision.reason}"
            )
            state.update(memory_summary=objects.mission_memory.summary(stage))
            if decision.done:
                return _complete_stage_from_memory(objects, path_stream, state, stage, decision)
        if _trusted_near_surface_completion(objects, stage, client):
            return _handle_inconclusive_completion_attempt(objects, path_stream, state, stage)
        objects.distance_estimator.clear()
        objects.navigation_metrics.invalidate_target(current_stage_key)
        if not objects.relocalizer.enabled:
            return _fail_current_stage_after_relocalization(
                objects,
                path_stream,
                state,
                current_stage_key,
                "fresh target not detected and relocalization disabled",
            )
        print("  [Relocalize] fresh RGB lost the target; scanning 360deg")
        relocalized = _search_with_memory_guidance(
            objects,
            client,
            stage,
            capture_mode,
            skip_initial_frame=True,
        )
        print(
            f"  [Relocalize] found={relocalized.found} "
            f"view={(relocalized.detection.camera if relocalized.detection else 'none')} "
            f"yaw_delta={relocalized.yaw_delta_deg:.1f}deg "
            f"elapsed={relocalized.elapsed:.2f}s reason={relocalized.reason}"
        )
        if not relocalized.found:
            return _fail_current_stage_after_relocalization(
                objects,
                path_stream,
                state,
                current_stage_key,
                relocalized.reason or "target not found after relocalization",
            )

        frame, down_frame, rgb_elapsed = _capture_fresh_rgb_frames(
            client,
            objects.completion_checker,
            capture_mode,
        )
        if frame is None:
            return _fail_current_stage_after_relocalization(
                objects,
                path_stream,
                state,
                current_stage_key,
                "fresh RGB capture failed after relocalization",
            )
        best_det, front_det, down_det, detect_elapsed, front_all, down_all = _detect_dual_view(
            objects,
            stage,
            task_text,
            frame,
            down_frame,
        )
        if best_det is None or not getattr(best_det, "visible", False):
            decision = _evaluate_memory_completion_now(
                objects,
                client,
                stage,
                fresh_visual_support=False,
                stop_radius_m=float(trigger_radius_m),
            )
            if decision is not None:
                print(
                    f"  [MemoryCompletion] status={decision.status} "
                    f"confidence={decision.confidence:.2f} reason={decision.reason}"
                )
                state.update(memory_summary=objects.mission_memory.summary(stage))
                if decision.done:
                    return _complete_stage_from_memory(objects, path_stream, state, stage, decision)
            if _trusted_near_surface_completion(objects, stage, client):
                return _handle_inconclusive_completion_attempt(objects, path_stream, state, stage)
            return _fail_current_stage_after_relocalization(
                objects,
                path_stream,
                state,
                current_stage_key,
                "target lost after relocalization",
            )

    depth_future = objects.slow_executor.submit(
        _capture_completion_depth,
        client,
        objects.completion_checker,
        capture_mode,
    )
    front_depth, down_depth, depth_elapsed = _wait_completion_depth(depth_future)
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
    _print_completion_evidence(stage, front_det, down_det, frame, down_frame)

    judge_started = time.perf_counter()
    completion = objects.completion_checker.evaluate_with_detection(
        stage,
        task_text,
        frame,
        down_frame,
        best_det,
        front_detection=front_det,
        down_detection=down_det,
        front_depth_meters=front_depth,
        down_depth_meters=down_depth,
        estimated_distance_m=cached_distance_m,
    )
    completion.capture_elapsed = max(rgb_elapsed, depth_elapsed)
    completion.detect_elapsed = detect_elapsed
    completion.judge_elapsed = time.perf_counter() - judge_started
    print(
        f"  [CompletionVLM] done={completion.done} detected={completion.target_detected} "
        f"accepted={completion.accepted_view} reason={completion.reason} "
        f"elapsed={completion.elapsed:.2f}s"
    )
    if completion.done:
        return _complete_stage_from_vlm(
            objects,
            path_stream,
            state,
            completion,
            cached_distance_m,
        )

    visual_score = max(
        _detection_reliability(stage, front_det, frame),
        _detection_reliability(stage, down_det, down_frame),
    )
    fresh_visual_support = _fresh_visual_support_matches_locked_memory(
        objects,
        stage,
        completion,
    )
    if not fresh_visual_support and is_view_relative_stage(stage):
        print(
            "  [MemoryCompletion] ignoring incompatible fresh detection; "
            "using locked surface memory"
        )
    decision = _evaluate_memory_completion_now(
        objects,
        client,
        stage,
        fresh_visual_support=fresh_visual_support,
        visual_score=visual_score if fresh_visual_support else 0.0,
        stop_radius_m=float(trigger_radius_m),
    )
    if decision is not None:
        print(
            f"  [MemoryCompletion] status={decision.status} "
            f"confidence={decision.confidence:.2f} reason={decision.reason}"
        )
        state.update(memory_summary=objects.mission_memory.summary(stage))
        if decision.done:
            return _complete_stage_from_memory(objects, path_stream, state, stage, decision)

    return _handle_inconclusive_completion_attempt(objects, path_stream, state, stage)


def _completion_retry_waiting(objects, stage) -> tuple[bool, float]:
    key = CompletionPipeline.stage_key(stage)
    retry_after = float(objects.completion_retry_after.get(key, 0.0) or 0.0)
    remaining = retry_after - time.perf_counter()
    if remaining <= 0.0:
        objects.completion_retry_after.pop(key, None)
        return False, 0.0
    return True, remaining


def _record_locked_target_collision(objects, stage, collision_world, collision_yaw_deg: float):
    """Turn a very-near collision into stage-specific facade contact evidence."""
    memory = getattr(objects, "mission_memory", None)
    if memory is None or not memory.has_primary(stage) or not memory.is_primary_locked(stage):
        return None
    context = memory.candidate_context(
        stage=stage,
        current_world=collision_world,
        yaw_deg=float(collision_yaw_deg),
    )
    if not bool(context.get("is_large_structure", False)) or not bool(context.get("uses_surface_geometry", False)):
        return None
    target_body = context.get("target_body") or []
    # Collision contact is a finite-patch safety signal.  Unlike completion,
    # it may project onto the patch interior, because sparse corner samples
    # can be farther than the physical facade point that blocked the vehicle.
    instance = memory.primary_instance(stage)
    collision_geometry_distance = (
        distance_to_instance_geometry(collision_world, instance)
        if instance is not None
        else float("inf")
    )
    max_distance = float(memory.config.get("LARGE_STRUCTURE_COLLISION_CONTACT_MAX_DISTANCE_M", 2.5))
    max_lateral = float(memory.config.get("LARGE_STRUCTURE_COLLISION_CONTACT_MAX_LATERAL_M", 4.0))
    if (
        collision_geometry_distance > max_distance
        or len(target_body) < 3
        or float(target_body[0]) < -1.0
        or abs(float(target_body[1])) > max_lateral
    ):
        return None
    forward_offset = float(memory.config.get("LARGE_STRUCTURE_COLLISION_CONTACT_FORWARD_M", 0.8))
    return memory.record_locked_large_surface_contact(
        stage,
        observer_world=collision_world,
        observer_yaw_deg=float(collision_yaw_deg),
        obstacle_body=[max(0.1, forward_offset), 0.0, 0.0],
        contact_kind="collision",
    )


def _cached_target_distance(objects, stage, current_world):
    """Read the cheap cached target distance independently of waypoint events."""
    estimated = objects.distance_estimator.estimate_distance(
        stage_key=CompletionPipeline.stage_key(stage),
        current_world=current_world,
    )
    memory_estimate = None
    trusted_memory_estimate = None
    force_locked_large_surface_memory = False
    if getattr(objects, "mission_memory", None) is not None:
        memory = objects.mission_memory
        instance = memory.primary_instance(stage)
        locked_fn = getattr(memory, "is_primary_locked", None)
        force_locked_large_surface_memory = bool(
            instance is not None
            and callable(locked_fn)
            and locked_fn(stage)
            and bool(getattr(instance, "is_large_structure", False))
            and has_surface_samples(instance)
            and relation_kind(stage) == "near"
        )
        trusted_surface_fn = getattr(
            memory,
            "trusted_near_large_surface_estimate",
            None,
        )
        if callable(trusted_surface_fn):
            trusted_memory_estimate = trusted_surface_fn(stage, current_world)
        memory_estimate = (
            memory.estimate_distance(stage, current_world)
            if force_locked_large_surface_memory
            else trusted_memory_estimate
            if trusted_memory_estimate is not None
            else memory.estimate_distance(stage, current_world)
        )
    if memory_estimate is not None:
        memory_ns = SimpleNamespace(**memory_estimate)
        memory_cfg = getattr(objects.mission_memory, "config", {}) or {}
        prefer_memory = bool(memory_cfg.get("PREFER_MEMORY_DISTANCE", True))
        memory_conf = float(getattr(memory_ns, "confidence", 0.0) or 0.0)
        memory_uncertainty = float(getattr(memory_ns, "uncertainty_m", 999.0) or 999.0)
        max_uncertainty = float(memory_cfg.get("MAX_COMPLETION_UNCERTAINTY_M", 5.0))
        if (
            estimated is None
            or force_locked_large_surface_memory
            or (
                prefer_memory
                and (
                    trusted_memory_estimate is not None
                    or memory_conf >= float(memory_cfg.get("MEMORY_DISTANCE_MIN_CONFIDENCE", 0.45))
                )
                and memory_uncertainty <= max_uncertainty * float(memory_cfg.get("MEMORY_DISTANCE_UNCERTAINTY_RATIO", 1.5))
            )
        ):
            # 锁定实例已经稳定时，完成触发优先使用memory距离；单帧距离估计偶尔会跳到远处。
            estimated = memory_ns
    if estimated is None:
        return None
    if str(getattr(estimated, "source", "") or "") == "mission_memory":
        objects.navigation_metrics.update_target(
            CompletionPipeline.stage_key(stage),
            estimated.target_world,
            confidence=float(getattr(estimated, "confidence", 0.0) or 0.0),
            replace_stage_reference=True,
        )
    objects.navigation_metrics.record_distance(estimated.distance_m)
    return estimated


def _memory_distance_trigger_radius(objects, stage, trigger_radius_m: float) -> float:
    memory = getattr(objects, "mission_memory", None)
    if memory is None:
        return float(trigger_radius_m)
    instance = memory.primary_instance(stage)
    if instance is None:
        return float(trigger_radius_m)
    memory_cfg = getattr(memory, "config", {}) or {}
    uncertainty = instance.effective_uncertainty(
        stale_growth_per_s=float(memory_cfg.get("STALE_UNCERTAINTY_GROWTH_MPS", 0.03))
    )
    max_uncertainty = float(memory_cfg.get("MAX_COMPLETION_UNCERTAINTY_M", 5.0))
    if _is_above_stage(stage):
        return float(trigger_radius_m)
    uses_surface = bool(
        has_surface_geometry(instance)
    )
    if uses_surface:
        # Memory distance is already measured to the nearest observed surface.
        # Do not add the building/car footprint a second time.
        return max(
            float(trigger_radius_m),
            float(memory_cfg.get("SURFACE_APPROACH_RADIUS_M", memory_cfg.get("NEAR_APPROACH_RADIUS_M", 4.5))),
        )
    outer_radius = max(
        float(memory_cfg.get("NEAR_STANDOFF_M", trigger_radius_m))
        + float(memory_cfg.get("LOW_ALTITUDE_EXTRA_STANDOFF_M", 0.0)),
        float(trigger_radius_m)
        + float(memory_cfg.get("NEAR_RADIUS_MARGIN_M", 1.0))
        + min(max(float(uncertainty), 0.0), max_uncertainty),
    )
    # 完成判定允许“圆内任意点”，但飞行触发不要卡在外圆边界；先飞进更自然的内圈。
    return max(
        float(trigger_radius_m),
        _memory_near_approach_radius_from_values(
            memory_cfg,
            footprint=float(getattr(instance, "footprint_radius_m", 1.5) or 1.5),
            uncertainty=float(uncertainty),
            outer_radius=outer_radius,
        ),
    )


def _memory_completion_outer_radius(objects, stage, trigger_radius_m: float) -> float:
    memory = getattr(objects, "mission_memory", None)
    if memory is None:
        return float(trigger_radius_m)
    instance = memory.primary_instance(stage)
    if instance is None:
        return float(trigger_radius_m)
    memory_cfg = getattr(memory, "config", {}) or {}
    uncertainty = instance.effective_uncertainty(
        stale_growth_per_s=float(memory_cfg.get("STALE_UNCERTAINTY_GROWTH_MPS", 0.03))
    )
    if _is_above_stage(stage):
        return (
            float(getattr(instance, "footprint_radius_m", 1.5) or 1.5)
            + float(memory_cfg.get("ABOVE_HORIZONTAL_RADIUS_M", 3.5))
            + min(max(float(uncertainty), 0.0), float(memory_cfg.get("MAX_COMPLETION_UNCERTAINTY_M", 5.0)))
        )
    uses_surface = bool(
        has_surface_geometry(instance)
    )
    if uses_surface:
        return max(
            float(trigger_radius_m),
            float(memory_cfg.get("SURFACE_NEAR_RADIUS_M", memory_cfg.get("NEAR_STANDOFF_M", 6.0))),
        )
    return max(
        float(memory_cfg.get("NEAR_STANDOFF_M", trigger_radius_m))
        + float(memory_cfg.get("LOW_ALTITUDE_EXTRA_STANDOFF_M", 0.0)),
        float(trigger_radius_m)
        + float(memory_cfg.get("NEAR_RADIUS_MARGIN_M", 1.0))
        + min(max(float(uncertainty), 0.0), float(memory_cfg.get("MAX_COMPLETION_UNCERTAINTY_M", 5.0))),
    )


def _should_trigger_completion_vlm(objects, stage, estimated, trigger_radius_m: float) -> bool:
    if estimated is None:
        return False
    source = str(getattr(estimated, "source", "distance_estimator") or "distance_estimator")
    if source != "mission_memory" and not objects.distance_estimator.use_for_completion:
        return False
    radius = _memory_distance_trigger_radius(objects, stage, trigger_radius_m) if source == "mission_memory" else float(trigger_radius_m)
    try:
        estimated.trigger_radius_m = radius
    except Exception:
        pass
    return bool(float(estimated.distance_m) <= float(radius))


def _queue_reaches_memory_arrival(objects, stage, trigger_radius_m: float) -> bool:
    memory = getattr(objects, "mission_memory", None)
    if memory is None or not memory.has_primary(stage):
        return False
    radius = _memory_distance_trigger_radius(objects, stage, trigger_radius_m)
    for waypoint in list(objects.controller.queue.world_waypoints or []):
        estimate = memory.estimate_distance(stage, waypoint)
        if estimate is not None and float(estimate.get("distance_m", 999.0)) <= radius:
            return True
    return False


def _truncate_queue_at_completion_radius(
    objects,
    path_stream,
    state,
    stage,
    current_world,
    cached_distance,
    trigger_radius_m: float,
) -> str:
    """Stop/shorten a path before it crosses the target completion circle.

    The distance trigger is evaluated from the actual pose every runtime
    iteration, but an AirSim path may contain several future waypoints.  If a
    segment crosses the target circle, leaving those waypoints active can send
    the vehicle through the target while the completion check is running.
    Keep only the prefix up to the first XY circle entry and reissue that
    bounded path on the next scheduler pass.  ``arrived`` means the current
    pose is already inside the circle and lets the caller start completion
    immediately.
    """
    if cached_distance is None:
        return "none"
    relation = relation_kind(stage)
    if relation != "near":
        # Above/over stages still use their relation-specific 3-D completion
        # rules; this guard is deliberately the near-target XY contract.
        return "none"
    queue = list(getattr(getattr(objects, "controller", None), "queue", SimpleNamespace(world_waypoints=[])).world_waypoints or [])
    if not queue:
        return "none"

    def value(name: str, default=None):
        if isinstance(cached_distance, dict):
            return cached_distance.get(name, default)
        return getattr(cached_distance, name, default)

    target = value("target_world") or []
    if len(target) < 3:
        return "none"
    targets = [[float(target[0]), float(target[1]), float(target[2])]]
    source = str(value("source", "distance_estimator") or "distance_estimator")
    memory = getattr(objects, "mission_memory", None)
    instance = memory.primary_instance(stage) if source == "mission_memory" and memory is not None else None
    if instance is not None and has_surface_samples(instance):
        samples = instance_surface_sample_points(instance)
        if samples:
            # Completion is the union of XY circles around every retained
            # surface sample, so path clipping must use the same geometry.
            targets = samples
    radius = value("trigger_radius_m", None)
    if radius is None:
        radius = (
            _memory_distance_trigger_radius(objects, stage, trigger_radius_m)
            if source == "mission_memory"
            else float(trigger_radius_m)
        )
    radius = max(0.0, float(radius))
    fast_slow_cfg = {**(cfg.get("FAST_SLOW", {}) or {}), **function_section(cfg, "FAST_SLOW")}
    entry_margin = max(
        0.0,
        min(
            float(fast_slow_cfg.get("COMPLETION_PATH_ENTRY_MARGIN_M", 0.25)),
            max(0.0, radius - 0.1),
        ),
    )
    path_entry_radius = max(0.0, radius - entry_margin)
    current = [float(v) for v in current_world[:3]]
    current_distance = min(
        math.hypot(current[0] - candidate[0], current[1] - candidate[1])
        for candidate in targets
    )
    if current_distance <= radius + 1e-6:
        return "arrived"

    prefix: list[list[float]] = []
    previous = current
    for index, waypoint in enumerate(queue):
        if waypoint is None or len(waypoint) < 3:
            continue
        point = [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])]
        entries = [
            entry
            for candidate in targets
            if (entry := _segment_circle_entry_xy(previous, point, candidate, path_entry_radius)) is not None
        ]
        entry = min(entries, key=lambda candidate: _distance3_body(previous, candidate)) if entries else None
        endpoint_distance = min(
            math.hypot(point[0] - candidate[0], point[1] - candidate[1])
            for candidate in targets
        )
        if entry is not None or endpoint_distance <= path_entry_radius:
            # This queue is already bounded at the circle (for example the
            # boundary waypoint left by the previous guard pass).  Let the
            # path stream issue/finish that one waypoint instead of stopping
            # it on every 50 ms scheduler tick.
            tail_has_outside_point = any(
                min(
                    math.hypot(float(rest[0]) - candidate[0], float(rest[1]) - candidate[1])
                    for candidate in targets
                ) > path_entry_radius + 1e-6
                for rest in queue[index:]
                if rest is not None and len(rest) >= 3
            )
            if (
                endpoint_distance <= path_entry_radius
                and not tail_has_outside_point
            ):
                return "none"
            boundary = entry or point
            prefix.append([round(float(v), 3) for v in boundary])
            # The path stream owns AirSim's active command. Stop it before
            # mutating the queue so its old remaining list cannot be polled
            # against the shortened queue on the next iteration.
            emergency_stop = getattr(path_stream, "emergency_stop", None)
            if callable(emergency_stop):
                emergency_stop()
            else:
                path_stream.stop()
            controller = getattr(objects, "controller", None)
            if controller is not None:
                discard = getattr(controller, "discard_plan", None)
                if callable(discard):
                    discard()
                controller.queue.world_waypoints[:] = prefix
            state.update(
                trajectory_queue=[list(wp) for wp in prefix],
                qwen_waypoints=[],
            )
            print(
                "  [CompletionPathGuard] truncated at first XY radius entry "
                f"completion_radius={radius:.2f}m path_entry={path_entry_radius:.2f}m "
                f"boundary={prefix[-1]} discarded={max(0, len(queue) - len(prefix))}"
            )
            return "truncated"
        prefix.append([round(float(v), 3) for v in point])
        previous = point
    return "none"


def _active_queue_overshoots_locked_surface(objects, stage, current_world, trigger_radius_m: float) -> bool:
    """Detect a stale world queue that enters a locked facade radius and then exits it."""
    memory = getattr(objects, "mission_memory", None)
    if memory is None or not memory.has_primary(stage) or not memory.is_primary_locked(stage):
        return False
    instance = memory.primary_instance(stage)
    if (
        instance is None
        or not bool(getattr(instance, "is_large_structure", False))
        or not has_surface_geometry(instance)
        or relation_kind(stage) != "near"
    ):
        return False
    waypoints = list(getattr(getattr(objects, "controller", None), "queue", SimpleNamespace(world_waypoints=[])).world_waypoints or [])
    if not waypoints:
        return False
    estimate = memory.estimate_distance(stage, current_world)
    target = list((estimate or {}).get("target_world") or [])
    if len(target) < 3:
        return False
    radius = _memory_distance_trigger_radius(objects, stage, trigger_radius_m)
    exit_margin = max(0.2, float((getattr(memory, "config", {}) or {}).get("PATH_EXIT_MARGIN_M", 2.0)))
    exit_radius = radius + exit_margin
    prev = [float(v) for v in current_world[:3]]
    entered = math.sqrt((prev[0] - target[0]) ** 2 + (prev[1] - target[1]) ** 2) <= radius
    for waypoint in waypoints:
        cur = [float(v) for v in waypoint[:3]]
        segment_enters = _segment_point_distance_xy(prev, cur, target) <= radius
        endpoint_distance = math.sqrt((cur[0] - target[0]) ** 2 + (cur[1] - target[1]) ** 2)
        if not entered and segment_enters:
            entered = True
        if entered and endpoint_distance > exit_radius:
            return True
        prev = cur
    return False


def _invalidate_unsafe_locked_surface_queue(objects, path_stream, state, stage, current_world, trigger_radius_m: float) -> bool:
    if not _active_queue_overshoots_locked_surface(objects, stage, current_world, trigger_radius_m):
        return False
    _clear_stopped_queue(objects, path_stream, state)
    print("  [MemoryPathGuard] stopped stale active queue that crossed the locked facade arrival radius")
    return True


def _should_trigger_idle_memory_completion(objects, stage, estimated, trigger_radius_m: float) -> bool:
    if estimated is None:
        return False
    if str(getattr(estimated, "source", "") or "") != "mission_memory":
        return False
    memory = getattr(objects, "mission_memory", None)
    if memory is None or not bool(memory.config.get("IDLE_OUTER_COMPLETION_TRIGGER_ENABLED", True)):
        return False
    if objects.controller.planning or objects.controller.has_plan_job:
        return False
    if objects.controller.queue.world_waypoints:
        return False
    instance = memory.primary_instance(stage)
    locked_fn = getattr(memory, "is_primary_locked", None)
    if (
        instance is not None
        and callable(locked_fn)
        and locked_fn(stage)
        and bool(getattr(instance, "is_large_structure", False))
        and has_surface_samples(instance)
        and relation_kind(stage) == "near"
    ):
        # Large structures bypass VLM entirely.  Do not use the wider outer
        # radius as a reason to launch a visual confirmation before 4.5m.
        outer_radius = float(
            memory.config.get("SURFACE_APPROACH_RADIUS_M", 4.5)
        )
    else:
        outer_radius = _memory_completion_outer_radius(objects, stage, trigger_radius_m)
    try:
        estimated.trigger_radius_m = outer_radius
    except Exception:
        pass
    return bool(float(getattr(estimated, "distance_m", 999.0)) <= float(outer_radius))


def _completion_needs_relocalization(completion) -> bool:
    """Relocalize only when fresh dual-view completion evidence lost the target."""
    return bool(completion is None or getattr(completion, "target_detected", None) is not True)


def _trusted_near_surface_completion(objects, stage, client) -> bool:
    """Whether completion is close enough to trust the locked facade without a turn."""
    memory = getattr(objects, "mission_memory", None)
    if memory is None or not hasattr(memory, "trusted_near_large_surface_estimate"):
        return False
    try:
        pos_now, _yaw_now = client.get_pose()
        return memory.trusted_near_large_surface_estimate(stage, pos_now) is not None
    except Exception:
        return False


def _handle_inconclusive_completion_attempt(objects, path_stream, state, stage) -> bool:
    """Count a stopped completion check; the second check must terminate the stage."""
    current_stage_key = CompletionPipeline.stage_key(stage)
    objects.distance_estimator.clear()
    objects.navigation_metrics.invalidate_target(current_stage_key)
    attempts = int(objects.completion_attempts.get(current_stage_key, 0)) + 1
    objects.completion_attempts[current_stage_key] = attempts
    completion_cfg = function_section(cfg, "FAST_SLOW")
    max_attempts = max(1, int(completion_cfg.get("MAX_COMPLETION_ATTEMPTS", 2)))
    if attempts >= max_attempts:
        return _fail_current_stage_after_relocalization(
            objects,
            path_stream,
            state,
            current_stage_key,
            f"completion evidence remained inconsistent after {attempts} stopped checks",
        )
    cooldown = max(0.1, float(completion_cfg.get("COMPLETION_RETRY_COOLDOWN_S", 0.2)))
    objects.completion_retry_after[current_stage_key] = time.perf_counter() + cooldown
    print(
        f"  [CompletionVLM] not complete; holding position before retry "
        f"attempt={attempts}/{max_attempts} cooldown={cooldown:.1f}s"
    )
    return False


def _signed_yaw_delta_deg(target_yaw_deg: float, current_yaw_deg: float) -> float:
    return (float(target_yaw_deg) - float(current_yaw_deg) + 180.0) % 360.0 - 180.0


def _reorient_to_locked_target_if_behind(
    objects,
    client,
    path_stream,
    state,
    stage,
    *,
    allow_queued: bool = False,
) -> bool:
    """Turn toward a locked target before a ForwardOnly path is planned/issued."""
    memory = getattr(objects, "mission_memory", None)
    controller = getattr(objects, "controller", None)
    if memory is None or controller is None or stage is None:
        return False
    memory_cfg = getattr(memory, "config", {}) or {}
    if not bool(memory_cfg.get("RETURN_TARGET_REORIENT_ENABLED", True)):
        return False
    if getattr(stage, "mode", "") not in {"target", "detect"} or relation_kind(stage) == "pass":
        return False
    if not memory.has_primary(stage):
        return False
    if bool(getattr(controller, "planning", False)) or bool(getattr(controller, "has_plan_job", False)):
        return False
    queued = list(getattr(getattr(controller, "queue", None), "world_waypoints", []) or [])
    if queued and not allow_queued:
        return False

    current_pos, current_yaw = client.get_pose()
    preferred_yaw = memory.preferred_yaw_deg(stage, current_pos)
    if preferred_yaw is None:
        return False
    yaw_delta = _signed_yaw_delta_deg(preferred_yaw, current_yaw)
    trigger_deg = max(45.0, float(memory_cfg.get("RETURN_TARGET_REORIENT_TRIGGER_DEG", 90.0)))
    if abs(yaw_delta) < trigger_deg:
        return False

    path_stream.stop()
    print(
        f"  [MemoryReorient] target_behind yaw={float(current_yaw):.1f}->"
        f"{float(preferred_yaw):.1f} delta={yaw_delta:.1f}deg"
    )
    try:
        state.update(status="reorienting", pose=list(current_pos), yaw=float(current_yaw))
        timeout = float(memory_cfg.get("RETURN_TARGET_REORIENT_TIMEOUT_S", 8.0))
        client.rotate_to_yaw(float(preferred_yaw), timeout=timeout)
        new_pos, new_yaw = client.get_pose()
        memory.record_pose(stage, new_pos, new_yaw)
        state.update(
            status="running",
            pose=list(new_pos),
            yaw=float(new_yaw),
            memory_summary=memory.summary(stage),
        )
        print(f"  [MemoryReorient] complete yaw={float(new_yaw):.1f}")
        return True
    except Exception as exc:
        state.update(status="running")
        print(f"  [MemoryReorient] failed: {exc}")
        return False


def _drop_path_points_behind_vehicle(objects, current_pos, current_yaw_deg) -> int:
    """Remove queue-head points that are behind the current vehicle heading."""
    waypoints = objects.controller.queue.world_waypoints
    if not waypoints:
        return 0

    yaw = math.radians(float(current_yaw_deg))
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)
    dropped = 0
    while waypoints:
        point = waypoints[0]
        dx = float(point[0]) - float(current_pos[0])
        dy = float(point[1]) - float(current_pos[1])
        forward_projection = dx * forward_x + dy * forward_y
        if forward_projection >= 0.0:
            break
        waypoints.pop(0)
        dropped += 1
    if dropped:
        print(
            f"  [PathProgress] dropped_behind={dropped} "
            f"pose=({float(current_pos[0]):.2f},{float(current_pos[1]):.2f},{float(current_pos[2]):.2f}) "
            f"yaw={float(current_yaw_deg):.1f}"
        )
    return dropped


def _sync_path_if_ready(
    objects,
    path_stream,
    client,
    state,
    stage=None,
) -> int:
    current_pos, current_yaw = client.get_pose()
    progress = path_stream.poll(objects.controller.queue.world_waypoints, current_pos)
    if progress.consumed > 0:
        objects.controller.mark_executed(progress.consumed)
        print(
            f"  [FlightPose] world=({current_pos[0]:.2f},{current_pos[1]:.2f},{current_pos[2]:.2f}) "
            f"yaw={current_yaw:.1f} consumed={progress.consumed} "
            f"remaining={len(objects.controller.queue.world_waypoints)}"
        )
        state.update(
            pose=current_pos,
            yaw=current_yaw,
            trajectory_queue=[list(wp) for wp in objects.controller.queue.world_waypoints],
        )
        _debug_print(
            f"  [PathProgress] consumed={progress.consumed} before_reissue "
            f"remaining={len(objects.controller.queue.world_waypoints)}"
        )
    if progress.collided:
        contact = _record_locked_target_collision(objects, stage, current_pos, current_yaw) if stage is not None else None
        objects.controller.clear()
        recovery = objects.collision_recovery.recover(client)
        print(f"  [Collision] before_path_reissue=True recovery_attempted={recovery.attempted}")
        if contact is not None:
            print(
                "  [TargetSurfaceContact] collision matched locked facade "
                f"range={contact['contact_range_m']:.2f}m memory_distance={contact['distance_m']:.2f}m"
            )
        state.update(collided=True, trajectory_queue=[])
        return progress.consumed

    # A newly planned return waypoint may intentionally be behind the old
    # heading. Rotate first so the generic stale-prefix filter does not delete
    # that valid waypoint as if it had already been passed.
    if _reorient_to_locked_target_if_behind(
        objects,
        client,
        path_stream,
        state,
        stage,
        allow_queued=True,
    ):
        current_pos, current_yaw = client.get_pose()
    dropped_behind = _drop_path_points_behind_vehicle(objects, current_pos, current_yaw)
    waypoints = objects.controller.queue.world_waypoints
    if not waypoints:
        return progress.consumed
    was_active = path_stream.active
    velocity = _continuous_path_velocity(objects, current_pos)
    issued = path_stream.sync(waypoints, current_pos, velocity)
    if issued:
        state.update(status="executing")
        mode = "extended" if was_active else "started"
        print(
            f"  [WorldPath] {mode} velocity={velocity:.2f}m/s "
            f"world_wp={_format_waypoints(waypoints, limit=20)}"
        )
        if not was_active:
            print(f"  [Flight] started remaining={len(waypoints)} velocity={velocity:.1f}m/s")
    return progress.consumed + dropped_behind
def _continuous_path_velocity(objects, current_pos) -> float:
    """Avoid near-hover speed while preserving some planner response time."""
    nominal = float(cfg.get("SIM", {}).get("AIRSIM_VELOCITY", 2.0))
    if not objects.controller.planning:
        return nominal
    waypoints = objects.controller.queue.world_waypoints
    if not waypoints:
        return nominal
    previous = [float(current_pos[0]), float(current_pos[1]), float(current_pos[2])]
    path_length = 0.0
    for waypoint in waypoints:
        point = [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])]
        path_length += math.sqrt(sum((point[i] - previous[i]) ** 2 for i in range(3)))
        previous = point
    reserve_time = max(float(objects.controller.reserve_time_s), 1e-6)
    fast_slow_cfg = function_section(cfg, "FAST_SLOW")
    minimum = max(0.0, float(fast_slow_cfg.get("CONTINUOUS_MIN_SPEED_MPS", 1.0)))
    minimum = min(minimum, nominal)
    return max(minimum, min(nominal, path_length / reserve_time))


class _CompletionRadiusWatchdog:
    """Keep the completion circle monitored while RPC/model calls block."""

    def __init__(self, objects, client, path_stream, stage, trigger_radius_m: float, interval_s: float):
        self.objects = objects
        self.client = client
        self.path_stream = path_stream
        self.stage = stage
        self.stage_key = CompletionPipeline.stage_key(stage)
        self.trigger_radius_m = float(trigger_radius_m)
        self.interval_s = max(0.02, float(interval_s))
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="completion-radius-watchdog",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=max(0.1, self.interval_s * 3.0))

    def _run(self) -> None:
        while not self._stop_event.wait(self.interval_s):
            try:
                current_stage = self.objects.task_manager.current_stage()
                if CompletionPipeline.stage_key(current_stage) != self.stage_key:
                    return
                controller = getattr(self.objects, "controller", None)
                queue = getattr(controller, "queue", None)
                if not (
                    self.path_stream.active
                    or bool(getattr(queue, "world_waypoints", []) or [])
                    or bool(getattr(controller, "planning", False))
                    or bool(getattr(controller, "has_plan_job", False))
                ):
                    continue
                current_world, _yaw = self.client.get_pose()
                estimated = _cached_target_distance(self.objects, self.stage, current_world)
                if not _should_trigger_completion_vlm(
                    self.objects,
                    self.stage,
                    estimated,
                    self.trigger_radius_m,
                ):
                    continue
                # This is deliberately the only action performed by the
                # watchdog.  Completion/VLM/memory decisions remain on the
                # runtime thread after it regains control.
                emergency_stop = getattr(self.path_stream, "emergency_stop", None)
                if callable(emergency_stop):
                    emergency_stop()
                else:
                    self.path_stream.stop()
                if controller is not None:
                    controller.clear()
                print("  [CompletionWatchdog] live pose entered target radius; path stopped")
                return
            except Exception:
                # AirSim can reject one concurrent pose RPC while an image is
                # being captured. The next tick retries without affecting the
                # navigation state.
                continue


def run_fast_slow_loop(
    state,
    initial_task: str,
    max_steps: int,
    client,
    capturer=None,
    *,
    isolated_planning_capture: bool = False,
) -> None:
    global _LAST_NAVIGATION_METRICS
    objects = _build_runtime_objects()
    objects.display_max_steps = int(max_steps)
    _LAST_NAVIGATION_METRICS = objects.navigation_metrics
    capture_mode = client.resolve_capture_mode()
    fast_slow_cfg = {**(cfg.get("FAST_SLOW", {}) or {}), **function_section(cfg, "FAST_SLOW")}
    stop_radius = float(fast_slow_cfg.get(
        "STOP_RADIUS",
        getattr(objects.completion_checker, "stop_depth", cfg.get("AGENT", {}).get("STOP_DEPTH_THRESHOLD", 8.0)),
    ))
    slow_radius = float(fast_slow_cfg.get("SLOW_RADIUS", max(stop_radius * 1.8, stop_radius + 5.0)))
    distance_trigger_radius = float(
        objects.distance_estimator.trigger_radius_m
        or getattr(objects.completion_checker, "stop_depth", stop_radius)
    )
    objects.completion_pipeline = CompletionPipeline(
        detector=objects.detector,
        checker=objects.completion_checker,
        client=client,
        capture_mode=capture_mode,
        detect_executor=objects.detect_executor,
        slow_executor=objects.slow_executor,
        # This pipeline only refreshes detector/depth observations. Completion
        # is exclusively decided from the cached world-pose distance below.
        stop_radius_m=-1.0,
        slow_radius_m=slow_radius,
        distance_view_policy=fast_slow_cfg.get("DISTANCE_VIEW_POLICY", "relation_aware"),
        debug_logs=bool(fast_slow_cfg.get("DEBUG_LOGS", False)),
    )
    path_stream = ContinuousPathStream(client, fast_slow_cfg)
    stream_poll_interval = float(fast_slow_cfg.get("PATH_POLL_INTERVAL_S", 0.05))
    print(
        f"[Runtime] continuous_path=True velocity={float(cfg.get('SIM', {}).get('AIRSIM_VELOCITY', 2.0)):.1f}m/s "
        f"stop_radius={stop_radius:.1f}m distance_trigger={distance_trigger_radius:.1f}m "
        f"completion=distance_triggered_memory_or_vlm"
    )

    task_text = initial_task.strip()
    if getattr(objects, "mission_memory", None) is not None:
        objects.mission_memory.reset(task_text)
        state.update(memory_summary=objects.mission_memory.summary(), memory_events=[])
    if getattr(objects, "obstacle_avoider", None) is not None:
        objects.obstacle_avoider.reset()
        state.update(obstacle_summary=objects.obstacle_avoider.summary())
    if task_text:
        print("[TASK PARSER] parsing task...")
        t0 = time.perf_counter()
        parsed = build_task_parser().parse(task_text)
        if parsed:
            objects.task_manager.start_with_stages(task_text, parsed)
            print(f"[TASK PARSER] parsed {len(parsed)} stages in {time.perf_counter() - t0:.2f}s")
            print(f"[TASK] {objects.task_manager.summary()}")
            _bootstrap_mission_memory(objects, client, state, task_text, capture_mode)
        else:
            objects.task_manager.start(task_text)

    last_stage_key = None
    distance_check_pending = False
    radius_watchdog: _CompletionRadiusWatchdog | None = None
    step = 0
    while step < max_steps:
        stage = objects.task_manager.current_stage()
        if stage is None:
            print("  [TASK] no active stage")
            break
        instruction = stage.instruction or task_text
        stage_key = (stage.index, stage.instruction)
        if stage_key != last_stage_key:
            if radius_watchdog is not None:
                radius_watchdog.stop()
                radius_watchdog = None
            path_stream.stop()
            objects.controller.clear()
            if objects.completion_pipeline is not None:
                objects.completion_pipeline.clear()
            objects.distance_estimator.clear()
            # 阶段刚切换时先检查一次memory/距离缓存；如果已经在目标附近，不再盲目起新规划。
            distance_check_pending = True
            state.update(
                trajectory_queue=[],
                qwen_waypoints=[],
                trajectory_candidates=[],
                selected_trajectory={},
                memory_summary=objects.mission_memory.summary(stage) if getattr(objects, "mission_memory", None) else {},
            )
            last_stage_key = stage_key
            objects.navigation_metrics.start_stage(CompletionPipeline.stage_key(stage))
            print(f"\n[Stage {stage.index + 1}/{len(objects.task_manager.stages)}] {instruction}")
            if is_view_relative_stage(stage):
                # Resolve the relative identity only after all preceding move/
                # turn stages have completed and this stage is truly active.
                with web_helpers.pause_background_capture(capturer):
                    _bind_view_relative_stage(objects, client, state, stage, task_text)

        # Future-target work is polled without waiting. MissionMemory remains
        # single-writer because completed observations are committed here.
        _poll_future_memory_scan(objects, state, stage)

        if stage.mode == "action":
            path_stream.stop()
            objects.controller.clear()
            _execute_action_stage(client, stage)
            objects.task_manager.complete_current("action executed")
            step += 1
            print(f"  [TASK] {objects.task_manager.summary()}")
            if objects.task_manager.is_done():
                state.update(status="done", task_done=True, step=0)
                break
            continue

        pos_now, yaw_now = client.get_pose()
        objects.navigation_metrics.record_pose(pos_now)
        if getattr(objects, "mission_memory", None) is not None:
            objects.mission_memory.record_pose(stage, pos_now, yaw_now)
            state.update(memory_summary=objects.mission_memory.summary(stage))
        stream_event = path_stream.poll(objects.controller.queue.world_waypoints, pos_now)
        if stream_event.consumed > 0:
            objects.controller.mark_executed(stream_event.consumed)
            distance_check_pending = True
            print(
                f"  [FlightPose] world=({pos_now[0]:.2f},{pos_now[1]:.2f},{pos_now[2]:.2f}) "
                f"yaw={yaw_now:.1f} consumed={stream_event.consumed} "
                f"remaining={len(objects.controller.queue.world_waypoints)}"
            )
            step += stream_event.consumed
            _debug_print(
                f"  [PathProgress] consumed={stream_event.consumed} "
                f"queue_after={len(objects.controller.queue.world_waypoints)} "
                f"pose=({pos_now[0]:.2f},{pos_now[1]:.2f},{pos_now[2]:.2f}) yaw={yaw_now:.1f}"
            )
            state.update(
                pose=pos_now,
                yaw=yaw_now,
                trajectory_queue=[list(wp) for wp in objects.controller.queue.world_waypoints],
            )
        if stream_event.collided:
            contact = _record_locked_target_collision(objects, stage, pos_now, yaw_now)
            objects.controller.clear()
            recovery = objects.collision_recovery.recover(client)
            print(f"  [Collision] new_path_collision=True recovery_attempted={recovery.attempted}")
            if contact is not None:
                print(
                    "  [TargetSurfaceContact] collision matched locked facade "
                    f"range={contact['contact_range_m']:.2f}m memory_distance={contact['distance_m']:.2f}m"
                )
            state.update(collided=True, trajectory_queue=[])
            continue

        if radius_watchdog is None:
            radius_watchdog = _CompletionRadiusWatchdog(
                objects,
                client,
                path_stream,
                stage,
                distance_trigger_radius,
                stream_poll_interval,
            )
            radius_watchdog.start()

        # memory/距离缓存很便宜；每轮都查一次，避免无人机已经到目标外圈但还沿旧队列冲进去。
        cached_distance = _cached_target_distance(objects, stage, pos_now)
        distance_check_pending = False
        current_stage_key = CompletionPipeline.stage_key(stage)
        retry_waiting, retry_remaining = _completion_retry_waiting(objects, stage)
        if retry_waiting:
            path_stream.stop()
            _debug_print(f"  [CompletionHold] retry in {retry_remaining:.2f}s")
            time.sleep(max(0.01, min(stream_poll_interval, retry_remaining)))
            continue
        # Check the live pose and the currently commanded queue before any
        # completion decision or new plan is issued.  A queued segment can
        # cross the arrival circle while RGB/depth or Qwen work is running.
        queue_guard = _truncate_queue_at_completion_radius(
            objects,
            path_stream,
            state,
            stage,
            pos_now,
            cached_distance,
            distance_trigger_radius,
        )
        if queue_guard == "arrived":
            # The trigger handler below performs the emergency stop before
            # beginning fresh completion evidence capture. Keeping this pass
            # together avoids issuing two consecutive AirSim stop RPCs.
            pass
        elif queue_guard == "truncated":
            # Let the next fast loop observe the stopped pose and enter the
            # normal completion trigger.  Do not immediately reissue the
            # shortened boundary waypoint in this same iteration.
            time.sleep(max(0.01, stream_poll_interval))
            continue
        trigger_vlm = _should_trigger_completion_vlm(
            objects,
            stage,
            cached_distance,
            distance_trigger_radius,
        )
        if queue_guard == "arrived" and (
            str(getattr(cached_distance, "source", "") or "") == "mission_memory"
            or bool(getattr(objects.distance_estimator, "use_for_completion", False))
        ):
            trigger_vlm = True
        if not trigger_vlm and _should_trigger_idle_memory_completion(
            objects,
            stage,
            cached_distance,
            distance_trigger_radius,
        ):
            trigger_vlm = True
        if trigger_vlm:
            should_break = _handle_distance_completion_trigger(
                objects,
                client,
                path_stream,
                state,
                stage,
                task_text,
                capture_mode,
                cached_distance,
                distance_trigger_radius,
            )
            if should_break:
                break
            continue

        if _invalidate_unsafe_locked_surface_queue(
            objects,
            path_stream,
            state,
            stage,
            pos_now,
            distance_trigger_radius,
        ):
            continue

        # A locked target from an earlier stage can legitimately be behind the
        # drone. Turn before capturing the next planner images so Qwen sees the
        # correct instance in front instead of inventing another forward path.
        with web_helpers.pause_background_capture(capturer):
            reoriented = _reorient_to_locked_target_if_behind(
                objects,
                client,
                path_stream,
                state,
                stage,
            )
        if reoriented:
            time.sleep(max(0.01, stream_poll_interval))
            continue

        frame = down_frame = None
        with web_helpers.pause_background_capture(capturer):
            state.update(step=step, status="running", error="")

            _poll_plan(objects, stage=stage, frame=frame, down_frame=down_frame, state=state)

            event = objects.completion_pipeline.poll() if objects.completion_pipeline is not None else None
            if event is not None:
                current_pipeline_key = CompletionPipeline.stage_key(stage)
                if event.stage_key != current_pipeline_key:
                    print(f"  [CompletionAsync] stale event={event.kind}; discarded")
                elif event.kind == "target_lost":
                    reason = (
                        getattr(event.bundle, "distance_reason", "")
                        if event.bundle is not None
                        else "target not detected"
                    )
                    should_break = _handle_background_target_lost(
                        objects,
                        client,
                        path_stream,
                        state,
                        stage,
                        capture_mode,
                        reason or "target not detected",
                    )
                    if should_break:
                        break
                    continue
                elif event.kind == "observation":
                    updated = _update_target_pose_from_bundle(objects, stage, event.bundle)
                elif event.kind in {"near_stop", "done_candidate", "not_done"}:
                    # Compatibility guard for stale jobs created before the
                    # distance-only mode was enabled. They may refresh the
                    # target pose but can never complete or reject a stage.
                    _update_target_pose_from_bundle(objects, stage, event.bundle)

            # Poll again because Qwen may have finished while detector results were processed.
            _poll_plan(objects, stage=stage, frame=frame, down_frame=down_frame, state=state)

            pos_for_plan, _yaw_for_plan = client.get_pose()
            velocity = float(cfg.get("SIM", {}).get("AIRSIM_VELOCITY", 2.0))
            queue_reaches_memory = _queue_reaches_memory_arrival(objects, stage, distance_trigger_radius)
            planning_decision = objects.controller.continuous_planning_decision(pos_for_plan, velocity)
            if queue_reaches_memory:
                planning_decision = SimpleNamespace(
                    submit=False,
                    queue_time_s=0.0,
                    after_next_time_s=0.0,
                    reason="queue_reaches_memory_arrival",
                )
                _debug_print("  [MemoryHold] pending queue already reaches memory arrival radius; skip Qwen append")
            if planning_decision.submit:
                _debug_print(
                    f"  [ContinuousSchedule] reason={planning_decision.reason} "
                    f"queue_time={planning_decision.queue_time_s:.2f}s "
                    f"after_next={planning_decision.after_next_time_s:.2f}s"
                )
                state.update(status="capturing")
                frame, down_frame, _submitted, memory_snapshot = _capture_and_submit_plan(
                    objects,
                    client,
                    stage,
                    instruction,
                    capture_mode,
                    isolated_capture=isolated_planning_capture,
                )
                _maybe_submit_future_memory_scan(
                    objects,
                    stage,
                    task_text,
                    memory_snapshot,
                )

                # Captures can take seconds while AirSim follows the old path.
                # Reconcile the new pose before that path is ever reissued.
                post_capture_pos, _post_capture_yaw = client.get_pose()
                post_capture_distance = _cached_target_distance(objects, stage, post_capture_pos)
                post_capture_guard = _truncate_queue_at_completion_radius(
                    objects,
                    path_stream,
                    state,
                    stage,
                    post_capture_pos,
                    post_capture_distance,
                    distance_trigger_radius,
                )
                if post_capture_guard == "truncated":
                    continue
                if post_capture_guard == "arrived" or _should_trigger_completion_vlm(
                    objects,
                    stage,
                    post_capture_distance,
                    distance_trigger_radius,
                ):
                    _clear_stopped_queue(objects, path_stream, state)
                    print("  [CompletionTrigger] target radius reached during capture; old path cancelled")
                    continue
                if _invalidate_unsafe_locked_surface_queue(
                    objects,
                    path_stream,
                    state,
                    stage,
                    post_capture_pos,
                    distance_trigger_radius,
                ):
                    continue

            # The planner result may have appended a path that crosses the
            # radius even when the vehicle did not move during capture. Apply
            # the same guard immediately before path sync so AirSim never
            # receives the post-target suffix.
            pre_sync_pos, _pre_sync_yaw = client.get_pose()
            pre_sync_distance = _cached_target_distance(objects, stage, pre_sync_pos)
            pre_sync_guard = _truncate_queue_at_completion_radius(
                objects,
                path_stream,
                state,
                stage,
                pre_sync_pos,
                pre_sync_distance,
                distance_trigger_radius,
            )
            if pre_sync_guard == "arrived":
                _clear_stopped_queue(objects, path_stream, state)
                continue
            if pre_sync_guard == "truncated":
                continue

            # Sync after Qwen submission so the bounded in-flight speed policy
            # can preserve planning reserve without ever degrading to a hover.
            synced_consumed = _sync_path_if_ready(objects, path_stream, client, state, stage=stage)
            step += synced_consumed
            if synced_consumed > 0:
                distance_check_pending = True

            if (
                objects.completion_pipeline is not None
                and objects.controller.should_check_completion()
                and not objects.completion_pipeline.active
            ):
                submitted = objects.completion_pipeline.submit(stage, task_text, None, None)
                if submitted:
                    _debug_print("  [TargetObservation] submitted detector+depth snapshot")

            _poll_plan(objects, stage=stage, frame=frame, down_frame=down_frame, state=state)

        if capturer is not None and hasattr(capturer, "resume"):
            capturer.resume()

        # Do not issue a plan that may have returned during a blocking capture
        # or detector call. The next loop first reconciles actual AirSim
        # progress against the old commanded path, then safely appends/reissues.
        time.sleep(max(0.01, stream_poll_interval))

    final_pos, _final_yaw = client.get_pose()
    objects.navigation_metrics.record_pose(final_pos)
    objects.navigation_metrics.print_summary(
        task_completed=objects.task_manager.is_done() and not objects.task_failed
    )
    if radius_watchdog is not None:
        radius_watchdog.stop()
    path_stream.stop()
    objects.controller.shutdown()
    objects.detect_executor.shutdown(wait=False, cancel_futures=True)
    objects.slow_executor.shutdown(wait=False, cancel_futures=True)
    objects.future_scan_executor.shutdown(wait=False, cancel_futures=True)
    objects.future_detect_executor.shutdown(wait=False, cancel_futures=True)


def print_last_navigation_summary(*, task_completed: bool = False) -> None:
    """Print the active run's metrics after an outer-loop interruption."""
    if _LAST_NAVIGATION_METRICS is not None:
        _LAST_NAVIGATION_METRICS.print_summary(task_completed=task_completed)


def run_fast_slow_web() -> None:
    script_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    sys.path.insert(0, script_dir)
    get_cfg(os.path.join(script_dir, "config", "default.yaml"))

    state = SharedState()
    app = create_app(state)
    web_port = cfg.get("WEB", {}).get("PORT", 5000)

    def run_web():
        import logging

        logging.getLogger("werkzeug").setLevel(logging.ERROR)
        cli = sys.modules.get("flask.cli")
        if cli:
            cli.show_server_banner = lambda *_, **__: None
        app.run(host="0.0.0.0", port=web_port, debug=False, use_reloader=False)

    threading.Thread(target=run_web, daemon=True).start()
    print("=" * 50)
    print(f"  Dashboard: http://localhost:{web_port}")
    print("  Config: config/default.yaml")
    print("  Runtime: fast_slow")
    print("=" * 50)

    if bool(cfg.get("SIM", {}).get("APPLY_SETTINGS_ON_WEB_START", True)):
        from sim.airsim_settings import write_local_airsim_settings

        settings_path = write_local_airsim_settings(make_backup=True)
        print(f"[AirSimSettings] wrote: {settings_path}")
        print("[AirSimSettings] start/restart AirSim now so the camera settings take effect")

    print("[AirSim] connecting...", flush=True)
    client = web_helpers.connect_web_airsim_client()
    client.warmup_capture()
    client.enable_api_control(True)
    client.arm(True)
    should_takeoff, start_pos, start_yaw, landed_text = web_helpers.should_takeoff(client)
    print(
        f"[AirSim] startup pose=({start_pos[0]:.1f}, {start_pos[1]:.1f}, {start_pos[2]:.1f}) "
        f"yaw={start_yaw:.1f}deg landed_state={landed_text}"
    )
    if should_takeoff:
        print("[AirSim] takeoff...")
        client.takeoff()
    else:
        print("[AirSim] skip takeoff: vehicle already airborne")

    capturer = FrameCapturer(state, interval=0.1)
    capturer.start()
    print("[FrameCapturer] background capture started")

    warmup_from_config()

    print("[AirSim] waiting for first frame...")
    while True:
        rgb, depth = capturer.get_latest_frame()
        if rgb is not None and depth is not None:
            print(f"[Ready] first frame ready (depth shape={depth.shape})")
            break
        time.sleep(0.1)

    current_task = ""
    while not current_task.strip():
        time.sleep(1)
        current_task = state.get_state().get("task", "").strip()

    try:
        while True:
            print(f"\n{'=' * 50}")
            print(f"  New task: {current_task}")
            print(f"{'=' * 50}")
            state.update(status="running", task_done=False)
            run_fast_slow_loop(
                state,
                initial_task=current_task,
                max_steps=int(cfg.get("EVAL", {}).get("MAX_STEPS", 100)),
                client=client,
                capturer=capturer,
            )
            state.update(status="waiting_task", task="", task_done=True)
            print("\n[TASK] task complete, waiting for next task...")
            current_task = ""
            while not current_task.strip():
                time.sleep(1)
                current_task = state.get_state().get("task", "").strip()
    except KeyboardInterrupt:
        print("\n[Exit] shutting down...")
        print_last_navigation_summary()
    finally:
        try:
            client.cleanup()
        except Exception:
            pass
