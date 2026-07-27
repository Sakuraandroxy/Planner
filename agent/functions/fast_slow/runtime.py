"""Fast-slow AirSim runtime built on decoupled agent functions."""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any
from types import SimpleNamespace
from concurrent.futures import ThreadPoolExecutor

from config import cfg, get_cfg

from agent.functions.candidate.pipeline import prepare_candidates_for_world_model, select_best_candidate
from agent.models.planner.sliding_window_planner import cumulative_to_incremental, incremental_to_cumulative
from agent.functions.common.config_access import function_section
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
from agent.functions.planning.direction_hint import direction_hint_from_front_detection
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
    distance_estimator: Any
    arrival_completion: Any
    navigation_metrics: NavigationMetricsTracker
    world_model: Any = None
    completion_pipeline: CompletionPipeline | None = None
    display_step: int = 0
    display_max_steps: int = 0
    task_failed: bool = False


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
        distance_estimator=build_distance_estimator(),
        arrival_completion=arrival_completion,
        navigation_metrics=NavigationMetricsTracker(arrival_completion.arrival_radius_m),
        world_model=build_world_model(),
    )


def _update_target_pose_from_bundle(objects: RuntimeObjects, stage, bundle: DetectionDepthBundle | None):
    if bundle is None or not objects.distance_estimator.enabled:
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
    return (
        getattr(stage, "target_query", None)
        or getattr(stage, "target", None)
        or getattr(stage, "instruction", None)
        or fallback_instruction
        or ""
    )


def _is_above_stage(stage: Any) -> bool:
    relation = str(getattr(stage, "relation", "") or "").strip().lower()
    instruction = str(getattr(stage, "instruction", "") or "").strip().lower()
    return (
        relation in {"above", "over", "on top", "on top of"}
        or "above" in instruction
        or "on top" in instruction
    )


def _detection_reliability(stage: Any, detection: Any, image: Any = None) -> float:
    if not detection or not getattr(detection, "visible", False):
        return 0.0
    score = max(0.0, min(1.0, float(getattr(detection, "score", 0.0) or 0.0)))
    if image is None or not getattr(detection, "bbox", None) or not hasattr(image, "size"):
        return score

    width, height = float(image.size[0]), float(image.size[1])
    if width <= 1.0 or height <= 1.0:
        return score

    x1, y1, x2, y2 = [float(v) for v in detection.bbox[:4]]
    box_w = max(0.0, min(width, x2) - max(0.0, x1))
    box_h = max(0.0, min(height, y2) - max(0.0, y1))
    area_ratio = (box_w * box_h) / max(width * height, 1.0)
    span_x = box_w / width
    span_y = box_h / height
    touches_border = x1 <= 2.0 or y1 <= 2.0 or x2 >= width - 2.0 or y2 >= height - 2.0

    quality = 1.0
    if area_ratio <= 0.0002:
        quality *= 0.35
    elif area_ratio <= 0.001:
        quality *= 0.65
    if span_x >= 0.90 or span_y >= 0.90:
        return 0.0
    if area_ratio >= 0.55:
        quality *= 0.03
    elif area_ratio >= 0.30 or span_x >= 0.75 or span_y >= 0.75:
        quality *= 0.20
    elif area_ratio >= 0.18:
        quality *= 0.45
    if touches_border:
        quality *= 0.45 if _is_above_stage(stage) else 0.35
    return score * quality


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


def _detect_dual_view(objects: RuntimeObjects, stage, task_text, frame, down_frame):
    from agent.models.detection.base import DetectionResult

    caption = _caption_for_stage(stage, task_text)

    def detect_one(image, camera_name: str):
        if image is None:
            return DetectionResult(visible=False, camera=camera_name)
        return objects.detector.detect(image, caption, depth_meters=None, camera_name=camera_name)

    t0 = time.perf_counter()
    front_future = objects.detect_executor.submit(detect_one, frame, "front")
    down_future = objects.detect_executor.submit(detect_one, down_frame, "down")
    front_det = front_future.result()
    down_det = down_future.result()
    _suppress_unreliable_detection(stage, front_det, frame)
    _suppress_unreliable_detection(stage, down_det, down_frame)
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
    return best, front_det, down_det, elapsed


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
        best_det, front_det, down_det, detect_elapsed = _detect_dual_view(
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
        best_det, front_det, down_det, detect_elapsed = _detect_dual_view(
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
        getattr(stage, "target_query", None)
        or getattr(stage, "target", None)
        or getattr(stage, "instruction", None)
        or instruction,
    )
    direction_text = direction_hint.text
    print(
        f"  [PlanInput] instruction={inst!r} pending={len(pending_for_model)} "
        f"pending_wp={_format_waypoints(pending_for_model)} "
        f"direction={direction_text!r} direction_reason={direction_hint.reason} "
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
        )
        output._selection_front_frame = frame
        output._selection_down_frame = down_frame
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
    selection = prepare_candidates_for_world_model(
        prepared,
        detection=None,
        direction=getattr(stage, "instruction", "") if stage else "",
        stop_threshold=float(cfg.get("AGENT", {}).get("STOP_DEPTH_THRESHOLD", 8.0)),
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
    if isolated_capture and planning_capture_mode == "batch":
        profile = str(planning_cfg.get("CAPTURE_PROFILE", "front_down") or "front_down")
        (
            frame,
            down_frame,
            _front_depth,
            _down_depth,
            _timing,
            capture_pos,
            capture_yaw,
        ) = web_helpers.capture_profile_isolated_with_pose(client, profile)
        if hasattr(client, "get_pose_full"):
            capture_pos, capture_yaw, capture_rot = client.get_pose_full()
        else:
            print("  [Capture] full body rotation unavailable for Qwen plan, retry later")
            return None, None, False
    elif planning_capture_mode == "batch" and hasattr(client, "capture_planning_views_with_pose"):
        camera_offset = planning_cfg.get("FRONT_CAMERA_OFFSET", [1.0, 0.0, 0.0])
        frame, down_frame, capture_pos, capture_yaw, capture_rot, _timing = (
            client.capture_planning_views_with_pose(camera_offset=camera_offset)
        )
    else:
        frame, down_frame, _front_depth, _down_depth, _timing = _capture_rgb_for_stage(
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
    if frame is None:
        print("  [Capture] no frame for Qwen plan, retry later")
        return None, None, False
    if capture_rot is None:
        print("  [Capture] full body rotation unavailable for Qwen plan, retry later")
        return None, None, False
    submitted = _submit_plan_if_needed(
        objects,
        client,
        stage,
        instruction,
        frame,
        down_frame,
        plan_pos=capture_pos,
        plan_yaw=capture_yaw,
        plan_rot=capture_rot,
    )
    return frame, down_frame, submitted


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
    objects.task_manager.complete_current(completion.reason)
    print(f"  [TASK] {objects.task_manager.summary()}")
    task_done = objects.task_manager.is_done()
    if task_done:
        objects.navigation_metrics.task_completed = True
        state.update(status="done", task_done=True, step=0)
    return task_done


def _clear_stopped_queue(objects, path_stream, state) -> None:
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
    objects.task_manager.complete_current(reason)
    print(f"  [Completion] done=False source=relocalization_failed reason={reason}")
    print(f"  [TASK] {objects.task_manager.summary()}")
    if objects.task_manager.is_done():
        state.update(status="failed", task_done=False, step=0)
        return True
    return False


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

    relocalized = objects.relocalizer.search(
        client,
        stage,
        capture_mode=capture_mode,
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
    _clear_stopped_queue(objects, path_stream, state)
    print(
        f"\n  [CompletionTrigger] distance={cached_distance.distance_m:.2f}m "
        f"<= {float(trigger_radius_m):.2f}m; stopped, cleared queue, checking fresh evidence"
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
            "fresh RGB capture failed",
        )

    best_det, front_det, down_det, detect_elapsed = _detect_dual_view(
        objects,
        stage,
        task_text,
        frame,
        down_frame,
    )
    if best_det is None or not getattr(best_det, "visible", False):
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
        relocalized = objects.relocalizer.search(
            client,
            stage,
            capture_mode=capture_mode,
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
        best_det, front_det, down_det, detect_elapsed = _detect_dual_view(
            objects,
            stage,
            task_text,
            frame,
            down_frame,
        )
        if best_det is None or not getattr(best_det, "visible", False):
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
        estimated_distance_m=cached_distance.distance_m,
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
            cached_distance.distance_m,
        )

    objects.distance_estimator.clear()
    objects.navigation_metrics.invalidate_target(current_stage_key)
    print("  [CompletionVLM] not complete; replanning with empty queue")
    return False


def _cached_target_distance(objects, stage, current_world):
    """Read the cheap cached target distance independently of waypoint events."""
    estimated = objects.distance_estimator.estimate_distance(
        stage_key=CompletionPipeline.stage_key(stage),
        current_world=current_world,
    )
    if estimated is None:
        return None
    objects.navigation_metrics.record_distance(estimated.distance_m)
    return estimated


def _should_trigger_completion_vlm(objects, stage_key, estimated, trigger_radius_m: float) -> bool:
    if estimated is None or not objects.distance_estimator.use_for_completion:
        return False
    return bool(estimated.distance_m <= float(trigger_radius_m))


def _completion_needs_relocalization(completion) -> bool:
    """Relocalize only when fresh dual-view completion evidence lost the target."""
    return bool(completion is None or getattr(completion, "target_detected", None) is not True)


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
        objects.controller.clear()
        recovery = objects.collision_recovery.recover(client)
        print(f"  [Collision] before_path_reissue=True recovery_attempted={recovery.attempted}")
        state.update(collided=True, trajectory_queue=[])
        return progress.consumed

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
    """Keep enough active-path flight time for an in-flight Qwen request."""
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
    return min(nominal, path_length / reserve_time)


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
        f"completion=distance_triggered_vlm"
    )

    task_text = initial_task.strip()
    if task_text:
        print("[TASK PARSER] parsing task...")
        t0 = time.perf_counter()
        parsed = build_task_parser().parse(task_text)
        if parsed:
            objects.task_manager.start_with_stages(task_text, parsed)
            print(f"[TASK PARSER] parsed {len(parsed)} stages in {time.perf_counter() - t0:.2f}s")
            print(f"[TASK] {objects.task_manager.summary()}")
        else:
            objects.task_manager.start(task_text)

    last_stage_key = None
    distance_check_pending = False
    step = 0
    while step < max_steps:
        stage = objects.task_manager.current_stage()
        if stage is None:
            print("  [TASK] no active stage")
            break
        instruction = stage.instruction or task_text
        stage_key = (stage.index, stage.instruction)
        if stage_key != last_stage_key:
            path_stream.stop()
            objects.controller.clear()
            if objects.completion_pipeline is not None:
                objects.completion_pipeline.clear()
            objects.distance_estimator.clear()
            distance_check_pending = False
            state.update(
                trajectory_queue=[],
                qwen_waypoints=[],
                trajectory_candidates=[],
                selected_trajectory={},
            )
            last_stage_key = stage_key
            objects.navigation_metrics.start_stage(CompletionPipeline.stage_key(stage))
            print(f"\n[Stage {stage.index + 1}/{len(objects.task_manager.stages)}] {instruction}")

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
            objects.controller.clear()
            recovery = objects.collision_recovery.recover(client)
            print(f"  [Collision] new_path_collision=True recovery_attempted={recovery.attempted}")
            state.update(collided=True, trajectory_queue=[])
            continue

        # Background observations update the target world pose. Distance is
        # evaluated only when waypoint progress is consumed.
        cached_distance = None
        if distance_check_pending:
            cached_distance = _cached_target_distance(objects, stage, pos_now)
            distance_check_pending = False
        current_stage_key = CompletionPipeline.stage_key(stage)
        trigger_vlm = _should_trigger_completion_vlm(
            objects,
            current_stage_key,
            cached_distance,
            distance_trigger_radius,
        )
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
            planning_decision = objects.controller.continuous_planning_decision(pos_for_plan, velocity)
            if planning_decision.submit:
                _debug_print(
                    f"  [ContinuousSchedule] reason={planning_decision.reason} "
                    f"queue_time={planning_decision.queue_time_s:.2f}s "
                    f"after_next={planning_decision.after_next_time_s:.2f}s"
                )
                state.update(status="capturing")
                frame, down_frame, _submitted = _capture_and_submit_plan(
                    objects,
                    client,
                    stage,
                    instruction,
                    capture_mode,
                    isolated_capture=isolated_planning_capture,
                )

            # Sync path AFTER Qwen submission so _continuous_path_velocity
            # sees planning=True and slows down immediately.  Previously we
            # synced first (planning=False → 2.0 m/s) then submitted Qwen,
            # so the drone burned path-prefix capacity at full speed for one
            # whole iteration before the velocity drop took effect.
            synced_consumed = _sync_path_if_ready(objects, path_stream, client, state)
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
    path_stream.stop()
    objects.controller.shutdown()
    objects.detect_executor.shutdown(wait=False, cancel_futures=True)
    objects.slow_executor.shutdown(wait=False, cancel_futures=True)


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
