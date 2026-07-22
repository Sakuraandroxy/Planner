"""Fast-slow AirSim runtime built on decoupled agent functions."""

from __future__ import annotations

import io
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

from agent.functions.candidate.pipeline import prepare_candidates_for_world_model
from agent.functions.common.image_encoder import ImageEncoder
from agent.models.planner.sliding_window_planner import cumulative_to_incremental, incremental_to_cumulative
from agent.functions.common.config_access import function_section
from agent.functions.common.task_manager import TaskManager
from agent.functions.common.warmup import warmup_from_config
from agent.functions.common import web_runtime_helpers as web_helpers
from agent.functions.completion import build_task_completion_checker
from agent.functions.direction import build_direction_estimator
from agent.functions.fast_slow.completion_pipeline import CompletionPipeline
from agent.functions.fast_slow.controller import FastSlowController
from agent.functions.planning.sliding_window_planning import SlidingWindowPlanningFunction
from agent.functions.recovery.collision_recovery import CollisionRecovery
from agent.functions.relocalization import TargetRelocalizer
from agent.functions.task_parser import build_task_parser
from agent.models.detection import build_detector
from sim.frame_capturer import FrameCapturer
from web.app import create_app
from web.shared_state import SharedState


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
    completion_pipeline: CompletionPipeline | None = None


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
    )


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
    print(
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
    print(
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
    print(
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
        front_depth, down_depth, depth_timing = _capture_completion_depth(
            client,
            checker,
            capture_mode,
        )
        if front_depth is not None or down_depth is not None:
            print(
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
    print(
        f"  [ConfirmCapture] profile={profile} time={timing.get('total_s', elapsed):.2f}s  "
        f"front={web_helpers.shape_text(frame)} down={web_helpers.shape_text(down_frame)} "
        f"front_depth={web_helpers.shape_text(front_depth)} down_depth={web_helpers.shape_text(down_depth)}"
    )
    return frame, down_frame, front_depth, down_depth


def _confirm_completion_now(objects: RuntimeObjects, client, stage, task_text, capture_mode: str):
    """Pause-time final confirmation using fresh RGB and depth frames."""
    started = time.perf_counter()
    frame, down_frame, front_depth, down_depth = _capture_fresh_completion_frames(
        client,
        objects.completion_checker,
        capture_mode,
    )
    if frame is None:
        return None

    if getattr(objects.completion_checker, "uses_detector", False) and objects.completion_checker.is_detector_enabled():
        best_det, front_det, down_det, _detect_elapsed = _detect_dual_view(
            objects,
            stage,
            task_text,
            frame,
            down_frame,
        )
        print(
            "  [TargetDepth] "
            + web_helpers.target_depth_text("front", front_det, frame, front_depth)
            + "  "
            + web_helpers.target_depth_text("down", down_det, down_frame, down_depth)
        )
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
        )
    else:
        completion = objects.completion_checker.evaluate(
            stage,
            task_text,
            frame,
            down_frame,
            front_depth_meters=front_depth,
            down_depth_meters=down_depth,
        )
    completion.elapsed = time.perf_counter() - started
    return completion


def _submit_plan_if_needed(objects: RuntimeObjects, client, stage, instruction, frame, down_frame) -> bool:
    pos_now, yaw_now = client.get_pose()
    pending_for_model = objects.controller.queue.pending_for_model(
        pos_now,
        yaw_now,
        current_rot_body_to_world=None,
    )

    # 检测当前帧目标方向
    target = getattr(stage, "target", "") if stage else ""
    target_angle_deg = None
    if target and frame is not None:
        try:
            det = objects.detector.detect(frame, target)
            if det is not None and det.bbox is not None:
                import math
                bx1, by1, bx2, by2 = det.bbox
                cx = (bx1 + bx2) / 2.0
                fov = 90.0
                fx = frame.width / (2.0 * math.tan(math.radians(fov) / 2.0))
                angle = math.degrees(math.atan2((cx - frame.width / 2.0) / fx, 1.0))
                target_angle_deg = angle
        except Exception:
            pass

    inst = instruction
    if target_angle_deg is not None:
        side = "left" if target_angle_deg < 0 else "right"
        inst = f"{instruction} ({abs(target_angle_deg):.0f}° to the {side})"

    def plan_fn(pending_waypoints):
        return objects.planner.plan(
            frame,
            down_frame,
            inst,
            pending_waypoints=pending_waypoints,
            relation=getattr(stage, "relation", "") if stage else "",
            target=target,
        )

    submitted = objects.controller.maybe_submit_plan(
        current_pos=pos_now,
        current_yaw_deg=yaw_now,
        current_rot_body_to_world=None,
        plan_fn=plan_fn,
    )
    if submitted:
        pending = len(pending_for_model)
        print(
            f"  [SlowPlan] submitted pending={pending} "
            f"queue={len(objects.controller.queue.world_waypoints)}"
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
    chosen = all_candidates[0] if all_candidates else None
    if chosen is None:
        return result

    print("  [Candidate] candidates:")
    for i, cand in enumerate(all_candidates):
        src = getattr(cand, "source", "?")
        pre = float(getattr(cand, "pre_score", 0.0) or 0.0)
        wps = getattr(cand, "waypoints", [])
        wps_str = " ".join(f"[{wp[0]:.2f},{wp[1]:.2f},{wp[2]:.2f}]" for wp in (wps or [])[:3])
        print(f"    [{i}] source={src} pre={pre:.4f} waypoints={wps_str}...")
    print(f"  [Candidate] selected index=0 source={getattr(chosen, 'source', '?')} "
          f"pre={float(getattr(chosen, 'pre_score', 0.0) or 0.0):.4f}")

    chosen_cumulative = [list(wp) for wp in getattr(chosen, "waypoints", []) or []]
    chosen_incremental = cumulative_to_incremental(chosen_cumulative)
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
    transformed.selected_index = 0
    transformed.selected_candidate = chosen.to_dict()
    return transformed


def _poll_plan(objects: RuntimeObjects, stage=None, frame=None, down_frame=None, state=None) -> None:
    select_fn = lambda r: _select_and_transform_plan(objects, stage, r, frame, down_frame)
    result = objects.controller.poll_plan(select_fn=select_fn)
    if result is None:
        return
    qwen_inc = getattr(result, "waypoints", []) or []
    print(
        f"  [QwenSliding] raw={_format_waypoints(qwen_inc)}"
    )
    print(
        f"  [SlowPlan] appended={len(qwen_inc)} "
        f"{objects.controller.queue.summary()} queue={_format_waypoints(objects.controller.queue.world_waypoints)}"
    )
    if state is not None:
        state.update(
            qwen_waypoints=qwen_inc,
            trajectory_queue=[list(wp) for wp in objects.controller.queue.world_waypoints],
        )


def _execute_one_from_queue(objects: RuntimeObjects, client, state) -> bool:
    n = 1
    n = max(1, min(n, len(objects.controller.queue.world_waypoints)))
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


def run_fast_slow_loop(state, initial_task: str, max_steps: int, client, capturer=None) -> None:
    objects = _build_runtime_objects()
    capture_mode = client.resolve_capture_mode()
    fast_slow_cfg = {**(cfg.get("FAST_SLOW", {}) or {}), **function_section(cfg, "FAST_SLOW")}
    stop_radius = float(fast_slow_cfg.get(
        "STOP_RADIUS",
        getattr(objects.completion_checker, "stop_depth", cfg.get("AGENT", {}).get("STOP_DEPTH_THRESHOLD", 8.0)),
    ))
    slow_radius = float(fast_slow_cfg.get("SLOW_RADIUS", max(stop_radius * 1.8, stop_radius + 5.0)))
    objects.completion_pipeline = CompletionPipeline(
        detector=objects.detector,
        checker=objects.completion_checker,
        client=client,
        capture_mode=capture_mode,
        detect_executor=objects.detect_executor,
        slow_executor=objects.slow_executor,
        stop_radius_m=stop_radius,
        slow_radius_m=slow_radius,
        distance_view_policy=fast_slow_cfg.get("DISTANCE_VIEW_POLICY", "relation_aware"),
    )
    relocalization_enabled = bool(cfg.get("RELOCALIZATION", {}).get("ENABLED", False))
    print(
        f"[FastSlow] enabled={objects.controller.enabled} "
        f"low={objects.controller.low_watermark} exec={objects.controller.execute_count} "
        f"completion_interval={objects.controller.completion_interval_s:.1f}s "
        f"stop_radius={stop_radius:.1f}m"
    )
    print(
        f"[CompletionConfig] name={getattr(objects.completion_checker, 'name', type(objects.completion_checker).__name__)} "
        f"enabled={objects.completion_checker.enabled}"
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
    step = 0
    while step < max_steps:
        step += 1
        print(f"\n[Step {step}/{max_steps}]")
        stage = objects.task_manager.current_stage()
        if stage is None:
            print("  [TASK] no active stage")
            break
        instruction = stage.instruction or task_text
        print(
            f"  stage {stage.index + 1}/{len(objects.task_manager.stages)} "
            f"mode={stage.mode}: {instruction}"
        )

        stage_key = (stage.index, stage.instruction)
        if stage_key != last_stage_key:
            objects.controller.clear()
            if objects.completion_pipeline is not None:
                objects.completion_pipeline.clear()
            state.update(
                trajectory_queue=[],
                qwen_waypoints=[],
                trajectory_candidates=[],
                selected_trajectory={},
            )
            last_stage_key = stage_key

        if stage.mode == "action":
            objects.controller.clear()
            _execute_action_stage(client, stage)
            objects.task_manager.complete_current("action executed")
            print(f"  [TASK] {objects.task_manager.summary()}")
            if objects.task_manager.is_done():
                state.update(status="done", task_done=True, step=0)
                break
            continue

        with web_helpers.pause_background_capture(capturer):
            state.update(step=step, status="capturing", error="")
            frame, down_frame, front_depth, down_depth, _timing = _capture_rgb_for_stage(
                client,
                objects.completion_checker,
                stage,
                capture_mode,
            )
            if frame is None:
                print("  [Capture] no frame, retry")
                time.sleep(0.1)
                continue
            ImageEncoder.encode_front(frame)
            if down_frame is not None:
                ImageEncoder.encode_down(down_frame)
            _publish_frames(state, client, frame, down_frame, None)

            _poll_plan(objects, stage=stage, frame=frame, down_frame=down_frame, state=state)

            event = objects.completion_pipeline.poll() if objects.completion_pipeline is not None else None
            if event is not None:
                current_pipeline_key = CompletionPipeline.stage_key(stage)
                if event.stage_key != current_pipeline_key:
                    print(f"  [CompletionAsync] stale event={event.kind}; discarded")
                elif event.kind in {"near_stop", "done_candidate"}:
                    distance_text = ""
                    if event.bundle is not None and event.bundle.distance_m is not None:
                        distance_text = f" distance={event.bundle.distance_m:.1f}m"
                    print(
                        f"  [CompletionAsync] event={event.kind}{distance_text} "
                        f"elapsed={event.elapsed:.2f}s -> pause and confirm"
                    )
                    completion = _confirm_completion_now(objects, client, stage, task_text, capture_mode)
                    if completion is not None:
                        print(
                            f"  [ConfirmCompletion] detected={completion.target_detected} "
                            f"accepted={completion.accepted_view} done={completion.done} "
                            f"time={completion.elapsed:.2f}s"
                        )
                        if completion.done:
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
                            if objects.task_manager.is_done():
                                state.update(status="done", task_done=True, step=0)
                                break
                            continue
                        objects.controller.queue.clear()
                        objects.controller.discard_plan()
                        state.update(
                            trajectory_queue=[],
                            qwen_waypoints=[],
                        )
                        print("  [ConfirmCompletion] done=False -> discard stale plan and replan")
                        if (
                            completion.checked
                            and completion.target_detected is False
                            and getattr(stage, "allow_relocalize", False)
                            and relocalization_enabled
                        ):
                            relocalized = objects.relocalizer.search(
                                client,
                                stage,
                                front_image=frame,
                                down_image=down_frame,
                                capture_mode=capture_mode,
                                skip_initial_frame=True,
                            )
                            print(
                                f"  [Relocalize] found={relocalized.found} "
                                f"time={relocalized.elapsed:.2f}s reason={relocalized.reason}"
                            )
                            state.update(status="relocalized" if relocalized.found else "target_not_found", step=step)
                            continue
                        continue
                elif event.kind == "not_done":
                    completion = event.completion
                    print(
                        f"  [CompletionAsync] done=False detected={getattr(completion, 'target_detected', None)} "
                        f"accepted={getattr(completion, 'accepted_view', 'none')} elapsed={event.elapsed:.2f}s"
                    )

            # Poll again — confirm/evaluate above takes ~5s, Qwen may have finished in that time.
            _poll_plan(objects, stage=stage, frame=frame, down_frame=down_frame, state=state)

            _submit_plan_if_needed(objects, client, stage, instruction, frame, down_frame)

            if (
                objects.completion_pipeline is not None
                and objects.controller.should_check_completion()
                and not objects.completion_pipeline.active
            ):
                submitted = objects.completion_pipeline.submit(stage, task_text, frame, down_frame)
                if submitted:
                    print("  [CompletionAsync] submitted detect+depth/VLM pipeline")

            _poll_plan(objects, stage=stage, frame=frame, down_frame=down_frame, state=state)

        if capturer is not None and hasattr(capturer, "resume"):
            capturer.resume()

        executed = _execute_one_from_queue(objects, client, state)
        if not executed:
            if objects.controller.planning:
                print("  [FastSlow] queue empty; waiting for background Qwen")
                time.sleep(0.2)
                _poll_plan(objects, stage=stage, frame=frame, down_frame=down_frame, state=state)
            else:
                print("  [FastSlow] queue empty and no plan available")
                time.sleep(0.2)

    objects.controller.shutdown()
    objects.detect_executor.shutdown(wait=False, cancel_futures=True)
    objects.slow_executor.shutdown(wait=False, cancel_futures=True)


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
    finally:
        try:
            client.cleanup()
        except Exception:
            pass
