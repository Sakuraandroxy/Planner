"""Fast-slow AirSim runtime built on decoupled agent functions."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any
from types import SimpleNamespace
from concurrent.futures import Future

from config import cfg

from agent.functions.candidate.pipeline import prepare_candidates_for_world_model, select_best_candidate
from agent.models.planner.sliding_window_planner import cumulative_to_incremental, incremental_to_cumulative
from agent.functions.common.config_access import function_section
from agent.functions.common.detection_policy import is_large_structure_stage
from agent.functions.common import web_runtime_helpers as web_helpers
from agent.functions.completion import (
    CompletionResult,
    NavigationMetricsTracker,
)
from agent.functions.debug import TargetSnapshotRecorder
from agent.functions.fast_slow.completion_pipeline import CompletionPipeline
from agent.functions.fast_slow.completion_pipeline import DetectionDepthBundle
from agent.functions.fast_slow.path_stream import ContinuousPathStream
from agent.functions.fast_slow.runtime_context import (
    RuntimeObjects,
    build_runtime_objects as _build_runtime_objects,
)
from agent.functions.fast_slow.above_runtime import (
    AboveStageRuntimeState,
    _above_max_allowed_world_z,
    _above_pre_roof_facade_clearance_context,
    _above_queue_violation_reason,
    _above_roof_candidate_ready,
    _above_stage_state,
    _apply_above_altitude_path_guard,
    _apply_above_roof_acquisition_path_guard,
    _apply_airsim_vertical_path_quantization,
    _cumulative_path_length,
    _apply_small_above_target_guidance,
    _handle_small_above_down_center_observation,
    _small_above_confirmation_pending,
    _is_above_stage,
    _locked_large_above_requires_roof,
    _small_above_target,
    _minimum_airsim_climb_command_m,
    _minimum_airsim_vertical_command_m,
    _normalize_direct_vertical_action_m,
    _quantized_climb_body_z,
    _record_synchronized_roof_plane,
    _straight_body_path,
)
from agent.functions.fast_slow.target_runtime import (
    _active_relocalization_session_id,
    _bearing_only_active,
    _bump_lock_generation,
    _bump_stage_generation,
    _detection_matches_view_relative_sector,
    _generation_map,
    _identity_approved_detections,
    _locked_instance_id,
    _lock_generation,
    _memory_estimate_is_trustworthy,
    _prebind_target_from_bundle,
    _primary_update_event,
    _record_locked_target_snapshot,
    _record_target_bearing,
    _select_identity_approved_down_detection,
    _select_identity_approved_front_detection,
    _select_prebind_front_detection,
    _stage_generation,
    _target_lost_event_is_stale,
    _target_lost_recovery,
)
from agent.functions.fast_slow.relocalization_runtime import (
    _build_locked_relocalization_validator,
    _bundle_from_relocalization_result,
    _large_target_expected_offscreen,
    _relocalization_memory_is_reliable,
)
from agent.functions.fast_slow.planning_runtime import (
    _apply_memory_path_guard,
    _apply_obstacle_path_guard,
    _candidate_trace_text,
    _clamp,
    _clip_path_at_target_radius,
    _closest_path_distance_to_target,
    _direct_memory_waypoint,
    _distance3_body,
    _format_waypoints,
    _guard_distance,
    _limit_cumulative_path_length,
    _memory_near_approach_radius,
    _memory_near_approach_radius_from_values,
    _memory_near_standoff_radius,
    _memory_path_clip_radius,
    _norm3,
    _planning_geometry_context,
    _project_xy,
    _segment_circle_entry_xy,
    _segment_point_distance,
    _segment_point_distance_xy,
    _segment_sphere_entry,
)
from agent.functions.fast_slow.completion_runtime import (
    _attach_depth_to_detection_lists,
    _best_detection_from_list,
    _capture_completion_depth,
    _capture_fresh_completion_frames,
    _capture_fresh_rgb_frames,
    _capture_rgb_for_stage,
    _caption_for_stage,
    _detect_dual_view,
    _detection_reliability,
    _estimated_pose_xy_distance,
    _evaluate_completion,
    _evaluate_completion_fast_slow,
    _publish_frames,
    _select_reliable_detection,
    _suppress_unreliable_detection,
    _update_memory_from_fresh_detection,
    _wait_completion_depth,
)
from agent.functions.fast_slow.execution_runtime import (
    CompletionRadiusWatchdog as _ExecutionCompletionRadiusWatchdog,
    _clear_stopped_queue,
    _continuous_path_velocity,
    _drop_path_points_behind_vehicle,
    _execute_action_stage,
    _execute_one_from_queue,
    _needs_planning_snapshot,
    _sync_path_if_ready as _execution_sync_path_if_ready,
)
from agent.functions.fast_slow.guidance_runtime import (
    apply_bearing_path_guard as _guidance_apply_bearing_path_guard,
    apply_locked_target_direction_guard as _apply_locked_target_direction_guard,
)
from agent.functions.fast_slow.web_runtime import run_fast_slow_web
from agent.functions.memory import (
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
from agent.functions.memory.mission_memory import stage_key as _mission_stage_key
from agent.functions.perception import (
    detection_is_excluded_by_bearing,
    estimate_down_roof_plane,
    metric_depth_usable,
)
from agent.functions.planning.direction_hint import (
    direction_hint_from_front_detection,
    direction_hint_from_locked_body_target,
    direction_hint_from_angle,
)
from agent.functions.task_parser import build_task_parser


_LAST_NAVIGATION_METRICS: NavigationMetricsTracker | None = None


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


def _runtime_stage_key(stage) -> tuple:
    return CompletionPipeline.stage_key(stage)




def _apply_bearing_path_guard(objects, stage, cumulative_waypoints, *, selection_pos, selection_yaw):
    return _guidance_apply_bearing_path_guard(
        objects,
        stage,
        cumulative_waypoints,
        selection_pos=selection_pos,
        selection_yaw=selection_yaw,
    )


def _update_target_pose_from_bundle(objects: RuntimeObjects, stage, bundle: DetectionDepthBundle | None):
    if bundle is None:
        return None
    memory = getattr(objects, "mission_memory", None)
    above_state = None
    if _is_above_stage(stage) and bundle.observer_world is not None:
        above_state = _above_stage_state(objects, stage, bundle.observer_world)
    previous_lock = (
        str(memory.stage_locks.get(_mission_stage_key(stage), "") or "")
        if memory is not None
        else ""
    )
    previous_primary = memory.primary_instance(stage) if memory is not None else None
    previous_anchor = list(previous_primary.target_world) if previous_primary is not None else None
    approved_front_all = _identity_approved_detections(objects, stage, bundle, "front")
    approved_down_all = _identity_approved_detections(objects, stage, bundle, "down")
    approved_front = _select_identity_approved_front_detection(objects, stage, bundle)
    approved_bearing = approved_front or _select_identity_approved_down_detection(objects, stage, bundle)
    bearing_observation = _record_target_bearing(objects, stage, bundle, approved_bearing)
    if (
        getattr(objects, "mission_memory", None) is not None
        and bundle.observer_world is not None
        and bundle.observer_yaw_deg is not None
    ):
        events = objects.mission_memory.update_from_detections(
            stage=stage,
            detections_by_view={
                "front": approved_front_all,
                "down": approved_down_all,
            },
            images_by_view={"front": bundle.front_image, "down": bundle.down_image},
            observer_world=bundle.observer_world,
            observer_yaw_deg=bundle.observer_yaw_deg,
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
                f"  [Memory] updates={len(events)} "
                f"primary={(primary.instance_id if primary else 'none')}"
                f"{fallback_text}"
            )
        post_update_lock = str(
            objects.mission_memory.stage_locks.get(_mission_stage_key(stage), "") or ""
        )
        if post_update_lock != previous_lock:
            _bump_lock_generation(objects, stage)
        if not previous_lock and post_update_lock:
            # The pre-lock frame may contain several same-class candidates.
            # Re-run the gate now that the selected ordinal/instance is known,
            # so the bearing tracker cannot retain a different candidate from
            # that same activation frame.
            tracker = getattr(objects, "target_bearing_tracker", None)
            if tracker is not None:
                tracker.clear(
                    _runtime_stage_key(stage),
                    preserve_activation_guard=True,
                )
            lock_event = _primary_update_event(objects.mission_memory, stage, events)
            approved_front = getattr(lock_event, "detection", None)
            bearing_observation = _record_target_bearing(objects, stage, bundle, approved_bearing)
            activation = (
                tracker.activation_guard(_runtime_stage_key(stage))
                if tracker is not None and hasattr(tracker, "activation_guard")
                else None
            )
            if activation is not None and bearing_observation is not None:
                correction = abs(
                    (bearing_observation.world_bearing_deg - activation.world_bearing_deg + 180.0)
                    % 360.0
                    - 180.0
                )
                if correction >= float(
                    objects.mission_memory.config.get("PREBIND_CORRECTION_LOG_DEG", 5.0)
                ):
                    print(
                        "  [TargetPrebindCorrection] "
                        f"rgb_bearing={activation.world_bearing_deg:+.1f}deg "
                        f"locked_bearing={bearing_observation.world_bearing_deg:+.1f}deg "
                        f"delta={correction:.1f}deg instance={post_update_lock}"
                    )
        _record_locked_target_snapshot(
            objects,
            stage,
            bundle,
            source="active_observation",
            events=events,
        )
        if approved_front_all or approved_down_all:
            tracker = getattr(objects, "target_bearing_tracker", None)
            if tracker is not None and hasattr(tracker, "reset_lost"):
                tracker.reset_lost(_runtime_stage_key(stage))
            _target_lost_recovery(objects).mark_observed(_runtime_stage_key(stage))
        if above_state is not None:
            current_primary = objects.mission_memory.primary_instance(stage)
            current_lock = str(
                objects.mission_memory.stage_locks.get(_mission_stage_key(stage), "") or ""
            )
            current_anchor = list(current_primary.target_world) if current_primary is not None else None
            if not previous_lock and current_lock:
                above_state.pending_queue_review = "lock_acquired"
            elif (
                current_lock
                and current_lock == previous_lock
                and previous_anchor is not None
                and current_anchor is not None
                and math.hypot(
                    float(current_anchor[0]) - float(previous_anchor[0]),
                    float(current_anchor[1]) - float(previous_anchor[1]),
                )
                > float(objects.mission_memory.config.get("ABOVE_ANCHOR_REPLAN_SHIFT_M", 3.0))
            ):
                above_state.pending_queue_review = "anchor_shift"
            above_state.last_lock_id = current_lock
            above_state.last_anchor_world = current_anchor
            facade_clearance = _above_pre_roof_facade_clearance_context(
                objects,
                stage,
                bundle.observer_world,
            )
            if facade_clearance:
                clearance_z = float(facade_clearance["target_world_z"])
                prior_clearance_z = above_state.last_facade_clearance_z
                replan_shift = max(
                    0.1,
                    float(objects.mission_memory.config.get("ABOVE_FACADE_TOP_REPLAN_SHIFT_M", 0.75)),
                )
                climb_tolerance = max(
                    0.05,
                    float(objects.mission_memory.config.get("ABOVE_PRE_ROOF_CLIMB_TOLERANCE_M", 0.4)),
                )
                if (
                    bundle.observer_world[2] > clearance_z + climb_tolerance
                    and (
                        prior_clearance_z is None
                        or clearance_z < float(prior_clearance_z) - replan_shift
                    )
                ):
                    above_state.pending_queue_review = (
                        f"{above_state.pending_queue_review}+facade_top_rise"
                        if above_state.pending_queue_review
                        else "facade_top_rise"
                    )
                above_state.last_facade_clearance_z = (
                    clearance_z
                    if prior_clearance_z is None
                    else min(float(prior_clearance_z), clearance_z)
                )
        if is_view_relative_stage(stage) and not objects.mission_memory.has_primary(stage):
            # A detector hit is not a navigation identity until the requested
            # activation-view ordinal has been resolved and locked.
            objects.distance_estimator.clear()
            if bearing_observation is None:
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
        # Do not fall back to the raw detector result here.  When the
        # identity gate rejects every front candidate, the remaining result
        # may be the previous building's wall and must not become a new
        # metric target anchor.  The RGB bearing tracker has already retained
        # any approved bearing-only observation above.
        detection, image = approved_front, bundle.front_image
        if detection is None:
            approved_down = _select_identity_approved_down_detection(objects, stage, bundle)
            if _is_above_stage(stage) and approved_down is not None:
                detection, image, view = approved_down, bundle.down_image, "down"
            else:
                objects.distance_estimator.clear()
                if bearing_observation is None:
                    objects.navigation_metrics.invalidate_target(CompletionPipeline.stage_key(stage))
                _debug_print("  [DistanceEstimate] cleared: front detections rejected by target identity gate")
                return None
    elif view == "down":
        detection, image = _select_identity_approved_down_detection(objects, stage, bundle), bundle.down_image
        if detection is None:
            objects.distance_estimator.clear()
            if bearing_observation is None:
                objects.navigation_metrics.invalidate_target(CompletionPipeline.stage_key(stage))
            _debug_print("  [DistanceEstimate] cleared: down detections rejected by target identity gate")
            return None
    else:
        objects.distance_estimator.clear()
        if bearing_observation is None:
            objects.navigation_metrics.invalidate_target(CompletionPipeline.stage_key(stage))
        _debug_print("  [DistanceEstimate] cleared: target not detected")
        return None
    if bundle.observer_world is None or bundle.observer_yaw_deg is None:
        return None
    memory_config = dict(getattr(getattr(objects, "mission_memory", None), "config", {}) or {})
    estimator_config = dict(getattr(getattr(objects, "distance_estimator", None), "config", {}) or {})
    metric_config = {**estimator_config, **memory_config}
    metric_config.setdefault(
        "METRIC_LOCK_MAX_DEPTH_M",
        estimator_config.get("MAX_RELIABLE_DEPTH_M", estimator_config.get("MAX_DEPTH_M", 120.0)),
    )
    if not metric_depth_usable(detection, metric_config)[0]:
        objects.distance_estimator.clear()
        if bearing_observation is None:
            objects.navigation_metrics.invalidate_target(CompletionPipeline.stage_key(stage))
        _debug_print("  [DistanceEstimate] cleared: target depth failed quality gate")
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
        if bearing_observation is None:
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


def _review_pending_above_queue(objects, path_stream, state, stage, current_world) -> bool:
    if not _is_above_stage(stage):
        return False
    runtime_state = _above_stage_state(objects, stage, current_world)
    trigger = str(runtime_state.pending_queue_review or "")
    if not trigger:
        return False
    runtime_state.pending_queue_review = ""
    violation = _above_queue_violation_reason(objects, stage, current_world)
    if not violation:
        _debug_print(f"  [TargetLockReplan] review={trigger} existing queue remains compatible")
        return False
    _clear_stopped_queue(objects, path_stream, state)
    print(f"  [TargetLockReplan] reason={violation} trigger={trigger}; old path cancelled")
    return True




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
    snapshot_stages: list | tuple | None = None,
    prebind_stages: list | tuple | None = None,
) -> int:
    if getattr(objects, "mission_memory", None) is None or not objects.mission_memory.enabled:
        return 0
    if frame is None or not query_stages:
        return 0
    updated_total = 0
    snapshot_stage_ids = {id(item) for item in (snapshot_stages or [])}
    prebind_stage_ids = {id(item) for item in (prebind_stages or [])}
    for query_stage in list(query_stages)[: max(0, int(max_queries))]:
        try:
            best_det, front_det, down_det, detect_elapsed, front_all, down_all = _detect_dual_view(
                objects,
                query_stage,
                task_text,
                frame,
                down_frame,
            )
        except Exception as exc:
            print(
                "  [PerceptionSync] detector_service_error "
                f"source={reason} target={_caption_for_stage(query_stage, task_text)!r} "
                f"error={type(exc).__name__}: {exc}; memory unchanged"
            )
            # A service/network error is not a negative observation.  Stop
            # this scan without aging, replacing, or clearing target memory.
            break
        observation_bundle = DetectionDepthBundle(
            best_detection=best_det,
            front_detection=front_det,
            down_detection=down_det,
            front_detections=list(front_all or []),
            down_detections=list(down_all or []),
            front_image=frame,
            down_image=down_frame,
            front_depth=front_depth,
            down_depth=down_depth,
            detect_elapsed=detect_elapsed,
            observer_world=list(observer_world) if observer_world is not None else None,
            observer_yaw_deg=(
                float(observer_yaw_deg) if observer_yaw_deg is not None else None
            ),
        )
        if id(query_stage) in prebind_stage_ids:
            # This call intentionally precedes depth attachment.  The saved
            # frame and tracker state therefore represent what RGB alone knew
            # before any surface point was formally accepted.
            _prebind_target_from_bundle(
                objects,
                query_stage,
                observation_bundle,
                source=f"{reason}_prebind",
            )
        _attach_depth_to_detection_lists(front_all, down_all, frame, down_frame, front_depth, down_depth)
        events = objects.mission_memory.update_from_detections(
            stage=query_stage,
            detections_by_view={"front": front_all or [], "down": down_all or []},
            images_by_view={"front": frame, "down": down_frame},
            observer_world=observer_world,
            observer_yaw_deg=observer_yaw_deg,
        )
        if id(query_stage) in prebind_stage_ids:
            lock_event = _primary_update_event(objects.mission_memory, query_stage, events)
            approved_front = getattr(lock_event, "detection", None)
            metric_usable, _depth_state = metric_depth_usable(
                approved_front,
                objects.mission_memory.config,
            )
            if objects.mission_memory.is_primary_locked(query_stage) and metric_usable:
                # Upgrade the temporary RGB bearing to the detector/depth
                # observation that actually agrees with the formal lock.
                _record_target_bearing(
                    objects,
                    query_stage,
                    observation_bundle,
                    approved_front,
                    source=f"{reason}_metric_lock",
                )
        if id(query_stage) in snapshot_stage_ids:
            _record_locked_target_snapshot(
                objects,
                query_stage,
                observation_bundle,
                source=reason,
                events=events,
            )
        if events:
            updated_total += len(events)
            primary = objects.mission_memory.primary_instance(query_stage)
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
                f"  [MemoryScan] reason={reason} target={_caption_for_stage(query_stage, task_text)!r} "
                f"updates={len(events)} primary={(primary.instance_id if primary else 'none')}"
                f"{fallback_text} "
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
        # Auxiliary landmark queries may update memory but are not navigation
        # targets, so only real task stages receive user-facing snapshots.
        snapshot_stages=list(objects.task_manager.stages or []),
        prebind_stages=list(objects.task_manager.stages or []),
    )


def _prebind_stage_target(objects: RuntimeObjects, client, stage, task_text: str) -> bool:
    """Synchronously acquire the active target's RGB bearing before planning."""

    if getattr(stage, "mode", "") not in {"target", "detect"}:
        return False
    memory = getattr(objects, "mission_memory", None)
    tracker = getattr(objects, "target_bearing_tracker", None)
    config = (
        getattr(memory, "config", {})
        if memory is not None
        else getattr(tracker, "config", {})
    ) or {}
    if not bool(config.get("PREBIND_ENABLED", True)):
        return False

    profile = str(config.get("PREBIND_CAPTURE_PROFILE", "front_down") or "front_down")
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
        best_det, front_det, down_det, detect_elapsed, front_all, down_all = _detect_dual_view(
            objects,
            stage,
            task_text,
            frame,
            down_frame,
        )
    except Exception as exc:
        print(
            f"  [TargetPrebind] skipped target={_caption_for_stage(stage, task_text)!r} "
            f"error={exc}"
        )
        return False

    bundle = DetectionDepthBundle(
        best_detection=best_det,
        front_detection=front_det,
        down_detection=down_det,
        front_detections=list(front_all or []),
        down_detections=list(down_all or []),
        front_image=frame,
        down_image=down_frame,
        # A custom profile may include depth, but prebinding must remain a
        # purely visual operation.  Formal depth attachment happens only in
        # the normal synchronized observation pipeline.
        front_depth=None,
        down_depth=None,
        detect_elapsed=detect_elapsed,
        observer_world=list(observer_world) if observer_world is not None else None,
        observer_yaw_deg=(float(observer_yaw_deg) if observer_yaw_deg is not None else None),
    )
    success = _prebind_target_from_bundle(
        objects,
        stage,
        bundle,
        source="stage_activation_prebind",
    )
    if not success:
        print(
            "  [TargetPrebind] "
            f"target={_caption_for_stage(stage, task_text)!r} candidate=none "
            f"profile={profile} capture={float((timing or {}).get('total_s', 0.0) or 0.0):.2f}s"
        )
    return success


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
        snapshot_stages=[stage],
        prebind_stages=[stage],
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
                    snapshot_stages=[stage],
                    prebind_stages=[stage],
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
        # The previous timestamp may be older than the detector's read
        # timeout. Restart the normal interval here instead of immediately
        # flooding an unavailable service with another opportunistic scan.
        objects.last_future_scan_s = time.perf_counter()
        print(
            "  [FutureScan] detector_service_error "
            f"elapsed={elapsed:.1f}s error={type(exc).__name__}: {exc}; "
            "current navigation and memory kept"
        )
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
            camera_frames=[
                camera_frame
                for camera_frame in (
                    client.camera_frame_for_image(front_depth)
                    if hasattr(client, "camera_frame_for_image")
                    else None,
                    client.camera_frame_for_image(down_depth)
                    if hasattr(client, "camera_frame_for_image")
                    else None,
                )
                if camera_frame is not None
            ],
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
    memory = getattr(objects, "mission_memory", None)
    overhead = (
        memory.above_overhead_context(stage, pos_now)
        if memory is not None and _is_above_stage(stage) and memory.has_primary(stage)
        else None
    )
    locked_primary = bool(memory is not None and memory.is_primary_locked(stage))
    if bool((overhead or {}).get("active", False)):
        direction_hint = SimpleNamespace(
            text="",
            reason="above overhead phase uses down view, not front RGB",
            angle_deg=None,
            bbox=None,
            score=0.0,
        )
        tracker = getattr(objects, "target_bearing_tracker", None)
        if tracker is not None:
            tracker.clear(CompletionPipeline.stage_key(stage))
    elif locked_primary:
        # A class-only, depthless detection cannot prove instance identity.
        # The approved asynchronous bearing or locked world geometry below is
        # authoritative once this stage has an immutable lock.
        direction_hint = SimpleNamespace(
            text="",
            reason="locked target requires identity-approved bearing",
            angle_deg=None,
            bbox=None,
            score=0.0,
        )
    else:
        direction_hint = direction_hint_from_front_detection(
            objects.detector,
            frame,
            _caption_for_stage(stage, instruction),
            stage=stage,
            additional_images=[down_frame] if down_frame is not None else [],
            observer_yaw_deg=yaw_now,
            excluded_bearings=(
                memory.previous_entity_exclusions(stage, pos_now, yaw_now)
                if memory is not None
                else []
            ),
        )
    tracker = getattr(objects, "target_bearing_tracker", None)
    bearing_observation = (
        tracker.current(CompletionPipeline.stage_key(stage))
        if tracker is not None and not bool((overhead or {}).get("active", False))
        else None
    )
    if not direction_hint.text and bearing_observation is not None:
        direction_hint = direction_hint_from_angle(
            bearing_observation.relative_to_yaw(yaw_now),
            score=bearing_observation.score,
            reason=(
                f"cached RGB bearing ({bearing_observation.depth_state}, "
                f"{bearing_observation.source})"
            ),
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
            memory_estimate = objects.mission_memory.estimate_distance(stage, pos_now)
            memory_trustworthy = _memory_estimate_is_trustworthy(objects, stage, pos_now)
            visual_is_fresh = bool(direction_hint.text and bearing_observation is not None)
            if (
                locked_confidence >= lock_min_confidence
                and (
                    not visual_is_fresh
                    or memory_trustworthy
                    and bool(getattr(stage, "return_target", False))
                )
            ):
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
            if (
                visual_is_fresh
                and not memory_trustworthy
                and memory_estimate is not None
            ):
                # Keep transition exclusions, but do not feed a stale/high-
                # uncertainty anchor back to Qwen as the active bearing.
                transition_hint = objects.mission_memory.transition_exclusion_hint(
                    stage,
                    pos_now,
                    yaw_now,
                )
                memory_hint = transition_hint
    if bearing_observation is not None and bearing_observation.depth_state != "metric":
        bearing_source = str(getattr(bearing_observation, "source", "runtime") or "runtime")
        memory_hint += (
            f"Fresh RGB bearing is authoritative: {bearing_observation.relative_to_yaw(yaw_now):+.1f} degrees "
            f"relative to the current nose (source={bearing_source}). Move only a short leg and reacquire depth; "
            "do not invent a far world coordinate. "
        )
    direction_text = direction_hint.text
    camera_images, geometry_context = _planning_geometry_context(
        objects,
        client,
        stage,
        pos_now,
        yaw_now,
        [frame, down_frame],
    )
    print(
        f"  [PlanInput] instruction={inst!r} pending={len(pending_for_model)} "
        f"pending_wp={_format_waypoints(pending_for_model)} "
        f"direction={direction_text!r} direction_reason={direction_hint.reason} "
        f"memory_hint={'yes' if memory_hint else 'no'} "
        f"front={web_helpers.shape_text(frame)} down={web_helpers.shape_text(down_frame)} "
        f"pose=({pos_now[0]:.2f},{pos_now[1]:.2f},{pos_now[2]:.2f}) yaw={yaw_now:.1f}"
    )

    nominal_velocity = max(0.1, float(cfg.get("SIM", {}).get("AIRSIM_VELOCITY", 2.0)))
    horizon_fn = getattr(objects.controller, "planning_horizon_m", None)
    planning_horizon_m = (
        float(horizon_fn(nominal_velocity))
        if callable(horizon_fn)
        else nominal_velocity * float(getattr(objects.controller, "reserve_time_s", 4.5))
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
            camera_images=camera_images,
            geometry_context=geometry_context,
            planning_horizon_m=planning_horizon_m,
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
        velocity_mps=nominal_velocity,
        plan_fn=plan_fn,
    )
    if submitted:
        pending = len(pending_for_model)
        _debug_print(
            f"  [Plan] submitted pending={pending} "
            f"remaining={len(objects.controller.queue.world_waypoints)}"
        )
    return submitted


def _select_and_transform_plan(objects: RuntimeObjects, stage, result, frame, down_frame):
    if result is None:
        return None
    cand_cfg = cfg.get("CANDIDATE", {}) or {}
    if not bool(cand_cfg.get("ENABLED", False)):
        raw_cumulative = incremental_to_cumulative(
            [list(wp) for wp in (getattr(result, "waypoints", []) or [])]
        )
        memory = getattr(objects, "mission_memory", None)
        selection_pos = getattr(result, "_selection_pos", [0.0, 0.0, 0.0])
        selection_yaw = float(getattr(result, "_selection_yaw", 0.0) or 0.0)
        guided, guidance_reason = _apply_locked_target_direction_guard(
            objects,
            stage,
            raw_cumulative,
            selection_pos=selection_pos,
            selection_yaw=selection_yaw,
        )
        if guidance_reason:
            print(
                f"  [TargetGuidance] {guidance_reason} "
                f"from={_format_waypoints(raw_cumulative)} to={_format_waypoints(guided)}"
            )
            raw_cumulative = guided
        bearing_guarded, bearing_reason = _apply_bearing_path_guard(
            objects,
            stage,
            raw_cumulative,
            selection_pos=selection_pos,
            selection_yaw=selection_yaw,
        )
        if bearing_reason:
            print(
                f"  [BearingPathGuard] {bearing_reason} "
                f"from={_format_waypoints(raw_cumulative)} to={_format_waypoints(bearing_guarded)}"
            )
            raw_cumulative = bearing_guarded
        vertical_guarded, vertical_reason = _apply_airsim_vertical_path_quantization(
            raw_cumulative,
            config=getattr(memory, "config", {}) or {},
        )
        if vertical_reason:
            print(
                f"  [AirSimVerticalGuard] {vertical_reason} "
                f"from={_format_waypoints(raw_cumulative)} to={_format_waypoints(vertical_guarded)}"
            )
        result.waypoints = cumulative_to_incremental(vertical_guarded)
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
    selection_pos = getattr(result, "_selection_pos", [0.0, 0.0, 0.0])
    selection_yaw = float(getattr(result, "_selection_yaw", 0.0) or 0.0)
    tracker = getattr(objects, "target_bearing_tracker", None)
    activation_bearing_guard = (
        tracker.activation_guard(_runtime_stage_key(stage))
        if tracker is not None and hasattr(tracker, "activation_guard")
        else None
    )
    bearing_only = _bearing_only_active(objects, stage, selection_pos)
    avoidance_memory_context = memory_context
    if bearing_only:
        # A stale metric anchor is still retained for explicit return stages,
        # but it must not bias a new far-target trajectory.
        memory_context = {}
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
    if bearing_only:
        guarded_cumulative, guard_reason = chosen_cumulative, ""
    else:
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
    if bearing_only:
        bearing_guarded, bearing_reason = _apply_bearing_path_guard(
            objects,
            stage,
            chosen_cumulative,
            selection_pos=selection_pos,
            selection_yaw=selection_yaw,
        )
        if bearing_reason:
            print(
                f"  [BearingPathGuard] {bearing_reason} "
                f"from={_format_waypoints(chosen_cumulative)} to={_format_waypoints(bearing_guarded)}"
            )
            chosen_cumulative = bearing_guarded
            guard_reason = bearing_reason
    small_guided, small_guidance_reason = _apply_small_above_target_guidance(
        objects,
        stage,
        chosen_cumulative,
        memory_context,
        selection_pos=selection_pos,
        planning_wall_s=qwen_elapsed,
    )
    if small_guidance_reason:
        print(
            f"  [AboveSmallGuidance] {small_guidance_reason} "
            f"from={_format_waypoints(chosen_cumulative)} to={_format_waypoints(small_guided)}"
        )
        chosen_cumulative = small_guided
        guard_reason = small_guidance_reason
    target_guarded, target_guidance_reason = _apply_locked_target_direction_guard(
        objects,
        stage,
        chosen_cumulative,
        selection_pos=selection_pos,
        selection_yaw=selection_yaw,
    )
    if target_guidance_reason:
        print(
            f"  [TargetGuidance] {target_guidance_reason} "
            f"from={_format_waypoints(chosen_cumulative)} to={_format_waypoints(target_guarded)}"
        )
        chosen_cumulative = target_guarded
        guard_reason = target_guidance_reason
    if bearing_only:
        roof_guarded, roof_guard_reason = chosen_cumulative, ""
    else:
        roof_guarded, roof_guard_reason = _apply_above_roof_acquisition_path_guard(
            objects,
            stage,
            chosen_cumulative,
            selection_pos=selection_pos,
            selection_yaw=selection_yaw,
            planning_wall_s=qwen_elapsed,
        )
    if roof_guard_reason:
        print(
            f"  [AboveRoofAcquire] {roof_guard_reason} "
            f"from={_format_waypoints(chosen_cumulative)} to={_format_waypoints(roof_guarded)}"
        )
        chosen_cumulative = roof_guarded
    obstacle_guarded, obstacle_reason, obstacle_result = _apply_obstacle_path_guard(
        objects,
        chosen_cumulative,
        selection_pos=selection_pos,
        selection_yaw=selection_yaw,
        memory_context=avoidance_memory_context,
    )
    if obstacle_reason:
        print(
            f"  [DepthObstacleGuard] {obstacle_reason} "
            f"from={_format_waypoints(chosen_cumulative)} to={_format_waypoints(obstacle_guarded)}"
        )
        chosen_cumulative = obstacle_guarded
    above_guarded, above_reason = _apply_above_altitude_path_guard(
        objects,
        stage,
        chosen_cumulative,
        selection_pos=selection_pos,
    )
    if above_reason:
        print(
            f"  [AbovePathGuard] {above_reason} "
            f"from={_format_waypoints(chosen_cumulative)} to={_format_waypoints(above_guarded)}"
        )
        chosen_cumulative = above_guarded
    vertical_guarded, vertical_reason = _apply_airsim_vertical_path_quantization(
        chosen_cumulative,
        config=(getattr(getattr(objects, "mission_memory", None), "config", {}) or {}),
    )
    if vertical_reason:
        print(
            f"  [AirSimVerticalGuard] {vertical_reason} "
            f"from={_format_waypoints(chosen_cumulative)} to={_format_waypoints(vertical_guarded)}"
        )
        chosen_cumulative = vertical_guarded
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
        "above_roof_acquisition_guard": roof_guard_reason,
        "obstacle_path_guard": obstacle_reason,
        "above_altitude_guard": above_reason,
        "airsim_vertical_guard": vertical_reason,
        "all_candidates": [cand.to_dict() for cand in all_candidates],
        "wm_candidates": [cand.to_dict() for cand in (selection.wm_candidates or [])],
        "selected_candidate": chosen.to_dict(),
    }
    transformed.candidates = [cand.to_dict() for cand in all_candidates]
    transformed.selected_index = selected_index
    transformed.selected_candidate = chosen.to_dict()
    transformed._consume_activation_bearing_guard = bool(
        activation_bearing_guard is not None and chosen_incremental
    )
    return transformed


def _poll_plan(objects: RuntimeObjects, stage=None, frame=None, down_frame=None, state=None, client=None) -> None:
    select_fn = lambda r: _select_and_transform_plan(objects, stage, r, frame, down_frame)
    current_pos = current_yaw = None
    if client is not None:
        try:
            current_pos, current_yaw = client.get_pose()
        except Exception:
            current_pos = current_yaw = None
    try:
        result = objects.controller.poll_plan(
            select_fn=select_fn,
            current_pos=current_pos,
            current_yaw_deg=current_yaw,
        )
    except TypeError as exc:
        # Keep compatibility with lightweight controller doubles and older
        # integrations that expose the pre-rebase ``poll_plan(select_fn)``
        # signature.  Real controller errors are re-raised.
        if "unexpected keyword" not in str(exc):
            raise
        result = objects.controller.poll_plan(select_fn=select_fn)
    if result is None:
        return
    qwen_inc = getattr(result, "waypoints", []) or []
    if qwen_inc and bool(getattr(result, "_consume_activation_bearing_guard", False)):
        tracker = getattr(objects, "target_bearing_tracker", None)
        consumed = (
            tracker.consume_activation_guard(_runtime_stage_key(stage))
            if tracker is not None and hasattr(tracker, "consume_activation_guard")
            else None
        )
        if consumed is not None:
            print(
                "  [TargetPrebind] activation bearing consumed after first path enqueue "
                f"angle={consumed.relative_angle_deg:+.1f}deg"
            )
    rejection_reason = getattr(result, "rejection_reason", "")
    if rejection_reason:
        print(f"  [PlanRejected] {rejection_reason}; existing queue kept")
    _debug_print(f"  [QwenSliding] raw={_format_waypoints(qwen_inc)}")
    if bool(getattr(result, "_stale_plan_rebased", False)):
        print(
            "  [PlanRebase] stale Qwen suffix rebased to live pose "
            f"reason={getattr(result, '_stale_plan_rebase_reason', 'pose_drift')}"
        )
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
        or _is_above_stage(stage)
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
    if (
        snapshot_depth_aligned
        and snapshot_down_depth is not None
        and snapshot_pos is not None
        and snapshot_yaw is not None
    ):
        _record_synchronized_roof_plane(
            objects,
            stage,
            snapshot_down_depth,
            snapshot_pos,
            snapshot_yaw,
            source="planning_snapshot",
            camera_frame=(
                client.camera_frame_for_image(snapshot_down_depth)
                if hasattr(client, "camera_frame_for_image")
                else None
            ),
            additional_camera_frames=[
                client.camera_frame_for_image(snapshot_front_depth)
                if hasattr(client, "camera_frame_for_image")
                else None
            ],
        )
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
    if str(getattr(completion, "accepted_view", "none") or "none").lower() == "none":
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
    event=None,
) -> bool:
    """Recover by target scale without allowing repeated building scans."""
    current_stage_key = CompletionPipeline.stage_key(stage)
    bearing_tracker = getattr(objects, "target_bearing_tracker", None)
    memory = getattr(objects, "mission_memory", None)
    stale, stale_reason = _target_lost_event_is_stale(objects, stage, event)
    if stale:
        print(f"  [TargetLost] stale_event_discarded reason={stale_reason}")
        return False
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

    pos_now, yaw_now = client.get_pose()
    instance = memory.primary_instance(stage) if memory is not None and memory.has_primary(stage) else None
    large_target = bool(
        is_large_structure_stage(stage)
        or (instance is not None and bool(getattr(instance, "is_large_structure", False)))
    )
    if large_target:
        expected_offscreen, offscreen_reason = _large_target_expected_offscreen(
            objects,
            stage,
            pos_now,
        )
        recovery = _target_lost_recovery(objects)
        budget = recovery.large_loss_budget(
            stage_key=current_stage_key,
            instance_id=str(getattr(instance, "instance_id", "") or ""),
            current_position=pos_now,
            reason=reason,
            expected_offscreen=expected_offscreen,
        )
        controller = getattr(objects, "controller", None)
        queue = getattr(controller, "queue", None)
        navigation_active = bool(
            getattr(path_stream, "active", False)
            or list(getattr(queue, "world_waypoints", []) or [])
            or bool(getattr(controller, "planning", False))
            or bool(getattr(controller, "has_plan_job", False))
        )
        bearing_support = None
        if bearing_tracker is not None:
            activation_fn = getattr(bearing_tracker, "activation_guard", None)
            current_fn = getattr(bearing_tracker, "current", None)
            bearing_support = (
                activation_fn(current_stage_key)
                if callable(activation_fn)
                else None
            ) or (
                current_fn(current_stage_key)
                if callable(current_fn)
                else None
            )
        if instance is None and bearing_support is None and not navigation_active:
            return _fail_current_stage_after_relocalization(
                objects,
                path_stream,
                state,
                current_stage_key,
                (
                    "large target has neither a locked surface nor a fresh RGB bearing; "
                    "stopped instead of issuing an unguided path or rotating in place"
                ),
            )
        if bool(budget.get("continue", False)):
            if bearing_tracker is not None and expected_offscreen and hasattr(bearing_tracker, "reset_lost"):
                bearing_tracker.reset_lost(current_stage_key)
            phase = "expected_offscreen_down_view" if expected_offscreen else "bounded_memory_navigation"
            print(
                f"  [TargetLost] large_structure_no_yaw_search phase={phase} "
                f"active_navigation={navigation_active} reason={offscreen_reason}; "
                f"blind_elapsed={float(budget.get('elapsed_s', 0.0)):.1f}s "
                f"blind_distance={float(budget.get('distance_m', 0.0)):.1f}m; "
                "queue and completed planner job kept"
            )
            if memory is not None:
                state.update(memory_summary=memory.summary(stage))
            return False
        return _fail_current_stage_after_relocalization(
            objects,
            path_stream,
            state,
            current_stage_key,
            (
                "large target remained unexpectedly invisible beyond no-yaw blind budget "
                f"({float(budget.get('elapsed_s', 0.0)):.1f}s, "
                f"{float(budget.get('distance_m', 0.0)):.1f}m); "
                "stopped instead of rotating in place"
            ),
        )

    if bearing_tracker is not None:
        lost_count = bearing_tracker.mark_lost(current_stage_key)
        required_losses = int(
            getattr(memory, "config", {}).get("BEARING_LOST_CONFIRMATIONS", 2)
        )
        if lost_count < max(1, required_losses):
            print(
                f"  [TargetLost] transient_rgb_miss count={lost_count}/{required_losses}; "
                "active path kept"
            )
            return False

    recovery = _target_lost_recovery(objects)
    reliable_memory = _relocalization_memory_is_reliable(objects, stage, pos_now)
    preferred_yaw = (
        memory.preferred_yaw_deg(stage, pos_now)
        if reliable_memory and memory is not None
        else None
    )
    session, session_reason = recovery.begin_small_session(
        stage_key=current_stage_key,
        instance_id=str(getattr(instance, "instance_id", "") or ""),
        current_position=pos_now,
        current_yaw_deg=yaw_now,
        preferred_yaw_deg=preferred_yaw,
        reliable_memory=reliable_memory,
    )
    if session is None:
        if "cooldown" in session_reason:
            print(f"  [TargetLost] compact_target_recovery_deferred reason={session_reason}; path kept")
            return False
        return _fail_current_stage_after_relocalization(
            objects,
            path_stream,
            state,
            current_stage_key,
            session_reason,
        )
    if not getattr(objects, "relocalizer", None) or not objects.relocalizer.enabled:
        return _fail_current_stage_after_relocalization(
            objects,
            path_stream,
            state,
            current_stage_key,
            "compact target lost and bounded relocalization is disabled",
        )

    _clear_stopped_queue(objects, path_stream, state)
    objects.distance_estimator.clear()
    objects.navigation_metrics.invalidate_target(current_stage_key)
    offsets = recovery.search_offsets(session)
    max_rotation = float(
        recovery.config.get(
            "SMALL_GLOBAL_MAX_TOTAL_ROTATION_DEG" if session.global_search else "SMALL_LOCAL_MAX_TOTAL_ROTATION_DEG",
            420.0,
        )
    )
    print(
        "  [TargetLost] compact_target_bounded_search "
        f"session={session.session_id} reliable_memory={reliable_memory} "
        f"base_yaw={session.preferred_yaw_deg:.1f} offsets={offsets} reason={reason}"
    )
    relocalized = objects.relocalizer.search(
        client,
        stage,
        capture_mode=capture_mode,
        skip_initial_frame=True,
        validator=_build_locked_relocalization_validator(objects, client, stage),
        yaw_offsets_deg=offsets,
        base_yaw_deg=session.preferred_yaw_deg,
        max_total_rotation_deg=max_rotation,
        session_id=session.session_id,
        expected_target_world=(
            list((memory.estimate_distance(stage, pos_now) or {}).get("target_world") or [])
            if memory is not None
            else None
        ),
    )
    recovery.finish_small_session(
        session,
        found=bool(relocalized.found),
        searched_yaws_deg=relocalized.searched_yaws_deg,
        total_rotation_deg=relocalized.total_rotation_deg,
    )
    print(
        f"  [Relocalize] session={session.session_id} found={relocalized.found} "
        f"view={(relocalized.detection.camera if relocalized.detection else 'none')} "
        f"yaw_delta={relocalized.yaw_delta_deg:.1f}deg "
        f"total_rotation={relocalized.total_rotation_deg:.1f}deg "
        f"views={len(relocalized.searched_yaws_deg)} elapsed={relocalized.elapsed:.2f}s "
        f"reason={relocalized.reason}"
    )
    if not relocalized.found:
        return _fail_current_stage_after_relocalization(
            objects,
            path_stream,
            state,
            current_stage_key,
            relocalized.reason or "compact target not found in bounded search",
        )

    previous_instance_id = str(getattr(instance, "instance_id", "") or "")
    bundle = _bundle_from_relocalization_result(relocalized)
    _update_target_pose_from_bundle(objects, stage, bundle)
    current_instance_id = _locked_instance_id(objects, stage)
    if previous_instance_id and current_instance_id != previous_instance_id:
        return _fail_current_stage_after_relocalization(
            objects,
            path_stream,
            state,
            current_stage_key,
            (
                "relocalization candidate did not preserve locked identity "
                f"{previous_instance_id} (got {current_instance_id or 'none'})"
            ),
        )
    if bearing_tracker is not None and hasattr(bearing_tracker, "reset_lost"):
        bearing_tracker.reset_lost(current_stage_key)
    recovery.mark_observed(current_stage_key)
    if getattr(objects, "completion_pipeline", None) is not None:
        objects.completion_pipeline.clear()
    state.update(memory_summary=memory.summary(stage) if memory is not None else {})
    return False

def _handle_above_completion_trigger(
    objects: RuntimeObjects,
    client,
    path_stream,
    state,
    stage,
    task_text: str,
    trigger_radius_m: float,
) -> bool | None:
    """Complete a locked large-building ``above`` stage from synchronized roof geometry.

    ``None`` means this is not a locked large-structure case and lets the
    caller retain the legacy small-target/VLM path.
    """
    memory = getattr(objects, "mission_memory", None)
    instance = memory.primary_instance(stage) if memory is not None else None
    if not (
        memory is not None
        and instance is not None
        and memory.is_primary_locked(stage)
        and bool(getattr(instance, "is_large_structure", False))
    ):
        return None

    profile = str(
        getattr(objects.completion_checker, "depth_profile", "front_down_both_depth")
        or "front_down_both_depth"
    )
    try:
        (
            frame,
            down_frame,
            front_depth,
            down_depth,
            timing,
            observer_world,
            observer_yaw,
        ) = web_helpers.capture_profile_isolated_with_pose(client, profile)
    except Exception as exc:
        print(f"  [AboveCompletion] synchronized capture failed: {exc}; resuming navigation")
        above_state = _above_stage_state(objects, stage, client.get_pose()[0])
        above_state.completion_resume_pose = list(client.get_pose()[0])
        return False
    timing = dict(timing or {})
    print(
        f"  [AboveCompletion] synchronized profile={profile} "
        f"time={float(timing.get('total_s', 0.0) or 0.0):.2f}s "
        f"down_depth={web_helpers.shape_text(down_depth)}"
    )
    _record_synchronized_roof_plane(
        objects,
        stage,
        down_depth,
        observer_world,
        observer_yaw,
        state=state,
        source="above_completion",
        camera_frame=(
            client.camera_frame_for_image(down_depth)
            if hasattr(client, "camera_frame_for_image")
            else None
        ),
        additional_camera_frames=[
            client.camera_frame_for_image(front_depth)
            if hasattr(client, "camera_frame_for_image")
            else None
        ],
    )

    # Down RGB is optional semantic support.  The full-frame depth plane is
    # still evaluated when GroundingDINO cannot produce a meaningful bbox.
    approved_down = None
    visual_score = 0.0
    try:
        _best, front_det, down_det, _elapsed, front_all, down_all = _detect_dual_view(
            objects,
            stage,
            task_text,
            frame,
            down_frame,
        )
        _attach_depth_to_detection_lists(
            front_all,
            down_all,
            frame,
            down_frame,
            front_depth,
            down_depth,
        )
        bundle = DetectionDepthBundle(
            best_detection=_best,
            front_detection=front_det,
            down_detection=down_det,
            front_detections=list(front_all or []),
            down_detections=list(down_all or []),
            front_image=frame,
            down_image=down_frame,
            front_depth=front_depth,
            down_depth=down_depth,
            observer_world=list(observer_world),
            observer_yaw_deg=float(observer_yaw),
            distance_view="down" if down_det is not None and getattr(down_det, "visible", False) else "front",
            distance_reason="above_synchronized_completion",
        )
        approved_down = _select_identity_approved_down_detection(objects, stage, bundle)
        _update_target_pose_from_bundle(objects, stage, bundle)
        if approved_down is not None:
            visual_score = _detection_reliability(stage, approved_down, down_frame)
    except Exception as exc:
        _debug_print(f"  [AboveCompletion] optional RGB detection skipped: {exc}")

    decision = memory.evaluate_completion(
        stage=stage,
        current_world=observer_world,
        fresh_visual_support=approved_down is not None,
        visual_score=visual_score,
        stop_radius_m=float(trigger_radius_m),
    )
    state.update(memory_summary=memory.summary(stage))
    print(
        f"  [AboveCompletion] status={decision.status} confidence={decision.confidence:.2f} "
        f"reason={decision.reason}"
    )
    if decision.done:
        return _complete_stage_from_memory(objects, path_stream, state, stage, decision)

    above_state = _above_stage_state(objects, stage, observer_world)
    roof = memory.roof_navigation_context(stage, observer_world)
    if not bool((roof or {}).get("trusted", False)):
        above_state.require_roof_before_next_completion = True
        above_state.completion_resume_pose = list(observer_world)
        print(
            "  [AboveCompletion] roof evidence is not yet stable; "
            "resuming altitude-held XY navigation until synchronized down depth confirms a roof"
        )
    else:
        above_state.completion_resume_pose = list(observer_world)
        print("  [AboveCompletion] roof is locked but constraints are not met; resuming bounded adjustment")
    objects.completion_attempts.pop(CompletionPipeline.stage_key(stage), None)
    objects.completion_retry_after.pop(CompletionPipeline.stage_key(stage), None)
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
    # Small ``above`` targets (fountains, cars, people, ...) are completed by
    # two fresh down-view center confirmations.  A distance trigger must never
    # stop or clear their target-guided path before that visual contract is met.
    # Keeping this guard here as well as in ``_should_trigger_completion_vlm``
    # protects callers that submit a stale/asynchronous trigger directly.
    if _small_above_target(objects, stage) is not None:
        print("  [CompletionGate] small_above_distance_trigger_ignored; down-view center confirmation owns completion")
        return False
    cached_distance_m = float(
        cached_distance.get("distance_m") if isinstance(cached_distance, dict)
        else getattr(cached_distance, "distance_m", 0.0)
    )
    trigger_display_m = float(
        cached_distance.get("trigger_radius_m") if isinstance(cached_distance, dict) and cached_distance.get("trigger_radius_m") is not None
        else getattr(cached_distance, "trigger_radius_m", trigger_radius_m)
    )
    # Never clear an active path from an old/asynchronous distance estimate.
    # Re-read the pose and compare it with the same locked target before the
    # stop-and-confirm transition.
    try:
        live_pos, _live_yaw = client.get_pose()
    except Exception:
        live_pos = None
    live_distance = _estimated_pose_xy_distance(cached_distance, live_pos)
    live_tolerance = max(
        0.0,
        float(function_section(cfg, "FAST_SLOW").get("COMPLETION_LIVE_DISTANCE_TOLERANCE_M", 0.25)),
    )
    if live_distance is not None and live_distance > trigger_display_m + live_tolerance:
        print(
            "  [CompletionGate] stale_trigger_ignored "
            f"live_distance={live_distance:.2f}m "
            f"estimate={cached_distance_m:.2f}m radius={trigger_display_m:.2f}m"
        )
        return False
    if live_distance is None:
        cached_target = (
            cached_distance.get("target_world")
            if isinstance(cached_distance, dict)
            else getattr(cached_distance, "target_world", None)
        )
        if live_pos is None or cached_target:
            # A cached distance is not sufficient evidence to stop an active
            # path when the same target can be recomputed from the live pose.
            # Surface-only completion records may intentionally omit a point
            # target; those are evaluated by MissionMemory below.
            print(
                "  [CompletionGate] live_pose_unavailable "
                f"estimate={cached_distance_m:.2f}m radius={trigger_display_m:.2f}m; queue kept"
            )
            return False
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

    if _is_above_stage(stage):
        above_result = _handle_above_completion_trigger(
            objects,
            client,
            path_stream,
            state,
            stage,
            task_text,
            trigger_radius_m,
        )
        if above_result is not None:
            return bool(above_result)

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

    try:
        best_det, front_det, down_det, detect_elapsed, front_all, down_all = _detect_dual_view(
            objects,
            stage,
            task_text,
            frame,
            down_frame,
        )
    except Exception as exc:
        completion_cfg = function_section(cfg, "FAST_SLOW")
        backoff_s = max(
            0.1,
            float(completion_cfg.get("PERCEPTION_ERROR_RETRY_BACKOFF_S", 5.0)),
        )
        objects.completion_retry_after[current_stage_key] = time.perf_counter() + backoff_s
        print(
            "  [PerceptionSync] detector_service_error "
            f"error={type(exc).__name__}: {exc} retry_after={backoff_s:.1f}s; "
            "completion postponed, locked memory and yaw kept"
        )
        return False
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
        print("  [TargetLost] fresh completion RGB miss; applying scale-aware recovery")
        return _handle_background_target_lost(
            objects,
            client,
            path_stream,
            state,
            stage,
            capture_mode,
            "target not detected in fresh completion RGB",
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

    # Small ``above`` targets use a down-view center contract, not the VLM
    # building/roof completion path.  Apply the same contract here as in the
    # asynchronous observation pipeline; otherwise a visible-but-off-center
    # fountain/car would stop the path and enter a VLM retry loop.
    if _small_above_target(objects, stage) is not None:
        try:
            observer_world, observer_yaw = client.get_pose()
        except Exception:
            observer_world, observer_yaw = [], 0.0
        small_bundle = DetectionDepthBundle(
            best_detection=best_det,
            front_detection=front_det,
            down_detection=down_det,
            front_detections=list(front_all or []),
            down_detections=list(down_all or []),
            front_image=frame,
            down_image=down_frame,
            front_depth=front_depth,
            down_depth=down_depth,
            observer_world=list(observer_world or []),
            observer_yaw_deg=float(observer_yaw or 0.0),
            distance_view="down" if down_det is not None else "front",
            distance_reason="synchronous_small_above_completion",
            capture_id=f"sync-{time.perf_counter_ns()}",
        )
        small_handled, small_done = _handle_small_above_down_center_observation(
            objects,
            path_stream,
            state,
            stage,
            small_bundle,
        )
        if small_handled:
            return bool(small_done)
        print("  [AboveSmall] target not centered in down view; resuming target-guided path")
        return False

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
    # Keep the live pose beside the estimate.  Completion decisions may run
    # after an RGB/depth or Qwen call, so the stored estimator distance alone
    # is not sufficient to prove that AirSim has entered the arrival region.
    try:
        estimated.current_world = [float(value) for value in current_world[:3]]
        estimated.distance_observed_at_s = time.perf_counter()
    except (AttributeError, TypeError, ValueError):
        pass
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
    # Small ``above`` completion is a visual center contract, not a radius
    # contract.  Distance-based stopping here used to clear the active queue
    # while the target was still off-center, leaving no path to recover.
    if _small_above_target(objects, stage) is not None:
        return False
    if _is_above_stage(stage):
        current = list(
            (estimated.get("current_world") if isinstance(estimated, dict) else getattr(estimated, "current_world", None))
            or []
        )
        if len(current) < 3 and _locked_large_above_requires_roof(objects, stage):
            return False
        if len(current) >= 3:
            above_state = _above_stage_state(objects, stage, current)
            memory = getattr(objects, "mission_memory", None)
            roof = memory.roof_navigation_context(stage, current) if memory is not None else None
            if not _above_roof_candidate_ready(
                objects,
                stage,
                current,
                require_trusted=above_state.require_roof_before_next_completion,
            ):
                return False
            if bool((roof or {}).get("trusted", False)):
                clearance = float((roof or {}).get("clearance_m", 0.0) or 0.0)
                min_clearance = float(memory.config.get("ABOVE_MIN_CLEARANCE_M", 0.3))
                if clearance < min_clearance:
                    # The UAV must be numerically above the trusted roof.  No
                    # maximum clearance is part of the semantic relation.
                    return False
            resume_pose = list(above_state.completion_resume_pose or [])
            if len(resume_pose) >= 3:
                moved = math.sqrt(sum((float(current[i]) - float(resume_pose[i])) ** 2 for i in range(3)))
                min_motion = float(
                    getattr(memory, "config", {}).get("ABOVE_COMPLETION_RESUME_MOTION_M", 0.75)
                    if memory is not None
                    else 0.75
                )
                if moved < min_motion:
                    return False
                above_state.completion_resume_pose = None
    source = str(getattr(estimated, "source", "distance_estimator") or "distance_estimator")
    if source != "mission_memory" and not objects.distance_estimator.use_for_completion:
        return False
    radius = _memory_distance_trigger_radius(objects, stage, trigger_radius_m) if source == "mission_memory" else float(trigger_radius_m)
    try:
        estimated.trigger_radius_m = radius
    except Exception:
        pass
    live_distance = _estimated_pose_xy_distance(estimated)
    tolerance = max(
        0.0,
        float(
            function_section(cfg, "FAST_SLOW").get(
                "COMPLETION_LIVE_DISTANCE_TOLERANCE_M",
                0.25,
            )
        ),
    )
    if live_distance is not None and live_distance > radius + tolerance:
        _debug_print(
            "  [CompletionGate] stale_estimate_ignored "
            f"live_distance={live_distance:.2f}m estimate={float(getattr(estimated, 'distance_m', live_distance)):.2f}m "
            f"radius={radius:.2f}m"
        )
        return False
    return bool(float(estimated.distance_m) <= float(radius))


def _queue_reaches_memory_arrival(objects, stage, trigger_radius_m: float) -> bool:
    memory = getattr(objects, "mission_memory", None)
    if memory is None or not memory.has_primary(stage):
        return False
    queued = list(objects.controller.queue.world_waypoints or [])
    if not queued:
        return False
    require_trusted_roof = bool(
        _is_above_stage(stage)
        and _above_stage_state(objects, stage, queued[0]).require_roof_before_next_completion
    )
    if _locked_large_above_requires_roof(objects, stage) and not _above_roof_candidate_ready(
        objects,
        stage,
        queued[0],
        require_trusted=require_trusted_roof,
    ):
        # A facade arrival circle must not suppress the next planner request;
        # the vehicle still needs a continuous XY path into the roof footprint.
        return False
    radius = _memory_distance_trigger_radius(objects, stage, trigger_radius_m)
    for waypoint in queued:
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
    if relation != "near" or _is_above_stage(stage):
        # ``above`` stages are governed by their own completion contract:
        # buildings use roof geometry and small targets use down-view center
        # confirmations.  Neither should be clipped by the generic XY radius
        # guard, which can stop a small target before its visual center is
        # observable and can also fight the roof-acquisition path.
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
    if _is_above_stage(stage):
        above_violation = _above_queue_violation_reason(objects, stage, current_world)
        if above_violation:
            _clear_stopped_queue(objects, path_stream, state)
            print(
                "  [AbovePathGuard] stopped unsafe active queue "
                f"reason={above_violation}"
            )
            return True
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
    if _is_above_stage(stage):
        # ``above`` is already handled by its normal roof-distance trigger.
        # The wide facade/footprint outer radius is only a phase-switch hint;
        # using it for idle completion caused an unmoving capture loop.
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
    above_state = None
    if _is_above_stage(stage):
        above_state = _above_stage_state(objects, stage, current_pos)
        # A vertical-first facade climb is deliberately yaw invariant.  The
        # next RGB/depth observation must be captured before any horizontal
        # recovery can rotate the camera away from the wall.
        pending_climb_z = getattr(above_state, "pre_roof_climb_target_z", None)
        climb_tolerance = float(memory_cfg.get("ABOVE_PRE_ROOF_CLIMB_TOLERANCE_M", 0.4))
        if pending_climb_z is not None and float(current_pos[2]) > float(pending_climb_z) + climb_tolerance:
            return False
        overhead_fn = getattr(memory, "above_overhead_context", None)
        overhead = overhead_fn(stage, current_pos) if callable(overhead_fn) else {}
        primary_fn = getattr(memory, "primary_instance", None)
        target_instance = primary_fn(stage) if callable(primary_fn) else None
        target_world = list(getattr(target_instance, "target_world", None) or []) if target_instance is not None else []
        target_body = world_to_body(target_world, current_pos, current_yaw) if len(target_world) >= 3 else []
        target_is_behind = bool(
            len(target_body) >= 2
            and float(target_body[0]) < -float(memory_cfg.get("PATH_TARGET_BEHIND_X_M", 2.0))
        )
        try:
            roof_ready = _above_roof_candidate_ready(
                objects,
                stage,
                current_pos,
                require_trusted=bool(getattr(above_state, "require_roof_before_next_completion", False)),
            )
        except (AttributeError, TypeError):
            # Lightweight clients/tests may expose only the legacy memory
            # methods.  In that case a behind-target recovery is safer than
            # assuming a roof is already trusted.
            roof_ready = False
        overhead_active = bool((overhead or {}).get("active", False))
        # Being inside the overhead envelope is not by itself a reason to
        # forbid recovery.  A target can be at the edge of that envelope and
        # already behind the UAV after an overshoot.  Only a roof-confirmed,
        # completion-ready large target suppresses the turn.
        if overhead_active and roof_ready and not target_is_behind:
            return False
        if overhead_active and roof_ready and target_is_behind:
            overhead_radius = float((overhead or {}).get("trigger_radius_m", 0.0) or 0.0)
            horizontal = math.hypot(float(target_body[0]), float(target_body[1])) if len(target_body) >= 2 else float("inf")
            if overhead_radius > 0.0 and horizontal <= overhead_radius:
                return False
        if above_state is not None:
            max_attempts = max(1, int(memory_cfg.get("ABOVE_MAX_REORIENT_ATTEMPTS", 1)))
            if int(getattr(above_state, "reorient_attempts", 0) or 0) >= max_attempts:
                print("  [MemoryReorient] bounded recovery already used; keeping target-directed queue")
                return False
            cooldown = max(0.0, float(memory_cfg.get("ABOVE_REORIENT_COOLDOWN_S", 4.0)))
            if time.perf_counter() - float(getattr(above_state, "last_reorient_at", 0.0) or 0.0) < cooldown:
                return False
    preferred_yaw = memory.preferred_yaw_deg(stage, current_pos)
    if preferred_yaw is None:
        return False
    yaw_delta = _signed_yaw_delta_deg(preferred_yaw, current_yaw)
    trigger_deg = max(45.0, float(memory_cfg.get("RETURN_TARGET_REORIENT_TRIGGER_DEG", 90.0)))
    if abs(yaw_delta) < trigger_deg:
        return False

    path_stream.stop()
    # For an ``above`` recovery the queued path was generated from the old
    # heading and may already have crossed the target, so the next scheduler
    # pass must capture a fresh view.  Ordinary near/return stages preserve an
    # intentional world-coordinate return waypoint after rotating.
    if above_state is not None:
        clear_controller = getattr(controller, "clear", None)
        if callable(clear_controller):
            clear_controller()
        elif getattr(controller, "queue", None) is not None:
            controller.queue.world_waypoints[:] = []
        if getattr(objects, "completion_pipeline", None) is not None:
            objects.completion_pipeline.clear()
        state.update(trajectory_queue=[], qwen_waypoints=[])
    if above_state is not None:
        above_state.reorient_attempts = int(getattr(above_state, "reorient_attempts", 0) or 0) + 1
        above_state.last_reorient_at = time.perf_counter()
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


def _sync_path_if_ready(objects, path_stream, client, state, stage=None) -> int:
    """Compatibility wrapper that injects runtime collision/reorientation policies."""
    return _execution_sync_path_if_ready(
        objects,
        path_stream,
        client,
        state,
        stage=stage,
        record_collision=_record_locked_target_collision,
        reorient_if_behind=_reorient_to_locked_target_if_behind,
        debug_print=_debug_print,
        drop_behind=_drop_path_points_behind_vehicle,
        velocity_fn=_continuous_path_velocity,
        format_fn=_format_waypoints,
    )


class _CompletionRadiusWatchdog(_ExecutionCompletionRadiusWatchdog):
    """Compatibility wrapper binding runtime completion callbacks."""

    def __init__(self, objects, client, path_stream, stage, trigger_radius_m: float, interval_s: float):
        super().__init__(
            objects,
            client,
            path_stream,
            stage,
            trigger_radius_m,
            interval_s,
            stage_key_fn=CompletionPipeline.stage_key,
            distance_fn=_cached_target_distance,
            trigger_fn=_should_trigger_completion_vlm,
        )


def run_fast_slow_loop(
    state,
    initial_task: str,
    max_steps: int,
    client,
    capturer=None,
    *,
    isolated_planning_capture: bool = False,
    target_snapshot_recorder: TargetSnapshotRecorder | None = None,
) -> None:
    global _LAST_NAVIGATION_METRICS
    if target_snapshot_recorder is not None:
        target_snapshot_recorder.begin_task(initial_task)
        objects = _build_runtime_objects(target_snapshot_recorder)
    else:
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
        error_retry_backoff_s=float(fast_slow_cfg.get("PERCEPTION_ERROR_RETRY_BACKOFF_S", 5.0)),
        error_retry_max_backoff_s=float(fast_slow_cfg.get("PERCEPTION_ERROR_RETRY_MAX_BACKOFF_S", 30.0)),
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
        if getattr(objects, "target_bearing_tracker", None) is not None:
            objects.target_bearing_tracker.clear()
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
            _bump_stage_generation(objects, stage)
            _target_lost_recovery(objects).reset(CompletionPipeline.stage_key(stage))
            objects.distance_estimator.clear()
            # 阶段刚切换时先检查一次memory/距离缓存；如果已经在目标附近，不再盲目起新规划。
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
            if _is_above_stage(stage):
                stage_entry_pos, _stage_entry_yaw = client.get_pose()
                _above_stage_state(objects, stage, stage_entry_pos)
            if getattr(stage, "mode", "") in {"target", "detect"}:
                # Resolve/prebind the active target synchronously before the
                # first slow plan.  Otherwise the planner can consume an RGB
                # frame before its asynchronous detector has supplied any
                # target direction and emit a default straight-ahead leg.
                with web_helpers.pause_background_capture(capturer):
                    if is_view_relative_stage(stage):
                        # Relative identity is defined only after all prior
                        # move/turn stages have completed in this exact view.
                        _bind_view_relative_stage(objects, client, state, stage, task_text)
                    else:
                        _prebind_stage_target(objects, client, stage, task_text)

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

            _poll_plan(objects, stage=stage, frame=frame, down_frame=down_frame, state=state, client=client)

            event = objects.completion_pipeline.poll() if objects.completion_pipeline is not None else None
            if event is not None:
                current_pipeline_key = CompletionPipeline.stage_key(stage)
                if event.stage_key != current_pipeline_key:
                    print(f"  [CompletionAsync] stale event={event.kind}; discarded")
                elif event.kind == "perception_error":
                    print(
                        "  [PerceptionAsync] detector_service_error "
                        f"error={event.error} retry_after={event.retry_after_s:.1f}s; "
                        "active path, locked memory and yaw kept"
                    )
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
                        event=event,
                    )
                    if should_break:
                        break
                    continue
                elif event.kind == "observation":
                    small_handled, small_done = _handle_small_above_down_center_observation(
                        objects,
                        path_stream,
                        state,
                        stage,
                        event.bundle,
                    )
                    if small_handled:
                        if small_done:
                            break
                        continue
                    _update_target_pose_from_bundle(objects, stage, event.bundle)
                    review_pos, _review_yaw = client.get_pose()
                    if _review_pending_above_queue(objects, path_stream, state, stage, review_pos):
                        continue
                elif event.kind in {"near_stop", "done_candidate", "not_done"}:
                    # Compatibility guard for stale jobs created before the
                    # distance-only mode was enabled. They may refresh the
                    # target pose but can never complete or reject a stage.
                    small_handled, small_done = _handle_small_above_down_center_observation(
                        objects,
                        path_stream,
                        state,
                        stage,
                        event.bundle,
                    )
                    if small_handled:
                        if small_done:
                            break
                        continue
                    _update_target_pose_from_bundle(objects, stage, event.bundle)
                    review_pos, _review_yaw = client.get_pose()
                    if _review_pending_above_queue(objects, path_stream, state, stage, review_pos):
                        continue

            # Poll again because Qwen may have finished while detector results were processed.
            _poll_plan(objects, stage=stage, frame=frame, down_frame=down_frame, state=state, client=client)

            if _small_above_confirmation_pending(objects, stage):
                # The first centered down-view hit already stopped the path.
                # Hold that pose until the next distinct observation arrives;
                # submitting another Qwen leg here would move away from the
                # target and make the two-frame completion contract flaky.
                path_stream.stop()
                objects.controller.clear()
                if (
                    objects.completion_pipeline is not None
                    and objects.controller.should_check_completion()
                    and not objects.completion_pipeline.active
                ):
                    submitted = objects.completion_pipeline.submit(
                        stage,
                        task_text,
                        None,
                        None,
                        stage_generation=_stage_generation(objects, stage),
                        lock_generation=_lock_generation(objects, stage),
                        session_id=_active_relocalization_session_id(objects, stage),
                        locked_instance_id=_locked_instance_id(objects, stage),
                    )
                    if submitted:
                        _debug_print("  [AboveSmall] waiting for distinct centered down-view confirmation")
                continue

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

            if (
                objects.completion_pipeline is not None
                and objects.controller.should_check_completion()
                and not objects.completion_pipeline.active
            ):
                submitted = objects.completion_pipeline.submit(
                    stage,
                    task_text,
                    None,
                    None,
                    stage_generation=_stage_generation(objects, stage),
                    lock_generation=_lock_generation(objects, stage),
                    session_id=_active_relocalization_session_id(objects, stage),
                    locked_instance_id=_locked_instance_id(objects, stage),
                )
                if submitted:
                    _debug_print("  [TargetObservation] submitted detector+depth snapshot")

            _poll_plan(objects, stage=stage, frame=frame, down_frame=down_frame, state=state, client=client)

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
