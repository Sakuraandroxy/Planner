"""Task-internal lightweight memory for UAV planning and completion."""

from __future__ import annotations

import math
import re
import time
from copy import copy
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence

from agent.functions.common.detection_policy import detection_reliability
from agent.functions.memory.appearance_signature import (
    appearance_similarity,
    build_appearance_signature,
    merge_prototype_set,
)
from agent.functions.memory.geometry import (
    bearing_yaw_deg,
    bbox_area_ratio,
    bbox_quality,
    bounds_from_points,
    distance3,
    distance_to_instance_geometry,
    estimate_detection_world,
    estimate_detection_surface_world,
    footprint_radius_from_detection,
    has_roof_geometry,
    has_surface_geometry,
    horizontal_distance,
    horizontal_distance_to_instance_roof,
    horizontal_distance_to_instance_surface_samples,
    has_surface_samples,
    nearest_instance_roof_point,
    nearest_instance_surface_point,
    nearest_instance_surface_sample_point,
    roof_interior_margin,
    world_to_body,
)
from agent.functions.memory.schemas import (
    MemoryCompletionDecision,
    PoseRecord,
    StageSummary,
    TargetInstanceBelief,
    TargetMemory,
)
from agent.functions.memory.spatial_reasoning import evaluate_memory_completion, relation_kind
from agent.functions.perception.bearing_tracker import (
    bbox_center_angle_deg,
    metric_depth_usable,
    signed_angle_delta_deg,
)
from agent.functions.perception.camera_geometry import triangulate_world_rays


_EN_ORDINALS = {
    "first": 1,
    "1st": 1,
    "second": 2,
    "2nd": 2,
    "third": 3,
    "3rd": 3,
    "fourth": 4,
    "4th": 4,
    "fifth": 5,
    "5th": 5,
}
_ZH_NUMERALS = {
    "一": 1,
    "二": 2,
    "两": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
    "七": 7,
    "八": 8,
    "九": 9,
}
_LARGE_STRUCTURE_TOKENS = (
    "building", "tower", "skyscraper", "warehouse", "hangar", "factory",
    "apartment", "office block", "高楼", "楼房", "建筑", "大厦", "塔",
    "仓库", "厂房",
)
_SMALL_TARGET_TOKENS = (
    "car", "automobile", "vehicle", "truck", "bus", "van", "fountain",
    "statue", "bench", "chair", "cone", "汽车", "轿车", "车辆", "卡车",
    "公交车", "面包车", "喷泉", "雕像", "长椅", "椅子", "路锥",
)


def _explicit_large_structure_target(stage: Any, detection: Any) -> bool:
    identity_text = " ".join((
        str(getattr(stage, "target", "") or ""),
        str(getattr(stage, "target_query", "") or ""),
        str(getattr(detection, "label", "") or ""),
    )).lower()
    if any(token in identity_text for token in _SMALL_TARGET_TOKENS):
        return False
    context_text = " ".join((
        identity_text,
        str(getattr(stage, "instruction", "") or ""),
        str(getattr(stage, "completion_condition", "") or ""),
    )).lower()
    return any(token in context_text for token in _LARGE_STRUCTURE_TOKENS)


def _large_structure_surface_lock_detection(
    detection: Any,
    image: Any,
    config: dict,
    depth_reason: str,
) -> Any:
    """Return a depth-safe facade observation from a mixed building bbox.

    GroundingDINO building boxes frequently include windows, sky and a rear
    building.  The ordinary metric gate must continue rejecting that mixed
    range as an object point, while ``above`` navigation still needs the
    coherent foreground facade to establish identity and climb before contact.
    """

    if not bool(config.get("LARGE_STRUCTURE_SURFACE_FALLBACK_ENABLED", True)):
        return None
    if str(getattr(detection, "camera", "front") or "front").strip().lower() != "front":
        return None
    if depth_reason not in {"mixed_bbox_depth", "mixed_center_depth", "too_noisy"}:
        return None
    samples = []
    max_depth = float(config.get("METRIC_LOCK_MAX_DEPTH_M", 120.0))
    for raw in list(getattr(detection, "surface_depth_samples", None) or []):
        if not isinstance(raw, (list, tuple)) or len(raw) < 3:
            continue
        try:
            u, v, depth = float(raw[0]), float(raw[1]), float(raw[2])
        except (TypeError, ValueError):
            continue
        if not all(math.isfinite(value) for value in (u, v, depth)):
            continue
        if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0 and 0.0 < depth <= max_depth):
            continue
        samples.append([u, v, depth])
    min_samples = max(4, int(config.get("LARGE_STRUCTURE_SURFACE_FALLBACK_MIN_SAMPLES", 6)))
    if len(samples) < min_samples:
        return None

    ordered_depths = sorted(float(sample[2]) for sample in samples)
    anchor = _median(ordered_depths)
    cluster_tolerance = max(
        float(config.get("LARGE_STRUCTURE_SURFACE_FALLBACK_MIN_TOLERANCE_M", 2.0)),
        anchor * float(config.get("LARGE_STRUCTURE_SURFACE_FALLBACK_CLUSTER_RATIO", 0.10)),
    )
    cluster = [sample for sample in samples if abs(float(sample[2]) - anchor) <= cluster_tolerance]
    if len(cluster) < min_samples:
        return None
    min_support_ratio = float(config.get("LARGE_STRUCTURE_SURFACE_FALLBACK_MIN_SUPPORT_RATIO", 0.35))
    if len(cluster) / max(float(len(samples)), 1.0) < min_support_ratio:
        return None

    depths = sorted(float(sample[2]) for sample in cluster)
    cluster_median = _median(depths)
    depth_p10 = _percentile(depths, 0.10)
    depth_p90 = _percentile(depths, 0.90)
    max_span = max(
        float(config.get("LARGE_STRUCTURE_SURFACE_FALLBACK_MIN_SPAN_M", 2.5)),
        cluster_median * float(config.get("LARGE_STRUCTURE_SURFACE_FALLBACK_MAX_SPAN_RATIO", 0.12)),
    )
    if depth_p90 - depth_p10 > max_span:
        return None

    bbox = list(getattr(detection, "bbox", None) or [])
    if len(bbox) < 4 or image is None or not hasattr(image, "size"):
        return None
    width, height = float(image.size[0]), float(image.size[1])
    if width <= 1.0 or height <= 1.0:
        return None
    bbox_u_span = max(1.0 / width, abs(float(bbox[2]) - float(bbox[0])) / width)
    bbox_v_span = max(1.0 / height, abs(float(bbox[3]) - float(bbox[1])) / height)
    u_values = [float(sample[0]) for sample in cluster]
    v_values = [float(sample[1]) for sample in cluster]
    coverage_u = (max(u_values) - min(u_values)) / bbox_u_span
    coverage_v = (max(v_values) - min(v_values)) / bbox_v_span
    min_extent = float(config.get("LARGE_STRUCTURE_SURFACE_FALLBACK_MIN_EXTENT_RATIO", 0.18))
    if coverage_u < min_extent or coverage_v < min_extent:
        return None

    projected = copy(detection)
    projected.depth_median = float(cluster_median)
    projected.depth_bbox_median = float(cluster_median)
    projected.depth_p10_m = float(depth_p10)
    projected.depth_p90_m = float(depth_p90)
    projected.depth_mad_m = _median([abs(depth - cluster_median) for depth in depths])
    projected.depth_valid_ratio = max(
        float(getattr(detection, "depth_valid_ratio", 0.0) or 0.0),
        len(cluster) / max(float(len(samples)), 1.0),
    )
    projected.depth_sample_count = len(cluster)
    projected.surface_depth_samples = [list(sample) for sample in cluster]
    projected.surface_anchor_uv = [_median(u_values), _median(v_values)]
    projected.surface_lock_fallback = True
    projected.surface_lock_depth_reason = str(depth_reason)
    return projected


def _median(values: Sequence[float]) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return float("nan")
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return float("nan")
    position = max(0.0, min(1.0, float(fraction))) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


@dataclass
class MemoryUpdateEvent:
    target_key: str
    instance_id: str
    kind: str
    confidence: float
    world: List[float]
    view: str
    score: float = 0.0
    reliability: float = 0.0
    observer_world: List[float] = field(default_factory=list)
    observer_yaw_deg: Optional[float] = None
    detection: Any = field(default=None, repr=False, compare=False)
    image: Any = field(default=None, repr=False, compare=False)

    def to_summary_dict(self) -> dict:
        return {
            "target": self.target_key,
            "instance": self.instance_id,
            "kind": self.kind,
            "confidence": round(float(self.confidence), 3),
            "world": [round(float(v), 2) for v in self.world[:3]],
            "view": self.view,
            "score": round(float(self.score), 3),
            "reliability": round(float(self.reliability), 3),
            "observer_world": [round(float(v), 2) for v in self.observer_world[:3]],
            "observer_yaw_deg": (
                None
                if self.observer_yaw_deg is None
                else round(float(self.observer_yaw_deg), 2)
            ),
        }


class MissionMemory:
    """Lightweight object memory shared across stages of one root task."""

    def __init__(self, config: Optional[dict] = None, sim_config: Optional[dict] = None):
        config = config or {}
        sim_config = sim_config or {}
        self.config = dict(config)
        self.sim_config = dict(sim_config)
        self.enabled = bool(self.config.get("ENABLED", True))
        self.max_target_memories = int(self.config.get("MAX_TARGET_MEMORIES", 24))
        self.max_instances_per_target = int(self.config.get("MAX_INSTANCES_PER_TARGET", 8))
        self.max_pose_records = int(self.config.get("MAX_POSE_RECORDS", 240))
        self.max_events = int(self.config.get("MAX_EVENTS", 40))
        self.root_instruction = ""
        # target_memories 保存每类目标的多实例记忆，例如 red car 下有第1/第2/第3辆。
        self.target_memories: Dict[str, TargetMemory] = {}
        # stage_locks 保证每个小任务锁定同一个物体实例，避免视野变化后把“第2辆”重新编号。
        self.stage_locks: Dict[str, str] = {}
        # 延迟绑定阶段只在激活时观测到的局部候选中编号，不复用全局同类目标顺序。
        self.stage_local_instances: Dict[str, List[str]] = {}
        # Save the stage activation frame so delayed detections keep the
        # original meaning of "left/right/front" after the UAV moves.
        self.stage_activation_views: Dict[str, dict] = {}
        self.pose_history: List[PoseRecord] = []
        self.stage_summaries: List[StageSummary] = []
        self.events: List[dict] = []
        self._switch_candidates: Dict[str, tuple[str, int]] = {}
        self.last_completion_decision: Optional[MemoryCompletionDecision] = None
        self._bearing_ray_tracks: Dict[str, List[dict]] = {}

    def reset(self, root_instruction: str = "") -> None:
        self.root_instruction = (root_instruction or "").strip()
        self.target_memories.clear()
        self.stage_locks.clear()
        self.stage_local_instances.clear()
        self.stage_activation_views.clear()
        self.pose_history.clear()
        self.stage_summaries.clear()
        self.events.clear()
        self._switch_candidates.clear()
        self.last_completion_decision = None
        self._bearing_ray_tracks.clear()

    def local_instance_ids(self, stage: Any) -> List[str]:
        """Return the activation-view candidates owned by one stage."""
        return list(self.stage_local_instances.get(stage_key(stage), []))

    def reset_view_relative_binding(self, stage: Any) -> List[str]:
        """Clear a stage-local binding without deleting global mission memory."""
        if not is_view_relative_stage(stage):
            return []
        key = stage_key(stage)
        cleared = list(self.stage_local_instances.pop(key, []))
        self.stage_activation_views.pop(key, None)
        locked_id = self.stage_locks.pop(key, "")
        target_key = normalize_target_key(target_name_for_stage(stage))
        memory = self.target_memories.get(target_key)
        if memory is not None and memory.primary_instance_id in set(cleared + [locked_id]):
            memory.primary_instance_id = ""
        self._switch_candidates.pop(key, None)
        self._append_event({
            "type": "view_relative_reset",
            "stage": key,
            "target": target_key,
            "cleared": cleared,
        })
        return cleared

    def begin_view_relative_binding(
        self,
        stage: Any,
        observer_world: Sequence[float],
        observer_yaw_deg: float,
    ) -> None:
        """Freeze the coordinate frame used by one delayed-binding stage."""
        if not is_view_relative_stage(stage):
            return
        key = stage_key(stage)
        self.stage_activation_views[key] = {
            "observer_world": [float(v) for v in observer_world[:3]],
            "observer_yaw_deg": float(observer_yaw_deg),
            "started_at": time.perf_counter(),
        }
        self._append_event({
            "type": "view_relative_activation",
            "stage": key,
            "observer_world": [round(float(v), 3) for v in observer_world[:3]],
            "observer_yaw_deg": round(float(observer_yaw_deg), 2),
        })

    def view_relative_binding_expired(self, stage: Any, current_world: Sequence[float]) -> tuple[bool, str]:
        """Bound exploratory delayed binding by both elapsed time and distance."""
        activation = self.stage_activation_views.get(stage_key(stage))
        if activation is None:
            return False, "activation view unavailable"
        elapsed = max(0.0, time.perf_counter() - float(activation.get("started_at", 0.0)))
        traveled = distance3(current_world, activation.get("observer_world", current_world))
        max_age = float(self.config.get("VIEW_RELATIVE_DELAY_BIND_MAX_AGE_S", 45.0))
        max_distance = float(self.config.get("VIEW_RELATIVE_DELAY_BIND_MAX_DISTANCE_M", 30.0))
        if elapsed > max_age:
            return True, f"delayed binding timed out after {elapsed:.1f}s"
        if traveled > max_distance:
            return True, f"delayed binding exceeded {traveled:.1f}m exploration limit"
        return False, f"waiting for delayed binding elapsed={elapsed:.1f}s traveled={traveled:.1f}m"

    def record_pose(self, stage: Any, position: Sequence[float], yaw_deg: float) -> None:
        if not self.enabled:
            return
        self.pose_history.append(
            PoseRecord(
                timestamp=time.perf_counter(),
                position=[float(v) for v in position[:3]],
                yaw_deg=float(yaw_deg),
                stage_key=stage_key(stage),
            )
        )
        del self.pose_history[: max(0, len(self.pose_history) - self.max_pose_records)]

    def update_from_detections(
        self,
        *,
        stage: Any,
        detections_by_view: Dict[str, Iterable[Any]],
        images_by_view: Dict[str, Any],
        observer_world: Sequence[float],
        observer_yaw_deg: float,
    ) -> List[MemoryUpdateEvent]:
        """Fuse depth-backed detections into the target instance memory."""
        if not self.enabled or stage is None or getattr(stage, "mode", "") not in {"target", "detect"}:
            return []
        target_name = target_name_for_stage(stage)
        if not target_name:
            return []
        target_key = normalize_target_key(target_name)
        if not target_key:
            return []
        memory = self._target_memory_for(stage, target_key, target_name)
        observations = self._collect_observations(
            stage=stage,
            detections_by_view=detections_by_view,
            images_by_view=images_by_view,
            observer_world=observer_world,
            observer_yaw_deg=observer_yaw_deg,
        )
        if not observations:
            self._decay_unseen_instances(memory)
            return []

        observations = self._deduplicate_frame_observations(observations)
        # 视角相对的“第一栋”表示激活视角指定扇区中实际距离最近的独立实体，
        # GroundingDINO 的语义分数只负责准入，不能决定空间序号。
        if is_view_relative_stage(stage):
            observations.sort(
                key=lambda obs: (
                    obs["distance_from_observer"],
                    obs["forward_projection"],
                    -float(obs["score"]),
                )
            )
        else:
            observations.sort(key=lambda obs: (obs["forward_projection"], obs["distance_from_observer"]))
        events: List[MemoryUpdateEvent] = []
        binding_frame_instances = set()
        binding_unlocked = bool(
            is_view_relative_stage(stage)
            and not self.stage_locks.get(stage_key(stage), "")
        )
        for obs in observations:
            instance, kind = self._match_or_create_instance(
                memory,
                obs,
                stage,
                exclude_instance_ids=binding_frame_instances if binding_unlocked else None,
            )
            if instance is None:
                # A view-relative identity is immutable after activation. A
                # same-class detection outside the locked geometry belongs to
                # another object and must not extend this stage's local set.
                continue
            self._update_instance(instance, obs, stage)
            binding_frame_instances.add(instance.instance_id)
            if is_view_relative_stage(stage) and binding_unlocked:
                local_ids = self.stage_local_instances.setdefault(stage_key(stage), [])
                if instance.instance_id not in local_ids:
                    local_ids.append(instance.instance_id)
            events.append(
                MemoryUpdateEvent(
                    target_key=target_key,
                    instance_id=instance.instance_id,
                    kind=kind,
                    confidence=instance.confidence,
                    world=list(obs["world"]),
                    view=str(obs["view"]),
                    score=float(obs["score"]),
                    reliability=float(obs["quality"]),
                    observer_world=list(obs.get("observer_world") or []),
                    observer_yaw_deg=obs.get("observer_yaw_deg"),
                    detection=obs.get("detection"),
                    image=obs.get("image"),
                )
            )
        lock_key = stage_key(stage)
        previous_lock = self.stage_locks.get(lock_key, "")
        self._ensure_stage_lock(stage, memory, observer_world, observer_yaw_deg)
        if self.stage_locks.get(lock_key, "") != previous_lock:
            # Unbound rays may contain several same-class candidates. A fresh
            # physical lock starts a clean ray track for that one instance.
            self._bearing_ray_tracks.pop(lock_key, None)
        self._prune_memory()
        for event in events:
            self._append_event(event.to_summary_dict())
        return events

    def estimate_distance(self, stage: Any, current_world: Sequence[float]):
        instance = self.primary_instance(stage)
        if instance is None:
            return None
        current = [float(v) for v in current_world[:3]]
        relation = relation_kind(stage)
        # Partial facades are reliable for near/beside distance, but only the
        # separately retained down-view plane is roof geometry for ``above``.
        surface_geometry = has_surface_geometry(instance) and relation == "near"
        roof_geometry = has_roof_geometry(instance) and relation == "above"
        nearest_surface = nearest_instance_surface_point(current, instance)
        nearest_roof = nearest_instance_roof_point(current, instance) if roof_geometry else None
        surface_sample_distance = (
            horizontal_distance_to_instance_surface_samples(current, instance)
            if surface_geometry
            else None
        )
        if surface_geometry:
            navigation_target = nearest_surface
        elif roof_geometry:
            navigation_target = nearest_roof
        elif relation == "above":
            navigation_target = [
                float(instance.target_world[0]),
                float(instance.target_world[1]),
                float(current[2]),
            ]
        else:
            navigation_target = instance.target_world
        horizontal_distance_m = (
            horizontal_distance_to_instance_roof(current, instance)
            if roof_geometry
            else horizontal_distance(current, navigation_target)
        )
        vertical_delta_m = abs(float(current[2]) - float(navigation_target[2]))
        return {
            "stage_key": stage_key_tuple(stage),
            "distance_m": (
                surface_sample_distance
                if surface_geometry
                else horizontal_distance_m
                if relation == "above"
                else distance3(current, instance.target_world)
            ),
            "target_world": list(navigation_target),
            "identity_anchor_world": list(instance.target_world),
            "nearest_surface_world": list(nearest_surface) if surface_geometry else None,
            "nearest_roof_world": list(nearest_roof) if roof_geometry else None,
            "distance_kind": "surface" if surface_geometry else "roof" if roof_geometry else "point",
            "distance_plane": "xy" if surface_geometry or relation == "above" else "3d",
            "horizontal_distance_m": horizontal_distance_m,
            "vertical_delta_m": vertical_delta_m,
            "roof_geometry": roof_geometry,
            "roof_confidence": float(getattr(instance, "roof_confidence", 0.0) or 0.0),
            "roof_observations": int(getattr(instance, "roof_observation_count", 0) or 0),
            "current_world": current,
            "observation_age_s": instance.age_s(),
            "source": "mission_memory",
            "confidence": float(instance.confidence),
            "uncertainty_m": instance.effective_uncertainty(
                stale_growth_per_s=float(self.config.get("STALE_UNCERTAINTY_GROWTH_MPS", 0.03))
            ),
            "instance_id": instance.instance_id,
        }

    def record_roof_plane(
        self,
        stage: Any,
        plane: Any,
        *,
        observer_world: Sequence[float],
    ) -> Optional[dict]:
        """Associate one full down-depth plane with the locked ``above`` target."""
        if (
            not self.enabled
            or relation_kind(stage) != "above"
            or plane is None
            or not bool(getattr(plane, "valid", False))
            or not self.is_primary_locked(stage)
        ):
            return None
        instance = self.primary_instance(stage)
        if instance is None or not bool(getattr(instance, "is_large_structure", False)):
            return None
        center_world = list(getattr(plane, "center_world", None) or [])
        roof_z = getattr(plane, "roof_z_world", None)
        if len(center_world) < 3 or roof_z is None:
            return None
        try:
            center_world = [float(value) for value in center_world[:3]]
            roof_z = float(roof_z)
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in center_world + [roof_z]):
            return None

        current = [float(value) for value in observer_world[:3]]
        clearance = roof_z - current[2]
        min_clearance = float(self.config.get("ABOVE_MIN_CLEARANCE_M", 0.3))
        max_sensor_clearance = float(self.config.get("ABOVE_ROOF_MAX_DEPTH_M", 120.0))
        if clearance < min_clearance or clearance > max_sensor_clearance:
            self._append_event({
                "type": "reject_roof_plane_clearance",
                "stage": stage_key(stage),
                "instance": instance.instance_id,
                "clearance_m": round(clearance, 2),
            })
            return None

        anchor_distance = horizontal_distance(center_world, instance.target_world)
        surface_distance = (
            horizontal_distance_to_instance_surface_samples(center_world, instance)
            if has_surface_samples(instance)
            else anchor_distance
        )
        roof_distance = (
            horizontal_distance_to_instance_roof(center_world, instance)
            if has_roof_geometry(instance)
            else None
        )
        association_distance = min(
            anchor_distance,
            surface_distance,
            roof_distance if roof_distance is not None else float("inf"),
        )
        # The first down-view roof must remain on the identity ray established
        # by the exact facade observation that created this instance. A broad
        # 25 m facade/roof radius alone can otherwise attach a neighbouring
        # building that happens to sit beside the locked one.
        identity_origin = list(getattr(instance, "identity_observer_world", None) or [])
        identity_target = list(getattr(instance, "identity_world", None) or instance.target_world)
        if len(identity_origin) >= 2 and len(identity_target) >= 2:
            ray_x = float(identity_target[0]) - float(identity_origin[0])
            ray_y = float(identity_target[1]) - float(identity_origin[1])
            ray_norm = math.hypot(ray_x, ray_y)
            if ray_norm > 1e-6:
                unit_x, unit_y = ray_x / ray_norm, ray_y / ray_norm
                roof_x = float(center_world[0]) - float(identity_origin[0])
                roof_y = float(center_world[1]) - float(identity_origin[1])
                roof_along = roof_x * unit_x + roof_y * unit_y
                roof_cross = abs(-roof_x * unit_y + roof_y * unit_x)
                corridor = (
                    float(self.config.get("ABOVE_ROOF_IDENTITY_CORRIDOR_HALF_WIDTH_M", 10.0))
                    + min(max(0.0, float(instance.uncertainty_m)), 2.0)
                    + min(max(0.0, float(instance.footprint_radius_m)) * 0.20, 4.0)
                )
                max_before = float(
                    self.config.get("ABOVE_ROOF_MAX_BEFORE_FACADE_M", 8.0)
                )
                max_beyond = max(
                    float(self.config.get("ABOVE_ROOF_MAX_BEYOND_FACADE_M", 24.0)),
                    min(32.0, 2.0 * max(1.0, float(instance.footprint_radius_m))),
                )
                if (
                    roof_cross > corridor
                    or roof_along < ray_norm - max_before
                    or roof_along > ray_norm + max_beyond
                ):
                    self._append_event({
                        "type": "reject_roof_plane_identity_corridor",
                        "stage": stage_key(stage),
                        "instance": instance.instance_id,
                        "cross_track_m": round(roof_cross, 2),
                        "corridor_m": round(corridor, 2),
                        "along_track_m": round(roof_along, 2),
                        "facade_range_m": round(ray_norm, 2),
                    })
                    return None
        association_radius = (
            max(1.0, float(instance.footprint_radius_m))
            + min(max(0.0, float(instance.uncertainty_m)), 5.0)
            + float(self.config.get("ABOVE_ROOF_ASSOCIATION_MARGIN_M", 6.0))
        )
        if roof_distance is None and bool(instance.is_large_structure):
            # The first nadir roof point can be near the centre of a wide roof
            # while the identity anchor lies on a facade.  Use the same bounded
            # large-structure scale as facade association for this first link;
            # later observations are gated against the retained roof itself.
            association_radius = max(
                association_radius,
                float(self.config.get("ABOVE_ROOF_LARGE_STRUCTURE_ASSOCIATION_M", 25.0)),
            )
        if association_distance > association_radius:
            self._append_event({
                "type": "reject_roof_plane_xy_mismatch",
                "stage": stage_key(stage),
                "instance": instance.instance_id,
                "distance_m": round(association_distance, 2),
                "radius_m": round(association_radius, 2),
            })
            return None

        # Ground under/behind a facade is lower in NED (larger Z).  A roof may
        # be well above the facade anchor, but must not be materially below it.
        max_below_anchor = float(self.config.get("ABOVE_ROOF_MAX_BELOW_ANCHOR_M", 3.0))
        if roof_z > float(instance.target_world[2]) + max_below_anchor:
            self._append_event({
                "type": "reject_roof_plane_below_anchor",
                "stage": stage_key(stage),
                "instance": instance.instance_id,
                "roof_z": round(roof_z, 2),
                "anchor_z": round(float(instance.target_world[2]), 2),
            })
            return None

        plane_confidence = float(getattr(plane, "confidence", 0.0) or 0.0)
        if plane_confidence < float(self.config.get("ABOVE_ROOF_MIN_CONFIDENCE", 0.45)):
            return None
        existing_z = getattr(instance, "roof_z_median", None)
        continuity = float(self.config.get("ABOVE_ROOF_Z_CONTINUITY_M", 2.5))
        if existing_z is not None and abs(float(existing_z) - roof_z) > continuity:
            self._append_event({
                "type": "reject_roof_plane_z_mismatch",
                "stage": stage_key(stage),
                "instance": instance.instance_id,
                "roof_z": round(roof_z, 2),
                "locked_roof_z": round(float(existing_z), 2),
            })
            return None

        alpha = min(0.45, max(0.18, 0.40 * plane_confidence))
        instance.roof_z_median = (
            roof_z
            if existing_z is None
            else (1.0 - alpha) * float(existing_z) + alpha * roof_z
        )
        z_mad = float(getattr(plane, "z_mad_m", 0.5) or 0.5)
        observation_uncertainty = max(0.15, z_mad + (1.0 - plane_confidence) * 1.5)
        if int(instance.roof_observation_count or 0) <= 0:
            instance.roof_uncertainty_m = observation_uncertainty
        else:
            instance.roof_uncertainty_m = max(
                0.12,
                (1.0 - alpha) * float(instance.roof_uncertainty_m) + alpha * observation_uncertainty,
            )
        instance.roof_confidence = min(
            0.99,
            1.0 - (1.0 - float(instance.roof_confidence)) * (1.0 - 0.65 * plane_confidence),
        )
        instance.roof_observation_count = int(instance.roof_observation_count or 0) + 1
        instance.last_roof_seen_s = time.perf_counter()
        instance.last_roof_full_frame = bool(getattr(plane, "full_frame", False))

        new_points = [
            [float(value) for value in point[:3]]
            for point in list(getattr(plane, "sample_points_world", None) or [])
            if point is not None and len(point) >= 3
        ]
        instance.roof_points_world.extend(new_points)
        max_points = max(8, int(self.config.get("ABOVE_ROOF_MEMORY_MAX_POINTS", 160)))
        if len(instance.roof_points_world) > max_points:
            # Retain points across the accumulated sequence instead of only
            # the newest image, so a large roof can grow while being crossed.
            step = max(1.0, len(instance.roof_points_world) / float(max_points))
            retained = []
            cursor = 0.0
            while int(cursor) < len(instance.roof_points_world) and len(retained) < max_points:
                retained.append(instance.roof_points_world[int(cursor)])
                cursor += step
            instance.roof_points_world = retained
        instance.roof_bounds_world = bounds_from_points(instance.roof_points_world)
        self._append_event({
            "type": "roof_plane_update",
            "stage": stage_key(stage),
            "instance": instance.instance_id,
            "roof_z": round(float(instance.roof_z_median), 2),
            "confidence": round(float(instance.roof_confidence), 3),
            "observations": int(instance.roof_observation_count),
            "full_frame": bool(instance.last_roof_full_frame),
        })
        return self.roof_navigation_context(stage, current)

    def navigation_context(
        self,
        stage: Any,
        current_world: Sequence[float],
        current_yaw_deg: float,
    ) -> dict:
        """Return camera-independent coordinates used by the planning prompt.

        Output waypoints are always expressed in the current horizontal
        navigation frame.  View-relative language keeps the origin/yaw frozen
        at stage activation so a later vehicle turn cannot change the meaning
        of "left", "right" or "front".
        """
        current = [float(value) for value in current_world[:3]]
        context = {
            "coordinate_convention": "AirSim NED; +x forward, +y right, +z down in navigation frame",
            "current_origin_world": current,
            "current_yaw_deg": float(current_yaw_deg),
        }
        activation = self.stage_activation_views.get(stage_key(stage))
        if activation is not None:
            activation_origin = [
                float(value)
                for value in list(activation.get("observer_world") or current)[:3]
            ]
            activation_yaw = float(activation.get("observer_yaw_deg", current_yaw_deg))
            context["stage_activation"] = {
                "origin_world": activation_origin,
                "yaw_deg": activation_yaw,
                "direction_words_use_this_yaw": True,
            }

        estimate = self.estimate_distance(stage, current)
        if estimate is None:
            return context
        target_world = list(estimate.get("target_world") or [])
        if len(target_world) < 3:
            return context
        context["target_world"] = [float(value) for value in target_world[:3]]
        context["target_nav_xyz"] = [
            round(float(value), 3)
            for value in world_to_body(target_world, current, current_yaw_deg)
        ]
        context["target_distance_m"] = float(estimate.get("distance_m", 0.0) or 0.0)
        context["target_confidence"] = float(estimate.get("confidence", 0.0) or 0.0)
        context["target_uncertainty_m"] = float(estimate.get("uncertainty_m", 0.0) or 0.0)
        if activation is not None:
            context["target_activation_nav_xyz"] = [
                round(float(value), 3)
                for value in world_to_body(
                    target_world,
                    context["stage_activation"]["origin_world"],
                    context["stage_activation"]["yaw_deg"],
                )
            ]
        return context

    def roof_navigation_context(
        self,
        stage: Any,
        current_world: Sequence[float],
    ) -> Optional[dict]:
        instance = self.primary_instance(stage)
        if instance is None or relation_kind(stage) != "above" or not has_roof_geometry(instance):
            return None
        current = [float(value) for value in current_world[:3]]
        nearest = nearest_instance_roof_point(current, instance)
        age_s = max(0.0, time.perf_counter() - float(instance.last_roof_seen_s or 0.0))
        min_observations = int(self.config.get("ABOVE_ROOF_MEMORY_MIN_OBSERVATIONS", 2))
        min_confidence = float(self.config.get("ABOVE_ROOF_MEMORY_MIN_CONFIDENCE", 0.60))
        max_age_s = float(self.config.get("ABOVE_ROOF_MAX_AGE_S", 20.0))
        trusted = bool(
            int(instance.roof_observation_count or 0) >= min_observations
            and float(instance.roof_confidence) >= min_confidence
            and age_s <= max_age_s
        )
        return {
            "instance_id": instance.instance_id,
            "trusted": trusted,
            "target_world": list(nearest),
            "roof_z_world": float(instance.roof_z_median),
            "horizontal_distance_m": horizontal_distance_to_instance_roof(current, instance),
            "clearance_m": float(instance.roof_z_median) - current[2],
            "interior_margin_m": roof_interior_margin(current, instance),
            "confidence": float(instance.roof_confidence),
            "uncertainty_m": float(instance.roof_uncertainty_m),
            "observations": int(instance.roof_observation_count),
            "age_s": age_s,
            "full_frame": bool(instance.last_roof_full_frame),
        }

    def above_overhead_context(
        self,
        stage: Any,
        current_world: Sequence[float],
    ) -> Optional[dict]:
        """Return the phase switch used to hand navigation from front to down."""
        instance = self.primary_instance(stage)
        if instance is None or relation_kind(stage) != "above" or not self.is_primary_locked(stage):
            return None
        roof = self.roof_navigation_context(stage, current_world)
        if roof is not None and bool(roof.get("trusted", False)):
            return {**roof, "active": True, "phase": "roof_confirmed"}
        current = [float(value) for value in current_world[:3]]
        horizontal = horizontal_distance(current, instance.target_world)
        uncertainty = instance.effective_uncertainty(
            stale_growth_per_s=float(self.config.get("STALE_UNCERTAINTY_GROWTH_MPS", 0.03))
        )
        radius = (
            max(1.0, float(instance.footprint_radius_m))
            + float(self.config.get("ABOVE_HORIZONTAL_RADIUS_M", 3.5))
            + min(max(0.0, uncertainty), float(self.config.get("MAX_COMPLETION_UNCERTAINTY_M", 5.0)))
        )
        return {
            "active": bool(horizontal <= radius),
            "phase": "verify" if horizontal <= radius else "approach",
            "instance_id": instance.instance_id,
            "horizontal_distance_m": horizontal,
            "trigger_radius_m": radius,
            "trusted": False,
        }

    def trusted_near_large_surface_estimate(
        self,
        stage: Any,
        current_world: Sequence[float],
    ) -> Optional[dict]:
        """Return a locked facade estimate that is safe to trust near a building."""
        if not self.enabled or not bool(
            self.config.get("LARGE_STRUCTURE_NEAR_MEMORY_TRUST_ENABLED", True)
        ):
            return None
        instance = self.primary_instance(stage)
        if instance is None:
            return None
        locked_id = self.stage_locks.get(stage_key(stage), "")
        if locked_id != instance.instance_id:
            return None
        if (
            relation_kind(stage) != "near"
            or not bool(instance.is_large_structure)
            or not has_surface_geometry(instance)
        ):
            return None
        if int(instance.surface_observation_count) < int(
            self.config.get("LARGE_STRUCTURE_NEAR_MEMORY_TRUST_MIN_OBSERVATIONS", 1)
        ):
            return None

        estimate = self.estimate_distance(stage, current_world)
        if estimate is None or str(estimate.get("distance_kind", "")) != "surface":
            return None
        trust_radius = max(
            float(self.config.get("SURFACE_NEAR_RADIUS_M", 6.0)),
            float(self.config.get("LARGE_STRUCTURE_NEAR_MEMORY_TRUST_RADIUS_M", 12.0)),
        )
        if float(estimate.get("distance_m", float("inf"))) > trust_radius:
            return None
        if float(estimate.get("confidence", 0.0)) < float(
            self.config.get(
                "LARGE_STRUCTURE_NEAR_MEMORY_TRUST_MIN_CONFIDENCE",
                self.config.get("LOCK_MIN_CONFIDENCE", 0.35),
            )
        ):
            return None
        if float(estimate.get("uncertainty_m", float("inf"))) > float(
            self.config.get("LARGE_STRUCTURE_NEAR_MEMORY_TRUST_MAX_UNCERTAINTY_M", 3.0)
        ):
            return None
        if float(estimate.get("observation_age_s", float("inf"))) > float(
            self.config.get("SURFACE_COMPLETION_MAX_AGE_S", 120.0)
        ):
            return None

        trusted = dict(estimate)
        trusted["trust_radius_m"] = trust_radius
        trusted["trust_reason"] = "locked_near_large_surface"
        return trusted

    def record_locked_large_surface_contact(
        self,
        stage: Any,
        *,
        observer_world: Sequence[float],
        observer_yaw_deg: float,
        obstacle_body: Sequence[float],
        contact_kind: str = "depth",
    ) -> Optional[dict]:
        """Fuse a near depth barrier into the currently locked building facade."""
        instance = self.primary_instance(stage)
        if (
            instance is None
            or self.stage_locks.get(stage_key(stage), "") != instance.instance_id
            or relation_kind(stage) != "near"
            or not bool(instance.is_large_structure)
            or not has_surface_geometry(instance)
            or obstacle_body is None
            or len(obstacle_body) < 3
        ):
            return None
        body = [float(obstacle_body[0]), float(obstacle_body[1]), float(obstacle_body[2])]
        contact_range = distance3([0.0, 0.0, 0.0], body)
        if body[0] <= 0.0 or contact_range > float(
            self.config.get("LARGE_STRUCTURE_DEPTH_CONTACT_MAX_RANGE_M", 8.0)
        ):
            return None

        yaw = math.radians(float(observer_yaw_deg))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        contact_world = [
            round(float(observer_world[0]) + cos_yaw * body[0] - sin_yaw * body[1], 4),
            round(float(observer_world[1]) + sin_yaw * body[0] + cos_yaw * body[1], 4),
            round(float(observer_world[2]) + body[2], 4),
        ]
        instance.surface_patches_world.append([contact_world])
        max_patches = max(1, int(self.config.get("MAX_SURFACE_PATCHES_PER_INSTANCE", 12)))
        del instance.surface_patches_world[:-max_patches]
        instance.surface_points_world = self._merge_surface_points(
            instance.surface_points_world,
            [contact_world],
        )
        instance.surface_bounds_world = bounds_from_points(instance.surface_points_world)
        instance.surface_observation_count += 1
        instance.observation_count += 1
        instance.last_seen_s = time.perf_counter()
        instance.last_surface_contact_s = instance.last_seen_s
        instance.last_surface_contact_kind = str(contact_kind or "depth")
        instance.confidence = max(
            float(instance.confidence),
            float(self.config.get("LARGE_STRUCTURE_DEPTH_CONTACT_MIN_CONFIDENCE", 0.38)),
        )
        instance.uncertainty_m = min(
            float(instance.uncertainty_m),
            float(self.config.get("LARGE_STRUCTURE_DEPTH_CONTACT_UNCERTAINTY_M", 1.5)),
        )
        instance.sigma_xy = round(max(0.35, instance.uncertainty_m * 0.75), 3)
        instance.sigma_z = round(max(0.25, instance.uncertainty_m * 0.45), 3)
        estimate = self.estimate_distance(stage, observer_world)
        if estimate is not None:
            estimate = dict(estimate)
            estimate["contact_world"] = contact_world
            estimate["contact_range_m"] = contact_range
        return estimate

    def recent_locked_large_surface_contact(self, stage: Any, *, max_age_s: Optional[float] = None) -> Optional[dict]:
        """Return a short-lived facade contact tied to this stage's locked instance."""
        instance = self.primary_instance(stage)
        if (
            instance is None
            or not self.is_primary_locked(stage)
            or not bool(instance.is_large_structure)
            or not has_surface_geometry(instance)
            or float(instance.last_surface_contact_s) <= 0.0
        ):
            return None
        limit = float(
            max_age_s
            if max_age_s is not None
            else self.config.get("LARGE_STRUCTURE_CONTACT_COMPLETION_MAX_AGE_S", 8.0)
        )
        age = max(0.0, time.perf_counter() - float(instance.last_surface_contact_s))
        if age > limit:
            return None
        return {
            "instance_id": instance.instance_id,
            "age_s": age,
            "kind": instance.last_surface_contact_kind,
        }

    def evaluate_completion(
        self,
        *,
        stage: Any,
        current_world: Sequence[float],
        fresh_visual_support: bool = False,
        visual_score: float = 0.0,
        stop_radius_m: float = 4.0,
    ) -> MemoryCompletionDecision:
        if not self.enabled:
            decision = MemoryCompletionDecision.not_complete(reason="memory disabled")
            self.last_completion_decision = decision
            return decision
        # Pure locked-surface memory may complete a large-building approach
        # when the facade has disappeared at close range.  If a fresh visual
        # claim exists, however, it must pass the normal identity/confidence
        # constraints below instead of bypassing them through this shortcut.
        large_surface_arrival = (
            None
            if fresh_visual_support or is_view_relative_stage(stage)
            else self.evaluate_large_surface_arrival(
                stage,
                current_world,
                radius_m=float(self.config.get("SURFACE_APPROACH_RADIUS_M", 4.5)),
            )
        )
        if large_surface_arrival is not None:
            return large_surface_arrival
        instance = self.primary_instance(stage)
        if (
            instance is not None
            and self.is_primary_locked(stage)
            and bool(instance.is_large_structure)
            and has_surface_samples(instance)
            and relation_kind(stage) == "near"
            and not fresh_visual_support
        ):
            # For large structures the direct XY surface rule is the complete
            # contract.  Do not fall back to SURFACE_NEAR_RADIUS_M (which may
            # be wider than 4.5m) and accidentally complete early.
            distance_m = horizontal_distance_to_instance_surface_samples(current_world, instance)
            nearest = nearest_instance_surface_sample_point(current_world, instance)
            radius = max(0.0, float(self.config.get("SURFACE_APPROACH_RADIUS_M", 4.5)))
            if distance_m > radius:
                decision = MemoryCompletionDecision.not_complete(
                    instance_id=instance.instance_id,
                    confidence=float(instance.confidence),
                    distance_m=float(distance_m),
                    horizontal_distance_m=float(distance_m),
                    vertical_delta_m=abs(float(current_world[2]) - float(nearest[2])),
                    target_world=list(nearest),
                    uncertainty_m=float(instance.effective_uncertainty(
                        stale_growth_per_s=float(self.config.get("STALE_UNCERTAINTY_GROWTH_MPS", 0.03))
                    )),
                    required_radius_m=radius,
                    reason="outside_large_surface_xy_radius",
                    details={
                        "geometry": instance.geometry_kind,
                        "large_structure": True,
                        "uses_surface_samples": True,
                        "xy_only": True,
                    },
                )
                self.last_completion_decision = decision
                self._append_event({"type": "completion_decision", **decision.to_summary_dict()})
                return decision
        trusted_surface = self.trusted_near_large_surface_estimate(stage, current_world)
        completion_config = self.config
        if trusted_surface is not None:
            # A facade can cease to look like a whole building at close range.
            # Once its stage-specific lock and surface geometry are stable,
            # detector confidence must not block geometrically valid arrival.
            completion_config = dict(self.config)
            trust_min_confidence = float(
                self.config.get(
                    "LARGE_STRUCTURE_NEAR_MEMORY_TRUST_MIN_CONFIDENCE",
                    self.config.get("LOCK_MIN_CONFIDENCE", 0.35),
                )
            )
            trust_min_observations = int(
                self.config.get("LARGE_STRUCTURE_NEAR_MEMORY_TRUST_MIN_OBSERVATIONS", 1)
            )
            completion_config["LARGE_STRUCTURE_MEMORY_ONLY_MIN_CONFIDENCE"] = trust_min_confidence
            completion_config["LARGE_STRUCTURE_MEMORY_ONLY_MIN_OBSERVATIONS"] = trust_min_observations
        decision = evaluate_memory_completion(
            stage=stage,
            instance=instance,
            current_world=current_world,
            config=completion_config,
            fresh_visual_support=fresh_visual_support,
            visual_score=visual_score,
            stop_radius_m=stop_radius_m,
        )
        if trusted_surface is not None:
            decision.details = dict(decision.details or {})
            decision.details["trusted_near_surface_memory"] = True
            decision.details["trust_radius_m"] = round(
                float(trusted_surface["trust_radius_m"]), 2
            )
        self.last_completion_decision = decision
        self._append_event({"type": "completion_decision", **decision.to_summary_dict()})
        return decision

    def evaluate_large_surface_arrival(
        self,
        stage: Any,
        current_world: Sequence[float],
        *,
        radius_m: float = 4.5,
    ) -> Optional[MemoryCompletionDecision]:
        """Complete a locked large target from observed surface XY geometry.

        Large structures commonly become partial facade detections at close
        range.  Once their identity and surface memory are locked, a fresh VLM
        judgment is not useful for deciding arrival.  This rule intentionally
        uses only the nearest retained surface sample in XY and ignores Z.
        Small objects continue through the normal detector/VLM completion path.
        """
        if not self.enabled:
            return None
        instance = self.primary_instance(stage)
        if (
            instance is None
            or not self.is_primary_locked(stage)
            or not bool(instance.is_large_structure)
            or not has_surface_samples(instance)
            or relation_kind(stage) != "near"
        ):
            return None
        distance_m = horizontal_distance_to_instance_surface_samples(current_world, instance)
        radius = max(0.0, float(radius_m))
        if distance_m > radius:
            return None
        nearest = nearest_instance_surface_sample_point(current_world, instance)
        decision = MemoryCompletionDecision.complete(
            instance_id=instance.instance_id,
            confidence=float(instance.confidence),
            distance_m=float(distance_m),
            horizontal_distance_m=float(distance_m),
            vertical_delta_m=abs(float(current_world[2]) - float(nearest[2])),
            target_world=list(nearest),
            uncertainty_m=float(instance.effective_uncertainty(
                stale_growth_per_s=float(self.config.get("STALE_UNCERTAINTY_GROWTH_MPS", 0.03))
            )),
            required_radius_m=radius,
            reason="large_surface_xy_radius_complete",
            details={
                "geometry": instance.geometry_kind,
                "large_structure": True,
                "uses_surface_samples": True,
                "xy_only": True,
                "nearest_surface_sample_world": [round(float(v), 2) for v in nearest[:3]],
            },
        )
        self.last_completion_decision = decision
        self._append_event({"type": "completion_decision", **decision.to_summary_dict()})
        return decision

    def archive_stage(self, stage: Any, reason: str = "") -> None:
        instance = self.primary_instance(stage)
        target_key = normalize_target_key(target_name_for_stage(stage))
        self.stage_summaries.append(
            StageSummary(
                stage_key=stage_key(stage),
                instruction=str(getattr(stage, "instruction", "") or ""),
                target_key=target_key,
                primary_instance_id=instance.instance_id if instance else "",
                reason=reason or "",
            )
        )
        if instance is not None:
            instance.status = "completed"

    def primary_instance(self, stage: Any) -> Optional[TargetInstanceBelief]:
        target_key = normalize_target_key(target_name_for_stage(stage))
        memory = self.target_memories.get(target_key)
        if memory is None:
            return None
        transition_instance = self._transition_instance_for_stage(stage, memory)
        if transition_instance is not None:
            return transition_instance
        ordinal = getattr(stage, "ordinal", None)
        locked_id = self.stage_locks.get(stage_key(stage), "")
        if locked_id:
            return memory.instances.get(locked_id)
        if is_view_relative_stage(stage):
            local_ids = self.local_instance_ids(stage)
            ordinal = max(1, int(ordinal or 1))
            if ordinal <= len(local_ids):
                return memory.instances.get(local_ids[ordinal - 1])
            # Never fall through to a mission-global building/car for a target
            # whose identity is defined by this stage's activation view.
            return None
        if ordinal:
            for instance in memory.instances.values():
                if int(instance.encounter_order) == int(ordinal):
                    return instance
            return None
        if memory.primary_instance_id:
            return memory.instances.get(memory.primary_instance_id)
        anchored = self._desired_instance_by_anchor(stage, memory.sorted_instances())
        if anchored is not None:
            return anchored
        instances = memory.sorted_instances()
        return instances[0] if instances else None

    def has_primary(self, stage: Any) -> bool:
        return self.primary_instance(stage) is not None

    def is_primary_locked(self, stage: Any) -> bool:
        instance = self.primary_instance(stage)
        return bool(
            instance is not None
            and self.stage_locks.get(stage_key(stage), "") == instance.instance_id
        )

    def preferred_yaw_deg(self, stage: Any, current_world: Sequence[float]) -> Optional[float]:
        instance = self.primary_instance(stage)
        if instance is None:
            return None
        relation = relation_kind(stage)
        if relation == "near" and has_surface_geometry(instance):
            target = nearest_instance_surface_point(current_world, instance)
        elif relation == "above" and has_roof_geometry(instance):
            target = nearest_instance_roof_point(current_world, instance)
        else:
            target = instance.target_world
        return bearing_yaw_deg(current_world, target)

    def planner_hint(
        self,
        *,
        stage: Any,
        current_world: Sequence[float],
        yaw_deg: float,
    ) -> str:
        if not self.enabled or not bool(self.config.get("PLANNER_MEMORY_HINT_ENABLED", True)):
            return ""
        transition_hint = self.transition_exclusion_hint(stage, current_world, yaw_deg)
        instance = self.primary_instance(stage)
        if instance is None:
            return transition_hint
        relation = relation_kind(stage)
        surface_geometry = has_surface_geometry(instance) and relation == "near"
        roof_geometry = has_roof_geometry(instance) and relation == "above"
        if surface_geometry:
            navigation_target = nearest_instance_surface_point(current_world, instance)
        elif roof_geometry:
            navigation_target = nearest_instance_roof_point(current_world, instance)
        elif relation == "above":
            # A front-view facade anchor identifies the building in XY, but its
            # Z coordinate is not a roof height.  Keep the planner's vertical
            # target at the current flight level until down depth confirms a
            # roof plane.  AirSim uses NED: positive body/world dz is descent.
            navigation_target = [
                float(instance.target_world[0]),
                float(instance.target_world[1]),
                float(current_world[2]),
            ]
        else:
            navigation_target = instance.target_world
        body = world_to_body(navigation_target, current_world, yaw_deg)
        anchor_hint = self._anchor_hint(stage, current_world, yaw_deg)
        above_hint = ""
        if relation == "above":
            if roof_geometry:
                roof = self.roof_navigation_context(stage, current_world) or {}
                above_hint = (
                    "Above-target vertical contract: use the retained roof geometry; "
                    f"roof_trusted={bool(roof.get('trusted', False))}, "
                    f"roof_clearance={float(roof.get('clearance_m', 0.0)):.1f}m. "
                    "In AirSim NED positive dz means descent. "
                )
            else:
                above_hint = (
                    "Above-target vertical contract: roof height is unknown. If the observed facade extends "
                    "above the UAV, climb vertically above the retained facade envelope before any XY crossing; "
                    "otherwise hold altitude while approaching, and never descend toward the facade anchor. "
                    "In AirSim NED positive dz means descent. "
                )
        return (
            "Memory hint: keep the locked target instance. "
            f"Target='{target_name_for_stage(stage)}', instance={instance.instance_id}, "
            f"encounter_order={instance.encounter_order}, relation={relation}, "
            f"navigation_anchor_body_xyz=[{body[0]:.1f},{body[1]:.1f},{body[2]:.1f}], "
            f"geometry={instance.geometry_kind}, "
            f"confidence={instance.confidence:.2f}, uncertainty={instance.uncertainty_m:.1f}m. "
            f"{above_hint}{anchor_hint}{transition_hint}"
            "Do not switch to another same-class object unless the locked instance is clearly impossible."
        )

    def previous_entity_exclusions(
        self,
        stage: Any,
        current_world: Sequence[float],
        yaw_deg: float,
    ) -> List[dict]:
        """Return a front-image bearing gate for the preceding entity."""
        previous = self._previous_completed_instance(stage)
        if previous is None or is_return_target_stage(stage) or is_same_target_stage(stage):
            return []
        target = (
            nearest_instance_surface_point(current_world, previous)
            if has_surface_geometry(previous)
            else previous.target_world
        )
        body = world_to_body(target, current_world, yaw_deg)
        horizontal_range = math.hypot(float(body[0]), float(body[1]))
        if horizontal_range <= 1e-6:
            tolerance = 35.0
        else:
            radius = (
                max(0.5, float(previous.footprint_radius_m))
                + min(max(0.0, float(previous.uncertainty_m)), 3.0)
                + float(self.config.get("PREVIOUS_ENTITY_BEARING_MARGIN_M", 1.5))
            )
            tolerance = math.degrees(math.atan2(radius, horizontal_range))
        return [{
            "instance_id": previous.instance_id,
            "bearing_deg": math.degrees(math.atan2(float(body[1]), float(body[0]))),
            "tolerance_deg": max(6.0, min(35.0, tolerance)),
            "body": [float(v) for v in body[:3]],
        }]

    def evaluate_locked_detection_identity(
        self,
        stage: Any,
        detection: Any,
        image: Any,
        *,
        observer_world: Sequence[float],
        observer_yaw_deg: float,
        view: str = "front",
    ) -> dict:
        """Apply one immutable identity gate before any runtime consumer.

        Metric observations must remain continuous with the locked world
        geometry.  RGB-only front observations must lie in the predicted
        bearing cone.  A down-view observation without metric depth cannot be
        associated with a world instance and is therefore not allowed to move
        the lock or the distance estimator.
        """
        if detection is None or not bool(getattr(detection, "visible", False)):
            return {"accepted": False, "reason": "not_visible"}
        instance = self.primary_instance(stage)
        if instance is None or not self.is_primary_locked(stage):
            return {"accepted": True, "reason": "stage_not_locked"}

        view_name = str(view or getattr(detection, "camera", "front") or "front").lower()
        camera_frame = getattr(detection, "camera_frame", None) or getattr(image, "camera_frame", None)
        optical_axis = list(getattr(camera_frame, "optical_axis_world", []) or [])
        camera_points_downward = bool(
            len(optical_axis) >= 3
            and float(optical_axis[2]) > 0.35
            and float(optical_axis[2]) > 0.5 * math.hypot(float(optical_axis[0]), float(optical_axis[1]))
        )
        current = [float(value) for value in observer_world[:3]]
        if relation_kind(stage) == "above" and not camera_points_downward:
            overhead = self.above_overhead_context(stage, current)
            roof_candidate = self.roof_navigation_context(stage, current)
            if (
                bool((overhead or {}).get("active", False))
                # XY proximity alone does not mean the UAV is above the roof.
                # Keep accepting the locked facade while climbing so a taller
                # upper wall can extend the vertical safety envelope.  Switch
                # to down-view-only identity only after a roof plane exists.
                and roof_candidate is not None
                and bool(self.config.get("ABOVE_OVERHEAD_SUPPRESS_FRONT_IDENTITY", True))
            ):
                return {
                    "accepted": False,
                    "reason": "above_overhead_down_view_primary",
                    "instance_id": instance.instance_id,
                }

        depth_usable, depth_reason = metric_depth_usable(detection, self.config)
        if depth_usable:
            observed_world = estimate_detection_world(
                detection,
                image,
                current,
                observer_yaw_deg,
                memory_config=self.config,
                sim_config=self.sim_config,
            )
            if observed_world is None:
                return {"accepted": False, "reason": "metric_projection_failed"}
            if relation_kind(stage) == "above" and camera_points_downward:
                if has_roof_geometry(instance):
                    distance_m = horizontal_distance_to_instance_roof(observed_world, instance)
                else:
                    distance_m = horizontal_distance(observed_world, instance.target_world)
                radius_m = (
                    max(1.0, float(instance.footprint_radius_m))
                    + min(max(0.0, float(instance.uncertainty_m)), 5.0)
                    + float(self.config.get("LOCKED_DOWN_ASSOCIATION_MARGIN_M", 6.0))
                )
            else:
                distance_m = distance_to_instance_geometry(observed_world, instance)
                radius_m = max(
                    float(self.config.get("MIN_ASSOCIATION_RADIUS_M", 2.0)),
                    float(instance.footprint_radius_m)
                    + min(max(0.0, float(instance.uncertainty_m)), 5.0)
                    + float(self.config.get("LOCKED_DETECTION_ASSOCIATION_MARGIN_M", 1.2)),
                )
                if bool(instance.is_large_structure):
                    radius_m = max(
                        radius_m,
                        float(self.config.get("LOCKED_LARGE_STRUCTURE_CONTINUITY_M", 8.0)),
                    )
                if is_view_relative_stage(stage) and has_surface_geometry(instance):
                    radius_m = min(
                        radius_m,
                        float(self.config.get("VIEW_RELATIVE_LOCKED_SURFACE_CONTINUITY_M", 8.0)),
                    )
            accepted = bool(distance_m <= radius_m)
            return {
                "accepted": accepted,
                "reason": "metric_geometry_continuity" if accepted else "metric_locked_geometry_mismatch",
                "instance_id": instance.instance_id,
                "observed_world": [float(value) for value in observed_world[:3]],
                "distance_m": float(distance_m),
                "radius_m": float(radius_m),
                "depth_state": depth_reason,
            }

        world_ray = getattr(detection, "world_ray", None)
        if world_ray is None and not view_name.startswith("front"):
            return {
                "accepted": False,
                "reason": f"{view_name}_without_metric_depth",
                "instance_id": instance.instance_id,
                "depth_state": depth_reason,
            }
        if world_ray is not None:
            direction = world_ray.direction_world
            if math.hypot(float(direction[0]), float(direction[1])) <= 1e-9:
                observed_angle = None
            else:
                observed_angle = signed_angle_delta_deg(
                    math.degrees(math.atan2(float(direction[1]), float(direction[0]))),
                    observer_yaw_deg,
                )
        else:
            intrinsics = getattr(camera_frame, "rgb_intrinsics", None)
            fov = (
                float(intrinsics.horizontal_fov_deg)
                if intrinsics is not None
                else float(self.config.get("FRONT_FOV_DEG", self.sim_config.get("FRONT_FOV", 90.0)))
            )
            observed_angle = bbox_center_angle_deg(getattr(detection, "bbox", None), image, fov)
        if observed_angle is None:
            return {"accepted": False, "reason": "rgb_bearing_unavailable"}
        target = (
            nearest_instance_roof_point(current, instance)
            if relation_kind(stage) == "above" and has_roof_geometry(instance)
            else nearest_instance_surface_point(current, instance)
            if has_surface_geometry(instance)
            else instance.target_world
        )
        body = world_to_body(target, current, observer_yaw_deg)
        expected_angle = math.degrees(math.atan2(float(body[1]), float(body[0])))
        horizontal_range = max(0.1, math.hypot(float(body[0]), float(body[1])))
        angular_radius = math.degrees(math.atan2(
            max(0.5, float(instance.footprint_radius_m))
            + float(self.config.get("LOCKED_BEARING_MARGIN_M", 2.0)),
            horizontal_range,
        ))
        tolerance = max(
            float(self.config.get("LOCKED_BEARING_MIN_TOLERANCE_DEG", 8.0)),
            min(
                float(self.config.get("LOCKED_BEARING_MAX_TOLERANCE_DEG", 32.0)),
                angular_radius,
            ),
        )
        delta = abs(signed_angle_delta_deg(observed_angle, expected_angle))
        accepted = bool(float(body[0]) > -max(1.0, float(instance.footprint_radius_m)) and delta <= tolerance)
        return {
            "accepted": accepted,
            "reason": "rgb_locked_bearing_cone" if accepted else "rgb_locked_bearing_mismatch",
            "instance_id": instance.instance_id,
            "observed_bearing_deg": float(observed_angle),
            "expected_bearing_deg": float(expected_angle),
            "delta_deg": float(delta),
            "tolerance_deg": float(tolerance),
            "depth_state": depth_reason,
        }

    def evaluate_relocalization_identity(
        self,
        stage: Any,
        detection: Any,
        image: Any,
        *,
        observer_world: Sequence[float],
        observer_yaw_deg: float,
        view: str = "front",
    ) -> dict:
        """Use geometry/bearing first and appearance only as a reliable veto.

        GroundingDINO confidence proves that a crop matches the target noun; it
        does not prove that it is the already locked physical instance.  This
        stricter gate is reserved for relocalization candidates, where a false
        positive would otherwise redirect the vehicle to another same-class
        object.
        """
        decision = self.evaluate_locked_detection_identity(
            stage,
            detection,
            image,
            observer_world=observer_world,
            observer_yaw_deg=observer_yaw_deg,
            view=view,
        )
        if not bool(decision.get("accepted", False)):
            return decision
        instance = self.primary_instance(stage)
        if (
            instance is None
            or not self.is_primary_locked(stage)
            or not bool(self.config.get("RELOCALIZATION_APPEARANCE_ENABLED", True))
            or not instance.appearance_prototypes
        ):
            return decision

        signature = build_appearance_signature(
            image,
            getattr(detection, "bbox", None),
            view=str(view or "front"),
        )
        if signature is None:
            return decision
        prototype_reliability = max(
            (float(getattr(proto, "reliability", 0.0) or 0.0) for proto in instance.appearance_prototypes),
            default=0.0,
        )
        joint_reliability = min(float(signature.reliability), prototype_reliability)
        similarity = appearance_similarity(signature, instance.appearance_prototypes)
        min_reliability = float(self.config.get("RELOCALIZATION_APPEARANCE_MIN_RELIABILITY", 0.45))
        min_similarity = float(self.config.get("RELOCALIZATION_APPEARANCE_MIN_SIMILARITY", 0.12))
        accepted = bool(joint_reliability < min_reliability or similarity >= min_similarity)
        enriched = dict(decision)
        enriched.update({
            "accepted": accepted,
            "reason": (
                "relocalization_identity_match"
                if accepted
                else "relocalization_appearance_mismatch"
            ),
            "appearance_similarity": float(similarity),
            "appearance_reliability": float(joint_reliability),
        })
        return enriched

    def transition_exclusion_hint(
        self,
        stage: Any,
        current_world: Sequence[float],
        yaw_deg: float,
    ) -> str:
        exclusions = self.previous_entity_exclusions(stage, current_world, yaw_deg)
        if not exclusions:
            return ""
        exclusion = exclusions[0]
        bearing = float(exclusion["bearing_deg"])
        side = "right" if bearing >= 0.0 else "left"
        return (
            "Transition constraint: the immediately preceding target "
            f"instance={exclusion['instance_id']} is about {abs(bearing):.0f} degrees to the {side}; "
            "do not select its remaining facade or visible fragment as this stage's new target. "
        )

    def candidate_context(
        self,
        *,
        stage: Any,
        current_world: Sequence[float],
        yaw_deg: float,
    ) -> dict:
        if not self.enabled or not bool(self.config.get("CANDIDATE_MEMORY_SCORING_ENABLED", True)):
            return {}
        target_key = normalize_target_key(target_name_for_stage(stage))
        memory = self.target_memories.get(target_key)
        instance = self.primary_instance(stage)
        if memory is None or instance is None:
            return {}
        relation = relation_kind(stage)
        surface_geometry = has_surface_geometry(instance) and relation == "near"
        roof_geometry = has_roof_geometry(instance) and relation == "above"
        roof_context = self.roof_navigation_context(stage, current_world) if roof_geometry else None
        if surface_geometry:
            navigation_target = nearest_instance_surface_point(current_world, instance)
        elif roof_geometry:
            navigation_target = nearest_instance_roof_point(current_world, instance)
        elif relation == "above":
            navigation_target = [
                float(instance.target_world[0]),
                float(instance.target_world[1]),
                float(current_world[2]),
            ]
        else:
            navigation_target = instance.target_world
        completion_radius = (
            float(self.config.get("SURFACE_NEAR_RADIUS_M", 6.0))
            if surface_geometry
            else float(self.config.get("NEAR_STANDOFF_M", 6.0))
        )
        non_primary = []
        for other in memory.instances.values():
            if other.instance_id == instance.instance_id:
                continue
            non_primary.append(world_to_body(other.target_world, current_world, yaw_deg))
        return {
            "enabled": True,
            "target_body": world_to_body(navigation_target, current_world, yaw_deg),
            "target_world": list(navigation_target),
            "identity_anchor_world": list(instance.target_world),
            "relation": relation,
            "completion_radius_m": completion_radius,
            "confidence": float(instance.confidence),
            "uncertainty_m": instance.effective_uncertainty(
                stale_growth_per_s=float(self.config.get("STALE_UNCERTAINTY_GROWTH_MPS", 0.03))
            ),
            # Once a surface is known, target_body already lies on the nearest
            # facade/body surface.  Do not add the whole object's radius again.
            "footprint_radius_m": (
                float(self.config.get("SURFACE_LOCAL_FOOTPRINT_RADIUS_M", 0.5))
                if surface_geometry
                else float(instance.footprint_radius_m)
            ),
            "uses_surface_geometry": surface_geometry,
            "uses_roof_geometry": roof_geometry,
            "roof_trusted": bool((roof_context or {}).get("trusted", False)),
            "roof_clearance_m": (roof_context or {}).get("clearance_m"),
            "geometry_kind": instance.geometry_kind,
            "is_large_structure": bool(instance.is_large_structure),
            "surface_bounds_world": (
                [list(bound) for bound in instance.surface_bounds_world]
                if instance.surface_bounds_world
                else None
            ),
            "non_primary_bodies": non_primary,
            "instance_id": instance.instance_id,
            "encounter_order": int(instance.encounter_order),
        }

    def _anchor_hint(self, stage: Any, current_world: Sequence[float], yaw_deg: float) -> str:
        parts: List[str] = []
        for key in auxiliary_target_keys(stage):
            memory = self.target_memories.get(key)
            anchor = memory.primary_instance() if memory is not None else None
            if anchor is None:
                continue
            body = world_to_body(anchor.target_world, current_world, yaw_deg)
            parts.append(f"{memory.target_name} body_xyz=[{body[0]:.1f},{body[1]:.1f},{body[2]:.1f}]")
        if not parts:
            return ""
        return "Anchor landmark memory: " + "; ".join(parts) + ". "

    def summary(self, stage: Any = None) -> dict:
        if not self.enabled:
            return {"enabled": False}
        stage_key_text = stage_key(stage) if stage is not None else ""
        memories = []
        for key, memory in self.target_memories.items():
            memories.append(memory.to_summary_dict(stage_lock=self.stage_locks.get(stage_key_text, "")))
        return {
            "enabled": True,
            "root_instruction": self.root_instruction,
            "active_stage_key": stage_key_text,
            "active_stage_local_instances": list(self.stage_local_instances.get(stage_key_text, [])),
            "targets": memories,
            "pose_records": len(self.pose_history),
            "stage_summaries": [
                {
                    "stage": s.stage_key,
                    "target": s.target_key,
                    "primary": s.primary_instance_id,
                    "reason": s.reason,
                }
                for s in self.stage_summaries[-5:]
            ],
            "last_completion": (
                self.last_completion_decision.to_summary_dict()
                if self.last_completion_decision is not None
                else None
            ),
            "events": list(self.events[-10:]),
        }

    def _target_memory_for(self, stage: Any, target_key: str, target_name: str) -> TargetMemory:
        memory = self.target_memories.get(target_key)
        ordinal = getattr(stage, "ordinal", None) or infer_ordinal(
            " ".join(
                [
                    str(getattr(stage, "instruction", "") or ""),
                    str(getattr(stage, "target", "") or ""),
                    str(getattr(stage, "completion_condition", "") or ""),
                ]
            )
        )
        selection_rule = str(getattr(stage, "selection_rule", "") or "").strip().lower()
        if not selection_rule:
            selection_rule = "ordinal" if ordinal else "stable"
        if memory is None:
            memory = TargetMemory(
                target_key=target_key,
                target_name=target_name,
                selection_rule=selection_rule,
                ordinal=int(ordinal) if ordinal else None,
            )
            self.target_memories[target_key] = memory
        elif ordinal and memory.ordinal is None:
            memory.ordinal = int(ordinal)
            memory.selection_rule = "ordinal"
        return memory

    def _record_rgb_world_ray(
        self,
        *,
        stage: Any,
        detection: Any,
        view_name: str,
    ) -> Optional[list[float]]:
        """Keep an RGB bearing and optionally triangulate a locked instance.

        Unbound rays are direction-only evidence and never create a physical
        target. This prevents two similar buildings from being cross-matched
        before the ordinal/identity lock has been established by metric depth.
        """
        ray = getattr(detection, "world_ray", None)
        if ray is None or not bool(self.config.get("WORLD_RAY_MEMORY_ENABLED", True)):
            return None
        key = stage_key(stage)
        now = time.perf_counter()
        max_age_s = max(0.5, float(self.config.get("WORLD_RAY_MAX_AGE_S", 20.0)))
        track = [
            entry
            for entry in self._bearing_ray_tracks.get(key, [])
            if now - float(entry.get("observed_at", 0.0)) <= max_age_s
        ]
        entry = {
            "ray": ray,
            "observed_at": now,
            "camera_id": str(getattr(detection, "camera_id", "") or getattr(ray, "camera_id", "")),
            "capture_id": str(getattr(detection, "capture_id", "") or getattr(ray, "capture_id", "")),
            "score": float(getattr(detection, "score", 0.0) or 0.0),
            "view": str(view_name or getattr(detection, "camera", "unknown")),
        }
        track.append(entry)
        max_rays = max(2, int(self.config.get("WORLD_RAY_TRACK_MAX", 12)))
        self._bearing_ray_tracks[key] = track[-max_rays:]

        instance = self.primary_instance(stage)
        if instance is not None and self.is_primary_locked(stage):
            instance.bearing_rays.append({
                "camera_id": entry["camera_id"],
                "capture_id": entry["capture_id"],
                "origin_world": [round(float(v), 4) for v in ray.origin_world[:3]],
                "direction_world": [round(float(v), 7) for v in ray.direction_world[:3]],
                "observed_at": now,
            })
            del instance.bearing_rays[:-max_rays]
        if (
            instance is None
            or not self.is_primary_locked(stage)
            or not bool(self.config.get("WORLD_RAY_TRIANGULATION_ENABLED", True))
        ):
            return None

        # Use rays from distinct captures. Same-capture RGB boxes from two
        # cameras are allowed; duplicate rays from one exact image are not.
        candidates = []
        seen = set()
        for candidate in reversed(track):
            identity = (candidate["camera_id"], candidate["capture_id"])
            if identity in seen:
                continue
            seen.add(identity)
            candidates.append(candidate["ray"])
            if len(candidates) >= max(2, int(self.config.get("WORLD_RAY_TRIANGULATION_MAX_RAYS", 4))):
                break
        result = triangulate_world_rays(
            candidates,
            minimum_baseline_m=float(self.config.get("WORLD_RAY_MIN_BASELINE_M", 1.0)),
            minimum_angle_deg=float(self.config.get("WORLD_RAY_MIN_ANGLE_DEG", 1.5)),
        )
        if result is None:
            return None
        world, residual_m = result
        if residual_m > float(self.config.get("WORLD_RAY_MAX_RESIDUAL_M", 3.0)):
            return None
        range_m = distance3(ray.origin_world, world)
        if not (
            float(self.config.get("WORLD_RAY_MIN_RANGE_M", 1.0))
            <= range_m
            <= float(self.config.get("WORLD_RAY_MAX_RANGE_M", 250.0))
        ):
            return None
        continuity_radius = max(
            float(self.config.get("WORLD_RAY_LOCK_CONTINUITY_M", 12.0)),
            float(instance.footprint_radius_m) + min(float(instance.uncertainty_m), 5.0) + 2.0,
        )
        if distance_to_instance_geometry(world, instance) > continuity_radius:
            return None
        detection.projection_source = "world_ray_triangulation"
        self._append_event({
            "type": "world_ray_triangulation",
            "stage": key,
            "instance": instance.instance_id,
            "camera_id": entry["camera_id"],
            "capture_id": entry["capture_id"],
            "residual_m": round(float(residual_m), 3),
            "world": [round(float(value), 3) for value in world[:3]],
        })
        return [float(value) for value in world[:3]]

    def _collect_observations(
        self,
        *,
        stage: Any,
        detections_by_view: Dict[str, Iterable[Any]],
        images_by_view: Dict[str, Any],
        observer_world: Sequence[float],
        observer_yaw_deg: float,
    ) -> List[dict]:
        observations: List[dict] = []
        yaw = math.radians(float(observer_yaw_deg))
        forward = [math.cos(yaw), math.sin(yaw)]
        for view_name, detections in (detections_by_view or {}).items():
            image = images_by_view.get(view_name)
            for detection in list(detections or []):
                if detection is None or not getattr(detection, "visible", False):
                    continue
                score = float(getattr(detection, "score", 0.0) or 0.0)
                if score < float(self.config.get("MIN_DETECTION_SCORE", 0.35)):
                    continue
                explicitly_large = _explicit_large_structure_target(stage, detection)
                quality = bbox_quality(detection, image)
                if quality <= 0.0 and explicitly_large:
                    # A facade that fills the front image is weak appearance
                    # evidence but can still provide a coherent target surface.
                    quality = detection_reliability(stage, detection, image)
                if quality <= 0.0:
                    continue
                projection_detection = detection
                depth_usable, depth_reason = metric_depth_usable(projection_detection, self.config)
                triangulated_world = None
                if depth_usable:
                    world = estimate_detection_world(
                        projection_detection,
                        image,
                        observer_world,
                        observer_yaw_deg,
                        memory_config=self.config,
                        sim_config=self.sim_config,
                        camera_frame=getattr(projection_detection, "camera_frame", None),
                    )
                    if world is None:
                        continue
                else:
                    projection_detection = (
                        _large_structure_surface_lock_detection(
                            detection,
                            image,
                            self.config,
                            depth_reason,
                        )
                        if explicitly_large
                        else None
                    )
                    if projection_detection is None:
                        triangulated_world = self._record_rgb_world_ray(
                            stage=stage,
                            detection=detection,
                            view_name=view_name,
                        )
                        if triangulated_world is None:
                            # Direction is retained, but an unbound or
                            # degenerate ray must not invent a 3-D target.
                            continue
                        projection_detection = detection
                        world = triangulated_world
                    else:
                        depth_usable, _surface_reason = metric_depth_usable(
                            projection_detection,
                            self.config,
                        )
                        if not depth_usable:
                            self._record_rgb_world_ray(
                                stage=stage,
                                detection=detection,
                                view_name=view_name,
                            )
                            continue
                        world = estimate_detection_world(
                            projection_detection,
                            image,
                            observer_world,
                            observer_yaw_deg,
                            memory_config=self.config,
                            sim_config=self.sim_config,
                            camera_frame=getattr(projection_detection, "camera_frame", None),
                        )
                        if world is None:
                            continue
                surface_world = (
                    []
                    if triangulated_world is not None
                    else estimate_detection_surface_world(
                        projection_detection,
                        image,
                        observer_world,
                        observer_yaw_deg,
                        memory_config=self.config,
                        sim_config=self.sim_config,
                        camera_frame=getattr(projection_detection, "camera_frame", None),
                    )
                )
                dx = world[0] - float(observer_world[0])
                dy = world[1] - float(observer_world[1])
                forward_projection = dx * forward[0] + dy * forward[1]
                lateral_projection = -dx * forward[1] + dy * forward[0]
                binding_observation = {
                    "view": str(view_name or getattr(detection, "camera", "unknown")),
                    "world": [float(v) for v in world[:3]],
                    "forward_projection": forward_projection,
                    "lateral_projection": lateral_projection,
                }
                if (
                    is_view_relative_stage(stage)
                    and not self.stage_locks.get(stage_key(stage), "")
                    and not self._view_relative_observation_allowed(stage, binding_observation)
                ):
                    continue
                signature = build_appearance_signature(
                    image,
                    getattr(projection_detection, "bbox", None),
                    view=str(view_name or getattr(detection, "camera", "unknown")),
                )
                footprint = footprint_radius_from_detection(
                    projection_detection,
                    image,
                    memory_config=self.config,
                    sim_config=self.sim_config,
                )
                bbox = list(getattr(projection_detection, "bbox", []) or [])
                bbox_span = 0.0
                if len(bbox) >= 4 and image is not None and hasattr(image, "size"):
                    width, height = float(image.size[0]), float(image.size[1])
                    if width > 1.0 and height > 1.0:
                        bbox_span = max(
                            max(0.0, float(bbox[2]) - float(bbox[0])) / width,
                            max(0.0, float(bbox[3]) - float(bbox[1])) / height,
                        )
                target_identity_text = " ".join((
                    str(getattr(stage, "target", "") or ""),
                    str(getattr(projection_detection, "label", "") or ""),
                )).lower()
                target_text = " ".join((
                    target_identity_text,
                    str(getattr(stage, "instruction", "") or ""),
                )).lower()
                explicitly_small = any(
                    token in target_identity_text for token in _SMALL_TARGET_TOKENS
                )
                is_large_structure = (
                    not explicitly_small
                    and (
                        footprint >= float(self.config.get("LARGE_STRUCTURE_MIN_FOOTPRINT_M", 4.5))
                        or explicitly_large
                        or any(token in target_text for token in _LARGE_STRUCTURE_TOKENS)
                    )
                )
                observations.append(
                    {
                        "world": [float(v) for v in world[:3]],
                        "detection": projection_detection,
                        "image": image,
                        "view": str(view_name or getattr(detection, "camera", "unknown")),
                        "score": score,
                        "quality": quality,
                        "area_ratio": bbox_area_ratio(detection, image),
                        "depth": getattr(projection_detection, "depth_median", None),
                        "footprint": footprint,
                        "surface_world": surface_world,
                        "bbox_span": bbox_span,
                        "is_large_structure": is_large_structure,
                        "signature": signature,
                        "distance_from_observer": distance3(observer_world, world),
                        "forward_projection": forward_projection,
                        "lateral_projection": lateral_projection,
                        "observer_world": [float(v) for v in observer_world[:3]],
                        "observer_yaw_deg": float(observer_yaw_deg),
                        "camera_id": str(getattr(projection_detection, "camera_id", "") or ""),
                        "capture_id": str(getattr(projection_detection, "capture_id", "") or ""),
                        "optical_axis_world": list(
                            getattr(
                                getattr(projection_detection, "camera_frame", None),
                                "optical_axis_world",
                                [],
                            )
                            or []
                        )[:3],
                        "projection_source": str(getattr(projection_detection, "projection_source", "legacy") or "legacy"),
                    }
                )
        return observations

    def _deduplicate_frame_observations(self, observations: List[dict]) -> List[dict]:
        """Collapse multiple detector boxes for one depth-backed entity.

        GroundingDINO commonly emits nested ``building``/``facade`` boxes for
        the same wall. During delayed ordinal binding those duplicates must not
        become building #1 and building #2. Overlapping boxes at materially
        different depths are deliberately retained as separate entities.
        """

        if len(observations) <= 1:
            return list(observations)
        iou_threshold = float(self.config.get("FRAME_DUPLICATE_BBOX_IOU", 0.62))
        containment_threshold = float(
            self.config.get("FRAME_DUPLICATE_BBOX_CONTAINMENT", 0.82)
        )
        max_world_distance = float(
            self.config.get("FRAME_DUPLICATE_MAX_WORLD_DISTANCE_M", 5.0)
        )
        max_depth_ratio = float(
            self.config.get("FRAME_DUPLICATE_MAX_DEPTH_RATIO", 0.12)
        )
        retained: List[dict] = []
        ranked = sorted(
            observations,
            key=lambda obs: (
                float(obs.get("quality", 0.0) or 0.0),
                float(obs.get("score", 0.0) or 0.0),
                -float(getattr(obs.get("detection"), "depth_mad_m", 0.0) or 0.0),
            ),
            reverse=True,
        )
        for candidate in ranked:
            duplicate = False
            for accepted in retained:
                if str(candidate.get("view", "")) != str(accepted.get("view", "")):
                    continue
                overlap, containment = _bbox_overlap_statistics(
                    getattr(candidate.get("detection"), "bbox", None),
                    getattr(accepted.get("detection"), "bbox", None),
                )
                if overlap < iou_threshold and containment < containment_threshold:
                    continue
                candidate_depth = float(candidate.get("depth", 0.0) or 0.0)
                accepted_depth = float(accepted.get("depth", 0.0) or 0.0)
                depth_limit = max(
                    2.0,
                    min(candidate_depth, accepted_depth) * max_depth_ratio,
                )
                if abs(candidate_depth - accepted_depth) > depth_limit:
                    continue
                if distance3(candidate["world"], accepted["world"]) > max_world_distance:
                    continue
                duplicate = True
                self._append_event({
                    "type": "deduplicate_frame_detection",
                    "kept_bbox": list(getattr(accepted.get("detection"), "bbox", []) or []),
                    "dropped_bbox": list(getattr(candidate.get("detection"), "bbox", []) or []),
                    "depth_m": round(candidate_depth, 2),
                })
                break
            if not duplicate:
                retained.append(candidate)
        return retained

    def _view_relative_observation_allowed(self, stage: Any, observation: dict) -> bool:
        """Apply the activation-view direction before a local instance exists."""
        forward = float(observation.get("forward_projection", 0.0) or 0.0)
        lateral = float(observation.get("lateral_projection", 0.0) or 0.0)
        activation = self.stage_activation_views.get(stage_key(stage))
        world = observation.get("world") or []
        if activation is not None and len(world) >= 3:
            activation_body = world_to_body(
                world,
                activation.get("observer_world", [0.0, 0.0, 0.0]),
                float(activation.get("observer_yaw_deg", 0.0) or 0.0),
            )
            forward = float(activation_body[0])
            lateral = float(activation_body[1])
        min_forward = float(self.config.get("VIEW_RELATIVE_MIN_FORWARD_M", 0.5))
        max_behind = max(0.0, float(self.config.get("VIEW_RELATIVE_MAX_BEHIND_M", 1.0)))
        lateral_margin = max(0.0, float(self.config.get("VIEW_RELATIVE_LATERAL_MARGIN_M", 0.5)))
        direction = view_relative_direction(stage)
        if direction == "front_left":
            return forward >= min_forward and lateral <= -lateral_margin
        if direction == "front_right":
            return forward >= min_forward and lateral >= lateral_margin
        if direction == "left":
            return forward >= -max_behind and lateral <= -lateral_margin
        if direction == "right":
            return forward >= -max_behind and lateral >= lateral_margin
        return forward >= min_forward

    def _match_or_create_instance(
        self,
        memory: TargetMemory,
        obs: dict,
        stage: Any,
        *,
        exclude_instance_ids: Optional[set[str]] = None,
    ):
        previous = self._previous_completed_instance(stage)
        if (
            previous is not None
            and not is_return_target_stage(stage)
            and not is_same_target_stage(stage)
            and self._observation_matches_previous(obs, previous)
        ):
            self._append_event({
                "type": "reject_previous_entity",
                "stage": stage_key(stage),
                "instance": previous.instance_id,
            })
            return None, "reject_previous_entity"

        best = None
        best_score = -1.0
        excluded_ids = exclude_instance_ids or set()
        local_ids = set(self.local_instance_ids(stage)) if is_view_relative_stage(stage) else None
        locked_id = self.stage_locks.get(stage_key(stage), "")
        if local_ids is not None and locked_id:
            local_ids = {locked_id}
        for instance in memory.instances.values():
            if instance.instance_id in excluded_ids:
                continue
            if local_ids is not None and instance.instance_id not in local_ids:
                continue
            geo_dist = distance_to_instance_geometry(obs["world"], instance)
            assoc_radius = max(
                float(self.config.get("MIN_ASSOCIATION_RADIUS_M", 2.0)),
                float(instance.footprint_radius_m) + float(instance.uncertainty_m) + 1.2,
            )
            if bool(instance.is_large_structure) or bool(obs.get("is_large_structure", False)):
                # Observations of the same facade can be many metres apart;
                # compare against the accumulated surface and use a wider gate.
                assoc_radius = max(
                    assoc_radius,
                    float(self.config.get("LARGE_STRUCTURE_ASSOCIATION_RADIUS_M", 25.0)),
                )
            if (
                locked_id
                and is_view_relative_stage(stage)
                and instance.instance_id == locked_id
                and has_surface_geometry(instance)
            ):
                # Locked facades grow through overlapping observations. Do not
                # jump across a street/gap merely because both detections have
                # the generic class "building".
                assoc_radius = min(
                    assoc_radius,
                    float(self.config.get("VIEW_RELATIVE_LOCKED_SURFACE_CONTINUITY_M", 8.0)),
                )
            if geo_dist > assoc_radius:
                continue
            geo_score = 1.0 - min(1.0, geo_dist / max(assoc_radius, 1e-6))
            app_score = appearance_similarity(obs.get("signature"), instance.appearance_prototypes)
            reliability = float(getattr(obs.get("signature"), "reliability", 0.0) or 0.0)
            # 几何始终是主证据；颜色/纹理只作为弱辅助，避免光照变化导致换身份。
            combined = 0.78 * geo_score + 0.22 * reliability * app_score
            if combined > best_score:
                best_score = combined
                best = instance
        if best is not None and best_score >= float(self.config.get("ASSOCIATION_MIN_SCORE", 0.28)):
            return best, "update"
        if locked_id and is_view_relative_stage(stage):
            # Never create a replacement for an activation-time lock. Near a
            # building, the detector commonly finds a complete but distant
            # building while the locked facade only appears as a partial wall.
            return None, "reject_locked_mismatch"
        instance_id = f"{memory.target_key}:{memory.next_encounter_order}"
        memory.next_encounter_order += 1
        instance = TargetInstanceBelief(
            instance_id=instance_id,
            encounter_order=memory.next_encounter_order - 1,
            target_world=[round(float(v), 4) for v in obs["world"]],
            confidence=max(0.05, min(0.95, 0.55 * float(obs["score"]) + 0.45 * float(obs["quality"]))),
            observation_count=0,
            sigma_xy=2.5,
            sigma_z=1.2,
            uncertainty_m=self._initial_uncertainty(obs),
            footprint_radius_m=float(obs["footprint"]),
            surface_points_world=[],
            surface_observation_count=0,
            geometry_kind="point",
            is_large_structure=bool(obs.get("is_large_structure", False)),
            last_bbox_span=float(obs.get("bbox_span", 0.0) or 0.0),
            depth_median=None if obs["depth"] is None else float(obs["depth"]),
            bbox_quality=float(obs["quality"]),
            last_seen_view=str(obs["view"]),
            last_seen_optical_axis_world=[
                float(value) for value in list(obs.get("optical_axis_world") or [])[:3]
            ],
            last_label=str(getattr(obs["detection"], "label", "") or ""),
            identity_observer_world=list(obs.get("observer_world") or []),
            identity_observer_yaw_deg=obs.get("observer_yaw_deg"),
            identity_bbox=list(getattr(obs["detection"], "bbox", []) or []),
            identity_score=float(obs["score"]),
            identity_reliability=float(obs["quality"]),
            identity_depth_median=(
                None if obs.get("depth") is None else float(obs["depth"])
            ),
            identity_world=[float(v) for v in obs["world"][:3]],
            identity_view=str(obs["view"]),
            identity_label=str(getattr(obs["detection"], "label", "") or ""),
            observed_camera_ids=([str(obs.get("camera_id"))] if obs.get("camera_id") else []),
            observed_capture_ids=([str(obs.get("capture_id"))] if obs.get("capture_id") else []),
        )
        memory.instances[instance_id] = instance
        return instance, "create"

    def _previous_completed_instance(self, stage: Any) -> Optional[TargetInstanceBelief]:
        target_key = normalize_target_key(target_name_for_stage(stage))
        memory = self.target_memories.get(target_key)
        if memory is None:
            return None
        current_key = stage_key(stage)
        for summary in reversed(self.stage_summaries):
            if summary.stage_key == current_key or summary.target_key != target_key:
                continue
            instance = memory.instances.get(summary.primary_instance_id)
            if instance is not None:
                return instance
        return None

    def _transition_instance_for_stage(
        self,
        stage: Any,
        memory: TargetMemory,
    ) -> Optional[TargetInstanceBelief]:
        if is_same_target_stage(stage):
            return self._previous_completed_instance(stage)
        if not is_return_target_stage(stage):
            return None
        ordinal = getattr(stage, "ordinal", None)
        if ordinal:
            for instance in memory.instances.values():
                if int(instance.encounter_order) == int(ordinal):
                    return instance
        return self._previous_completed_instance(stage)

    def _observation_matches_previous(self, obs: dict, instance: TargetInstanceBelief) -> bool:
        geo_dist = distance_to_instance_geometry(obs["world"], instance)
        radius = max(
            float(self.config.get("MIN_ASSOCIATION_RADIUS_M", 2.0)),
            float(instance.footprint_radius_m) + min(float(instance.uncertainty_m), 3.0) + 1.2,
        )
        if bool(instance.is_large_structure) or bool(obs.get("is_large_structure", False)):
            radius = max(radius, float(self.config.get("PREVIOUS_ENTITY_EXCLUSION_RADIUS_M", 8.0)))
            radius = min(radius, float(self.config.get("PREVIOUS_ENTITY_EXCLUSION_MAX_RADIUS_M", 10.0)))
        return geo_dist <= radius

    def _update_instance(self, instance: TargetInstanceBelief, obs: dict, stage: Any) -> None:
        now = time.perf_counter()
        obs_quality = max(0.0, min(1.0, 0.55 * float(obs["score"]) + 0.45 * float(obs["quality"])))
        alpha = max(0.18, min(0.65, obs_quality))
        if instance.observation_count <= 0:
            alpha = 1.0
        # target_world 仍作为实例关联/旧接口锚点；导航完成优先使用不被
        # 多视角平均到物体内部的 surface_points_world。
        instance.target_world = [
            round((1.0 - alpha) * float(instance.target_world[i]) + alpha * float(obs["world"][i]), 4)
            for i in range(3)
        ]
        instance.observation_count += 1
        instance.last_seen_s = now
        instance.status = "active"
        instance.confidence = max(
            instance.confidence,
            min(0.99, 0.72 * float(instance.confidence) + 0.38 * obs_quality),
        )
        new_uncertainty = self._initial_uncertainty(obs) / math.sqrt(max(1.0, instance.observation_count * 0.7))
        instance.uncertainty_m = round(max(0.45, min(float(instance.uncertainty_m), new_uncertainty)), 3)
        instance.sigma_xy = round(max(0.35, instance.uncertainty_m * 0.75), 3)
        instance.sigma_z = round(max(0.25, instance.uncertainty_m * 0.45), 3)
        instance.footprint_radius_m = round(
            max(0.5, 0.70 * float(instance.footprint_radius_m) + 0.30 * float(obs["footprint"])),
            3,
        )
        new_surface = list(obs.get("surface_world") or [])
        if new_surface:
            patch = self._merge_surface_points([], new_surface)
            if patch:
                max_patches = max(1, int(self.config.get("MAX_SURFACE_PATCHES_PER_INSTANCE", 12)))
                instance.surface_patches_world.append(patch)
                del instance.surface_patches_world[:-max_patches]
            instance.surface_points_world = self._merge_surface_points(
                instance.surface_points_world,
                new_surface,
            )
            instance.surface_bounds_world = bounds_from_points(instance.surface_points_world)
            instance.surface_observation_count += 1
            instance.geometry_kind = "large_surface" if (
                instance.is_large_structure or bool(obs.get("is_large_structure", False))
            ) else "surface_bounds"
        instance.is_large_structure = bool(
            instance.is_large_structure or obs.get("is_large_structure", False)
        )
        instance.last_bbox_span = max(
            float(instance.last_bbox_span),
            float(obs.get("bbox_span", 0.0) or 0.0),
        )
        instance.depth_median = None if obs["depth"] is None else float(obs["depth"])
        instance.bbox_quality = float(obs["quality"])
        instance.last_seen_view = str(obs["view"])
        instance.last_seen_optical_axis_world = [
            float(value) for value in list(obs.get("optical_axis_world") or [])[:3]
        ]
        instance.last_label = str(getattr(obs["detection"], "label", "") or instance.last_label)
        camera_id = str(obs.get("camera_id", "") or "")
        capture_id = str(obs.get("capture_id", "") or "")
        if camera_id and camera_id not in instance.observed_camera_ids:
            instance.observed_camera_ids.append(camera_id)
            del instance.observed_camera_ids[:-8]
        if capture_id and capture_id not in instance.observed_capture_ids:
            instance.observed_capture_ids.append(capture_id)
            del instance.observed_capture_ids[:-12]
        key = stage_key(stage)
        if key and key not in instance.observed_stage_keys:
            instance.observed_stage_keys.append(key)
            del instance.observed_stage_keys[:-8]
        merge_prototype_set(
            instance.appearance_prototypes,
            obs.get("signature"),
            max_prototypes=int(self.config.get("MAX_APPEARANCE_PROTOTYPES", 5)),
            merge_threshold=float(self.config.get("APPEARANCE_MERGE_THRESHOLD", 0.78)),
        )

    def _merge_surface_points(
        self,
        existing: Sequence[Sequence[float]],
        observed: Sequence[Sequence[float]],
    ) -> List[List[float]]:
        """Voxel-deduplicate and cap per-instance surface memory."""
        resolution = max(0.1, float(self.config.get("SURFACE_VOXEL_SIZE_M", 0.75)))
        max_points = max(8, int(self.config.get("MAX_SURFACE_POINTS_PER_INSTANCE", 96)))
        cells: Dict[tuple[int, int, int], List[float]] = {}
        for point in list(existing or []) + list(observed or []):
            if point is None or len(point) < 3:
                continue
            value = [float(point[0]), float(point[1]), float(point[2])]
            if not all(math.isfinite(v) for v in value):
                continue
            key = tuple(int(round(v / resolution)) for v in value)
            cells[key] = [round(v, 4) for v in value]
        points = list(cells.values())
        if len(points) <= max_points:
            return points

        # Preserve geometric extrema, then sample evenly from the remaining
        # voxels.  This keeps facade/vehicle bounds stable under long missions.
        keep_indices = set()
        for axis in range(3):
            keep_indices.add(min(range(len(points)), key=lambda i: points[i][axis]))
            keep_indices.add(max(range(len(points)), key=lambda i: points[i][axis]))
        remaining = [i for i in range(len(points)) if i not in keep_indices]
        slots = max(0, max_points - len(keep_indices))
        if slots and remaining:
            stride = len(remaining) / float(slots)
            for index in range(slots):
                keep_indices.add(remaining[min(len(remaining) - 1, int(index * stride))])
        return [points[index] for index in sorted(keep_indices)[:max_points]]

    def _initial_uncertainty(self, obs: dict) -> float:
        depth = obs.get("depth")
        depth = float(depth) if depth is not None else 30.0
        area = max(float(obs.get("area_ratio", 0.0) or 0.0), 1e-6)
        quality = max(float(obs.get("quality", 0.0) or 0.0), 0.05)
        depth_term = depth * float(self.config.get("DEPTH_UNCERTAINTY_RATIO", 0.06))
        small_box_term = min(5.0, 0.08 / math.sqrt(area))
        quality_term = (1.0 - quality) * 2.0
        return round(max(0.7, min(8.0, depth_term + small_box_term + quality_term)), 3)

    def _ensure_stage_lock(
        self,
        stage: Any,
        memory: TargetMemory,
        current_world: Sequence[float],
        current_yaw_deg: Optional[float] = None,
    ) -> None:
        if not bool(self.config.get("INSTANCE_LOCK_ENABLED", True)):
            return
        key = stage_key(stage)
        current_lock = self.stage_locks.get(key, "")
        if current_lock and current_lock in memory.instances:
            if is_view_relative_stage(stage):
                return
            desired = self._desired_instance_for_stage(stage, memory, current_world)
            if desired is not None and desired.instance_id != current_lock:
                if auxiliary_target_keys(stage) and bool(self.config.get("ANCHOR_SWITCH_ENABLED", True)):
                    current = memory.instances.get(current_lock)
                    if current is None or desired.confidence >= current.confidence - 0.20:
                        # 带锚点的目标（如灌木旁的红车）允许被锚点关系纠正一次锁定结果。
                        self.stage_locks[key] = desired.instance_id
                        memory.primary_instance_id = desired.instance_id
                        self._append_event({
                            "type": "anchor_lock_switch",
                            "stage": key,
                            "target": memory.target_key,
                            "from": current_lock,
                            "to": desired.instance_id,
                        })
                        return
                self._maybe_switch_lock(key, memory, desired)
            return

        desired = self._desired_instance_for_stage(stage, memory, current_world)
        if desired is None:
            return
        min_lock_confidence = float(self.config.get("LOCK_MIN_CONFIDENCE", 0.35))
        if (
            is_view_relative_stage(stage)
            and bool(desired.is_large_structure)
            and has_surface_geometry(desired)
        ):
            # The activation view already resolves an ordinal target within
            # the local candidate list. Multiple buildings in that view do
            # not make the selected ordinal ambiguous.
            min_lock_confidence = min(
                min_lock_confidence,
                float(self.config.get("LARGE_STRUCTURE_SURFACE_LOCK_MIN_CONFIDENCE", 0.15)),
            )
        if desired.confidence < min_lock_confidence:
            return
        self.stage_locks[key] = desired.instance_id
        memory.primary_instance_id = desired.instance_id
        self._append_event({
            "type": "lock",
            "stage": key,
            "target": memory.target_key,
            "instance": desired.instance_id,
            "order": desired.encounter_order,
        })

    def _desired_instance_for_stage(
        self,
        stage: Any,
        memory: TargetMemory,
        current_world: Sequence[float],
    ) -> Optional[TargetInstanceBelief]:
        transition_instance = self._transition_instance_for_stage(stage, memory)
        if transition_instance is not None:
            return transition_instance
        if is_view_relative_stage(stage):
            local_instances = [
                memory.instances[instance_id]
                for instance_id in self.local_instance_ids(stage)
                if instance_id in memory.instances
            ]
            if not local_instances:
                return None
            ordinal = max(1, int(getattr(stage, "ordinal", None) or 1))
            return local_instances[ordinal - 1] if ordinal <= len(local_instances) else None
        ordinal = getattr(stage, "ordinal", None)
        if ordinal:
            for instance in memory.instances.values():
                if int(instance.encounter_order) == int(ordinal):
                    return instance
            return None
        rule = str(getattr(stage, "selection_rule", "") or memory.selection_rule or "stable").strip().lower()
        instances = memory.sorted_instances()
        if not instances:
            return None
        anchored = self._desired_instance_by_anchor(stage, instances)
        if anchored is not None and rule in {"anchored", "stable", "nearest"}:
            return anchored
        if rule in {"nearest", "closest"}:
            return min(instances, key=lambda inst: distance_to_instance_geometry(current_world, inst))
        if bool(self.config.get("LOCK_ON_FIRST_STABLE_INSTANCE", True)):
            stable = [
                inst for inst in instances
                if inst.confidence >= float(self.config.get("LOCK_MIN_CONFIDENCE", 0.35))
            ]
            if stable:
                return stable[0]
        return max(instances, key=lambda inst: (inst.confidence, inst.observation_count))

    def _desired_instance_by_anchor(
        self,
        stage: Any,
        instances: List[TargetInstanceBelief],
    ) -> Optional[TargetInstanceBelief]:
        anchor_instances: List[TargetInstanceBelief] = []
        for key in auxiliary_target_keys(stage):
            memory = self.target_memories.get(key)
            if memory is None:
                continue
            anchor_instances.extend(memory.instances.values())
        if not anchor_instances:
            return None
        max_anchor_distance = float(self.config.get("ANCHOR_MAX_DISTANCE_M", 25.0))
        best = None
        best_score = -1e9
        for instance in instances:
            nearest_anchor = min(horizontal_distance(instance.target_world, anchor.target_world) for anchor in anchor_instances)
            distance_score = 1.0 - min(nearest_anchor / max(max_anchor_distance, 1e-6), 1.5)
            score = 0.72 * distance_score + 0.28 * float(instance.confidence)
            if score > best_score:
                best_score = score
                best = instance
        if best is None:
            return None
        return best

    def _maybe_switch_lock(self, stage_key_text: str, memory: TargetMemory, desired: TargetInstanceBelief) -> None:
        current_id = self.stage_locks.get(stage_key_text, "")
        current = memory.instances.get(current_id)
        if current is None:
            self.stage_locks[stage_key_text] = desired.instance_id
            memory.primary_instance_id = desired.instance_id
            return
        margin = float(self.config.get("INSTANCE_SWITCH_MARGIN", 0.25))
        if desired.confidence < current.confidence + margin:
            return
        cand_id, count = self._switch_candidates.get(stage_key_text, ("", 0))
        count = count + 1 if cand_id == desired.instance_id else 1
        self._switch_candidates[stage_key_text] = (desired.instance_id, count)
        if count >= int(self.config.get("INSTANCE_SWITCH_CONFIRMATIONS", 2)):
            self.stage_locks[stage_key_text] = desired.instance_id
            memory.primary_instance_id = desired.instance_id
            self._append_event({
                "type": "lock_switch",
                "stage": stage_key_text,
                "target": memory.target_key,
                "from": current_id,
                "to": desired.instance_id,
            })

    def _decay_unseen_instances(self, memory: TargetMemory) -> None:
        now = time.perf_counter()
        for instance in memory.instances.values():
            age = now - float(instance.last_seen_s)
            if age > float(self.config.get("TARGET_MAX_AGE_S", 60.0)):
                instance.status = "stale"

    def _prune_memory(self) -> None:
        if len(self.target_memories) > self.max_target_memories:
            items = sorted(
                self.target_memories.items(),
                key=lambda item: max((inst.last_seen_s for inst in item[1].instances.values()), default=0.0),
            )
            for key, _memory in items[: len(self.target_memories) - self.max_target_memories]:
                self.target_memories.pop(key, None)
        for memory in self.target_memories.values():
            if len(memory.instances) <= self.max_instances_per_target:
                continue
            instances = sorted(
                memory.instances.values(),
                key=lambda inst: (inst.confidence, inst.last_seen_s),
            )
            for instance in instances[: len(memory.instances) - self.max_instances_per_target]:
                if instance.instance_id == memory.primary_instance_id:
                    continue
                memory.instances.pop(instance.instance_id, None)

    def _append_event(self, event: dict) -> None:
        event = dict(event or {})
        event.setdefault("time", round(time.perf_counter(), 3))
        self.events.append(event)
        del self.events[: max(0, len(self.events) - self.max_events)]


def _bbox_overlap_statistics(first: Any, second: Any) -> tuple[float, float]:
    """Return (IoU, intersection/smaller-area) for two xyxy boxes."""

    a = list(first or [])
    b = list(second or [])
    if len(a) < 4 or len(b) < 4:
        return 0.0, 0.0
    ax1, ay1, ax2, ay2 = [float(value) for value in a[:4]]
    bx1, by1, bx2, by2 = [float(value) for value in b[:4]]
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    if area_a <= 0.0 or area_b <= 0.0:
        return 0.0, 0.0
    intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(
        0.0,
        min(ay2, by2) - max(ay1, by1),
    )
    union = area_a + area_b - intersection
    return (
        intersection / max(union, 1e-9),
        intersection / max(min(area_a, area_b), 1e-9),
    )


def target_name_for_stage(stage: Any) -> str:
    return (
        str(getattr(stage, "target_query", "") or "")
        or str(getattr(stage, "target", "") or "")
        or str(getattr(stage, "instruction", "") or "")
    ).strip()


def is_return_target_stage(stage: Any) -> bool:
    if bool(getattr(stage, "return_target", False)):
        return True
    text = _stage_direction_text(stage)
    return bool(
        re.search(r"\b(?:fly|go|come|head|navigate)?\s*back\s+to\b", text)
        or re.search(r"\breturn\s+to\b", text)
        or any(
            token in text
            for token in (
                "previously visited",
                "visited before",
                "previously passed",
                "飞回",
                "返回",
                "回到",
                "回去",
                "之前经过",
                "先前经过",
            )
        )
    )


def is_same_target_stage(stage: Any) -> bool:
    if stage is None or is_return_target_stage(stage):
        return False
    if bool(getattr(stage, "same_target", False)):
        return True
    text = " ".join((
        str(getattr(stage, "instruction", "") or ""),
        str(getattr(stage, "completion_condition", "") or ""),
    )).lower()
    return bool(
        re.search(r"\b(?:the\s+)?same\s+(?:target|object|building|car|vehicle|tower)\b", text)
        or any(token in text for token in ("同一栋", "同一个", "同一座", "该建筑", "这栋楼", "它的上方", "到它旁边"))
    )


def is_view_relative_stage(stage: Any) -> bool:
    if stage is None or is_return_target_stage(stage):
        return False
    if bool(getattr(stage, "view_relative", False)):
        return True
    rule = str(getattr(stage, "selection_rule", "") or "").strip().lower()
    return rule in {"view_relative", "viewpoint", "current_view", "viewpoint_ordinal"}


def view_relative_direction(stage: Any) -> str:
    text = _stage_direction_text(stage)
    compact = text.replace(" ", "")
    has_front = any(token in text for token in ("ahead", "in front", "forward")) or "前方" in compact
    has_left = any(token in text for token in ("on the left", "to the left", "left side")) or any(
        token in compact for token in ("左侧", "左边")
    )
    has_right = any(token in text for token in ("on the right", "to the right", "right side")) or any(
        token in compact for token in ("右侧", "右边")
    )
    if (
        any(token in text for token in ("front left", "left front"))
        or "左前方" in compact
        or (has_front and has_left)
    ):
        return "front_left"
    if (
        any(token in text for token in ("front right", "right front"))
        or "右前方" in compact
        or (has_front and has_right)
    ):
        return "front_right"
    if has_left:
        return "left"
    if has_right:
        return "right"
    return "front"


def _stage_direction_text(stage: Any) -> str:
    text = " ".join((
        str(getattr(stage, "instruction", "") or ""),
        str(getattr(stage, "completion_condition", "") or ""),
    )).lower()
    return re.sub(r"[-_/]+", " ", text)


def normalize_target_key(target: str) -> str:
    text = clean_target_text(target)
    text = re.sub(r"\b(the|a|an)\b", " ", text, flags=re.IGNORECASE)
    text = re.sub(r"[^0-9a-zA-Z\u4e00-\u9fff]+", " ", text)
    text = re.sub(r"\s+", " ", text).strip().lower()
    return text


def clean_target_text(target: str) -> str:
    text = str(target or "").strip()
    match = re.search(
        r"\b(.+?)\s+(?:near|beside|next to|by|adjacent to|close to|in front of|behind)\s+",
        text,
        flags=re.IGNORECASE,
    )
    if match:
        text = match.group(1).strip()
    text = re.sub(r"\b(first|second|third|fourth|fifth|\d+(?:st|nd|rd|th))\b", " ", text, flags=re.IGNORECASE)
    text = re.sub(r"第\s*[0-9一二两三四五六七八九]+\s*[个辆台座只架]?", " ", text)
    text = re.sub(r"\s+", " ", text)
    return text.strip()


def auxiliary_target_keys(stage: Any) -> List[str]:
    keys: List[str] = []
    for target in list(getattr(stage, "auxiliary_targets", []) or []):
        key = normalize_target_key(str(target or ""))
        if key and key not in keys:
            keys.append(key)
    return keys


def infer_ordinal(text: str) -> Optional[int]:
    lowered = str(text or "").lower()
    for token, value in _EN_ORDINALS.items():
        if re.search(rf"\b{re.escape(token)}\b", lowered):
            return value
    zh = re.search(r"第\s*([0-9一二两三四五六七八九]+)", lowered)
    if zh:
        value = zh.group(1)
        if value.isdigit():
            return int(value)
        return _ZH_NUMERALS.get(value)
    en_num = re.search(r"\b([1-9])(?:st|nd|rd|th)\b", lowered)
    if en_num:
        return int(en_num.group(1))
    return None


def selection_rule_for_text(text: str, ordinal: Optional[int] = None) -> str:
    lowered = str(text or "").lower()
    if ordinal:
        return "ordinal"
    if any(token in lowered for token in ("nearest", "closest", "最近", "最近的")):
        return "nearest"
    return "stable"


def stage_key(stage: Any) -> str:
    if stage is None:
        return ""
    return "|".join(
        [
            str(getattr(stage, "index", "")),
            str(getattr(stage, "instruction", "") or ""),
            str(getattr(stage, "mode", "") or ""),
        ]
    )


def stage_key_tuple(stage: Any) -> tuple:
    return (
        getattr(stage, "index", None),
        getattr(stage, "instruction", None),
        getattr(stage, "mode", None),
    )
