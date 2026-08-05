"""Task-internal lightweight memory for UAV planning and completion."""

from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence

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
    has_surface_geometry,
    horizontal_distance,
    nearest_instance_surface_point,
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


@dataclass
class MemoryUpdateEvent:
    target_key: str
    instance_id: str
    kind: str
    confidence: float
    world: List[float]
    view: str

    def to_summary_dict(self) -> dict:
        return {
            "target": self.target_key,
            "instance": self.instance_id,
            "kind": self.kind,
            "confidence": round(float(self.confidence), 3),
            "world": [round(float(v), 2) for v in self.world[:3]],
            "view": self.view,
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
        self.pose_history: List[PoseRecord] = []
        self.stage_summaries: List[StageSummary] = []
        self.events: List[dict] = []
        self._switch_candidates: Dict[str, tuple[str, int]] = {}
        self.last_completion_decision: Optional[MemoryCompletionDecision] = None

    def reset(self, root_instruction: str = "") -> None:
        self.root_instruction = (root_instruction or "").strip()
        self.target_memories.clear()
        self.stage_locks.clear()
        self.pose_history.clear()
        self.stage_summaries.clear()
        self.events.clear()
        self._switch_candidates.clear()
        self.last_completion_decision = None

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

        # 同一帧内先按无人机前进方向排序，新的实例 encounter_order 才更接近“路上遇到的第N个”。
        observations.sort(key=lambda obs: (obs["forward_projection"], obs["distance_from_observer"]))
        events: List[MemoryUpdateEvent] = []
        for obs in observations:
            instance, kind = self._match_or_create_instance(memory, obs, stage)
            self._update_instance(instance, obs, stage)
            events.append(
                MemoryUpdateEvent(
                    target_key=target_key,
                    instance_id=instance.instance_id,
                    kind=kind,
                    confidence=instance.confidence,
                    world=list(instance.target_world),
                    view=instance.last_seen_view,
                )
            )
        self._ensure_stage_lock(stage, memory, observer_world)
        self._prune_memory()
        for event in events:
            self._append_event(event.to_summary_dict())
        return events

    def estimate_distance(self, stage: Any, current_world: Sequence[float]):
        instance = self.primary_instance(stage)
        if instance is None:
            return None
        current = [float(v) for v in current_world[:3]]
        # Partial facades are reliable for near/beside distance, but not yet a
        # complete roof footprint for "above" semantics.
        surface_geometry = has_surface_geometry(instance) and relation_kind(stage) == "near"
        nearest_surface = nearest_instance_surface_point(current, instance)
        navigation_target = nearest_surface if surface_geometry else instance.target_world
        return {
            "stage_key": stage_key_tuple(stage),
            "distance_m": (
                distance_to_instance_geometry(current, instance)
                if surface_geometry
                else distance3(current, instance.target_world)
            ),
            "target_world": list(navigation_target),
            "identity_anchor_world": list(instance.target_world),
            "nearest_surface_world": list(nearest_surface) if surface_geometry else None,
            "distance_kind": "surface" if surface_geometry else "point",
            "current_world": current,
            "observation_age_s": instance.age_s(),
            "source": "mission_memory",
            "confidence": float(instance.confidence),
            "uncertainty_m": instance.effective_uncertainty(
                stale_growth_per_s=float(self.config.get("STALE_UNCERTAINTY_GROWTH_MPS", 0.03))
            ),
            "instance_id": instance.instance_id,
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
        decision = evaluate_memory_completion(
            stage=stage,
            instance=self.primary_instance(stage),
            current_world=current_world,
            config=self.config,
            fresh_visual_support=fresh_visual_support,
            visual_score=visual_score,
            stop_radius_m=stop_radius_m,
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
        ordinal = getattr(stage, "ordinal", None)
        locked_id = self.stage_locks.get(stage_key(stage), "")
        if locked_id:
            return memory.instances.get(locked_id)
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

    def preferred_yaw_deg(self, stage: Any, current_world: Sequence[float]) -> Optional[float]:
        instance = self.primary_instance(stage)
        if instance is None:
            return None
        target = (
            nearest_instance_surface_point(current_world, instance)
            if has_surface_geometry(instance) and relation_kind(stage) == "near"
            else instance.target_world
        )
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
        instance = self.primary_instance(stage)
        if instance is None:
            return ""
        relation = relation_kind(stage)
        surface_geometry = has_surface_geometry(instance) and relation == "near"
        navigation_target = (
            nearest_instance_surface_point(current_world, instance)
            if surface_geometry
            else instance.target_world
        )
        body = world_to_body(navigation_target, current_world, yaw_deg)
        anchor_hint = self._anchor_hint(stage, current_world, yaw_deg)
        return (
            "Memory hint: keep the locked target instance. "
            f"Target='{target_name_for_stage(stage)}', instance={instance.instance_id}, "
            f"encounter_order={instance.encounter_order}, relation={relation}, "
            f"navigation_anchor_body_xyz=[{body[0]:.1f},{body[1]:.1f},{body[2]:.1f}], "
            f"geometry={instance.geometry_kind}, "
            f"confidence={instance.confidence:.2f}, uncertainty={instance.uncertainty_m:.1f}m. "
            f"{anchor_hint}"
            "Do not switch to another same-class object unless the locked instance is clearly impossible."
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
        navigation_target = (
            nearest_instance_surface_point(current_world, instance)
            if surface_geometry
            else instance.target_world
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
                quality = bbox_quality(detection, image)
                if quality <= 0.0:
                    continue
                world = estimate_detection_world(
                    detection,
                    image,
                    observer_world,
                    observer_yaw_deg,
                    memory_config=self.config,
                    sim_config=self.sim_config,
                )
                if world is None:
                    continue
                surface_world = estimate_detection_surface_world(
                    detection,
                    image,
                    observer_world,
                    observer_yaw_deg,
                    memory_config=self.config,
                    sim_config=self.sim_config,
                )
                score = float(getattr(detection, "score", 0.0) or 0.0)
                if score < float(self.config.get("MIN_DETECTION_SCORE", 0.35)):
                    continue
                dx = world[0] - float(observer_world[0])
                dy = world[1] - float(observer_world[1])
                signature = build_appearance_signature(
                    image,
                    getattr(detection, "bbox", None),
                    view=str(view_name or getattr(detection, "camera", "unknown")),
                )
                footprint = footprint_radius_from_detection(
                    detection,
                    image,
                    memory_config=self.config,
                    sim_config=self.sim_config,
                )
                bbox = list(getattr(detection, "bbox", []) or [])
                bbox_span = 0.0
                if len(bbox) >= 4 and image is not None and hasattr(image, "size"):
                    width, height = float(image.size[0]), float(image.size[1])
                    if width > 1.0 and height > 1.0:
                        bbox_span = max(
                            max(0.0, float(bbox[2]) - float(bbox[0])) / width,
                            max(0.0, float(bbox[3]) - float(bbox[1])) / height,
                        )
                target_text = " ".join((
                    str(getattr(stage, "target", "") or ""),
                    str(getattr(stage, "instruction", "") or ""),
                    str(getattr(detection, "label", "") or ""),
                )).lower()
                is_large_structure = (
                    footprint >= float(self.config.get("LARGE_STRUCTURE_MIN_FOOTPRINT_M", 4.5))
                    or any(token in target_text for token in _LARGE_STRUCTURE_TOKENS)
                )
                observations.append(
                    {
                        "world": [float(v) for v in world[:3]],
                        "detection": detection,
                        "image": image,
                        "view": str(view_name or getattr(detection, "camera", "unknown")),
                        "score": score,
                        "quality": quality,
                        "area_ratio": bbox_area_ratio(detection, image),
                        "depth": getattr(detection, "depth_median", None),
                        "footprint": footprint,
                        "surface_world": surface_world,
                        "bbox_span": bbox_span,
                        "is_large_structure": is_large_structure,
                        "signature": signature,
                        "distance_from_observer": distance3(observer_world, world),
                        "forward_projection": dx * forward[0] + dy * forward[1],
                    }
                )
        return observations

    def _match_or_create_instance(self, memory: TargetMemory, obs: dict, stage: Any):
        best = None
        best_score = -1.0
        for instance in memory.instances.values():
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
            last_label=str(getattr(obs["detection"], "label", "") or ""),
        )
        memory.instances[instance_id] = instance
        return instance, "create"

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
        instance.last_label = str(getattr(obs["detection"], "label", "") or instance.last_label)
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

    def _ensure_stage_lock(self, stage: Any, memory: TargetMemory, current_world: Sequence[float]) -> None:
        if not bool(self.config.get("INSTANCE_LOCK_ENABLED", True)):
            return
        key = stage_key(stage)
        current_lock = self.stage_locks.get(key, "")
        if current_lock and current_lock in memory.instances:
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
        if desired.confidence < float(self.config.get("LOCK_MIN_CONFIDENCE", 0.35)):
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


def target_name_for_stage(stage: Any) -> str:
    return (
        str(getattr(stage, "target_query", "") or "")
        or str(getattr(stage, "target", "") or "")
        or str(getattr(stage, "instruction", "") or "")
    ).strip()


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
