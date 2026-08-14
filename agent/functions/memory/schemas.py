"""Lightweight mission memory data contracts.

MissionMemory intentionally stores only compact statistics.  It never stores
raw RGB frames, depth maps, or point clouds.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


def now_s() -> float:
    return time.perf_counter()


@dataclass
class AppearancePrototype:
    """A compact visual prototype for one observed appearance state."""

    # 这些向量只保存统计摘要，不保存原图，避免长期任务中内存持续上涨。
    lab_ab_hist: List[float] = field(default_factory=list)
    hue_hist: List[float] = field(default_factory=list)
    rgb_ratio: List[float] = field(default_factory=list)
    relative_rgb_ratio: List[float] = field(default_factory=list)
    texture: float = 0.0
    reliability: float = 0.0
    view: str = "unknown"
    created_at: float = field(default_factory=now_s)
    observation_count: int = 1

    def to_summary_dict(self) -> Dict[str, Any]:
        return {
            "view": self.view,
            "reliability": round(float(self.reliability), 3),
            "observations": int(self.observation_count),
        }


@dataclass
class PoseRecord:
    """Sparse UAV pose history for relation checks and dashboard debugging."""

    # 只保留最近少量位姿点，用于“经过/路过”等关系后续扩展，不做稠密建图。
    timestamp: float
    position: List[float]
    yaw_deg: float
    stage_key: str = ""


@dataclass
class StageSummary:
    """Completed-stage memory snapshot."""

    stage_key: str
    instruction: str
    target_key: str
    primary_instance_id: str
    reason: str
    completed_at: float = field(default_factory=now_s)


@dataclass
class TargetInstanceBelief:
    """Belief state for one physical target instance."""

    # instance_id 是跨小任务保持稳定的物体身份，例如 red_car:2。
    instance_id: str
    encounter_order: int
    target_world: List[float]
    confidence: float
    observation_count: int = 1
    first_seen_s: float = field(default_factory=now_s)
    last_seen_s: float = field(default_factory=now_s)
    status: str = "active"
    # sigma_xy/sigma_z 是粗略不确定性，不是完整协方差；足够用于完成判定收紧/放宽半径。
    sigma_xy: float = 2.0
    sigma_z: float = 1.0
    uncertainty_m: float = 2.0
    # footprint_radius_m 近似目标在水平面的尺寸半径，用于“上方/旁边”的几何关系判定。
    footprint_radius_m: float = 1.5
    # Bounded target-surface memory.  ``target_world`` remains an identity
    # anchor for compatibility; navigation and completion should prefer these
    # observed surfaces so a large object's center never becomes the goal.
    surface_points_world: List[List[float]] = field(default_factory=list)
    # Each detection contributes one finite surface patch. Distances use these
    # patches independently so empty space between observations is never
    # filled by one global axis-aligned bounding box.
    surface_patches_world: List[List[List[float]]] = field(default_factory=list)
    surface_bounds_world: Optional[List[List[float]]] = None
    surface_observation_count: int = 0
    last_surface_contact_s: float = 0.0
    last_surface_contact_kind: str = ""
    geometry_kind: str = "point"
    is_large_structure: bool = False
    last_bbox_span: float = 0.0
    depth_median: Optional[float] = None
    bbox_quality: float = 0.0
    last_seen_view: str = "unknown"
    last_label: str = ""
    appearance_prototypes: List[AppearancePrototype] = field(default_factory=list)
    observed_stage_keys: List[str] = field(default_factory=list)

    def age_s(self, now: Optional[float] = None) -> float:
        stamp = now_s() if now is None else float(now)
        return max(0.0, stamp - float(self.last_seen_s))

    def effective_uncertainty(self, *, now: Optional[float] = None, stale_growth_per_s: float = 0.03) -> float:
        """Return uncertainty inflated by memory staleness."""
        return float(self.uncertainty_m) + self.age_s(now) * float(stale_growth_per_s)

    def to_summary_dict(self) -> Dict[str, Any]:
        return {
            "id": self.instance_id,
            "order": int(self.encounter_order),
            "world": [round(float(v), 2) for v in self.target_world[:3]],
            "confidence": round(float(self.confidence), 3),
            "observations": int(self.observation_count),
            "uncertainty_m": round(float(self.uncertainty_m), 2),
            "footprint_radius_m": round(float(self.footprint_radius_m), 2),
            "geometry": self.geometry_kind,
            "surface_points": len(self.surface_points_world),
            "surface_patches": len(self.surface_patches_world),
            "surface_observations": int(self.surface_observation_count),
            "surface_contact_age_s": (
                None
                if self.last_surface_contact_s <= 0.0
                else round(max(0.0, now_s() - self.last_surface_contact_s), 1)
            ),
            "surface_contact_kind": self.last_surface_contact_kind,
            "surface_bounds": (
                None
                if not self.surface_bounds_world
                else [
                    [round(float(v), 2) for v in self.surface_bounds_world[0][:3]],
                    [round(float(v), 2) for v in self.surface_bounds_world[1][:3]],
                ]
            ),
            "large_structure": bool(self.is_large_structure),
            "view": self.last_seen_view,
            "age_s": round(self.age_s(), 1),
            "status": self.status,
            "appearance": [p.to_summary_dict() for p in self.appearance_prototypes[:3]],
        }


@dataclass
class TargetMemory:
    """All instances remembered for a target query such as 'red car'."""

    # target_key 是归一化后的目标名，避免 “the red car / red car” 生成两份缓存。
    target_key: str
    target_name: str
    selection_rule: str = "stable"
    ordinal: Optional[int] = None
    primary_instance_id: str = ""
    instances: Dict[str, TargetInstanceBelief] = field(default_factory=dict)
    next_encounter_order: int = 1

    def sorted_instances(self) -> List[TargetInstanceBelief]:
        return sorted(
            self.instances.values(),
            key=lambda inst: (int(inst.encounter_order), str(inst.instance_id)),
        )

    def primary_instance(self) -> Optional[TargetInstanceBelief]:
        if self.primary_instance_id:
            return self.instances.get(self.primary_instance_id)
        return None

    def to_summary_dict(self, stage_lock: str = "") -> Dict[str, Any]:
        primary = self.instances.get(stage_lock or self.primary_instance_id)
        return {
            "target": self.target_name,
            "key": self.target_key,
            "selection_rule": self.selection_rule,
            "ordinal": self.ordinal,
            "primary": primary.to_summary_dict() if primary else None,
            "instances": [inst.to_summary_dict() for inst in self.sorted_instances()],
        }


@dataclass
class MemoryCompletionDecision:
    """Tri-state completion decision produced by memory."""

    status: str
    done: bool = False
    source: str = "memory"
    reason: str = ""
    confidence: float = 0.0
    distance_m: Optional[float] = None
    horizontal_distance_m: Optional[float] = None
    vertical_delta_m: Optional[float] = None
    instance_id: str = ""
    target_world: Optional[List[float]] = None
    uncertainty_m: Optional[float] = None
    required_radius_m: Optional[float] = None
    details: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def complete(cls, **kwargs) -> "MemoryCompletionDecision":
        return cls(status="COMPLETE", done=True, **kwargs)

    @classmethod
    def hold(cls, **kwargs) -> "MemoryCompletionDecision":
        return cls(status="HOLD_CONFIRM", done=False, **kwargs)

    @classmethod
    def not_complete(cls, **kwargs) -> "MemoryCompletionDecision":
        return cls(status="NOT_COMPLETE", done=False, **kwargs)

    def to_summary_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status,
            "done": bool(self.done),
            "source": self.source,
            "reason": self.reason,
            "confidence": round(float(self.confidence), 3),
            "distance_m": None if self.distance_m is None else round(float(self.distance_m), 2),
            "horizontal_m": (
                None if self.horizontal_distance_m is None else round(float(self.horizontal_distance_m), 2)
            ),
            "vertical_m": None if self.vertical_delta_m is None else round(float(self.vertical_delta_m), 2),
            "instance_id": self.instance_id,
            "target_world": (
                None if self.target_world is None else [round(float(v), 2) for v in self.target_world[:3]]
            ),
            "uncertainty_m": None if self.uncertainty_m is None else round(float(self.uncertainty_m), 2),
            "required_radius_m": (
                None if self.required_radius_m is None else round(float(self.required_radius_m), 2)
            ),
            "details": dict(self.details or {}),
        }
