"""Relation-aware completion reasoning for MissionMemory."""

from __future__ import annotations

import math
from typing import Any, Optional, Sequence

from agent.functions.memory.geometry import (
    distance3,
    distance_to_instance_geometry,
    has_surface_geometry,
    horizontal_distance,
    horizontal_distance_to_instance_geometry,
    nearest_instance_surface_point,
    vertical_distance_to_instance_geometry,
)
from agent.functions.memory.schemas import MemoryCompletionDecision, TargetInstanceBelief


ABOVE_RELATIONS = {"above", "over", "on top", "on top of", "top"}
NEAR_RELATIONS = {"", "near", "beside", "next to", "around", "by"}
PASS_RELATIONS = {"pass", "via", "through", "经过", "路过"}


def relation_kind(stage: Any) -> str:
    relation = str(getattr(stage, "relation", "") or "").strip().lower()
    instruction = str(getattr(stage, "instruction", "") or "").strip().lower()
    completion = str(getattr(stage, "completion_condition", "") or "").strip().lower()
    merged = f"{relation} {instruction} {completion}"
    if has_landing_intent(stage):
        return "land"
    if relation in ABOVE_RELATIONS or any(token in merged for token in ("above", "over", "on top", "上方", "上面")):
        return "above"
    if relation in PASS_RELATIONS or any(token in merged for token in ("pass", "via", "through", "经过", "路过")):
        return "pass"
    if relation in NEAR_RELATIONS or any(token in merged for token in ("near", "beside", "next to", "旁", "附近")):
        return "near"
    return "near"


def has_landing_intent(stage: Any) -> bool:
    text = " ".join(
        str(getattr(stage, name, "") or "").strip().lower()
        for name in ("instruction", "completion_condition", "relation", "action")
    )
    return any(token in text for token in ("land", "landing", "touch down", "降落", "着陆", "落地"))


def evaluate_memory_completion(
    *,
    stage: Any,
    instance: Optional[TargetInstanceBelief],
    current_world: Sequence[float],
    config: dict,
    fresh_visual_support: bool = False,
    visual_score: float = 0.0,
    stop_radius_m: float = 4.0,
) -> MemoryCompletionDecision:
    """Evaluate whether memory can complete a target stage."""
    if instance is None:
        return MemoryCompletionDecision.not_complete(reason="no primary memory instance")
    kind = relation_kind(stage)
    if kind == "land":
        return MemoryCompletionDecision.not_complete(
            instance_id=instance.instance_id,
            reason="landing must be executed physically; memory only guides target location",
            confidence=instance.confidence,
            target_world=list(instance.target_world),
        )
    if kind == "pass":
        return MemoryCompletionDecision.hold(
            instance_id=instance.instance_id,
            reason="pass/via relation needs path-history crossing confirmation",
            confidence=instance.confidence,
            target_world=list(instance.target_world),
        )

    current = [float(v) for v in current_world[:3]]
    target = [float(v) for v in instance.target_world[:3]]
    surface_geometry = has_surface_geometry(instance) and kind == "near"
    nearest_surface = (
        nearest_instance_surface_point(current, instance)
        if surface_geometry
        else target
    )
    d3 = distance_to_instance_geometry(current, instance) if surface_geometry else distance3(current, target)
    hdist = (
        horizontal_distance_to_instance_geometry(current, instance)
        if surface_geometry
        else horizontal_distance(current, target)
    )
    vdist = (
        vertical_distance_to_instance_geometry(current, instance)
        if surface_geometry
        else abs(float(current[2]) - float(target[2]))
    )
    uncertainty = instance.effective_uncertainty(
        stale_growth_per_s=float(config.get("STALE_UNCERTAINTY_GROWTH_MPS", 0.03))
    )
    max_uncertainty = float(config.get("MAX_COMPLETION_UNCERTAINTY_M", 5.0))
    confidence = _confidence_with_visual_support(instance, fresh_visual_support, visual_score)
    surface_memory_only = bool(
        surface_geometry
        and not fresh_visual_support
        and config.get("SURFACE_MEMORY_ONLY_COMPLETION_ENABLED", True)
    )
    if surface_memory_only:
        if bool(instance.is_large_structure):
            min_conf = float(config.get("LARGE_STRUCTURE_MEMORY_ONLY_MIN_CONFIDENCE", 0.55))
            min_obs = int(config.get("LARGE_STRUCTURE_MEMORY_ONLY_MIN_OBSERVATIONS", 1))
        else:
            min_conf = float(config.get("SURFACE_MEMORY_ONLY_MIN_CONFIDENCE", 0.65))
            min_obs = int(config.get("SURFACE_MEMORY_ONLY_MIN_OBSERVATIONS", 2))
    else:
        min_conf = _min_confidence(config, fresh_visual_support=fresh_visual_support)
        min_obs = int(config.get(
            "MIN_OBSERVATIONS_FOR_COMPLETION",
            1 if fresh_visual_support else config.get("MEMORY_ONLY_MIN_OBSERVATIONS", 2),
        ))

    if surface_memory_only and instance.age_s() > float(config.get("SURFACE_COMPLETION_MAX_AGE_S", 120.0)):
        return MemoryCompletionDecision.hold(
            instance_id=instance.instance_id,
            reason="target surface memory is too old",
            confidence=confidence,
            distance_m=d3,
            horizontal_distance_m=hdist,
            vertical_delta_m=vdist,
            target_world=list(nearest_surface),
            uncertainty_m=uncertainty,
            details={"geometry": instance.geometry_kind, "memory_age_s": round(instance.age_s(), 1)},
        )

    if kind == "above":
        required = (
            float(config.get("ABOVE_HORIZONTAL_RADIUS_M", 3.5))
            + min(uncertainty, max_uncertainty)
        )
        if not surface_geometry:
            required += float(instance.footprint_radius_m)
        clearance = float(target[2]) - float(current[2])
        min_clearance = float(config.get("ABOVE_MIN_CLEARANCE_M", 0.3))
        max_clearance = float(config.get("ABOVE_MAX_ALTITUDE_M", 60.0))
        horizontal_ok = hdist <= required
        height_ok = clearance >= min_clearance and clearance <= max_clearance
        return _decision_from_constraints(
            instance=instance,
            target_world=nearest_surface,
            confidence=confidence,
            min_conf=min_conf,
            min_obs=min_obs,
            uncertainty=uncertainty,
            max_uncertainty=max_uncertainty,
            distance_m=d3,
            horizontal_distance_m=hdist,
            vertical_delta_m=vdist,
            required_radius_m=required,
            constraints_ok=horizontal_ok and height_ok,
            near_but_uncertain=hdist <= required * 1.35,
            reason_ok="memory_above_geometry_complete",
            reason_far=(
                "not horizontally above target" if not horizontal_ok
                else "height clearance outside above relation"
            ),
            details={
                "relation": kind,
                "clearance_m": round(clearance, 2),
                "geometry": instance.geometry_kind,
            },
        )

    if surface_geometry:
        # The distance is already measured to the observed facade/body
        # surface, so adding the object's footprint again would double-count
        # large buildings.  Uncertainty only contributes a small tolerance;
        # it primarily belongs to safety inflation, not success inflation.
        required = max(
            float(stop_radius_m) + float(config.get("NEAR_RADIUS_MARGIN_M", 1.0)),
            float(config.get("SURFACE_NEAR_RADIUS_M", config.get("NEAR_STANDOFF_M", 6.0))),
        )
        required += min(
            uncertainty,
            float(config.get("SURFACE_COMPLETION_UNCERTAINTY_CAP_M", 2.0)),
        ) * float(config.get("SURFACE_COMPLETION_UNCERTAINTY_RATIO", 0.15))
    else:
        required = (
            float(stop_radius_m)
            + float(config.get("NEAR_RADIUS_MARGIN_M", 1.0))
            + min(uncertainty, max_uncertainty)
        )
        required = max(
            required,
            float(config.get("NEAR_STANDOFF_M", stop_radius_m))
            + float(config.get("LOW_ALTITUDE_EXTRA_STANDOFF_M", 0.0)),
        )
    max_altitude = float(config.get("NEAR_MAX_ALTITUDE_M", 8.0))
    horizontal_ok = hdist <= required
    height_ok = vdist <= max_altitude
    return _decision_from_constraints(
        instance=instance,
        target_world=nearest_surface,
        confidence=confidence,
        min_conf=min_conf,
        min_obs=min_obs,
        uncertainty=uncertainty,
        max_uncertainty=max_uncertainty,
        distance_m=d3,
        horizontal_distance_m=hdist,
        vertical_delta_m=vdist,
        required_radius_m=required,
        constraints_ok=horizontal_ok and height_ok,
        near_but_uncertain=hdist <= required * 1.25,
        reason_ok=("memory_surface_near_complete" if surface_geometry else "memory_near_geometry_complete"),
        reason_far=(
            "not within memory near radius" if not horizontal_ok
            else "height too different for near/beside completion"
        ),
        details={
            "relation": kind,
            "height_limit_m": round(max_altitude, 2),
            "geometry": instance.geometry_kind,
            "nearest_surface_world": [round(float(v), 2) for v in nearest_surface[:3]],
            "large_structure": bool(instance.is_large_structure),
        },
    )


def _confidence_with_visual_support(
    instance: TargetInstanceBelief,
    fresh_visual_support: bool,
    visual_score: float,
) -> float:
    base = float(instance.confidence)
    if not fresh_visual_support:
        return base
    # 新鲜弱视觉证据只加一点置信度，不能让错误实例强行完成。
    return max(base, min(0.99, 0.75 * base + 0.25 * float(visual_score or 0.0) + 0.10))


def _min_confidence(config: dict, *, fresh_visual_support: bool) -> float:
    if fresh_visual_support and bool(config.get("WEAK_VISUAL_SUPPORT_ENABLED", True)):
        return float(config.get("MIN_CONFIDENCE_FOR_COMPLETION", 0.65))
    if not bool(config.get("MEMORY_ONLY_COMPLETION_ENABLED", True)):
        return 1.1
    return float(config.get("MEMORY_ONLY_MIN_CONFIDENCE", 0.88))


def _decision_from_constraints(
    *,
    instance: TargetInstanceBelief,
    target_world: Sequence[float],
    confidence: float,
    min_conf: float,
    min_obs: int,
    uncertainty: float,
    max_uncertainty: float,
    distance_m: float,
    horizontal_distance_m: float,
    vertical_delta_m: float,
    required_radius_m: float,
    constraints_ok: bool,
    near_but_uncertain: bool,
    reason_ok: str,
    reason_far: str,
    details: dict,
) -> MemoryCompletionDecision:
    common = dict(
        instance_id=instance.instance_id,
        confidence=confidence,
        distance_m=distance_m,
        horizontal_distance_m=horizontal_distance_m,
        vertical_delta_m=vertical_delta_m,
        target_world=[float(v) for v in target_world[:3]],
        uncertainty_m=uncertainty,
        required_radius_m=required_radius_m,
        details=details,
    )
    if not constraints_ok:
        if near_but_uncertain:
            return MemoryCompletionDecision.hold(reason=reason_far, **common)
        return MemoryCompletionDecision.not_complete(reason=reason_far, **common)
    if uncertainty > max_uncertainty:
        return MemoryCompletionDecision.hold(reason="memory uncertainty too high", **common)
    if int(instance.observation_count) < int(min_obs):
        return MemoryCompletionDecision.hold(reason="not enough memory observations", **common)
    if confidence < min_conf:
        return MemoryCompletionDecision.hold(reason="memory confidence below completion threshold", **common)
    return MemoryCompletionDecision.complete(reason=reason_ok, **common)
