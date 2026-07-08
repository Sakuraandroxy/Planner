"""候选轨迹生成与扰动。"""

from __future__ import annotations

import math
from typing import Iterable, List

from agent.candidate.base import CandidateTrajectory
from agent.planner.api_atomic_planner import _actions_to_body_waypoints, _compute_delta


def build_seed_candidates(result) -> List[CandidateTrajectory]:
    """从 planner 输出构建基础候选。"""
    candidates: List[CandidateTrajectory] = []

    if getattr(result, "candidates", None):
        for idx, raw in enumerate(result.candidates):
            actions = [str(a) for a in raw.get("actions", [])]
            waypoints = _normalize_waypoints(raw.get("waypoints") or _actions_to_body_waypoints(actions))
            candidates.append(
                CandidateTrajectory(
                    actions=actions,
                    waypoints=waypoints,
                    reason=str(raw.get("reason", "")),
                    delta=list(raw.get("delta", _compute_delta(actions))),
                    scale=float(raw.get("scale", 1.0)),
                    source=str(raw.get("source", "planner")),
                    metadata={"planner_index": idx},
                )
            )

    if not candidates and getattr(result, "waypoints", None):
        waypoints = _normalize_waypoints(result.waypoints)
        candidates.append(
            CandidateTrajectory(
                actions=[str(a) for a in getattr(result, "actions", [])],
                waypoints=waypoints,
                reason=getattr(result, "reasoning", "") or "planner primary trajectory",
                delta=_delta_from_waypoints(waypoints),
                scale=1.0,
                source="planner_primary",
                metadata={"planner_index": 0},
            )
        )
    return candidates


def generate_perturbed_candidates(
    seed: CandidateTrajectory,
    scale_factors: Iterable[float],
    yaw_offsets_deg: Iterable[float],
    lateral_offsets_m: Iterable[float],
    max_candidates: int,
) -> List[CandidateTrajectory]:
    """围绕单条主轨迹生成扰动候选。"""
    if not seed.waypoints:
        return []

    out: List[CandidateTrajectory] = []
    seen = set()
    base_waypoints = _normalize_waypoints(seed.waypoints)

    for scale in scale_factors:
        for yaw_deg in yaw_offsets_deg:
            for lateral in lateral_offsets_m:
                new_waypoints = _transform_waypoints(base_waypoints, scale=scale, yaw_deg=yaw_deg, lateral=lateral)
                key = tuple(tuple(round(v, 2) for v in wp) for wp in new_waypoints)
                if key in seen:
                    continue
                seen.add(key)
                out.append(
                    CandidateTrajectory(
                        actions=list(seed.actions),
                        waypoints=new_waypoints,
                        reason=(
                            f"generated from planner seed "
                            f"(scale={scale:.2f}, yaw={yaw_deg:.1f}deg, lateral={lateral:.1f}m)"
                        ),
                        delta=_delta_from_waypoints(new_waypoints),
                        scale=scale,
                        source="generated",
                        metadata={
                            "seed_source": seed.source,
                            "yaw_offset_deg": yaw_deg,
                            "lateral_offset_m": lateral,
                        },
                    )
                )
                if len(out) >= max_candidates:
                    return out
    return out


def _transform_waypoints(
    waypoints: List[List[float]],
    scale: float,
    yaw_deg: float,
    lateral: float,
) -> List[List[float]]:
    rad = math.radians(yaw_deg)
    cos_v = math.cos(rad)
    sin_v = math.sin(rad)
    transformed: List[List[float]] = []
    for wp in waypoints:
        x = float(wp[0]) * scale
        y = float(wp[1]) * scale + lateral
        z = float(wp[2])
        xr = x * cos_v - y * sin_v
        yr = x * sin_v + y * cos_v
        transformed.append([round(xr, 3), round(yr, 3), round(z, 3)])
    return transformed


def _normalize_waypoints(waypoints) -> List[List[float]]:
    normalized: List[List[float]] = []
    for wp in waypoints or []:
        if not isinstance(wp, (list, tuple)) or len(wp) < 3:
            continue
        x, y, z = float(wp[0]), float(wp[1]), float(wp[2])
        if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
            continue
        normalized.append([round(x, 3), round(y, 3), round(z, 3)])
    return normalized


def _delta_from_waypoints(waypoints: List[List[float]]) -> List[float]:
    if not waypoints:
        return [0.0, 0.0, 0.0, 0.0]
    last = waypoints[-1]
    return [round(float(last[0]), 3), round(float(last[1]), 3), round(float(last[2]), 3), 0.0]
