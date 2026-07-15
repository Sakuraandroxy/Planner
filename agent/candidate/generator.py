"""候选轨迹生成与扰动。"""

from __future__ import annotations

import math
from typing import Iterable, List, Optional

import numpy as np

from agent.candidate.base import CandidateTrajectory, actions_to_cumulative_body_waypoints


def build_seed_candidates(result) -> List[CandidateTrajectory]:
    """从 planner 输出构建基础候选。"""
    candidates: List[CandidateTrajectory] = []

    if getattr(result, "candidates", None):
        for idx, raw in enumerate(result.candidates):
            actions = [str(a) for a in raw.get("actions", [])]
            waypoints = _normalize_waypoints(raw.get("waypoints") or actions_to_cumulative_body_waypoints(actions))
            candidates.append(
                CandidateTrajectory(
                    actions=actions,
                    waypoints=waypoints,
                    reason=str(raw.get("reason", "")),
                    delta=list(raw.get("delta", _delta_from_actions(actions))),
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
    random_seed: Optional[int] = None,
) -> List[CandidateTrajectory]:
    """围绕单条主轨迹生成扰动候选。"""
    if not seed.waypoints or max_candidates <= 0:
        return []

    all_candidates: List[CandidateTrajectory] = []
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
                all_candidates.append(
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

    if random_seed is not None:
        rng = np.random.default_rng(random_seed)
        order = rng.permutation(len(all_candidates)).tolist()
        all_candidates = [all_candidates[i] for i in order]

    return all_candidates[:max_candidates]


def generate_smooth_bridge_candidates(
    seed: CandidateTrajectory,
    max_candidates: int,
    lateral_sigma_m: float = 1.0,
    vertical_sigma_m: float = 0.25,
    smooth_length_scale: float = 0.35,
    max_attempts: int = 64,
    max_turn_deg: float = 120.0,
    max_segment_length_m: float = 0.0,
    max_path_length_ratio: float = 1.5,
    progress_tolerance_m: float = 1.0,
    random_seed: Optional[int] = None,
) -> List[CandidateTrajectory]:
    """生成固定起终点的平滑桥候选。

    planner/Qwen 输出的轨迹点是累计 body-frame waypoint，不包含无人机当前位姿。
    这里补上隐式起点 [0, 0, 0]，固定起点和最终 waypoint，只扰动中间 waypoint。
    """
    if not seed.waypoints or max_candidates <= 0:
        return []

    base_waypoints = _normalize_waypoints(seed.waypoints)
    if len(base_waypoints) < 2:
        return []

    base_path = [[0.0, 0.0, 0.0]] + base_waypoints
    free_indices = list(range(1, len(base_path) - 1))
    if not free_indices:
        return []

    out: List[CandidateTrajectory] = []
    seen = set()
    rng = np.random.default_rng(random_seed)
    s = _normalized_arclength(base_path)
    s_free = np.array([s[i] for i in free_indices], dtype=float)
    covariance = _smooth_covariance(
        s_free=s_free,
        smooth_length_scale=max(smooth_length_scale, 1e-3),
    )
    normals = _path_normals(base_path)
    base_length = _path_length_from_absolute_waypoints(base_path)
    attempts = max(max_attempts, max_candidates * 4)

    for _ in range(attempts):
        lateral_noise = _sample_bridge_noise(
            rng=rng,
            covariance=covariance,
            sigma=lateral_sigma_m,
            s_free=s_free,
        )
        vertical_noise = _sample_bridge_noise(
            rng=rng,
            covariance=covariance,
            sigma=vertical_sigma_m,
            s_free=s_free,
        )

        new_path = [list(wp) for wp in base_path]
        for noise_idx, path_idx in enumerate(free_indices):
            normal = normals[path_idx]
            base_wp = base_path[path_idx]
            new_path[path_idx] = [
                round(base_wp[0] + lateral_noise[noise_idx] * normal[0], 3),
                round(base_wp[1] + lateral_noise[noise_idx] * normal[1], 3),
                round(base_wp[2] + vertical_noise[noise_idx], 3),
            ]

        new_path[0] = list(base_path[0])
        new_path[-1] = list(base_path[-1])

        if not _is_feasible_bridge(
            path=new_path,
            base_path=base_path,
            base_length=base_length,
            max_turn_deg=max_turn_deg,
            max_segment_length_m=max_segment_length_m,
            max_path_length_ratio=max_path_length_ratio,
            progress_tolerance_m=progress_tolerance_m,
        ):
            continue

        new_waypoints = _normalize_waypoints(new_path[1:])
        key = tuple(tuple(round(v, 2) for v in wp) for wp in new_waypoints)
        if key in seen:
            continue
        seen.add(key)
        out.append(
            CandidateTrajectory(
                actions=list(seed.actions),
                waypoints=new_waypoints,
                reason=(
                    "smooth bridge from planner seed "
                    f"(lat_sigma={lateral_sigma_m:.2f}m, "
                    f"z_sigma={vertical_sigma_m:.2f}m, "
                    f"length_scale={smooth_length_scale:.2f})"
                ),
                delta=_delta_from_waypoints(new_waypoints),
                scale=1.0,
                source="smooth_bridge",
                metadata={
                    "seed_source": seed.source,
                    "lateral_sigma_m": float(lateral_sigma_m),
                    "vertical_sigma_m": float(vertical_sigma_m),
                    "smooth_length_scale": float(smooth_length_scale),
                    "fixed_start": [0.0, 0.0, 0.0],
                    "fixed_end": list(base_path[-1]),
                },
            )
        )
        if len(out) >= max_candidates:
            break

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


def _delta_from_actions(actions: List[str]) -> List[float]:
    waypoints = actions_to_cumulative_body_waypoints(actions)
    return _delta_from_waypoints(waypoints)


def _sample_bridge_noise(
    rng: np.random.Generator,
    covariance: np.ndarray,
    sigma: float,
    s_free: np.ndarray,
) -> np.ndarray:
    if sigma <= 0.0 or len(s_free) == 0:
        return np.zeros(len(s_free), dtype=float)
    noise = rng.multivariate_normal(
        mean=np.zeros(len(s_free), dtype=float),
        cov=(sigma ** 2) * covariance,
    )
    # Gaussian bridge: 起点/终点扰动为 0，中间扰动更充分。
    return noise * np.sin(np.pi * s_free)


def _smooth_covariance(s_free: np.ndarray, smooth_length_scale: float) -> np.ndarray:
    if len(s_free) == 0:
        return np.zeros((0, 0), dtype=float)
    d = s_free[:, None] - s_free[None, :]
    cov = np.exp(-0.5 * (d / smooth_length_scale) ** 2)
    cov += np.eye(len(s_free)) * 1e-6
    return cov


def _normalized_arclength(path: List[List[float]]) -> List[float]:
    if not path:
        return []
    cumulative = [0.0]
    total = 0.0
    prev = path[0]
    for cur in path[1:]:
        total += _distance(prev, cur)
        cumulative.append(total)
        prev = cur
    if total <= 1e-6:
        count = max(len(path) - 1, 1)
        return [i / count for i in range(len(path))]
    return [v / total for v in cumulative]


def _path_normals(path: List[List[float]]) -> List[List[float]]:
    normals: List[List[float]] = []
    fallback = _xy_unit([path[-1][0] - path[0][0], path[-1][1] - path[0][1]])
    if fallback is None:
        fallback = [1.0, 0.0]

    for idx, _ in enumerate(path):
        prev = path[max(0, idx - 1)]
        nxt = path[min(len(path) - 1, idx + 1)]
        tangent = _xy_unit([nxt[0] - prev[0], nxt[1] - prev[1]])
        if tangent is None:
            tangent = fallback
        normals.append([round(-tangent[1], 6), round(tangent[0], 6), 0.0])
    return normals


def _xy_unit(vec: List[float]) -> Optional[List[float]]:
    norm = math.hypot(float(vec[0]), float(vec[1]))
    if norm <= 1e-6:
        return None
    return [float(vec[0]) / norm, float(vec[1]) / norm]


def _is_feasible_bridge(
    path: List[List[float]],
    base_path: List[List[float]],
    base_length: float,
    max_turn_deg: float,
    max_segment_length_m: float,
    max_path_length_ratio: float,
    progress_tolerance_m: float,
) -> bool:
    if len(path) != len(base_path):
        return False
    if path[0] != base_path[0] or path[-1] != base_path[-1]:
        return False

    length = _path_length_from_absolute_waypoints(path)
    if base_length > 1e-6 and length > base_length * max_path_length_ratio + 1e-6:
        return False

    if max_segment_length_m > 0.0:
        for prev, cur in zip(path, path[1:]):
            if _distance(prev, cur) > max_segment_length_m:
                return False

    if max_turn_deg > 0.0 and _max_turn_angle_deg(path) > max_turn_deg:
        return False

    return _has_monotonic_goal_progress(path, progress_tolerance_m=progress_tolerance_m)


def _max_turn_angle_deg(path: List[List[float]]) -> float:
    max_angle = 0.0
    segments = []
    for prev, cur in zip(path, path[1:]):
        vec = [cur[0] - prev[0], cur[1] - prev[1], cur[2] - prev[2]]
        if _norm(vec) > 1e-6:
            segments.append(vec)
    for a, b in zip(segments, segments[1:]):
        denom = _norm(a) * _norm(b)
        if denom <= 1e-6:
            continue
        cos_v = max(-1.0, min(1.0, _dot(a, b) / denom))
        max_angle = max(max_angle, math.degrees(math.acos(cos_v)))
    return max_angle


def _has_monotonic_goal_progress(
    path: List[List[float]],
    progress_tolerance_m: float,
) -> bool:
    goal_vec = [path[-1][0] - path[0][0], path[-1][1] - path[0][1], path[-1][2] - path[0][2]]
    goal_norm = _norm(goal_vec)
    if goal_norm <= 1e-6:
        return True
    axis = [v / goal_norm for v in goal_vec]
    prev_progress = -float("inf")
    for point in path:
        rel = [point[0] - path[0][0], point[1] - path[0][1], point[2] - path[0][2]]
        progress = _dot(rel, axis)
        if progress + progress_tolerance_m < prev_progress:
            return False
        prev_progress = max(prev_progress, progress)
    return True


def _distance(a: List[float], b: List[float]) -> float:
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    )


def _path_length_from_absolute_waypoints(path: List[List[float]]) -> float:
    return sum(_distance(prev, cur) for prev, cur in zip(path, path[1:]))


def _dot(a: List[float], b: List[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def _norm(vec: List[float]) -> float:
    return math.sqrt(_dot(vec, vec))


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
