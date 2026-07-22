"""Candidate trajectory pre-scoring."""

from __future__ import annotations

import math
from typing import Dict, List

from agent.functions.candidate.base import CandidateTrajectory


def score_candidates(
    candidates: List[CandidateTrajectory],
    detection=None,
    direction: str = "",
    stop_threshold: float = 8.0,
) -> List[CandidateTrajectory]:
    """Assign pre_score for ranking and confidence for execution trust."""
    if not candidates:
        return candidates

    for cand in candidates:
        breakdown = _score_one(
            cand,
            detection=detection,
            direction=direction,
            stop_threshold=stop_threshold,
        )
        total = sum(breakdown.values())
        cand.pre_score = round(total, 4)
        cand.score_breakdown = {k: round(v, 4) for k, v in breakdown.items()}

    det_conf = float(getattr(detection, "score", 0.0) or 0.0)

    for cand in candidates:
        trajectory_quality = max(0.0, min(1.0, float(cand.pre_score)))
        cand.confidence = round(
            max(0.0, min(1.0, 0.70 * trajectory_quality + 0.30 * det_conf)),
            4,
        )
    return candidates


def _score_one(
    cand: CandidateTrajectory,
    detection=None,
    direction: str = "",
    stop_threshold: float = 8.0,
) -> Dict[str, float]:
    endpoint = cand.waypoints[-1] if cand.waypoints else [0.0, 0.0, 0.0]
    path_len = _path_length(cand.waypoints)
    det_depth = getattr(detection, "depth_median", None)
    camera = getattr(detection, "camera", "none") if detection is not None else "none"

    progress = 0.0
    alignment = 0.0
    safety = 0.0
    smoothness = 0.0

    if camera == "front":
        progress = _front_progress_score(endpoint, det_depth, stop_threshold)
        alignment = _front_alignment_score(endpoint, direction)
    elif camera == "down":
        progress = _down_progress_score(endpoint, path_len, det_depth)
        alignment = _down_alignment_score(endpoint, detection)
    else:
        progress = min(max(endpoint[0] / 10.0, -0.5), 0.5)
        alignment = 0.0

    safety = _safety_score(endpoint, path_len, det_depth, camera)
    smoothness = _smoothness_score(cand.waypoints)

    return {
        "progress": 0.45 * progress,
        "alignment": 0.25 * alignment,
        "safety": 0.20 * safety,
        "smoothness": 0.10 * smoothness,
    }


def _front_progress_score(endpoint, det_depth, stop_threshold: float) -> float:
    x = float(endpoint[0])
    if det_depth is None:
        return max(-0.5, min(1.0, x / 10.0))
    desired = max(float(det_depth) - stop_threshold, 0.0)
    if desired <= 1e-3:
        return 1.0 - min(abs(x) / 3.0, 1.0)
    err = abs(x - desired)
    return 1.0 - min(err / max(desired, 1.0), 1.5)


def _front_alignment_score(endpoint, direction: str) -> float:
    y = float(endpoint[1])
    text = (direction or "").lower()
    if "front-left" in text:
        return 1.0 if y < -0.2 else max(0.0, 1.0 - abs(y + 1.0) / 3.0)
    if "front-right" in text:
        return 1.0 if y > 0.2 else max(0.0, 1.0 - abs(y - 1.0) / 3.0)
    if "straight" in text or "ahead" in text:
        return 1.0 - min(abs(y) / 3.0, 1.0)
    return 0.5


def _down_progress_score(endpoint, path_len: float, det_depth) -> float:
    if det_depth is not None and float(det_depth) < 3.0:
        return 1.0 - min(path_len / 6.0, 1.0)
    return 1.0 - min(path_len / 10.0, 1.0)


def _down_alignment_score(endpoint, detection) -> float:
    bbox = getattr(detection, "bbox", None)
    if not bbox:
        return 0.5
    target_x, target_y = _desired_xy_from_down_bbox(bbox)
    dx = float(endpoint[0]) - target_x
    dy = float(endpoint[1]) - target_y
    err = math.sqrt(dx * dx + dy * dy)
    return 1.0 - min(err / 4.0, 1.0)


def _desired_xy_from_down_bbox(bbox) -> tuple[float, float]:
    x1, y1, x2, y2 = bbox
    cx = (float(x1) + float(x2)) / 2.0
    cy = (float(y1) + float(y2)) / 2.0
    # Use a coarse default image size to estimate body-frame direction.
    nx = (cx / 640.0) - 0.5
    ny = (cy / 640.0) - 0.5
    desired_x = -ny * 4.0
    desired_y = nx * 4.0
    return desired_x, desired_y


def _safety_score(endpoint, path_len: float, det_depth, camera: str) -> float:
    z_penalty = min(abs(float(endpoint[2])) / 5.0, 1.0)
    length_penalty = min(path_len / 25.0, 1.0)
    overshoot_penalty = 0.0
    if camera == "front" and det_depth is not None:
        overshoot_limit = max(float(det_depth) + 2.0, 0.0)
        overshoot_penalty = min(max(float(endpoint[0]) - overshoot_limit, 0.0) / 5.0, 1.0)
    return 1.0 - min(1.0, 0.5 * z_penalty + 0.3 * length_penalty + 0.2 * overshoot_penalty)


def _smoothness_score(waypoints: List[List[float]]) -> float:
    if len(waypoints) <= 1:
        return 1.0
    deltas = []
    prev = [0.0, 0.0, 0.0]
    for wp in waypoints:
        deltas.append([wp[0] - prev[0], wp[1] - prev[1], wp[2] - prev[2]])
        prev = wp
    curvatures = []
    for i in range(1, len(deltas)):
        a = deltas[i - 1]
        b = deltas[i]
        dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
        na = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        nb = math.sqrt(b[0] ** 2 + b[1] ** 2 + b[2] ** 2)
        if na < 1e-6 or nb < 1e-6:
            continue
        curvatures.append(max(-1.0, min(1.0, dot / (na * nb))))
    if not curvatures:
        return 1.0
    avg_cos = sum(curvatures) / len(curvatures)
    return max(0.0, min(1.0, (avg_cos + 1.0) / 2.0))


def _path_length(waypoints: List[List[float]]) -> float:
    prev = [0.0, 0.0, 0.0]
    total = 0.0
    for wp in waypoints:
        dx = float(wp[0]) - prev[0]
        dy = float(wp[1]) - prev[1]
        dz = float(wp[2]) - prev[2]
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
        prev = [float(wp[0]), float(wp[1]), float(wp[2])]
    return total



