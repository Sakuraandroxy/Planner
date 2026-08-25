"""Front-view target direction hints for Qwen planning prompts."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence

from agent.functions.common.config_access import first_value, function_section
from agent.functions.common.detection_policy import allows_clipped_large_structure
from agent.functions.perception.bearing_tracker import detection_is_excluded_by_bearing
from config import cfg


@dataclass
class DirectionHintResult:
    text: str = ""
    reason: str = ""
    angle_deg: float | None = None
    bbox: list[int] | None = None
    score: float = 0.0


def direction_hint_from_angle(
    angle_deg: float,
    *,
    score: float = 0.0,
    reason: str = "visual bearing memory",
) -> DirectionHintResult:
    angle = float(angle_deg)
    return DirectionHintResult(
        text=_format_direction_text(angle, use_angle=_direction_hint_use_angle()),
        reason=reason,
        angle_deg=angle,
        score=max(0.0, min(1.0, float(score or 0.0))),
    )


def direction_hint_from_locked_body_target(
    target_body_xyz: Sequence[float] | None,
    *,
    confidence: float = 0.0,
) -> DirectionHintResult:
    """Build an identity-safe hint from the locked memory instance.

    A class-only front-view detection (for example, ``white car``) cannot
    prove that the visible object is the instance locked by an earlier stage.
    Once memory has a world-space lock, its body-frame bearing is therefore
    the authoritative direction hint.
    """
    values = list(target_body_xyz or [])
    if len(values) < 2:
        return DirectionHintResult(reason="locked memory target unavailable")
    try:
        x = float(values[0])
        y = float(values[1])
    except (TypeError, ValueError):
        return DirectionHintResult(reason="invalid locked memory target")
    if not math.isfinite(x) or not math.isfinite(y) or math.hypot(x, y) <= 1e-6:
        return DirectionHintResult(reason="invalid locked memory target")

    angle = math.degrees(math.atan2(y, x))
    if x < 0.0:
        text = "The locked target is behind the drone. Turn around toward it before moving forward."
    else:
        text = _format_direction_text(angle, use_angle=_direction_hint_use_angle())
    return DirectionHintResult(
        text=text,
        reason="locked memory anchor",
        angle_deg=angle,
        score=max(0.0, min(1.0, float(confidence or 0.0))),
    )


def direction_hint_from_front_detection(
    detector: Any,
    front_image: Any,
    target: str,
    *,
    stage: Any = None,
    excluded_bearings: list[dict] | None = None,
) -> DirectionHintResult:
    if not _direction_hint_enabled():
        return DirectionHintResult(reason="direction hint disabled")
    target = str(target or "").strip()
    if detector is None:
        return DirectionHintResult(reason="detector unavailable")
    if front_image is None or not hasattr(front_image, "size"):
        return DirectionHintResult(reason="front image unavailable")
    if not target:
        return DirectionHintResult(reason="empty target")

    try:
        if hasattr(detector, "detect_all"):
            detections = list(
                detector.detect_all(front_image, target, depth_meters=None, camera_name="front") or []
            )
        else:
            detection = detector.detect(front_image, target, depth_meters=None, camera_name="front")
            detections = [detection] if detection is not None else []
    except Exception as exc:
        return DirectionHintResult(reason=f"detection failed: {exc}")

    visible = [detection for detection in detections if detection and getattr(detection, "visible", False)]
    if excluded_bearings:
        visible = [
            detection
            for detection in visible
            if not detection_is_excluded_by_bearing(
                detection,
                front_image,
                excluded_bearings,
                horizontal_fov_deg=_camera_hfov_deg(),
            )
        ]
    if not visible:
        reason = "previous target excluded" if excluded_bearings and detections else "target not detected"
        return DirectionHintResult(reason=reason)
    detection = max(visible, key=lambda item: float(getattr(item, "score", 0.0) or 0.0))
    if not detection or not getattr(detection, "visible", False):
        return DirectionHintResult(reason="target not detected")
    bbox = list(getattr(detection, "bbox", None) or [])
    if len(bbox) < 4:
        return DirectionHintResult(reason="bbox unavailable")

    score = float(getattr(detection, "score", 0.0) or 0.0)
    if score < _min_confidence():
        return DirectionHintResult(bbox=bbox[:4], score=score, reason=f"low confidence {score:.2f}")

    reliable, reason = _bbox_is_reliable(bbox, front_image, stage=stage, detection=detection)
    if not reliable:
        return DirectionHintResult(bbox=bbox[:4], score=score, reason=reason)

    angle = _bbox_center_angle_deg(bbox, front_image)
    return DirectionHintResult(
        text=_format_direction_text(angle, use_angle=_direction_hint_use_angle()),
        reason="front bbox accepted",
        angle_deg=angle,
        bbox=bbox[:4],
        score=score,
    )


def _direction_hint_enabled() -> bool:
    pcfg = function_section(cfg, "PLANNING")
    return bool(first_value(
        pcfg.get("DIRECTION_HINT_ENABLED"),
        pcfg.get("USE_DIRECTION_HINT"),
        default=False,
    ))


def _direction_hint_use_angle() -> bool:
    pcfg = function_section(cfg, "PLANNING")
    return bool(first_value(
        pcfg.get("DIRECTION_HINT_USE_ANGLE"),
        pcfg.get("DIRECTION_HINT_WITH_ANGLE"),
        default=True,
    ))


def _min_confidence() -> float:
    rcfg = function_section(cfg, "RELOCALIZATION")
    pcfg = function_section(cfg, "PERCEPTION")
    ag = cfg.get("AGENT", {}) or {}
    return float(first_value(
        rcfg.get("MIN_CONFIDENCE"),
        pcfg.get("MIN_CONFIDENCE"),
        ag.get("DETECTOR_MIN_CONFIDENCE"),
        ag.get("DETECTOR_BOX_THRESHOLD"),
        default=0.0,
    ))


def _max_bbox_span() -> float:
    rcfg = function_section(cfg, "RELOCALIZATION")
    ccfg = function_section(cfg, "COMPLETION")
    return float(first_value(
        rcfg.get("MAX_BBOX_SPAN"),
        ccfg.get("MAX_DETECTION_SPAN"),
        default=0.90,
    ))


def _camera_hfov_deg() -> float:
    rcfg = function_section(cfg, "RELOCALIZATION")
    sim = cfg.get("SIM", {}) or {}
    return float(first_value(rcfg.get("CAMERA_HFOV_DEG"), sim.get("FRONT_FOV"), default=90.0))


def _bbox_is_reliable(
    bbox: list[int],
    image: Any,
    *,
    stage: Any = None,
    detection: Any = None,
) -> tuple[bool, str]:
    width, height = float(image.size[0]), float(image.size[1])
    if width <= 1.0 or height <= 1.0:
        return False, "invalid image size"

    x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
    box_w = max(0.0, min(width, x2) - max(0.0, x1))
    box_h = max(0.0, min(height, y2) - max(0.0, y1))
    max_span = _max_bbox_span()

    if box_w / width >= max_span or box_h / height >= max_span:
        if detection is not None and allows_clipped_large_structure(
            stage,
            detection,
            image,
            max_span=max_span,
        ):
            return False, "clipped building facade; use depth-backed memory bearing"
        return False, f"bbox span too large ({box_w / width:.2f}, {box_h / height:.2f})"
    return True, "bbox accepted"


def _bbox_center_angle_deg(bbox: list[int], image: Any) -> float:
    width = float(image.size[0])
    center_x = (float(bbox[0]) + float(bbox[2])) / 2.0
    normalized_x = center_x / width - 0.5
    return normalized_x * _camera_hfov_deg()


def _straight_threshold_deg() -> float:
    pcfg = function_section(cfg, "PLANNING")
    rcfg = function_section(cfg, "RELOCALIZATION")
    return float(first_value(
        pcfg.get("DIRECTION_HINT_STRAIGHT_DEG"),
        rcfg.get("MIN_CENTER_OFFSET_DEG"),
        default=3.0,
    ))


def _format_direction_text(angle_deg: float, *, use_angle: bool = True) -> str:
    rounded = int(round(abs(float(angle_deg))))
    if abs(float(angle_deg)) <= _straight_threshold_deg():
        return "The target is 0 degrees straight." if use_angle else "The target is straight."
    side = "right" if angle_deg > 0 else "left"
    if not use_angle:
        return f"The target is on the {side}."
    degree_word = "degree" if rounded == 1 else "degrees"
    return f"The target is {rounded} {degree_word} on the {side}."
