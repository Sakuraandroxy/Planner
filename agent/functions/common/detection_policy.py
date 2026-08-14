"""Shared detector reliability policy for large, partially visible structures."""

from __future__ import annotations

from typing import Any


_LARGE_STRUCTURE_TOKENS = (
    "building",
    "high rise",
    "high-rise",
    "tower",
    "skyscraper",
    "warehouse",
    "hangar",
    "facade",
    "exterior wall",
    "楼",
    "建筑",
    "大厦",
    "高楼",
    "塔",
    "仓库",
    "外墙",
    "墙面",
)


def stage_target_text(stage: Any) -> str:
    return " ".join((
        str(getattr(stage, "target_query", "") or ""),
        str(getattr(stage, "target", "") or ""),
        str(getattr(stage, "instruction", "") or ""),
        str(getattr(stage, "completion_condition", "") or ""),
    )).lower()


def is_large_structure_stage(stage: Any) -> bool:
    text = stage_target_text(stage)
    return any(token in text for token in _LARGE_STRUCTURE_TOKENS)


def detection_caption_for_stage(stage: Any, fallback: str = "") -> str:
    caption = str(
        getattr(stage, "target_query", None)
        or getattr(stage, "target", None)
        or getattr(stage, "instruction", None)
        or fallback
        or ""
    ).strip()
    if caption and is_large_structure_stage(stage):
        lowered = caption.lower()
        if not any(token in lowered for token in ("facade", "exterior wall", "wall surface", "外墙", "墙面")):
            return f"{caption}. building facade. exterior building wall"
    return caption


def bbox_statistics(detection: Any, image: Any) -> dict[str, float | bool] | None:
    bbox = list(getattr(detection, "bbox", []) or [])
    if len(bbox) < 4 or image is None or not hasattr(image, "size"):
        return None
    width, height = float(image.size[0]), float(image.size[1])
    if width <= 1.0 or height <= 1.0:
        return None
    x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
    box_w = max(0.0, min(width, x2) - max(0.0, x1))
    box_h = max(0.0, min(height, y2) - max(0.0, y1))
    return {
        "span_x": box_w / width,
        "span_y": box_h / height,
        "area_ratio": (box_w * box_h) / max(width * height, 1.0),
        "touches_border": bool(x1 <= 2.0 or y1 <= 2.0 or x2 >= width - 2.0 or y2 >= height - 2.0),
    }


def allows_clipped_large_structure(
    stage: Any,
    detection: Any,
    image: Any,
    *,
    max_span: float = 0.90,
) -> bool:
    stats = bbox_statistics(detection, image)
    if stats is None or not is_large_structure_stage(stage):
        return False
    camera = str(getattr(detection, "camera", "front") or "front").strip().lower()
    if camera != "front":
        return False
    clipped = float(stats["span_x"]) >= max_span or float(stats["span_y"]) >= max_span
    return bool(clipped and float(stats["area_ratio"]) >= 0.01 and stats["touches_border"])


def detection_reliability(
    stage: Any,
    detection: Any,
    image: Any = None,
    *,
    max_span: float = 0.90,
) -> float:
    if not detection or not getattr(detection, "visible", False):
        return 0.0
    score = max(0.0, min(1.0, float(getattr(detection, "score", 0.0) or 0.0)))
    stats = bbox_statistics(detection, image)
    if stats is None:
        return score

    area_ratio = float(stats["area_ratio"])
    span_x = float(stats["span_x"])
    span_y = float(stats["span_y"])
    touches_border = bool(stats["touches_border"])
    clipped_structure = allows_clipped_large_structure(
        stage,
        detection,
        image,
        max_span=max_span,
    )

    quality = 1.0
    if area_ratio <= 0.0002:
        quality *= 0.35
    elif area_ratio <= 0.001:
        quality *= 0.65
    if span_x >= max_span or span_y >= max_span:
        if not clipped_structure:
            return 0.0
        # A clipped facade is weak for instance appearance/direction, but it
        # remains useful semantic support for a depth-backed building surface.
        quality *= 0.12
    elif area_ratio >= 0.55:
        quality *= 0.03
    elif area_ratio >= 0.30 or span_x >= 0.75 or span_y >= 0.75:
        quality *= 0.20
    elif area_ratio >= 0.18:
        quality *= 0.45
    if touches_border:
        quality *= 0.45 if _is_above_stage(stage) else 0.35
    return score * quality


def _is_above_stage(stage: Any) -> bool:
    relation = str(getattr(stage, "relation", "") or "").strip().lower()
    instruction = str(getattr(stage, "instruction", "") or "").strip().lower()
    return relation in {"above", "over", "on top", "on top of"} or "above" in instruction or "on top" in instruction
