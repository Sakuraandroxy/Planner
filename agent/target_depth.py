"""Target bbox detection and metric depth estimation helpers."""
import json
import re
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


TARGET_BBOX_PROMPT = """请观察这张RGB图，定位导航任务目标。

任务目标：{task_description}
当前图片尺寸：宽 {image_width} 像素，高 {image_height} 像素。

必须直接返回一行JSON对象，包含这些字段：visible、bbox_norm、confidence、target、candidates。

规则：
1. visible 是 true 或 false。
2. bbox_norm 是4个数字组成的数组，范围 0.0 到 1.0，顺序是左、上、右、下。
3. confidence 是0.0到1.0之间的数字。
4. target 是目标名字符串，例如 car。
5. candidates 是候选目标数组；每个候选对象包含 bbox_norm、confidence、target。
6. 如果画面中有多个同类目标，把所有候选都放入 candidates。
7. 如果任务包含“较远/远处/远的/far”，最终 bbox_norm 选择视觉上更远的目标，不要选择最近目标。
8. 如果目标不可见，输出 visible=false，bbox_norm=null，confidence=0.0，target=""，candidates=[]。
9. 禁止复述本提示词，禁止输出 x1、y1、x2、y2，禁止输出解释、Markdown、分析过程或动作规划。
10. 如果无法精确计算坐标，也必须基于视觉估计给出数字 bbox_norm，不要解释原因。"""

TARGET_BBOX_SYSTEM_PROMPT = """你是目标检测器。根据图片输出真实目标框JSON。
只输出一行JSON对象；禁止思考过程、解释、Markdown、动作规划、占位符。"""


@dataclass
class TargetDepthEstimate:
    """Depth statistics for the VLM-detected target bbox."""
    visible: bool
    bbox: Optional[list[int]] = None
    confidence: float = 0.0
    target: str = ""
    raw_bbox: Optional[list[float]] = None
    depth_bbox: Optional[list[int]] = None
    depth_median: Optional[float] = None
    depth_min: Optional[float] = None
    depth_mean: Optional[float] = None
    depth_center_median: Optional[float] = None
    valid_pixel_count: int = 0

    def as_dict(self) -> dict:
        return {
            "target_visible": self.visible,
            "target_bbox": self.bbox,
            "target_depth_bbox": self.depth_bbox,
            "target_confidence": self.confidence,
            "target_name": self.target,
            "target_depth_median": self.depth_median,
            "target_depth_min": self.depth_min,
            "target_depth_mean": self.depth_mean,
            "target_depth_center_median": self.depth_center_median,
            "target_depth_valid_pixels": self.valid_pixel_count,
        }


def build_target_bbox_messages(base64_image: str, task_description: str,
                               image_size: tuple[int, int]) -> list[dict]:
    """Build a compact VLM request that returns only the target bbox."""
    return [
        {"role": "system", "content": TARGET_BBOX_SYSTEM_PROMPT},
        {
        "role": "user",
        "content": [
            {
                "type": "image_url",
                "image_url": {"url": f"data:image/png;base64,{base64_image}"},
            },
            {
                "type": "text",
                "text": TARGET_BBOX_PROMPT.format(
                    task_description=task_description,
                    image_width=image_size[0],
                    image_height=image_size[1],
                ),
            },
        ],
    }]


def parse_target_bbox(response_text: str, image_size: tuple[int, int]) -> TargetDepthEstimate:
    """Parse the selected target bbox JSON from an MLLM response."""
    estimates = parse_target_candidates(response_text, image_size)
    return estimates[0] if estimates else TargetDepthEstimate(visible=False)


def parse_target_candidates(response_text: str, image_size: tuple[int, int]) -> list[TargetDepthEstimate]:
    """Parse selected and candidate target bboxes from an MLLM response."""
    text = re.sub(r"```json\s*", "", response_text)
    text = re.sub(r"```\s*", "", text)
    json_match = re.search(r"\{.*\}", text, re.DOTALL)
    if not json_match:
        raise ValueError(f"Cannot extract target bbox JSON: {response_text[:200]}")

    raw_json = json_match.group()
    try:
        data = json.loads(raw_json, strict=False)
    except json.JSONDecodeError as exc:
        data = load_relaxed_json(raw_json)
        if data is None:
            raise ValueError(
                f"Cannot parse target bbox JSON: {exc}; raw={raw_json[:300]}"
            ) from exc
    visible = bool(data.get("visible", False))
    if not visible:
        return [TargetDepthEstimate(visible=False)]

    raw_candidates = data.get("candidates") or []
    if data.get("bbox_norm") or data.get("bbox"):
        raw_candidates.insert(0, data)
    estimates = []
    seen = set()
    for candidate in raw_candidates:
        bbox = candidate.get("bbox_norm") if isinstance(candidate, dict) else None
        if bbox is None:
            bbox = candidate.get("bbox") if isinstance(candidate, dict) else None
        if not bbox:
            continue
        normalized = normalize_bbox(bbox, image_size)
        key = tuple(normalized)
        if key in seen:
            continue
        seen.add(key)
        estimates.append(TargetDepthEstimate(
            visible=True,
            bbox=normalized,
            confidence=float(candidate.get("confidence", 0.0) or 0.0),
            target=str(candidate.get("target", data.get("target", "")) or ""),
            raw_bbox=[float(value) for value in bbox],
        ))
    return estimates or [TargetDepthEstimate(visible=False)]


def load_relaxed_json(raw_json: str) -> Optional[dict]:
    """Parse common JSON-like bbox outputs from MLLMs."""
    fixed = raw_json.strip()
    fixed = fixed.replace("None", "null").replace("True", "true").replace("False", "false")
    fixed = fixed.replace("'", '"')
    fixed = re.sub(r",\s*([}\]])", r"\1", fixed)
    fixed = re.sub(r"([{,]\s*)([A-Za-z_][A-Za-z0-9_]*)\s*:", r'\1"\2":', fixed)
    try:
        return json.loads(fixed, strict=False)
    except json.JSONDecodeError:
        pass

    bbox_match = re.search(
        r'"?(?:bbox_norm|bbox)"?\s*:\s*\[\s*([0-9.]+)\s*,\s*([0-9.]+)\s*,\s*([0-9.]+)\s*,\s*([0-9.]+)\s*\]',
        fixed,
    )
    if not bbox_match:
        return None
    bbox = [float(value) for value in bbox_match.groups()]
    target_match = re.search(r'"target"\s*:\s*"([^"]*)"', fixed)
    confidence_match = re.search(r'"confidence"\s*:\s*([0-9.]+)', fixed)
    return {
        "visible": True,
        "bbox_norm": bbox,
        "confidence": float(confidence_match.group(1)) if confidence_match else 0.0,
        "target": target_match.group(1) if target_match else "target",
        "candidates": [],
    }


def normalize_bbox(bbox: Sequence[float], image_size: tuple[int, int]) -> list[int]:
    """Clamp bbox to image bounds, accepting pixel, normalized, or 0-1000 coordinates."""
    width, height = image_size
    if len(bbox) != 4:
        raise ValueError(f"bbox must contain 4 numbers, got {bbox}")

    values = [float(value) for value in bbox]
    if max(values) <= 1.5:
        x1, y1, x2, y2 = values
        values = [x1 * width, y1 * height, x2 * width, y2 * height]
    elif max(values) <= 1000 and (max(values[0], values[2]) > width or max(values[1], values[3]) > height):
        x1, y1, x2, y2 = values
        values = [x1 / 1000 * width, y1 / 1000 * height, x2 / 1000 * width, y2 / 1000 * height]

    x1, y1, x2, y2 = values
    left = int(round(max(0, min(x1, x2))))
    top = int(round(max(0, min(y1, y2))))
    right = int(round(min(width, max(x1, x2))))
    bottom = int(round(min(height, max(y1, y2))))
    if right <= left or bottom <= top:
        raise ValueError(f"invalid bbox after clamping: {bbox}")
    return [left, top, right, bottom]


def estimate_depth_in_bbox(
    depth_meters: np.ndarray,
    estimate: TargetDepthEstimate,
    bbox_image_size: Optional[tuple[int, int]] = None,
    min_depth: float = 0.1,
    max_depth: float = 1000.0,
) -> TargetDepthEstimate:
    """Fill depth statistics by sampling valid depth pixels inside bbox."""
    if not estimate.visible or not estimate.bbox:
        return estimate

    x1, y1, x2, y2 = scale_bbox_to_depth(estimate.bbox, bbox_image_size, depth_meters.shape)
    estimate.depth_bbox = [x1, y1, x2, y2]
    region = depth_meters[y1:y2, x1:x2]
    valid = region[(region > min_depth) & (region < max_depth) & np.isfinite(region)]
    if valid.size == 0:
        return estimate

    center_region = crop_center_region(region, ratio=0.5)
    center_valid = center_region[
        (center_region > min_depth) & (center_region < max_depth) & np.isfinite(center_region)
    ]

    estimate.depth_median = float(np.median(valid))
    estimate.depth_min = float(np.min(valid))
    estimate.depth_mean = float(np.mean(valid))
    if center_valid.size > 0:
        estimate.depth_center_median = float(np.median(center_valid))
        estimate.depth_median = estimate.depth_center_median
    estimate.valid_pixel_count = int(valid.size)
    return estimate


def crop_center_region(region: np.ndarray, ratio: float = 0.5) -> np.ndarray:
    """Crop the central part of a bbox to reduce road/background contamination."""
    height, width = region.shape
    crop_width = max(1, int(width * ratio))
    crop_height = max(1, int(height * ratio))
    left = max(0, (width - crop_width) // 2)
    top = max(0, (height - crop_height) // 2)
    return region[top:top + crop_height, left:left + crop_width]


def scale_bbox_to_depth(
    bbox: list[int],
    bbox_image_size: Optional[tuple[int, int]],
    depth_shape: tuple[int, int],
) -> list[int]:
    """Scale an RGB-image bbox onto the depth matrix resolution."""
    if not bbox_image_size:
        return bbox
    image_width, image_height = bbox_image_size
    depth_height, depth_width = depth_shape
    if image_width == depth_width and image_height == depth_height:
        return bbox
    x_scale = depth_width / image_width
    y_scale = depth_height / image_height
    x1, y1, x2, y2 = bbox
    return normalize_bbox(
        [x1 * x_scale, y1 * y_scale, x2 * x_scale, y2 * y_scale],
        (depth_width, depth_height),
    )


def choose_target_estimate(estimates: list[TargetDepthEstimate], task_description: str) -> TargetDepthEstimate:
    """Choose the target estimate, preferring farthest depth for far-target tasks."""
    visible = [estimate for estimate in estimates if estimate.visible]
    if not visible:
        return estimates[0] if estimates else TargetDepthEstimate(visible=False)
    far_keywords = ("较远", "远处", "远的", "更远", "最远", "far", "distant")
    if any(keyword in task_description.lower() for keyword in far_keywords):
        with_depth = [estimate for estimate in visible if estimate.depth_median is not None]
        if with_depth:
            return max(with_depth, key=lambda estimate: estimate.depth_median)
    return max(visible, key=lambda estimate: estimate.confidence)
