"""Target bbox detection and metric depth estimation helpers."""
import json
import re
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


TARGET_BBOX_PROMPT = """请观察这张RGB图，定位导航任务目标。

任务目标：{task_description}
当前图片尺寸：宽 {image_width} 像素，高 {image_height} 像素。

必须直接返回一行JSON对象，只包含这些字段：visible、bbox_norm、confidence、target。

规则：
1. visible 是 true 或 false。
2. bbox_norm 是4个数字组成的数组，范围 0.0 到 1.0，顺序是左、上、右、下。
3. confidence 是0.0到1.0之间的数字。
4. target 是目标名字符串，例如 car。
5. 如果画面中有多个同类目标，必须根据任务目标直接选择最终要导航的那个目标框，不要输出候选框列表。
6. 如果任务包含“较远/远处/远的/far”，bbox_norm 直接选择视觉上更远的目标，不要选择最近目标。
7. 如果目标不可见，输出 visible=false，bbox_norm=null，confidence=0.0，target=""。
8. 禁止输出 candidates 字段，禁止复述本提示词，禁止输出 x1、y1、x2、y2，禁止输出解释、Markdown、分析过程或动作规划。
9. 如果无法精确计算坐标，也必须基于视觉估计给出数字 bbox_norm，不要解释原因。"""

TARGET_BBOX_SYSTEM_PROMPT = """你是目标检测器。根据图片输出真实目标框JSON。
只输出一行JSON对象；禁止思考过程、解释、Markdown、动作规划、占位符。"""
SCENE_BBOX_SYSTEM_PROMPT = (
    "你是无人机场景检测器。只输出一个合法JSON对象，不要解释、不要Markdown、不要思考过程。"
)

SCENE_BBOX_PROMPT = """请观察RGB图，结合任务描述，输出导航目标和可能影响飞行的障碍物/参照物边界框。
任务：{task_description}
图像尺寸：宽 {image_width} 像素，高 {image_height} 像素。

只输出一个JSON对象，格式必须如下：
{{
  "target": {{"visible": true, "name": "car", "bbox_norm": [0.1,0.2,0.3,0.4], "confidence": 0.9, "position": "画面中央偏右/远处"}},
  "objects": [
    {{"role": "target", "name": "car", "bbox_norm": [0.1,0.2,0.3,0.4], "confidence": 0.9, "position": "画面中央偏右/远处", "description": "任务目标"}},
    {{"role": "obstacle", "name": "tree", "bbox_norm": [0.0,0.3,0.2,0.8], "confidence": 0.8, "position": "左前方", "description": "可能阻挡左侧绕行"}},
    {{"role": "reference", "name": "road", "bbox_norm": [0.2,0.5,0.8,1.0], "confidence": 0.7, "position": "前方道路", "description": "可通行区域"}}
  ]
}}

规则：
1. bbox_norm必须是0~1之间数字，顺序为[left, top, right, bottom]。
2. target必须是任务最终要到达或寻找的对象/语义地标/可通行区域；如果任务说电线杆/汽车/房屋等具体类别，只能选择该类别或同义词目标；如果任务说三岔路口、十字路口、道路尽头、路口穿越区域、空旷区域，则选择对应道路结构或可通行区域作为target。
3. 如果该任务目标类别或语义区域不可见，target.visible=false，target.bbox_norm=null；禁止用无关树、房屋、车辆等顶替目标，但道路/路口/空旷区域任务允许把对应道路区域作为target。
4. objects包含目标、明显障碍物、道路/路口/建筑等关键参照物；最多输出8个，优先输出会影响规划的物体。
5. 障碍物包括树、房屋、墙、车辆、杆子、近处大型物体；道路/空旷区域可标为reference。
6. 不要输出动作规划、任务阶段拆分，不要解释原因，只输出JSON。
"""


@dataclass
class TargetDepthEstimate:
    """Depth statistics for the VLM-detected target bbox."""
    visible: bool
    bbox: Optional[list[int]] = None
    confidence: float = 0.0
    target: str = ""
    role: str = "target"
    position: str = ""
    description: str = ""
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
            "target_role": self.role,
            "target_position": self.position,
            "target_description": self.description,
            "target_depth_median": self.depth_median,
            "target_depth_min": self.depth_min,
            "target_depth_mean": self.depth_mean,
            "target_depth_center_median": self.depth_center_median,
            "target_depth_valid_pixels": self.valid_pixel_count,
        }

    def scene_dict(self, index: int) -> dict:
        return {
            "id": f"obj{index}",
            "role": self.role or "obstacle",
            "name": self.target or "unknown",
            "position": self.position or "",
            "description": self.description or "",
            "bbox": self.bbox,
            "depth_bbox": self.depth_bbox,
            "confidence": self.confidence,
            "depth_median": self.depth_median,
            "valid_pixels": self.valid_pixel_count,
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


def build_scene_bbox_messages(base64_image: str, task_description: str,
                              image_size: tuple[int, int]) -> list[dict]:
    """Build a VLM request that returns target plus obstacle/reference bboxes."""
    prompt_text = SCENE_BBOX_PROMPT.format(
        task_description=task_description,
        image_width=image_size[0],
        image_height=image_size[1],
    ) + (
        "\n坐标硬性要求：所有 bbox_norm 必须是 [left, top, right, bottom]，"
        "四个值都必须在 0.0 到 1.0 之间；禁止输出 46、540、1000 等非归一化坐标。"
        "如果某个障碍物框不确定，就省略该障碍物，不要输出无效框。"
    )
    return [
        {"role": "system", "content": SCENE_BBOX_SYSTEM_PROMPT},
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/png;base64,{base64_image}"},
                },
                {
                    "type": "text",
                    "text": prompt_text,
                },
            ],
        },
    ]


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
        bbox_is_norm = bbox is not None
        if bbox is None:
            bbox = candidate.get("bbox") if isinstance(candidate, dict) else None
        if not bbox:
            continue
        if bbox_is_norm and not is_valid_normalized_bbox(bbox):
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
            role=str(candidate.get("role", "target") or "target"),
            position=str(candidate.get("position", "") or ""),
            description=str(candidate.get("description", "") or ""),
            raw_bbox=[float(value) for value in bbox],
        ))
    return estimates or [TargetDepthEstimate(visible=False)]


def parse_scene_candidates(response_text: str, image_size: tuple[int, int]) -> list[TargetDepthEstimate]:
    """Parse scene JSON with target and obstacle/reference objects."""
    text = re.sub(r"```json\s*", "", response_text)
    text = re.sub(r"```\s*", "", text)
    json_match = re.search(r"\{.*\}", text, re.DOTALL)
    if not json_match:
        raise ValueError(f"Cannot extract scene bbox JSON: {response_text[:200]}")

    raw_json = json_match.group()
    try:
        data = json.loads(raw_json, strict=False)
    except json.JSONDecodeError as exc:
        data = load_relaxed_json(raw_json)
        if data is None:
            raise ValueError(
                f"Cannot parse scene bbox JSON: {exc}; raw={raw_json[:300]}"
            ) from exc

    if "objects" not in data and ("bbox_norm" in data or "bbox" in data):
        return parse_target_candidates(response_text, image_size)

    raw_objects = []
    target = data.get("target") if isinstance(data.get("target"), dict) else None
    if target and target.get("visible", True) and (target.get("bbox_norm") or target.get("bbox")):
        target_obj = dict(target)
        target_obj.setdefault("role", "target")
        if "target" not in target_obj and "name" in target_obj:
            target_obj["target"] = target_obj["name"]
        raw_objects.append(target_obj)

    for obj in data.get("objects") or []:
        if isinstance(obj, dict):
            raw_objects.append(obj)

    estimates = []
    seen = set()
    for obj in raw_objects[:10]:
        bbox = obj.get("bbox_norm")
        bbox_is_norm = bbox is not None
        if bbox is None:
            bbox = obj.get("bbox")
        if not bbox:
            continue
        if bbox_is_norm and not is_valid_normalized_bbox(bbox):
            continue
        try:
            normalized = normalize_bbox(bbox, image_size)
            raw_bbox = [float(value) for value in bbox]
        except (TypeError, ValueError):
            continue
        key = tuple(normalized)
        if key in seen:
            continue
        seen.add(key)
        role = str(obj.get("role", "obstacle") or "obstacle").lower()
        name = str(obj.get("name", obj.get("target", "")) or "")
        estimates.append(TargetDepthEstimate(
            visible=True,
            bbox=normalized,
            confidence=float(obj.get("confidence", 0.0) or 0.0),
            target=name,
            role=role,
            position=str(obj.get("position", "") or ""),
            description=str(obj.get("description", "") or ""),
            raw_bbox=raw_bbox,
        ))

    if not estimates:
        return [TargetDepthEstimate(visible=False)]
    return estimates


def is_valid_normalized_bbox(bbox: Sequence[float]) -> bool:
    """Return True only for real [0,1] normalized bbox values."""
    try:
        values = [float(value) for value in bbox]
    except (TypeError, ValueError):
        return False
    return len(values) == 4 and all(0.0 <= value <= 1.0 for value in values)


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
    expanded_region = crop_expanded_bbox(depth_meters, [x1, y1, x2, y2], scale=2.5)
    expanded_valid = expanded_region[
        (expanded_region > min_depth) & (expanded_region < max_depth) & np.isfinite(expanded_region)
    ]

    estimate.depth_median = float(np.median(valid))
    estimate.depth_min = float(np.min(valid))
    estimate.depth_mean = float(np.mean(valid))
    robust_depths = [estimate.depth_median]
    if center_valid.size > 0:
        estimate.depth_center_median = float(np.median(center_valid))
        robust_depths.append(estimate.depth_center_median)
    if expanded_valid.size >= valid.size:
        robust_depths.append(float(np.percentile(expanded_valid, 20)))
    estimate.depth_median = float(min(robust_depths))
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


def crop_expanded_bbox(depth_meters: np.ndarray, bbox: list[int], scale: float = 2.5) -> np.ndarray:
    """Crop an expanded bbox to catch small distant objects missed by low-res depth."""
    x1, y1, x2, y2 = bbox
    height, width = depth_meters.shape
    box_width = max(1, x2 - x1)
    box_height = max(1, y2 - y1)
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    expanded_width = box_width * scale
    expanded_height = box_height * scale
    left = int(max(0, round(cx - expanded_width / 2)))
    right = int(min(width, round(cx + expanded_width / 2)))
    top = int(max(0, round(cy - expanded_height / 2)))
    bottom = int(min(height, round(cy + expanded_height / 2)))
    return depth_meters[top:bottom, left:right]


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
    px1 = int(round(x1 * x_scale)); py1 = int(round(y1 * y_scale))
    px2 = int(round(x2 * x_scale)); py2 = int(round(y2 * y_scale))
    left   = max(0, min(px1, px2))
    top    = max(0, min(py1, py2))
    right  = min(depth_width,  max(px1, px2))
    bottom = min(depth_height, max(py1, py2))
    if right <= left or bottom <= top:
        raise ValueError(f"invalid depth bbox after scaling: {bbox}")
    return [left, top, right, bottom]


def choose_target_estimate(estimates: list[TargetDepthEstimate], task_description: str) -> TargetDepthEstimate:
    """Choose the target estimate, preferring farthest depth for far-target tasks."""
    visible = [estimate for estimate in estimates if estimate.visible]
    if not visible:
        return estimates[0] if estimates else TargetDepthEstimate(visible=False)
    target_visible = [estimate for estimate in visible if (estimate.role or "").lower() == "target"]
    if not target_visible:
        return TargetDepthEstimate(visible=False)
    visible = target_visible
    desired_class = infer_task_target_class(task_description)
    if desired_class:
        matched = [estimate for estimate in visible if normalize_target_class(estimate.target) == desired_class]
        if matched:
            visible = matched
        else:
            return TargetDepthEstimate(visible=False, target=desired_class)
    far_keywords = ("较远", "远处", "远的", "更远", "最远", "far", "distant")
    if any(keyword in task_description.lower() for keyword in far_keywords):
        with_depth = [estimate for estimate in visible if estimate.depth_median is not None]
        if with_depth:
            return max(with_depth, key=lambda estimate: estimate.depth_median)
    return max(visible, key=lambda estimate: estimate.confidence)


def infer_task_target_class(task_description: str) -> str:
    """Infer a coarse target class from the task text for target-switch guards."""
    text = normalize_target_class(task_description)
    for canonical in ("pole", "car", "house", "tree", "road"):
        if canonical in text:
            return canonical
    return ""


def normalize_target_class(name: str) -> str:
    """Normalize common target aliases used by bbox VLMs."""
    text = str(name or "").strip().lower()
    text = text.replace("_", " ").replace("-", " ")
    compact = "".join(text.split())
    alias_groups = {
        "pole": (
            "pole", "telephonepole", "powerpole", "utilitypole", "electricpole",
            "streetpole", "lightpole", "lampost", "lamppost",
            "电线杆", "电杆", "杆子", "线杆", "路灯杆", "灯杆",
        ),
        "car": (
            "car", "vehicle", "automobile", "sedan", "suv", "truck",
            "汽车", "车辆", "小车", "轿车", "红色汽车", "蓝色汽车",
        ),
        "house": (
            "house", "home", "building", "residence", "住宅", "房屋", "房子", "建筑",
        ),
        "tree": ("tree", "trees", "树", "树木"),
        "road": ("road", "street", "道路", "街道", "路"),
    }
    for canonical, aliases in alias_groups.items():
        if compact in aliases or any(alias in compact for alias in aliases if len(alias) >= 2):
            return canonical
    return compact
