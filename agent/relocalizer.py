"""Independent 360-degree visual relocalizer for lost targets.

The relocalizer does not directly decide navigation. It asks the VLM to mark
possible task-target candidates in each panorama view, computes depth/world
coordinates for each candidate, then selects the candidate closest to the
locked target position when such memory is available.
"""
from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .target_depth import TargetDepthEstimate, estimate_depth_in_bbox, normalize_bbox


RELOCALIZER_SYSTEM_PROMPT = """你是无人机目标重定位候选检测器。
你只负责在多张环视RGB图中找出“可能是任务目标”的可疑目标框。
不要做路径规划，不要输出动作，不要选择最终方向，只输出合法JSON。"""


RELOCALIZER_USER_PROMPT = """任务目标：{task_description}

参考坐标信息（单位：米）：
{coord_hint}

无人机会原地环视多个方向。图片按顺序给出：
{view_descriptions}

请对每张图分别输出所有“可能是任务目标”的可疑目标，最多每张图3个。
如果某张图没有可疑任务目标，该图 targets 为空数组。
只输出JSON，格式如下：
{{
  "views": [
    {{
      "view_index": 0,
      "targets": [
        {{"name": "car", "bbox_norm": [0.1, 0.2, 0.3, 0.4], "confidence": 0.85, "reason": "可能是任务目标"}}
      ]
    }}
  ]
}}

要求：
1. view_index 必须对应输入图片编号。
2. bbox_norm 必须是[left, top, right, bottom]，每个值在0到1之间。
3. 只输出和任务目标语义相关的可疑目标；障碍物、道路、树等不要放进 targets。
4. 不要输出 forward/left/right/up/down。
5. 不要解释，不要Markdown。"""


@dataclass
class RelocalizeView:
    index: int
    yaw: float
    image: Any
    depth_meters: Any = None
    pose: Optional[List[float]] = None


@dataclass
class RelocalizeResult:
    found: bool
    view_index: Optional[int] = None
    target_yaw: Optional[float] = None
    bbox_norm: Optional[List[float]] = None
    confidence: float = 0.0
    reason: str = ""
    raw_response: str = ""
    elapsed: float = 0.0
    selected_world_pos: Optional[List[float]] = None
    selected_depth: Optional[float] = None
    candidates: List[Dict[str, Any]] = field(default_factory=list)


class Relocalizer:
    """Capture panorama views and select the best target candidate."""

    def __init__(self, client, vlm, prompter, config):
        self.client = client
        self.vlm = vlm
        self.prompter = prompter
        self.config = config

    def run(self, task_description: str,
            locked_world_pos: Optional[List[float]] = None) -> RelocalizeResult:
        started = time.perf_counter()
        _, start_yaw = self.client.get_pose()
        view_count = int(getattr(self.config, "relocalizer_view_count", 4))
        yaw_step = float(getattr(self.config, "relocalizer_yaw_step_deg", 90.0))
        views = self._capture_views(start_yaw, view_count, yaw_step)
        self.client.rotate_to_yaw(start_yaw)

        response_text, reasoning_content = self.vlm.call(
            self._build_messages(task_description, views, locked_world_pos=locked_world_pos),
            max_tokens=int(getattr(self.config, "relocalizer_max_tokens", 512)),
        )
        raw_response = response_text or reasoning_content
        result = self._parse_response(raw_response, views, locked_world_pos)
        result.elapsed = time.perf_counter() - started
        return result

    def _capture_views(self, start_yaw: float, view_count: int, yaw_step: float) -> List[RelocalizeView]:
        views = []
        for index in range(max(1, view_count)):
            target_yaw = self._normalize_yaw(start_yaw + index * yaw_step)
            if index > 0:
                self.client.rotate_to_yaw(target_yaw)
                time.sleep(float(getattr(self.config, "relocalizer_settle_seconds", 0.3)))
            image, depth_meters = self.client.get_scene_and_depth_meters()
            pose, actual_yaw = self.client.get_pose()
            views.append(RelocalizeView(
                index=index,
                yaw=float(actual_yaw),
                image=image,
                depth_meters=depth_meters,
                pose=pose,
            ))
        return views

    def _build_messages(self, task_description: str, views: List[RelocalizeView],
                            locked_world_pos: Optional[List[float]] = None) -> List[Dict[str, Any]]:
        view_descriptions = "\n".join(
            f"- view_index={view.index}: yaw={view.yaw:.1f}度"
            for view in views
        )
        coord_parts = []
        current_pose = views[0].pose if views and views[0].pose else None
        if current_pose:
            coord_parts.append(
                f"当前无人机世界坐标 x={current_pose[0]:.2f}, y={current_pose[1]:.2f}, z={current_pose[2]:.2f}"
            )
        if locked_world_pos:
            coord_parts.append(
                f"目标锁定世界坐标 x={locked_world_pos[0]:.2f}, y={locked_world_pos[1]:.2f}, z={locked_world_pos[2]:.2f}"
            )
        coord_hint = "\n".join(coord_parts) if coord_parts else "无可用坐标信息"
        user_text = RELOCALIZER_USER_PROMPT.format(
            task_description=task_description,
            coord_hint=coord_hint,
            view_descriptions=view_descriptions,
        )
        content: List[Dict[str, Any]] = [{"type": "text", "text": user_text}]
        for view in views:
            content.append({"type": "text", "text": f"view_index={view.index}, yaw={view.yaw:.1f}度"})
            content.append({
                "type": "image_url",
                "image_url": {"url": f"data:image/png;base64,{self.prompter.encode_image(view.image)}"},
            })
        return [
            {"role": "system", "content": RELOCALIZER_SYSTEM_PROMPT},
            {"role": "user", "content": content},
        ]

    def _parse_response(self, response_text: str, views: List[RelocalizeView],
                        locked_world_pos: Optional[List[float]]) -> RelocalizeResult:
        data = self._loads_json(response_text)
        candidates = self._build_world_candidates(data, views, locked_world_pos)
        threshold = float(getattr(self.config, "relocalizer_confidence_threshold", 0.5))
        candidates = [candidate for candidate in candidates if candidate["confidence"] >= threshold]
        selected = self._select_candidate(candidates, locked_world_pos)
        if selected is None:
            return RelocalizeResult(
                found=False,
                reason="no relocalizer candidates",
                raw_response=response_text,
                candidates=candidates,
            )
        return RelocalizeResult(
            found=True,
            view_index=selected["view_index"],
            target_yaw=selected["world_yaw"],
            bbox_norm=selected["bbox_norm"],
            confidence=selected["confidence"],
            reason=selected.get("reason", ""),
            raw_response=response_text,
            selected_world_pos=selected["world_pos"],
            selected_depth=selected["depth_median"],
            candidates=candidates,
        )

    def _build_world_candidates(self, data: Dict[str, Any], views: List[RelocalizeView],
                                locked_world_pos: Optional[List[float]]) -> List[Dict[str, Any]]:
        raw_views = data.get("views")
        if raw_views is None and "view_index" in data:
            raw_views = [{
                "view_index": data.get("view_index"),
                "targets": [{
                    "name": data.get("name", data.get("target", "target")),
                    "bbox_norm": data.get("bbox_norm"),
                    "confidence": data.get("confidence", 0.0),
                    "reason": data.get("reason", ""),
                }],
            }]

        view_by_index = {view.index: view for view in views}
        candidates: List[Dict[str, Any]] = []
        for raw_view in raw_views or []:
            if not isinstance(raw_view, dict):
                continue
            try:
                view_index = int(raw_view.get("view_index"))
            except (TypeError, ValueError):
                continue
            view = view_by_index.get(view_index)
            if view is None or view.image is None or view.depth_meters is None:
                continue
            for raw_target in raw_view.get("targets") or []:
                if isinstance(raw_target, dict):
                    candidate = self._target_to_world_candidate(raw_target, view, locked_world_pos)
                    if candidate is not None:
                        candidates.append(candidate)
        return candidates

    def _target_to_world_candidate(self, raw_target: Dict[str, Any], view: RelocalizeView,
                                   locked_world_pos: Optional[List[float]]) -> Optional[Dict[str, Any]]:
        bbox_norm = self._valid_bbox(raw_target.get("bbox_norm"))
        if not bbox_norm:
            return None
        try:
            bbox = normalize_bbox(bbox_norm, view.image.size)
        except Exception:
            return None
        estimate = TargetDepthEstimate(
            visible=True,
            bbox=bbox,
            confidence=float(raw_target.get("confidence", 0.0) or 0.0),
            target=str(raw_target.get("name", raw_target.get("target", "target")) or "target"),
            raw_bbox=bbox_norm,
        )
        estimate_depth_in_bbox(view.depth_meters, estimate, bbox_image_size=view.image.size)
        if estimate.depth_median is None:
            return None
        world_yaw = self._bbox_world_yaw(bbox, view.image.size, view.yaw)
        world_pos = self._world_pos_from_depth(view.pose, world_yaw, estimate.depth_median)
        if world_pos is None:
            return None
        return {
            "view_index": view.index,
            "view_yaw": view.yaw,
            "name": estimate.target,
            "bbox_norm": bbox_norm,
            "bbox": bbox,
            "confidence": estimate.confidence,
            "depth_median": estimate.depth_median,
            "world_yaw": world_yaw,
            "world_pos": world_pos,
            "distance_to_lock": self._distance(world_pos, locked_world_pos),
            "reason": str(raw_target.get("reason", "") or ""),
        }

    def _select_candidate(self, candidates: List[Dict[str, Any]],
                          locked_world_pos: Optional[List[float]]) -> Optional[Dict[str, Any]]:
        if not candidates:
            return None
        if locked_world_pos is not None:
            with_distance = [candidate for candidate in candidates if candidate.get("distance_to_lock") is not None]
            if with_distance:
                return min(
                    with_distance,
                    key=lambda candidate: (
                        float(candidate["distance_to_lock"]) - 2.0 * float(candidate["confidence"]),
                        -float(candidate["confidence"]),
                    ),
                )
        return max(candidates, key=lambda candidate: (float(candidate["confidence"]), -float(candidate["depth_median"])))

    def _bbox_world_yaw(self, bbox: List[int], image_size, view_yaw: float) -> float:
        image_width, _ = image_size
        center_x = ((bbox[0] + bbox[2]) / 2.0) / max(1, image_width)
        fov = float(getattr(self.config, "context_camera_fov_deg", 90.0))
        bearing = (center_x - 0.5) * fov
        return self._normalize_yaw(float(view_yaw) + bearing)

    @staticmethod
    def _world_pos_from_depth(pose, world_yaw: float, depth_m: float) -> Optional[List[float]]:
        if pose is None:
            return None
        try:
            yaw_rad = math.radians(float(world_yaw))
            return [
                float(pose[0]) + float(depth_m) * math.cos(yaw_rad),
                float(pose[1]) + float(depth_m) * math.sin(yaw_rad),
                float(pose[2]),
            ]
        except Exception:
            return None

    @staticmethod
    def _distance(pos_a, pos_b) -> Optional[float]:
        if pos_a is None or pos_b is None:
            return None
        try:
            dx = float(pos_a[0]) - float(pos_b[0])
            dy = float(pos_a[1]) - float(pos_b[1])
            dz = float(pos_a[2]) - float(pos_b[2]) if len(pos_a) > 2 and len(pos_b) > 2 else 0.0
            return math.sqrt(dx * dx + dy * dy + dz * dz)
        except Exception:
            return None

    @staticmethod
    def _loads_json(response_text: str) -> Dict[str, Any]:
        text = re.sub(r"```json\s*", "", response_text or "")
        text = re.sub(r"```\s*", "", text)
        match = re.search(r"\{.*\}", text, re.DOTALL)
        if not match:
            return {"views": []}
        raw = match.group()
        try:
            return json.loads(raw, strict=False)
        except json.JSONDecodeError:
            fixed = raw.replace("'", '"')
            fixed = re.sub(r",\s*([}\]])", r"\1", fixed)
            try:
                return json.loads(fixed, strict=False)
            except json.JSONDecodeError:
                return {"views": []}

    @staticmethod
    def _valid_bbox(bbox) -> Optional[List[float]]:
        if not isinstance(bbox, list) or len(bbox) != 4:
            return None
        try:
            values = [float(value) for value in bbox]
        except (TypeError, ValueError):
            return None
        if not all(0.0 <= value <= 1.0 for value in values):
            return None
        return values

    @staticmethod
    def _normalize_yaw(yaw: float) -> float:
        return (float(yaw) + 180.0) % 360.0 - 180.0
