"""Structured context memory for closed-loop aerial VLM navigation."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional

import math

from planner.trajectory import parse_action


class NavigationState(str, Enum):
    """High-level closed-loop navigation state."""

    SEARCHING = "SEARCHING"
    APPROACHING = "APPROACHING"
    ARRIVED = "ARRIVED"
    RECOVERING = "RECOVERING"


class ObservationStatus(str, Enum):
    """State-machine result for the latest target observation."""

    INIT_DETECT = "INIT_DETECT"
    TRACKING = "TRACKING"
    NEAR_GOAL_CONFIRM = "NEAR_GOAL_CONFIRM"
    LOST_OR_OCCLUDED = "LOST_OR_OCCLUDED"
    RELOCALIZE = "RELOCALIZE"
    REDETECT = "REDETECT"


@dataclass
class ObservationDecision:
    """Action gates derived from target memory and the latest observation."""

    status: ObservationStatus = ObservationStatus.REDETECT
    accepted: bool = False
    reason: str = ""
    target_depth: Optional[float] = None
    expected_depth: Optional[float] = None


@dataclass
class TargetMemory:
    """Persistent memory for the selected navigation target."""

    target_class: str = ""
    bbox: Optional[List[int]] = None
    depth_median: Optional[float] = None
    confidence: float = 0.0
    visible_streak: int = 0
    lost_streak: int = 0
    last_seen_step: int = 0
    last_bbox_step: int = 0
    last_seen_yaw: Optional[float] = None
    target_bearing_deg: Optional[float] = None
    target_world_yaw: Optional[float] = None
    target_world_pos: Optional[List[float]] = None
    target_world_pos_step: int = 0
    bbox_area_ratio: Optional[float] = None
    accumulated_forward_at_last_seen: float = 0.0
    last_seen_pose: Optional[List[float]] = None
    depth_history: List[float] = field(default_factory=list)
    bbox_history: List[List[int]] = field(default_factory=list)

    @property
    def has_target(self) -> bool:
        return self.bbox is not None and self.depth_median is not None


@dataclass
class TaskMemory:
    """Task progress memory, similar to a lightweight TODO memory."""

    instruction: str = ""
    accumulated_forward_distance: float = 0.0
    executed_actions: List[str] = field(default_factory=list)
    current_state: NavigationState = NavigationState.SEARCHING
    last_replan_step: int = 0


class ContextManager:
    """Manages only target memory and a lightweight task state machine."""

    def __init__(self, config=None, max_steps: int = 5):
        self.config = config
        self.max_steps = max_steps
        self.messages: List[Dict[str, str]] = []
        self.target = TargetMemory()
        self.task = TaskMemory()
        self.last_decision = ObservationDecision()

    def ensure_task(self, instruction: str):
        instruction = (instruction or "").strip()
        if instruction and instruction != self.task.instruction:
            self.clear()
            self.task.instruction = instruction

    def add_step(self, step_num: int, actions: List[str],
                 old_pos, new_pos, yaw_deg: float, collided: bool):
        """Record one executed step for both structured memory and VLM context."""
        self.task.executed_actions.extend(actions or [])
        self.task.accumulated_forward_distance += self._sum_forward_distance(actions or [])

        summary = (
            f"[Step {step_num}] executed={actions}; "
            f"pose=({old_pos[0]:.2f},{old_pos[1]:.2f},{old_pos[2]:.2f})"
            f"->({new_pos[0]:.2f},{new_pos[1]:.2f},{new_pos[2]:.2f}); "
            f"yaw={yaw_deg:.1f}deg; collided={bool(collided)}; "
            f"target_state={self.task.current_state.value}; "
            f"target_depth={self._fmt(self.target.depth_median)}; "
            f"target_world_pos={self._fmt_pos(self.target.target_world_pos)}; "
            f"lost_streak={self.target.lost_streak}; "
            f"acc_forward={self.task.accumulated_forward_distance:.2f}m."
        )
        self.messages.append({"role": "user", "content": summary})
        self.messages.append({
            "role": "assistant",
            "content": "已记录执行结果，下一步继续遵循目标记忆和任务状态。",
        })
        max_pairs = self.max_steps * 2
        if len(self.messages) > max_pairs:
            self.messages = self.messages[-max_pairs:]

    def get_messages(self) -> List[Dict[str, str]]:
        compact = self.build_compact_context()
        return compact + list(self.messages)

    def clear(self):
        self.messages = []
        self.target = TargetMemory()
        self.task = TaskMemory()
        self.last_decision = ObservationDecision()

    def build_compact_context(self) -> List[Dict[str, str]]:
        """Return a short textual memory block for the planning VLM."""
        if not self.context_enabled:
            return []
        target = (
            f"class={self.target.target_class or 'unknown'}, "
            f"bbox={self.target.bbox}, depth={self._fmt(self.target.depth_median)}, "
            f"bearing={self._fmt_deg(self.target.target_bearing_deg)}, "
            f"world_yaw={self._fmt_deg(self.target.target_world_yaw)}, "
            f"world_pos={self._fmt_pos(self.target.target_world_pos)}, "
            f"visible_streak={self.target.visible_streak}, "
            f"lost_streak={self.target.lost_streak}, "
            f"last_seen_step={self.target.last_seen_step}"
        )
        task = (
            f"instruction={self.task.instruction or 'unknown'}, "
            f"state={self.task.current_state.value}, "
            f"accumulated_forward={self.task.accumulated_forward_distance:.2f}m"
        )
        content = (
            "上下文记忆：\n"
            f"- 任务进度：{task}\n"
            f"- 目标记忆：{target}\n"
            "要求：不要因为单帧误检轻易切换目标；目标短暂丢失时不要让 planner 追逐新的同类目标。"
        )
        return [{"role": "user", "content": content}]

    def annotate_estimate_geometry(self, estimate: Optional[Dict[str, Any]],
                                   image_size=None, yaw_deg: Optional[float] = None,
                                   pose=None):
        """Add bearing/world-yaw/world-position/bbox-area fields to a target estimate."""
        if not estimate or not estimate.get("target_visible"):
            return estimate
        bbox = estimate.get("target_bbox")
        if not bbox or not image_size:
            return estimate
        image_width, image_height = image_size
        x1, y1, x2, y2 = bbox
        center_x = ((x1 + x2) / 2.0) / max(1, image_width)
        center_y = ((y1 + y2) / 2.0) / max(1, image_height)
        fov = float(getattr(self.config, "context_camera_fov_deg", 90.0))
        bearing = (center_x - 0.5) * fov
        area = max(0.0, (x2 - x1) * (y2 - y1)) / max(1.0, image_width * image_height)
        estimate["target_bearing_deg"] = float(bearing)
        estimate["target_center_norm"] = [float(center_x), float(center_y)]
        estimate["target_bbox_area_ratio"] = float(area)
        if yaw_deg is not None:
            world_yaw = self._normalize_angle(float(yaw_deg) + float(bearing))
            estimate["target_world_yaw"] = world_yaw
            depth = estimate.get("target_depth_median")
            if pose is not None and depth is not None:
                estimate["target_world_pos"] = self.estimate_target_world_position(
                    pose, world_yaw, float(depth)
                )
        return estimate

    def estimate_target_world_position(self, pose, world_yaw_deg: float,
                                       depth_m: float) -> Optional[List[float]]:
        """Estimate target XY world position from drone pose, target yaw and depth."""
        if pose is None or depth_m is None:
            return None
        try:
            x, y, z = float(pose[0]), float(pose[1]), float(pose[2])
            depth = max(0.0, float(depth_m))
            yaw_rad = math.radians(float(world_yaw_deg))
            return [
                x + depth * math.cos(yaw_rad),
                y + depth * math.sin(yaw_rad),
                z,
            ]
        except Exception:
            return None

    def update_obstacle_memory(self, depth_meters, image_size=None, target_bbox=None):
        """Compatibility no-op: obstacle/corridor memory was removed from ContextManager."""
        return None

    def should_try_reuse_target_bbox(self, step_num: int) -> bool:
        """Compatibility no-op: bbox reuse is intentionally disabled."""
        return False

    def accept_reused_depth(self, depth_median: Optional[float], valid_pixels: int) -> bool:
        return False

    def evaluate_observation(self, estimate: Optional[Dict[str, Any]], step_num: int,
                             pose=None, yaw_deg: Optional[float] = None,
                             task_description: str = "") -> ObservationDecision:
        """Update target memory and report the lightweight tracking state."""
        visible = bool(
            estimate
            and estimate.get("target_visible")
            and estimate.get("target_depth_median") is not None
        )
        current_depth = float(estimate["target_depth_median"]) if visible else None

        if not self.target.has_target:
            if visible:
                self.update_target_from_estimate(estimate, step_num, pose, yaw_deg, source="bbox_api")
                decision = ObservationDecision(
                    status=ObservationStatus.INIT_DETECT,
                    accepted=True,
                    reason="initial target locked",
                    target_depth=current_depth,
                    expected_depth=current_depth,
                )
            else:
                self.update_target_from_estimate(None, step_num, pose, yaw_deg, source="no_initial_target")
                decision = ObservationDecision(
                    status=ObservationStatus.REDETECT,
                    accepted=False,
                    reason="no target memory and no reliable observation",
                )
            self.last_decision = decision
            return decision

        expected_depth = self.expected_target_depth()

        if not visible:
            self.mark_suspicious_detection(step_num, pose, yaw_deg, estimate=None)
            decision = ObservationDecision(
                status=ObservationStatus.LOST_OR_OCCLUDED,
                accepted=False,
                reason="target not visible",
                expected_depth=expected_depth,
            )
            self.last_decision = decision
            return decision

        class_match = self._class_matches(estimate.get("target_name"))

        if class_match:
            if not self.should_accept_detection(estimate, step_num):
                self.mark_suspicious_detection(step_num, pose, yaw_deg, estimate=estimate)
                decision = ObservationDecision(
                    status=ObservationStatus.LOST_OR_OCCLUDED,
                    accepted=False,
                    reason="current same-class detection does not match locked target memory",
                    target_depth=current_depth,
                    expected_depth=expected_depth,
                )
                self.last_decision = decision
                return decision
            self.update_target_from_estimate(estimate, step_num, pose, yaw_deg, source="bbox_api")
            decision = ObservationDecision(
                status=ObservationStatus.TRACKING,
                accepted=True,
                reason="current target visible; planner API required",
                target_depth=current_depth,
                expected_depth=expected_depth,
            )
            self.last_decision = decision
            return decision

        self.mark_suspicious_detection(step_num, pose, yaw_deg, estimate=estimate)
        decision = ObservationDecision(
            status=ObservationStatus.LOST_OR_OCCLUDED,
            accepted=False,
            reason="current detection class differs from locked target",
            target_depth=current_depth,
            expected_depth=expected_depth,
        )
        self.last_decision = decision
        return decision

    def should_accept_detection(self, estimate: Dict[str, Any], step_num: int) -> bool:
        """Reject suspicious single-frame target switches using motion-compensated checks."""
        if not self.context_enabled:
            return True
        if not estimate or not estimate.get("target_visible"):
            return True
        if not self.target.has_target:
            return True
        old_depth = self.target.depth_median
        new_depth = estimate.get("target_depth_median")
        if old_depth is None or new_depth is None:
            return True
        class_match = self._class_matches(estimate.get("target_name"))

        expected_depth = self.expected_target_depth()
        expected_error = abs(float(new_depth) - float(expected_depth))
        expected_tol = max(5.0, float(old_depth) * 0.25)
        direction_ok = self._direction_consistent(estimate)
        if class_match and expected_error <= expected_tol:
            return True
        if class_match and direction_ok and expected_error <= expected_tol * 1.8:
            return True

        diff = abs(float(new_depth) - float(expected_depth))
        ratio = diff / max(1.0, float(old_depth))
        if step_num - self.target.last_seen_step <= 3 and ratio > 0.60 and not direction_ok:
            return False
        return True

    def update_target_from_estimate(self, estimate: Optional[Dict[str, Any]], step_num: int,
                                    pose=None, yaw_deg: Optional[float] = None,
                                    source: str = "bbox_api"):
        """Update target/search state from a target-depth estimate."""
        visible = bool(estimate and estimate.get("target_visible") and estimate.get("target_depth_median") is not None)
        if visible:
            self.target.target_class = estimate.get("target_name") or self.target.target_class or "target"
            self.target.bbox = estimate.get("target_bbox") or self.target.bbox
            self.target.depth_median = float(estimate.get("target_depth_median"))
            self.target.confidence = float(estimate.get("target_confidence") or self.target.confidence or 0.0)
            self.target.visible_streak += 1
            self.target.lost_streak = 0
            self.target.last_seen_step = step_num
            if source == "bbox_api":
                self.target.last_bbox_step = step_num
            self.target.last_seen_yaw = yaw_deg
            self.target.target_bearing_deg = estimate.get("target_bearing_deg", self.target.target_bearing_deg)
            self.target.target_world_yaw = estimate.get("target_world_yaw", self.target.target_world_yaw)
            new_world_pos = estimate.get("target_world_pos")
            if new_world_pos is not None:
                self.target.target_world_pos = self._smooth_world_pos(new_world_pos)
                self.target.target_world_pos_step = step_num
            self.target.bbox_area_ratio = estimate.get("target_bbox_area_ratio", self.target.bbox_area_ratio)
            self.target.accumulated_forward_at_last_seen = self.task.accumulated_forward_distance
            self.target.last_seen_pose = list(pose) if pose is not None else None
            self.target.depth_history.append(self.target.depth_median)
            if self.target.bbox:
                self.target.bbox_history.append(list(self.target.bbox))
            self.target.depth_history = self.target.depth_history[-10:]
            self.target.bbox_history = self.target.bbox_history[-10:]
            self.task.current_state = NavigationState.APPROACHING
            return

        self.target.visible_streak = 0
        if self.target.has_target:
            self.target.lost_streak += 1
            self.task.current_state = NavigationState.RECOVERING
        else:
            self.task.current_state = NavigationState.SEARCHING

    def mark_suspicious_detection(self, step_num: int, pose=None, yaw_deg: Optional[float] = None,
                                  estimate: Optional[Dict[str, Any]] = None):
        """Treat a rejected one-frame target switch as temporary target loss."""
        self.target.visible_streak = 0
        if self.target.has_target:
            self.target.lost_streak += 1
            self.task.current_state = NavigationState.RECOVERING
        else:
            self.task.current_state = NavigationState.SEARCHING

    def record_scan_candidate(self, estimate: Dict[str, Any], step_num: int,
                              yaw_deg: Optional[float], suspicious: bool = False):
        """Compatibility no-op: relocalization owns scan candidates."""
        return

    def expected_target_depth(self) -> float:
        """Predict current target depth from last depth minus executed forward distance."""
        if self.target.depth_median is None:
            return 0.0
        moved_forward = max(
            0.0,
            self.task.accumulated_forward_distance - self.target.accumulated_forward_at_last_seen,
        )
        return max(0.0, float(self.target.depth_median) - moved_forward)

    def _expected_depth_tolerance(self) -> float:
        base_depth = float(self.target.depth_median or 0.0)
        return max(5.0, base_depth * 0.25)

    def score_candidate(self, estimate: Dict[str, Any]) -> float:
        """Score a detected candidate against target memory."""
        if not estimate:
            return 0.0
        confidence = max(0.0, min(1.0, float(estimate.get("target_confidence") or 0.0)))
        class_score = 1.0 if self._class_matches(estimate.get("target_name")) else 0.0
        direction_score = 0.5
        if estimate.get("target_world_yaw") is not None and self.target.target_world_yaw is not None:
            diff = abs(self._normalize_angle(float(estimate["target_world_yaw"]) - float(self.target.target_world_yaw)))
            tolerance = 35.0
            direction_score = max(0.0, 1.0 - diff / max(1.0, tolerance))
        depth_score = 0.5
        if estimate.get("target_depth_median") is not None and self.target.depth_median is not None:
            expected = self.expected_target_depth()
            error = abs(float(estimate["target_depth_median"]) - expected)
            tolerance = 5.0
            depth_score = max(0.0, 1.0 - error / max(1.0, tolerance * 2.0))
        return 0.35 * confidence + 0.25 * class_score + 0.25 * direction_score + 0.15 * depth_score

    def _class_matches(self, target_name: Optional[str]) -> bool:
        if not self.target.target_class:
            return True
        if not target_name:
            return True
        return self._normalize_target_class(target_name) == self._normalize_target_class(self.target.target_class)

    @staticmethod
    def _normalize_target_class(name: Optional[str]) -> str:
        """Normalize common multilingual/synonym target names for tracking."""
        text = str(name or "").strip().lower()
        text = text.replace("_", " ").replace("-", " ")
        compact = "".join(text.split())
        if not compact:
            return ""
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

    def _direction_consistent(self, estimate: Dict[str, Any]) -> bool:
        if self.target.target_world_yaw is None or estimate.get("target_world_yaw") is None:
            return True
        diff = abs(self._normalize_angle(float(estimate["target_world_yaw"]) - float(self.target.target_world_yaw)))
        return diff <= 35.0

    def _smooth_world_pos(self, new_pos) -> Optional[List[float]]:
        if new_pos is None:
            return self.target.target_world_pos
        try:
            new_values = [
                float(new_pos[0]),
                float(new_pos[1]),
                float(new_pos[2]) if len(new_pos) > 2 else 0.0,
            ]
        except Exception:
            return self.target.target_world_pos
        old = self.target.target_world_pos
        if old is None:
            return new_values
        alpha = float(getattr(self.config, "context_world_pos_update_alpha", 0.35))
        alpha = max(0.0, min(1.0, alpha))
        return [
            float(old[0]) * (1.0 - alpha) + new_values[0] * alpha,
            float(old[1]) * (1.0 - alpha) + new_values[1] * alpha,
            float(old[2]) * (1.0 - alpha) + new_values[2] * alpha,
        ]

    @property
    def context_enabled(self) -> bool:
        return bool(getattr(self.config, "context_enabled", True))

    def _sum_forward_distance(self, actions: List[str]) -> float:
        total = 0.0
        for action in actions:
            try:
                name, value = parse_action(action, self.config)
            except Exception:
                continue
            if name == "forward":
                total += float(value)
        return total

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return (angle + 180.0) % 360.0 - 180.0

    @staticmethod
    def _fmt_pos(value) -> str:
        if value is None:
            return "None"
        try:
            return f"({float(value[0]):.2f},{float(value[1]):.2f},{float(value[2]):.2f})"
        except Exception:
            return str(value)

    @staticmethod
    def _fmt(value: Optional[float]) -> str:
        return "None" if value is None else f"{float(value):.2f}m"

    @staticmethod
    def _fmt_deg(value: Optional[float]) -> str:
        return "None" if value is None else f"{float(value):.1f}deg"
