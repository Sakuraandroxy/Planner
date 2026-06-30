"""Structured context memory for closed-loop aerial VLM navigation."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional

import math

import numpy as np

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
    allow_local_forward: bool = False
    allow_auto_done: bool = False
    should_call_planning_api: bool = True
    recovery_action: Optional[List[str]] = None
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


@dataclass
class SearchMemory:
    """Memory used during target-loss rotation/search."""

    active: bool = False
    scan_start_yaw: Optional[float] = None
    scan_steps: int = 0
    seen_candidates: List[Dict[str, Any]] = field(default_factory=list)
    suspicious_candidates: List[Dict[str, Any]] = field(default_factory=list)
    best_seen_target: Optional[Dict[str, Any]] = None


@dataclass
class ObstacleMemory:
    """Recent local depth-channel safety memory."""

    front_safe_depth: Optional[float] = None
    target_corridor_safe_depth: Optional[float] = None
    blocked: bool = False
    blocked_reason: str = ""


class ContextManager:
    """Manages target/task/search/obstacle memory and compact VLM context."""

    def __init__(self, config=None, max_steps: int = 5):
        self.config = config
        self.max_steps = max_steps
        self.messages: List[Dict[str, str]] = []
        self.target = TargetMemory()
        self.task = TaskMemory()
        self.search = SearchMemory()
        self.obstacle = ObstacleMemory()
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
            "content": "已记录执行结果，下一步继续遵循目标记忆和安全深度约束。",
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
        self.search = SearchMemory()
        self.obstacle = ObstacleMemory()
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
        obstacle = (
            f"front_safe_depth={self._fmt(self.obstacle.front_safe_depth)}, "
            f"target_corridor_safe_depth={self._fmt(self.obstacle.target_corridor_safe_depth)}, "
            f"blocked={self.obstacle.blocked}, reason={self.obstacle.blocked_reason or 'none'}"
        )
        search = "inactive"
        if self.search.active:
            search = (
                f"active, scan_steps={self.search.scan_steps}, "
                f"best_seen={self.search.best_seen_target}"
            )
        content = (
            "上下文记忆：\n"
            f"- 任务进度：{task}\n"
            f"- 目标记忆：{target}\n"
            f"- 局部障碍：{obstacle}\n"
            f"- 搜索记忆：{search}\n"
            "要求：不要因为单帧误检轻易切换目标；目标短暂丢失时优先回到历史目标方向；"
            "只有前方/目标方向深度通道安全时才允许连续前进。"
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
        """Update local depth-channel safety from the latest raw depth matrix."""
        if depth_meters is None:
            self.obstacle = ObstacleMemory()
            return self.obstacle
        height, width = depth_meters.shape
        front_half_width = float(getattr(self.config, "context_front_corridor_half_width", 0.03))
        front_y_min = float(getattr(self.config, "context_front_corridor_y_min", 0.45))
        front_y_max = float(getattr(self.config, "context_front_corridor_y_max", 0.60))
        target_half_width = float(getattr(self.config, "context_target_corridor_half_width", 0.05))
        target_y_min = float(getattr(self.config, "context_target_corridor_y_min", 0.45))
        target_y_max = float(getattr(self.config, "context_target_corridor_y_max", 0.65))
        front = self._depth_percentile(
            depth_meters,
            x1=int(width * (0.50 - front_half_width)),
            x2=int(width * (0.50 + front_half_width)),
            y1=int(height * front_y_min),
            y2=int(height * front_y_max),
        )
        target_safe = None
        if target_bbox and image_size:
            image_width, image_height = image_size
            bx1, by1, bx2, by2 = target_bbox
            cx = ((bx1 + bx2) / 2.0) / max(1, image_width)
            dx = int(width * target_half_width)
            center_x = int(width * cx)
            target_safe = self._depth_percentile(
                depth_meters,
                x1=max(0, center_x - dx),
                x2=min(width, center_x + dx),
                y1=int(height * target_y_min),
                y2=int(height * target_y_max),
            )
        safe_depth = target_safe if target_safe is not None else front
        min_safe = float(getattr(self.config, "context_obstacle_min_safe_depth", 4.0))
        blocked = safe_depth is not None and safe_depth < min_safe
        reason = f"safe_depth<{min_safe:.1f}m" if blocked else ""
        self.obstacle = ObstacleMemory(
            front_safe_depth=front,
            target_corridor_safe_depth=target_safe,
            blocked=blocked,
            blocked_reason=reason,
        )
        return self.obstacle

    def should_try_reuse_target_bbox(self, step_num: int) -> bool:
        """Conservative bbox reuse: only when previous motion was small/recent."""
        if not self.context_enabled or not self.reuse_bbox_enabled:
            return False
        if not self.target.has_target:
            return False
        if self.task.current_state not in (NavigationState.APPROACHING, NavigationState.RECOVERING):
            return False
        if self.target.lost_streak > 0:
            return False
        max_age = int(getattr(self.config, "context_bbox_reuse_max_steps", 1))
        if step_num - self.target.last_bbox_step > max_age:
            return False
        max_forward = float(getattr(self.config, "context_bbox_reuse_max_forward", 2.0))
        return self._recent_forward_distance() <= max_forward

    def accept_reused_depth(self, depth_median: Optional[float], valid_pixels: int) -> bool:
        if depth_median is None or valid_pixels <= 0:
            return False
        if self.target.depth_median is None:
            return True
        if depth_median > self.target.depth_median * float(getattr(self.config, "context_depth_jump_ratio", 1.5)):
            return False
        return True

    def evaluate_observation(self, estimate: Optional[Dict[str, Any]], step_num: int,
                             pose=None, yaw_deg: Optional[float] = None,
                             task_description: str = "") -> ObservationDecision:
        """State-machine gate for target tracking, recovery and local actions."""
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
                    allow_local_forward=not self.is_arrival_depth(current_depth),
                    allow_auto_done=(
                        self.is_arrival_depth(current_depth)
                        and self.can_auto_complete_by_depth(task_description)
                    ),
                    should_call_planning_api=False,
                    reason="initial target locked",
                    target_depth=current_depth,
                    expected_depth=current_depth,
                )
            else:
                self.update_target_from_estimate(None, step_num, pose, yaw_deg, source="no_initial_target")
                decision = ObservationDecision(
                    status=ObservationStatus.REDETECT,
                    accepted=False,
                    allow_local_forward=False,
                    should_call_planning_api=True,
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
                allow_local_forward=False,
                allow_auto_done=False,
                should_call_planning_api=False,
                recovery_action=self.build_recovery_actions(yaw_deg, current_pose=pose),
                reason="target not visible; context recovery required",
                expected_depth=expected_depth,
            )
            self.last_decision = decision
            return decision

        class_match = self._class_matches(estimate.get("target_name"))
        near_current = self.is_arrival_depth(current_depth)

        if class_match:
            if not self.should_accept_detection(estimate, step_num):
                self.mark_suspicious_detection(step_num, pose, yaw_deg, estimate=estimate)
                decision = ObservationDecision(
                    status=ObservationStatus.LOST_OR_OCCLUDED,
                    accepted=False,
                    allow_local_forward=False,
                    allow_auto_done=False,
                    should_call_planning_api=False,
                    recovery_action=self.build_recovery_actions(yaw_deg, current_pose=pose),
                    reason="current same-class detection does not match locked target memory; context recovery required",
                    target_depth=current_depth,
                    expected_depth=expected_depth,
                )
                self.last_decision = decision
                return decision
            self.update_target_from_estimate(estimate, step_num, pose, yaw_deg, source="bbox_api")
            decision = ObservationDecision(
                status=ObservationStatus.NEAR_GOAL_CONFIRM if near_current else ObservationStatus.TRACKING,
                accepted=True,
                allow_local_forward=False,
                allow_auto_done=False,
                should_call_planning_api=True,
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
            allow_local_forward=False,
            allow_auto_done=False,
            should_call_planning_api=False,
            recovery_action=self.build_recovery_actions(yaw_deg, current_pose=pose),
            reason="current detection class differs from locked target; context recovery required",
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
        if not self.target.has_target or self.target.lost_streak >= self.max_lost_before_redetect:
            return True
        old_depth = self.target.depth_median
        new_depth = estimate.get("target_depth_median")
        if old_depth is None or new_depth is None:
            return True
        class_match = self._class_matches(estimate.get("target_name"))

        expected_depth = self.expected_target_depth()
        expected_error = abs(float(new_depth) - float(expected_depth))
        expected_tol = max(
            float(getattr(self.config, "context_expected_depth_abs_tolerance", 5.0)),
            float(old_depth) * float(getattr(self.config, "context_expected_depth_ratio_tolerance", 0.25)),
        )
        direction_ok = self._direction_consistent(estimate)
        if class_match and expected_error <= expected_tol:
            return True
        if class_match and direction_ok and expected_error <= expected_tol * 1.8:
            return True

        diff = abs(float(new_depth) - float(expected_depth))
        ratio = diff / max(1.0, float(old_depth))
        max_ratio = float(getattr(self.config, "context_switch_depth_ratio", 0.60))
        max_recent_steps = int(getattr(self.config, "context_switch_guard_steps", 3))
        if step_num - self.target.last_seen_step <= max_recent_steps and ratio > max_ratio and not direction_ok:
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
            self.record_scan_candidate(estimate, step_num, yaw_deg)
            self.search.active = False
            return

        self.target.visible_streak = 0
        if self.target.has_target:
            self.target.lost_streak += 1
            self.task.current_state = NavigationState.RECOVERING
            if not self.search.active:
                self.search = SearchMemory(active=True, scan_start_yaw=yaw_deg)
            self.search.scan_steps += 1
        else:
            self.task.current_state = NavigationState.SEARCHING

    def mark_suspicious_detection(self, step_num: int, pose=None, yaw_deg: Optional[float] = None,
                                  estimate: Optional[Dict[str, Any]] = None):
        """Treat a rejected one-frame target switch as temporary target loss."""
        if estimate and estimate.get("target_visible"):
            self.record_scan_candidate(estimate, step_num, yaw_deg, suspicious=True)
        self.target.visible_streak = 0
        if self.target.has_target:
            self.target.lost_streak += 1
            self.task.current_state = NavigationState.RECOVERING
            if not self.search.active:
                self.search = SearchMemory(active=True, scan_start_yaw=yaw_deg)
            self.search.scan_steps += 1
        else:
            self.task.current_state = NavigationState.SEARCHING

    def record_scan_candidate(self, estimate: Dict[str, Any], step_num: int,
                              yaw_deg: Optional[float], suspicious: bool = False):
        if not estimate or not estimate.get("target_visible"):
            return
        candidate = {
            "step": step_num,
            "yaw": yaw_deg,
            "target": estimate.get("target_name"),
            "bbox": estimate.get("target_bbox"),
            "depth": estimate.get("target_depth_median"),
            "bearing": estimate.get("target_bearing_deg"),
            "world_yaw": estimate.get("target_world_yaw"),
            "world_pos": estimate.get("target_world_pos"),
            "bbox_area_ratio": estimate.get("target_bbox_area_ratio"),
            "confidence": estimate.get("target_confidence", 0.0),
            "score": self.score_candidate(estimate),
        }
        self.search.seen_candidates.append(candidate)
        self.search.seen_candidates = self.search.seen_candidates[-8:]
        if suspicious:
            self.search.suspicious_candidates.append(candidate)
            self.search.suspicious_candidates = self.search.suspicious_candidates[-8:]
        best = self.search.best_seen_target
        if best is None or float(candidate.get("score") or 0.0) >= float(best.get("score") or 0.0):
            self.search.best_seen_target = candidate

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
        return max(
            float(getattr(self.config, "context_expected_depth_abs_tolerance", 5.0)),
            base_depth * float(getattr(self.config, "context_expected_depth_ratio_tolerance", 0.25)),
        )

    def score_candidate(self, estimate: Dict[str, Any]) -> float:
        """Score a detected candidate against target memory."""
        if not estimate:
            return 0.0
        confidence = max(0.0, min(1.0, float(estimate.get("target_confidence") or 0.0)))
        class_score = 1.0 if self._class_matches(estimate.get("target_name")) else 0.0
        direction_score = 0.5
        if estimate.get("target_world_yaw") is not None and self.target.target_world_yaw is not None:
            diff = abs(self._normalize_angle(float(estimate["target_world_yaw"]) - float(self.target.target_world_yaw)))
            tolerance = float(getattr(self.config, "context_direction_tolerance_deg", 35.0))
            direction_score = max(0.0, 1.0 - diff / max(1.0, tolerance))
        depth_score = 0.5
        if estimate.get("target_depth_median") is not None and self.target.depth_median is not None:
            expected = self.expected_target_depth()
            error = abs(float(estimate["target_depth_median"]) - expected)
            tolerance = float(getattr(self.config, "context_expected_depth_abs_tolerance", 5.0))
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
        tolerance = float(getattr(self.config, "context_direction_tolerance_deg", 35.0))
        return diff <= tolerance

    def should_use_local_forward(self, target_depth: Optional[float]) -> bool:
        if not self.context_enabled or not self.local_forward_enabled:
            return False
        if target_depth is None:
            return False
        if not self.last_decision.allow_local_forward:
            return False
        if self.task.current_state != NavigationState.APPROACHING:
            return False
        if self.target.lost_streak > 0 or self.obstacle.blocked:
            return False
        min_depth = float(getattr(self.config, "context_local_forward_min_depth", 20.0))
        return float(target_depth) >= min_depth

    def build_local_forward_actions(self, target_depth: float, distance_scale: float = 1.0) -> List[str]:
        """Generate safe segmented forward actions without calling the planning VLM."""
        max_step = float(getattr(self.config, "max_forward_step", 10.0))
        max_len = int(getattr(self.config, "max_trajectory_length", 5))
        goal_margin = float(getattr(self.config, "context_goal_safety_margin", 2.0))
        obstacle_margin = float(getattr(self.config, "context_obstacle_safety_margin", 2.0))
        target_limit = max(0.0, float(target_depth) - goal_margin)
        corridor = self.obstacle.target_corridor_safe_depth or self.obstacle.front_safe_depth
        corridor_limit = target_limit
        if corridor is not None:
            corridor_limit = max(0.0, min(corridor_limit, float(corridor) - obstacle_margin))
        total = min(target_limit, corridor_limit, max_step * max_len) * max(0.0, min(1.0, distance_scale))
        if total < float(getattr(self.config, "context_local_forward_min_action", 2.0)):
            return []
        actions = []
        remaining = total
        while remaining > 0.05 and len(actions) < max_len:
            value = min(max_step, remaining)
            actions.append(f"forward {round(value, 2)}")
            remaining -= value
        return actions

    def build_recovery_actions(self, current_yaw: Optional[float], current_pose=None) -> List[str]:
        """Return a yaw action toward target world position, then fan-scan around it."""
        if not self.context_enabled or not self.recovery_enabled:
            return []
        if not self.target.has_target:
            return []
        if self.target.lost_streak <= 0 or self.target.lost_streak > self.max_lost_before_redetect:
            return []
        step = float(getattr(self.config, "context_search_yaw_step_deg", 15.0))
        anchor_yaw = self._relocalization_anchor_yaw(current_pose)
        if current_yaw is None or anchor_yaw is None:
            return [f"left {round(step, 2)}"]

        offsets = self._fan_scan_offsets(step)
        start_index = max(0, self.search.scan_steps - 1)
        for offset in offsets[start_index:] + offsets[:start_index]:
            target_yaw = self._normalize_angle(anchor_yaw + offset)
            diff = self._normalize_angle(target_yaw - float(current_yaw))
            if abs(diff) > 2.0:
                value = min(step, abs(diff))
                return [f"right {round(value, 2)}"] if diff > 0 else [f"left {round(value, 2)}"]
        return [f"left {round(step, 2)}"]

    def build_search_variant_actions(self, index: int) -> List[str]:
        """Candidate display variants for recovery; all are in-place rotations."""
        step = float(getattr(self.config, "context_search_yaw_step_deg", 15.0))
        values = [
            ("left", step),
            ("right", step),
            ("left", step * 2),
            ("right", step * 2),
            ("left", step * 3),
        ]
        direction, value = values[index % len(values)]
        return [f"{direction} {round(value, 2)}"]

    def can_auto_complete_by_depth(self, task_description: str) -> bool:
        """Only near/beside/approach goals can be completed by distance alone."""
        text = (task_description or "").lower()
        near_keywords = (
            "旁", "旁边", "附近", "接近", "靠近", "到达目标", "飞到目标",
            "near", "beside", "next to", "close to", "approach",
        )
        relation_keywords = (
            "上方", "上面", "顶部", "屋顶", "楼顶", "下方", "下面",
            "左侧", "右侧", "后方", "前方", "绕过", "穿过", "越过",
            "above", "over", "top", "roof", "below", "behind",
            "left side", "right side", "pass", "through", "around",
        )
        return any(keyword in text for keyword in near_keywords) and not any(
            keyword in text for keyword in relation_keywords
        )

    def is_arrival_depth(self, target_depth: Optional[float]) -> bool:
        if target_depth is None:
            return False
        threshold = float(getattr(self.config, "context_arrival_depth", 5.0))
        return float(target_depth) <= threshold

    def _relocalization_anchor_yaw(self, current_pose=None) -> Optional[float]:
        best = self.search.best_seen_target
        min_score = float(getattr(self.config, "context_candidate_anchor_min_score", 0.55))
        if best and best.get("world_pos") is not None and float(best.get("score") or 0.0) >= min_score:
            yaw = self._yaw_from_pose_to_world_pos(current_pose, best.get("world_pos"))
            if yaw is not None:
                return yaw
        if self.target.target_world_pos is not None:
            yaw = self._yaw_from_pose_to_world_pos(current_pose, self.target.target_world_pos)
            if yaw is not None:
                return yaw
        if best and best.get("world_yaw") is not None and float(best.get("score") or 0.0) >= min_score:
            return float(best["world_yaw"])
        if self.target.target_world_yaw is not None:
            return float(self.target.target_world_yaw)
        if self.target.last_seen_yaw is not None:
            return float(self.target.last_seen_yaw)
        return None

    def _yaw_from_pose_to_world_pos(self, current_pose, world_pos) -> Optional[float]:
        if current_pose is None or world_pos is None:
            return None
        try:
            dx = float(world_pos[0]) - float(current_pose[0])
            dy = float(world_pos[1]) - float(current_pose[1])
            if abs(dx) < 1e-3 and abs(dy) < 1e-3:
                return None
            return self._normalize_angle(math.degrees(math.atan2(dy, dx)))
        except Exception:
            return None

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

    @staticmethod
    def _fan_scan_offsets(step: float) -> List[float]:
        return [0.0, -step, step, -2 * step, 2 * step, -3 * step, 3 * step, -4 * step, 4 * step]

    @property
    def context_enabled(self) -> bool:
        return bool(getattr(self.config, "context_enabled", True))

    @property
    def reuse_bbox_enabled(self) -> bool:
        return bool(getattr(self.config, "context_reuse_bbox_enabled", True))

    @property
    def local_forward_enabled(self) -> bool:
        return bool(getattr(self.config, "context_local_forward_enabled", True))

    @property
    def recovery_enabled(self) -> bool:
        return bool(getattr(self.config, "context_recovery_enabled", True))

    @property
    def max_lost_before_redetect(self) -> int:
        return int(getattr(self.config, "context_max_lost_before_redetect", 2))

    def _recent_forward_distance(self) -> float:
        return self._sum_forward_distance(self.task.executed_actions[-5:])

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
    def _depth_percentile(depth_meters, x1: int, x2: int, y1: int, y2: int,
                          percentile: float = 10.0) -> Optional[float]:
        region = depth_meters[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
        valid = region[(region > 0.1) & (region < 1000.0) & np.isfinite(region)]
        if valid.size == 0:
            return None
        return float(np.percentile(valid, percentile))

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
