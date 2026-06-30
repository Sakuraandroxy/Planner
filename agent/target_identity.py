"""Target instance lock for closed-loop navigation.

This module owns target identity. The bbox VLM may propose the most likely
task target in the current frame, but it is not allowed to silently replace the
instance that was locked at the beginning of the task.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class IdentityDecision:
    """Result of comparing the current bbox result with the locked target."""

    accepted: bool
    reason: str
    locked: bool = False
    distance_m: Optional[float] = None
    tolerance_m: Optional[float] = None


@dataclass
class LockedTarget:
    """Persistent identity of the first task target instance."""

    task: str = ""
    name: str = ""
    first_step: int = 0
    last_seen_step: int = 0
    bbox: Optional[List[int]] = None
    depth_median: Optional[float] = None
    world_pos: Optional[List[float]] = None
    world_yaw: Optional[float] = None
    bearing_deg: Optional[float] = None
    confidence: float = 0.0
    lost_count: int = 0
    history: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def has_lock(self) -> bool:
        return self.world_pos is not None or self.bbox is not None


class TargetIdentityManager:
    """Locks the first target and rejects later same-class target switches."""

    def __init__(self, config=None):
        self.config = config
        self.lock = LockedTarget()

    def clear(self):
        self.lock = LockedTarget()

    def ensure_task(self, task_description: str):
        task_description = (task_description or "").strip()
        if task_description and task_description != self.lock.task:
            self.clear()
            self.lock.task = task_description

    def evaluate(self, estimate: Optional[Dict[str, Any]], step_num: int,
                 task_description: str = "") -> IdentityDecision:
        """Accept the first visible target, then gate all later observations."""
        self.ensure_task(task_description)
        if not self.enabled:
            return IdentityDecision(accepted=True, reason="target identity disabled")

        visible = bool(
            estimate
            and estimate.get("target_visible")
            and estimate.get("target_depth_median") is not None
        )
        if not visible:
            if self.lock.has_lock:
                self.lock.lost_count += 1
            return IdentityDecision(
                accepted=False,
                locked=self.lock.has_lock,
                reason="no reliable target observation",
            )

        if not self.lock.has_lock:
            self._lock_from_estimate(estimate, step_num, task_description)
            return IdentityDecision(
                accepted=True,
                locked=True,
                reason="initial target instance locked",
            )

        same_instance, distance, tolerance, match_reason = self._same_world_instance(estimate)
        if same_instance:
            self._update_from_estimate(estimate, step_num)
            return IdentityDecision(
                accepted=True,
                locked=True,
                distance_m=distance,
                tolerance_m=tolerance,
                reason=match_reason,
            )

        self.lock.lost_count += 1
        return IdentityDecision(
            accepted=False,
            locked=True,
            distance_m=distance,
            tolerance_m=tolerance,
            reason="candidate is a different target instance",
        )

    def reject_estimate(self, estimate: Dict[str, Any], decision: IdentityDecision) -> Dict[str, Any]:
        """Return a copy marked invisible while preserving diagnostics."""
        rejected = dict(estimate or {})
        rejected["target_visible"] = False
        rejected["target_identity_rejected"] = True
        rejected["target_identity_reason"] = decision.reason
        rejected["target_identity_distance_m"] = decision.distance_m
        rejected["target_identity_tolerance_m"] = decision.tolerance_m
        rejected["locked_target_world_pos"] = list(self.lock.world_pos) if self.lock.world_pos else None
        rejected["locked_target_name"] = self.lock.name
        return rejected

    def distance_to_locked_target(self, pose) -> Optional[float]:
        """Return XY distance from current drone pose to the locked target."""
        if pose is None or self.lock.world_pos is None:
            return None
        try:
            dx = float(self.lock.world_pos[0]) - float(pose[0])
            dy = float(self.lock.world_pos[1]) - float(pose[1])
            return math.sqrt(dx * dx + dy * dy)
        except Exception:
            return None

    def yaw_to_locked_target(self, pose) -> Optional[float]:
        """Return world yaw from current drone pose toward the locked target."""
        if pose is None or self.lock.world_pos is None:
            return None
        try:
            dx = float(self.lock.world_pos[0]) - float(pose[0])
            dy = float(self.lock.world_pos[1]) - float(pose[1])
            if abs(dx) < 1e-3 and abs(dy) < 1e-3:
                return None
            return math.degrees(math.atan2(dy, dx))
        except Exception:
            return None

    def is_near_locked_target(self, pose) -> bool:
        """Return True when the drone is already close enough to the locked target."""
        distance = self.distance_to_locked_target(pose)
        if distance is None:
            return False
        radius = float(getattr(self.config, "target_identity_arrival_radius", 6.0))
        return distance <= radius

    def is_yaw_consistent_with_lock(self, yaw_deg: Optional[float], pose) -> bool:
        """Check whether a relocalizer view points toward the locked target."""
        if yaw_deg is None:
            return False
        expected_yaw = self.yaw_to_locked_target(pose)
        if expected_yaw is None:
            return True
        diff = abs(self._normalize_angle(float(yaw_deg) - float(expected_yaw)))
        tolerance = float(getattr(self.config, "target_identity_relocalizer_yaw_tolerance_deg", 60.0))
        return diff <= tolerance

    def _lock_from_estimate(self, estimate: Dict[str, Any], step_num: int, task_description: str):
        self.lock = LockedTarget(task=(task_description or "").strip())
        self._copy_estimate_to_lock(estimate, step_num, reset_lost=True)

    def _update_from_estimate(self, estimate: Dict[str, Any], step_num: int):
        self._copy_estimate_to_lock(estimate, step_num, reset_lost=True)

    def _copy_estimate_to_lock(self, estimate: Dict[str, Any], step_num: int, reset_lost: bool):
        world_pos = estimate.get("target_world_pos")
        if world_pos is not None:
            world_pos = [float(world_pos[0]), float(world_pos[1]), float(world_pos[2]) if len(world_pos) > 2 else 0.0]
        old_world_pos = self.lock.world_pos
        if old_world_pos is not None and world_pos is not None:
            alpha = float(getattr(self.config, "target_identity_update_alpha", 0.35))
            alpha = max(0.0, min(1.0, alpha))
            world_pos = [
                old_world_pos[0] * (1.0 - alpha) + world_pos[0] * alpha,
                old_world_pos[1] * (1.0 - alpha) + world_pos[1] * alpha,
                old_world_pos[2] * (1.0 - alpha) + world_pos[2] * alpha,
            ]
        self.lock.name = str(estimate.get("target_name") or self.lock.name or "target")
        self.lock.bbox = list(estimate.get("target_bbox") or self.lock.bbox or [])
        self.lock.depth_median = float(estimate.get("target_depth_median"))
        self.lock.world_pos = world_pos or self.lock.world_pos
        self.lock.world_yaw = estimate.get("target_world_yaw", self.lock.world_yaw)
        self.lock.bearing_deg = estimate.get("target_bearing_deg", self.lock.bearing_deg)
        self.lock.confidence = float(estimate.get("target_confidence") or self.lock.confidence or 0.0)
        if self.lock.first_step <= 0:
            self.lock.first_step = step_num
        self.lock.last_seen_step = step_num
        if reset_lost:
            self.lock.lost_count = 0
        self.lock.history.append({
            "step": step_num,
            "name": self.lock.name,
            "bbox": self.lock.bbox,
            "depth": self.lock.depth_median,
            "world_pos": self.lock.world_pos,
            "world_yaw": self.lock.world_yaw,
            "bearing": self.lock.bearing_deg,
            "confidence": self.lock.confidence,
        })
        self.lock.history = self.lock.history[-10:]

    def _same_world_instance(self, estimate: Dict[str, Any]) -> tuple[bool, Optional[float], Optional[float], str]:
        """Accept only candidates inside a configurable world-distance gate.

        Depth-derived world positions are noisy, especially for small/far targets,
        so the gate is deliberately a radius instead of an exact match. Accepted
        observations then update the locked position with momentum smoothing.
        """
        locked_pos = self.lock.world_pos
        current_pos = estimate.get("target_world_pos")
        if locked_pos is None or current_pos is None:
            return True, None, None, "world position unavailable; keep current visible target"
        try:
            dx = float(current_pos[0]) - float(locked_pos[0])
            dy = float(current_pos[1]) - float(locked_pos[1])
            dz = float(current_pos[2]) - float(locked_pos[2]) if len(current_pos) > 2 else 0.0
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        except Exception:
            return True, None, None, "world position invalid; keep current visible target"

        depth = estimate.get("target_depth_median")
        reference_depth = max(float(depth or 0.0), float(self.lock.depth_median or 0.0), 1.0)
        tolerance = max(
            float(getattr(self.config, "target_identity_world_tolerance_abs", 15.0)),
            reference_depth * float(getattr(self.config, "target_identity_world_tolerance_ratio", 0.35)),
        )
        if distance <= tolerance:
            return True, distance, tolerance, "candidate is inside locked target distance gate"

        return False, distance, tolerance, "candidate is a different target instance"

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return (float(angle) + 180.0) % 360.0 - 180.0

    @property
    def enabled(self) -> bool:
        return bool(getattr(self.config, "target_identity_enabled", True))
