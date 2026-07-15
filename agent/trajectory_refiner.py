"""Trajectory refinement policies for target-following stages.

The planner remains responsible for proposing body-frame waypoints. This module
only applies relation-aware execution guards before AirSim receives them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, List, Optional

from agent.common.trajectory import normalize_xyz_waypoints


ABOVE_RELATIONS = {"above", "over", "on top", "on top of"}


@dataclass
class RefinementResult:
    waypoints: List[List[float]]
    changes: List[str] = field(default_factory=list)
    relation: str = ""
    camera: str = "none"
    depth: Optional[float] = None

    @property
    def changed(self) -> bool:
        return bool(self.changes)

    def summary(self) -> str:
        depth_text = "unknown" if self.depth is None else f"{self.depth:.1f}m"
        return (
            f"relation={self.relation or 'none'} camera={self.camera} "
            f"depth={depth_text} {', '.join(self.changes)}"
        )


class TrajectoryRefiner:
    """Apply small, explicit safety guards to planned body-frame waypoints."""

    def __init__(self, stop_threshold: float):
        self.stop_threshold = float(stop_threshold)

    def refine_result(
        self,
        result: Any,
        stage: Any,
        detection: Any,
        front_image=None,
        down_image=None,
    ) -> RefinementResult:
        raw_waypoints = normalize_xyz_waypoints(getattr(result, "waypoints", None))
        relation = self._stage_relation(stage)
        camera = str(getattr(detection, "camera", "none") or "none")
        depth = self._detection_depth(detection)
        output = RefinementResult(
            waypoints=raw_waypoints,
            relation=relation,
            camera=camera,
            depth=depth,
        )

        if result is None or stage is None:
            return output
        if not raw_waypoints or detection is None or not getattr(detection, "visible", False):
            return output
        if depth is None:
            return output

        waypoints = [list(wp) for wp in raw_waypoints]
        if relation in ABOVE_RELATIONS:
            waypoints, changes = self._refine_above(waypoints, detection, depth, front_image, down_image)
        else:
            waypoints, changes = self._refine_approach(waypoints, depth)

        output.waypoints = waypoints
        output.changes = changes
        if changes:
            result.waypoints = waypoints
        return output

    def _refine_approach(self, waypoints: List[List[float]], depth: float):
        margin = max(1.0, min(3.0, self.stop_threshold * 0.3))
        max_forward = max(0.0, depth - margin)
        waypoints, clipped = self._limit_forward_progress(waypoints, max_forward)
        return waypoints, [f"clip_x<={max_forward:.1f}m"] if clipped else []

    def _refine_above(self, waypoints, detection, depth: float, front_image, down_image):
        camera = str(getattr(detection, "camera", "front") or "front")
        changes: List[str] = []

        if camera == "front":
            overfly_margin = max(1.0, min(3.0, self.stop_threshold * 0.25))
            max_forward = max(1.0, depth + overfly_margin)
            waypoints, clipped = self._limit_forward_progress(waypoints, max_forward)
            if clipped:
                changes.append(f"overfly_x<={max_forward:.1f}m")

            bbox = getattr(detection, "bbox", None)
            _, cy = self._bbox_center_norm(bbox, front_image)
            area_ratio = self._bbox_area_ratio(bbox, front_image)
            close_visual = (
                area_ratio >= 0.10
                or cy >= 0.60
                or depth <= self.stop_threshold + 6.0
            )
            if close_visual:
                close_hold = max(1.5, min(max_forward, self.stop_threshold * 0.6))
                waypoints, close_clipped = self._limit_forward_progress(waypoints, close_hold)
                if close_clipped:
                    changes.append(f"front_hold<={close_hold:.1f}m")

            ascent_cap = 0.5 if close_visual else 1.5
            waypoints, ascent_limited = self._limit_upward_motion(waypoints, ascent_cap)
            if ascent_limited:
                changes.append(f"limit_up<={ascent_cap:.1f}m")
            return waypoints, changes

        if camera == "down":
            hold_radius = max(1.5, min(self.stop_threshold, 4.0))
            waypoints, clipped = self._limit_forward_progress(waypoints, hold_radius)
            if clipped:
                changes.append(f"down_hold<={hold_radius:.1f}m")
            descent = min(3.0, max(0.0, depth - self.stop_threshold))
            waypoints, descended = self._enforce_downward_approach(waypoints, descent)
            if descended:
                changes.append(f"descend={descent:.1f}m")
        return waypoints, changes

    @staticmethod
    def _stage_relation(stage) -> str:
        return str(getattr(stage, "relation", "") or "").strip().lower()

    @staticmethod
    def _detection_depth(detection) -> Optional[float]:
        value = getattr(detection, "depth_median", None)
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value) or value <= 0.0:
            return None
        return value

    @staticmethod
    def _bbox_center_norm(bbox, image) -> tuple[float, float]:
        if not bbox or image is None:
            return 0.5, 0.5
        x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
        cx = ((x1 + x2) * 0.5) / max(float(getattr(image, "width", 1)), 1.0)
        cy = ((y1 + y2) * 0.5) / max(float(getattr(image, "height", 1)), 1.0)
        return cx, cy

    @staticmethod
    def _bbox_area_ratio(bbox, image) -> float:
        if not bbox or image is None:
            return 0.0
        x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
        width = max(float(getattr(image, "width", 1)), 1.0)
        height = max(float(getattr(image, "height", 1)), 1.0)
        box_w = max(0.0, x2 - x1)
        box_h = max(0.0, y2 - y1)
        return max(0.0, min(1.0, (box_w * box_h) / (width * height)))

    @staticmethod
    def _limit_forward_progress(waypoints, max_forward: float):
        if not waypoints:
            return waypoints, False
        max_forward = max(0.0, float(max_forward))
        end_x = float(waypoints[-1][0])
        if end_x <= max_forward + 1e-6 or end_x <= 1e-6:
            return waypoints, False
        scale = max_forward / end_x
        clipped = []
        for wp in waypoints:
            clipped.append([
                round(float(wp[0]) * scale, 3),
                round(float(wp[1]) * scale, 3),
                round(float(wp[2]), 3),
            ])
        return clipped, True

    @staticmethod
    def _limit_upward_motion(waypoints, max_ascent_m: float):
        if not waypoints:
            return waypoints, False
        min_z = -max(0.0, float(max_ascent_m))
        changed = False
        adjusted = []
        for wp in waypoints:
            z = float(wp[2])
            if z < min_z:
                z = min_z
                changed = True
            adjusted.append([round(float(wp[0]), 3), round(float(wp[1]), 3), round(z, 3)])
        return adjusted, changed

    @staticmethod
    def _enforce_downward_approach(waypoints, descent_m: float):
        if not waypoints:
            return waypoints, False
        descent_m = max(0.0, float(descent_m))
        if descent_m <= 1e-6:
            return waypoints, False
        final_z = float(waypoints[-1][2])
        if final_z >= descent_m - 1e-6:
            return waypoints, False
        total = len(waypoints)
        adjusted = []
        for idx, wp in enumerate(waypoints):
            ratio = float(idx + 1) / float(total)
            desired_z = descent_m * ratio
            adjusted.append([
                round(float(wp[0]), 3),
                round(float(wp[1]), 3),
                round(max(float(wp[2]), desired_z), 3),
            ])
        return adjusted, True
