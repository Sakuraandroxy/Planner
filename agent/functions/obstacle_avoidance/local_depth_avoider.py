"""Lightweight local obstacle avoidance from depth and sparse memory.

This module intentionally keeps the map coarse: it stores a bounded number of
world-frame obstacle cells, not raw images, depth maps, or dense point clouds.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, List, Sequence

from agent.functions.memory.geometry import world_to_body
from agent.functions.obstacle_avoidance.schemas import AvoidanceResult, ObstacleCell


@dataclass
class _CollisionHit:
    segment_index: int
    total_distance_m: float
    segment_t: float
    path_point: List[float]
    obstacle_body: List[float]
    source: str = "depth"


class DepthObstacleAvoider:
    """Depth-based local safety layer between VLM planning and AirSim execution."""

    def __init__(self, config: dict | None = None, sim_config: dict | None = None):
        config = config or {}
        sim_config = sim_config or {}
        self.config = dict(config)
        self.sim_config = dict(sim_config)
        self.enabled = bool(self.config.get("ENABLED", True))
        # obstacle_cells 是稀疏局部地图：key是粗网格坐标，value只保存一个代表性障碍点。
        self.obstacle_cells: dict[str, ObstacleCell] = {}
        self.last_update_s = 0.0
        self.last_filter_reason = ""

    def reset(self) -> None:
        self.obstacle_cells.clear()
        self.last_update_s = 0.0
        self.last_filter_reason = ""

    def needs_plan_depth(self) -> bool:
        return bool(self.enabled and self.config.get("CAPTURE_DEPTH_ON_PLAN", True))

    def update_from_depth(
        self,
        *,
        front_depth_meters=None,
        down_depth_meters=None,
        observer_world: Sequence[float],
        observer_yaw_deg: float,
    ) -> int:
        """Fuse a depth snapshot into the sparse local obstacle memory."""
        if not self.enabled:
            return 0
        added_or_updated = 0
        for point_body, view in self._iter_depth_points(front_depth_meters, down_depth_meters):
            if not self._accept_depth_point(point_body):
                continue
            world = self._body_to_world(point_body, observer_world, observer_yaw_deg)
            key = self._cell_key(world)
            depth = max(float(point_body[0]), 0.1)
            conf = 1.0 - min(0.85, depth / max(float(self.config.get("FRONT_MAX_DEPTH_M", 30.0)), 1.0))
            existing = self.obstacle_cells.get(key)
            if existing is None:
                self.obstacle_cells[key] = ObstacleCell(
                    key=key,
                    center_world=[round(float(v), 4) for v in world],
                    confidence=max(0.15, conf),
                    observation_count=1,
                    source_view=view,
                )
            else:
                alpha = 0.35
                existing.center_world = [
                    round((1.0 - alpha) * float(existing.center_world[i]) + alpha * float(world[i]), 4)
                    for i in range(3)
                ]
                existing.confidence = max(float(existing.confidence), min(0.99, 0.75 * existing.confidence + 0.25 * conf))
                existing.observation_count += 1
                existing.last_seen_s = time.perf_counter()
                existing.source_view = view
            added_or_updated += 1

        self.last_update_s = time.perf_counter()
        self._prune()
        return added_or_updated

    def filter_cumulative_waypoints(
        self,
        cumulative_waypoints: Iterable[Sequence[float]] | None,
        *,
        current_world: Sequence[float],
        yaw_deg: float,
        memory_context: dict | None = None,
    ) -> AvoidanceResult:
        """Return a locally safe replacement for cumulative body-frame waypoints."""
        original = self._normalize_waypoints(cumulative_waypoints)
        if not self.enabled or not original:
            return AvoidanceResult(waypoints=original)

        target_result = self._filter_target_keepout(original, memory_context)
        if target_result.changed:
            self.last_filter_reason = target_result.reason
            return target_result

        obstacles = self._active_obstacles_body(current_world, yaw_deg, memory_context=memory_context)
        if not obstacles:
            return AvoidanceResult(waypoints=original)

        hit = self._first_collision(original, obstacles)
        if hit is None:
            return AvoidanceResult(waypoints=original)

        if self._obstacle_matches_locked_target(hit.obstacle_body, memory_context):
            stopped = self._stop_before_hit(original, hit)
            result = AvoidanceResult(
                waypoints=stopped,
                changed=True,
                blocked=not bool(stopped),
                reason="stop_before_target_depth_obstacle",
                obstacle_body=[round(float(v), 2) for v in hit.obstacle_body],
            )
            self.last_filter_reason = result.reason
            return result

        if bool(self.config.get("BYPASS_ENABLED", True)):
            bypass = self._try_bypass(original, hit, obstacles, memory_context)
            if bypass is not None:
                result = AvoidanceResult(
                    waypoints=bypass,
                    changed=True,
                    reason="depth_lateral_bypass",
                    obstacle_body=[round(float(v), 2) for v in hit.obstacle_body],
                    details={"segment": hit.segment_index},
                )
                self.last_filter_reason = result.reason
                return result

        stopped = self._stop_before_hit(original, hit)
        result = AvoidanceResult(
            waypoints=stopped,
            changed=True,
            blocked=not bool(stopped),
            reason="stop_before_depth_obstacle",
            obstacle_body=[round(float(v), 2) for v in hit.obstacle_body],
            details={"segment": hit.segment_index},
        )
        self.last_filter_reason = result.reason
        return result

    def summary(self) -> dict:
        cells = sorted(
            self.obstacle_cells.values(),
            key=lambda cell: (cell.age_s(), -cell.confidence),
        )
        return {
            "enabled": self.enabled,
            "cells": len(self.obstacle_cells),
            "last_filter": self.last_filter_reason,
            "sample": [cell.to_summary_dict() for cell in cells[:5]],
        }

    def _iter_depth_points(self, front_depth, down_depth):
        if front_depth is not None:
            yield from self._iter_front_depth_points(front_depth)
        if bool(self.config.get("USE_DOWN_DEPTH", False)) and down_depth is not None:
            yield from self._iter_down_depth_points(down_depth)

    def _iter_front_depth_points(self, depth):
        try:
            import numpy as np

            arr = np.asarray(depth)
            h, w = arr.shape[:2]
        except Exception:
            return
        stride = max(1, int(self.config.get("FRONT_DEPTH_STRIDE", 8)))
        fov = float(self.config.get("FRONT_FOV_DEG", self.sim_config.get("FRONT_FOV", 90.0)))
        fx = float(w) / (2.0 * math.tan(math.radians(fov) * 0.5))
        fy = fx
        min_depth = float(self.config.get("FRONT_MIN_DEPTH_M", 0.8))
        max_depth = float(self.config.get("FRONT_MAX_DEPTH_M", 28.0))
        offset = self._point3(self.config.get("FRONT_CAMERA_OFFSET", self.sim_config.get("FRONT_CAMERA_OFFSET", [1.0, 0.0, 0.0])))
        for y in range(stride // 2, h, stride):
            for x in range(stride // 2, w, stride):
                value = float(arr[y, x])
                if not math.isfinite(value) or value < min_depth or value > max_depth:
                    continue
                ray = [1.0, (float(x) - w * 0.5) / fx, (float(y) - h * 0.5) / fy]
                norm = math.sqrt(ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2])
                ray = [v / max(norm, 1e-9) for v in ray]
                yield ([offset[i] + ray[i] * value for i in range(3)], "front")

    def _iter_down_depth_points(self, depth):
        try:
            import numpy as np

            arr = np.asarray(depth)
            h, w = arr.shape[:2]
        except Exception:
            return
        stride = max(1, int(self.config.get("DOWN_DEPTH_STRIDE", 16)))
        fov = float(self.config.get("DOWN_FOV_DEG", self.sim_config.get("DOWN_FOV", 90.0)))
        fx = float(w) / (2.0 * math.tan(math.radians(fov) * 0.5))
        fy = fx
        min_depth = float(self.config.get("DOWN_MIN_DEPTH_M", 0.8))
        max_depth = float(self.config.get("DOWN_MAX_DEPTH_M", 60.0))
        offset = self._point3(self.config.get("DOWN_CAMERA_OFFSET", self.sim_config.get("DOWN_CAMERA_OFFSET", [0.0, 0.0, 0.0])))
        for y in range(stride // 2, h, stride):
            for x in range(stride // 2, w, stride):
                value = float(arr[y, x])
                if not math.isfinite(value) or value < min_depth or value > max_depth:
                    continue
                ray = [1.0, (float(x) - w * 0.5) / fx, (float(y) - h * 0.5) / fy]
                norm = math.sqrt(ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2])
                ray = [v / max(norm, 1e-9) for v in ray]
                point_camera = [v * value for v in ray]
                # down相机向下看：相机前方对应机体系 +Z(NED向下)。
                point_body = [
                    offset[0] - point_camera[2],
                    offset[1] + point_camera[1],
                    offset[2] + point_camera[0],
                ]
                yield (point_body, "down")

    def _accept_depth_point(self, point_body: Sequence[float]) -> bool:
        x, y, z = [float(v) for v in point_body[:3]]
        if x < -1.0 or x > float(self.config.get("LOOKAHEAD_M", 18.0)):
            return False
        if abs(y) > float(self.config.get("SIDE_RANGE_M", 9.0)):
            return False
        if abs(z) > float(self.config.get("VERTICAL_RANGE_M", 4.5)):
            return False
        return True

    def _active_obstacles_body(self, current_world: Sequence[float], yaw_deg: float, *, memory_context: dict | None):
        ttl = float(self.config.get("OBSTACLE_TTL_S", 12.0))
        min_conf = float(self.config.get("MIN_CELL_CONFIDENCE", 0.12))
        lookahead = float(self.config.get("LOOKAHEAD_M", 18.0))
        side_range = float(self.config.get("SIDE_RANGE_M", 9.0))
        vertical_range = float(self.config.get("VERTICAL_RANGE_M", 4.5))
        target_body = (memory_context or {}).get("target_body") or []
        obstacles = []
        for cell in self.obstacle_cells.values():
            if cell.age_s() > ttl or float(cell.confidence) < min_conf:
                continue
            body = world_to_body(cell.center_world, current_world, yaw_deg)
            if body[0] < -1.0 or body[0] > lookahead:
                continue
            if abs(body[1]) > side_range or abs(body[2]) > vertical_range:
                continue
            if len(target_body) >= 3 and self._distance3(body, target_body) <= float(self.config.get("TARGET_DEPTH_EXCLUSION_M", 0.0)):
                continue
            obstacles.append(body)
        return obstacles

    def _filter_target_keepout(self, waypoints: List[List[float]], memory_context: dict | None) -> AvoidanceResult:
        if not bool(self.config.get("TARGET_KEEP_OUT_ENABLED", True)):
            return AvoidanceResult(waypoints=waypoints)
        if not memory_context or not bool(memory_context.get("enabled", False)):
            return AvoidanceResult(waypoints=waypoints)
        relation = str(memory_context.get("relation", "near") or "near").lower()
        if relation in {"above", "over", "on top", "on top of"}:
            return AvoidanceResult(waypoints=waypoints)
        target = memory_context.get("target_body") or []
        if len(target) < 3:
            return AvoidanceResult(waypoints=waypoints)
        footprint = max(0.5, float(memory_context.get("footprint_radius_m", 1.5) or 1.5))
        uncertainty = max(0.0, float(memory_context.get("uncertainty_m", 0.0) or 0.0))
        radius = max(
            float(self.config.get("TARGET_KEEP_OUT_RADIUS_M", 3.0)),
            footprint + float(self.config.get("TARGET_KEEP_OUT_EXTRA_M", 2.2)) + min(uncertainty, 2.0) * 0.35,
        )
        hit = self._first_collision(waypoints, [target], safety_radius=radius, vertical_clearance=float(self.config.get("TARGET_KEEP_OUT_VERTICAL_M", 2.2)), source="target_keepout")
        if hit is None:
            return AvoidanceResult(waypoints=waypoints)
        stopped = self._stop_before_hit(waypoints, hit, stop_buffer=float(self.config.get("TARGET_KEEP_OUT_STOP_BUFFER_M", 1.0)))
        return AvoidanceResult(
            waypoints=stopped,
            changed=True,
            blocked=not bool(stopped),
            reason=f"target_keepout_{radius:.1f}m",
            obstacle_body=[round(float(v), 2) for v in target[:3]],
            details={"radius_m": round(radius, 2)},
        )

    def _first_collision(
        self,
        waypoints: List[List[float]],
        obstacles: List[List[float]],
        *,
        safety_radius: float | None = None,
        vertical_clearance: float | None = None,
        source: str = "depth",
    ) -> _CollisionHit | None:
        radius = float(safety_radius if safety_radius is not None else self.config.get("SAFETY_RADIUS_M", 1.35))
        z_clear = float(vertical_clearance if vertical_clearance is not None else self.config.get("VERTICAL_CLEARANCE_M", 1.15))
        max_check = float(self.config.get("PATH_CHECK_DISTANCE_M", self.config.get("LOOKAHEAD_M", 18.0)))
        prev = [0.0, 0.0, 0.0]
        traveled = 0.0
        best: _CollisionHit | None = None
        for index, waypoint in enumerate(waypoints):
            cur = [float(v) for v in waypoint[:3]]
            seg = [cur[i] - prev[i] for i in range(3)]
            seg_len = self._norm3(seg)
            if seg_len <= 1e-6:
                prev = cur
                continue
            for obstacle in obstacles:
                t, point = self._closest_point_on_segment(prev, cur, obstacle)
                along = traveled + t * seg_len
                if along > max_check:
                    continue
                horizontal = math.sqrt((point[0] - obstacle[0]) ** 2 + (point[1] - obstacle[1]) ** 2)
                vertical = abs(point[2] - obstacle[2])
                if horizontal <= radius and vertical <= z_clear:
                    hit = _CollisionHit(
                        segment_index=index,
                        total_distance_m=along,
                        segment_t=t,
                        path_point=point,
                        obstacle_body=[float(v) for v in obstacle[:3]],
                        source=source,
                    )
                    if best is None or hit.total_distance_m < best.total_distance_m:
                        best = hit
            traveled += seg_len
            if traveled > max_check and best is not None:
                break
            prev = cur
        return best

    def _stop_before_hit(self, waypoints: List[List[float]], hit: _CollisionHit, *, stop_buffer: float | None = None) -> List[List[float]]:
        buffer_m = float(stop_buffer if stop_buffer is not None else self.config.get("STOP_BUFFER_M", 2.2))
        min_stop = float(self.config.get("MIN_STOP_WAYPOINT_M", 0.8))
        stop_distance = max(0.0, float(hit.total_distance_m) - buffer_m)
        if stop_distance < min_stop:
            return []
        point = self._point_at_path_distance(waypoints, stop_distance)
        return [[round(float(point[0]), 3), round(float(point[1]), 3), round(float(point[2]), 3)]]

    def _try_bypass(
        self,
        waypoints: List[List[float]],
        hit: _CollisionHit,
        obstacles: List[List[float]],
        memory_context: dict | None,
    ) -> List[List[float]] | None:
        if float(hit.total_distance_m) > float(self.config.get("BYPASS_MAX_HIT_DISTANCE_M", 12.0)):
            return None
        base = self._point_at_path_distance(
            waypoints,
            max(float(self.config.get("MIN_STOP_WAYPOINT_M", 0.8)), hit.total_distance_m - float(self.config.get("STOP_BUFFER_M", 2.2))),
        )
        lateral = max(float(self.config.get("BYPASS_LATERAL_M", 3.0)), float(self.config.get("SAFETY_RADIUS_M", 1.35)) + 1.2)
        z_lift = float(self.config.get("BYPASS_UP_M", 0.0))
        preferred = self._preferred_bypass_sides(memory_context, hit.obstacle_body)
        suffix = [list(wp) for wp in waypoints[hit.segment_index:]]
        for side in preferred:
            detour_y = float(hit.obstacle_body[1]) + side * lateral
            detour = [
                max(float(base[0]), min(float(hit.obstacle_body[0]) - 0.7, float(base[0]) + float(self.config.get("BYPASS_FORWARD_M", 2.5)))),
                detour_y,
                float(base[2]) - z_lift,
            ]
            candidate = [[round(detour[0], 3), round(detour[1], 3), round(detour[2], 3)]]
            candidate.extend(suffix)
            if self._first_collision(candidate, obstacles) is None:
                return candidate
        return None

    def _preferred_bypass_sides(self, memory_context: dict | None, obstacle_body: Sequence[float]) -> List[int]:
        target = (memory_context or {}).get("target_body") or []
        if len(target) >= 3:
            # 优先从远离目标中心的一侧绕开，避免“飞到车旁”时绕进车身。
            return [-1, 1] if float(target[1]) >= float(obstacle_body[1]) else [1, -1]
        return [-1, 1]

    def _obstacle_matches_locked_target(self, obstacle_body: Sequence[float], memory_context: dict | None) -> bool:
        target = (memory_context or {}).get("target_body") or []
        if len(target) < 3:
            return False
        radius = max(
            2.0,
            float(memory_context.get("footprint_radius_m", 1.5) or 1.5)
            + float(self.config.get("TARGET_MATCH_MARGIN_M", 2.0)),
        )
        return self._distance3(obstacle_body, target) <= radius

    def _prune(self) -> None:
        ttl = float(self.config.get("OBSTACLE_TTL_S", 12.0))
        for key in [key for key, cell in self.obstacle_cells.items() if cell.age_s() > ttl]:
            self.obstacle_cells.pop(key, None)
        max_cells = max(1, int(self.config.get("MAX_CELLS", 256)))
        if len(self.obstacle_cells) <= max_cells:
            return
        cells = sorted(
            self.obstacle_cells.values(),
            key=lambda cell: (cell.confidence, -cell.age_s(), cell.observation_count),
        )
        for cell in cells[: len(self.obstacle_cells) - max_cells]:
            self.obstacle_cells.pop(cell.key, None)

    def _cell_key(self, world: Sequence[float]) -> str:
        res = max(0.2, float(self.config.get("GRID_RESOLUTION_M", 1.0)))
        return ":".join(str(int(round(float(v) / res))) for v in world[:3])

    def _body_to_world(self, body: Sequence[float], observer_world: Sequence[float], yaw_deg: float) -> List[float]:
        yaw = math.radians(float(yaw_deg))
        c = math.cos(yaw)
        s = math.sin(yaw)
        x, y, z = [float(v) for v in body[:3]]
        return [
            float(observer_world[0]) + c * x - s * y,
            float(observer_world[1]) + s * x + c * y,
            float(observer_world[2]) + z,
        ]

    @staticmethod
    def _normalize_waypoints(waypoints: Iterable[Sequence[float]] | None) -> List[List[float]]:
        out: List[List[float]] = []
        for waypoint in list(waypoints or []):
            if not isinstance(waypoint, (list, tuple)) or len(waypoint) < 3:
                continue
            values = [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])]
            if all(math.isfinite(v) for v in values):
                out.append(values)
        return out

    @staticmethod
    def _closest_point_on_segment(a: Sequence[float], b: Sequence[float], p: Sequence[float]):
        ab = [float(b[i]) - float(a[i]) for i in range(3)]
        ap = [float(p[i]) - float(a[i]) for i in range(3)]
        denom = sum(v * v for v in ab)
        if denom <= 1e-9:
            return 0.0, [float(v) for v in a[:3]]
        t = max(0.0, min(1.0, sum(ap[i] * ab[i] for i in range(3)) / denom))
        return t, [float(a[i]) + t * ab[i] for i in range(3)]

    def _point_at_path_distance(self, waypoints: List[List[float]], distance_m: float) -> List[float]:
        prev = [0.0, 0.0, 0.0]
        remaining = max(0.0, float(distance_m))
        for waypoint in waypoints:
            cur = [float(v) for v in waypoint[:3]]
            seg = [cur[i] - prev[i] for i in range(3)]
            seg_len = self._norm3(seg)
            if seg_len <= 1e-9:
                prev = cur
                continue
            if remaining <= seg_len:
                t = remaining / seg_len
                return [prev[i] + t * seg[i] for i in range(3)]
            remaining -= seg_len
            prev = cur
        return list(waypoints[-1])

    @staticmethod
    def _point3(value: Sequence[float]) -> List[float]:
        return [float(value[0]), float(value[1]), float(value[2])]

    @staticmethod
    def _norm3(point: Sequence[float]) -> float:
        return math.sqrt(sum(float(v) * float(v) for v in point[:3]))

    @staticmethod
    def _distance3(a: Sequence[float], b: Sequence[float]) -> float:
        return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))
