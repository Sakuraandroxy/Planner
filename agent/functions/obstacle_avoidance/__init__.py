"""Obstacle avoidance package."""

from __future__ import annotations

from config import cfg

from agent.functions.common.config_access import function_section
from agent.functions.obstacle_avoidance.local_depth_avoider import DepthObstacleAvoider
from agent.functions.obstacle_avoidance.schemas import AvoidanceResult, ObstacleCell


def build_depth_obstacle_avoider() -> DepthObstacleAvoider:
    """Build the lightweight local depth avoider from FUNCTIONS.OBSTACLE_AVOIDANCE."""
    return DepthObstacleAvoider(
        config=function_section(cfg, "OBSTACLE_AVOIDANCE"),
        sim_config=cfg.get("SIM", {}) or {},
    )


__all__ = [
    "AvoidanceResult",
    "DepthObstacleAvoider",
    "ObstacleCell",
    "build_depth_obstacle_avoider",
]
