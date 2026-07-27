"""Configurable target distance estimation function."""

from config import cfg

from agent.functions.common.config_access import function_section
from agent.functions.distance_estimation.estimator import (
    DistanceEstimate,
    GeometricTargetDistanceEstimator,
    TargetPoseEstimate,
)


def build_distance_estimator() -> GeometricTargetDistanceEstimator:
    config = function_section(cfg, "DISTANCE_ESTIMATION")
    name = str(config.get("NAME", "geometric_target_pose")).strip().lower()
    if name not in {"geometric_target_pose", "geometric", "target_pose"}:
        raise ValueError(f"Unknown DISTANCE_ESTIMATION.NAME: {name}")
    return GeometricTargetDistanceEstimator(config=config, sim_config=cfg.get("SIM", {}) or {})


__all__ = [
    "DistanceEstimate",
    "GeometricTargetDistanceEstimator",
    "TargetPoseEstimate",
    "build_distance_estimator",
]

