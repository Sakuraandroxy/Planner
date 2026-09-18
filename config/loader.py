from __future__ import annotations

import os
from pathlib import Path

import yaml

from planner.errors import ConfigurationError
from .schema import AirSimConfig, ApiConfig, DepthConfig, PlannerConfig, TrajectoryConfig


def load_config(path: str | Path) -> PlannerConfig:
    config_path = Path(path).expanduser().resolve()
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        parser = raw["task_parser"]
        planning = raw["trajectory_planner"]
        sim = raw["airsim"]
        depth = raw["depth"]
        trajectory = raw["trajectory"]
        result = PlannerConfig(
            task_parser=_api_config(parser),
            trajectory_planner=_api_config(planning),
            airsim=AirSimConfig(
                host=str(sim.get("host", "")), port=int(sim["port"]), camera_id=str(sim["camera_id"]),
                connect_timeout_s=float(sim["connect_timeout_s"]),
                move_timeout_s=float(sim["move_timeout_s"]), speed_mps=float(sim["speed_mps"]),
            ),
            depth=DepthConfig(float(depth["min_m"]), float(depth["max_m"])),
            trajectory=TrajectoryConfig(
                int(trajectory["point_count"]), float(trajectory["max_step_m"]),
                float(trajectory["max_yaw_step_deg"]),
            ),
        )
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ConfigurationError(f"invalid config {config_path}: {exc}") from exc
    if result.trajectory.point_count <= 0 or result.airsim.speed_mps <= 0:
        raise ConfigurationError("point_count and speed_mps must be positive")
    if not 0 < result.depth.min_m < result.depth.max_m:
        raise ConfigurationError("depth range must satisfy 0 < min_m < max_m")
    return result


def _api_config(raw: dict) -> ApiConfig:
    env_name = str(raw.get("api_key_env", "")).strip()
    api_key = os.environ.get(env_name, "") if env_name else str(raw.get("api_key", "no-key"))
    return ApiConfig(str(raw["url"]), str(raw["model"]), api_key or "no-key", float(raw["timeout_s"]))

