from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ApiConfig:
    url: str
    model: str
    api_key: str
    timeout_s: float


@dataclass(frozen=True)
class AirSimConfig:
    host: str
    port: int
    camera_id: str
    connect_timeout_s: float
    move_timeout_s: float
    speed_mps: float


@dataclass(frozen=True)
class DepthConfig:
    min_m: float
    max_m: float


@dataclass(frozen=True)
class TrajectoryConfig:
    point_count: int
    max_step_m: float
    max_yaw_step_deg: float


@dataclass(frozen=True)
class PlannerConfig:
    task_parser: ApiConfig
    trajectory_planner: ApiConfig
    airsim: AirSimConfig
    depth: DepthConfig
    trajectory: TrajectoryConfig

