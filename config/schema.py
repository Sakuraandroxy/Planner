from __future__ import annotations

from dataclasses import dataclass, field
from planner.domain.motion import MotionLimits
from .visual_goal import VisualGoalConfig
from .basic_motion import BasicMotionConfig
from planner.domain.sliding_window import SlidingWindowSettings
from .task_completion import TaskCompletionConfig
from .target_detection import TargetDetectionConfig


@dataclass(frozen=True)
class MotionConfig:
    limits: MotionLimits = MotionLimits()


@dataclass(frozen=True)
class ApiConfig:
    url: str
    model: str
    api_key: str
    timeout_s: float
    thinking: str | None = None
    require_api_key: bool = field(default=False, kw_only=True)


@dataclass(frozen=True)
class TaskParserConfig(ApiConfig):
    max_completion_tokens: int | None = None
    reasoning_effort: str | None = None
    request_format: str = "openai"


@dataclass(frozen=True)
class AirSimConfig:
    host: str
    port: int
    camera_id: str
    connect_timeout_s: float
    move_timeout_s: float
    speed_mps: float
    depth_horizontal_fov_deg: float | None = None
    maximum_pair_time_gap_s: float = 0.05


@dataclass(frozen=True)
class DepthConfig:
    min_m: float
    max_m: float
    mapping: str = "log_inverse_gray_v1"


@dataclass(frozen=True)
class TrajectoryConfig:
    point_count: int
    max_step_m: float
    max_yaw_step_deg: float


@dataclass(frozen=True)
class RecordingConfig:
    output_root: str
    fps: float


@dataclass(frozen=True)
class ProgressReviewConfig:
    enabled: bool = False


@dataclass(frozen=True)
class PlannerConfig:
    task_parser: TaskParserConfig
    trajectory_planner: ApiConfig
    airsim: AirSimConfig
    depth: DepthConfig
    trajectory: TrajectoryConfig
    recording: RecordingConfig
    basic_motion: BasicMotionConfig = BasicMotionConfig()
    motion: MotionConfig = MotionConfig()
    visual_goal: VisualGoalConfig = VisualGoalConfig()
    task_completion: TaskCompletionConfig = TaskCompletionConfig()
    sliding_window: SlidingWindowSettings = SlidingWindowSettings()
    target_detection: TargetDetectionConfig | None = None
    progress_review: ProgressReviewConfig = ProgressReviewConfig()

