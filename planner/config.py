"""Configuration loading for the ProjectAirSim VLN skeleton."""

from __future__ import annotations

from dataclasses import dataclass, field, fields, replace
import math
from pathlib import Path
from typing import Any

import yaml

from .domain.depth import validate_depth_protocol
from .domain.depth_foreground import DepthForegroundSettings
from .domain.motion import ContinuousTrackingSettings
from .domain.sliding_window import SlidingWindowSettings
from .domain.visual_goal_completion import VisualGoalTriggerRules
from .domain.visual_goal_limits import VisualGoalLimits
from .domain.view import VIEW_MODES
from .services.segment_motion import MotionLimits


@dataclass(frozen=True)
class ProjectAirSimSettings:
    """Connection and scene identity for the ProjectAirSim server."""

    address: str = "127.0.0.1"
    port_topics: int = 8991
    port_services: int = 8992
    timeout_s: float = 10.0
    # Scene config loaded by the client; robot-config files sit beside it.
    scene_file: str = "scene_city_dual.jsonc"
    sim_config_path: str = "config/scenes/city"
    # The dual scene starts with the clock paused; this delay only waits for
    # the renderer so the first calibration screenshot has content.
    delay_after_load_s: float = 2.0
    # Robot names must match the actor names in the scene config.
    rover_name: str = "Rover1"
    drone_name: str = "Drone1"
    # Sensor ids must match the robot config.
    front_camera: str = "FrontCamera"
    chase_camera: str = "Chase"
    down_camera: str = "DownCamera"
    # Streaming views whose camera sets capture-enabled: false. Calibration
    # locates them by elimination instead of waiting for a frame timeout.
    uncapturable_views: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not str(self.address).strip():
            raise ValueError("projectairsim.address must be non-empty")
        for name in ("port_topics", "port_services"):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= 65535:
                raise ValueError(f"projectairsim.{name} must be in [1, 65535]")
        if self.timeout_s <= 0:
            raise ValueError("projectairsim.timeout_s must be positive")
        if not str(self.scene_file).strip():
            raise ValueError("projectairsim.scene_file must be non-empty")
        if not str(self.sim_config_path).strip():
            raise ValueError("projectairsim.sim_config_path must be non-empty")
        if self.delay_after_load_s < 0:
            raise ValueError("projectairsim.delay_after_load_s must be >= 0")
        for name in (
            "rover_name",
            "drone_name",
            "front_camera",
            "chase_camera",
            "down_camera",
        ):
            if not str(getattr(self, name)).strip():
                raise ValueError(f"projectairsim.{name} must be non-empty")
        views = tuple(self.uncapturable_views)
        if len(set(views)) != len(views):
            raise ValueError(
                "projectairsim.uncapturable_views must not repeat a view"
            )
        unknown = [view for view in views if view not in VIEW_MODES]
        if unknown:
            raise ValueError(
                "projectairsim.uncapturable_views has unknown views: "
                + ", ".join(unknown)
            )


@dataclass(frozen=True)
class DroneSettings:
    """``drone`` section: UAV flight envelope, timeouts and camera mount."""

    speed_mps: float = 2.0
    move_timeout_s: float = 60.0
    takeoff_timeout_s: float = 30.0
    land_timeout_s: float = 30.0
    readiness_stable_s: float = 2.0
    readiness_poll_interval_s: float = 0.25
    # FrontCamera mount in the body frame (forward, right, down; metres) and
    # its NED roll/pitch/yaw orientation. The reference UAV mount is level and
    # one metre ahead of the body origin.
    camera_offset_m: tuple[float, float, float] = (1.0, 0.0, 0.0)
    camera_mount_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        for name in (
            "speed_mps",
            "move_timeout_s",
            "takeoff_timeout_s",
            "land_timeout_s",
            "readiness_poll_interval_s",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"drone.{name} must be finite and positive")
        if (
            isinstance(self.readiness_stable_s, bool)
            or not isinstance(self.readiness_stable_s, (int, float))
            or not math.isfinite(self.readiness_stable_s)
            or self.readiness_stable_s < 0
        ):
            raise ValueError(
                "drone.readiness_stable_s must be finite and non-negative"
            )
        for name in ("camera_offset_m", "camera_mount_rpy_deg"):
            values = getattr(self, name)
            if len(values) != 3 or not all(
                not isinstance(value, bool)
                and isinstance(value, (int, float))
                and math.isfinite(float(value))
                for value in values
            ):
                raise ValueError(
                    f"drone.{name} must contain three finite numbers"
                )

    def camera(self, base: CameraSettings) -> CameraSettings:
        """Use the drone's level, forward camera mount instead of the rover's."""
        return replace(
            base,
            offset_m=self.camera_offset_m,
            mount_rpy_deg=self.camera_mount_rpy_deg,
        )


@dataclass(frozen=True)
class CameraSettings:
    width: int = 1280
    height: int = 720
    hfov_deg: float = 90.0
    # ProjectAirSim depth (image-type 2) is metric ray range in metres,
    # saturated to 65504 m on the sky; far_m is kept for config validation
    # only and does not participate in decoding.
    far_m: float = 1000.0
    depth_is_range: bool = True
    # Mount offset in the body frame: forward, right, down (metres), matching
    # the planner's NED body convention and the robot jsonc sensor origin.
    offset_m: tuple[float, float, float] = (1.8, 0.0, -0.8)
    # Camera mount orientation relative to the body (degrees, NED roll/pitch/yaw),
    # used to recover the body pose from the camera pose published on topics.
    mount_rpy_deg: tuple[float, float, float] = (0.0, -3.0, 0.0)


@dataclass(frozen=True)
class TargetDetectionSettings:
    url: str = ""
    model: str = ""
    # Inline key (takes precedence); empty falls back to api_key_env.
    api_key: str = ""
    api_key_env: str = "VLN_API_KEY"
    timeout_s: float = 30.0
    reasoning_effort: str = "none"
    request_format: str = "openai"
    max_completion_tokens: int = 2048
    max_image_side: int = 1024
    minimum_depth_m: float = 0.5

    def __post_init__(self) -> None:
        if not self.timeout_s > 0:
            raise ValueError("target_detection.timeout_s must be positive")
        if self.reasoning_effort not in {"none", "low", "medium", "high"}:
            raise ValueError("invalid target_detection.reasoning_effort")
        if self.request_format not in ("openai", "deepseek", "anthropic"):
            raise ValueError(
                "target_detection.request_format must be openai, deepseek or anthropic"
            )
        if self.request_format == "anthropic" and self.reasoning_effort != "none":
            raise ValueError(
                "target_detection.anthropic requires reasoning_effort 'none'"
            )
        if (
            type(self.max_completion_tokens) is not int
            or not 1 <= self.max_completion_tokens <= 32768
        ):
            raise ValueError(
                "target_detection.max_completion_tokens must be in [1, 32768]"
            )
        if (
            type(self.max_image_side) is not int
            or not 32 <= self.max_image_side <= 8192
        ):
            raise ValueError(
                "target_detection.max_image_side must be in [32, 8192]"
            )
        if not self.minimum_depth_m > 0:
            raise ValueError("target_detection.minimum_depth_m must be positive")


@dataclass(frozen=True)
class TrajectoryPlannerSettings:
    url: str = ""
    model: str = ""
    # Inline key (takes precedence); empty falls back to api_key_env.
    api_key: str = ""
    api_key_env: str = "VLN_API_KEY"
    timeout_s: float = 30.0
    max_tokens: int = 256
    reasoning_effort: str = "none"
    request_format: str = "openai"
    point_count: int = 5
    max_step_m: float = 2.0
    max_yaw_step_deg: float = 45.0

    def __post_init__(self) -> None:
        if self.point_count != 5:
            raise ValueError("the trajectory protocol requires exactly 5 points")
        for name in ("max_step_m", "max_yaw_step_deg"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not value > 0:
                raise ValueError(f"trajectory_planner.{name} must be positive")
        if self.max_yaw_step_deg > 180:
            raise ValueError(
                "trajectory_planner.max_yaw_step_deg must not exceed 180"
            )
        if self.reasoning_effort not in {"none", "low", "medium", "high"}:
            raise ValueError("invalid trajectory_planner.reasoning_effort")
        if self.request_format not in ("openai", "deepseek", "anthropic"):
            raise ValueError(
                "trajectory_planner.request_format must be openai, deepseek or anthropic"
            )
        if self.request_format == "anthropic" and self.reasoning_effort != "none":
            raise ValueError(
                "trajectory_planner.anthropic requires reasoning_effort 'none'"
            )


@dataclass(frozen=True)
class DepthSettings:
    mapping: str = "log_inverse_gray_v1"
    min_m: float = 0.5
    max_m: float = 200.0

    def __post_init__(self) -> None:
        validate_depth_protocol(self.mapping, self.min_m, self.max_m)


@dataclass(frozen=True)
class TaskCompletionSettings:
    url: str = ""
    model: str = ""
    # Inline key (takes precedence); empty falls back to api_key_env.
    api_key: str = ""
    api_key_env: str = "VLN_API_KEY"
    timeout_s: float = 20.0
    max_completion_tokens: int = 2048
    reasoning_effort: str = "none"
    request_format: str = "openai"

    def __post_init__(self) -> None:
        if self.timeout_s <= 0:
            raise ValueError("task_completion.timeout_s must be positive")
        if self.max_completion_tokens < 1:
            raise ValueError(
                "task_completion.max_completion_tokens must be a positive integer"
            )
        if self.reasoning_effort not in {"none", "low", "medium", "high"}:
            raise ValueError("invalid task_completion.reasoning_effort")
        if self.request_format not in ("openai", "deepseek", "anthropic"):
            raise ValueError(
                "task_completion.request_format must be openai, deepseek or anthropic"
            )
        if self.request_format == "anthropic" and self.reasoning_effort != "none":
            raise ValueError(
                "task_completion.anthropic requires reasoning_effort 'none'"
            )


@dataclass(frozen=True)
class TaskParserSettings:
    """Natural-language mission parser (``task_parser`` section)."""

    url: str = ""
    model: str = ""
    # Inline key (takes precedence); empty falls back to api_key_env.
    api_key: str = ""
    api_key_env: str = "VLN_API_KEY"
    timeout_s: float = 60.0
    max_completion_tokens: int = 2048
    reasoning_effort: str = "none"
    request_format: str = "openai"

    def __post_init__(self) -> None:
        if not self.timeout_s > 0:
            raise ValueError("task_parser.timeout_s must be positive")
        if (
            type(self.max_completion_tokens) is not int
            or not 1 <= self.max_completion_tokens <= 32768
        ):
            raise ValueError(
                "task_parser.max_completion_tokens must be in [1, 32768]"
            )
        if self.reasoning_effort not in {"none", "low", "medium", "high"}:
            raise ValueError("invalid task_parser.reasoning_effort")
        if self.request_format not in ("openai", "deepseek", "anthropic"):
            raise ValueError(
                "task_parser.request_format must be openai, deepseek or anthropic"
            )
        if self.request_format == "anthropic" and self.reasoning_effort != "none":
            raise ValueError(
                "task_parser.anthropic requires reasoning_effort 'none'"
            )


@dataclass(frozen=True)
class VisualGoalNavigationSettings:
    enabled: bool = True
    geometry_verified: bool = False
    collision_check_enabled: bool = False
    output_root: str = "output/visual_goal"
    limits: VisualGoalLimits = field(default_factory=VisualGoalLimits)
    completion_rules_file: str | None = None
    completion_rules: VisualGoalTriggerRules = field(
        default_factory=VisualGoalTriggerRules
    )
    depth_foreground_filter: DepthForegroundSettings = field(
        default_factory=DepthForegroundSettings
    )

    def __post_init__(self) -> None:
        for name in ("enabled", "geometry_verified", "collision_check_enabled"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} must be boolean")
        if not isinstance(self.output_root, str) or not self.output_root.strip():
            raise ValueError("visual goal output_root must be non-empty text")
        if self.completion_rules_file is not None and (
            not isinstance(self.completion_rules_file, str)
            or not self.completion_rules_file.strip()
        ):
            raise ValueError("completion_rules_file must be non-empty text")


@dataclass(frozen=True)
class FreeNavigationSettings:
    """``free_navigation`` section: native-contact recovery knobs."""

    collision_backoff_m: float = 3.0
    maximum_collision_recoveries: int = 3
    # Sustained forward progress that clears the consecutive recovery count.
    collision_recovery_reset_m: float = 20.0

    def __post_init__(self) -> None:
        for name in ("collision_backoff_m", "collision_recovery_reset_m"):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if (
            type(self.maximum_collision_recoveries) is not int
            or self.maximum_collision_recoveries < 1
        ):
            raise ValueError(
                "maximum_collision_recoveries must be a positive integer"
            )


@dataclass(frozen=True)
class ServerSettings:
    """HTTP server and in-process worker knobs (``server`` section)."""

    host: str = "127.0.0.1"
    port: int = 8000
    detector: str = "manual"
    planner: str = "geometric"
    manual_bbox: tuple[float, float, float, float] = (0.45, 0.82, 0.55, 0.92)
    manual_surface_points: tuple[tuple[float, float], ...] = ()
    allowed_origins: tuple[str, ...] = ("*",)
    log_buffer_size: int = 500
    reconnect_interval_s: float = 2.0
    startup_timeout_s: float = 60.0
    video_fps: float = 10.0
    # Vehicle ids the goal router accepts; every task names exactly one.
    vehicles: tuple[str, ...] = ("rover", "drone")
    default_vehicle: str = "rover"

    def __post_init__(self) -> None:
        if not 1 <= self.port <= 65535:
            raise ValueError("server.port must be in [1, 65535]")
        if self.detector not in ("manual", "vlm"):
            raise ValueError("server.detector must be 'manual' or 'vlm'")
        if self.planner not in ("geometric", "vlm"):
            raise ValueError("server.planner must be 'geometric' or 'vlm'")
        box = self.manual_bbox
        if len(box) != 4 or not (
            0.0 <= box[0] < box[2] <= 1.0 and 0.0 <= box[1] < box[3] <= 1.0
        ):
            raise ValueError(
                "server.manual_bbox must be ordered normalized "
                "xmin,ymin,xmax,ymax"
            )
        if len(self.manual_surface_points) > 5:
            raise ValueError("server.manual_surface_points allows at most 5 points")
        if not self.allowed_origins:
            raise ValueError("server.allowed_origins must not be empty")
        if self.log_buffer_size <= 0:
            raise ValueError("server.log_buffer_size must be positive")
        if self.reconnect_interval_s <= 0:
            raise ValueError("server.reconnect_interval_s must be positive")
        if self.startup_timeout_s <= 0:
            raise ValueError("server.startup_timeout_s must be positive")
        if not 0.0 < self.video_fps <= 60.0:
            raise ValueError("server.video_fps must be in (0, 60]")
        if not self.vehicles or any(
            not str(vehicle).strip() for vehicle in self.vehicles
        ):
            raise ValueError("server.vehicles must contain non-empty ids")
        if len(set(self.vehicles)) != len(self.vehicles):
            raise ValueError("server.vehicles must be unique")
        if self.default_vehicle not in self.vehicles:
            raise ValueError(
                "server.default_vehicle must appear in server.vehicles"
            )


@dataclass(frozen=True)
class MapEntry:
    """One switchable simulator map (``config/maps.yaml``)."""

    id: str
    label: str
    # First argument of launch_projectairsim_xpra.sh.
    launch_map: str
    # Dual-vehicle scene JSONC inside sim_config_path.
    scene_file: str
    # World scene-config directory, relative to the repository root.
    sim_config_path: str

    def __post_init__(self) -> None:
        for name in ("id", "label", "launch_map", "scene_file", "sim_config_path"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"map {name} must be non-empty text")
        for name in ("id", "launch_map"):
            value = getattr(self, name)
            if any(ch.isspace() for ch in value):
                raise ValueError(f"map {name} must not contain whitespace")


@dataclass(frozen=True)
class VehicleModelOption:
    """One appearance model the console may select (``vehicle_models``)."""

    id: str
    label: str
    # Scene variant per map id; maps not listed use the map entry's scene.
    scene_files: tuple[tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        for name in ("id", "label"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"vehicle model {name} must be non-empty text")
            if name == "id" and any(ch.isspace() for ch in value):
                raise ValueError("vehicle model id must not contain whitespace")
        for map_id, scene_file in self.scene_files:
            if not isinstance(map_id, str) or not map_id.strip():
                raise ValueError("vehicle model scene map ids must be non-empty")
            if not isinstance(scene_file, str) or not scene_file.strip():
                raise ValueError("vehicle model scene files must be non-empty")

    def scene_for(self, map_id: str, default: str) -> str:
        for key, value in self.scene_files:
            if key == map_id:
                return value
        return default


@dataclass(frozen=True)
class VehicleModelGroup:
    """Selectable appearance models for one vehicle id."""

    vehicle: str
    default: str
    options: tuple[VehicleModelOption, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.vehicle, str) or not self.vehicle.strip():
            raise ValueError("vehicle model group must name a vehicle")
        if not self.options:
            raise ValueError(
                f"vehicle model group '{self.vehicle}' needs at least one option"
            )
        ids = [option.id for option in self.options]
        if len(set(ids)) != len(ids):
            raise ValueError(
                f"vehicle model ids for '{self.vehicle}' must be unique"
            )
        if self.default not in ids:
            raise ValueError(
                f"default model '{self.default}' is not an option for "
                f"'{self.vehicle}'"
            )

    def option(self, model_id: str) -> VehicleModelOption | None:
        for option in self.options:
            if option.id == model_id:
                return option
        return None

    @property
    def ids(self) -> tuple[str, ...]:
        return tuple(option.id for option in self.options)

    @property
    def labels(self) -> dict[str, str]:
        return {option.id: option.label for option in self.options}


@dataclass(frozen=True)
class VehicleModelCatalog:
    """Appearance models per vehicle; empty ``groups`` disables switching."""

    groups: tuple[VehicleModelGroup, ...] = ()

    def __post_init__(self) -> None:
        vehicles = [group.vehicle for group in self.groups]
        if len(set(vehicles)) != len(vehicles):
            raise ValueError("vehicle model groups must be unique per vehicle")
        # Two vehicles overriding the same map with different scenes would make
        # the effective scene ambiguous; reject that at load time.
        overrides: dict[str, str] = {}
        for group in self.groups:
            for option in group.options:
                for map_id, scene_file in option.scene_files:
                    previous = overrides.setdefault(map_id, scene_file)
                    if previous != scene_file:
                        raise ValueError(
                            f"map '{map_id}' has conflicting vehicle model "
                            f"scene variants: '{previous}' and '{scene_file}'"
                        )

    def group(self, vehicle: str) -> VehicleModelGroup | None:
        for group in self.groups:
            if group.vehicle == vehicle:
                return group
        return None

    @property
    def vehicles(self) -> tuple[str, ...]:
        return tuple(group.vehicle for group in self.groups)

    @property
    def defaults(self) -> dict[str, str]:
        return {group.vehicle: group.default for group in self.groups}


def default_vehicle_models() -> VehicleModelCatalog:
    """Deployments without a ``vehicle_models`` section keep the default look."""
    return VehicleModelCatalog(
        groups=(
            VehicleModelGroup(
                "rover",
                "offroad",
                (VehicleModelOption("offroad", "越野车"),),
            ),
            VehicleModelGroup(
                "drone",
                "quadrotor",
                (VehicleModelOption("quadrotor", "四旋翼"),),
            ),
        )
    )


@dataclass(frozen=True)
class MapCatalog:
    """Map ids the console may switch to; ``default_map`` must be listed."""

    default_map: str
    maps: tuple[MapEntry, ...]

    def __post_init__(self) -> None:
        if not self.maps:
            raise ValueError("maps catalog must contain at least one map")
        ids = [entry.id for entry in self.maps]
        if len(set(ids)) != len(ids):
            raise ValueError("map ids must be unique")
        if self.default_map not in ids:
            raise ValueError("default_map must appear in maps")

    def entry(self, map_id: str) -> MapEntry | None:
        for entry in self.maps:
            if entry.id == map_id:
                return entry
        return None

    @property
    def ids(self) -> tuple[str, ...]:
        return tuple(entry.id for entry in self.maps)

    @property
    def labels(self) -> dict[str, str]:
        return {entry.id: entry.label for entry in self.maps}


@dataclass(frozen=True)
class SimulatorSettings:
    """``simulator`` section: process ownership for map switching."""

    launch_script: str = "scripts/launch_projectairsim_xpra.sh"
    stop_script: str = "scripts/stop_projectairsim_xpra.sh"
    display: str = ":104"
    xpra_port: int = 10010
    # Must match projectairsim.port_topics/port_services.
    topics_port: int = 8991
    services_port: int = 8992
    runtime_dir: str = "output/xpra"
    map_catalog_file: str = "maps.yaml"
    map_marker_file: str = "output/xpra/current_map"
    launch_timeout_s: float = 180.0
    stop_timeout_s: float = 40.0
    port_release_timeout_s: float = 30.0
    restart_attempts: int = 2

    def __post_init__(self) -> None:
        for name in ("launch_script", "stop_script", "runtime_dir",
                     "map_catalog_file", "map_marker_file"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"simulator.{name} must be non-empty text")
        if not isinstance(self.display, str) or not (
            self.display.startswith(":") and self.display[1:].isdigit()
        ):
            raise ValueError("simulator.display must look like ':104'")
        for name in ("xpra_port", "topics_port", "services_port"):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= 65535:
                raise ValueError(f"simulator.{name} must be in [1, 65535]")
        for name in ("launch_timeout_s", "stop_timeout_s",
                     "port_release_timeout_s"):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"simulator.{name} must be finite and positive")
        if type(self.restart_attempts) is not int or self.restart_attempts < 1:
            raise ValueError("simulator.restart_attempts must be a positive integer")


@dataclass(frozen=True)
class AppConfig:
    projectairsim: ProjectAirSimSettings
    camera: CameraSettings
    motion: MotionLimits
    maps: MapCatalog
    vehicle_models: VehicleModelCatalog = field(
        default_factory=default_vehicle_models
    )
    simulator: SimulatorSettings = field(default_factory=SimulatorSettings)
    drone: DroneSettings = field(default_factory=DroneSettings)
    target_detection: TargetDetectionSettings = field(
        default_factory=TargetDetectionSettings
    )
    trajectory_planner: TrajectoryPlannerSettings = field(
        default_factory=TrajectoryPlannerSettings
    )
    server: ServerSettings = field(default_factory=ServerSettings)
    depth: DepthSettings = field(default_factory=DepthSettings)
    continuous: ContinuousTrackingSettings = field(
        default_factory=ContinuousTrackingSettings
    )
    sliding_window: SlidingWindowSettings = field(
        default_factory=SlidingWindowSettings
    )
    task_completion: TaskCompletionSettings = field(
        default_factory=TaskCompletionSettings
    )
    task_parser: TaskParserSettings = field(
        default_factory=TaskParserSettings
    )
    visual_goal_navigation: VisualGoalNavigationSettings = field(
        default_factory=VisualGoalNavigationSettings
    )
    free_navigation: FreeNavigationSettings = field(
        default_factory=FreeNavigationSettings
    )


def _section(raw: dict[str, Any], key: str) -> dict[str, Any]:
    value = raw.get(key, {})
    if not isinstance(value, dict):
        raise ValueError(f"config section '{key}' must be a mapping")
    return value


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def load_map_catalog(
    path: str | Path, *, fallback: MapEntry | None = None
) -> MapCatalog:
    """Load ``maps.yaml``; without the file fall back to a single map entry.

    Tests and small deployments use a config file with no catalog next to it;
    they reconnect to whatever scene ``projectairsim`` names instead of
    switching maps. Explicit catalog entries are checked against the
    repository tree so a typo cannot silently select a missing scene.
    """
    catalog_path = Path(path)
    if not catalog_path.is_file():
        if fallback is None:
            raise FileNotFoundError(f"map catalog not found: {catalog_path}")
        return MapCatalog(default_map=fallback.id, maps=(fallback,))
    raw = yaml.safe_load(catalog_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError("map catalog root must be a mapping")
    entries_raw = raw.get("maps")
    if not isinstance(entries_raw, list) or not entries_raw:
        raise ValueError("map catalog must list at least one map")
    entries = []
    for item in entries_raw:
        if not isinstance(item, dict):
            raise ValueError("map catalog entries must be mappings")
        entries.append(
            MapEntry(
                id=str(item.get("id", "")),
                label=str(item.get("label", "")),
                launch_map=str(item.get("launch_map", "")),
                scene_file=str(item.get("scene_file", "")),
                sim_config_path=str(item.get("sim_config_path", "")),
            )
        )
    catalog = MapCatalog(
        default_map=str(raw.get("default_map", "")),
        maps=tuple(entries),
    )
    root = _project_root()
    for entry in catalog.maps:
        config_dir = Path(entry.sim_config_path)
        if not config_dir.is_absolute():
            config_dir = root / config_dir
        if not config_dir.is_dir():
            raise ValueError(
                f"map '{entry.id}' sim_config_path is not a directory: "
                f"{entry.sim_config_path}"
            )
        if not (config_dir / entry.scene_file).is_file():
            raise ValueError(
                f"map '{entry.id}' scene_file is missing: "
                f"{entry.sim_config_path}/{entry.scene_file}"
            )
    return catalog


def load_vehicle_models(
    raw: dict[str, Any],
    *,
    maps: MapCatalog,
    vehicles: tuple[str, ...],
) -> VehicleModelCatalog:
    """Parse the ``vehicle_models`` section and check its scene variants.

    Every option that names scene variants must cover each catalog map, and
    each named scene file must exist beside that map's robot configs. The
    section is optional; without it the default appearance stays selected.
    """
    if not raw:
        return default_vehicle_models()
    groups = []
    for vehicle, group_raw in raw.items():
        vehicle_id = str(vehicle).strip()
        if vehicle_id not in vehicles:
            raise ValueError(
                f"vehicle_models names unknown vehicle '{vehicle}', "
                f"expected one of {list(vehicles)}"
            )
        if not isinstance(group_raw, dict):
            raise ValueError(
                f"vehicle_models.{vehicle_id} must be a mapping"
            )
        options_raw = group_raw.get("options")
        if not isinstance(options_raw, list) or not options_raw:
            raise ValueError(
                f"vehicle_models.{vehicle_id}.options must be a non-empty list"
            )
        options = []
        for item in options_raw:
            if not isinstance(item, dict):
                raise ValueError(
                    f"vehicle_models.{vehicle_id}.options entries must be mappings"
                )
            scene_raw = item.get("scene_files", {}) or {}
            if not isinstance(scene_raw, dict):
                raise ValueError(
                    f"vehicle_models.{vehicle_id} scene_files must be a mapping"
                )
            options.append(
                VehicleModelOption(
                    id=str(item.get("id", "")),
                    label=str(item.get("label", "")),
                    scene_files=tuple(
                        (str(map_id).strip(), str(scene).strip())
                        for map_id, scene in scene_raw.items()
                    ),
                )
            )
        groups.append(
            VehicleModelGroup(
                vehicle_id,
                str(group_raw.get("default", "")),
                tuple(options),
            )
        )
    catalog = VehicleModelCatalog(groups=tuple(groups))
    root = _project_root()
    for group in catalog.groups:
        for option in group.options:
            if not option.scene_files:
                continue
            declared = {map_id for map_id, _ in option.scene_files}
            missing = set(maps.ids) - declared
            if missing:
                raise ValueError(
                    f"vehicle model '{group.vehicle}/{option.id}' needs a "
                    f"scene variant for every map; missing {sorted(missing)}"
                )
            for map_id, scene_file in option.scene_files:
                entry = maps.entry(map_id)
                if entry is None:
                    raise ValueError(
                        f"vehicle model '{group.vehicle}/{option.id}' names "
                        f"unknown map '{map_id}'"
                    )
                config_dir = Path(entry.sim_config_path)
                if not config_dir.is_absolute():
                    config_dir = root / config_dir
                if not (config_dir / scene_file).is_file():
                    raise ValueError(
                        f"vehicle model '{group.vehicle}/{option.id}' scene "
                        f"file is missing: {entry.sim_config_path}/{scene_file}"
                    )
    return catalog


def load_config(path: str | Path) -> AppConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError("config root must be a mapping")

    pas_raw = _section(raw, "projectairsim")
    camera_raw = _section(raw, "camera")
    motion_raw = _section(raw, "motion")

    offset = tuple(float(v) for v in camera_raw.get("offset_m", (1.8, 0.0, -0.8)))
    if len(offset) != 3:
        raise ValueError("camera.offset_m must contain three numbers")
    mount_rpy = tuple(
        float(v) for v in camera_raw.get("mount_rpy_deg", (0.0, -3.0, 0.0))
    )
    if len(mount_rpy) != 3:
        raise ValueError("camera.mount_rpy_deg must contain three numbers")

    projectairsim = ProjectAirSimSettings(
        address=str(pas_raw.get("address", "127.0.0.1")),
        port_topics=int(pas_raw.get("port_topics", 8991)),
        port_services=int(pas_raw.get("port_services", 8992)),
        timeout_s=float(pas_raw.get("timeout_s", 10.0)),
        scene_file=str(pas_raw.get("scene_file", "scene_city_dual.jsonc")),
        sim_config_path=str(
            pas_raw.get("sim_config_path", "config/scenes/city")
        ),
        delay_after_load_s=float(pas_raw.get("delay_after_load_s", 2.0)),
        rover_name=str(pas_raw.get("rover_name", "Rover1")),
        drone_name=str(pas_raw.get("drone_name", "Drone1")),
        front_camera=str(pas_raw.get("front_camera", "FrontCamera")),
        chase_camera=str(pas_raw.get("chase_camera", "Chase")),
        down_camera=str(pas_raw.get("down_camera", "DownCamera")),
        uncapturable_views=tuple(
            str(view) for view in pas_raw.get("uncapturable_views", ())
        ),
    )

    simulator_raw = _section(raw, "simulator")
    simulator = SimulatorSettings(
        launch_script=str(
            simulator_raw.get(
                "launch_script", "scripts/launch_projectairsim_xpra.sh"
            )
        ),
        stop_script=str(
            simulator_raw.get(
                "stop_script", "scripts/stop_projectairsim_xpra.sh"
            )
        ),
        display=str(simulator_raw.get("display", ":104")),
        xpra_port=int(simulator_raw.get("xpra_port", 10010)),
        topics_port=int(simulator_raw.get("topics_port", 8991)),
        services_port=int(simulator_raw.get("services_port", 8992)),
        runtime_dir=str(simulator_raw.get("runtime_dir", "output/xpra")),
        map_catalog_file=str(
            simulator_raw.get("map_catalog_file", "maps.yaml")
        ),
        map_marker_file=str(
            simulator_raw.get("map_marker_file", "output/xpra/current_map")
        ),
        launch_timeout_s=float(simulator_raw.get("launch_timeout_s", 180.0)),
        stop_timeout_s=float(simulator_raw.get("stop_timeout_s", 40.0)),
        port_release_timeout_s=float(
            simulator_raw.get("port_release_timeout_s", 30.0)
        ),
        restart_attempts=int(simulator_raw.get("restart_attempts", 2)),
    )
    if simulator_raw and (
        simulator.topics_port != projectairsim.port_topics
        or simulator.services_port != projectairsim.port_services
    ):
        raise ValueError(
            "simulator.topics_port/services_port must match "
            "projectairsim.port_topics/port_services"
        )
    maps = load_map_catalog(
        Path(path).parent / simulator.map_catalog_file,
        fallback=MapEntry(
            id="default",
            label="默认地图",
            launch_map="cityenviron",
            scene_file=projectairsim.scene_file,
            sim_config_path=projectairsim.sim_config_path,
        ),
    )

    camera = CameraSettings(
        width=int(camera_raw.get("width", 1280)),
        height=int(camera_raw.get("height", 720)),
        hfov_deg=float(camera_raw.get("hfov_deg", 90.0)),
        far_m=float(camera_raw.get("far_m", 1000.0)),
        depth_is_range=bool(camera_raw.get("depth_is_range", True)),
        offset_m=offset,
        mount_rpy_deg=mount_rpy,
    )
    if camera.width <= 0 or camera.height <= 0:
        raise ValueError("camera dimensions must be positive")
    if not 0.0 < camera.hfov_deg < 180.0:
        raise ValueError("camera.hfov_deg must be in (0, 180)")
    if camera.far_m <= 0:
        raise ValueError("camera.far_m must be positive")

    drone_raw = _section(raw, "drone")
    drone_offset = tuple(
        float(v) for v in drone_raw.get("camera_offset_m", (1.0, 0.0, 0.0))
    )
    if len(drone_offset) != 3:
        raise ValueError("drone.camera_offset_m must contain three numbers")
    drone_mount = tuple(
        float(v)
        for v in drone_raw.get("camera_mount_rpy_deg", (0.0, 0.0, 0.0))
    )
    if len(drone_mount) != 3:
        raise ValueError(
            "drone.camera_mount_rpy_deg must contain three numbers"
        )
    drone = DroneSettings(
        speed_mps=float(drone_raw.get("speed_mps", 2.0)),
        move_timeout_s=float(drone_raw.get("move_timeout_s", 60.0)),
        takeoff_timeout_s=float(drone_raw.get("takeoff_timeout_s", 30.0)),
        land_timeout_s=float(drone_raw.get("land_timeout_s", 30.0)),
        readiness_stable_s=float(drone_raw.get("readiness_stable_s", 2.0)),
        readiness_poll_interval_s=float(
            drone_raw.get("readiness_poll_interval_s", 0.25)
        ),
        camera_offset_m=drone_offset,
        camera_mount_rpy_deg=drone_mount,
    )

    motion = MotionLimits(
        max_speed_mps=float(motion_raw.get("max_speed_mps", 3.0)),
        max_accel_mps2=float(motion_raw.get("max_accel_mps2", 1.5)),
        max_yaw_rate_dps=float(motion_raw.get("max_yaw_rate_dps", 30.0)),
        max_yaw_accel_dps2=float(motion_raw.get("max_yaw_accel_dps2", 60.0)),
        control_hz=float(motion_raw.get("control_hz", 20.0)),
        min_duration_s=float(motion_raw.get("min_duration_s", 1.0)),
    )

    depth_raw = _section(raw, "depth")
    depth = DepthSettings(
        mapping=str(depth_raw.get("mapping", "log_inverse_gray_v1")),
        min_m=float(depth_raw.get("min_m", 0.5)),
        max_m=float(depth_raw.get("max_m", 200.0)),
    )

    continuous_raw = motion_raw.get("continuous", {})
    if not isinstance(continuous_raw, dict):
        raise ValueError("motion.continuous must be a mapping")
    continuous_names = {item.name for item in fields(ContinuousTrackingSettings)}
    unknown = set(continuous_raw) - continuous_names
    if unknown:
        raise ValueError(
            f"unknown motion.continuous settings: {sorted(unknown)}"
        )
    continuous = ContinuousTrackingSettings(
        **{
            name: float(value)
            for name, value in continuous_raw.items()
        }
    )

    detection_raw = _section(raw, "target_detection")
    target_detection = TargetDetectionSettings(
        url=str(detection_raw.get("url", "")),
        model=str(detection_raw.get("model", "")),
        api_key=str(detection_raw.get("api_key", "")),
        api_key_env=str(detection_raw.get("api_key_env", "VLN_API_KEY")),
        timeout_s=float(detection_raw.get("timeout_s", 30.0)),
        reasoning_effort=str(detection_raw.get("reasoning_effort", "none")),
        request_format=str(detection_raw.get("request_format", "openai")),
        max_completion_tokens=int(
            detection_raw.get("max_completion_tokens", 2048)
        ),
        max_image_side=int(detection_raw.get("max_image_side", 1024)),
        minimum_depth_m=float(detection_raw.get("minimum_depth_m", 0.5)),
    )

    planner_raw = _section(raw, "trajectory_planner")
    trajectory_planner = TrajectoryPlannerSettings(
        url=str(planner_raw.get("url", "")),
        model=str(planner_raw.get("model", "")),
        api_key=str(planner_raw.get("api_key", "")),
        api_key_env=str(planner_raw.get("api_key_env", "VLN_API_KEY")),
        timeout_s=float(planner_raw.get("timeout_s", 30.0)),
        max_tokens=int(planner_raw.get("max_tokens", 256)),
        reasoning_effort=str(planner_raw.get("reasoning_effort", "none")),
        request_format=str(planner_raw.get("request_format", "openai")),
        point_count=int(planner_raw.get("point_count", 5)),
        max_step_m=float(planner_raw.get("max_step_m", 2.0)),
        max_yaw_step_deg=float(
            planner_raw.get("max_yaw_step_deg", 45.0)
        ),
    )
    if trajectory_planner.timeout_s <= 0:
        raise ValueError("trajectory_planner.timeout_s must be positive")
    if trajectory_planner.max_tokens <= 0:
        raise ValueError("trajectory_planner.max_tokens must be positive")

    server_raw = _section(raw, "server")
    bbox_raw = server_raw.get(
        "manual_bbox", [0.45, 0.82, 0.55, 0.92]
    )
    if not isinstance(bbox_raw, (list, tuple)) or len(bbox_raw) != 4:
        raise ValueError("server.manual_bbox must list four numbers")
    points_raw = server_raw.get("manual_surface_points", [])
    if not isinstance(points_raw, (list, tuple)):
        raise ValueError("server.manual_surface_points must be a list")
    points = []
    for point in points_raw:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(
                "server.manual_surface_points entries must be [u, v] pairs"
            )
        points.append((float(point[0]), float(point[1])))
    origins_raw = server_raw.get("allowed_origins", ["*"])
    if not isinstance(origins_raw, (list, tuple)):
        raise ValueError("server.allowed_origins must be a list")
    server = ServerSettings(
        host=str(server_raw.get("host", "127.0.0.1")),
        port=int(server_raw.get("port", 8000)),
        detector=str(server_raw.get("detector", "manual")),
        planner=str(server_raw.get("planner", "geometric")),
        manual_bbox=tuple(float(value) for value in bbox_raw),
        manual_surface_points=tuple(points),
        allowed_origins=tuple(str(origin) for origin in origins_raw),
        log_buffer_size=int(server_raw.get("log_buffer_size", 500)),
        reconnect_interval_s=float(server_raw.get("reconnect_interval_s", 2.0)),
        startup_timeout_s=float(server_raw.get("startup_timeout_s", 60.0)),
        video_fps=float(server_raw.get("video_fps", 10.0)),
        vehicles=tuple(
            str(vehicle) for vehicle in server_raw.get("vehicles", ("rover", "drone"))
        ),
        default_vehicle=str(server_raw.get("default_vehicle", "rover")),
    )
    vehicle_models = load_vehicle_models(
        _section(raw, "vehicle_models"),
        maps=maps,
        vehicles=server.vehicles,
    )

    sliding_raw = _section(raw, "sliding_window")
    sliding_names = {item.name for item in fields(SlidingWindowSettings)}
    unknown = set(sliding_raw) - sliding_names
    if unknown:
        raise ValueError(f"unknown sliding_window settings: {sorted(unknown)}")
    sliding_window = SlidingWindowSettings(**sliding_raw)

    free_raw = _section(raw, "free_navigation")
    free_names = {item.name for item in fields(FreeNavigationSettings)}
    unknown = set(free_raw) - free_names
    if unknown:
        raise ValueError(f"unknown free_navigation settings: {sorted(unknown)}")
    free_navigation = FreeNavigationSettings(**free_raw)

    completion_raw = _section(raw, "task_completion")
    task_completion = TaskCompletionSettings(
        url=str(completion_raw.get("url", "")),
        model=str(completion_raw.get("model", "")),
        api_key=str(completion_raw.get("api_key", "")),
        api_key_env=str(completion_raw.get("api_key_env", "VLN_API_KEY")),
        timeout_s=float(completion_raw.get("timeout_s", 20.0)),
        max_completion_tokens=int(
            completion_raw.get("max_completion_tokens", 2048)
        ),
        reasoning_effort=str(completion_raw.get("reasoning_effort", "none")),
        request_format=str(completion_raw.get("request_format", "openai")),
    )

    parser_raw = _section(raw, "task_parser")
    task_parser = TaskParserSettings(
        url=str(parser_raw.get("url", "")),
        model=str(parser_raw.get("model", "")),
        api_key=str(parser_raw.get("api_key", "")),
        api_key_env=str(parser_raw.get("api_key_env", "VLN_API_KEY")),
        timeout_s=float(parser_raw.get("timeout_s", 60.0)),
        max_completion_tokens=int(
            parser_raw.get("max_completion_tokens", 2048)
        ),
        reasoning_effort=str(parser_raw.get("reasoning_effort", "none")),
        request_format=str(parser_raw.get("request_format", "openai")),
    )

    visual_raw = _section(raw, "visual_goal_navigation")
    visual_names = {
        "enabled",
        "geometry_verified",
        "collision_check_enabled",
        "output_root",
        "limits",
        "completion_rules_file",
        "depth_foreground_filter",
    }
    unknown = set(visual_raw) - visual_names
    if unknown:
        raise ValueError(
            f"unknown visual-goal configuration: {sorted(unknown)}"
        )
    limits_raw = visual_raw.get("limits", {})
    if not isinstance(limits_raw, dict):
        raise ValueError("visual-goal limits must be a mapping")
    if "maximum_target_position_jump_m" in limits_raw:
        raise ValueError(
            "replace maximum_target_position_jump_m with "
            "target_position_update_radius_m (relative to first target anchor)"
        )
    if "target_center_margin" in limits_raw:
        raise ValueError(
            "remove target_center_margin: current target visibility is "
            "evaluated by completion model"
        )
    foreground_raw = visual_raw.get("depth_foreground_filter", {})
    if not isinstance(foreground_raw, dict):
        raise ValueError("depth_foreground_filter must be a mapping")
    flags = {}
    for name, default in (
        ("enabled", True),
        ("geometry_verified", False),
        ("collision_check_enabled", False),
    ):
        value = visual_raw.get(name, default)
        if type(value) is not bool:
            raise ValueError(f"{name} must be boolean")
        flags[name] = value
    completion_rules_file = visual_raw.get("completion_rules_file")
    if completion_rules_file is not None and (
        not isinstance(completion_rules_file, str)
        or not completion_rules_file.strip()
    ):
        raise ValueError("completion_rules_file must be non-empty text")
    visual_goal_navigation = VisualGoalNavigationSettings(
        **flags,
        output_root=str(visual_raw.get("output_root", "output/visual_goal")),
        limits=VisualGoalLimits(**limits_raw),
        completion_rules_file=completion_rules_file,
        completion_rules=(
            load_completion_rules(path, completion_rules_file)
            if flags["enabled"]
            else VisualGoalTriggerRules()
        ),
        depth_foreground_filter=DepthForegroundSettings(**foreground_raw),
    )

    return AppConfig(
        projectairsim=projectairsim,
        camera=camera,
        motion=motion,
        maps=maps,
        vehicle_models=vehicle_models,
        simulator=simulator,
        drone=drone,
        target_detection=target_detection,
        trajectory_planner=trajectory_planner,
        server=server,
        depth=depth,
        continuous=continuous,
        sliding_window=sliding_window,
        task_completion=task_completion,
        task_parser=task_parser,
        visual_goal_navigation=visual_goal_navigation,
        free_navigation=free_navigation,
    )


def load_completion_rules(
    config_path: str | Path, filename: str | None = None
) -> VisualGoalTriggerRules:
    """Load the reference visual-goal completion rules.

    With an explicit filename the path is resolved relative to the main config
    file; otherwise it falls back to this project's
    ``config/tasks/visual_goal_navigation.yaml``.
    """
    path = (
        Path(config_path).parent / filename
        if filename is not None
        else Path(__file__).resolve().parent.parent
        / "config"
        / "tasks"
        / "visual_goal_navigation.yaml"
    )
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or set(raw) - {
        f.name for f in fields(VisualGoalTriggerRules)
    }:
        raise ValueError("invalid visual-goal completion rules")
    options = dict(raw)
    if "required_conditions" in options:
        if not isinstance(options["required_conditions"], list):
            raise ValueError("required_conditions must be a list")
        options["required_conditions"] = tuple(options["required_conditions"])
    return VisualGoalTriggerRules(**options)