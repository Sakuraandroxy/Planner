"""Configuration loading for the ProjectAirSim VLN skeleton.

Public names are re-exported here so ``from planner.config import load_config``
keeps working across layers; the submodules hold the implementation.
"""

from __future__ import annotations

from .camera import build_camera_settings
from .engine_family import (
    FamilyCatalog,
    SimulatorFamily,
    parse_family_catalog,
)
from .loader import (
    PROFILE_SECTIONS,
    AppConfig,
    load_completion_rules,
    load_config,
)
from .maps import MapCatalog, MapEntry, MapScene, load_map_catalog
from .paths import (
    available_profiles,
    default_config_path,
    default_profile,
    profile_config_path,
    profile_of,
    profiles_root,
    shared_or_profile,
)
from .profile import (
    PROFILE_SWITCHABLE_SECTIONS,
    ProfileInfo,
    profile_catalog,
    profile_drift,
)
from .sections import (
    CameraSettings,
    CollisionSettings,
    DepthSettings,
    DroneSettings,
    FreeNavigationSettings,
    ProjectAirSimSettings,
    ServerSettings,
    SimulatorSettings,
    TargetDetectionSettings,
    TaskCompletionSettings,
    TaskParserSettings,
    TrajectoryPlannerSettings,
    VisualGoalNavigationSettings,
)
from .vehicles import (
    VehicleModelCatalog,
    VehicleModelGroup,
    VehicleModelOption,
    default_vehicle_models,
    load_vehicle_models,
)

__all__ = [
    "AppConfig",
    "CameraSettings",
    "CollisionSettings",
    "DepthSettings",
    "DroneSettings",
    "FamilyCatalog",
    "FreeNavigationSettings",
    "MapCatalog",
    "MapEntry",
    "MapScene",
    "PROFILE_SECTIONS",
    "PROFILE_SWITCHABLE_SECTIONS",
    "ProfileInfo",
    "ProjectAirSimSettings",
    "ServerSettings",
    "SimulatorFamily",
    "SimulatorSettings",
    "TargetDetectionSettings",
    "TaskCompletionSettings",
    "TaskParserSettings",
    "TrajectoryPlannerSettings",
    "VehicleModelCatalog",
    "VehicleModelGroup",
    "VehicleModelOption",
    "VisualGoalNavigationSettings",
    "available_profiles",
    "build_camera_settings",
    "default_config_path",
    "default_profile",
    "default_vehicle_models",
    "load_completion_rules",
    "load_config",
    "load_map_catalog",
    "load_vehicle_models",
    "parse_family_catalog",
    "profile_catalog",
    "profile_config_path",
    "profile_drift",
    "profile_of",
    "profiles_root",
    "shared_or_profile",
]
