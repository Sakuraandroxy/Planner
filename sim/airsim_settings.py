"""AirSim settings helpers."""

from __future__ import annotations

import copy
import json
import time
from pathlib import Path
from typing import Any, Dict

from config import cfg


LOCAL_AIRSIM_SETTINGS_PATH = Path.home() / "Documents" / "AirSim" / "settings.json"


def _deep_merge(base: Dict[str, Any], updates: Dict[str, Any]) -> Dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def load_local_airsim_settings() -> Dict[str, Any]:
    """Load the user's local AirSim settings.json if it exists."""
    if not LOCAL_AIRSIM_SETTINGS_PATH.exists():
        return {}
    try:
        return json.loads(LOCAL_AIRSIM_SETTINGS_PATH.read_text(encoding="utf-8-sig"))
    except Exception as exc:
        print(f"[AirSimSettings] failed to read {LOCAL_AIRSIM_SETTINGS_PATH}: {exc}")
        return {}


def build_airsim_settings_with_overrides() -> Dict[str, Any]:
    """Update capture settings while preserving user-configured camera poses."""
    sim_cfg = cfg.get("SIM", {})
    base = load_local_airsim_settings()
    vehicle_name = str(sim_cfg.get("VEHICLE_NAME", "Drone_1") or "Drone_1")
    configured = sim_cfg.get("CAMERAS") or {}
    if isinstance(configured, dict):
        camera_specs = {str(camera_id): dict(value or {}) for camera_id, value in configured.items()}
    else:
        camera_specs = {}
        for item in list(configured or []):
            if isinstance(item, str):
                camera_specs[str(item)] = {}
            elif isinstance(item, dict):
                camera_id = item.get("ID") or item.get("id") or item.get("CAMERA_ID")
                if camera_id:
                    camera_specs[str(camera_id)] = dict(item)
    if not camera_specs:
        camera_specs = {
            "front_center": {"ROLE": "primary"},
            "down_center": {"ROLE": "auxiliary"},
        }

    existing_cameras = (
        ((base.get("Vehicles") or {}).get(vehicle_name) or {}).get("Cameras") or {}
    )
    preserve_pose = bool(sim_cfg.get("PRESERVE_CAMERA_POSE_ON_SETTINGS_WRITE", True))
    camera_overrides = {}
    for index, (camera_id, spec) in enumerate(camera_specs.items()):
        roles = spec.get("ROLES", spec.get("ROLE", []))
        if isinstance(roles, str):
            roles = [roles]
        role_names = {str(role).strip().lower() for role in list(roles or [])}
        primary = "primary" in role_names or (not role_names and index == 0)
        if primary:
            rgb_width = int(spec.get("WIDTH", spec.get("RGB_WIDTH", sim_cfg.get("FRONT_WIDTH", 1920))))
            rgb_height = int(spec.get("HEIGHT", spec.get("RGB_HEIGHT", sim_cfg.get("FRONT_HEIGHT", 1080))))
            rgb_fov = float(spec.get("FOV", sim_cfg.get("FRONT_FOV", 90)))
            depth_width = int(spec.get("DEPTH_WIDTH", sim_cfg.get("DEPTH_WIDTH", 256)))
            depth_height = int(spec.get("DEPTH_HEIGHT", sim_cfg.get("DEPTH_HEIGHT", 256)))
            depth_fov = float(spec.get("DEPTH_FOV", sim_cfg.get("DEPTH_FOV", rgb_fov)))
        else:
            rgb_width = int(spec.get("WIDTH", spec.get("RGB_WIDTH", sim_cfg.get("DOWN_WIDTH", 1920))))
            rgb_height = int(spec.get("HEIGHT", spec.get("RGB_HEIGHT", sim_cfg.get("DOWN_HEIGHT", 1080))))
            rgb_fov = float(spec.get("FOV", sim_cfg.get("DOWN_FOV", 90)))
            depth_width = int(spec.get("DEPTH_WIDTH", sim_cfg.get("DOWN_DEPTH_WIDTH", 256)))
            depth_height = int(spec.get("DEPTH_HEIGHT", sim_cfg.get("DOWN_DEPTH_HEIGHT", 256)))
            depth_fov = float(spec.get("DEPTH_FOV", sim_cfg.get("DOWN_DEPTH_FOV", rgb_fov)))
        camera_update = {
            "CaptureSettings": [
                {
                    "ImageType": 0,
                    "Width": rgb_width,
                    "Height": rgb_height,
                    "FOV_Degrees": rgb_fov,
                },
                {
                    "ImageType": 2,
                    "Width": depth_width,
                    "Height": depth_height,
                    "FOV_Degrees": depth_fov,
                },
            ],
        }
        existing = dict(existing_cameras.get(camera_id) or {})
        if not existing or not preserve_pose:
            pose_defaults = {
                "X": 1.0 if primary else 0.0,
                "Y": 0.0,
                "Z": 0.0,
                "Pitch": 0.0 if primary else -90.0,
                "Roll": 0.0,
                "Yaw": 0.0,
            }
            for key, default in pose_defaults.items():
                camera_update[key] = float(spec.get(key, spec.get(f"DEFAULT_{key}", default)))
        camera_overrides[camera_id] = camera_update

    overrides = {
        "SettingsVersion": base.get("SettingsVersion", 1.2),
        "SimMode": base.get("SimMode", "Multirotor"),
        "ClockSpeed": base.get("ClockSpeed", 1),
        "Vehicles": {
            vehicle_name: {
                "VehicleType": "SimpleFlight",
                "AutoCreate": True,
                "Cameras": camera_overrides,
            }
        },
    }
    return _deep_merge(base, overrides)


def write_local_airsim_settings(make_backup: bool = True) -> Path:
    """Write local AirSim settings.json using config/default.yaml SIM overrides."""
    settings = build_airsim_settings_with_overrides()
    LOCAL_AIRSIM_SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)

    if make_backup and LOCAL_AIRSIM_SETTINGS_PATH.exists():
        stamp = time.strftime("%Y%m%d_%H%M%S")
        backup_path = LOCAL_AIRSIM_SETTINGS_PATH.with_name(f"settings.backup_{stamp}.json")
        backup_path.write_text(
            LOCAL_AIRSIM_SETTINGS_PATH.read_text(encoding="utf-8-sig"),
            encoding="utf-8",
        )

    LOCAL_AIRSIM_SETTINGS_PATH.write_text(
        json.dumps(settings, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return LOCAL_AIRSIM_SETTINGS_PATH
