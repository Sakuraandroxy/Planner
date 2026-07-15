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
    """Use local settings.json as the base, then override key SIM camera params."""
    sim_cfg = cfg.get("SIM", {})
    base = load_local_airsim_settings()

    overrides = {
        "SettingsVersion": base.get("SettingsVersion", 1.2),
        "SimMode": base.get("SimMode", "Multirotor"),
        "ClockSpeed": base.get("ClockSpeed", 1),
        "Vehicles": {
            "Drone_1": {
                "VehicleType": "SimpleFlight",
                "AutoCreate": True,
                "Cameras": {
                    "front_center": {
                        "X": 1,
                        "Y": 0,
                        "Z": 0,
                        "Pitch": 0,
                        "Roll": 0,
                        "Yaw": 0,
                        "CaptureSettings": [
                            {
                                "ImageType": 0,
                                "Width": int(sim_cfg.get("FRONT_WIDTH", 1920)),
                                "Height": int(sim_cfg.get("FRONT_HEIGHT", 1080)),
                                "FOV_Degrees": float(sim_cfg.get("FRONT_FOV", 90)),
                            },
                            {
                                "ImageType": 2,
                                "Width": int(sim_cfg.get("DEPTH_WIDTH", 640)),
                                "Height": int(sim_cfg.get("DEPTH_HEIGHT", 360)),
                                "FOV_Degrees": float(sim_cfg.get("DEPTH_FOV", sim_cfg.get("FRONT_FOV", 90))),
                            },
                        ],
                    },
                    "down_center": {
                        "X": 0,
                        "Y": 0,
                        "Z": 0,
                        "Pitch": -90,
                        "Roll": 0,
                        "Yaw": 0,
                        "CaptureSettings": [
                            {
                                "ImageType": 0,
                                "Width": int(sim_cfg.get("DOWN_WIDTH", 640)),
                                "Height": int(sim_cfg.get("DOWN_HEIGHT", 640)),
                                "FOV_Degrees": float(sim_cfg.get("DOWN_FOV", 90)),
                            },
                            {
                                "ImageType": 2,
                                "Width": int(sim_cfg.get("DOWN_DEPTH_WIDTH", sim_cfg.get("DEPTH_WIDTH", 640))),
                                "Height": int(sim_cfg.get("DOWN_DEPTH_HEIGHT", sim_cfg.get("DEPTH_HEIGHT", 360))),
                                "FOV_Degrees": float(sim_cfg.get("DOWN_DEPTH_FOV", sim_cfg.get("DOWN_FOV", 90))),
                            },
                        ],
                    },
                },
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
