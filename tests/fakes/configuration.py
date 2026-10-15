"""Offline configurations derived from the single maintained runtime YAML."""
from pathlib import Path
import shutil

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAIN_CONFIG = PROJECT_ROOT / "config" / "base.yaml"
VISUAL_RULES = PROJECT_ROOT / "config" / "tasks" / "visual_goal_navigation.yaml"
VISUAL_RULES_REFERENCE = "tasks/visual_goal_navigation.yaml"


def main_config_data() -> dict:
    """Return fresh data; tests never modify the maintained configuration."""
    raw = yaml.safe_load(MAIN_CONFIG.read_text(encoding="utf-8"))
    # Keep local credentials out of temporary offline fixtures and failure output.
    for role in ("task_parser", "trajectory_planner", "target_detection", "task_completion"):
        raw[role]["api_key"] = ""
    return raw


def visual_config_data() -> dict:
    """Synthetic calibrated camera and window settings, not real-scene confirmation."""
    raw = main_config_data()
    raw["airsim"]["depth_horizontal_fov_deg"] = 90.0
    raw["sliding_window"].update(enabled=True, max_observation_age_s=45.0)
    return raw


def alternate_backend_config_data() -> dict:
    raw = main_config_data()
    raw["visual_goal_navigation"]["enabled"] = False
    raw["trajectory_planner"] = {
        "url": "https://api.deepseek.com",
        "model": "deepseek-flash",
        "api_key": "",
        "api_key_env": "DEEPSEEK_API_KEY",
        "require_api_key": True,
        "thinking": "disabled",
        "timeout_s": 120,
    }
    return raw


def write_config(path: Path, raw: dict, *, copy_visual_rules: bool = True) -> Path:
    """Copy the known rule fixture beside a temporary config when explicitly used.

    Other paths are never repaired. Existing rule files are never overwritten,
    so a missing/custom/malformed-rule test can exercise the actual loader.
    """
    path.write_text(yaml.safe_dump(raw, allow_unicode=True, sort_keys=False), encoding="utf-8")
    visual = raw.get("visual_goal_navigation", {})
    if (copy_visual_rules and visual.get("enabled") is True and
            visual.get("completion_rules_file") == VISUAL_RULES_REFERENCE):
        target = path.parent / VISUAL_RULES_REFERENCE
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(VISUAL_RULES, target)
    return path
