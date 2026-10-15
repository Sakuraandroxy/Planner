"""Visual-navigation completion rules; no backend settings."""
from dataclasses import fields
from pathlib import Path
import yaml
from planner.domain.visual_goal_completion import VisualGoalTriggerRules


def completion_rules(config_path, filename=None):
    path = (Path(config_path).parent / filename if filename is not None else
            Path(__file__).parent / "tasks" / "visual_goal_navigation.yaml")
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or set(raw)-{f.name for f in fields(VisualGoalTriggerRules)}:
        raise ValueError("invalid visual-goal completion rules")
    options = dict(raw)
    if "required_conditions" in options:
        if not isinstance(options["required_conditions"], list):
            raise ValueError("required_conditions must be a list")
        options["required_conditions"] = tuple(options["required_conditions"])
    return VisualGoalTriggerRules(**options)
