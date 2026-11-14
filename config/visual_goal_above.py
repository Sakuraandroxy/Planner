"""Configuration of the above workflow; no model credentials or flight policy."""
from dataclasses import dataclass, fields
from pathlib import Path
import yaml

from planner.domain.visual_goal_above import positive


@dataclass(frozen=True)
class VisualGoalAboveConfig:
    enabled: bool = True
    default_height_above_m: float = 5.0
    height_tolerance_m: float = 0.5
    observation_interval_s: float = 1.0
    view_checkpoint_interval_s: float = 10.0
    completion_interval_s: float = 5.0
    detection_interval_s: float = 5.0
    maximum_duration_s: float = 600.0
    maximum_planning_calls: int = 100
    model_request_timeout_s: float = 120.0
    output_root: str = "output/visual_goal_above"

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError("visual_goal_above.enabled must be boolean")
        for name in ("default_height_above_m", "height_tolerance_m", "observation_interval_s",
                     "view_checkpoint_interval_s", "completion_interval_s", "detection_interval_s",
                     "maximum_duration_s", "model_request_timeout_s"):
            if not positive(getattr(self, name)):
                raise ValueError(f"visual_goal_above.{name} must be finite and positive")
        if type(self.maximum_planning_calls) is not int or self.maximum_planning_calls < 1:
            raise ValueError("maximum_planning_calls must be a positive integer")
        if not isinstance(self.output_root, str) or not self.output_root.strip():
            raise ValueError("above output_root must be non-empty text")


def visual_goal_above_config(raw, config_path):
    if not isinstance(raw, dict):
        raise ValueError("visual_goal_above must be a mapping")
    names = {f.name for f in fields(VisualGoalAboveConfig)}
    if set(raw)-names-{"settings_file"}:
        raise ValueError("unknown visual_goal_above settings")
    # Existing configurations remain valid. The packaged task file supplies defaults.
    path = (Path(config_path).parent / raw["settings_file"] if "settings_file" in raw else
            Path(__file__).parent / "tasks" / "visual_goal_above.yaml")
    options = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(options, dict) or set(options)-names:
        raise ValueError("invalid visual_goal_above task file")
    options.update({k: v for k, v in raw.items() if k != "settings_file"})
    return VisualGoalAboveConfig(**options)
