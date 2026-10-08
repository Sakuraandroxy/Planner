from dataclasses import dataclass, fields
import math

from planner.domain.visual_goal_limits import VisualGoalLimits


@dataclass(frozen=True)
class VisualGoalConfig:
    enabled: bool = False
    geometry_verified: bool = False
    initial_clearance_confirmed: bool = False
    depth_horizontal_fov_deg: float = 90.0
    output_root: str = "output/visual_goal"
    limits: VisualGoalLimits = VisualGoalLimits()


def visual_goal_config(raw):
    if not isinstance(raw, dict):
        raise ValueError("visual_goal_navigation must be a mapping")
    names = {f.name for f in fields(VisualGoalConfig)}
    if set(raw) - names:
        raise ValueError(f"unknown visual-goal configuration: {sorted(set(raw)-names)}")
    options = {k: v for k, v in raw.items() if k != "limits"}
    for key in ("enabled", "geometry_verified", "initial_clearance_confirmed"):
        if key in options and not isinstance(options[key], bool):
            raise ValueError(f"{key} must be boolean")
    value = VisualGoalConfig(**options, limits=VisualGoalLimits(**raw.get("limits", {})))
    if isinstance(value.depth_horizontal_fov_deg, bool) or not math.isfinite(value.depth_horizontal_fov_deg) or not (
        0 < value.depth_horizontal_fov_deg < 180
    ):
        raise ValueError("depth FOV must be finite and between 0 and 180")
    if not isinstance(value.output_root, str) or not value.output_root.strip():
        raise ValueError("visual goal output_root must be non-empty text")
    return value
