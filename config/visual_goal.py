from dataclasses import dataclass, fields

from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.domain.visual_goal_completion import VisualGoalTriggerRules


@dataclass(frozen=True)
class VisualGoalConfig:
    enabled: bool = True
    geometry_verified: bool = False
    collision_check_enabled: bool = False
    output_root: str = "output/visual_goal"
    limits: VisualGoalLimits = VisualGoalLimits()
    completion_rules_file: str | None = None
    completion_rules: VisualGoalTriggerRules = VisualGoalTriggerRules()


def visual_goal_config(raw, *, completion_rules=None):
    if not isinstance(raw, dict):
        raise ValueError("visual_goal_navigation must be a mapping")
    names = {f.name for f in fields(VisualGoalConfig)} - {"completion_rules"}
    if set(raw) - names:
        raise ValueError(f"unknown visual-goal configuration: {sorted(set(raw)-names)}")
    limits = raw.get('limits', {})
    if not isinstance(limits, dict):
        raise ValueError('visual-goal limits must be a mapping')
    if 'maximum_target_position_jump_m' in limits:
        raise ValueError('replace maximum_target_position_jump_m with target_position_update_radius_m (relative to first target anchor)')
    if 'target_center_margin' in limits:
        raise ValueError('remove target_center_margin: current target visibility is evaluated by completion model')
    options = {k: v for k, v in raw.items() if k != "limits"}
    for key in ("enabled", "geometry_verified", "collision_check_enabled"):
        if key in options and not isinstance(options[key], bool):
            raise ValueError(f"{key} must be boolean")
    if "completion_rules_file" in options and (not isinstance(options["completion_rules_file"], str) or not options["completion_rules_file"].strip()):
        raise ValueError("completion_rules_file must be non-empty text")
    value = VisualGoalConfig(**options, limits=VisualGoalLimits(**raw.get("limits", {})),
                             completion_rules=completion_rules or VisualGoalTriggerRules())
    if not isinstance(value.output_root, str) or not value.output_root.strip():
        raise ValueError("visual goal output_root must be non-empty text")
    return value
