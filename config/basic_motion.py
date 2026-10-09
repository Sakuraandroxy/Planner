"""Configuration decoding only; workflows receive domain values, not YAML."""
from dataclasses import dataclass

from planner.domain.basic_motion import ConfirmedFreeSpace


@dataclass(frozen=True)
class BasicMotionConfig:
    free_space: ConfirmedFreeSpace = ConfirmedFreeSpace()


def basic_motion_config(raw: dict) -> BasicMotionConfig:
    if not isinstance(raw, dict) or set(raw)-{"free_space"}:
        raise ValueError("basic_motion accepts only a free_space mapping")
    space = raw.get("free_space", {})
    allowed = {"confirmed", "minimum_world_ned_m", "maximum_world_ned_m", "clearance_radius_m"}
    if not isinstance(space, dict) or set(space)-allowed:
        raise ValueError("unknown basic_motion.free_space settings")
    values = dict(space)
    for name in ("minimum_world_ned_m", "maximum_world_ned_m"):
        if name in values and isinstance(values[name], (list, tuple)):
            values[name] = tuple(values[name])
    return BasicMotionConfig(ConfirmedFreeSpace(**values))
