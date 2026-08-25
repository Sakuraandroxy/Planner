"""Mission memory package."""

from __future__ import annotations

from config import cfg

from agent.functions.common.config_access import function_section
from agent.functions.memory.mission_memory import (
    MissionMemory,
    is_return_target_stage,
    is_same_target_stage,
    is_view_relative_stage,
    view_relative_direction,
)
from agent.functions.memory.schemas import (
    AppearancePrototype,
    MemoryCompletionDecision,
    TargetInstanceBelief,
    TargetMemory,
)


def build_mission_memory() -> MissionMemory:
    """Build task-internal memory from FUNCTIONS.MEMORY."""
    config = function_section(cfg, "MEMORY")
    return MissionMemory(config=config, sim_config=cfg.get("SIM", {}) or {})


__all__ = [
    "AppearancePrototype",
    "MemoryCompletionDecision",
    "MissionMemory",
    "TargetInstanceBelief",
    "TargetMemory",
    "build_mission_memory",
    "is_return_target_stage",
    "is_same_target_stage",
    "is_view_relative_stage",
    "view_relative_direction",
]
