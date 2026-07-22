"""World-model registry."""

from __future__ import annotations

from typing import Optional

from agent.models.world_model.base import BaseWorldModel, WorldModelResult

_world_model_registry = {}


def register_world_model(name: str):
    def decorator(cls):
        _world_model_registry[name] = cls
        return cls

    return decorator


def build_world_model(name: str = None) -> Optional[BaseWorldModel]:
    from config import cfg
    from agent.models.world_model.api_world_model import ApiWorldModel  # noqa: F401

    wm_cfg = cfg.get("WORLD_MODEL", {})
    if not wm_cfg.get("ENABLED", False):
        return None
    if name is None:
        name = wm_cfg.get("NAME", "api_world_model")
    cls = _world_model_registry.get(name)
    if cls is None:
        print(f"  [WorldModel] implementation not found: {name}")
        return None
    return cls()


__all__ = ["BaseWorldModel", "WorldModelResult", "build_world_model", "register_world_model"]
