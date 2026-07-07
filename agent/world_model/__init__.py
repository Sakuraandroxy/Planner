"""
agent/world_model/__init__.py — 世界模型注册表。

用法:
    from agent.world_model import build_world_model
    wm = build_world_model()
    result = wm.score(...)
"""

from typing import Optional
from agent.world_model.base import BaseWorldModel, WorldModelResult

_world_model_registry = {}


def register_world_model(name: str):
    """装饰器：注册世界模型实现。"""
    def decorator(cls):
        _world_model_registry[name] = cls
        return cls
    return decorator


def build_world_model(name: str = None) -> Optional[BaseWorldModel]:
    """根据配置构建世界模型实例。

    如果配置中 WORLD_MODEL.ENABLED=false，返回 None。
    """
    from config import cfg
    wm_cfg = cfg.get("WORLD_MODEL", {})
    if not wm_cfg.get("ENABLED", False):
        return None

    if name is None:
        name = wm_cfg.get("NAME", "api_world_model")

    cls = _world_model_registry.get(name)
    if cls is None:
        print(f"  [WorldModel] ⚠️ 未找到实现 '{name}'")
        return None

    return cls()


# 自动注册内置实现
from agent.world_model.api_world_model import ApiWorldModel  # noqa: E402, F401
