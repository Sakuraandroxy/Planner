"""Planner model registry."""

from config import cfg
from agent.functions.common.config_access import first_value, function_section
from agent.models.planner.base import BasePlanner, TrajectoryResult

_PLANNER_REGISTRY = {}
Planner = None


def register_planner(name):
    def wrapper(cls):
        _PLANNER_REGISTRY[name] = cls
        return cls

    return wrapper


def build_planner():
    from agent.models.planner.api_atomic_planner import ApiAtomicPlanner  # noqa: F401
    from agent.models.planner.qwen_planner import QwenPlanner  # noqa: F401
    from agent.models.planner.sliding_window_planner import SlidingWindowQwenPlanner  # noqa: F401

    global Planner
    planning_cfg = function_section(cfg, "PLANNING")
    ag = cfg.get("AGENT", {}) or {}
    name = first_value(planning_cfg.get("MODEL"), ag.get("PLANNER"), default="qwen_sliding_window_planner")
    if name not in _PLANNER_REGISTRY:
        raise KeyError(f"Unknown planner [{name}], registered={list(_PLANNER_REGISTRY.keys())}")
    if Planner is None:
        Planner = QwenPlanner
    return _PLANNER_REGISTRY[name]()


__all__ = ["BasePlanner", "TrajectoryResult", "build_planner", "register_planner"]
