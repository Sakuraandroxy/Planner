"""Direction estimation function registry."""

from config import cfg
from agent.functions.common.config_access import first_value, function_section
from agent.functions.direction.base import BaseDirectionEstimator

_DIRECTION_REGISTRY = {}


def register_direction(name):
    def wrapper(cls):
        _DIRECTION_REGISTRY[name] = cls
        return cls

    return wrapper


def build_direction_estimator():
    from agent.functions.direction.three_dg_estimator import ThreeDGDirectionEstimator  # noqa: F401

    direction_cfg = function_section(cfg, "DIRECTION")
    ag = cfg.get("AGENT", {}) or {}
    name = first_value(direction_cfg.get("NAME"), ag.get("DIRECTION"), default="three_dg")
    if name not in _DIRECTION_REGISTRY:
        raise KeyError(f"Unknown direction estimator [{name}], registered={list(_DIRECTION_REGISTRY.keys())}")
    return _DIRECTION_REGISTRY[name]()


__all__ = ["BaseDirectionEstimator", "build_direction_estimator", "register_direction"]
