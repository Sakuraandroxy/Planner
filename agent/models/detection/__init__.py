"""Detection model registry."""

from config import cfg
from agent.functions.common.config_access import function_section
from agent.models.detection.base import BaseDetector, DetectionResult

_DETECTOR_REGISTRY = {}


def register_detector(name):
    def wrapper(cls):
        _DETECTOR_REGISTRY[name] = cls
        return cls

    return wrapper


def build_detector():
    from agent.models.detection.groundingdino_detector import GroundingDINODetector  # noqa: F401
    from agent.models.detection.noop_detector import NoopDetector  # noqa: F401
    from agent.models.detection.vlm_detector import VLMDetector  # noqa: F401

    perception_cfg = function_section(cfg, "PERCEPTION")
    name = perception_cfg.get("MODEL") or cfg.get("AGENT", {}).get("DETECTOR", "groundingdino")
    if name not in _DETECTOR_REGISTRY:
        raise KeyError(f"Unknown detector [{name}], registered={list(_DETECTOR_REGISTRY.keys())}")
    return _DETECTOR_REGISTRY[name]()


__all__ = ["BaseDetector", "DetectionResult", "build_detector", "register_detector"]
