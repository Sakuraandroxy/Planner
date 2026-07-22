"""Named loader used by the model/function split.

The config should expose stable names such as ``dual_view_detector`` or
``qwen_incremental``.  Python import paths stay here, so users do not need to
edit long module paths when switching implementations.
"""

from __future__ import annotations

import importlib
from typing import Any, Mapping


FUNCTION_REGISTRY = {
    "dual_view_detector": "agent.functions.perception.dual_view_detector.DualViewDetector",
    "sliding_window_planning": "agent.functions.planning.sliding_window_planning.SlidingWindowPlanningFunction",
    "world_queue": "agent.functions.trajectory_queue.world_queue.WorldTrajectoryQueue",
    "fast_slow_controller": "agent.functions.fast_slow.controller.FastSlowController",
}

MODEL_REGISTRY = {
    "detector_default": "agent.models.detection.configured_detector.ConfiguredDetectorModel",
    "groundingdino": "agent.models.detection.groundingdino_detector.GroundingDinoDetector",
    "vlm_detector": "agent.models.detection.vlm_detector.VLMDetector",
    "qwen_cumulative": "agent.models.planner.configured_qwen.CumulativeQwenModel",
    "qwen_incremental": "agent.models.planner.configured_qwen.IncrementalQwenModel",
    "qwen_planner": "agent.models.planner.qwen_planner.QwenPlanner",
    "api_atomic_planner": "agent.models.planner.api_atomic_planner.APIAtomicPlanner",
    "api_world_model": "agent.models.world_model.api_world_model.APIWorldModel",
}


def import_string(path: str):
    """Import ``package.module:Class`` or ``package.module.Class``."""
    if not path or not isinstance(path, str):
        raise ValueError("import path must be a non-empty string")
    module_path, sep, attr = path.replace(":", ".").rpartition(".")
    if not sep or not module_path or not attr:
        raise ValueError(f"invalid import path: {path!r}")
    module = importlib.import_module(module_path)
    return getattr(module, attr)


def resolve_registered_name(name: str, registry: Mapping[str, str] | None = None) -> str:
    """Resolve a short config name to an import path.

    ``registry`` can be supplied by a caller that wants a smaller namespace.
    Full import paths are still accepted for developer experiments, but normal
    project configs should use short names.
    """
    if not name or not isinstance(name, str):
        raise ValueError("registered name must be a non-empty string")
    if "." in name:
        return name
    lookup = registry or {**FUNCTION_REGISTRY, **MODEL_REGISTRY}
    if name not in lookup:
        known = ", ".join(sorted(lookup))
        raise KeyError(f"unknown implementation name {name!r}; known: {known}")
    return lookup[name]


def build_from_config(
    config: Mapping[str, Any],
    default_path: str | None = None,
    registry: Mapping[str, str] | None = None,
    **kwargs,
):
    """Build an object from a config section.

    Accepted keys are ``IMPLEMENTATION`` or ``NAME``.  User-facing configs use
    short implementation names; only this registry owns import paths.
    """
    cfg = dict(config or {})
    name = (
        cfg.get("IMPLEMENTATION")
        or cfg.get("NAME")
        or default_path
    )
    path = resolve_registered_name(str(name), registry=registry) if name else default_path
    cls = import_string(str(path))
    return cls(cfg, **kwargs)
