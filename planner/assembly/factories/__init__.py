"""Build detectors, planners and geometry from the shared configuration.

The CLI runner and the HTTP server both use these helpers so a detector or
planner mode always resolves to the same adapter.
"""

from __future__ import annotations

from .completion import build_completion_checker, build_trace_factory
from .detectors import build_detector, build_geometry
from .parsers import build_task_parser
from .planners import api_key, build_free_navigation_planner, build_planner

__all__ = [
    "api_key",
    "build_completion_checker",
    "build_detector",
    "build_free_navigation_planner",
    "build_geometry",
    "build_planner",
    "build_task_parser",
    "build_trace_factory",
]
