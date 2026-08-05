"""Shared data contracts between agent functions and model backends."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence


@dataclass
class ImageBundle:
    """Front/down RGB and optional depth images for one decision moment."""

    front_rgb: Any = None
    down_rgb: Any = None
    front_depth: Any = None
    down_depth: Any = None
    timing: Dict[str, float] = field(default_factory=dict)


@dataclass
class Detection:
    """Model-agnostic target detection result."""

    visible: bool
    view: str = "none"
    bbox: Optional[List[int]] = None
    score: float = 0.0
    label: str = ""
    depth_median: Optional[float] = None
    # Sparse normalized image samples ``[u, v, depth_m]`` extracted from the
    # depth image.  They preserve target surface geometry without retaining a
    # full frame or point cloud in long-lived memory.
    surface_depth_samples: Optional[List[List[float]]] = None
    source: str = ""
    raw: Any = None


@dataclass
class PlanOutput:
    """Model-agnostic planning result.

    ``waypoint_format`` is part of the contract so runtime code knows whether
    points are cumulative from the current pose or incremental from the previous
    waypoint.
    """

    waypoints: List[List[float]] = field(default_factory=list)
    waypoint_format: str = "cumulative_body"
    done: bool = False
    reasoning: str = ""
    rejection_reason: str = ""
    raw: Any = None


@dataclass
class CompletionOutput:
    checked: bool = False
    done: bool = False
    target_detected: Optional[bool] = None
    accepted_view: str = "none"
    detection: Optional[Detection] = None
    reason: str = ""
    raw: Any = None


@dataclass
class Pose2D:
    position: Sequence[float]
    yaw_deg: float
