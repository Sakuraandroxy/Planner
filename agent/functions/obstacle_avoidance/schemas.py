"""Compact data contracts for depth-based obstacle avoidance."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


def now_s() -> float:
    return time.perf_counter()


@dataclass
class ObstacleCell:
    """One compact obstacle-map cell.

    The avoider stores only sparse cells, never raw depth images or point clouds.
    """

    key: str
    center_world: List[float]
    confidence: float = 0.5
    observation_count: int = 1
    last_seen_s: float = field(default_factory=now_s)
    source_view: str = "front"

    def age_s(self) -> float:
        return max(0.0, now_s() - float(self.last_seen_s))

    def to_summary_dict(self) -> Dict[str, Any]:
        return {
            "key": self.key,
            "world": [round(float(v), 2) for v in self.center_world[:3]],
            "confidence": round(float(self.confidence), 3),
            "observations": int(self.observation_count),
            "age_s": round(self.age_s(), 1),
            "view": self.source_view,
        }


@dataclass
class AvoidanceResult:
    """Result of filtering a body-frame cumulative path."""

    waypoints: List[List[float]]
    changed: bool = False
    blocked: bool = False
    reason: str = ""
    obstacle_body: Optional[List[float]] = None
    details: Dict[str, Any] = field(default_factory=dict)

