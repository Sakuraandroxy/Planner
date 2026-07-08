"""候选轨迹数据结构。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List


@dataclass
class CandidateTrajectory:
    """统一候选轨迹表示。"""

    actions: List[str] = field(default_factory=list)
    waypoints: List[List[float]] = field(default_factory=list)
    reason: str = ""
    delta: List[float] = field(default_factory=list)
    scale: float = 1.0
    source: str = "planner"
    pre_score: float = 0.0
    confidence: float = 0.0
    score_breakdown: Dict[str, float] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "actions": self.actions,
            "waypoints": self.waypoints,
            "reason": self.reason,
            "delta": self.delta,
            "scale": self.scale,
            "source": self.source,
            "pre_score": self.pre_score,
            "confidence": self.confidence,
            "score_breakdown": self.score_breakdown,
            "metadata": self.metadata,
        }


@dataclass
class CandidatePreparationResult:
    """候选轨迹准备结果。"""

    all_candidates: List[CandidateTrajectory] = field(default_factory=list)
    wm_candidates: List[CandidateTrajectory] = field(default_factory=list)
    planner_selected_index: int = 0
    prefilter_reason: str = ""
