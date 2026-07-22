"""Candidate trajectory data structures."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List

from agent.functions.common.trajectory import (
    actions_to_cumulative_body_waypoints,
    cumulative_to_step_body_waypoints,
    normalize_xyz_waypoints,
)


def cumulative_to_step_waypoints(waypoints) -> List[List[float]]:
    """Convert cumulative body-frame waypoints to step-wise displacements."""
    return cumulative_to_step_body_waypoints(waypoints)


def candidate_dict_to_world_model(candidate: Dict[str, Any]) -> Dict[str, Any]:
    """Convert a candidate dict to the world-model input format."""
    out = dict(candidate)
    if out.get("waypoint_mode") == "relative_step_body_xyz":
        out["waypoints"] = normalize_xyz_waypoints(out.get("waypoints", []))
        out.setdefault(
            "cumulative_waypoints",
            normalize_xyz_waypoints(out.get("cumulative_waypoints", [])),
        )
        return out

    cumulative = normalize_xyz_waypoints(out.get("waypoints", []))
    if not cumulative and out.get("actions"):
        cumulative = actions_to_cumulative_body_waypoints(out.get("actions", []))
    out["cumulative_waypoints"] = cumulative
    out["waypoints"] = cumulative_to_step_waypoints(cumulative)
    out["waypoint_mode"] = "relative_step_body_xyz"
    return out


@dataclass
class CandidateTrajectory:
    """Unified candidate trajectory representation."""

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

    def to_world_model_dict(self) -> Dict[str, Any]:
        return candidate_dict_to_world_model(self.to_dict())


@dataclass
class CandidatePreparationResult:
    """Candidate trajectory preparation result."""

    all_candidates: List[CandidateTrajectory] = field(default_factory=list)
    wm_candidates: List[CandidateTrajectory] = field(default_factory=list)
    planner_selected_index: int = 0
    prefilter_reason: str = ""
