"""Immutable multi-surface evidence; IDs are local to one observation, not tracks."""
from dataclasses import dataclass

from .visual_goal import GoalGeometry, GoalObservation, GoalVisibility


@dataclass(frozen=True)
class SurfaceCandidate:
    candidate_id: str
    point: tuple[float, float]
    description: str
    matches_reference_region: bool = False


@dataclass(frozen=True)
class GoalSurfaceObservation:
    status: GoalVisibility
    bbox: tuple[float, float, float, float] | None
    surface_candidates: tuple[SurfaceCandidate, ...]
    description: str
    evidence: str
    same_target: bool = True  # Object identity only, not surface correspondence.

    def at(self, candidate: SurfaceCandidate) -> GoalObservation:
        """Explicit projection into the unchanged single-region planner context."""
        return GoalObservation(self.status, self.bbox, candidate.point, self.description,
                               self.evidence, self.same_target)


@dataclass(frozen=True)
class SurfaceEstimate:
    candidate: SurfaceCandidate
    geometry: GoalGeometry
    eligible: bool
    reason: str


@dataclass(frozen=True)
class SurfaceSelection:
    estimates: tuple[SurfaceEstimate, ...]
    selected: SurfaceEstimate | None
    goal: GoalObservation | None
    geometry: GoalGeometry
    reason: str

    @property
    def reliable_world_points(self) -> tuple[tuple[float, float, float], ...]:
        # Ineligible approach regions can still be reliable nearby hazard evidence.
        return tuple(item.geometry.world_point for item in self.estimates
                     if item.geometry.reliable and item.geometry.world_point is not None)
