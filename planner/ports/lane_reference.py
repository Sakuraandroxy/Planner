"""Measured lane-frame prior supplied to road-following planning."""
from typing import Protocol

from planner.domain.lane_reference import LaneReference
from planner.domain.pose import WorldPose


class LaneReferenceProvider(Protocol):
    def lane_reference(self, pose: WorldPose) -> LaneReference | None: ...