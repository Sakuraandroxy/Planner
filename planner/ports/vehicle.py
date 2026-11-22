"""Vehicle port, mirroring the reference stopped-segment and streaming interface."""

from __future__ import annotations

from typing import Protocol

from planner.domain import CollisionState, ExecutionBudget, MotionCapability, PoseCheckpoint
from planner.domain import WorldPose
from planner.domain import MotionSegment


class Vehicle(Protocol):
    def current_pose(self) -> WorldPose: ...

    def motion_capability(self) -> MotionCapability: ...

    def execute_segment(
        self,
        segment: MotionSegment,
        *,
        budget: ExecutionBudget | None = None,
        pose_checkpoint: PoseCheckpoint | None = None,
    ) -> None: ...

    def cancel(self) -> None: ...

    def hover(self) -> None: ...

    def collision_state(self) -> CollisionState: ...
