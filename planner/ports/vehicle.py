"""Vehicle port, mirroring the reference stopped-segment and streaming interface."""

from __future__ import annotations

from typing import Protocol

from ..domain.execution import ExecutionBudget, PoseCheckpoint
from ..domain.pose import WorldPose
from ..domain.trajectory import MotionSegment


class Vehicle(Protocol):
    def current_pose(self) -> WorldPose: ...

    def execute_segment(
        self,
        segment: MotionSegment,
        *,
        budget: ExecutionBudget | None = None,
        pose_checkpoint: PoseCheckpoint | None = None,
    ) -> None: ...

    def cancel(self) -> None: ...

    def hover(self) -> None: ...

    def collision_state(self) -> bool: ...