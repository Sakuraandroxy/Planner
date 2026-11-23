"""Vehicle port, mirroring the reference stopped-segment and streaming interface."""

from __future__ import annotations

from typing import Protocol

from planner.domain import (
    CollisionState,
    ExecutionBudget,
    MotionCapability,
    MotionLimits,
    PoseCheckpoint,
)
from planner.domain import WorldPose
from planner.domain import MotionSegment


class Vehicle(Protocol):
    def current_pose(self) -> WorldPose: ...

    @property
    def tracking_limits(self) -> MotionLimits:
        """The envelope the planner tracks against, in domain units."""

    def attach_loop(self, loop) -> None:
        """Take the control loop the assembly bound to this vehicle."""

    def motion_capability(self) -> MotionCapability: ...

    def execute_segment(
        self,
        segment: MotionSegment,
        *,
        budget: ExecutionBudget | None = None,
        pose_checkpoint: PoseCheckpoint | None = None,
    ) -> None: ...

    def cancel(self) -> None: ...

    def hold(self) -> None:
        """Hold the vehicle still for one control period: a drone hovers, a car brakes."""

    def collision_state(self) -> CollisionState: ...
