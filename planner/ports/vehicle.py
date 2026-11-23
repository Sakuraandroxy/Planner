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

    def motion_capability(self) -> MotionCapability: ...

    def execute_segment(
        self,
        segment: MotionSegment,
        *,
        budget: ExecutionBudget | None = None,
        pose_checkpoint: PoseCheckpoint | None = None,
    ) -> None:
        """Execute one segment before returning.

        The run path does not call this: ``ControlLoop.execute`` drives segments
        through ``stream_command``. ``GuidedVehicle`` is the implementation this
        port asks for.
        """

    def cancel(self) -> None: ...

    def hold(self) -> None:
        """Hold the vehicle still for one control period: a drone hovers, a car brakes."""

    def collision_state(self) -> CollisionState: ...
