"""Deterministic geometric approach planner.

This is the fallback used when no trajectory model endpoint is configured
(and a useful reference implementation for tests): it turns the estimated
target surface into five short body-frame waypoints that close the horizontal
gap, correct heading, and apply a bounded vertical adjustment. It never aims
inside the arrival sphere-the workflow checks arrival before planning.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from ...domain.pose import RelativePoseDelta
from ...domain.trajectory import RelativeTrajectory
from ...domain.visual_goal import VisualGoalPlanningContext
from ...errors import ProtocolError
from ...services.target_coordinates import point_in_horizontal_body
from ...services.trajectory_transform import relative_to_world

WAYPOINT_COUNT = 5


@dataclass(frozen=True)
class GeometricPlannerSettings:
    max_advance_m: float = 6.0
    max_vertical_m: float = 2.0
    max_yaw_step_deg: float = 20.0
    # Leave this fraction of the arrival radius as a stopping buffer.
    arrival_buffer_ratio: float = 0.8

    def __post_init__(self) -> None:
        for name in ("max_advance_m", "max_vertical_m", "max_yaw_step_deg"):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be positive")
        if not 0.0 <= self.arrival_buffer_ratio < 1.0:
            raise ValueError("arrival_buffer_ratio must be in [0, 1)")


class GeometricTargetPlanner:
    def __init__(self, settings: GeometricPlannerSettings | None = None):
        self.settings = settings or GeometricPlannerSettings()

    def plan(
        self,
        observation,
        instruction: str,
        *,
        current_guidance: str | None = None,
        guidance_start=None,
        round_index: int = 1,
        initial_pose=None,
        previous_trajectory: RelativeTrajectory | None = None,
        previous_trajectory_start=None,
        pending_trajectory: RelativeTrajectory | None = None,
        task_context: VisualGoalPlanningContext | None = None,
        budget=None,
        timeout_s: float | None = None,
    ) -> RelativeTrajectory:
        if not isinstance(task_context, VisualGoalPlanningContext):
            raise ProtocolError("geometric planning requires typed target context")
        if not task_context.geometry.reliable or task_context.geometry.world_point is None:
            raise ProtocolError("geometric planning requires an estimated target position")
        pose = observation.pose
        pending = pending_trajectory
        if pending is not None and pending.points:
            # NEW actions are anchored at the unexecuted pending predicted tail,
            # matching the reference model contract.
            pose = relative_to_world(pending, pose).poses[-1]
        forward, right, down = point_in_horizontal_body(
            task_context.geometry.world_point, pose
        )
        horizontal = math.hypot(forward, right)
        stop_distance = self.settings.arrival_buffer_ratio * task_context.arrival_radius_m
        advance_total = min(
            self.settings.max_advance_m,
            max(0.0, horizontal - stop_distance),
        )
        count = WAYPOINT_COUNT
        advance = advance_total / count
        vertical = max(
            -self.settings.max_vertical_m,
            min(self.settings.max_vertical_m, down),
        )
        dz = vertical / count
        bearing = (
            math.degrees(math.atan2(right, forward))
            if horizontal > 1e-6
            else 0.0
        )
        points: list[RelativePoseDelta] = []
        turned = 0.0
        for _ in range(count):
            remaining = bearing - turned
            step_yaw = max(
                -self.settings.max_yaw_step_deg,
                min(self.settings.max_yaw_step_deg, remaining),
            )
            turned += step_yaw
            angle = math.radians(remaining)
            points.append(
                RelativePoseDelta(
                    dx=advance * math.cos(angle),
                    dy=advance * math.sin(angle),
                    dz=dz,
                    dyaw_deg=step_yaw,
                )
            )
        return RelativeTrajectory(tuple(points))