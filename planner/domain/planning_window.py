"""Frozen task-independent inputs for one owner-submitted continuation request."""
from dataclasses import dataclass
import math

from .pose import WorldPose
from .trajectory import RelativeTrajectory


@dataclass(frozen=True)
class PlanningWindowInput:
    initial_pose: WorldPose
    maximum_observation_age_s: float
    model_request_timeout_s: float
    previous_trajectory: RelativeTrajectory | None = None
    previous_trajectory_start: WorldPose | None = None

    def __post_init__(self):
        if not isinstance(self.initial_pose, WorldPose):
            raise ValueError("window input requires a measured initial world pose")
        for value in (self.maximum_observation_age_s, self.model_request_timeout_s):
            try:
                valid = not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value) and value > 0
            except OverflowError:
                valid = False
            if not valid:
                raise ValueError("window request limits must be finite and positive")
