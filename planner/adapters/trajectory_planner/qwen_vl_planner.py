"""OpenAI-compatible VLM trajectory planner (RGB + aligned depth visualization)."""

from __future__ import annotations

import logging
import math
import time
from typing import Generic, TypeVar

from planner.domain.execution import ExecutionBudget
from planner.domain.observation import Observation
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.errors import BudgetExceeded, ProtocolError

from ..model_visual_inputs import aligned_depth_image
from .prompts import BaseTrajectoryPrompt, TrajectoryPromptBuilder
from .response_parser import parse_trajectory

logger = logging.getLogger(__name__)


ContextT = TypeVar("ContextT")


class QwenVLPlanner(Generic[ContextT]):
    def __init__(self, transport, *, expected_points: int = 5, timeout_s: float = 30.0,
                 prompt_builder: TrajectoryPromptBuilder[ContextT] | None = None):
        if expected_points != 5:
            raise ValueError("trajectory model protocol requires exactly 5 new points")
        self.transport = transport
        self.expected_points = expected_points
        self.timeout_s = float(timeout_s)
        self.prompt_builder = prompt_builder or BaseTrajectoryPrompt()

    def plan(
        self,
        observation: Observation,
        instruction: str,
        *,
        current_guidance: str | None = None,
        guidance_start: WorldPose | None = None,
        round_index: int = 1,
        initial_pose: WorldPose | None = None,
        previous_trajectory: RelativeTrajectory | None = None,
        previous_trajectory_start: WorldPose | None = None,
        pending_trajectory: RelativeTrajectory | None = None,
        task_context: ContextT | None = None,
        budget: ExecutionBudget | None = None,
        timeout_s: float | None = None,
    ) -> RelativeTrajectory:
        if pending_trajectory is not None:
            if not isinstance(pending_trajectory, RelativeTrajectory):
                raise ProtocolError("pending_trajectory must be a RelativeTrajectory or None")
            if len(pending_trajectory.points) > 5 or any(
                abs(point.dyaw_deg) > 180 for point in pending_trajectory.points
            ):
                raise ProtocolError("pending requires 0..5 points with yaw in [-180, 180]")
        request_timeout = self.timeout_s if timeout_s is None else min(self.timeout_s, timeout_s)
        if not math.isfinite(request_timeout) or request_timeout <= 0:
            raise ProtocolError("planning timeout must be finite and positive")
        logger.info("[TrajectoryPlanner] instruction=%s", instruction)
        logger.info(
            "[TrajectoryPlanner] RGB=%s depth=%s pose=%s",
            observation.rgb.shape,
            None if observation.depth_image is None else observation.depth_image.shape,
            observation.pose,
        )
        system_prompt = self.prompt_builder.system_prompt()
        extra = {} if task_context is None else {"task_context": task_context}
        user_prompt = self.prompt_builder.build_user_prompt(
            instruction,
            observation,
            self.expected_points,
            current_guidance=current_guidance,
            **({"guidance_start": guidance_start} if guidance_start is not None else {}),
            round_index=round_index,
            initial_pose=initial_pose or observation.pose,
            previous_trajectory=previous_trajectory,
            **({"previous_trajectory_start": previous_trajectory_start}
               if previous_trajectory_start is not None else {}),
            pending_trajectory=pending_trajectory,
            **extra,
        )
        model_depth = aligned_depth_image(observation)
        images = ((observation.rgb, "JPEG"), (model_depth, "PNG"))
        started = time.perf_counter()
        timeout = (request_timeout if budget is None
                   else min(request_timeout, budget.remaining(time.monotonic())))
        if timeout <= 0:
            raise BudgetExceeded("deadline before trajectory request")
        raw = self.transport.request(
            system_prompt,
            user_prompt,
            images,
            timeout_s=timeout,
            operation="trajectory planning",
        )
        if budget is not None and budget.remaining(time.monotonic()) <= 0:
            raise BudgetExceeded("late trajectory response discarded")
        logger.info(
            "[TrajectoryPlanner] %.2fs raw response:\n%s",
            time.perf_counter() - started,
            raw,
        )
        trajectory = parse_trajectory(raw, self.expected_points)
        if len(trajectory.points) != 5:
            raise ProtocolError(
                f"trajectory model must return exactly 5 new points; received {len(trajectory.points)}"
            )
        return trajectory