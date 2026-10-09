"""Compatibility wrapper; new workflows inject a task-specific prompt builder."""
from planner.adapters.trajectory_planner.prompts import BaseTrajectoryPrompt
from planner.domain.observation import Observation


def build_prompt(instruction: str, observation: Observation, max_points: int = 5) -> str:
    builder = BaseTrajectoryPrompt()
    return "\n\n".join([
        builder.system_prompt(),
        builder.build_user_prompt(instruction, observation, max_points),
    ])

