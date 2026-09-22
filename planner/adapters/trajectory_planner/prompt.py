"""Compatibility wrapper; new workflows inject a task-specific prompt builder."""
from planner.adapters.trajectory_planner.prompts import NavigationTrajectoryPrompt
from planner.domain.observation import Observation


def build_prompt(instruction: str, observation: Observation, max_points: int = 5) -> str:
    return NavigationTrajectoryPrompt().build(instruction, observation, max_points)

