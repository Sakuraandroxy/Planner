"""Pinned pre-refactor output of every trajectory prompt text.

``prompt_render_baseline.json`` holds the strings that the modules under
``planner/adapters/trajectory_planner/prompts`` produced at git HEAD before the
text moved into ``prompts/templates``. Each case renders with the fixed
parameters below, so the loaded templates must reproduce those strings
byte for byte.
"""

import json
from pathlib import Path

import numpy as np
import pytest

from planner.adapters.trajectory_planner.prompts import (
    AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT,
    AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT,
    AERIAL_VISUAL_GOAL_OUTPUT_GUIDANCE,
    AERIAL_VISUAL_GOAL_SYSTEM_PROMPT,
    BASE_TRAJECTORY_SYSTEM_PROMPT,
    FREE_NAVIGATION_SYSTEM_PROMPT,
    VISUAL_GOAL_OUTPUT_GUIDANCE,
    VISUAL_GOAL_SYSTEM_PROMPT,
    VISUAL_GOAL_VIEW_GUIDANCE,
    FreeNavigationTrajectoryPrompt,
    VisualGoalTrajectoryPrompt,
    build_aerial_base_user_prompt,
    build_base_user_prompt,
)
from planner.domain import CameraIntrinsics, Observation
from planner.domain import RelativePoseDelta, WorldPose
from planner.domain import RelativeTrajectory
from planner.domain import (
    GoalGeometry,
    GoalObservation,
    GoalVisibility,
    VisualGoalPlanningContext,
    VisualGoalState,
)

BASELINE_PATH = Path(__file__).with_name("prompt_render_baseline.json")

OBSERVATION_POSE = WorldPose(1.5, -2.25, 0.0, 15.0)
MISSION_START = WorldPose(0.25, -1.0, 0.0, 5.0)
GUIDANCE_START = WorldPose(0.5, -1.5, 0.0, 10.0)


def _observation(
    pose: WorldPose = OBSERVATION_POSE,
    *,
    capture_id: str = "capture-7",
    captured_monotonic: float = 12.5,
) -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(64, 48, 90.0)
    depth = np.tile(np.linspace(4.0, 24.0, 64, dtype=np.float32), (48, 1))
    return Observation(
        frame=7,
        timestamp_s=12.0,
        pose=pose,
        rgb=np.zeros((48, 64, 3), dtype=np.uint8),
        depth_m=depth,
        intrinsics=intrinsics,
        camera_matrix=np.eye(4),
        timestamp_ns=12_000_000_000,
        captured_monotonic=captured_monotonic,
        capture_id=capture_id,
        stopped=True,
        camera_id="front",
    )


def _trajectory(*points) -> RelativeTrajectory:
    return RelativeTrajectory(tuple(RelativePoseDelta(*point) for point in points))


def _previous_trajectory() -> RelativeTrajectory:
    return _trajectory((1.0, 0.0, 0.0, 0.0), (2.5, -0.5, 0.0, 7.5))


def _pending_trajectory() -> RelativeTrajectory:
    return _trajectory((1.0, 0.2, 0.0, 3.0))


def _visual_context(*, keep_target_in_view: bool = True) -> VisualGoalPlanningContext:
    goal = GoalObservation(
        GoalVisibility.VISIBLE,
        (0.4, 0.4, 0.6, 0.6),
        None,
        "the blue truck",
        "visible",
        True,
        "model_surface_points",
    )
    geometry = GoalGeometry(
        True,
        "sampled",
        14.5,
        (8.0, 2.0, 0.0),
        surface_points_world=((8.0, 2.0, 0.0), (9.0, 2.4, 0.1)),
    )
    return VisualGoalPlanningContext(
        "the blue truck",
        goal,
        geometry,
        VisualGoalState.APPROACHING,
        6.0,
        0.75,
        1.5,
        target_capture_id="capture-7",
        target_captured_monotonic=11.0,
        keep_target_in_view=keep_target_in_view,
    )


def _cases() -> dict[str, str]:
    observation = _observation()
    previous = _previous_trajectory()
    pending = _pending_trajectory()
    execution_context = {
        "current_guidance": "stay left of the cones",
        "guidance_start": GUIDANCE_START,
        "round_index": 2,
        "initial_pose": MISSION_START,
        "previous_trajectory": previous,
        "previous_trajectory_start": MISSION_START,
        "pending_trajectory": pending,
    }
    instruction = "drive to the junction and turn right"
    return {
        "base_system_ground": BASE_TRAJECTORY_SYSTEM_PROMPT,
        "base_system_aerial": AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT,
        "free_navigation_system_ground": FREE_NAVIGATION_SYSTEM_PROMPT,
        "free_navigation_system_aerial": AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT,
        "visual_goal_system_ground": VISUAL_GOAL_SYSTEM_PROMPT,
        "visual_goal_system_aerial": AERIAL_VISUAL_GOAL_SYSTEM_PROMPT,
        "visual_goal_view_guidance": VISUAL_GOAL_VIEW_GUIDANCE,
        "visual_goal_output_guidance": VISUAL_GOAL_OUTPUT_GUIDANCE,
        "aerial_visual_goal_output_guidance": AERIAL_VISUAL_GOAL_OUTPUT_GUIDANCE,
        "base_user_prompt_initial": build_base_user_prompt(instruction, observation, 5),
        "base_user_prompt_continuing": build_base_user_prompt(
            instruction, observation, 5, **execution_context
        ),
        "aerial_base_user_prompt_continuing": build_aerial_base_user_prompt(
            instruction, observation, 5, **execution_context
        ),
        "free_navigation_user_prompt_without_lane": (
            FreeNavigationTrajectoryPrompt().build_user_prompt(
                "follow the road", observation, 5
            )
        ),
        "visual_goal_user_prompt_ground": VisualGoalTrajectoryPrompt().build_user_prompt(
            instruction, observation, 5,
            task_context=_visual_context(), **execution_context,
        ),
        "visual_goal_user_prompt_aerial": VisualGoalTrajectoryPrompt(
            aerial=True
        ).build_user_prompt(
            instruction, observation, 5,
            task_context=_visual_context(), **execution_context,
        ),
        "visual_goal_user_prompt_without_view_guidance": (
            VisualGoalTrajectoryPrompt().build_user_prompt(
                instruction, observation, 5,
                task_context=_visual_context(keep_target_in_view=False),
            )
        ),
    }


CASES = _cases()


@pytest.fixture(scope="module")
def baseline() -> dict[str, str]:
    return json.loads(BASELINE_PATH.read_text(encoding="utf-8"))


def _first_difference(expected: str, actual: str) -> str:
    for index, (left, right) in enumerate(zip(expected, actual)):
        if left != right:
            return (
                f"first difference at character {index}: "
                f"expected {expected[index:index + 40]!r}, "
                f"got {actual[index:index + 40]!r}"
            )
    return f"lengths differ: expected {len(expected)}, got {len(actual)}"


@pytest.mark.parametrize("name", sorted(CASES))
def test_prompt_text_matches_the_pre_refactor_output(name, baseline):
    expected = baseline[name]
    assert CASES[name] == expected, _first_difference(expected, CASES[name])


def test_baseline_covers_exactly_the_rendered_cases(baseline):
    assert sorted(baseline) == sorted(CASES)
