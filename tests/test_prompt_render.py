"""Pinned pre-refactor output of every adapter prompt text.

``prompt_render_baseline.json`` holds the strings that the adapter prompt
modules produced at git HEAD before the text moved into ``prompts/templates``.
Each case renders with the fixed parameters below, so the loaded templates
must reproduce those strings byte for byte. The combined hash case catches a
rewrite that changes two cases in compensating ways.
"""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from planner.adapters.target_detection.prompts import (
    TARGET_DETECTION_SYSTEM_PROMPT,
    VisualGoalTargetDetectionPrompt,
)
from planner.adapters.task_completion.prompts import VisualGoalCompletionPrompt

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
from planner.domain import (
    TargetDetectionInput,
    TargetObservationEvidence,
    TargetPositionState,
    VisualGoalCompletionInput,
    VisualGoalNavigationParameters,
    VisualGoalTriggerRules,
)
from tests.support import ue_camera_matrix

BASELINE_PATH = Path(__file__).with_name("prompt_render_baseline.json")

#: SHA-256 of ``json.dumps(CASES, sort_keys=True, ensure_ascii=False)``.
CASES_SHA256 = "da5ae4d1203cfaa68628fb165f6608c9cdcd0d07363337a4fad9cf2cb4120dc7"

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


def _completion_observation() -> Observation:
    width, height = 8, 6
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        rgb=np.zeros((height, width, 3), dtype=np.uint8),
        depth_m=np.full((height, width), 5.0, dtype=np.float32),
        depth_image=np.zeros((height, width), dtype=np.uint8),
        intrinsics=CameraIntrinsics.from_horizontal_fov(width, height, 90.0),
        camera_matrix=ue_camera_matrix(0.0, 0.0, 0.0, 0.0),
        timestamp_ns=10,
        captured_monotonic=1.0,
        capture_id="capture-1",
        stopped=True,
    )


def _completion_goal() -> GoalObservation:
    return GoalObservation(
        GoalVisibility.VISIBLE,
        (0.4, 0.4, 0.6, 0.6),
        None,
        "the red car",
        "visible",
        True,
        "model_surface_points",
        surface_points=((0.5, 0.5),),
    )


def _completion_request(**overrides) -> VisualGoalCompletionInput:
    values = dict(
        stage_id="stage-1",
        parameters=VisualGoalNavigationParameters(
            "drive to the red car", "the red car"
        ),
        observation=_completion_observation(),
        initial_pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        goal=_completion_goal(),
        geometry=GoalGeometry(True, "sampled", 5.0, (5.0, 0.0, 0.0)),
        rules=VisualGoalTriggerRules(radius_m=7.0, pause_radius_m=8.0),
        uncertainty_margin_m=0.5,
        height_tolerance_m=None,
        confirmation_count=2,
        required_confirmation_count=2,
        reference_description="the red car",
    )
    values.update(overrides)
    return VisualGoalCompletionInput(**values)


def _target_state() -> TargetPositionState:
    return TargetPositionState(
        "stage-1",
        (5.0, 0.0, 0.0),
        (5.0, 0.0, 0.0),
        "capture-0",
        0.5,
        5,
        anchor_surface_points_world=((5.0, 0.0, 0.0),),
        surface_points_world=((5.0, 0.0, 0.0),),
    )


def _target_evidence(rejection: str | None = None) -> TargetObservationEvidence:
    return TargetObservationEvidence(
        "stage-1", "capture-1", 0.75, 8, _completion_goal(), rejection
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
        "target_detection_system": TARGET_DETECTION_SYSTEM_PROMPT,
        "target_detection_user_prompt": (VisualGoalTargetDetectionPrompt().build(
            TargetDetectionInput(_completion_observation(), instruction, "the blue truck")
        )),
        "target_detection_user_prompt_with_reference": (
            VisualGoalTargetDetectionPrompt().build(
                TargetDetectionInput(
                    _completion_observation(),
                    instruction,
                    "the blue truck",
                    reference_rgb=np.zeros((4, 4, 3), dtype=np.uint8),
                )
            )
        ),
        "completion_system": VisualGoalCompletionPrompt.system_prompt(),
        "completion_user_prompt": VisualGoalCompletionPrompt.build_user_prompt(
            _completion_request()
        ),
        "completion_user_prompt_with_reference": (
            VisualGoalCompletionPrompt.build_user_prompt(
                _completion_request(
                    reference_rgb=np.zeros((4, 4, 3), dtype=np.uint8),
                    height_tolerance_m=0.5,
                    parameters=VisualGoalNavigationParameters(
                        "drive to the red car", "the red car", keep_target_in_view=False
                    ),
                )
            )
        ),
        "completion_user_prompt_with_target_state": (
            VisualGoalCompletionPrompt.build_user_prompt(
                _completion_request(
                    target_state=_target_state(), target_observation=_target_evidence()
                )
            )
        ),
        "completion_user_prompt_with_rejected_update": (
            VisualGoalCompletionPrompt.build_user_prompt(
                _completion_request(
                    target_state=_target_state(),
                    target_observation=_target_evidence("jump exceeds the limit"),
                )
            )
        ),
        "completion_user_prompt_with_stale_evidence": (
            VisualGoalCompletionPrompt.build_user_prompt(
                _completion_request(
                    target_observation=TargetObservationEvidence(
                        "stage-2", "capture-1", 0.75, 8, _completion_goal(), None
                    )
                )
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


def test_rendered_prompt_hash_is_pinned():
    payload = json.dumps(CASES, sort_keys=True, ensure_ascii=False)
    assert hashlib.sha256(payload.encode("utf-8")).hexdigest() == CASES_SHA256
