"""Scenario set behind ``golden.json``.

Every entry renders one prompt through the builders that ship today, with fixed
inputs, and ``freeze_golden.py`` writes the results to ``golden.json``. A later
template rewrite is compared against that file, so the pinned text is the
wording that shipped before the templates existed.
"""

from __future__ import annotations

import numpy as np

from planner.adapters.task_completion.prompts import VisualGoalCompletionPrompt
from planner.adapters.target_detection.prompts import (
    TARGET_DETECTION_SYSTEM_PROMPT,
    VisualGoalTargetDetectionPrompt,
)
from planner.adapters.task_parser import prompt as task_parser_prompt
from planner.adapters.trajectory_planner.prompts import (
    AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT,
    AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT,
    AERIAL_VISUAL_GOAL_SYSTEM_PROMPT,
    BASE_TRAJECTORY_SYSTEM_PROMPT,
    FREE_NAVIGATION_SYSTEM_PROMPT,
    VISUAL_GOAL_SYSTEM_PROMPT,
    FreeNavigationTrajectoryPrompt,
    VisualGoalTrajectoryPrompt,
    build_aerial_base_user_prompt,
    build_base_user_prompt,
)
from planner.domain import (
    CameraIntrinsics,
    GoalGeometry,
    GoalObservation,
    GoalVisibility,
    Observation,
    RelativePoseDelta,
    RelativeTrajectory,
    TargetDetectionInput,
    TargetObservationEvidence,
    TargetPositionState,
    TaskKind,
    VisualGoalCompletionInput,
    VisualGoalNavigationParameters,
    VisualGoalPlanningContext,
    VisualGoalState,
    VisualGoalTriggerRules,
    WorldPose,
)
from tests.support import ue_camera_matrix

INSTRUCTION = "drive to the junction and turn right"
OBSERVATION_POSE = WorldPose(1.5, -2.25, 0.0, 15.0)
MISSION_START = WorldPose(0.25, -1.0, 0.0, 5.0)
GUIDANCE_START = WorldPose(0.5, -1.5, 0.0, 10.0)


def _observation(
    pose: WorldPose = OBSERVATION_POSE,
    *,
    depth: np.ndarray | None = None,
    capture_id: str = "capture-7",
    captured_monotonic: float = 12.5,
) -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(64, 48, 90.0)
    if depth is None:
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


def _trajectory(*points) -> RelativeTrajectory:
    return RelativeTrajectory(tuple(RelativePoseDelta(*point) for point in points))


def _execution_context() -> dict:
    return {
        "current_guidance": "stay left of the cones",
        "guidance_start": GUIDANCE_START,
        "round_index": 2,
        "initial_pose": MISSION_START,
        "previous_trajectory": _trajectory((1.0, 0.0, 0.0, 0.0), (2.5, -0.5, 0.0, 7.5)),
        "previous_trajectory_start": MISSION_START,
        "pending_trajectory": _trajectory((1.0, 0.2, 0.0, 3.0)),
    }


def _visual_context(
    *, world_point=(8.0, 2.0, 0.0), keep_target_in_view: bool = True
) -> VisualGoalPlanningContext:
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
        world_point,
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
        parameters=VisualGoalNavigationParameters("drive to the red car", "the red car"),
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


def build_cases() -> dict[str, str]:
    """Every pinned scenario, keyed by the id stored in ``golden.json``."""
    observation = _observation()
    no_depth = _observation(depth=np.full((48, 64), np.nan, dtype=np.float32))
    context = _execution_context()
    completion = VisualGoalCompletionPrompt
    detection = VisualGoalTargetDetectionPrompt()
    detection_input = TargetDetectionInput(
        _completion_observation(), INSTRUCTION, "the blue truck"
    )
    return {
        # task parser
        "task_parser_system_ground": task_parser_prompt.task_parser_system_prompt(
            {TaskKind.BASIC_MOTION, TaskKind.VISUAL_GOAL_NAVIGATION}
        ),
        "task_parser_system_aerial": task_parser_prompt.task_parser_system_prompt(
            {TaskKind.BASIC_MOTION, TaskKind.VISUAL_GOAL_NAVIGATION}, aerial=True
        ),
        "task_parser_user": task_parser_prompt.user_prompt(INSTRUCTION),
        # target detection
        "target_detection_system": TARGET_DETECTION_SYSTEM_PROMPT,
        "target_detection_user": detection.build(detection_input),
        "target_detection_user_with_reference": detection.build(
            TargetDetectionInput(
                _completion_observation(),
                INSTRUCTION,
                "the blue truck",
                reference_rgb=np.zeros((4, 4, 3), dtype=np.uint8),
            )
        ),
        # completion
        "completion_system": completion.system_prompt(),
        "completion_user": completion.build_user_prompt(_completion_request()),
        "completion_user_with_reference": completion.build_user_prompt(
            _completion_request(
                reference_rgb=np.zeros((4, 4, 3), dtype=np.uint8),
                height_tolerance_m=0.5,
                parameters=VisualGoalNavigationParameters(
                    "drive to the red car", "the red car", keep_target_in_view=False
                ),
            )
        ),
        "completion_user_with_target_state": completion.build_user_prompt(
            _completion_request(
                target_state=_target_state(), target_observation=_target_evidence()
            )
        ),
        "completion_user_with_rejected_update": completion.build_user_prompt(
            _completion_request(
                target_state=_target_state(),
                target_observation=_target_evidence("jump exceeds the limit"),
            )
        ),
        "completion_user_with_stale_evidence": completion.build_user_prompt(
            _completion_request(
                target_observation=TargetObservationEvidence(
                    "stage-2", "capture-1", 0.75, 8, _completion_goal(), None
                )
            )
        ),
        # base trajectory
        "base_system_ground": BASE_TRAJECTORY_SYSTEM_PROMPT,
        "base_system_aerial": AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT,
        "base_user_initial": build_base_user_prompt(INSTRUCTION, observation, 5),
        "base_user_continuing": build_base_user_prompt(
            INSTRUCTION, observation, 5, **context
        ),
        "base_user_continuing_aerial": build_aerial_base_user_prompt(
            INSTRUCTION, observation, 5, **context
        ),
        # free navigation
        "free_navigation_system_ground": FREE_NAVIGATION_SYSTEM_PROMPT,
        "free_navigation_system_aerial": AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT,
        "free_navigation_user_initial": FreeNavigationTrajectoryPrompt().build_user_prompt(
            "follow the road", observation, 5
        ),
        "free_navigation_user_continuing": (
            FreeNavigationTrajectoryPrompt().build_user_prompt(
                "follow the road", observation, 5, **context
            )
        ),
        "free_navigation_user_no_ground": (
            FreeNavigationTrajectoryPrompt().build_user_prompt(
                "follow the road", no_depth, 5
            )
        ),
        "free_navigation_user_aerial": (
            FreeNavigationTrajectoryPrompt(aerial=True).build_user_prompt(
                "fly along the river", observation, 5, **context
            )
        ),
        # visual goal
        "visual_goal_system_ground": VISUAL_GOAL_SYSTEM_PROMPT,
        "visual_goal_system_aerial": AERIAL_VISUAL_GOAL_SYSTEM_PROMPT,
        "visual_goal_user_ground_pending": VisualGoalTrajectoryPrompt().build_user_prompt(
            INSTRUCTION, observation, 5, task_context=_visual_context(), **context
        ),
        "visual_goal_user_ground_no_pending": (
            VisualGoalTrajectoryPrompt().build_user_prompt(
                INSTRUCTION, observation, 5, task_context=_visual_context()
            )
        ),
        "visual_goal_user_ground_target_left": (
            VisualGoalTrajectoryPrompt().build_user_prompt(
                INSTRUCTION,
                observation,
                5,
                task_context=_visual_context(world_point=(8.0, -6.0, 0.0)),
            )
        ),
        "visual_goal_user_ground_target_at_pose": (
            VisualGoalTrajectoryPrompt().build_user_prompt(
                INSTRUCTION,
                observation,
                5,
                task_context=_visual_context(world_point=(1.5, -2.25, 0.0)),
            )
        ),
        "visual_goal_user_ground_no_view_guidance": (
            VisualGoalTrajectoryPrompt().build_user_prompt(
                INSTRUCTION,
                observation,
                5,
                task_context=_visual_context(keep_target_in_view=False),
            )
        ),
        "visual_goal_user_aerial": VisualGoalTrajectoryPrompt(
            aerial=True
        ).build_user_prompt(
            INSTRUCTION, observation, 5, task_context=_visual_context(), **context
        ),
    }


CASES = build_cases()
