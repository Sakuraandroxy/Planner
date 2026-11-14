import numpy as np
import pytest

from planner.adapters.trajectory_planner import QwenVLPlanner
from planner.adapters.trajectory_planner.prompts import (
    AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT,
    AERIAL_VISUAL_GOAL_SYSTEM_PROMPT,
    BASE_TRAJECTORY_SYSTEM_PROMPT,
    VISUAL_GOAL_SYSTEM_PROMPT,
    BaseTrajectoryPrompt,
    VisualGoalTrajectoryPrompt,
)
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.visual_goal import (
    GoalGeometry,
    GoalObservation,
    GoalVisibility,
    VisualGoalPlanningContext,
    VisualGoalState,
)
from planner.errors import ProtocolError


class _NoTransport:
    def request(self, *args, **kwargs):
        raise AssertionError("the model must not be called")


class _CapturingTransport:
    def __init__(self, reply: str):
        self.reply = reply
        self.calls = []

    def request(self, system, text, images, *, timeout_s, operation):
        self.calls.append((system, text, images, timeout_s, operation))
        return self.reply


def _observation(pose: WorldPose) -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(640, 480, 90.0)
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=pose,
        rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        depth_m=np.ones((480, 640), dtype=np.float32),
        depth_image=np.zeros((480, 640, 3), dtype=np.uint8),
        intrinsics=intrinsics,
        camera_matrix=np.eye(4),
        camera_id="front",
        capture_id="capture-1",
        captured_monotonic=1.0,
    )


def _context(
    target_description: str = "the red car",
    world_point: tuple[float, float, float] = (10.0, 0.0, 0.0),
) -> VisualGoalPlanningContext:
    goal = GoalObservation(
        GoalVisibility.VISIBLE, (0.4, 0.4, 0.6, 0.6), None,
        target_description, "visible", True, "model_surface_points",
    )
    geometry = GoalGeometry(
        True, "sampled", 10.0, world_point,
        surface_points_world=(world_point,),
    )
    return VisualGoalPlanningContext(
        target_description, goal, geometry, VisualGoalState.APPROACHING,
        7.0, 0.5, None,
        target_capture_id="capture-1", target_captured_monotonic=1.0,
    )


def _visual_prompt(
    *,
    pose: WorldPose = WorldPose(0.0, 0.0, 0.0, 0.0),
    context: VisualGoalPlanningContext | None = None,
    round_index: int = 1,
    **kwargs,
) -> str:
    return VisualGoalTrajectoryPrompt().build_user_prompt(
        "drive to the target",
        _observation(pose),
        5,
        round_index=round_index,
        task_context=context or _context(),
        **kwargs,
    )


def test_system_prompt_targets_a_ground_vehicle():
    assert "ground vehicle" in BASE_TRAJECTORY_SYSTEM_PROMPT
    assert "dz = 0" in BASE_TRAJECTORY_SYSTEM_PROMPT
    assert "UAV" not in BASE_TRAJECTORY_SYSTEM_PROMPT
    assert VISUAL_GOAL_SYSTEM_PROMPT.startswith(BASE_TRAJECTORY_SYSTEM_PROMPT)
    assert "The objective is to enter arrival_radius_m" in VISUAL_GOAL_SYSTEM_PROMPT
    assert "vertical motion is unavailable" in VISUAL_GOAL_SYSTEM_PROMPT


def test_aerial_system_prompt_targets_a_uav():
    assert "autonomous UAV" in AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT
    assert "ground vehicle" not in AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT
    assert "+dz is DOWN" in AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT
    assert AERIAL_VISUAL_GOAL_SYSTEM_PROMPT.startswith(
        AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT
    )
    assert (
        "The objective is to enter arrival_radius_m"
        in AERIAL_VISUAL_GOAL_SYSTEM_PROMPT
    )
    assert "Upward and downward motion are allowed" in AERIAL_VISUAL_GOAL_SYSTEM_PROMPT
    assert (
        "vertical motion is unavailable"
        not in AERIAL_VISUAL_GOAL_SYSTEM_PROMPT
    )
    assert (
        VisualGoalTrajectoryPrompt(aerial=True).system_prompt()
        == AERIAL_VISUAL_GOAL_SYSTEM_PROMPT
    )
    assert VisualGoalTrajectoryPrompt().system_prompt() == VISUAL_GOAL_SYSTEM_PROMPT


def test_aerial_messages_use_the_reference_uav_wording():
    # The drone arm must read exactly like /roxy/zhusai/Planner; the rover keeps
    # the carla-vln ground-vehicle wording. Only the pose attribute differs.
    ground = _visual_prompt(
        current_guidance="turn right",
        guidance_start=WorldPose(0.0, 0.0, 0.0, 0.0),
    )
    aerial = VisualGoalTrajectoryPrompt(aerial=True).build_user_prompt(
        "drive to the target",
        _observation(WorldPose(0.0, 0.0, 0.0, 0.0)),
        5,
        round_index=1,
        task_context=_context(),
        current_guidance="turn right",
        guidance_start=WorldPose(0.0, 0.0, 0.0, 0.0),
    )
    assert "Current vehicle pose:" in ground
    assert "Down: 0.000 m" in ground
    assert "Guidance reference vehicle pose" in ground
    assert "Current UAV pose:" in aerial
    assert "NED down: 0.000 m" in aerial
    assert "Guidance reference UAV pose" in aerial
    assert "Current vehicle pose:" not in aerial
    assert "Guidance reference vehicle pose" not in aerial
    assert "Current UAV pose:" not in ground


def test_aerial_visual_goal_output_guidance_allows_vertical_motion():
    prompt = VisualGoalTrajectoryPrompt(aerial=True).build_user_prompt(
        "fly to the target",
        _observation(WorldPose(0.0, 0.0, 0.0, 0.0)),
        5,
        round_index=1,
        task_context=_context(),
    )
    assert "[[0.5,0.1,-0.2,5.0]" in prompt
    assert "ground vehicle" not in prompt
    assert "dz = 0 because this is a ground vehicle" not in prompt


def test_visual_goal_prompt_requires_typed_context():
    with pytest.raises(ProtocolError):
        VisualGoalTrajectoryPrompt().build_user_prompt(
            "drive", _observation(WorldPose(0.0, 0.0, 0.0, 0.0)), 5,
        )


def test_base_prompt_rejects_task_context():
    with pytest.raises(ProtocolError):
        BaseTrajectoryPrompt().build_user_prompt(
            "drive", _observation(WorldPose(0.0, 0.0, 0.0, 0.0)), 5,
            task_context=_context(),
        )


def test_pending_tail_is_the_new_action_reference():
    pending = RelativeTrajectory(
        (RelativePoseDelta(dx=1.0), RelativePoseDelta(dx=1.0))
    )
    prompt = _visual_prompt(pending_trajectory=pending)
    assert '"new_actions_reference":"pending_tail"' in prompt
    assert '"pending_tail_is_predicted":true' in prompt
    assert '"target_relative_to_observation_body_m":[10.0,0.0,0.0]' in prompt
    assert '"target_relative_to_pending_tail_body_m":[8.0,0.0,0.0]' in prompt
    assert (
        "Pending incremental pose waypoints: "
        "[[1.0,0.0,0.0,0.0],[1.0,0.0,0.0,0.0]]" in prompt
    )
    assert "Start your NEW trajectory AFTER all pending actions" in prompt


def test_empty_pending_uses_the_observation_reference():
    prompt = _visual_prompt()
    assert '"new_actions_reference":"observation"' in prompt
    assert '"pending_tail_is_predicted":false' in prompt
    assert "Pending incremental pose waypoints: []" in prompt
    assert (
        "Pending is empty. Start your NEW trajectory at the current "
        "observation pose." in prompt
    )


def test_target_on_the_right_forbids_left_turns():
    prompt = _visual_prompt(context=_context(world_point=(10.0, 5.0, 0.0)))
    assert "The target is currently on the RIGHT side." in prompt
    assert "Negative dyaw_deg (turning left, away from the target) is forbidden" in prompt
    assert "dyaw_deg = 0 is allowed" in prompt


def test_target_on_the_left_forbids_right_turns():
    prompt = _visual_prompt(context=_context(world_point=(10.0, -5.0, 0.0)))
    assert "The target is currently on the LEFT side." in prompt
    assert "Positive dyaw_deg (turning right, away from the target) is forbidden" in prompt


def test_zero_bearing_adds_no_side_restriction():
    prompt = _visual_prompt()
    assert (
        "The target is directly ahead or behind; no dyaw_deg side restriction applies."
        in prompt
    )


def test_visual_goal_system_prompts_forbid_turning_away_from_the_target_side():
    for prompt in (VISUAL_GOAL_SYSTEM_PROMPT, AERIAL_VISUAL_GOAL_SYSTEM_PROMPT):
        assert "never output a positive" in prompt
        assert "never output a negative" in prompt
        assert "turns toward the target remain allowed" in prompt


def test_history_and_measured_change_are_reported():
    previous = RelativeTrajectory(
        (RelativePoseDelta(dx=0.5, dyaw_deg=5.0),)
    )
    prompt = _visual_prompt(
        pose=WorldPose(1.0, 2.0, 0.0, 90.0),
        initial_pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        previous_trajectory=previous,
        previous_trajectory_start=WorldPose(0.0, 0.0, 0.0, 0.0),
        round_index=2,
    )
    assert "Planning round: 2" in prompt
    assert "Task state: continuing" in prompt
    assert (
        "Recent confirmed executed actions, not a complete previous planning "
        "batch (do not execute it again): [[0.5,0.0,0.0,5.0]]" in prompt
    )
    assert "Execution-history reference start pose: [0.0,0.0,0.0,0.0]" in prompt
    assert "Forward: 1.000 m" in prompt
    assert "Right: 2.000 m" in prompt
    assert "Yaw change: 90.000 degrees" in prompt


def test_first_round_without_history_is_explicit():
    prompt = _visual_prompt()
    assert "Task state: initial" in prompt
    assert "No previous trajectory has been executed." in prompt


def test_pending_longer_than_five_points_is_rejected():
    pending = RelativeTrajectory(
        tuple(RelativePoseDelta(dx=1.0) for _ in range(6))
    )
    with pytest.raises(ProtocolError):
        QwenVLPlanner(_NoTransport()).plan(
            _observation(WorldPose(0.0, 0.0, 0.0, 0.0)),
            "drive",
            pending_trajectory=pending,
            task_context=_context(),
        )


def test_pending_yaw_outside_the_range_is_rejected():
    pending = RelativeTrajectory((RelativePoseDelta(dyaw_deg=181.0),))
    with pytest.raises(ProtocolError):
        QwenVLPlanner(_NoTransport()).plan(
            _observation(WorldPose(0.0, 0.0, 0.0, 0.0)),
            "drive",
            pending_trajectory=pending,
            task_context=_context(),
        )


def test_planner_requires_exactly_five_expected_points():
    with pytest.raises(ValueError):
        QwenVLPlanner(_NoTransport(), expected_points=4)


def test_planner_sends_rgb_and_aligned_depth_then_parses():
    observation = _observation(WorldPose(0.0, 0.0, 0.0, 0.0))
    transport = _CapturingTransport(
        "[[0.5,0.0,0.0,0.0],[0.5,0.0,0.0,0.0],[0.5,0.0,0.0,0.0],"
        "[0.5,0.0,0.0,0.0],[0.5,0.0,0.0,0.0]]"
    )
    planner = QwenVLPlanner(
        transport, timeout_s=12.0, prompt_builder=VisualGoalTrajectoryPrompt()
    )
    trajectory = planner.plan(
        observation, "drive", task_context=_context(), timeout_s=20.0,
    )
    assert len(trajectory.points) == 5
    system, text, images, timeout_s, operation = transport.calls[0]
    assert system == VISUAL_GOAL_SYSTEM_PROMPT
    assert "Visual goal context" in text
    assert images[0][0] is observation.rgb and images[0][1] == "JPEG"
    assert images[1][0] is observation.depth_image and images[1][1] == "PNG"
    assert timeout_s == 12.0
    assert operation == "trajectory planning"


def test_planner_rejects_a_response_without_five_points():
    transport = _CapturingTransport("[[0.5,0.0,0.0,0.0]]")
    planner = QwenVLPlanner(transport, prompt_builder=VisualGoalTrajectoryPrompt())
    with pytest.raises(ProtocolError):
        planner.plan(
            _observation(WorldPose(0.0, 0.0, 0.0, 0.0)),
            "drive",
            task_context=_context(),
        )