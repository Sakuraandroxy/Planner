from PIL import Image

from planner.adapters.trajectory_planner.prompts import (
    BASE_TRAJECTORY_SYSTEM_PROMPT,
    NAVIGATION_TRAJECTORY_SYSTEM_PROMPT,
    NavigationTrajectoryPrompt,
)
from planner.adapters.trajectory_planner.prompt import build_prompt
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.trajectory import RelativeTrajectory


def observation(pose: WorldPose) -> Observation:
    image = Image.new("RGB", (2, 2))
    return Observation(
        image, None, image, pose, "front_center",
        CameraIntrinsics(2, 2, 1, 1, 1, 1, 90),
        (pose.x, pose.y, pose.z),
        ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
        1,
    )


def test_navigation_uses_base_system_prompt_and_initial_user_context():
    builder = NavigationTrajectoryPrompt()
    current = observation(WorldPose(1, 2, -3, 10))

    prompt = builder.build_user_prompt("follow the road", current, 5)

    assert NAVIGATION_TRAJECTORY_SYSTEM_PROMPT == BASE_TRAJECTORY_SYSTEM_PROMPT
    assert builder.system_prompt() == NAVIGATION_TRAJECTORY_SYSTEM_PROMPT
    assert "Adverse-visibility rules:" in builder.system_prompt()
    assert "Adverse weather alone is not a reason to shorten the trajectory" in builder.system_prompt()
    normalized_system_prompt = " ".join(builder.system_prompt().split())
    assert "Do not reduce waypoint count or displacement solely" in normalized_system_prompt
    assert "Original navigation instruction for the current mission stage:\nfollow the road" in prompt
    assert "Planning round: 1" in prompt
    assert "Task state: initial" in prompt
    assert "No previous trajectory has been executed." in prompt
    assert "Forward: 0.000 m" in prompt
    assert "Output only 1 to 5 relative trajectory points." in prompt
    assert '"camera_id":"front_center"' in prompt

    legacy_prompt = build_prompt("follow the road", current, 5)
    assert legacy_prompt.startswith(BASE_TRAJECTORY_SYSTEM_PROMPT)
    assert "Original navigation instruction for the current mission stage:\nfollow the road" in legacy_prompt


def test_base_user_prompt_contains_continuing_guidance_and_history():
    builder = NavigationTrajectoryPrompt()
    start = WorldPose(0, 0, -2, 0)
    current = observation(WorldPose(4, 1, 1, 15))
    previous = RelativeTrajectory((RelativePoseDelta(2, 0, 1, 5),))

    prompt = builder.build_user_prompt(
        "follow the road",
        current,
        5,
        current_guidance="continue toward the junction",
        round_index=2,
        initial_pose=start,
        previous_trajectory=previous,
    )

    assert "Current guidance:\ncontinue toward the junction" in prompt
    assert "Planning round: 2" in prompt
    assert "Task state: continuing" in prompt
    assert "Previous successfully executed trajectory from the preceding planning round" in prompt
    assert "(do not execute it again): [[2,0,1,5]]" in prompt
    assert "Forward: 4.000 m" in prompt
    assert "Right: 1.000 m" in prompt
    assert "NED down: 3.000 m" in prompt
    assert "Yaw change: 15.000 degrees" in prompt


def test_continuing_prompt_without_guidance_does_not_restart_task():
    builder = NavigationTrajectoryPrompt()
    current = observation(WorldPose(1, 0, -2, 0))

    prompt = builder.build_user_prompt(
        "follow the road",
        current,
        5,
        round_index=2,
        initial_pose=WorldPose(0, 0, -2, 0),
    )

    assert "Continue the original navigation instruction" in prompt
    assert "without restarting completed actions" in prompt
    assert "Begin the original navigation instruction" not in prompt
