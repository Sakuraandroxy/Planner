from PIL import Image
import pytest

from planner.adapters.trajectory_planner.prompts import (
    BASE_TRAJECTORY_SYSTEM_PROMPT,
    BaseTrajectoryPrompt,
)
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


def test_base_system_prompt_and_initial_user_context():
    builder = BaseTrajectoryPrompt()
    current = observation(WorldPose(1, 2, -3, 10))

    prompt = builder.build_user_prompt("follow the road", current, 5)

    assert builder.system_prompt() == BASE_TRAJECTORY_SYSTEM_PROMPT
    normalized_system_prompt = " ".join(builder.system_prompt().split())
    assert "Visual and adverse-visibility rules:" in normalized_system_prompt
    assert "Adverse weather alone is not a reason to stop frequently or shorten motion" in normalized_system_prompt
    assert "Do not dismiss a real surface merely because the RGB image is degraded" in normalized_system_prompt
    assert "Original navigation instruction for the current mission stage:\nfollow the road" in prompt
    assert "Planning round: 1" in prompt
    assert "Task state: initial" in prompt
    assert "No previous trajectory has been executed." in prompt
    assert "Forward: 0.000 m" in prompt
    assert "Output exactly 5 NEW incremental pose waypoints" in prompt
    assert "Pending incremental pose waypoints: []" in prompt
    assert '"camera_id":"front_center"' in prompt



def test_base_user_prompt_contains_continuing_guidance_and_history():
    builder = BaseTrajectoryPrompt()
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
    assert "Recent confirmed executed actions, not a complete previous planning batch" in prompt
    assert "(do not execute it again): [[2,0,1,5]]" in prompt
    assert "Forward: 4.000 m" in prompt
    assert "Right: 1.000 m" in prompt
    assert "NED down: 3.000 m" in prompt
    assert "Yaw change: 15.000 degrees" in prompt


def test_continuing_prompt_without_guidance_does_not_restart_task():
    builder = BaseTrajectoryPrompt()
    current = observation(WorldPose(1, 0, -2, 0))

    prompt = builder.build_user_prompt(
        "follow the road",
        current,
        5,
        round_index=2,
        initial_pose=WorldPose(0, 0, -2, 0),
    )

    assert "Current guidance:\nNot supplied." in prompt
    assert "Execution history is not supplied; do not infer that no motion has occurred." in prompt


def test_pending_system_rules_preserve_current_empty_queue_runtime():
    system = " ".join(BaseTrajectoryPrompt().system_prompt().split())
    assert "If no pending queue is supplied, treat it as empty" in system
    assert "Never reinterpret the previous successfully executed trajectory as pending" in system
    assert "first NEW output action is relative to the final pending predicted pose" in system
    assert "If pending is empty, the first NEW output action is relative to the current UAV pose" in system
    assert "Identical numeric actions are allowed" in system
    assert "not verified free space" in system
    assert "Transform translation using the preceding pose's yaw, then add dyaw_deg" in system


def test_system_and_user_require_exactly_five_new_points():
    builder = BaseTrajectoryPrompt()
    system = " ".join(builder.system_prompt().split())
    assert "Return exactly 5 NEW actions" in system
    assert "When it specifies a range" not in system
    with pytest.raises(ValueError, match="exactly 5"):
        builder.build_user_prompt("follow the road", observation(WorldPose(0,0,-4,0)), 3)


def test_reserved_pending_prompt_is_distinct_from_executed_history():
    pending = RelativeTrajectory((RelativePoseDelta(2,0,0,15),) * 3)
    previous = RelativeTrajectory((RelativePoseDelta(1,0,0,-10),))
    user = BaseTrajectoryPrompt().build_user_prompt("follow the road",
        observation(WorldPose(0,0,-4,0)), 5,
        previous_trajectory=previous, pending_trajectory=pending)
    assert "Pending incremental pose waypoints: [[2,0,0,15],[2,0,0,15],[2,0,0,15]]" in user
    assert "(do not execute it again): [[1,0,0,-10]]" in user
    assert "Output exactly 5 NEW" in user


def test_reserved_prompt_rejects_more_than_five_pending_points():
    pending = RelativeTrajectory((RelativePoseDelta(1,0,0,0),) * 6)
    with pytest.raises(ValueError, match="0 to 5"):
        BaseTrajectoryPrompt().build_user_prompt("follow the road",
            observation(WorldPose(0,0,-4,0)), 5, pending_trajectory=pending)
