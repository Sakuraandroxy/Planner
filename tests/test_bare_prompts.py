"""The bare prompt keeps the instruction and the output format, nothing else."""

import re

import numpy as np

from planner.adapters.trajectory_planner.prompts.bare import (
    BareTrajectoryPrompt,
    bare_system_prompt,
    build_bare_user_prompt,
)
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.trajectory import RelativeTrajectory

INSTRUCTION = "Drive forward and stop beside the red car."


def _observation() -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(640, 480, 90.0)
    return Observation(
        frame=7,
        timestamp_s=3.0,
        pose=WorldPose(41.5, -12.25, 0.75, 37.5),
        rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        depth_m=np.ones((480, 640), dtype=np.float32),
        depth_image=np.zeros((480, 640), dtype=np.uint8),
        intrinsics=intrinsics,
        camera_matrix=np.eye(4),
        camera_id="front",
        capture_id="capture-7",
        captured_monotonic=11.0,
    )


def _rendered_prompt(aerial: bool = False) -> str:
    builder = BareTrajectoryPrompt(aerial=aerial)
    user = builder.build_user_prompt(
        INSTRUCTION,
        _observation(),
        5,
        current_guidance="head for the gate",
        guidance_start=WorldPose(1.0, 2.0, 3.0, 4.0),
        round_index=9,
        initial_pose=WorldPose(1.0, 2.0, 3.0, 4.0),
        previous_trajectory=RelativeTrajectory((RelativePoseDelta(1.0, 0.0, 0.0, 5.0),)),
        previous_trajectory_start=WorldPose(1.0, 2.0, 3.0, 4.0),
        pending_trajectory=RelativeTrajectory((RelativePoseDelta(2.0, 0.0, 0.0, 0.0),)),
    )
    return builder.system_prompt() + "\n" + user


def test_prompt_keeps_the_original_instruction():
    assert INSTRUCTION in _rendered_prompt()
    assert INSTRUCTION in build_bare_user_prompt(INSTRUCTION)


def test_prompt_drops_pose_target_camera_queue_and_history():
    rendered = _rendered_prompt().lower()
    for withheld in ("pose", "world position", "camera", "pending", "history", "target"):
        assert withheld not in rendered, withheld


def test_prompt_keeps_the_output_format_description_and_one_example():
    rendered = _rendered_prompt()
    assert "[dx, dy, dz]" in rendered
    assert "Mandatory trajectory output format" in rendered
    assert "must contain exactly 3 finite JSON numbers" in rendered
    assert "Correct output example" in rendered
    assert "[[0.8,0.0,0.0]" in rendered


def test_prompt_never_asks_the_model_for_a_heading():
    for aerial in (False, True):
        rendered = _rendered_prompt(aerial=aerial).lower()
        assert "dyaw" not in rendered
        assert not re.search(r"\byaw|\bturns?\b|\bheading\b|\brotate", rendered)


def test_prompt_does_not_fix_the_waypoint_count():
    rendered = _rendered_prompt().lower()
    assert "exactly 5" not in rendered
    assert "five" not in rendered
    assert "the count is not fixed" in rendered


def test_aerial_variant_keeps_vertical_motion():
    ground = bare_system_prompt()
    aerial = bare_system_prompt(aerial=True)
    assert "output dz = 0" in ground and "output dz = 0" not in aerial
    assert "-dz is UP" in aerial and "-dz is UP" not in ground


def test_prompt_measures_every_waypoint_from_the_current_position():
    """Baseline geometry: the batch is not a chain of relative steps."""
    rendered = _rendered_prompt()
    assert "displacement from the current vehicle position" in rendered
    assert "Do not chain one waypoint onto the previous one" in rendered
