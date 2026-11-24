"""The prompt the bare pipeline actually gets is chosen at assembly time.

The bare workflow never builds a prompt of its own: ``visual_goal_navigation.execution``
decides the prompt and the output parser when the planner is constructed.
"""

from planner.adapters.trajectory_planner.prompts import (
    BareTrajectoryPrompt,
    VisualGoalTrajectoryPrompt,
)
from planner.adapters.trajectory_planner.response_parser import (
    parse_translation_trajectory,
)
from planner.application.factories import build_visual_goal_planner
from planner.config import default_config_path, load_config, profile_config_path


def test_the_bare_profile_builds_the_bare_prompt_and_three_number_parser():
    config = load_config(profile_config_path("vlm"))
    assert config.visual_goal_navigation.execution == "bare"
    planner = build_visual_goal_planner(config)
    assert isinstance(planner.prompt_builder, BareTrajectoryPrompt)
    assert planner.parser is parse_translation_trajectory


def test_the_staged_profile_keeps_the_target_prompt_and_five_point_parser():
    config = load_config(default_config_path())
    assert config.visual_goal_navigation.execution == "staged"
    planner = build_visual_goal_planner(config)
    assert isinstance(planner.prompt_builder, VisualGoalTrajectoryPrompt)
    assert planner.parser is None


def test_the_bare_prompt_is_built_for_the_drone_too():
    config = load_config(profile_config_path("vlm"))
    planner = build_visual_goal_planner(config, aerial=True)
    assert isinstance(planner.prompt_builder, BareTrajectoryPrompt)
    assert planner.prompt_builder.aerial is True
