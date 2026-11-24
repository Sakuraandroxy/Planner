"""The bare pipeline's own knobs: one speed, one deadline, both positive."""

import pytest

from planner.config import (
    BareNavigationSettings,
    VisualGoalNavigationSettings,
    load_config,
    profile_config_path,
)
from tests.support import write_config


def test_defaults_are_one_metre_per_second_and_one_minute():
    bare = BareNavigationSettings()
    assert bare.speed_mps == 1.0
    assert bare.path_timeout_s == 60.0
    assert VisualGoalNavigationSettings().bare == BareNavigationSettings()


@pytest.mark.parametrize("name", ["speed_mps", "path_timeout_s"])
@pytest.mark.parametrize(
    "value", [0, 0.0, -1.0, float("nan"), float("inf"), True, "1.0"]
)
def test_a_value_that_is_not_a_positive_number_is_rejected(name, value):
    with pytest.raises(ValueError, match=name):
        BareNavigationSettings(**{name: value})


def test_the_config_file_rejects_an_unknown_bare_sub_key(tmp_path):
    path = write_config(tmp_path, {"visual_goal_navigation": {"bare": {"speed": 2.0}}})
    with pytest.raises(ValueError, match="unknown bare settings"):
        load_config(path)


def test_the_vlm_profile_selects_the_bare_pipeline():
    settings = load_config(profile_config_path("vlm")).visual_goal_navigation
    assert settings.execution == "bare"
    assert settings.bare.speed_mps == 1.0
    assert settings.bare.path_timeout_s == 60.0


def test_the_config_file_rejects_a_zero_speed(tmp_path):
    """The speed reaches the engine now, so a zero has to stop at load time."""
    path = write_config(
        tmp_path, {"visual_goal_navigation": {"bare": {"speed_mps": 0}}}
    )
    with pytest.raises(ValueError, match="speed_mps"):
        load_config(path)
