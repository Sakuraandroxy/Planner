"""Which pipeline flies a visual-goal mission is a closed set of names.

The default stays ``staged``, so the world-model profile keeps flying what it
flies today without that profile's config file naming anything.
"""

import pytest

from planner.config import VisualGoalNavigationSettings, load_config
from tests.support import write_config


def test_staged_is_the_default():
    assert VisualGoalNavigationSettings().execution == "staged"


@pytest.mark.parametrize("name", ["staged", "bare"])
def test_both_pipelines_are_accepted(name):
    assert VisualGoalNavigationSettings(execution=name).execution == name


@pytest.mark.parametrize("name", ["", "bare ", "Bare", "segmented", None])
def test_an_unknown_pipeline_name_is_rejected(name):
    with pytest.raises(ValueError, match="execution"):
        VisualGoalNavigationSettings(execution=name)


def test_the_config_file_can_name_the_bare_pipeline(tmp_path):
    """The key reached this far as a dataclass field only; a profile may now set it."""
    path = write_config(tmp_path, {"visual_goal_navigation": {"execution": "bare"}})
    assert load_config(path).visual_goal_navigation.execution == "bare"
