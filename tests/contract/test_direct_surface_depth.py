"""Removed neighborhood options are rejected; direct-depth preparation needs no cv2."""
from dataclasses import fields
from unittest.mock import Mock
import sys

import pytest

from config import load_config
from planner.bootstrap import build_runtime
from planner.domain.mission import TaskKind
from planner.domain.visual_goal import GoalGeometry
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.errors import ConfigurationError
from tests.fakes.configuration import MAIN_CONFIG, main_config_data, visual_config_data, write_config


REMOVED_OPTIONS = ("surface_depth_support_enabled", "surface_depth_tolerance_m",
    "minimum_surface_support_ratio", "minimum_surface_support_pixels",
    "minimum_valid_depth_ratio", "maximum_depth_mad_m", "depth_window_fraction")


def test_neighborhood_options_are_absent_from_runtime_and_main_config():
    limits = {f.name for f in fields(VisualGoalLimits)}
    assert not limits.intersection(REMOVED_OPTIONS)
    assert not set(main_config_data()["visual_goal_navigation"]["limits"]).intersection(REMOVED_OPTIONS)
    assert not {f.name for f in fields(GoalGeometry)}.intersection({"valid_ratio", "mad_m"})
    load_config(MAIN_CONFIG)


@pytest.mark.parametrize("name", REMOVED_OPTIONS)
def test_removed_settings_are_not_silently_accepted(tmp_path, name):
    raw = visual_config_data()
    raw["visual_goal_navigation"]["limits"][name] = False if name.endswith("enabled") else .7
    with pytest.raises(ConfigurationError, match=name):
        load_config(write_config(tmp_path/"old-setting.yaml", raw))


def test_visual_preparation_and_diagnostics_need_no_opencv(tmp_path, monkeypatch):
    import planner.visual_goal_bootstrap as assembly
    raw = visual_config_data()
    raw['visual_goal_navigation'].update(geometry_verified=True)
    for role in ("target_detection", "task_completion"):
        monkeypatch.setenv(raw[role]["api_key_env"], "offline-"+role)
    monkeypatch.setitem(sys.modules, "cv2", None)
    monkeypatch.setattr("planner.bootstrap.AirSimConnection", Mock())
    post = Mock(side_effect=AssertionError("no model requests during preparation"))
    monkeypatch.setattr("requests.post", post)
    trace = Mock()
    monkeypatch.setattr(assembly, "VisualGoalTrace", Mock(return_value=trace))
    runtime = build_runtime(load_config(write_config(tmp_path/"direct.yaml", raw)))
    try:
        deferred = runtime.application.runner.router._workflows[TaskKind.VISUAL_GOAL_NAVIGATION]
        deferred.prepare()
        deferred.instance.trace_factory()
        data = trace.event.call_args.kwargs
        assert data["target_depth_sampling_mode"] == "direct_pixel"
        assert data["goal_protocol"] == "normalized_bbox_corners_v1"
        assert not set(data).intersection(REMOVED_OPTIONS)
        post.assert_not_called()
        runtime.connection.connect.assert_not_called()
        runtime.connection.takeoff.assert_not_called()
    finally:
        runtime.close()
