from tests.fakes.configuration import visual_config_data, write_config
import json
import math
from pathlib import Path
from unittest.mock import Mock
import requests
import pytest
import yaml
from config import load_config
from planner.adapters.trajectory_planner.response_parser import parse_trajectory
from planner.adapters.airsim.motion_tracker import limit_vector
from planner.adapters.task_parser.openai_task_parser import OpenAITaskParser
from planner.errors import ProtocolError, ExecutionError, ConfigurationError


@pytest.mark.parametrize("value", [True, False, "1", None, 10**500])
def test_waypoints_reject_non_json_numbers_and_overflow(value):
    with pytest.raises(ProtocolError):
        parse_trajectory(json.dumps([[value,0,0,0]]))


@pytest.mark.parametrize("vector,maximum", [((float("nan"),0,0),1), ((float("inf"),0,0),1),
    ((1,0,0),0), ((1,0,0),float("nan")), ((1,0,0),float("inf"))])
def test_motion_vector_requires_finite_valid_inputs(vector, maximum):
    with pytest.raises(ExecutionError):
        limit_vector(vector,maximum)


def test_large_finite_vector_is_normalized_without_square_overflow():
    result = limit_vector((1e308,1e308,0),1)
    assert math.hypot(*result) == pytest.approx(1)
    assert result[0] == pytest.approx(2**-.5)


@pytest.mark.parametrize("instruction", [None, 123, [], "", "   "])
def test_parser_invalid_input_never_sends_request(monkeypatch, instruction):
    post = Mock()
    monkeypatch.setattr("requests.post",post)
    with pytest.raises(ProtocolError):
        OpenAITaskParser("https://offline.invalid/v1","fake","test-key",20).parse(instruction)
    post.assert_not_called()


def test_parser_network_error_uses_safe_typed_message(monkeypatch):
    monkeypatch.setattr("requests.post",Mock(side_effect=requests.ConnectionError("private request details")))
    with pytest.raises(ProtocolError) as error:
        OpenAITaskParser("https://offline.invalid/v1","fake","test-key",20).parse("turn")
    assert "ConnectionError" in str(error.value) and "private request" not in str(error.value)


@pytest.mark.parametrize("section,key,value", [("root","goal_observer",{}),
    ("visual_goal_navigation","observer_kind","vlm_goal_observer"),
    ("visual_goal_navigation","depth_horizontal_fov_deg",90)])
def test_retired_config_fields_are_rejected(tmp_path,section,key,value):
    raw = yaml.safe_load(Path("config/base.yaml").read_text(encoding="utf-8"))
    if section == "root":
        raw[key] = value
    else:
        raw.setdefault(section,{})[key] = value
    path = tmp_path/"retired.yaml"
    write_config(path, raw)
    with pytest.raises(ConfigurationError):
        load_config(path)


@pytest.mark.parametrize("depth_fov", [None, 80.0])
def test_visual_goal_uses_one_calibrated_fov_in_source_and_trace(tmp_path,monkeypatch,depth_fov):
    from planner import bootstrap
    import planner.visual_goal_bootstrap as assembly
    from planner.domain.mission import TaskKind
    raw = visual_config_data()
    raw['visual_goal_navigation'].update(enabled=True, geometry_verified=True)
    raw["target_detection"]["api_key"] = "detector-test-key"
    raw["task_completion"]["api_key"] = "completion-test-key"
    raw["airsim"]["depth_horizontal_fov_deg"] = depth_fov
    path = tmp_path/"camera.yaml"
    write_config(path, raw)
    monkeypatch.setattr(bootstrap,"AirSimConnection",Mock())
    trace = Mock()
    monkeypatch.setattr(assembly,"VisualGoalTrace",Mock(return_value=trace))
    runtime = bootstrap.build_runtime(load_config(path))
    deferred = runtime.application.runner.router._workflows[TaskKind.VISUAL_GOAL_NAVIGATION]
    try:
        if depth_fov is None:
            with pytest.raises(ConfigurationError,match="airsim.depth_horizontal_fov_deg"):
                deferred.prepare()
        else:
            deferred.prepare()
            workflow = deferred.instance
            assert workflow.source.depth_horizontal_fov_deg == depth_fov
            workflow.trace_factory()
            assert trace.event.call_args.kwargs["depth_horizontal_fov_deg"] == depth_fov
    finally:
        runtime.close()
