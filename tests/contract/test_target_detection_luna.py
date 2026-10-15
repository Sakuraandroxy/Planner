from tests.fakes.target_detection import detection_json
from tests.fakes.configuration import visual_config_data, write_config
import base64
import io
from dataclasses import replace
from unittest.mock import Mock
import numpy as np
from PIL import Image
import pytest

from config import load_config
from planner.adapters.multimodal_chat import MultimodalChatClient
from planner.adapters.target_detection import OpenAICompatibleTargetDetector
from planner.adapters.target_detection.client import parse_target_detection
from planner.bootstrap import build_runtime
from planner.capabilities.targeting.bbox_goal_observation import BBoxGoalObserver
from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.capabilities.targeting.association import make_reference
from planner.domain.target_detection import TargetDetectionInput
from planner.domain.visual_goal import GoalVisibility
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.pose import WorldPose
from planner.errors import ConfigurationError, ProtocolError
from tests.fakes.visual_goal import wall_observation, VISIBLE, Clock
from tests.integration.test_visual_goal_workflow import setup


@pytest.mark.parametrize("raw", ["{}", "true", "null", "[[0,0],[0,1]]", "[[.1,0],[1,1]]",
    "[[true,0],[1,1]]", '[["0",0],[1,1]]', "[[-1,0],[1,1]]", "[[0,0],[2,1]]",
    "[[0,0],[NaN,1]]", "[[0,0],[1,1],[1,1]]", "```json\n[]\n```"])
def test_corner_protocol_is_strict(raw):
    with pytest.raises(ProtocolError):
        parse_target_detection(raw)


def test_empty_detection_is_distinct_from_invalid_reply():
    assert not parse_target_detection("[]").found
    assert parse_target_detection(detection_json(((0.1, 0.2), (0.7, 0.8)))).bbox.coordinates == (.1, .2, .7, .8)


def test_detection_wire_uses_short_prompt_none_and_own_key(monkeypatch):
    response = Mock()
    response.json.return_value = {"choices": [{"finish_reason": "stop", "message": {"content": detection_json(((.1,.2),(.7,.8)))}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr("requests.post", post)
    obs = replace(wall_observation(size=(1920, 1080)), rgb=Image.new("RGB", (1920, 1080)))
    transport = MultimodalChatClient("https://detection.invalid/v1", "detector-model", "detector-key", 20,
        max_completion_tokens=256, reasoning_effort="none")
    detector = OpenAICompatibleTargetDetector(transport)
    text = "Fly near the first car from left to right. 飞到从左往右数第一辆车附近。"
    detector.detect(TargetDetectionInput(obs, text, "first car from left to right"), timeout_s=10)
    body = post.call_args.kwargs["json"]
    assert body["model"] == "detector-model" and body["reasoning_effort"] == "none"
    assert body["max_completion_tokens"] == 256 and "max_tokens" not in body and "temperature" not in body
    assert post.call_args.kwargs["headers"]["Authorization"] == "Bearer detector-key"
    assert "origin at the top-left image boundary, x rightward, y downward" in body["messages"][0]["content"]
    assert "surface_points" not in body["messages"][0]["content"]
    content = body["messages"][1]["content"]
    assert len(content) == 2 and text in content[-1]["text"]
    image = Image.open(io.BytesIO(base64.b64decode(content[0]["image_url"]["url"].split(",", 1)[1])))
    assert image.size == (512, 288) and obs.rgb.size == (1920, 1080)


def detector_fixture():
    detector = Mock()
    detector.detect.return_value = parse_target_detection(detection_json(((0.25, 0.25), (0.75, 0.75))))
    geometry, limits = DepthGeometry(.5, 200, True), VisualGoalLimits()
    return BBoxGoalObserver(detector, 20, Clock()), detector, geometry, limits






@pytest.mark.parametrize("sliding", [False, True])
def test_current_goal_workflow_uses_box_capability_and_clears_on_completion(sliding):
    workflow, stage, source, old_observer, planner, vehicle, clock = setup()
    observer, detector, geometry, limits = detector_fixture()
    observer.clock = clock
    workflow.goal_observer = observer
    if sliding:
        from planner.services.planning_window import PlanningWindow
        from planner.domain.sliding_window import SlidingWindowSettings
        from planner.domain.motion import MotionLimits
        workflow.window = PlanningWindow(planner, workflow.validator, SlidingWindowSettings(enabled=True), MotionLimits(), clock)
    try:
        result = workflow.run(stage)
        assert result.success, result.error
        assert not old_observer.calls and detector.detect.call_count >= 2
        assert result.value.completion_confirmed and not result.value.remaining_motion.segments
        assert workflow.verification.checker.calls[-1].reference_rgb is not None
    finally:
        workflow.close()


def test_no_detection_dependency_for_unused_workflow(tmp_path, monkeypatch):
    raw = visual_config_data()
    raw["visual_goal_navigation"]["enabled"] = True
    raw["basic_motion"]["free_space"].update(confirmed=True, minimum_world_ned_m=[-20,-20,-20], maximum_world_ned_m=[20,20,20])
    raw.pop("target_detection")
    path = tmp_path / "unused.yaml"
    write_config(path, raw)
    monkeypatch.setattr("planner.bootstrap.AirSimConnection", Mock())
    constructor = Mock(side_effect=AssertionError("detector must not be built"))
    monkeypatch.setattr("planner.visual_goal_bootstrap.OpenAICompatibleTargetDetector", constructor)
    runtime = build_runtime(load_config(path))
    stage = MissionStage("basic", TaskKind.BASIC_MOTION, BasicMotionParameters("Turn right. 右转。", 0, 0, 0, 30))
    runtime.application.task_parser.parse = Mock(return_value=MissionPlan("mission_plan_v1", (stage,)))
    runtime.application.prepare("turn")
    constructor.assert_not_called()
    deferred = runtime.application.runner.router._workflows[TaskKind.VISUAL_GOAL_NAVIGATION]
    assert deferred.instance is None
    with pytest.raises(ConfigurationError, match="TARGET_DETECTION_API_KEY"):
        deferred.prepare()
    runtime.close()
