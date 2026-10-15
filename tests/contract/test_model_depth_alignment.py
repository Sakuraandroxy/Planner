"""Actual offline request payloads for both consumers of RGB/depth images."""
import base64
import io
from dataclasses import replace
from unittest.mock import Mock

import pytest
from PIL import Image

from planner.adapters.task_completion import OpenAICompatibleTaskCompletionChecker
from planner.adapters.task_completion.prompts import VisualGoalCompletionPrompt
from planner.adapters.trajectory_planner.qwen_vl_planner import QwenVLPlanner
from planner.errors import ObservationError
from tests.unit.test_model_visual_inputs import asymmetric_observation
from tests.unit.test_visual_goal_completion_client import request


@pytest.mark.parametrize("consumer", ["trajectory", "completion"])
@pytest.mark.parametrize("valid_calibration", [True, False])
def test_model_request_uses_aligned_depth_and_refuses_missing_calibration(monkeypatch, consumer, valid_calibration):
    observation = asymmetric_observation()
    original_pixels = observation.depth_image.tobytes()
    if not valid_calibration:
        observation = replace(observation, depth_intrinsics=None)
    raw = "true" if consumer == "completion" else "[[1,0,0,0],[1,0,0,0],[1,0,0,0],[1,0,0,0],[1,0,0,0]]"
    response = Mock()
    response.json.return_value = {"choices": [{"finish_reason": "stop", "message": {"content": raw}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr("requests.post", post)
    if consumer == "trajectory":
        client = QwenVLPlanner("http://offline/v1", "fake-planner", "no-key", 20)
        call = lambda: client.plan(observation, "approach target")
    else:
        client = OpenAICompatibleTaskCompletionChecker("http://offline/v1", "fake-completion", "no-key", 20, prompt_builder=VisualGoalCompletionPrompt())
        call = lambda: client.check(replace(request(), observation=observation,
            initial_pose=observation.vehicle_pose), timeout_s=10)
    if not valid_calibration:
        with pytest.raises(ObservationError):
            call()
        post.assert_not_called()
        return
    call()
    post.assert_called_once()
    messages = post.call_args.kwargs["json"]["messages"]
    assert [message["role"] for message in messages] == ["system", "user"]
    parts = messages[1]["content"]
    assert [part["type"] for part in parts] == ["image_url", "image_url", "text"]
    url = parts[1]["image_url"]["url"]
    with Image.open(io.BytesIO(base64.b64decode(url.split(",", 1)[1]))) as depth:
        assert depth.size == (64, 36)
        assert depth.getpixel((32, 9)) == (240, 240, 240)
        assert depth.getpixel((32, 27)) == (20, 20, 20)
    assert observation.depth_image.size == (64, 64)
    assert observation.depth_image.tobytes() == original_pixels
