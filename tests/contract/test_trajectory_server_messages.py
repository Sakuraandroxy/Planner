"""Offline contracts for the separately deployed server; no model/GPU loading."""
import base64
import importlib.util
import io
import sys
from contextlib import nullcontext
from pathlib import Path
from types import ModuleType
from unittest.mock import MagicMock, Mock

import pytest
from PIL import Image

from planner.adapters.trajectory_planner.prompts import BaseTrajectoryPrompt
from planner.adapters.trajectory_planner.prompts.visual_goal_navigation import VisualGoalTrajectoryPrompt
from planner.adapters.trajectory_planner.qwen_vl_planner import QwenVLPlanner
from planner.domain.visual_goal import GoalGeometry, VisualGoalPlanningContext, VisualGoalState
from tests.fakes.visual_goal import VISIBLE, wall_observation


@pytest.fixture
def server(monkeypatch):
    pytest.importorskip("flask", reason="standalone server contract requires Flask")
    path = Path(__file__).resolve().parents[3] / "server.py"
    if not path.is_file():
        pytest.skip("standalone world_model/server.py is not present beside Planner")

    class InputTensor:
        shape = (1, 3)

        def to(self, device):
            return self

    torch = ModuleType("torch")
    torch.Tensor = InputTensor
    torch.float16 = "fake-float16"
    torch.inference_mode = nullcontext
    transformers = ModuleType("transformers")
    processor = Mock()
    processor.apply_chat_template.return_value = "rendered-system-and-user"
    processor.return_value = {"input_ids": InputTensor()}
    processor.decode.return_value = "[[1,0,0,0],[1,0,0,0],[1,0,0,0],[1,0,0,0],[1,0,0,0]]"
    model = Mock(device="fake-cpu")
    outputs = MagicMock()
    outputs.__getitem__.return_value = [7, 8]
    model.generate.return_value = outputs
    transformers.AutoProcessor = Mock()
    transformers.AutoProcessor.from_pretrained.return_value = processor
    transformers.AutoModelForImageTextToText = Mock()
    transformers.AutoModelForImageTextToText.from_pretrained.return_value = model
    monkeypatch.setitem(sys.modules, "torch", torch)
    monkeypatch.setitem(sys.modules, "transformers", transformers)
    # External dependencies are replaced BEFORE import, including from_pretrained.
    spec = importlib.util.spec_from_file_location("offline_trajectory_server", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def image_part(color):
    buffer = io.BytesIO()
    Image.new("RGB", (2, 2), color).save(buffer, format="PNG")
    url = "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")
    return {"type": "image_url", "image_url": {"url": url}}


def test_roles_and_content_order_are_preserved_without_mutating_request(server):
    messages = [
        {"role": "system", "content": "coordinate rules"},
        {"role": "user", "content": "earlier request"},
        {"role": "assistant", "content": "earlier answer"},
        {"role": "user", "content": [
            {"type": "text", "text": "before RGB"}, image_part("red"),
            {"type": "text", "text": "before depth"}, image_part("gray"),
            {"type": "text", "text": "current task and pending"},
        ]},
    ]
    prepared, images = server.prepare_chat_messages(messages)
    assert [item["role"] for item in prepared] == ["system", "user", "assistant", "user"]
    assert prepared[:3] == messages[:3]
    assert [part["type"] for part in prepared[-1]["content"]] == ["text", "image", "text", "image", "text"]
    assert prepared[-1]["content"][-1]["text"] == "current task and pending"
    assert [image.getpixel((0, 0)) for image in images] == [(255, 0, 0), (128, 128, 128)]
    assert messages[-1]["content"][1]["type"] == "image_url"
    server.model.generate.assert_not_called()


@pytest.mark.parametrize("task_specific", [False, True])
def test_planner_to_service_keeps_roles_and_generates_once(server, monkeypatch, task_specific):
    client = server.app.test_client()
    post = Mock()

    def send(url, **kwargs):
        result = client.post("/v1/chat/completions", json=kwargs["json"])
        assert result.status_code == 200
        return Mock(json=Mock(return_value=result.get_json()))

    post.side_effect = send
    monkeypatch.setattr("planner.adapters.trajectory_planner.qwen_vl_planner.requests.post", post)
    builder = VisualGoalTrajectoryPrompt() if task_specific else BaseTrajectoryPrompt()
    planner = QwenVLPlanner("http://offline/v1", "offline-model", "no-key", 20, prompt_builder=builder)
    goal_context = VisualGoalPlanningContext('red house', VISIBLE, GoalGeometry(True, 'surface', 6, (6, 0, -4)), VisualGoalState.APPROACHING, 4, 0.3, None)
    extra = {"task_context": goal_context} if task_specific else {}
    result = planner.plan(wall_observation(), "approach the red house", **extra)
    assert len(result.points) == 5
    post.assert_called_once()
    server.model.generate.assert_called_once()
    server.processor.apply_chat_template.assert_called_once()
    call = server.processor.apply_chat_template.call_args
    prepared = call.args[0]
    assert call.kwargs == {"tokenize": False, "add_generation_prompt": True}
    assert [message["role"] for message in prepared] == ["system", "user"]
    assert prepared[0]["content"] == builder.system_prompt()
    user = prepared[1]["content"]
    assert [part["type"] for part in user] == ["image", "image", "text"]
    assert "Pending incremental pose waypoints: []" in user[-1]["text"]
    assert "Output exactly 5 NEW" in user[-1]["text"]
    assert ("Visual goal context" in user[-1]["text"]) is task_specific
    assert len(server.processor.call_args.kwargs["images"][0]) == 2


@pytest.mark.parametrize("messages", [
    [], None, "not a list", [None],
    [{"role": "tool", "content": "not supported"}],
    [{"role": "system", "content": "missing user"}],
    [{"role": "user", "content": {"text": "wrong content shape"}}],
    [{"role": "user", "content": [{"type": "text", "text": 12}]}],
    [{"role": "user", "content": [{"type": "audio", "data": "unsupported"}]}],
    [{"role": "user", "content": [{"type": "image_url", "image_url": {"url": "bad-image"}}]}],
])
def test_bad_chat_input_returns_400_before_generation(server, messages):
    response = server.app.test_client().post("/v1/chat/completions", json={"messages": messages})
    assert response.status_code == 400
    assert response.get_json()["error"]["type"] == "invalid_request_error"
    server.model.generate.assert_not_called()
    server.processor.apply_chat_template.assert_not_called()


def test_extra_image_is_rejected_instead_of_silently_dropped(server):
    response = server.app.test_client().post("/v1/chat/completions", json={"messages": [
        {"role": "user", "content": [image_part("red")] * 3},
    ]})
    assert response.status_code == 400
    server.model.generate.assert_not_called()


def test_text_only_chat_keeps_system_role(server):
    response = server.app.test_client().post("/v1/chat/completions", json={"messages": [
        {"role": "system", "content": "stable rules"},
        {"role": "user", "content": "current request"},
    ]})
    assert response.status_code == 200
    assert [item["role"] for item in server.processor.apply_chat_template.call_args.args[0]] == ["system", "user"]
    assert "images" not in server.processor.call_args.kwargs
    server.model.generate.assert_called_once()


def test_legacy_plan_still_works_and_health_reports_role_support(server):
    client = server.app.test_client()
    response = client.post("/plan", json={"instruction": "fly forward", "rgb_image": image_part("red")["image_url"]})
    assert response.status_code == 200 and response.get_json()["success"]
    prepared = server.processor.apply_chat_template.call_args.args[0]
    assert len(prepared) == 1 and prepared[0]["role"] == "user"
    assert "fly forward" in prepared[0]["content"][-1]["text"]
    server.model.generate.assert_called_once()
    health = client.get("/health").get_json()
    assert health["chat_message_roles_preserved"] is True
    assert health["pending_supported"] is False
