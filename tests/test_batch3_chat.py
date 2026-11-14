import numpy as np
import pytest

import planner.adapters.multimodal_chat as multimodal_chat
from tests.support import ue_camera_matrix
from planner.adapters.chat_request_options import generation_options
from planner.adapters.image_encoding import image_url
from planner.adapters.model_visual_inputs import aligned_depth_image
from planner.adapters.multimodal_chat import MultimodalChatClient
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import WorldPose
from planner.errors import ObservationError, ProtocolError


def _observation(*, depth_image=None) -> Observation:
    width, height = 8, 6
    pose = WorldPose(0.0, 0.0, 0.0, 0.0)
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=pose,
        rgb=np.zeros((height, width, 3), dtype=np.uint8),
        depth_m=np.full((height, width), 5.0, dtype=np.float32),
        intrinsics=CameraIntrinsics.from_horizontal_fov(width, height, 90.0),
        camera_matrix=ue_camera_matrix(pose.x, pose.y, -pose.z, pose.yaw_deg),
        depth_image=depth_image,
    )


def test_generation_options_match_the_selected_dialect():
    assert generation_options(2048, "none", "openai") == {
        "max_completion_tokens": 2048,
        "reasoning_effort": "none",
    }
    assert generation_options(None, None, "openai") == {"temperature": 0.0, "max_tokens": 1024}
    assert generation_options(2048, None, "openai") == {"max_completion_tokens": 2048}
    assert generation_options(2048, "none", "deepseek") == {
        "max_tokens": 2048,
        "thinking": {"type": "disabled"},
    }
    assert generation_options(2048, "high", "deepseek") == {
        "max_tokens": 2048,
        "thinking": {"type": "enabled"},
        "reasoning_effort": "high",
    }
    assert generation_options(2048, "none", "anthropic") == {
        "max_tokens": 2048,
        "temperature": 0.0,
        "thinking": {"type": "disabled"},
    }
    with pytest.raises(ValueError):
        generation_options(2048, "high", "anthropic")
    with pytest.raises(ValueError):
        generation_options(2048, "none", "xml")


def test_multimodal_client_normalizes_url_and_validates_format():
    client = MultimodalChatClient("http://host:8010/v1", "model", "key", 10.0)
    assert client.url == "http://host:8010/v1/chat/completions"
    already = MultimodalChatClient("http://host:8010/v1/chat/completions", "model", "key", 10.0)
    assert already.url == "http://host:8010/v1/chat/completions"
    with pytest.raises(ValueError):
        MultimodalChatClient("http://host", "model", "key", 10.0, request_format="xml")
    anthropic = MultimodalChatClient(
        "https://api.anthropic.com/v1", "model", "key", 10.0,
        request_format="anthropic")
    assert anthropic.url == "https://api.anthropic.com/v1/messages"
    exact = MultimodalChatClient(
        "https://api.anthropic.com/v1/messages", "model", "key", 10.0,
        request_format="anthropic")
    assert exact.url == "https://api.anthropic.com/v1/messages"


class _Response:
    def __init__(self, payload):
        self.payload = payload

    def raise_for_status(self):
        return None

    def json(self):
        return self.payload


def _completion(text="yes"):
    return {"choices": [{"finish_reason": "stop", "message": {"content": text}}],
            "usage": {"prompt_tokens": 1, "completion_tokens": 2}}


def test_multimodal_client_posts_images_text_and_generation_options(monkeypatch):
    captured = {}

    def fake_post(url, *, headers, json, timeout):
        captured.update(url=url, headers=headers, json=json, timeout=timeout)
        return _Response(_completion())

    monkeypatch.setattr(multimodal_chat.httpx, "post", fake_post)
    client = MultimodalChatClient("http://host:8010/v1", "model", "secret", 10.0,
                                  max_completion_tokens=512)
    text = client.request("system prompt", "is this complete?",
                          ((np.zeros((2, 2, 3), dtype=np.uint8), "PNG"),),
                          timeout_s=5.0, operation="completion")
    assert text == "yes"
    assert captured["url"] == "http://host:8010/v1/chat/completions"
    assert captured["headers"] == {"Authorization": "Bearer secret"}
    assert captured["timeout"] == 5.0
    messages = captured["json"]["messages"]
    assert messages[0]["content"] == "system prompt"
    parts = messages[1]["content"]
    assert parts[0]["image_url"]["url"].startswith("data:image/png;base64,")
    assert parts[-1] == {"type": "text", "text": "is this complete?"}
    assert captured["json"]["max_completion_tokens"] == 512


def test_multimodal_client_posts_anthropic_messages(monkeypatch):
    captured = {}

    def fake_post(url, *, headers, json, timeout):
        captured.update(url=url, headers=headers, json=json, timeout=timeout)
        return _Response({
            "content": [
                {"type": "thinking", "thinking": "hidden"},
                {"type": "text", "text": "yes"},
            ],
            "stop_reason": "end_turn",
            "usage": {"input_tokens": 3, "output_tokens": 1},
        })

    monkeypatch.setattr(multimodal_chat.httpx, "post", fake_post)
    client = MultimodalChatClient(
        "https://api.anthropic.com", "claude-test", "secret", 10.0,
        request_format="anthropic", max_completion_tokens=512)
    text = client.request("system prompt", "is this complete?",
                          ((np.zeros((2, 2, 3), dtype=np.uint8), "PNG"),),
                          timeout_s=5.0, operation="completion")
    assert text == "yes"
    assert captured["url"] == "https://api.anthropic.com/v1/messages"
    assert captured["headers"] == {
        "content-type": "application/json",
        "anthropic-version": "2023-06-01",
        "x-api-key": "secret",
    }
    body = captured["json"]
    assert body["model"] == "claude-test"
    assert body["system"] == "system prompt"
    assert body["max_tokens"] == 512
    assert body["temperature"] == 0.0
    assert body["thinking"] == {"type": "disabled"}
    assert "max_completion_tokens" not in body
    assert body["messages"][0]["role"] == "user"
    parts = body["messages"][0]["content"]
    assert parts[0]["type"] == "image"
    assert parts[0]["source"]["type"] == "base64"
    assert parts[0]["source"]["media_type"] == "image/png"
    assert parts[-1] == {"type": "text", "text": "is this complete?"}


def test_multimodal_client_rejects_anthropic_truncation(monkeypatch):
    client = MultimodalChatClient(
        "https://api.anthropic.com", "claude-test", "secret", 10.0,
        request_format="anthropic")
    monkeypatch.setattr(multimodal_chat.httpx, "post", lambda *a, **k: _Response({
        "content": [{"type": "text", "text": "yes"}],
        "stop_reason": "max_tokens",
    }))
    with pytest.raises(ProtocolError):
        client.request("s", "t", (), timeout_s=1.0, operation="grounding")
    monkeypatch.setattr(multimodal_chat.httpx, "post", lambda *a, **k: _Response({
        "content": [{"type": "thinking", "thinking": "hidden"}],
        "stop_reason": "end_turn",
    }))
    with pytest.raises(ProtocolError):
        client.request("s", "t", (), timeout_s=1.0, operation="grounding")


def test_multimodal_client_omits_authorization_without_key(monkeypatch):
    captured = {}

    def fake_post(url, *, headers, json, timeout):
        captured["headers"] = headers
        return _Response(_completion())

    monkeypatch.setattr(multimodal_chat.httpx, "post", fake_post)
    client = MultimodalChatClient("http://host:8010/v1", "model", "", 10.0)
    assert client.request("s", "t", (), timeout_s=5.0, operation="completion") == "yes"
    assert captured["headers"] == {}


def test_multimodal_client_rejects_bad_timeouts_and_responses(monkeypatch):
    client = MultimodalChatClient("http://host", "model", "key", 10.0)
    with pytest.raises(ProtocolError):
        client.request("s", "t", (), timeout_s=0.0, operation="grounding")

    def failing_post(*args, **kwargs):
        raise multimodal_chat.httpx.ConnectError("down")

    monkeypatch.setattr(multimodal_chat.httpx, "post", failing_post)
    with pytest.raises(ProtocolError):
        client.request("s", "t", (), timeout_s=1.0, operation="grounding")

    monkeypatch.setattr(multimodal_chat.httpx, "post",
                        lambda *a, **k: _Response({"choices": [
                            {"finish_reason": "length", "message": {"content": "x"}}]}))
    with pytest.raises(ProtocolError):
        client.request("s", "t", (), timeout_s=1.0, operation="grounding")

    monkeypatch.setattr(multimodal_chat.httpx, "post",
                        lambda *a, **k: _Response(_completion(text="   ")))
    with pytest.raises(ProtocolError):
        client.request("s", "t", (), timeout_s=1.0, operation="grounding")


def test_image_url_encodes_numpy_rgb():
    url = image_url(np.zeros((2, 2, 3), dtype=np.uint8), "PNG")
    assert url.startswith("data:image/png;base64,")
    jpeg = image_url(np.zeros((2, 2, 3), dtype=np.uint8), "JPEG")
    assert jpeg.startswith("data:image/jpeg;base64,")


def test_aligned_depth_image_requires_a_registered_visualisation():
    depth = np.full((6, 8, 3), 128, dtype=np.uint8)
    observation = _observation(depth_image=depth)
    assert aligned_depth_image(observation) is depth
    with pytest.raises(ObservationError):
        aligned_depth_image(_observation(depth_image=None))
    bad = Observation(
        frame=1,
        timestamp_s=0.0,
        pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        rgb=np.zeros((6, 8, 3), dtype=np.uint8),
        depth_m=np.full((6, 8), 5.0, dtype=np.float32),
        intrinsics=CameraIntrinsics.from_horizontal_fov(10, 10, 90.0),
        camera_matrix=ue_camera_matrix(0.0, 0.0, 0.0, 0.0),
        depth_image=depth,
    )
    with pytest.raises(ObservationError):
        aligned_depth_image(bad)