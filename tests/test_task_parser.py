import json

import pytest

import planner.adapters.task_parser.openai_task_parser as parser_module
from planner.adapters.task_parser import OpenAITaskParser
from planner.adapters.task_parser.prompt import (
    TASK_PARSER_SYSTEM_PROMPT,
    UAV_TASK_PARSER_SYSTEM_PROMPT,
    task_parser_system_prompt,
    user_prompt,
)
from planner.adapters.task_parser.response_schema import (
    parse_json_object,
    validate_mission_envelope,
)
from planner.adapters.task_parser.task_factory import mission_from_dict
from planner.adapters.task_parser.tasks import (
    TaskCodec,
    builtin_task_codecs,
)
from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.free_navigation import FreeNavigationParameters
from planner.domain.mission import TaskKind
from planner.errors import ProtocolError

json_module = json

ALL_KINDS = {
    TaskKind.BASIC_MOTION,
    TaskKind.VISUAL_GOAL_NAVIGATION,
    TaskKind.FREE_NAVIGATION,
}


def _basic(stage_id="stage_1", forward_m=5.0):
    return {
        "stage_id": stage_id,
        "kind": "basic_motion",
        "parameters": {
            "instruction": "Drive forward 5 metres. 向前直行 5 米。",
            "forward_m": forward_m,
            "yaw_right_deg": 0,
        },
    }


def _visual(stage_id="stage_2"):
    return {
        "stage_id": stage_id,
        "kind": "visual_goal_navigation",
        "parameters": {
            "instruction": "Drive to the red car. 开到红色汽车那里。",
            "target_description": "the red car",
            "arrival_relation": "near",
            "keep_target_in_view": True,
        },
    }


def _free(stage_id="stage_3"):
    return {
        "stage_id": stage_id,
        "kind": "free_navigation",
        "parameters": {
            "instruction": "Keep driving along the road. 沿道路一直前行。",
        },
    }


def _plan(*stages):
    return {"protocol_version": "mission_plan_v1", "stages": list(stages)}


def test_system_prompt_lists_allowed_kinds_and_rules():
    prompt = task_parser_system_prompt(ALL_KINDS)
    assert TASK_PARSER_SYSTEM_PROMPT in prompt
    assert "Allowed kinds for this runtime: " in prompt
    assert "basic_motion" in prompt
    assert "visual_goal_navigation" in prompt
    assert "free_navigation" in prompt
    assert "Parameters: instruction, forward_m, yaw_right_deg" in prompt
    assert "Parameters: instruction only" in prompt
    assert "must be the final stage" in prompt
    assert "forward-facing ground-vehicle camera" in prompt
    assert "ground-vehicle mission parser" in prompt


def test_aerial_system_prompt_uses_the_uav_rules():
    prompt = task_parser_system_prompt(ALL_KINDS, aerial=True)
    assert UAV_TASK_PARSER_SYSTEM_PROMPT in prompt
    assert "UAV mission parser" in prompt
    assert "ground-vehicle" not in prompt
    assert "fly forward 5m" in prompt
    assert "cannot strafe sideways or change altitude" in prompt
    assert "fly near/beside it" in prompt
    assert "Keep flying along the road" in prompt


def test_system_prompt_rejects_kinds_without_a_codec():
    only_basic = tuple(
        codec
        for codec in builtin_task_codecs()
        if codec.kind is TaskKind.BASIC_MOTION
    )
    with pytest.raises(ValueError):
        task_parser_system_prompt(
            {TaskKind.VISUAL_GOAL_NAVIGATION}, codecs=only_basic
        )


def test_user_prompt_preserves_the_instruction():
    text = user_prompt("  向前直行 5 米  ")
    assert "User instruction:\n向前直行 5 米" in text
    assert "Simplified Chinese translation" in text


def test_parse_json_object_accepts_fenced_json():
    assert parse_json_object('```json\n{"a": 1}\n```') == {"a": 1}


@pytest.mark.parametrize(
    "text",
    [
        '{"a": 1, "a": 2}',
        '{"a": NaN}',
        '{"a": Infinity}',
        "[1, 2]",
        "not json",
    ],
)
def test_parse_json_object_rejects_malformed_responses(text):
    with pytest.raises(ProtocolError):
        parse_json_object(text)


def test_envelope_rejects_the_whole_mission_explicitly():
    with pytest.raises(ProtocolError) as error:
        validate_mission_envelope(
            {
                "protocol_version": "mission_plan_v1",
                "stages": [],
                "error": "cannot fly",
            }
        )
    assert "mission rejected" in str(error.value)
    assert "cannot fly" in str(error.value)


@pytest.mark.parametrize(
    "data",
    [
        {"protocol_version": "mission_plan_v1", "stages": [_basic()], "error": "x"},
        {"protocol_version": "mission_plan_v1", "stages": [], "error": " "},
        {"protocol_version": "v2", "stages": [_basic()]},
        {"protocol_version": "mission_plan_v1", "stages": []},
        {"protocol_version": "mission_plan_v1", "stages": [_basic()], "extra": 1},
        {"protocol_version": "mission_plan_v1", "stages": "nope"},
    ],
)
def test_invalid_envelopes_are_rejected(data):
    with pytest.raises(ProtocolError):
        validate_mission_envelope(data)


def test_mission_factory_preserves_stage_order_and_types():
    plan = mission_from_dict(_plan(_basic(), _visual(), _free()))
    assert [stage.stage_id for stage in plan.stages] == [
        "stage_1",
        "stage_2",
        "stage_3",
    ]
    assert [stage.kind for stage in plan.stages] == [
        TaskKind.BASIC_MOTION,
        TaskKind.VISUAL_GOAL_NAVIGATION,
        TaskKind.FREE_NAVIGATION,
    ]
    assert isinstance(plan.stages[0].parameters, BasicMotionParameters)
    assert plan.stages[0].parameters.forward_m == 5.0
    assert isinstance(plan.stages[2].parameters, FreeNavigationParameters)
    assert plan.stages[2].parameters.instruction.startswith("Keep driving")


@pytest.mark.parametrize(
    "parameters",
    [
        {},
        {"instruction": ""},
        {"instruction": "   "},
        {"instruction": "follow the road", "forward_m": 5.0},
        {"instruction": "follow the road", "target_description": "a car"},
    ],
)
def test_free_navigation_codec_is_strict(parameters):
    with pytest.raises(ProtocolError):
        mission_from_dict(
            _plan(
                {
                    "stage_id": "stage_1",
                    "kind": "free_navigation",
                    "parameters": parameters,
                }
            )
        )


@pytest.mark.parametrize(
    "data",
    [
        _plan(_basic(), _basic()),
        _plan({"stage_id": "", "kind": "basic_motion", "parameters": {}}),
        _plan({**_basic(), "extra": 1}),
        _plan({"stage_id": "s", "kind": "orbit", "parameters": {}}),
        _plan({"stage_id": "s", "kind": "basic_motion", "parameters": {}}),
        _plan(
            {
                "stage_id": "s",
                "kind": "basic_motion",
                "parameters": {
                    "instruction": "turn",
                    "forward_m": 0,
                    "yaw_right_deg": 0,
                },
            }
        ),
        _plan(
            {
                "stage_id": "s",
                "kind": "basic_motion",
                "parameters": {
                    "instruction": "turn",
                    "forward_m": 0,
                    "yaw_right_deg": 90,
                    "up_m": 0,
                },
            }
        ),
    ],
)
def test_mission_factory_is_strict(data):
    with pytest.raises(ProtocolError):
        mission_from_dict(data)


def test_mission_factory_only_admits_injected_codecs():
    basic_codec = next(
        codec
        for codec in builtin_task_codecs()
        if codec.kind is TaskKind.BASIC_MOTION
    )
    with pytest.raises(ProtocolError):
        mission_from_dict(
            _plan(_visual()), codecs=(basic_codec,)
        )


def _parser(**kwargs):
    return OpenAITaskParser(
        "http://model/v1",
        "qwen-test",
        "",
        12.0,
        ALL_KINDS,
        max_completion_tokens=128,
        reasoning_effort="none",
        **kwargs,
    )


class FakeResponse:
    def __init__(self, payload):
        self.payload = payload

    def raise_for_status(self):
        pass

    def json(self):
        return self.payload


def _response(content, *, finish_reason="stop", refusal=None):
    return FakeResponse(
        {
            "choices": [
                {
                    "finish_reason": finish_reason,
                    "message": {"content": content, "refusal": refusal},
                }
            ]
        }
    )


def _anthropic_response(content, *, stop_reason="end_turn"):
    return FakeResponse({
        "content": [{"type": "text", "text": content}],
        "stop_reason": stop_reason,
    })


def test_openai_parser_sends_the_mission_prompt(monkeypatch):
    calls = []

    def fake_post(url, *, headers, json, timeout):
        calls.append(
            {"url": url, "headers": headers, "json": json, "timeout": timeout}
        )
        return _response(json_module.dumps(_plan(_basic(), _visual())))

    monkeypatch.setattr(parser_module.httpx, "post", fake_post)
    plan = _parser().parse("向前直行 5 米，然后开到红色汽车那里")

    assert [stage.kind for stage in plan.stages] == [
        TaskKind.BASIC_MOTION,
        TaskKind.VISUAL_GOAL_NAVIGATION,
    ]
    call = calls[0]
    assert call["url"] == "http://model/v1/chat/completions"
    assert call["headers"] == {}
    assert call["timeout"] == 12.0
    body = call["json"]
    assert body["model"] == "qwen-test"
    assert body["max_completion_tokens"] == 128
    assert body["reasoning_effort"] == "none"
    assert [item["role"] for item in body["messages"]] == ["system", "user"]
    assert "Allowed kinds for this runtime" in body["messages"][0]["content"]
    assert "向前直行 5 米" in body["messages"][1]["content"]


def test_openai_parser_sends_the_bearer_token(monkeypatch):
    calls = []
    monkeypatch.setattr(
        parser_module.httpx,
        "post",
        lambda url, *, headers, json, timeout: (
            calls.append(headers)
            or _response(json_module.dumps(_plan(_basic())))
        ),
    )
    OpenAITaskParser(
        "http://model/v1",
        "qwen-test",
        "secret",
        5.0,
        ALL_KINDS,
        max_completion_tokens=64,
    ).parse("drive forward 5 metres")
    assert calls == [{"Authorization": "Bearer secret"}]


@pytest.mark.parametrize(
    "response",
    [
        _response(json_module.dumps(_plan(_basic())), finish_reason="length"),
        _response("", refusal="cannot comply"),
        _response("   "),
        _response("not json"),
    ],
)
def test_openai_parser_rejects_invalid_responses(monkeypatch, response):
    monkeypatch.setattr(
        parser_module.httpx,
        "post",
        lambda url, *, headers, json, timeout: response,
    )
    with pytest.raises(ProtocolError):
        _parser().parse("drive forward 5 metres")


def test_anthropic_parser_sends_the_messages_api(monkeypatch):
    calls = []

    def fake_post(url, *, headers, json, timeout):
        calls.append(
            {"url": url, "headers": headers, "json": json, "timeout": timeout}
        )
        return _anthropic_response(json_module.dumps(_plan(_basic())))

    monkeypatch.setattr(parser_module.httpx, "post", fake_post)
    parser = OpenAITaskParser(
        "https://api.anthropic.com",
        "claude-test",
        "secret",
        12.0,
        ALL_KINDS,
        max_completion_tokens=128,
        reasoning_effort="none",
        request_format="anthropic",
    )
    plan = parser.parse("drive forward 5 metres")

    assert [stage.kind for stage in plan.stages] == [TaskKind.BASIC_MOTION]
    call = calls[0]
    assert call["url"] == "https://api.anthropic.com/v1/messages"
    assert call["headers"] == {
        "content-type": "application/json",
        "anthropic-version": "2023-06-01",
        "x-api-key": "secret",
    }
    body = call["json"]
    assert body["model"] == "claude-test"
    assert body["max_tokens"] == 128
    assert body["temperature"] == 0.0
    assert "max_completion_tokens" not in body
    assert body["system"].startswith(TASK_PARSER_SYSTEM_PROMPT[:20])
    assert body["messages"][0]["role"] == "user"
    assert "drive forward 5 metres" in body["messages"][0]["content"]


@pytest.mark.parametrize(
    "response",
    [
        _anthropic_response("not json", stop_reason="max_tokens"),
        _anthropic_response("not json", stop_reason="refusal"),
        FakeResponse({"content": [{"type": "thinking", "thinking": "x"}],
                      "stop_reason": "end_turn"}),
    ],
)
def test_anthropic_parser_rejects_invalid_responses(monkeypatch, response):
    monkeypatch.setattr(
        parser_module.httpx,
        "post",
        lambda url, *, headers, json, timeout: response,
    )
    with pytest.raises(ProtocolError):
        OpenAITaskParser(
            "https://api.anthropic.com",
            "claude-test",
            "secret",
            5.0,
            ALL_KINDS,
            max_completion_tokens=64,
            reasoning_effort="none",
            request_format="anthropic",
        ).parse("drive forward 5 metres")