from copy import deepcopy
from pathlib import Path
from unittest.mock import Mock

import pytest
import yaml

from config import load_config
from planner.adapters.task_parser.openai_task_parser import OpenAITaskParser
from planner.adapters.task_parser.prompt import task_parser_system_prompt
from planner.adapters.task_parser.response_schema import parse_json_object
from planner.adapters.task_parser.task_factory import mission_from_dict
from planner.application.mission_validator import MissionValidator
from planner.domain.mission import TaskKind
from planner.errors import ConfigurationError, ProtocolError


def envelope():
    return {"protocol_version": "mission_plan_v1", "stages": [{
        "stage_id": "s1", "kind": "basic_motion", "parameters": {
            "instruction": "Ascend 12 meters", "forward_m": 0, "right_m": 0,
            "up_m": 12, "yaw_right_deg": 0,
        },
    }]}


@pytest.mark.parametrize("field,value", [("up_m", True), ("forward_m", "1"), ("up_m", None),
    ("up_m", float("nan")), ("up_m", float("inf")), ("instruction", ""),
    ("unknown", 1), ("yaw_right_deg", 181)])
def test_invalid_basic_numeric_protocol(field, value):
    data = envelope()
    data["stages"][0]["parameters"][field] = value
    with pytest.raises(ProtocolError):
        mission_from_dict(data)


@pytest.mark.parametrize("mutation", ["missing", "duplicate_id", "unknown_stage", "unknown_top",
    "legacy_kind", "missing_id", "no_stages", "empty_action"])
def test_strict_plan_schema(mutation):
    data = envelope()
    if mutation == "missing":
        del data["stages"][0]["parameters"]["forward_m"]
    elif mutation == "duplicate_id":
        data["stages"].append(deepcopy(data["stages"][0]))
    elif mutation == "unknown_stage":
        data["stages"][0]["extra"] = 1
    elif mutation == "unknown_top":
        data["extra"] = 1
    elif mutation == "legacy_kind":
        data["stages"][0]["kind"] = "navigation"
    elif mutation == "missing_id":
        del data["stages"][0]["stage_id"]
    elif mutation == "no_stages":
        data["stages"] = []
    elif mutation == "empty_action":
        data["stages"][0]["parameters"]["up_m"] = 0
    with pytest.raises(ProtocolError):
        mission_from_dict(data)


def test_explicit_rejection_is_not_a_partial_plan():
    with pytest.raises(ProtocolError, match="mission rejected.*unsupported"):
        mission_from_dict(parse_json_object(
            '{"protocol_version":"mission_plan_v1","stages":[],"error":"unsupported road-follow task"}'
        ))
    for error in ("unsupported", None, ""):
        data = envelope()
        data["error"] = error
        with pytest.raises(ProtocolError, match="rejection"):
            mission_from_dict(data)


def test_whole_plan_limits_and_capabilities_are_checked():
    data = envelope()
    data["stages"][0]["parameters"]["up_m"] = 31
    with pytest.raises(ProtocolError, match="distance"):
        MissionValidator({TaskKind.BASIC_MOTION}).validate(mission_from_dict(data))
    with pytest.raises(ProtocolError, match="free_space"):
        MissionValidator({TaskKind.BASIC_MOTION}, basic_motion_ready=False).validate(mission_from_dict(envelope()))


def test_prompt_and_client_use_registered_types_without_model_fallback(monkeypatch):
    text = task_parser_system_prompt(set(TaskKind))
    assert "yaw_right_deg" in text and "up_m" in text
    assert "Sequential actions" in text and "simultaneous" in text
    assert "reject the WHOLE mission" in text
    assert "visual_goal_navigation" in text
    assert "Allowed kinds for this runtime: none" in task_parser_system_prompt(set())
    response = Mock()
    import json
    response.json.return_value = {"choices": [{"message": {"content": json.dumps(envelope())}}]}
    post = Mock(return_value=response)
    monkeypatch.setattr("planner.adapters.task_parser.openai_task_parser.requests.post", post)
    parser = OpenAITaskParser("http://invalid/v1", "fake", "no-key", 20, {TaskKind.BASIC_MOTION})
    assert parser.parse("上升12米").stages[0].parameters.up_m == 12
    messages = post.call_args.kwargs["json"]["messages"]
    assert messages[0]["content"] == parser.system_prompt
    assert "上升12米" in messages[1]["content"]


def test_legacy_config_requires_explicit_migration_and_unused_model_needs_no_key(tmp_path, monkeypatch):
    raw = yaml.safe_load(Path("config/base.yaml").read_text(encoding="utf-8"))
    raw["navigation"] = {"max_rounds": 10}
    path = tmp_path/"legacy.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="legacy navigation"):
        load_config(path)
    monkeypatch.delenv("DEEPSEEK_API_KEY", raising=False)
    config = load_config("config/deepseek.example.yaml")
    assert not config.visual_goal.enabled
    assert not config.basic_motion.free_space.confirmed
