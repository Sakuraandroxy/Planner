import pytest

from planner.adapters.task_parser.task_factory import mission_from_dict
from planner.domain.mission import TaskKind


def test_task_parser_contract_accepts_navigation_only():
    plan = mission_from_dict({
        "protocol_version": "mission_plan_v1",
        "stages": [{
            "stage_id": "stage_1",
            "kind": "navigation",
            "parameters": {"instruction": "Follow the road"},
        }],
    })
    assert plan.stages[0].kind is TaskKind.NAVIGATION


def test_task_parser_contract_rejects_unregistered_kind():
    with pytest.raises(Exception):
        mission_from_dict({
            "protocol_version": "mission_plan_v1",
            "stages": [{"kind": "detect", "parameters": {"instruction": "Find a car"}}],
        })

