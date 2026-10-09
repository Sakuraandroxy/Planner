import pytest

from planner.adapters.task_parser.task_factory import mission_from_dict
from planner.domain.mission import TaskKind


def test_task_parser_contract_accepts_typed_basic_motion():
    plan = mission_from_dict({
        "protocol_version": "mission_plan_v1",
        "stages": [{
            "stage_id": "stage_1",
            "kind": "basic_motion",
            "parameters": {"instruction": "Ascend 12m", "forward_m": 0,
                           "right_m": 0, "up_m": 12, "yaw_right_deg": 0},
        }],
    })
    assert plan.stages[0].kind is TaskKind.BASIC_MOTION
    assert plan.stages[0].parameters.up_m == 12


def test_task_parser_contract_rejects_unregistered_kind():
    with pytest.raises(Exception):
        mission_from_dict({
            "protocol_version": "mission_plan_v1",
            "stages": [{"kind": "detect", "parameters": {"instruction": "Find a car"}}],
        })

