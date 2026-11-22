"""Workflow 返回值的准入校验：缺证据、stage 不符、确认与依据矛盾。"""
from planner.domain import MissionStage, StageResult, StageSummary

UNCONFIRMED = (
    "workflow returned no confirmed stage completion evidence or mismatched stage ID"
)


def require_confirmed(result, stage: MissionStage) -> StageResult:
    """Return ``result`` when it carries confirmed evidence for ``stage``.

    三类判失败：返回值不是 ``StageResult``、``stage_id`` 与阶段不符、成功却没有
    确认依据。判失败时改写成 ``invalid_result`` 的失败结果，阶段 ID 取 ``stage``。
    """
    if (
        not isinstance(result, StageResult)
        or result.stage_id != stage.stage_id
        or (
            result.success
            and (result.summary is None or not result.summary.completion_confirmed)
        )
    ):
        return StageResult(
            stage.stage_id,
            False,
            error=UNCONFIRMED,
            summary=StageSummary("invalid_result", UNCONFIRMED),
        )
    return result
