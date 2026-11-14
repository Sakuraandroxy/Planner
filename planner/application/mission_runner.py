from planner.application.workflow_router import WorkflowRouter
from planner.domain.mission import MissionPlan
from planner.domain.result import MissionResult, StageResult, StageSummary

'''
MissionRunner 负责按顺序运行一份任务计划中的各个阶段，并汇总结果。
它不自己实现导航或飞行，而是通过 WorkflowRouter 找到每个阶段对应的 workflow

MissionRunner 遍历一个 stage
  → WorkflowRouter 按 stage.kind 找 workflow
  → MissionRunner 调用 workflow.run(stage)
  → 得到阶段结果，继续下一个或遇到失败就停止
'''
class MissionRunner:
    def __init__(self, router: WorkflowRouter):
        self.router = router

    def prepare(self, plan: MissionPlan) -> None:
        """Prepare only workflows used by the plan, without executing a stage."""
        prepared = set()
        for stage in plan.stages:
            if stage.kind not in prepared:
                self.router.workflow_for(stage).prepare()
                prepared.add(stage.kind)

    def run(self, plan: MissionPlan) -> MissionResult:
        results: list[StageResult[object]] = []
        for stage in plan.stages:
            result = self.router.workflow_for(stage).run(stage)
            if (not isinstance(result, StageResult) or result.stage_id != stage.stage_id or
                    (result.success and (result.summary is None or not result.summary.completion_confirmed))):
                reason = "workflow returned no confirmed stage completion evidence or mismatched stage ID"
                result = StageResult(stage.stage_id, False, error=reason,
                                     summary=StageSummary("invalid_result", reason))
            results.append(result)
            if not result.success:
                return MissionResult(False, tuple(results), error=result.error)
        return MissionResult(True, tuple(results))

