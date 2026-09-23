from planner.application.workflow_router import WorkflowRouter
from planner.domain.mission import MissionPlan
from planner.domain.result import MissionResult, StageResult

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

    def run(self, plan: MissionPlan) -> MissionResult:
        results: list[StageResult[object]] = []
        for stage in plan.stages:
            result = self.router.workflow_for(stage).run(stage)
            results.append(result)
            if not result.success:
                return MissionResult(False, tuple(results), error=result.error)
        return MissionResult(True, tuple(results))

