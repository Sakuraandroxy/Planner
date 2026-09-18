from planner.application.workflow_router import WorkflowRouter
from planner.domain.mission import MissionPlan
from planner.domain.result import MissionResult, StageResult


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

