from typing import Callable

from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.domain.mission import MissionPlan
from planner.domain.result import MissionResult
from planner.ports.task_parser import TaskParser


# 一次任务执行的应用层入口：解析 → 校验 → 执行。
class PlannerApplication:
    def __init__(
        self,
        task_parser: TaskParser,
        validator: MissionValidator,
        runner: MissionRunner,
        *,
        parse_call: Callable[[str], MissionPlan] | None = None,
    ):
        self.task_parser = task_parser
        self.validator = validator
        self.runner = runner
        self.parse_call = parse_call

    def prepare(self, instruction: str) -> MissionPlan:
        plan = (
            self.parse_call(instruction)
            if self.parse_call is not None
            else self.task_parser.parse(instruction)
        )
        self.validator.validate(plan)
        self.runner.prepare(plan)
        return plan

    def execute_plan(self, plan: MissionPlan) -> MissionResult:
        self.validator.validate(plan)
        self.runner.prepare(plan)
        return self.runner.run(plan)

    def execute(self, instruction: str) -> MissionResult:
        return self.execute_plan(self.prepare(instruction))