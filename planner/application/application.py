from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.domain.result import MissionResult
from planner.ports.task_parser import TaskParser


class PlannerApplication:
    def __init__(self, task_parser: TaskParser, validator: MissionValidator, runner: MissionRunner):
        self.task_parser = task_parser
        self.validator = validator
        self.runner = runner

    def execute(self, instruction: str) -> MissionResult:
        plan = self.task_parser.parse(instruction)
        self.validator.validate(plan)
        return self.runner.run(plan)

