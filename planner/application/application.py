from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.domain.result import MissionResult
from planner.ports.task_parser import TaskParser

'''一次任务执行的应用层入口。它把任务解析器、计划校验器和任务运行器串起来，按“解析 → 校验 → 执行”的顺序处理用户指令'''
class PlannerApplication:
    def __init__(self, task_parser: TaskParser, validator: MissionValidator, runner: MissionRunner):
        self.task_parser = task_parser
        self.validator = validator
        self.runner = runner

    def execute(self, instruction: str) -> MissionResult:
        plan = self.task_parser.parse(instruction)
        self.validator.validate(plan)
        return self.runner.run(plan)

