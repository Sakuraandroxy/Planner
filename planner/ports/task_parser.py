from typing import Protocol

from planner.domain.mission import MissionPlan

#任务解析器的接口约定，能作为任务解析器使用的对象，必须提供一个 parse() 方法
class TaskParser(Protocol):
    def parse(self, instruction: str) -> MissionPlan: ...

