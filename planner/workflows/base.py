from typing import Protocol

from planner.domain.mission import MissionStage
from planner.domain.result import StageResult

'''
任何对象只要想作为 Workflow 使用，就必须提供一个 run() 方法。
这个方法接收一个任务阶段 MissionStage，返回该阶段的执行结果 StageResult
'''
class Workflow(Protocol):
    def run(self, stage: MissionStage) -> StageResult[object]: ...

