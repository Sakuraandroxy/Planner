"""任务解析器抽象基类。"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class TaskStage:
    """统一任务阶段 —— 所有解析器实现必须返回此格式。

    包含 VLM 解析输出的全部字段，避免后续模块反向推导丢失信息。
    """
    index: int
    instruction: str
    mode: str = "target"           # target | detect | action
    target: str = ""               # 英文 target，给检测器用
    relation: str = ""             # "旁边"、"上方" 等
    action: str = ""               # forward/back/left/right/up/down
    value: Optional[float] = None  # action 数值（m / ° / s）
    unit: str = ""                 # m / ° / s
    requires_target: bool = False
    allow_relocalize: bool = False
    completion_condition: str = ""

    @property
    def target_query(self) -> str:
        """检测器可用的英文 target（仅 target/detect 模式有值）。"""
        return self.target if self.mode in ("target", "detect") else ""

    @property
    def is_direct_action(self) -> bool:
        return self.mode == "action" and bool(self.action)


class BaseTaskParser(ABC):
    """任务解析接口。"""

    @abstractmethod
    def parse(self, instruction: str) -> List[TaskStage]:
        ...
