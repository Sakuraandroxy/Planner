"""
agent/world_model/base.py — 世界模型抽象基类。

世界模型作用：对 K 条候选轨迹打分，返回最优轨迹下标。
运行在独立主机上，通过 Flask HTTP API 访问。
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class WorldModelResult:
    """世界模型打分结果。"""
    best_index: int = 0                       # 最优轨迹下标
    scores: List[float] = field(default_factory=list)  # 所有候选的分数
    reasoning: str = ""                       # 打分理由（可选）


class BaseWorldModel(ABC):
    """世界模型接口。"""

    @abstractmethod
    def score(self, front_img_b64: str, down_img_b64: str,
              instruction: str, candidates: List[dict]) -> WorldModelResult:
        """对候选轨迹打分，返回最优下标。

        Args:
            front_img_b64: 前视图 JPEG base64
            down_img_b64:  下视图 JPEG base64
            instruction:   英文任务指令
            candidates:    K 条候选，每条为
                           {"actions": [...], "waypoints": [[dx,dy,dz], ...], "reason": "..."}

        Returns:
            WorldModelResult with best_index and scores.
        """
        ...
