"""方向估计器抽象基类。"""
from abc import ABC, abstractmethod
from typing import List, Optional, Tuple


class BaseDirectionEstimator(ABC):
    """方向估计接口。"""

    @abstractmethod
    def estimate(self, bbox, camera_id: int,
                 image_size: Tuple[int, int]) -> str:
        ...