"""目标检测器抽象基类——所有检测实现必须继承此接口。"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple
from PIL import Image


@dataclass
class DetectionResult:
    """统一检测结果，所有检测器必须返回此格式。"""
    visible: bool
    bbox: Optional[List[int]] = None          # [x1,y1,x2,y2] 像素坐标
    score: float = 0.0
    label: str = ""
    depth_median: Optional[float] = None
    depth_bbox: Optional[List[int]] = None


class BaseDetector(ABC):
    """检测器接口。"""

    @abstractmethod
    def detect(self, image, caption: str, depth_meters=None) -> DetectionResult:
        """检测目标物体。"""
        ...

    def detect_all(self, image, caption: str,
                   depth_meters=None) -> List[DetectionResult]:
        result = self.detect(image, caption, depth_meters)
        return [result] if result.visible else []