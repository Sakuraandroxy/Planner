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
    camera: str = "front"                     # front | down | none


class BaseDetector(ABC):
    """检测器接口。"""

    @abstractmethod
    def detect(self, image, caption: str,
               depth_meters=None, camera_name: str = "front") -> DetectionResult:
        """检测目标物体。"""
        ...

    def detect_all(self, image, caption: str,
                   depth_meters=None, camera_name: str = "front") -> List[DetectionResult]:
        result = self.detect(image, caption, depth_meters, camera_name=camera_name)
        return [result] if result.visible else []

    def detect_with_fallback(
        self,
        front_image,
        down_image,
        caption: str,
        front_depth_meters=None,
        down_depth_meters=None,
    ) -> DetectionResult:
        """前视优先，前视未命中时再检测下视。"""
        front_result = self.detect(
            front_image,
            caption,
            front_depth_meters,
            camera_name="front",
        )
        if front_result.visible:
            front_result.camera = "front"
            return front_result

        if down_image is not None:
            down_result = self.detect(
                down_image,
                caption,
                down_depth_meters,
                camera_name="down",
            )
            if down_result.visible:
                down_result.camera = "down"
                return down_result

        return DetectionResult(visible=False, camera="none")
