"""检测器 —— 注册表模式，由 config 决定用哪个实现。

扩展方法（对标 fastreid.register_detector）:
    1. 新建 agent/detector/my_detector.py
    2. 在类上方加 @register_detector("my_name")
    3. 在 build_detector() 中添加 import 触发注册
    4. config 的 DETECTOR: my_name 即可使用

当前已注册:
    groundingdino  → GroundingDINODetector
"""
from config import cfg
from agent.detector.base import BaseDetector, DetectionResult

_DETECTOR_REGISTRY = {}

def register_detector(name):
    """装饰器：将检测器类注册到全局注册表。"""
    def wrapper(cls):
        _DETECTOR_REGISTRY[name] = cls
        return cls
    return wrapper


def build_detector():
    # 懒加载：import 触发 @register_detector 装饰器自动注册
    from agent.detector.groundingdino_detector import GroundingDINODetector  # noqa: F401
    from agent.detector.vlm_detector import VLMDetector  # noqa: F401
    name = cfg["AGENT"]["DETECTOR"]
    if name not in _DETECTOR_REGISTRY:
        raise KeyError(f"未知检测器 [{name}]，已注册: {list(_DETECTOR_REGISTRY.keys())}")
    return _DETECTOR_REGISTRY[name]()
