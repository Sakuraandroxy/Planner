"""方向估计器 —— 注册表模式，由 config 决定用哪个实现。

扩展方法:
    1. 新建 agent/direction/my_estimator.py
    2. 在类上方加 @register_direction("my_name")
    3. 在 build_direction_estimator() 中添加 import 触发注册
    4. config 的 DIRECTION: my_name 即可使用

已注册:
    three_dg  → ThreeDGDirectionEstimator
"""
from config import cfg
from agent.direction.base import BaseDirectionEstimator

_DIRECTION_REGISTRY = {}

def register_direction(name):
    """装饰器：将方向估计器类注册到全局注册表。"""
    def wrapper(cls):
        _DIRECTION_REGISTRY[name] = cls
        return cls
    return wrapper


def build_direction_estimator():
    # 懒加载：import 触发 @register_direction 装饰器自动注册
    from agent.direction.three_dg_estimator import ThreeDGDirectionEstimator  # noqa: F401
    name = cfg["AGENT"]["DIRECTION"]
    if name not in _DIRECTION_REGISTRY:
        raise KeyError(f"未知方向估计器 [{name}]，已注册: {list(_DIRECTION_REGISTRY.keys())}")
    return _DIRECTION_REGISTRY[name]()
