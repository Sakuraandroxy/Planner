"""规划器 —— 注册表模式，由 config 决定用哪个实现。

扩展方法:
    1. 新建 agent/planner/my_planner.py
    2. 在类上方加 @register_planner("my_name")
    3. 在 build_planner() 中添加 import 触发注册
    4. config 的 PLANNER: my_name 即可使用

已注册:
    qwen_planner  → QwenPlanner
    prompt_planner → PromptPlanner
"""
from config import cfg
from agent.planner.base import BasePlanner, TrajectoryResult

_PLANNER_REGISTRY = {}

def register_planner(name):
    """装饰器：将规划器类注册到全局注册表。"""
    def wrapper(cls):
        _PLANNER_REGISTRY[name] = cls
        return cls
    return wrapper

# QwenPlanner 即为默认 Planner（向后兼容）
Planner = None  # 懒加载，首次 build_planner() 时设置


def build_planner():
    # 懒加载：import 触发 @register_planner 装饰器自动注册
    from agent.planner.qwen_planner import QwenPlanner  # noqa: F401
    from agent.planner.api_atomic_planner import ApiAtomicPlanner  # noqa: F401
    global Planner
    name = cfg["AGENT"]["PLANNER"]
    if name not in _PLANNER_REGISTRY:
        raise KeyError(f"未知规划器 [{name}]，已注册: {list(_PLANNER_REGISTRY.keys())}")
    if Planner is None:
        Planner = QwenPlanner
    return _PLANNER_REGISTRY[name]()
