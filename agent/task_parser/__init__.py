"""任务解析器注册表 —— YAML 配置驱动。
build_task_parser() 会自动从 config 读取 API URL/Key 并创建 VLM 客户端。

扩展方法:
    1. 新建 agent/task_parser/my_parser.py，继承 BaseTaskParser
    2. 在类上方加 @register_task_parser("my_name")
    3. 在 build_task_parser() 中添加 import 触发注册
    4. config 的 TASK_PARSER: my_name 即可使用

已注册:
    vlm_parser  → TaskParser
"""
from config import cfg
from agent.task_parser.base import BaseTaskParser, TaskStage

_TASK_PARSER_REGISTRY = {}

def register_task_parser(name):
    """装饰器：将任务解析器类注册到全局注册表。"""
    def wrapper(cls):
        _TASK_PARSER_REGISTRY[name] = cls
        return cls
    return wrapper


def build_task_parser(vlm=None):
    """构建任务解析器。
    
    Args:
        vlm: 可选的外部 VLM 客户端。若为 None，TaskParser 内部使用 config 的 API 配置。
    """
    # 懒加载：import 触发 @register_task_parser 装饰器自动注册
    from agent.task_parser.vlm_task_parser import TaskParser, parse_task_parser_response  # noqa: F401
    name = cfg["AGENT"]["TASK_PARSER"]
    if name not in _TASK_PARSER_REGISTRY:
        raise KeyError(f"未知任务解析器 [{name}]，可用: {list(_TASK_PARSER_REGISTRY.keys())}")
    return _TASK_PARSER_REGISTRY[name](vlm)
