"""Task parser function registry."""

from config import cfg
from agent.functions.common.config_access import first_value, function_section
from agent.functions.task_parser.base import BaseTaskParser, TaskStage

_TASK_PARSER_REGISTRY = {}


def register_task_parser(name):
    def wrapper(cls):
        _TASK_PARSER_REGISTRY[name] = cls
        return cls

    return wrapper


def build_task_parser(vlm=None):
    from agent.functions.task_parser.vlm_task_parser import TaskParser, parse_task_parser_response  # noqa: F401

    parser_cfg = function_section(cfg, "TASK_PARSER")
    ag = cfg.get("AGENT", {}) or {}
    name = first_value(parser_cfg.get("NAME"), ag.get("TASK_PARSER"), default="vlm_parser")
    if name not in _TASK_PARSER_REGISTRY:
        raise KeyError(f"Unknown task parser [{name}], registered={list(_TASK_PARSER_REGISTRY.keys())}")
    return _TASK_PARSER_REGISTRY[name](vlm)


__all__ = ["BaseTaskParser", "TaskStage", "build_task_parser", "register_task_parser"]
