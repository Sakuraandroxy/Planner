"""配置系统 —— 读取并组合 YAML 配置文件，全局可访问。

用法:
    from config import cfg
    detector_name = cfg["AGENT"]["DETECTOR"]  # "groundingdino"

顶层 ``INCLUDES`` 可引用同目录下的其他 YAML 文件。被引用文件会先加载，
当前文件再通过递归字典合并覆盖它，因此运行时配置结构与单文件写法完全一致。
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from typing import Any

import yaml

_CFG = None


def _deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> dict:
    """Return a recursive merge without mutating either input mapping."""
    merged = dict(base)
    for key, value in override.items():
        previous = merged.get(key)
        if isinstance(previous, Mapping) and isinstance(value, Mapping):
            merged[key] = _deep_merge(previous, value)
        else:
            merged[key] = value
    return merged


def _include_paths(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, list) and all(isinstance(item, str) for item in value):
        return list(value)
    raise TypeError("config INCLUDES must be a path string or a list of path strings")


def load_config_file(path: str, *, _stack: tuple[str, ...] = ()) -> dict:
    """Load one YAML config, resolving relative includes and deep-merging them."""
    resolved_path = os.path.realpath(os.path.abspath(path))
    if resolved_path in _stack:
        chain = " -> ".join((*_stack, resolved_path))
        raise ValueError(f"cyclic config include detected: {chain}")

    with open(resolved_path, "r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, Mapping):
        raise TypeError(f"config root must be a mapping: {resolved_path}")

    current = dict(loaded)
    includes = _include_paths(current.pop("INCLUDES", None))
    merged: dict = {}
    next_stack = (*_stack, resolved_path)
    base_dir = os.path.dirname(resolved_path)
    for include in includes:
        include_path = include if os.path.isabs(include) else os.path.join(base_dir, include)
        merged = _deep_merge(merged, load_config_file(include_path, _stack=next_stack))
    return _deep_merge(merged, current)


def get_cfg(path=None):
    global _CFG
    if _CFG is not None:
        return _CFG
    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "default.yaml")
    _CFG = load_config_file(path)
    return _CFG


# 导入时自动加载
cfg = get_cfg()
