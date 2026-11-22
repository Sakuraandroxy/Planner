"""Layer rule for ``planner/algorithms/``.

``algorithms/`` 是计算层：纯函数与规划算法，只依赖 ``planner.domain``、
``planner.ports``、``planner.errors``。它不向上依赖执行层与编排层的包，
否则执行层的时间、线程、模型调用会渗进算法。

检查用 AST，相对 import 按文件所在包解析成绝对模块名。
"""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ALGORITHMS = ROOT / "planner/algorithms"

#: 计算层不得 import 的上层包。
FORBIDDEN_TARGETS = (
    "planner.runtime",
    "planner.workflows",
    "planner.assembly",
    "planner.adapters",
    "planner.simulators",
)

#: 既有债，不是计算层该有的依赖：算法模块直接持有执行层的服务对象。
#: 阶段 5 迁走后删除 —— 这两个文件随任务判定搬进
#: ``planner/workflows/visual_goal_navigation/policy.py``，依赖方向届时自然消失。
LAYERING_EXCEPTIONS = frozenset(
    {
        ("planner/algorithms/targeting/visual_goal_refresh.py", "planner.runtime.background_operation"),
        ("planner/algorithms/targeting/visual_goal_refresh.py", "planner.runtime.observation_snapshot"),
        ("planner/algorithms/completion/visual_goal_verification.py", "planner.runtime.model_calls"),
    }
)


def _imports(path: Path) -> set[str]:
    """绝对模块名集合，相对 import 按文件所在包解析。"""
    package = list(path.relative_to(ROOT).parent.parts)
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            found.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                base = package[: len(package) - (node.level - 1)]
                prefix = ".".join(base)
                found.add(".".join(part for part in (prefix, node.module) if part))
            elif node.module:
                found.add(node.module)
    return found


def test_domain_ports_never_import_another_layer():
    """D1：契约层只定义类型，不 import 任何其他层（``planner.errors`` 除外）。"""
    offenders = set()
    for package in ("domain", "ports"):
        for path in sorted((ROOT / "planner" / package).rglob("*.py")):
            if "__pycache__" in path.parts:
                continue
            for module in _imports(path):
                if not module.startswith("planner."):
                    continue
                if module.split(".")[1] in ("domain", "ports", "errors"):
                    continue
                offenders.add((str(path.relative_to(ROOT)), module))
    assert not offenders, (
        "planner/domain 与 planner/ports 依赖了其他层："
        f"{sorted(offenders)}"
    )


def test_motion_and_engine_capability_types_stay_in_their_own_module():
    """``MotionCapability`` 谈载具运动学，``Capabilities`` 谈引擎控件。"""
    definitions = {
        "MotionCapability": set(),
        "Capabilities": set(),
    }
    for path in sorted((ROOT / "planner").rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in tree.body:
            if isinstance(node, ast.ClassDef) and node.name in definitions:
                definitions[node.name].add(str(path.relative_to(ROOT)))
    assert definitions["MotionCapability"] == {"planner/domain/motion.py"}
    assert definitions["Capabilities"] == {"planner/domain/engine_family.py"}


def test_algorithms_never_import_an_upper_layer():
    offenders = set()
    for path in sorted(ALGORITHMS.rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        relpath = str(path.relative_to(ROOT))
        for module in _imports(path):
            if not any(module == target or module.startswith(f"{target}.") for target in FORBIDDEN_TARGETS):
                continue
            if (relpath, module) in LAYERING_EXCEPTIONS:
                continue
            offenders.add((relpath, module))
    assert not offenders, (
        "planner/algorithms/ 依赖了上层包；把纯计算留在计算层，"
        f"调用方持有执行层对象：{sorted(offenders)}"
    )
