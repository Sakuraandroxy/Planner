"""分层依赖规则（D1–D5）。

规则来自 `docs/refactor/README.md` 第二节的依赖表：

- D1：`domain/`、`ports/` 不 import 其他层；`planner.errors` 是契约层的第三个模块，谁都可以 import。
- D2：`algorithms/` 不 import `runtime/`、`workflows/`、`assembly/`、`adapters/`、`simulators/`。
- D3：`runtime/` 不 import `workflows/`、`assembly/`、`simulators/`。
- D4：任务包之间零互相 import。
- D5：只有 `assembly/` 同时 import 任务与 `simulators/`。

契约层内部两个包互通：`ports/` 的 Protocol 用 `domain/` 的类型标注。`01-contracts.md` §3.1 列的是
"不 import `algorithms/`、`runtime/`、`workflows/`、`assembly/`、`adapters/`、`simulator/`、`simulators/`"，
`domain/` 不在其中，所以 `ports/` → `domain/` 不算越层。

检查用 AST：相对 import 按文件所在包解析成绝对模块名，再按前两段判断属于哪一层。
每个检查器都收 `root` 参数，负例在 `tmp_path` 里造违规源码，断言检查器抓得到。
"""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

#: `planner/` 下的层目录，顺序即依赖由下到上。
LAYERS = (
    "domain",
    "ports",
    "algorithms",
    "runtime",
    "workflows",
    "assembly",
    "adapters",
    "simulators",
    "simulator",
)

#: 契约层：两个包算一层，内部互通。
CONTRACT = frozenset({"domain", "ports"})

#: 任务包目录名，`planner/workflows/` 下的子包。
TASK_PACKAGES = ("basic_motion", "free_navigation", "visual_goal_navigation")


def _files_under(root: Path, *parts: str):
    for path in sorted((root / "planner").joinpath(*parts).rglob("*.py")):
        if "__pycache__" not in path.parts:
            yield path


def _imports(path: Path, root: Path) -> set[str]:
    """绝对模块名集合，相对 import 按文件所在包解析。"""
    package = list(path.relative_to(root).parent.parts)
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


def _layer_of(module: str) -> str | None:
    """模块名属于哪一层；`planner.errors` 这类层外模块返回 ``None``。"""
    parts = module.split(".")
    if len(parts) < 2 or parts[0] != "planner":
        return None
    return parts[1] if parts[1] in LAYERS else None


def _layer_offenders(root: Path, sources: tuple[str, ...], forbidden: frozenset[str]) -> list[tuple[str, str]]:
    """``sources`` 里 import 了 ``forbidden`` 中任意一层的 (文件, 模块) 对。"""
    offenders = set()
    for layer in sources:
        for path in _files_under(root, layer):
            for module in _imports(path, root):
                if _layer_of(module) in forbidden:
                    offenders.add((str(path.relative_to(root)), module))
    return sorted(offenders)


def _task_package_offenders(root: Path) -> list[tuple[str, str]]:
    """一个任务包 import 另一个任务包的 (文件, 模块) 对。"""
    offenders = set()
    for package in TASK_PACKAGES:
        for path in _files_under(root, "workflows", package):
            for module in _imports(path, root):
                parts = module.split(".")
                if len(parts) < 3 or parts[:2] != ["planner", "workflows"]:
                    continue
                if parts[2] in TASK_PACKAGES and parts[2] != package:
                    offenders.add((str(path.relative_to(root)), module))
    return sorted(offenders)


def _dual_layer_offenders(root: Path) -> dict[str, dict[str, list[str]]]:
    """同时 import 任务与 `simulators/` 且不在 `assembly/` 的文件。"""
    offenders = {}
    for path in _files_under(root):
        modules = _imports(path, root)
        tasks = sorted(name for name in modules if name.split(".")[:2] == ["planner", "workflows"])
        simulators = sorted(name for name in modules if name.split(".")[:2] == ["planner", "simulators"])
        if not tasks or not simulators:
            continue
        relpath = str(path.relative_to(root))
        if relpath.startswith("planner/assembly/"):
            continue
        offenders[relpath] = {"tasks": tasks, "simulators": simulators}
    return offenders


def test_contract_layer_never_imports_an_upper_layer():
    offenders = _layer_offenders(ROOT, tuple(sorted(CONTRACT)), frozenset(LAYERS) - CONTRACT)
    assert not offenders, (
        "契约层 import 了上层；类型与 Protocol 不能依赖实现，"
        f"要的数据由调用方传进来：{offenders}"
    )


def test_algorithms_never_imports_an_upper_layer():
    offenders = _layer_offenders(
        ROOT,
        ("algorithms",),
        frozenset({"runtime", "workflows", "assembly", "adapters", "simulators"}),
    )
    assert not offenders, (
        "planner/algorithms/ 依赖了上层包；把纯计算留在计算层，"
        f"调用方持有执行层对象：{offenders}"
    )


def test_runtime_never_imports_tasks_or_wiring():
    offenders = _layer_offenders(ROOT, ("runtime",), frozenset({"workflows", "assembly", "simulators"}))
    assert not offenders, (
        "执行层 import 了任务层、装配层或引擎实现；执行层不认识具体任务，"
        f"也不做接线：{offenders}"
    )


def test_task_packages_never_import_each_other():
    offenders = _task_package_offenders(ROOT)
    assert not offenders, (
        "任务包之间互相 import 了；共用部分放 planner/workflows/ 顶层（阶段 5 后是 common/），"
        f"任务包之间只通过装配层连线：{offenders}"
    )


def test_only_assembly_imports_tasks_with_simulators():
    offenders = _dual_layer_offenders(ROOT)
    assert not offenders, (
        "任务与引擎实现的唯一交集必须在 planner/assembly/（D5）；"
        f"这些文件同时认识两边：{offenders}"
    )
    # 交集不能是空的：装配层今天就在任务侧，清空说明规则失效而不是代码变干净。
    task_importers = {
        str(path.relative_to(ROOT))
        for path in _files_under(ROOT)
        if any(name.split(".")[:2] == ["planner", "workflows"] for name in _imports(path, ROOT))
    }
    assert any(relpath.startswith("planner/assembly/") for relpath in task_importers), (
        f"没有任何 assembly/ 文件 import 任务包，交集规则退化成空断言：{sorted(task_importers)}"
    )


def test_contract_checker_catches_a_violation(tmp_path):
    """负例：契约层 import 执行层。"""
    _write(tmp_path, "planner/domain/pose.py", "from planner.runtime import ExecutionService\n")
    assert _layer_offenders(tmp_path, ("domain",), frozenset(LAYERS) - CONTRACT) == [
        ("planner/domain/pose.py", "planner.runtime")
    ]


def test_algorithms_checker_catches_a_relative_import(tmp_path):
    """负例：计算层用相对 import 拿执行层。"""
    _write(tmp_path, "planner/algorithms/limits.py", "from ..runtime import ExecutionService\n")
    assert _layer_offenders(tmp_path, ("algorithms",), frozenset({"runtime"})) == [
        ("planner/algorithms/limits.py", "planner.runtime")
    ]


def test_runtime_checker_catches_a_violation(tmp_path):
    """负例：执行层 import 任务层。"""
    _write(tmp_path, "planner/runtime/loop.py", "from planner.workflows.basic_motion import TASK_KIND\n")
    assert _layer_offenders(tmp_path, ("runtime",), frozenset({"workflows"})) == [
        ("planner/runtime/loop.py", "planner.workflows.basic_motion")
    ]


def test_task_package_checker_catches_a_violation(tmp_path):
    """负例：目标接近任务包 import 数值动作任务包。"""
    _write(
        tmp_path,
        "planner/workflows/visual_goal_navigation/policy.py",
        "from planner.workflows.basic_motion import TASK_KIND\n",
    )
    assert _task_package_offenders(tmp_path) == [
        ("planner/workflows/visual_goal_navigation/policy.py", "planner.workflows.basic_motion")
    ]


def test_dual_layer_checker_catches_a_violation(tmp_path):
    """负例：适配层同时 import 任务队与引擎实现；装配层同样写法不算。"""
    _write(
        tmp_path,
        "planner/adapters/http/routes.py",
        "from planner.workflows import BasicMotionWorkflow\nfrom planner.simulators.airsim import runtime\n",
    )
    _write(
        tmp_path,
        "planner/assembly/mission_assembly.py",
        "from planner.workflows import BasicMotionWorkflow\nfrom planner.simulators.airsim import runtime\n",
    )
    assert set(_dual_layer_offenders(tmp_path)) == {"planner/adapters/http/routes.py"}


def _write(root: Path, relpath: str, text: str) -> None:
    path = root / relpath
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
