"""分层依赖规则（D1–D5），九个层目录全部作为来源检查。

规则来自 `docs/refactor/README.md` 第二节的依赖表：每层只允许 import `ALLOWED`
里列出的层，其余一律算越层。`planner.errors` 是契约层的第三个模块，不属于任何层，
谁都可以 import。契约层内部两个包互通：`ports/` 的 Protocol 用 `domain/` 的类型标注。

`ALLOWED` 之外只剩三组跨层依赖边，逐文件列在 `EXCEPTIONS` 并写明理由：

1. 装配层 → `planner.simulator`：取引擎进程协议层的槽位表、`BackendWorker` 与事件枚举，
   拿到的是契约类型而不是引擎实现。
2. `planner.simulator` → 装配层：引擎进程栈反向取装配层的构造入口、能力位与任务结束事件，
   这是全项目唯一一处成环的依赖。
3. `adapters` → `workflows`：完成判定提示词复用任务层的证据校验函数，不复制判定逻辑。

白名单按（源文件, 目标模块）精确匹配，新增文件或新增目标模块都会立刻失败；
白名单条目本身也有测试盯着，指向的边消失或本来就被允许时报红，避免白名单变成死条目。

检查用 AST：相对 import 按文件所在包解析成绝对模块名，再按前两段判断属于哪一层。
每个检查器都收 `root` 参数，负例在 `tmp_path` 里造违规源码，断言检查器抓得到。
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

#: `planner/` 下的层目录。
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

#: 每层允许 import 的层。不在表里的层即禁止，`ALLOWED` 就是完整规则。
ALLOWED: dict[str, frozenset[str]] = {
    # 契约层：只认自己。errors 在层外，不受限。
    "domain": frozenset(),
    "ports": frozenset({"domain"}),
    # 纯计算：不碰执行、任务、装配、适配、引擎。
    "algorithms": frozenset({"domain", "ports"}),
    # 执行层：不认识具体任务，也不做接线。
    "runtime": frozenset({"domain", "ports", "algorithms"}),
    # 任务层：拿执行层的对象，不碰装配、适配与引擎。
    "workflows": frozenset({"domain", "ports", "algorithms", "runtime"}),
    # 适配层：只经装配层入口拿服务状态，反方向不允许，见 EXCEPTIONS。
    "adapters": frozenset({"domain", "ports", "algorithms", "runtime", "assembly"}),
    # 装配层：唯一同时认识任务与适配层的位置，引擎进程协议层另见 EXCEPTIONS。
    "assembly": frozenset(
        {"domain", "ports", "algorithms", "runtime", "workflows", "adapters"}
    ),
    # 引擎进程协议层：只认契约与执行层，反向取装配层的名字见 EXCEPTIONS。
    "simulator": frozenset({"domain", "ports", "algorithms", "runtime"}),
    # 具体引擎实现：认装配层（任务语义由装配层定义）与引擎进程协议层。
    "simulators": frozenset(
        {"domain", "ports", "algorithms", "runtime", "assembly", "simulator"}
    ),
}

#: 跨层依赖例外，键是（源文件, 目标模块前缀），值是这一行依赖存在的原因。
EXCEPTIONS: dict[tuple[str, str], str] = {
    (
        "planner/assembly/requirements.py",
        "planner.simulator",
    ): "装配层读引擎进程协议层的 `SimulatorPorts` 与 `PORT_SLOTS` 推导能力位，读的是契约不是引擎",
    (
        "planner/assembly/reporting.py",
        "planner.simulator",
    ): "报告层把引擎进程的 `WorkerEvent` 落成运行日志，事件类型由协议层定义",
    (
        "planner/assembly/controller/__init__.py",
        "planner.simulator",
    ): "控制器用 `BackendWorker` 协议驱动后台进程，`registry` 是引擎目录的装配入口",
    (
        "planner/assembly/controller/commands.py",
        "planner.simulator",
    ): "控制器把 `TaskSubmission` 与 `WorkerEventKind` 翻成命令与状态迁移",
    (
        "planner/assembly/controller/state.py",
        "planner.simulator",
    ): "状态机读 `WorkerEventKind` 决定 `ServerState`，枚举归协议层所有",
    (
        "planner/assembly/controller/weather.py",
        "planner.simulator",
    ): "天气命令按 `WorkerEventKind` 判定仿真器是否接下请求",
    (
        "planner/simulator/registry.py",
        "planner.assembly",
    ): "槽位到能力位的推导只有装配层有，`capabilities_for` 惰性取需求表",
    (
        "planner/simulator/worker.py",
        "planner.assembly",
    ): "worker 用装配层的 `task_finish_events` 把任务结束翻成引擎事件",
    (
        "planner/simulator/mission_child.py",
        "planner.assembly",
    ): "子进程入口用装配层的 `build_*` 与 `MissionResources` 造资源，再由装配层驱动本层",
    (
        "planner/adapters/task_completion/prompts/visual_goal_completion.py",
        "planner.workflows",
    ): "完成判定提示词复用任务层的 `require_visual_completion_evidence`，不复制判定逻辑",
}

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


def _exception_reason(relpath: str, module: str) -> str | None:
    """``(relpath, module)`` 命中白名单时返回理由。"""
    for (source, prefix), reason in EXCEPTIONS.items():
        if relpath == source and (module == prefix or module.startswith(prefix + ".")):
            return reason
    return None


def _layer_offenders(root: Path, layer: str) -> list[tuple[str, str]]:
    """``layer`` 里 import 了 `ALLOWED` 之外层、且不在白名单里的 (文件, 模块) 对。"""
    offenders = []
    for path in _files_under(root, layer):
        relpath = str(path.relative_to(root))
        for module in sorted(_imports(path, root)):
            target = _layer_of(module)
            if target is None or target == layer or target in ALLOWED[layer]:
                continue
            if _exception_reason(relpath, module) is not None:
                continue
            offenders.append((relpath, module))
    return offenders


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


@pytest.mark.parametrize("layer", LAYERS)
def test_layer_imports_only_layers_it_may(layer):
    offenders = _layer_offenders(ROOT, layer)
    assert not offenders, (
        f"planner/{layer}/ 依赖了不允许的层；规则见 ALLOWED，"
        "确属契约型依赖就进 EXCEPTIONS 并写清理由，"
        f"不要把实现层的名字引进来：{offenders}"
    )


def test_exceptions_are_real_documented_and_needed():
    """白名单每条都指向现存文件、真实 import，且这条边本来是被禁止的。"""
    for (relpath, prefix), reason in EXCEPTIONS.items():
        assert reason.strip(), f"白名单条目缺理由：{relpath} → {prefix}"
        path = ROOT / relpath
        assert path.is_file(), f"白名单指向的文件不存在：{relpath}"
        modules = _imports(path, ROOT)
        assert any(module == prefix or module.startswith(prefix + ".") for module in modules), (
            f"白名单条目已失效，{relpath} 不再 import {prefix}，删掉这一条"
        )
        source = relpath.split("/")[1]
        target = _layer_of(prefix)
        assert target not in ALLOWED[source], (
            f"白名单条目多余：{source} 已经允许 import {target}，"
            f"把 {relpath} → {prefix} 从 EXCEPTIONS 里删掉"
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


@pytest.mark.parametrize("layer", LAYERS)
def test_checker_catches_a_new_dependency(tmp_path, layer):
    """负例：每层新增一条不在白名单里的跨层依赖都必须被抓住。"""
    forbidden = next(
        name for name in LAYERS if name != layer and name not in ALLOWED[layer]
    )
    _write(tmp_path, f"planner/{layer}/probe.py", f"from planner.{forbidden} import Thing\n")
    assert _layer_offenders(tmp_path, layer) == [
        (f"planner/{layer}/probe.py", f"planner.{forbidden}")
    ]


def test_checker_catches_a_relative_import(tmp_path):
    """负例：计算层用相对 import 拿执行层。"""
    _write(tmp_path, "planner/algorithms/limits.py", "from ..runtime import ExecutionService\n")
    assert _layer_offenders(tmp_path, "algorithms") == [
        ("planner/algorithms/limits.py", "planner.runtime")
    ]


def test_checker_catches_the_same_layer_imported_from_a_new_file(tmp_path):
    """负例：白名单只放行列出的文件，同层另一个文件引同一模块仍失败。"""
    _write(
        tmp_path,
        "planner/adapters/http/routes.py",
        "from planner.workflows.visual_goal_navigation import require_visual_completion_evidence\n",
    )
    assert _layer_offenders(tmp_path, "adapters") == [
        (
            "planner/adapters/http/routes.py",
            "planner.workflows.visual_goal_navigation",
        )
    ]


def test_checker_allows_a_whitelisted_dependency(tmp_path):
    """白名单里的那一条照旧通过。"""
    _write(
        tmp_path,
        "planner/simulator/registry.py",
        "from planner.assembly import capabilities_for\n",
    )
    assert _layer_offenders(tmp_path, "simulator") == []


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
