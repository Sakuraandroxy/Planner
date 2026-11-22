"""跨层 import 只指向包入口（D10，裁决 R19a、R24）。

层入口是 `planner/<层>/__init__.py`；二级入口是 `planner.<层>.<子包>/__init__.py`，
例如 `planner.algorithms.motion`、`planner.workflows.<任务包>`。指向普通模块文件的跨层
import 一律算违规，`planner.simulator.ports`、`planner.runtime.motion.loop` 都是这种。

`planner.errors` 是契约层的第三个入口，它是单文件不是包，单列在 `ENTRY_MODULES` 里放行。

规则出处：`docs/refactor/README.md` 第五节「层间解耦靠三条机制」第一条与第十节 R19、R24。
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

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

#: 契约层第三个入口，单文件不构成包。
ENTRY_MODULES = frozenset({"planner.errors"})

#: 各源层当前仍有的跨层深层 import 条数，阶段 1g 收口后清零。
#: 数字是 `pytest -k deep` 之外的一次性统计：assembly 16、simulators 4、workflows 1。
DEEP_IMPORTS = {"assembly": 16, "simulators": 4, "workflows": 1}


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


def _is_entry(root: Path, parts: list[str]) -> bool:
    """`planner.a.b.c` 这类目标是否是某个包的 `__init__.py`。"""
    if len(parts) < 3:
        return True  # `planner.<层>` 本身是层入口
    return (root / "planner").joinpath(*parts[1:]).joinpath("__init__.py").exists()


def _deep_import_offenders(root: Path, source: str) -> list[tuple[str, str]]:
    """``source`` 层里指向另一个层的普通模块文件的 (文件, 模块) 对。"""
    offenders = set()
    for path in _files_under(root, source):
        for module in _imports(path, root):
            parts = module.split(".")
            if len(parts) < 2 or parts[0] != "planner":
                continue
            if parts[1] not in LAYERS or parts[1] == source:
                continue
            if module in ENTRY_MODULES or _is_entry(root, parts):
                continue
            offenders.add((str(path.relative_to(root)), module))
    return sorted(offenders)


def _params():
    params = []
    for layer in LAYERS:
        if layer in DEEP_IMPORTS:
            params.append(
                pytest.param(
                    layer,
                    marks=pytest.mark.xfail(
                        strict=True,
                        reason=f"阶段 1g 未收口：{layer}/ 仍有 {DEEP_IMPORTS[layer]} 条跨层深层 import",
                    ),
                )
            )
        else:
            params.append(pytest.param(layer))
    return params


@pytest.mark.parametrize("layer", _params())
def test_cross_layer_imports_point_at_a_package_entry(layer):
    offenders = _deep_import_offenders(ROOT, layer)
    assert not offenders, (
        f"planner/{layer}/ 里这些 import 指向了普通模块文件；跨层只写层入口或二级包入口，"
        f"深层路径留给同层内部：{offenders}"
    )


def test_deep_import_checker_flags_a_module_file(tmp_path):
    """负例：跨层 import 指向普通模块文件。"""
    (tmp_path / "planner/assembly").mkdir(parents=True)
    (tmp_path / "planner/assembly/thing.py").write_text(
        "from planner.runtime.motion.loop import ControlLoop\n", encoding="utf-8"
    )
    assert _deep_import_offenders(tmp_path, "assembly") == [
        ("planner/assembly/thing.py", "planner.runtime.motion.loop")
    ]


def test_deep_import_checker_allows_layer_and_second_level_entries(tmp_path):
    """层入口、二级包入口与 `planner.errors` 都放行。"""
    (tmp_path / "planner/workflows/basic_motion").mkdir(parents=True)
    (tmp_path / "planner/workflows/basic_motion/__init__.py").write_text("", encoding="utf-8")
    (tmp_path / "planner/assembly").mkdir(parents=True)
    (tmp_path / "planner/assembly/thing.py").write_text(
        "from planner.runtime import ExecutionService\n"
        "from planner.workflows.basic_motion import TASK_KIND\n"
        "from planner.errors import ProtocolError\n",
        encoding="utf-8",
    )
    assert _deep_import_offenders(tmp_path, "assembly") == []
