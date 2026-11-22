"""每层包入口的 `__all__` 与层文档 A 表的「保留」名单逐名一致（裁决 R21、R24）。

名单来源章节：

- `algorithms` ← `docs/refactor/02-algorithms.md` §8.1.1「层入口（11）」。
- `runtime` ← `docs/refactor/03-runtime.md` §八「A 对外名字逐条」。处置为「合并」的两行
  （`ModelCallService`、`BackgroundOperation`）在依据列给出新名 `BackgroundCall`，
  表里没有独立行，手工补进期望集，见 `EXTRA_EXPORTS`。
- `workflows` ← `docs/refactor/04-workflows.md` §8「表 A 对外名字逐条」。
- `assembly` ← `docs/refactor/05-assembly.md` §8.1「表 A 对外名字逐条」，处置列为「新增对外」的
  `build_resources` 一起算。
- `domain`、`ports` ← `docs/refactor/01-contracts.md` §八「A 对外名字逐条」。该表把契约层两个包
  混排在一张表里，按包拆不开，期望集合按 §1.1 / §1.2 的文件清单硬编码在 `HARDCODED`：
  17 个 `Protocol` 归 `ports`，其余数据类型与校验函数归 `domain`。

文档表解析不出来时（`_kept_names` 为空）`test_documented_tables_parse` 直接失败，不静默跳过。
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs/refactor"

#: 层名 → (文档, 表起始行前缀, 表结束行前缀)。
DOC_TABLES = {
    "algorithms": ("02-algorithms.md", "#### 8.1.1 层入口", "#### 8.1.2"),
    "runtime": ("03-runtime.md", "### A 对外名字逐条", "### B 历史残留删除清单"),
    "workflows": ("04-workflows.md", "## 8 削面复核", "四个自声明 Protocol"),
    "assembly": ("05-assembly.md", "### 8.1 表 A 对外名字逐条", "### 8.2"),
}

#: 解析结果应有的大小，取自各文档章节标题里的数字，用来发现表结构变化。
PARSED_COUNTS = {"algorithms": 11, "runtime": 9, "workflows": 7, "assembly": 12}

#: 只在「合并」行依据列出现的新名。
EXTRA_EXPORTS = {"runtime": frozenset({"BackgroundCall"})}

#: 契约层两个包的期望集合，出处见文件头。
HARDCODED = {
    "domain": frozenset(
        {
            "WorldPose",
            "RelativePoseDelta",
            "Observation",
            "CameraIntrinsics",
            "relative_to_world",
            "wrap_deg",
            "MotionModel",
            "MotionCapability",
            "HolonomicCommand",
            "CarCommand",
            "MotionCommand",
            "require_supported",
            "CollisionContact",
            "CollisionState",
            "CollisionRecoverySettings",
            "Capabilities",
            "MotionLimits",
            "HeadingHoldSettings",
            "ContinuousTrackingSettings",
            "ExecutionBudget",
            "PoseCheckpoint",
            "TimedVehicleState",
            "SlidingWindowSettings",
        }
    ),
    "ports": frozenset(
        {
            "Vehicle",
            "StreamingVehicle",
            "StreamGoalSource",
            "PositionHoldVehicle",
            "ObservationSource",
            "ControlledObservationSource",
            "MovingObservationSource",
            "ReturnHome",
            "WeatherControl",
            "StreamingViews",
            "AppearanceModelControl",
            "FramePublisher",
            "GoalObserver",
            "TargetDetector",
            "TaskParser",
            "TrajectoryPlanner",
            "TaskTrace",
        }
    ),
}

ENTRIES = ("domain", "ports", "algorithms", "runtime", "workflows", "assembly")

#: 现状与文档的差距，各条标 xfail；落地阶段见注释。
STILL_OPEN = {
    "domain": "阶段 1、2：R20 扩入口与 R21 削面未落地，入口仍导出 122 个名字",
    "ports": "阶段 1、2：R23 的 13 个文件与 R20 扩入口未落地，`CollisionCheck`、`BackendWorker` 等仍在",
    "algorithms": "阶段 1：A 表削面未落地，`motion/`、`depth/`、`targeting/` 二级入口未建",
    "runtime": "阶段 3、4、5：`ControlLoop`、`GuidedVehicle`、`QueueFollower` 尚未上移",
    "workflows": "阶段 5：任务包声明与 `common/` 未落地，入口仍未导出 `TASK_KIND` 一族",
    "assembly": "阶段 6：装配层 12 个名字的收口未落地",
}

NAME_RE = re.compile(r"`([A-Za-z_][A-Za-z0-9_]*)`")

#: 计入期望集的处置列取值。
KEPT_DISPOSITIONS = ("保留", "新增对外")


def _kept_names(lines: list[str], start: str, stop: str) -> list[str]:
    """表区间里处置列为「保留」或「新增对外」的名字，按出现顺序。"""
    names: list[str] = []
    inside = False
    for line in lines:
        if not inside:
            inside = line.startswith(start)
            continue
        if line.startswith(stop):
            break
        if not line.startswith("|") or set(line) <= set("|- "):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) < 4 or cells[3] == "处置":
            continue
        if not cells[3].startswith(KEPT_DISPOSITIONS):
            continue
        # 首列可能带「（现 `旧名`）」或「（新）」这类补注，只取补注前的名字。
        names.extend(NAME_RE.findall(cells[0].split("（")[0]))
    return names


def _expected(entry: str) -> frozenset[str]:
    if entry in HARDCODED:
        return HARDCODED[entry]
    doc, start, stop = DOC_TABLES[entry]
    lines = (DOCS / doc).read_text(encoding="utf-8").splitlines()
    return frozenset(_kept_names(lines, start, stop)) | EXTRA_EXPORTS.get(entry, frozenset())


def _exported(init: Path) -> frozenset[str] | None:
    """`__all__` 的字面值；入口没写 `__all__` 时返回 ``None``。"""
    tree = ast.parse(init.read_text(encoding="utf-8"), filename=str(init))
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        targets = [target.id for target in node.targets if isinstance(target, ast.Name)]
        if "__all__" not in targets:
            continue
        return frozenset(element.value for element in node.value.elts)
    return None


def _diff(expected: frozenset[str], exported: frozenset[str]) -> tuple[list[str], list[str]]:
    return sorted(expected - exported), sorted(exported - expected)


@pytest.mark.parametrize("entry", sorted(DOC_TABLES))
def test_documented_tables_parse(entry):
    """表结构变了就在这里停住，别让下面的比对在空集合上通过。"""
    doc, start, stop = DOC_TABLES[entry]
    lines = (DOCS / doc).read_text(encoding="utf-8").splitlines()
    names = _kept_names(lines, start, stop)
    assert len(names) == len(set(names)), f"表里有重名：{sorted(names)}"
    assert len(names) == PARSED_COUNTS[entry], f"解析到 {len(names)} 个名字：{sorted(names)}"


@pytest.mark.parametrize("entry", sorted(HARDCODED))
def test_hardcoded_sets_are_filled(entry):
    assert len(HARDCODED[entry]) == len(set(HARDCODED[entry]))
    assert HARDCODED[entry], f"{entry} 的期望集合是空的"


@pytest.mark.parametrize(
    "entry",
    [pytest.param(entry, marks=pytest.mark.xfail(strict=True, reason=STILL_OPEN[entry])) for entry in ENTRIES],
)
def test_layer_entry_exports_documented_names(entry):
    expected = _expected(entry)
    exported = _exported(ROOT / "planner" / entry / "__init__.py")
    assert exported is not None, f"planner/{entry}/__init__.py 没有写 __all__"
    missing, extra = _diff(expected, exported)
    assert not missing and not extra, (
        f"planner/{entry}/__init__.py 与文档名单不一致；"
        f"文档有而入口没有：{missing}；入口有而文档没有：{extra}"
    )


def test_checker_catches_missing_and_extra_names(tmp_path):
    """负例：入口多导出一个名字、少导出一个名字都要报出来。"""
    doc = tmp_path / "doc.md"
    doc.write_text(
        "\n".join(
            [
                "### A 对外名字逐条",
                "| 名字 | 现引用数 | 层外运行期调用点 | 处置 | 依据 |",
                "| --- | --- | --- | --- | --- |",
                "| `Kept` | 0 | 无 | 保留 | r |",
                "| `Closed` | 0 | 无 | 内部化 | r |",
                "### B 下一节",
            ]
        ),
        encoding="utf-8",
    )
    expected = frozenset(_kept_names(doc.read_text(encoding="utf-8").splitlines(), "### A 对外名字逐条", "### B"))

    entry = tmp_path / "planner/thing/__init__.py"
    entry.parent.mkdir(parents=True)
    entry.write_text('__all__ = ["Kept", "Extra"]\n', encoding="utf-8")
    missing, extra = _diff(expected, _exported(entry))
    assert missing == []
    assert extra == ["Extra"]

    entry.write_text('__all__ = ["Extra"]\n', encoding="utf-8")
    missing, extra = _diff(expected, _exported(entry))
    assert missing == ["Kept"]
    assert extra == ["Extra"]
