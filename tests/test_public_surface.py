"""每层包入口的 `__all__` 与层文档 A 表的「保留」名单逐名一致（裁决 R21、R24）。

名单来源章节：

- `algorithms`、`runtime`、`workflows` ← 按实际导出面校准，见 `CALIBRATED`：跨层（其他
  `planner` 包、`tests/`、`scripts/`）在用的名字留在入口，只有本层在用的名字不出入口。与文档终值
  的差额：`algorithms`、`runtime` 记在 `STILL_OPEN`，`workflows` 写在 `CALIBRATED` 的注释里。
- `workflows` 的文档来源是 `docs/refactor/04-workflows.md` §8「表 A 对外名字逐条」。
- `assembly` ← `docs/refactor/05-assembly.md` §8.1「表 A 对外名字逐条」，处置列为「新增对外」的
  `build_resources` 一起算。
- `ports` ← 同一张表，收口完成：18 个名字都有跨层调用点，期望集记在 `CALIBRATED`。
- `domain` ← 同一张表。该表把契约层两个包混排在一张表里，按包拆不开，所以期望集合是逐名实测
  的 A 类名单，硬编码在 `HARDCODED`：A 类 = 有跨层调用点（`planner/` 除本身两包外的层，或
  `tests/`）且确属契约类型。文档判“搬出契约层”而代码仍在用的名字不计入 A 类，这就是它那条
  xfail 的差距。

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

#: 已按实际导出面校准的五层。差额逐条记在 `STILL_OPEN`。
CALIBRATED = {
    # 文档 §8.1.1 的 11 个里 `reached` 已随走廊一并落地；入口多导出
    # `world_pose_delta_in_reference_frame`，文档 §1.3 判它私有，但 prompts 仍在调用。
    "algorithms": frozenset(
        {
            "RefillScheduler",
            "TrajectoryValidator",
            "bounded",
            "corridor_default",
            "horizontal_distance",
            "passed_lateral_corridor",
            "point_in_horizontal_body",
            "reached",
            "relative_to_world",
            "render_depth",
            "require_execution_anchor",
            "world_pose_delta_in_reference_frame",
            "world_to_relative",
        }
    ),
    "runtime": frozenset(
        {
            "BackgroundOperation",
            "ContinuousExecution",
            "ControlLoop",
            "ExecutionService",
            "FrameHub",
            "GuidedVehicle",
            "LogBuffer",
            "ModelCallService",
            "MotionPlanner",
            "MovingObservations",
            "PlanningWindow",
            "PositionHold",
            "QueueFollower",
            "SMOOTH_PROFILE",
            "SynchronizedMotionPlanner",
            "TrajectoryHub",
            "braking_speed",
            "close_resources",
            "duration_for",
            "freeze_observation",
            "freeze_reference",
            "heading_rate_deg_s",
            "plan_segment",
            "quintic",
        }
    ),
    "ports": frozenset(
        {
            "AppearanceModelControl",
            "ControlledObservationSource",
            "FramePublisher",
            "GoalObserver",
            "MovingObservationSource",
            "ObservationSource",
            "PositionHoldVehicle",
            "ReturnHome",
            "StreamGoalSource",
            "StreamingVehicle",
            "StreamingViews",
            "TargetDetector",
            "TaskCompletionChecker",
            "TaskParser",
            "TaskTrace",
            "TrajectoryPlanner",
            "Vehicle",
            "WeatherControl",
        }
    ),
    # 文档 §8 表 A 的 7 个里，`TASK_PACKAGES` 未落地（装配层 `requirements.py` 直读三个二级
    # 入口）；三个 `StageAdmission` 由任务包二级入口提供，入口不重复导出。
    "workflows": frozenset(
        {
            "BasicMotionWorkflow",
            "FreeNavigationWorkflow",
            "VisualGoalNavigationWorkflow",
            "Workflow",
            "require_confirmed",
        }
    ),
    # 文档 §8.1 的 12 个里 `StageValidator`、`WorkerFactory`、`SimulatorSupervisor`、
    # `SwitchRefused`、`build_resources` 尚未落地；`capabilities_for` 与四个 `build_*` 有层外
    # 运行期调用点，文档判内联进 `build_resources`，内联未做，名字留在入口。
    "assembly": frozenset(
        {
            "BackendUnavailable",
            "MapSelection",
            "MissionInputs",
            "MissionResources",
            "ServerController",
            "ServerState",
            "VehicleArm",
            "assemble_missions",
            "build_detector",
            "build_free_navigation_planner",
            "build_geometry",
            "build_planner",
            "capabilities_for",
            "require_application",
            "task_finish_events",
        }
    ),
}

#: `domain` 的期望集合，出处见文件头。
HARDCODED = {
    "domain": frozenset(
        {
            "ArrivalRelation",
            "BasicMotionParameters",
            "BasicMotionResult",
            "BasicMotionStatus",
            "CameraIntrinsics",
            "Capabilities",
            "CarCommand",
            "CollisionContact",
            "CollisionRecoverySettings",
            "CollisionState",
            "CompletionBasis",
            "ContinuationCandidate",
            "ContinuousReference",
            "ContinuousTrackingSettings",
            "DepthForegroundResult",
            "DepthForegroundSettings",
            "DepthPointLayer",
            "ExecutionBudget",
            "ExecutionReceipt",
            "ExecutionRecord",
            "FAR_VALID_PIXEL_VALUE",
            "FamilyCatalog",
            "FreeNavigationParameters",
            "FreeNavigationResult",
            "FreeNavigationStatus",
            "GoalGeometry",
            "GoalObservation",
            "GoalReference",
            "GoalVisibility",
            "HeadingHoldSettings",
            "HolonomicCommand",
            "INVALID_DEPTH_SENTINEL_M",
            "LEGACY_DEPTH_MAPPING",
            "MODE_CONTINUOUS",
            "MODE_IDLE",
            "MODE_NO_QUEUE",
            "MODE_PENDING",
            "MODE_STOPPED_BATCH",
            "MissionPlan",
            "MissionResult",
            "MissionStage",
            "MotionCapability",
            "MotionCommand",
            "MotionLimits",
            "MotionModel",
            "MotionSegment",
            "MotionTrajectory",
            "NormalizedBoundingBox",
            "Observation",
            "PlanningOutput",
            "PlanningSnapshot",
            "PlanningWindowInput",
            "PoseCheckpoint",
            "QueueSnapshot",
            "QueuedWaypoint",
            "RelativePoseDelta",
            "RelativeTrajectory",
            "ResultMeasurement",
            "SHARED_DEPTH_MAPPING",
            "SimulatorFamily",
            "SlidingWindowSettings",
            "StageResult",
            "StageSummary",
            "StopResult",
            "SurfacePointEstimate",
            "TargetDetection",
            "TargetDetectionInput",
            "TargetObservationEvidence",
            "TargetPositionState",
            "TaskKind",
            "TaskParameters",
            "TimedVehicleState",
            "TrajectoryPointSnapshot",
            "TrajectorySnapshot",
            "VEHICLE_DEFAULT_VIEW",
            "VEHICLE_FRONT_VIEW",
            "VIEW_LABELS",
            "VIEW_MODES",
            "VehicleSnapshot",
            "VisualGoalCompletionInput",
            "VisualGoalLimits",
            "VisualGoalNavigationParameters",
            "VisualGoalNavigationResult",
            "VisualGoalPlanningContext",
            "VisualGoalState",
            "VisualGoalStatus",
            "VisualGoalTargetSnapshot",
            "VisualGoalTriggerRules",
            "WEATHER_CATALOG",
            "WEATHER_FIELDS",
            "WEATHER_LABELS",
            "WeatherCatalog",
            "WorldPose",
            "WorldTrajectory",
            "build_queue_snapshot",
            "command_type_for",
            "cycle_delta",
            "default_view_for",
            "default_weather",
            "depth_display_description",
            "front_view_for",
            "merge_weather",
            "parse_family_catalog",
            "relative_to_world",
            "require_supported",
            "resolve_yaw_delta_deg",
            "snapshot_from_payload",
            "snapshot_to_payload",
            "validate_depth_protocol",
            "validate_view_mode",
            "validate_weather_values",
            "vehicle_snapshot",
            "view_index",
            "weather_catalog_from_payload",
            "wrap_deg",
            "wrap_yaw_deg",
            "QueueObservationAnchor",
        }
    ),
}

# 契约层已收口：`server.py`、`approach.py`、`streaming.py` 三类搬走后，入口与实测 A 类逐名一致。
CALIBRATED["domain"] = HARDCODED["domain"]  # 契约层收口后逐名一致

ENTRIES = ("domain", "ports", "algorithms", "runtime", "workflows", "assembly")

#: 现状与文档的差距，各条标 xfail；落地阶段见注释。
STILL_OPEN = {
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
    if entry in CALIBRATED:
        return CALIBRATED[entry]
    if entry in HARDCODED:
        return HARDCODED[entry]
    doc, start, stop = DOC_TABLES[entry]
    lines = (DOCS / doc).read_text(encoding="utf-8").splitlines()
    return frozenset(_kept_names(lines, start, stop))


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


@pytest.mark.parametrize("entry", sorted(HARDCODED | CALIBRATED))
def test_expected_sets_are_filled(entry):
    assert _expected(entry), f"{entry} 的期望集合是空的"


@pytest.mark.parametrize(
    "entry",
    [
        pytest.param(entry)
        if entry not in STILL_OPEN
        else pytest.param(
            entry, marks=pytest.mark.xfail(strict=True, reason=STILL_OPEN[entry])
        )
        for entry in ENTRIES
    ],
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
