> 上级：[项目架构](../ARCHITECTURE.md)
> 同层兄弟：[domain](domain.md)、[ports](ports.md)、[algorithms](algorithms.md)、[runtime](runtime.md)、[assembly](assembly.md)、[adapters](adapters.md)、[simulator](simulator.md)、[simulators](simulators.md)

# 任务层 workflows

## 1 一句话职责

把一份 `MissionStage` 的参数换成对 `ControlLoop` 的请求，把执行层返回的 receipt 换成 `StageResult`，中间的任务判定（暂停、确认、停滞、搜索、预算）都在本层完成。

## 2 边界

管这些：

- 三个任务包各自的参数类型检查（`policy.py` 里的 `*StageValidator.validate`）与构造期参数校验。
- 执行模式的选择：`basic_motion` 恒段执行；`visual_goal_navigation` 按 `moving_observation_source` 是否注入决定连续或停稳分批；`free_navigation` 同。
- 任务内判定：目标接近半径、确认计数、搜索转向、停滞检测、锚点新鲜度、模型调用预算与重试。
- `StageResult` / `StageSummary` 的构造与 `CompletionBasis` 的选择。

不管这些：

- 运动学、限幅、走廊容差：[algorithms](algorithms.md)。
- 20 Hz 下发、段采样、队列消耗、规划窗、取帧线程、模型单飞：[runtime](runtime.md)。
- 任务解析、阶段校验的编排、路由、能力求交、载具生命周期：[assembly](assembly.md)。
- 端口对象的实现与引擎互操作：[simulators](simulators.md)。

## 3 输入与输出

| 入口 | 签名 | 文件 |
| --- | --- | --- |
| `BasicMotionWorkflow(vehicle, motion, execution, clock=..., *, loop, limits, telemetry)` | `prepare() / run(stage) / close()` | `planner/workflows/basic_motion/workflow.py:31` |
| `VisualGoalNavigationWorkflow(observation_source, goal_observer, trajectory_planner, validator, motion_planner, execution, geometry, completion, limits, trace_factory=None, clock=..., sleep=..., *, loop, completion_checker=None, trigger_rules=..., sliding_window=..., motion_limits=..., moving_observation_source=None, stream_speed_mps=2.0, stream_waypoint_timeout_s=60.0, continuous_tracking=..., telemetry=None, vertical_motion=True)` | 同上 | `planner/workflows/visual_goal_navigation/workflow.py:67` |
| `FreeNavigationWorkflow(observation_source, planner, validator, motion, execution, clock=..., sleep=..., *, loop, model_request_timeout_s=30.0, max_consecutive_stalls=2, sliding_window=None, motion_limits=None, moving_observation_source=None, stream_speed_mps=2.0, stream_waypoint_timeout_s=60.0, continuous_tracking=..., telemetry=None, vertical_motion=True)` | 同上 | `planner/workflows/free_navigation/workflow.py:51` |

- 输入类型全部来自 `planner.domain`、`planner.ports`、`planner.errors` 与本层 Protocol；注解里没有别层的具体类。
- 返回：`StageResult[BasicMotionResult]`、`StageResult[object]`（值是 `VisualGoalNavigationResult`）、`StageResult[FreeNavigationResult]`。`planner/workflows/base.py:8-11` 声明的 `Workflow` 只有 `prepare/run/close` 三个方法。
- 异常：构造期 `ValueError`、`ProtocolError`；运行期 `ProtocolError`、`ObservationError`、`ExecutionError`、`BudgetExceeded`、`ModelCallTimeout`、`FreeNavigationStalled`、任务内 `VisualGoalFailure`（`planner/workflows/visual_goal_navigation/termination.py:11-14`）。
- 生命周期现状：`basic_motion` 的 `prepare()` 与 `close()` 是空实现（`planner/workflows/basic_motion/workflow.py:51-55`），`run()` 不调 `prepare()`（对比 `planner/workflows/free_navigation/workflow.py:203`、`planner/workflows/visual_goal_navigation/workflow.py:135`）。

## 4 对外名字表

| 名字 | 类型 | 用途 | 层外调用点 |
| --- | --- | --- | --- |
| `BasicMotionWorkflow` | 类 | 数值动作段执行 | `planner/assembly/mission_assembly.py:269`；`scripts/smoke_projectairsim_drone.py:242`；`scripts/smoke_projectairsim_rover.py:123` |
| `VisualGoalNavigationWorkflow` | 类 | 视觉目标接近（两种模式） | `planner/assembly/mission_assembly.py:246`；`scripts/smoke_projectairsim_drone.py:118` |
| `FreeNavigationWorkflow` | 类 | 无终点巡航 | `planner/assembly/mission_assembly.py:290`；`scripts/smoke_projectairsim_rover.py:160` |
| `Workflow` | Protocol | `prepare/run/close` 形状 | `planner/assembly/workflow_router.py:3,8,15`（注解） |
| `require_confirmed` | 函数 | 校验 stage 结果带确认证据 | `planner/assembly/mission_runner.py:4,24` |
| `TASK_KIND` / `REQUIRED_PORTS`（三个二级入口各一份） | `TaskKind` / `tuple[str, ...]` | 能力求交的唯一来源 | `planner/assembly/requirements.py:24-28`（字符串路径 + `import_module`）、`planner/assembly/requirements.py:53-54` |
| `BasicMotionStageValidator` | 类 | 数值动作阶段准入 | `planner/assembly/mission_assembly.py:23,282` |
| `VisualGoalStageValidator` | dataclass | 视觉目标任务阶段准入 | `planner/assembly/mission_assembly.py:25,283` |
| `FreeNavigationStageValidator` | 类 | 巡航阶段准入 | `planner/assembly/mission_assembly.py:24,308` |
| `VisualGoalCompletion` | 类 | 完成候选判定 | `planner/assembly/mission_assembly.py:21,254`；`scripts/smoke_projectairsim_drone.py:126` |
| `TargetApproach` | 类 | 半径判定 | `planner/assembly/mission_assembly.py:20,254`；`scripts/smoke_projectairsim_drone.py:126` |
| `BBoxGoalObserver` | 类 | `GoalObserver` 的检测器实现 | `planner/assembly/mission_assembly.py:22,232`；`scripts/smoke_projectairsim_drone.py:109` |

层入口 `planner/workflows/__init__.py:9-15` 的 `__all__` 是上面五个名字（三个工作流类、`Workflow`、`require_confirmed`）。其余名字从二级入口取。

## 5 关键流程

### 5.1 数值动作（恒段执行）

1. `run(stage)` 取 `began`，把 `status` 预置为 `EXECUTION_ERROR`：`planner/workflows/basic_motion/workflow.py:57-63`。
2. `execution.acquire_flight(self)` 取控制权，置 `leased`：`planner/workflows/basic_motion/workflow.py:66-67`。不设阶段 deadline，超时只由 `ControlLoop` 的 `timeout_s` 兜。
3. `vehicle.current_pose()` 取起点，可选发一条 `publish_no_queue` 遥测：`planner/workflows/basic_motion/workflow.py:68-74`。
4. `BasicMotionConverter().create_target(parameters, start)` 算终点，偏航取 `parameters.yaw_right_deg`：`planner/workflows/basic_motion/workflow.py:75-76`。
5. 合成一段 `MotionSegment`，profile 为 `SMOOTH_PROFILE`，时长来自 `duration_for`：`planner/workflows/basic_motion/workflow.py:83-96`。
6. `loop.execute(MotionTrajectory((segment,)))` 下发：`planner/workflows/basic_motion/workflow.py:97`；`ControlLoop.execute` 调 `run_segment` 并折成 `ExecutionRecord`：`planner/runtime/motion/loop.py:191-200`。
7. 段末比对位置与偏航容差，超差抛 `ExecutionError`：`planner/workflows/basic_motion/workflow.py:99-111`。
8. 组 `BasicMotionResult` 与 `StageSummary`，measurements 只有 `elapsed`、`stop_confirmed`：`planner/workflows/basic_motion/workflow.py:138-161`。
9. `finally` 里 `release_flight`：`planner/workflows/basic_motion/workflow.py:129-131`。

### 5.2 视觉目标：停稳分批

1. `run` 首行 `prepare()`：`planner/workflows/visual_goal_navigation/workflow.py:135`；`prepare` 检查已关闭、模型服务空闲、目标刷新隔离、完成校验器存在、窗口 worker 空闲、连续模式就绪：`planner/workflows/visual_goal_navigation/workflow.py:112-124`。
2. `acquire_flight` + `set_deadline(session.budget.deadline_monotonic)`：`planner/workflows/visual_goal_navigation/workflow.py:136-138`。
3. 有窗口则 `window.begin(deadline_monotonic=..., maximum_planning_rounds=...)`：`planner/workflows/visual_goal_navigation/workflow.py:139-141`。
4. 无连续实现时 `target_updates.begin()`：`planner/workflows/visual_goal_navigation/workflow.py:143-144`；建本轮 `VisualGoalRadiusTrigger`：`planner/workflows/visual_goal_navigation/workflow.py:145`。
5. 参数类型不符抛 `VisualGoalFailure(BLOCKED)`，半径与不确定度不合规由 `approach_settings` 抛 `ValueError`：`planner/workflows/visual_goal_navigation/workflow.py:147-149`。
6. 主循环 `_run`：`_capture` 取帧并验停稳（`planner/workflows/visual_goal_navigation/workflow.py:220-232`）→ `session.fresh` 判重（`planner/workflows/visual_goal_navigation/workflow.py:280-284`，`planner/workflows/visual_goal_navigation/session.py:67-76`）→ `target_state` 为空时调 `model_call` 跑 `goal_observer.observe`（`planner/workflows/visual_goal_navigation/workflow.py:285-291`）→ `anchor_valid`（`planner/workflows/visual_goal_navigation/workflow.py:292`、`planner/workflows/visual_goal_navigation/workflow.py:306-308`）→ `target_updates.accept`（`planner/workflows/visual_goal_navigation/workflow.py:298-300`）→ 暂停半径判定与 `completion.candidate`（`planner/workflows/visual_goal_navigation/workflow.py:316-321`）→ 确认计数（`planner/workflows/visual_goal_navigation/workflow.py:324-328`）→ 达 `confirmation_count` 后 `flight.verify`（`planner/workflows/visual_goal_navigation/workflow.py:330-340`）。
7. 确认成功：清 `pending_motion`、`window.complete()`、置 `completion_confirmed`、返回 `REACHED`：`planner/workflows/visual_goal_navigation/workflow.py:334-340`。
8. 未确认：`approach.run` 取规划并执行，`_execute_window` 逐段下发并在段间 `window.executed` 记账：`planner/workflows/visual_goal_navigation/approach.py:21-43`、`planner/workflows/visual_goal_navigation/flight.py:95-126`。
9. `_settle_completed_calls` 等在途调用落地，超时抛 `ModelCallTimeout`：`planner/workflows/visual_goal_navigation/workflow.py:156`、`planner/workflows/visual_goal_navigation/workflow.py:259-267`。
10. `finally`：连续实现有则 `flush_telemetry`，`window.end()`，`release_flight(self, self.has_pending_calls)`：`planner/workflows/visual_goal_navigation/workflow.py:179-188`。
11. 结果与 summary：`success = status is REACHED and session.completion_confirmed`，`CompletionBasis.MODEL_WITH_LOCAL_CHECKS`，measurements 为 `distance/elapsed/planning_calls/stop_confirmed`：`planner/workflows/visual_goal_navigation/workflow.py:198-218`。

### 5.3 视觉目标：连续

1. 构造期仅在 `moving_observation_source` 非空时建 `VisualGoalContinuous`，窗口未启用则抛 `ProtocolError`：`planner/workflows/visual_goal_navigation/workflow.py:102-110`。该分支用函数内 `from .continuous import VisualGoalContinuous`：`planner/workflows/visual_goal_navigation/workflow.py:105`。
2. `run` 不区分模式的部分一致（`prepare`、`acquire_flight`、`set_deadline`、`window.begin`），随后只在停稳分批调 `target_updates.begin()`：`planner/workflows/visual_goal_navigation/workflow.py:143-144`。
3. `_run` 直接进 `self.continuous.run(parameters, session, trace)`：`planner/workflows/visual_goal_navigation/workflow.py:270-271`。
4. 连续实现起手：`frames.begin()`、`targets.begin()`、循环 `_observe_stopped` 直到拿到停稳首帧、`stream.begin(window.queue, window.background.stage_token)`：`planner/workflows/visual_goal_navigation/continuous.py:224-229`。
5. 每拍：`require_time` → `stream.sample()` → `window.record = stream.record` → `frames.poll()` 有帧则 `targets.submit` → `targets.poll` → `window.poll_stream` → `stream.tick(pause_reason)`（`planner/runtime/continuous_execution.py:96-131`）→ `_sync_remaining` → `_publish` → 补点或补取帧 → 按控制周期 sleep：`planner/workflows/visual_goal_navigation/continuous.py:233-283`。
6. 暂停：`stream.paused` 非空时 `_check_pause`，先丢弃窗口后台任务、移动帧与目标刷新：`planner/workflows/visual_goal_navigation/continuous.py:134-138`；再取停稳帧、`stream.sample()`、`_sync_remaining`、`completion.candidate`、`flight.verify`；判定失败则 `stream.resume()` 继续，成功则 `window.complete()` 返回 `REACHED`：`planner/workflows/visual_goal_navigation/continuous.py:141-173`。
7. `finally` 丢弃在途帧与目标请求，再同步残余段：`planner/workflows/visual_goal_navigation/continuous.py:284-288`。

### 5.4 无终点巡航：连续

1. 构造期校验四个数值参数与非负整数停滞上限：`planner/workflows/free_navigation/workflow.py:73-106`；`moving_observation_source` 非空时要求窗口已启用且有 `motion_limits`，随后建 `PlanningWindow` 与 `FreeNavigationContinuous`：`planner/workflows/free_navigation/workflow.py:123-156`。
2. `run` 先 `prepare()`：`planner/workflows/free_navigation/workflow.py:203`；`prepare` 查关闭态，连续模式查窗口 worker 与 `continuous.prepare()`，否则查 `model_calls.require_available()`：`planner/workflows/free_navigation/workflow.py:158-165`。
3. `acquire_flight`，`set_deadline(None)`（无阶段时限）：`planner/workflows/free_navigation/workflow.py:209-211`。
4. `window.begin(deadline_monotonic=None, maximum_planning_rounds=None)`：`planner/workflows/free_navigation/workflow.py:213-216`。
5. `_capture_stopped()` 取首帧，未停稳抛 `ObservationError`：`planner/workflows/free_navigation/workflow.py:217`、`planner/workflows/free_navigation/workflow.py:364-374`。
6. `continuous.run(parameters, observation, self.window, stage_id=...)`：`planner/workflows/free_navigation/workflow.py:218-221`；正常返回被当成不可能，抛 `AssertionError`：`planner/workflows/free_navigation/workflow.py:222-224`。
7. 连续实现内建 `QueueFollower` 与 `MovingObservations`（构造期）：`planner/workflows/free_navigation/continuous.py:83-96`；`run` 里 `frames.begin`、`loop.recorder.begin`、`follower.begin(window.queue)`，再组 `StreamRunner` 与 `StreamPolicy`：`planner/workflows/free_navigation/continuous.py:144-170`。
8. `runner.run(window, initial_observation)` 无限循环 `StreamRunner.tick`：`planner/workflows/free_navigation/continuous.py:172`；每拍动作固定在 `planner/workflows/common/streaming.py:78-118`。
9. `finally` 丢弃在途帧并更新测量：`planner/workflows/free_navigation/continuous.py:173-175`；外层 `window.end()`、`release_flight(self, self.has_pending_calls)`：`planner/workflows/free_navigation/workflow.py:312-316`。
10. 结果恒为 `success=False`、`CompletionBasis.NOT_VERIFIED`，measurements 为 `planning_calls/elapsed/traveled/stop_confirmed/execution_mode`：`planner/workflows/free_navigation/workflow.py:334-362`。

### 5.5 无终点巡航：停稳分批回退

`moving_observation_source` 为空时走 `planner/workflows/free_navigation/workflow.py:225-278`：取停稳帧 → `begin` 记起点 → `model_calls.call` 调 `planner.plan`（`budget=None`）→ `validate_relative` → `relative_to_world` → `validate_world` → `loop.execute(motion.create_motion(world))` → 用 `record.translation_m` 与 `record.absolute_yaw_deg` 判停滞。该分支的规划调用不过 `model_call`，没有重试与 `planning_calls` 计数。

## 6 失败模式

| 报错类型 | 触发条件 | 调用方处理 |
| --- | --- | --- |
| `ValueError` | 构造期参数非法：`planner/workflows/free_navigation/workflow.py:73-106`、`planner/workflows/free_navigation/continuous.py:56-74`、`planner/workflows/visual_goal_navigation/policy.py:28-31`、`planner/workflows/visual_goal_navigation/policy.py:39-44` | 装配期失败，服务不起或该载具无管线 |
| `ProtocolError` | 阶段参数类型不符（三个 `policy.py`）；连续模式缺窗口（`planner/workflows/visual_goal_navigation/workflow.py:103-104`、`planner/workflows/free_navigation/workflow.py:124-127`）；完成校验器缺失（`planner/workflows/visual_goal_navigation/workflow.py:118-119`）；完成证据不合规（`planner/workflows/visual_goal_navigation/completion.py:32-70`） | 视觉任务映射 `MODEL_ERROR`，巡航映射 `MODEL_ERROR`，数值动作映射 `EXECUTION_ERROR` |
| `ExecutionError` | 控制权冲突、`stop()` 失败、关闭后调用（`planner/workflows/free_navigation/workflow.py:159-160`、`planner/workflows/visual_goal_navigation/workflow.py:112-114`）、连续模式缺 `stream_state/stream_command`（`planner/workflows/free_navigation/continuous.py:105-114`）、位置容差不足（`planner/workflows/basic_motion/workflow.py:108-111`） | 映射 `EXECUTION_ERROR`；`MissionRunner` 见到首败即停 |
| `BudgetExceeded` | `common/budget.require_time`（`planner/workflows/common/budget.py:9-12`）、`ControlLoop.run_segment/run_stream` 的 deadline、`ExecutionService.check_deadline` | 视觉与巡航映射 `TIMEOUT`；数值动作落进 `except Exception`，映射 `EXECUTION_ERROR` |
| `ModelCallTimeout` | `ModelCallService.call` 超时；`planner/workflows/visual_goal_navigation/workflow.py:260-265` 交接超时 | 同上（`TIMEOUT`） |
| `ObservationError` | 取帧异常或未停稳（`planner/workflows/free_navigation/workflow.py:364-374`、`planner/workflows/visual_goal_navigation/workflow.py:227-228`）；移动帧几何非法（`planner/workflows/visual_goal_navigation/continuous.py:241-244`） | 映射 `OBSERVATION_ERROR` |
| `FreeNavigationStalled` | 停稳分批连续两批零位移零转角（`planner/workflows/free_navigation/workflow.py:268-278`）；连续模式 `stall_timeout_s` 内无进展（`planner/workflows/free_navigation/continuous.py:206-218`） | 映射 `STUCK` |
| `VisualGoalFailure` | 候选与确认、搜索预算、停滞、锚点漂移、完成检查预算（`planner/workflows/visual_goal_navigation/termination.py:25-30,32-35,37-41,59-67,78-106`、`planner/workflows/visual_goal_navigation/flight.py:22-24`、`planner/workflows/visual_goal_navigation/workflow.py:377-380`） | 按 `status` 落到 `TARGET_*`/`PERCEPTION_UNRELIABLE`/`STUCK`/`TIMEOUT` |
| `AssertionError` | 连续巡航返回（`planner/workflows/free_navigation/workflow.py:222-224`）、重试循环出口（`planner/workflows/common/budget.py:52`、`planner/workflows/visual_goal_navigation/completion.py:182`） | 前一条被 `except Exception` 收成 `BLOCKED`；后两条不可达 |
| `KeyboardInterrupt` | 用户停止经 `ExecutionService` 检查点抛出 | 先 `stop_if_owner` 再原样上抛，不转 `StageResult` |

状态映射的落点：`planner/workflows/free_navigation/workflow.py:290-301`、`planner/workflows/visual_goal_navigation/workflow.py:159-172`、`planner/workflows/basic_motion/workflow.py:121-132`。

## 7 测试位置

| 文件 | 用例数 | 覆盖 |
| --- | --- | --- |
| `tests/test_workflow.py` | 10 | 视觉停稳分批：两次确认后到达、搜索关闭时 `BLOCKED`、深度非法、观测器失败、两次零位移 `STUCK`、规划预算 `TIMEOUT`、检查点取消、确认失败后重新计数、完成检查预算、trace 事件 |
| `tests/test_free_navigation.py` | 27 | 编解码与提示词 7 例；停稳分批 11 例（含 `prepare` 关闭态、`release_flight` 挂起清理、停滞、取消、取帧失败）；连续路径 9 例（提交停稳与移动帧、遥测、看门狗停滞与等待规划不误报、`prepare` 要求流式能力、需要窗口与限速、无期限开始、挂起调用隔离、失败映射、首帧必须停稳） |
| `tests/test_basic_motion.py` | 10 | 参数校验 3 例、转换器 2 例（四个参数组）、工作流完成/爬升/未到位/容差失败/取消/全转 5 例 |
| `tests/test_stream_runner.py` | 6 | 先取帧后授权、未用帧不补点、空队列每拍停一次、非空不停、暂停拍不驱动不取帧、取帧单飞且受超时约束 |
| `tests/test_capability_targeting.py` | 12 | 目标刷新、锚点接受/拒绝、几何绑定 |
| `tests/test_visual_goal_completion.py` | 10 | 完成证据准入、候选判定、验证器重试 |
| `tests/test_visual_goal_anchor.py` | 6 | 锚点年龄与漂移判定 |

空白：

- `VisualGoalContinuous`（`planner/workflows/visual_goal_navigation/continuous.py`，287 行）全仓无测试引用，`_check_pause`、`_submit`、`_sync_remaining` 均未覆盖。
- 视觉工作流的连续分支（`planner/workflows/visual_goal_navigation/workflow.py:143`、`planner/workflows/visual_goal_navigation/workflow.py:270-271`、`planner/workflows/visual_goal_navigation/workflow.py:206-207`）无测试；构造期连续装配无测试。
- `basic_motion` 的 `prepare()`/`close()` 是空实现，没有测试断言它们的行为，也没有关闭后再 `run` 的负例。
- `planner/workflows/free_navigation/continuous.py:177-228` 的 `_publish`/`_log_authorized` 只有遥测 1 例覆盖，日志分支未断言。
- `common/budget.model_call` 的重试与 `on_planning_call` 计数只在视觉侧被间接覆盖，直接单测缺失。

## 8 与其他层的联动

| 层 | 方向 | 数据形态 |
| --- | --- | --- |
| [domain](domain.md) | 上游 | `MissionStage`、三个 `*Parameters`/`*Result`/`*Status`、`MotionTrajectory`、`ExecutionBudget`、`ExecutionReceipt`、`StopResult`、`StageResult`、`StageSummary`、`SlidingWindowSettings`、`ContinuousTrackingSettings`、`VisualGoalLimits`、`VisualGoalTriggerRules` |
| [ports](ports.md) | 上游 | `ObservationSource`、`MovingObservationSource`、`GoalObserver`、`TrajectoryPlanner`、`TaskCompletionChecker`、`TaskTrace` |
| [algorithms](algorithms.md) | 上游 | `BasicMotionConverter`、`TrajectoryValidator`、`relative_to_world`、`world_to_relative`、`bound_target_geometry`、`bound_target_goal`、`horizontal_distance`、`accept_position`、`make_reference` |
| [runtime](runtime.md) | 上游（构造注入）/下游（调用） | `ControlLoop`（`execute`/`run_segment`/`run_stream`/`segment_sink`/`stream_sink`/`state`/`recorder`/`period`/`limits`）、`ExecutionService`、`PlanningWindow`、`QueueFollower`、`ContinuousExecution`、`MovingObservations`、`ModelCallService`、`BackgroundOperation`、`freeze_observation`、`freeze_reference`、`close_resources` |
| [assembly](assembly.md) | 下游 | 构造三个工作流与三个 `StageValidator`，读 `TASK_KIND`/`REQUIRED_PORTS`，调 `prepare/run/close`，用 `require_confirmed` 校验返回值 |
| [simulators](simulators.md) | 间接 | 只在 `execution.vehicle` 这一层间接到达（`planner/workflows/free_navigation/continuous.py:109-114`、`planner/workflows/free_navigation/continuous.py:220-224`、`planner/workflows/visual_goal_navigation/continuous.py:49-51`） |

## 9 疑问与争议

- Qdocc01 层：workflows 结论：`basic_motion` 没有阶段预算也没有关闭态，`run` 不调 `prepare`，`prepare/close` 空实现，关闭后再 `run` 仍会驱动载具 依据：`planner/workflows/basic_motion/workflow.py:51-55`、`planner/workflows/basic_motion/workflow.py:57-67`，对比 `planner/workflows/free_navigation/workflow.py:158-160`、`planner/workflows/visual_goal_navigation/workflow.py:112-114` 影响：否
- Qdocc02 层：workflows 结论：`basic_motion` 用 `except Exception` 收全部失败，`BudgetExceeded` 也映射成 `EXECUTION_ERROR`，而 `BasicMotionStatus` 只有 `COMPLETED` 与 `EXECUTION_ERROR` 两个取值，超时与控制器报错无法区分 依据：`planner/workflows/basic_motion/workflow.py:121-132` 影响：否
- Qdocc03 层：workflows 结论：视觉连续路径不复用 `StreamRunner` 的代码事实是三条缺口：`StreamPolicy` 没有暂停后确认再续跑的入口、`submit` 回调不带 anchor 与 trace 图像参数、取帧与窗口记录由两个不同对象提供（`ContinuousExecution` 对 `ControlLoop.run_stream`） 依据：`planner/workflows/common/streaming.py:16-118`、`planner/workflows/visual_goal_navigation/continuous.py:151-283`、`planner/workflows/free_navigation/continuous.py:151-172` 影响：是（runtime 层两套驱动并存）
- Qdocc04 层：workflows 结论：`StreamRunner` 只有一个生产调用点，`StreamPolicy` 五个回调默认全 `None`，属单用途抽象 依据：`planner/workflows/common/streaming.py:36-118`、`planner/workflows/free_navigation/continuous.py:151` 影响：否
- Qdocc05 层：workflows 结论：停滞判据在同一层有三套且阈值无配置出处：`1e-6 m`/`1e-6 deg` 计数、时间窗无进展、`0.2 m`/`2.0 deg` 计数 依据：`planner/workflows/free_navigation/workflow.py:268-278`、`planner/workflows/free_navigation/continuous.py:206-218`、`planner/workflows/visual_goal_navigation/termination.py:25-30` 影响：否
- Qdocc06 层：workflows 结论：视觉与巡航的连续实现各有一份 `_submit` 与 `_publish` 镜像，`timeline.at` 失败后的补锚点分支逐行相同 依据：`planner/workflows/visual_goal_navigation/continuous.py:105-127`、`planner/workflows/visual_goal_navigation/continuous.py:175-221`；`planner/workflows/free_navigation/continuous.py:177-195`、`planner/workflows/free_navigation/continuous.py:235-277` 影响：否
- Qdocc07 层：workflows 结论：`model_call` 的重试与规划计数只在视觉侧使用；巡航停稳分批直接调 `model_calls.call` 且给规划器传 `budget=None`，没有重试也没有计划数上限 依据：`planner/workflows/visual_goal_navigation/approach.py:31-35`、`planner/workflows/common/budget.py:33-52`、`planner/workflows/free_navigation/workflow.py:238-258` 影响：否
- Qdocc08 层：workflows 结论：`require_confirmed` 放在本层 `common/evidence.py` 并从层入口导出，唯一调用点在装配层，本层运行期无读者 依据：`planner/workflows/common/evidence.py:9-29`、`planner/workflows/__init__.py:4,14`、`planner/assembly/mission_runner.py:24` 影响：是（assembly.md 同名条目）
- Qdocc09 层：workflows 结论：巡航连续路径用 `raise AssertionError` 表达"函数不该返回"，随后被 `except Exception` 吞成 `BLOCKED`，异常类型即控制流 依据：`planner/workflows/free_navigation/workflow.py:218-224`、`planner/workflows/free_navigation/workflow.py:285-301` 影响：否
- Qdocc10 层：workflows 结论：视觉连续实现没有 `_closed` 标志，`prepare()` 也不查关闭态，与巡航连续实现不对称 依据：`planner/workflows/visual_goal_navigation/continuous.py:48-61`、对比 `planner/workflows/free_navigation/continuous.py:103-127` 影响：否
- Qdocc11 层：workflows 结论：两个连续实现都穿过 `ExecutionService` 直取 `execution.vehicle` 端口并调 `stream_state`/`hover`，本层声明的 Protocol 面没有这几个方法 依据：`planner/workflows/free_navigation/continuous.py:109-114`、`planner/workflows/free_navigation/continuous.py:220-224`；`planner/workflows/visual_goal_navigation/continuous.py:49-51` 影响：是（ports.md 的 `StreamingVehicle` 说明）
- Qdocc12 层：workflows 结论：`approach.checkpoint` 与 `flight.verify` 里的"碰撞中"分支不可达，因为 `ExecutionService.collision_state()` 恒返回 `False` 依据：`planner/workflows/visual_goal_navigation/approach.py:69-73`、`planner/workflows/visual_goal_navigation/flight.py:34-38`、`planner/runtime/execution.py:104-111` 影响：是（runtime.md 的碰撞职责条目）

## 10 已知缺口

三类任务的能力差异，逐格给依据。

| 项 | 数值动作 | 视觉目标接近 | 无终点巡航 |
| --- | --- | --- | --- |
| 阶段预算 | 无。不调 `set_deadline`，超时只由 `ControlLoop` 的 `timeout_s` 兜（`planner/workflows/basic_motion/workflow.py:66-97`） | 有。`maximum_task_duration_s` 写成 `ExecutionBudget` 并 `set_deadline`（`planner/workflows/visual_goal_navigation/workflow.py:128`、`planner/workflows/visual_goal_navigation/workflow.py:138`） | 无。`set_deadline(None)`，连续与停稳回退两条路都不设期限（`planner/workflows/free_navigation/workflow.py:211`） |
| 关闭态 | 无。`close()` 空实现，`run` 不查关闭标志（`planner/workflows/basic_motion/workflow.py:51-55`） | 有。`prepare` 抛 `ExecutionError`（`planner/workflows/visual_goal_navigation/workflow.py:112-114`）；连续实现缺这一个标志（`planner/workflows/visual_goal_navigation/continuous.py:48-61`） | 有（`planner/workflows/free_navigation/workflow.py:158-160`）；连续实现另有自己的标志（`planner/workflows/free_navigation/continuous.py:105-127`） |
| 异常映射 | 一律 `EXECUTION_ERROR`，`BudgetExceeded` 与 `ExecutionError` 不可分（`planner/workflows/basic_motion/workflow.py:121-132`） | 六类分支按类型映射（`planner/workflows/visual_goal_navigation/workflow.py:159-172`） | 六类分支按类型映射（`planner/workflows/free_navigation/workflow.py:290-301`） |
| 连续路径 | 无 | `ContinuousExecution`，每拍 `stream.tick`（`planner/workflows/visual_goal_navigation/continuous.py:233-283`） | `StreamRunner` 加 `QueueFollower`（`planner/workflows/free_navigation/continuous.py:83-172`） |
| 停滞判据 | 无 | 位移 `< 0.2 m` 且转角 `< 2.0°` 连续两次（`planner/workflows/visual_goal_navigation/termination.py:25-30`） | 停稳批次 `< 1e-6 m` 且 `< 1e-6°` 计 `max_consecutive_stalls` 次，连续走 `stall_timeout_s` 时间窗（`planner/workflows/free_navigation/workflow.py:268-278`、`planner/workflows/free_navigation/continuous.py:206-218`） |

缺口清单：

1. 阶段预算与关闭态在三个任务间不对称。风险：数值动作没有阶段时限，关闭后仍会驱动载具。触发条件：计划里出现 `basic_motion`，或同一会话内 `close()` 之后再次 `run`。
2. 视觉与巡航的连续实现各有一份 `_submit` 与 `_publish`，`timeline.at` 失败后的补锚点分支逐行相同（`planner/workflows/visual_goal_navigation/continuous.py:175-221`、`planner/workflows/free_navigation/continuous.py:235-277`）。生产路径两份都活，暂无死路。
3. `segment_motion` 的三个名字是死路，`planner/runtime/__init__.py:21` 仍导出 `plan_segment`、`quintic`、`duration_for`；生产用的是 `synchronized.duration_for`（`planner/runtime/motion/__init__.py:2`，调用点 `planner/workflows/basic_motion/workflow.py:25`）。任务侧不存在第二份 `duration_for`。
4. 停滞阈值 `0.2 m`、`2.0°`、`1e-6 m`、`1e-6°` 都是经验值，未标定，也没有配置出处。影响：低速或高噪声工况下 `STUCK` 会误报或漏报。
5. 巡航停稳回退分支没有重试与计划数上限，给规划器传 `budget=None`（`planner/workflows/free_navigation/workflow.py:238-258`）。触发条件：窗口未启用且使用该回退路径。
6. 巡航连续路径用 `AssertionError` 表达"不该返回"，随后被 `except Exception` 收成 `BLOCKED`（`planner/workflows/free_navigation/workflow.py:218-224`、`planner/workflows/free_navigation/workflow.py:285-301`）。
7. 测试空白：`VisualGoalContinuous` 无任何测试引用，`_check_pause`、`_submit`、`_sync_remaining` 未覆盖；视觉工作流的连续分支（`planner/workflows/visual_goal_navigation/workflow.py:143`、`planner/workflows/visual_goal_navigation/workflow.py:206-207`、`planner/workflows/visual_goal_navigation/workflow.py:270-271`）无测试；`basic_motion` 的 `prepare`/`close` 无断言；巡航 `_publish` 的日志分支未断言。
