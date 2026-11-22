# 任务层 workflows：文件清单、契约与迁移

任务层最终是 19 个文件、约 2110 行：公共编排 709 行，三个任务包合计 1404 行（现 `planner/workflows/` 加散在 `capabilities/` 的任务专属判定共约 1990 行）。三个任务包各自导出 `TASK_KIND` 与 `REQUIRED_PORTS`，公共编排收在 `workflows/common/`，任务包之间零 import。目标接近的停稳分批与运动中连续两种模式靠三个可替换协作者区分，任务包里同一段判定只写一次。

## 1 职责与文件清单

本层只做三件事：把一份 `MissionStage` 的参数变成对规划器的请求，把执行层返回的实测变成完成证据，把失败翻译成本任务的失败状态。移动怎么算、谁在什么时候占载具，都不是本层的事。

| 文件 | 类与函数签名 | 估计行数 | 从哪来 |
| --- | --- | --- | --- |
| `workflows/__init__.py` | 无导出，只写包说明 | 6 | 现 `workflows/__init__.py:10`，删三段 eager import（三个包必须能被单独 import，eager 会破坏零依赖测试） |
| `workflows/base.py` | `Workflow(Protocol)`：`prepare() / run(stage) / close()`；`TaskDeclaration`；`declaration_of(package) -> TaskDeclaration` | 30 | 现 `workflows/base.py:8-10` 加两个新符号 |
| `workflows/common/__init__.py` | 包说明 | 3 | 新建 |
| `workflows/common/budget.py` | `StageBudget`：`start(stage, limits)`、`remaining_s(clock)`、`require_time(clock)`、`count_model_call(clock)`、`unreliable_observation(ambiguous=False)`、`record(receipt)`、`stop`、`remaining_motion` | 120 | 现 `visual_goal_navigation/session.py:68` 的 `require_time`、同文件 `:84` 的 `model_call`、`visual_goal_navigation/termination.py` 的调用计数、`free_navigation/workflow.py` 的各参数校验 |
| `workflows/common/evidence.py` | `StageEvidence`（frozen dataclass）、`summary_for(evidence)`、`result_for(evidence)`、`require_confirmed(result, stage)`、`MEASUREMENT_NAMES` | 110 | 现 `application/mission_runner.py:20-45` 的结果检查，加 `domain/result.py:28,49` 的构造样板 |
| `workflows/common/loop.py` | `TaskCycle(Protocol)`：`begin(ctx) / round(ctx) / finish(ctx)`；`CycleContext`；`RoundOutcome`；`TaskLoop(cycle, budget, clock, sleep, checkpoint, event)`、`TaskLoop.drive(context) -> RoundOutcome` | 140 | 三个 `workflow.py` 里重复的 while 骨架（`visual_goal_navigation/workflow.py:285`、`free_navigation/workflow.py:237`、`basic_motion/workflow.py:58`） |
| `workflows/common/streaming.py` | `ObservationFeed(Protocol)`、`StoppedFeed`、`MovingFeed`；`MotionChannel(Protocol)`、`SegmentChannel`、`StreamChannel`；`ChannelReport` | 300 | `visual_goal_navigation/{continuous.py,approach.py,flight.py}` 与 `free_navigation/continuous.py` 的 tick 与下发骨架，去掉两处镜像 |
| `workflows/basic_motion/__init__.py` | `TASK_KIND`、`REQUIRED_PORTS`、`BasicMotionWorkflow`、`StageAdmission` | 10 | 现 `basic_motion/__init__.py:2` |
| `workflows/basic_motion/workflow.py` | `BasicMotionWorkflow(vehicle, motion, control_loop, execution, *, clock, limits, telemetry)`；`prepare/run/close` | 150 | 现 `basic_motion/workflow.py:161` |
| `workflows/basic_motion/policy.py` | `StageAdmission.validate(stage)`；`measurements(result)` | 45 | 现 `capabilities/task_validation/basic_motion.py:11` 加结果字段表 |
| `workflows/visual_goal_navigation/__init__.py` | `TASK_KIND`、`REQUIRED_PORTS`、`VisualGoalNavigationWorkflow`、`StageAdmission`、`select_channel` | 12 | 现 `visual_goal_navigation/__init__.py:4` |
| `workflows/visual_goal_navigation/workflow.py` | `VisualGoalNavigationWorkflow(observation, moving_capture, goal_observer, planner, validator, control_loop, execution, geometry, completion, limits, *, trigger_rules, sliding_window, motion_limits, continuous_tracking, trace_factory, clock, sleep, telemetry, vertical_motion)`；内部 `VisualGoalCycle` | 240 | 现 `visual_goal_navigation/workflow.py:414`，连续分支移出 |
| `workflows/visual_goal_navigation/policy.py` | `StageAdmission.validate(stage)`；`select_channel(streaming, moving_capture, sliding_window) -> Channel`；`VisualGoalExitPolicy`；`SearchPolicy` | 160 | `capabilities/task_validation/visual_goal_navigation.py:26`、`visual_goal_navigation/termination.py:109`、`capabilities/approach/target_approach.py:28` |
| `workflows/visual_goal_navigation/targeting.py` | `TargetTracker(observer, geometry, limits, clock, event)`：`begin/poll/request/submit/accept/close` | 175 | `capabilities/targeting/visual_goal_refresh.py:132` 与 `bbox_goal_observation.py:33`，线程部分改调 `runtime/background_operation.py` |
| `workflows/visual_goal_navigation/completion.py` | `CompletionGate(completion, verification, trigger_rules, limits, clock)`：`should_pause(pose)`、`candidate(...)`、`confirm(...)`、`verify(...)`、`measurements(...)` | 190 | `capabilities/completion/` 五个文件共 175 行加 `flight.py:23` 的 `verify` |
| `workflows/free_navigation/__init__.py` | `TASK_KIND`、`REQUIRED_PORTS`、`FreeNavigationWorkflow`、`StageAdmission` | 12 | 现 `free_navigation/__init__.py:5` |
| `workflows/free_navigation/workflow.py` | `FreeNavigationWorkflow(observation, moving_capture, planner, validator, control_loop, execution, *, limits, sliding_window, motion_limits, stream_speed_mps, stream_waypoint_timeout_s, stall_timeout_s, continuous_tracking, trace_factory, clock, sleep, telemetry, vertical_motion)`；`prepare/run/close` | 150 | 现 `free_navigation/workflow.py:403`，删停稳批次回退与车道先验 |
| `workflows/free_navigation/cruise.py` | `CruiseCycle(window, feed, channel, instruction, stall, path)`：`begin/round/finish` | 200 | 现 `free_navigation/continuous.py:444`，删 `_recover_from_collision`(`:312`) 与 `_lane_context`(`:435`) |
| `workflows/free_navigation/policy.py` | `StageAdmission.validate(stage)`；`StallWatch(limit_s, clock)`：`observe(distance_m)`、`check()` | 60 | `capabilities/task_validation/free_navigation.py:13` 加 `free_navigation/continuous.py:251` 的停滞判定 |

删除：`visual_goal_navigation/{approach.py,flight.py,session.py,continuous.py}`、`free_navigation/continuous.py`、`capabilities/{approach,completion,task_validation}/`。最后三个目录按 `docs/REFACTOR.md` 的约定并入各任务包的 `policy.py` 与 `completion.py`。

`common/streaming.py` 的三个类型就是两种模式的全部差异载体：

```python
class ObservationFeed(Protocol):
    def begin(self) -> None: ...
    def next(self) -> Observation: ...             # 停稳：取一帧并验 stopped；运动中：等下一帧落地
    def poll(self) -> Observation | None: ...       # 运动中：非阻塞取一帧
    def discard(self) -> None: ...
    def close(self) -> None: ...

@dataclass(frozen=True)
class ChannelReport:
    measured: TimedVehicleState | None
    distance_m: float
    paused_reason: str | None = None
    receipt: ExecutionReceipt | None = None

class MotionChannel(Protocol):
    def load(self, motion: MotionTrajectory) -> None: ...
    def send(self, *, pause_when: Callable[[WorldPose], str | None] | None = None) -> ChannelReport: ...
    def stop(self) -> None: ...
    def remaining(self) -> MotionTrajectory: ...
```

`SegmentChannel.load` 收下已授权的整条轨迹，`send` 逐段调 `run_segment`；`StreamChannel.load` 是空操作（航点在 `PlanningWindow` 的队列里），`send` 调一次 `run_stream`。两个 sink 是同一签名的别名：`stream_command(command: MotionCommand, duration_s: float) -> None`（裁决 R2）；段执行的采样循环在 `runtime/motion/loop.py`，引擎不再有 tracker（裁决 R1）。`visual_goal_navigation` 连续模式同时持有两个 `ObservationFeed`：`MovingFeed` 供循环取帧，`StoppedFeed` 供暂停后的确认取帧。

`planner/workflows/` 只 import `domain/`、`ports/`、`algorithms/`、`runtime/`、以及本包内的 `workflows.common`。`base.py` 除 `domain/mission.py` 外不 import 任何东西。

## 2 数据流与时序

通道判据三条，都能写成测试。

| 任务 | 判据 | 理由 | 可检验形式 |
| --- | --- | --- | --- |
| 数值动作 | 恒段执行 | 用户已说清动作，路径一段到位，没有重规划 | `basic_motion/**` 源码不出现 `run_stream` |
| 目标接近 | `CONTINUOUS` 当且仅当 `streaming` 与 `moving_capture` 两个槽位都非空且 `sliding_window.enabled`；否则 `STOPPED_BATCH` | 连续要同时具备速度流下发与运动中取帧；缺一项就退回停稳分批 | `select_channel(streaming, moving_capture, sliding_window)` 纯函数，8 组输入逐组断言 |
| 无终点巡航 | 恒速度流 | 无终点、路径只能边跑边给；`REQUIRED_PORTS` 已含 `streaming`，装配期就排除了跑不动的组合 | `free_navigation/**` 源码不出现 `run_segment` |

模式在构造期定下来，不在运行中切换：`VisualGoalNavigationWorkflow.__init__` 调一次 `select_channel`，算出哪个就用哪个；运行中端口消失不是本层处理的事，端口生命周期由会话拥有。

生命周期与 MissionRunner 的契约。

| 环节 | 约束 |
| --- | --- |
| `prepare()` | 幂等；不发指令、不取帧、不调模型；只检查端口与资源可用（模型调用服务空闲、滑窗 worker 可用、`moving_capture` 在连续模式下存在）。失败抛 `ProtocolError` 或 `ExecutionError`，这是构造期之外的第二次准入 |
| `run(stage)` | 首行自己调一次 `prepare()`，不依赖 runner 已经调过；只处理一个 stage；任何退出路径都返回 `StageResult`，用户停止与失败都是 `success=False` |
| `close()` | 幂等；置 `_closed`；之后 `run`/`prepare` 抛 `ExecutionError`。worker 线程与规则见第 5 节 |
| `MissionRunner` | 按 `plan.stages` 顺序调用 `workflow_for(stage).run(stage)`，首败即停；不认识 `stage.parameters` 的具体类型（那是 `StageAdmission` 的事）；用 `evidence.require_confirmed(result, stage)` 校验返回形状 |
| `require_confirmed` | `stage_id` 必须等于 `stage.stage_id`；`success=True` 必须带 `summary.completion_confirmed=True` 且 `completion_basis is not NOT_VERIFIED`；任一不满足则改写为失败，reason 写 `"workflow returned no confirmed stage completion evidence or mismatched stage ID"`。这三条现在是 `application/mission_runner.py:26-42`，搬进 `common/evidence.py` |

段执行路径：数值动作，以及没有 streaming 端口时的目标接近。

一拍等于一段。数值动作只有一段，目标接近一段一拍直到批次耗尽。

```
run(stage)
 ├ prepare()
 ├ execution.acquire_flight(self)                     # 唯一 owner
 ├ execution.set_deadline(budget.deadline_monotonic)
 ├ loop = TaskLoop(cycle, budget, ...)
 └ cycle.round(ctx) × N
     ├ observation = feed.next()                     # StoppedFeed: source.capture()
     ├ budget.require_time(clock)                     # 每次取帧前后各一次
     ├ targeting.bind(observation, params)            # 只在 target_state 为空时调模型
     ├ completion.candidate(geometry, ...) → 命中则 confirm()×N → verify()
     ├ plan_once(): planner.plan(...) → validator.validate_relative → algorithms.trajectory_transform.relative_to_world → validator.validate_world
     ├ channel.load(MotionTrajectory(segments))
     └ channel.send()                                 # 逐段 ControlLoop.run_segment
         ├ loop.run_segment(segment, loop.stream_sink(), budget=ExecutionBudget(stage_deadline))   # 到期抛 BudgetExceeded（裁决 R3）
         ├ 段间查 completion.should_pause(pose)；命中则停，remaining 写进 budget
         └ 段末 receipt → budget.record(receipt)，evidence 记实测
```

数值动作唯一差别：段由 `algorithms.basic_motion_conversion.BasicMotionConverter.create_target` 直接算出来，没有观测、没有规划器、没有完成判定，跑完比对实测位姿与目标位姿。

速度流路径：目标接近连续模式与无终点巡航。

一拍等于一个补点批次。`run_stream` 只在 `QueueFollower` 返回 `None`（队列耗尽或航点超时）时正常返回，`deadline_monotonic` 到期抛 `BudgetExceeded`（裁决 R3）。控制率仍由 `ControlLoop` 决定。

```
run(stage)
 ├ prepare()
 ├ execution.acquire_flight(self)
 ├ execution.set_deadline(deadline | None)            # 巡航为 None
 ├ window.begin(deadline_monotonic=..., maximum_rounds=limits.maximum_planning_rounds)
 ├ goals = QueueFollower(window.queue_view(), pause_when=self._pause_reason,                # 裁决 R4
 │                       waypoint_timeout_s=stream_waypoint_timeout_s)
 └ loop.drive(ctx)  # 一拍 = 一个补点批次
     ├ channel.send()
     │   └ loop.run_stream(goals, loop.stream_sink(), deadline_monotonic=stage_deadline)   # 裁决 R3
     ├ window.poll_stream(channel.measured, execution.checkpoint)  # True = 新一批已授权
     ├ feed.poll() → 新帧 → targeting.submit(frame)                  # 运动中取帧，运行期线程在 runtime
     ├ 目标接近：pause_when 返回理由 → window.pause_for_completion(checkpoint)
     │     → execution.cancel() → stopped.next() → confirm ×N → verify()
     │     → True: window.complete()；False: window.submit(...) 继续
     ├ 巡航：stall.observe(channel.distance_m) → 超时抛 FreeNavigationStalled
     └ budget.require_time(clock)                    # 超时抛 BudgetExceeded
```

时间尺度：`ControlLoop` 内部按 20 Hz 下发 `MotionCommand` + `duration_s`，段执行按 30 ms 采样五次多项式。任务层只在批次边界拿到控制权，不决定控制率。两个周期常量 `SEGMENT_PERIOD_S = 0.030` 与 `1/MotionLimits.control_hz` 写在 `runtime/motion/loop.py`，不进配置（裁决 R9）。

巡航的 `round()` 去掉车道先验后只剩三件事：提交、推进、查停滞。规划上下文一律 `None`。

```python
# workflows/free_navigation/cruise.py
class CruiseCycle:
    def round(self, ctx) -> RoundOutcome:
        report = self.channel.send(pause_when=None)
        self.window.poll_stream(report.measured, ctx.checkpoint)
        frame = self.feed.poll()
        if frame is not None:
            self.window.submit(frame, None, self.instruction, ctx.window_input())
        self.stall.observe(self.channel.distance_m)
        return RoundOutcome()          # 无完成条件；用户停止与本地检查失败之外永不返回 done
```

`task_context` 不再构造，`lane_reference_provider` 参数与 `_lane_context` 一起删。规划器只收到图像、扇面与指令；提示词侧的 `format_lane_reference` 在 `adapters/trajectory_planner/prompts/free_navigation.py`，由适配层方案删除。

## 3 跨层接口

给相邻层的东西。

给装配层（`assembly/`）：

```python
# planner/workflows/<package>/__init__.py —— 位置：包入口顶层字面量
TASK_KIND: TaskKind = TaskKind.FREE_NAVIGATION
REQUIRED_PORTS: tuple[str, ...] = ("vehicle", "observation", "streaming")

# planner/workflows/base.py
@dataclass(frozen=True)
class TaskDeclaration:
    kind: TaskKind
    required_ports: tuple[str, ...]
    package: str

def declaration_of(package: ModuleType) -> TaskDeclaration: ...
class Workflow(Protocol):
    def prepare(self) -> None: ...
    def run(self, stage: MissionStage) -> StageResult[object]: ...
    def close(self) -> None: ...

# planner/workflows/common/evidence.py
def require_confirmed(result: StageResult[object], stage: MissionStage) -> StageResult[object]: ...
```

用法三条，写进测试：

1. `assembly/requirements.py` 从一张显式元组 `EXECUTABLE_PACKAGES` 里逐个 import 任务包的 `__init__`，只读 `TASK_KIND` 与 `REQUIRED_PORTS`，生成 `TASK_REQUIREMENTS`（替掉 `planner/simulator/ports.py:44-48` 的表）。不做自动发现。
2. `assembly/workflow_router.py` 用 `TASK_KIND` 建路由表，不写 `TaskKind.X → 某类` 的字面映射。
3. 任务包自己不读回这两个常量，也不把槽位名映射成端口对象；槽位名到对象的解析在装配层。`declaration_of` 只校验形状：`kind` 是 `TaskKind`、`required_ports` 是非空 `tuple[str, ...]` 且首项为 `"vehicle"`。槽位名是否等于 `SimulatorPorts` 的字段名，由 `assembly/requirements.py` 交叉校验。

给装配层的构造参数只有上表列出的那些。三个工作流的构造器接受端口对象、`ControlLoop`、`ExecutionService` 与配置读出来的值，不接受配置对象本身。连续模式需要的 `PlanningWindow` 在构造期按 `select_channel` 的结果建一次，`run()` 里只调 `begin`/`end`；`basic_motion` 不建窗口。

要求相邻层提供的东西。

契约层 `domain/`：

- 类型：`TaskKind`、`MissionStage`、`StageResult`、`StageSummary`、`CompletionBasis`、`ResultMeasurement`、`ExecutionBudget`（`:16`）、`ExecutionReceipt`（`:32`）、`StopResult`（`:44`）、`MotionSegment`、`MotionTrajectory`、`RelativeTrajectory`、`MotionLimits`、`ContinuousTrackingSettings`、`SlidingWindowSettings`、`CollisionRecoverySettings`、`TimedVehicleState`（`domain/streaming.py:12`）、`MotionCommand`、`MotionCapability`、`CollisionState`、`Observation`（`domain/observation.py:48`）。
- 参数与状态枚举：`BasicMotionParameters`/`BasicMotionResult`/`BasicMotionStatus`、`VisualGoalNavigationParameters`/`VisualGoalNavigationResult`/`VisualGoalStatus`、`FreeNavigationParameters`/`FreeNavigationResult`/`FreeNavigationStatus`、`VisualGoalLimits`、`VisualGoalTriggerRules`、`VisualGoalCompletionInput`、`TargetPositionState`。
- 异常：`planner/errors.py` 的 `ProtocolError`、`ObservationError`、`ExecutionError`、`ModelCallTimeout`、`BudgetExceeded`、`FreeNavigationStalled`。

端口 `ports/`：`Vehicle`（`current_pose`、`motion_capability`、`execute_segment`、`cancel`、`hover`、`collision_state`）、`StreamingVehicle`（`stream_state`、`stream_command`）、`ObservationSource` 等。`Vehicle` 槽由 runtime 的 `GuidedVehicle(streaming, loop)` 组合对象兑现，引擎只实现 `stream_command(command, duration_s)` 与 `current_pose`/`motion_capability`/`collision_state`/`cancel`/`hover`（裁决 R1），过渡期保留三行委派、最终删除。`ObservationSource`、`ControlledObservationSource`、`MovingObservationSource`、`GoalObserver`、`TrajectoryPlanner`、`TaskCompletionChecker`、`TaskTrace`。

计算层 `algorithms/`：`BasicMotionConverter.create_target`、`TrajectoryValidator.validate_relative/validate_world`、`TrajectoryTransform.relative_to_world/world_to_relative`、`bound_target_geometry`/`bound_target_goal`、`point_in_horizontal_body`。本层不做几何，也不做限幅。

执行层 `runtime/`，这是本层最依赖的一块：

| 我用的名字 | 要求 |
| --- | --- |
| `ControlLoop.run_segment(segment, sink, *, budget) -> ExecutionReceipt` | 逐段下发五次多项式，30 ms 采样；到期抛 `BudgetExceeded`；检查点抛 `ExecutionError` |
| `ControlLoop.run_stream(goals, sink, *, deadline_monotonic) -> None` | 20 Hz 伺服；`goals.next(state)` 返回 `None` 时正常返回；`deadline_monotonic` 到期抛 `BudgetExceeded`（裁决 R3），任务层捕获后映射 `TIMEOUT`；**可重复进入**：跟随器、时间线、队列游标跨调用保持 |
| `ControlLoop.segment_sink() / stream_sink()` | 两个名字同一签名 `stream_command(command: MotionCommand, duration_s: float) -> None`（裁决 R2）；段执行的采样循环在 `runtime/motion/loop.py`，引擎无 tracker（裁决 R1）；任务层从 loop 取 sink 传回 loop，不直接调 `vehicle.execute_segment` |
| `PlanningWindow.begin/submit/poll_stream/pause_for_completion/complete/end` | 任务层只调这六个。`poll_stream(measured, checkpoint) -> bool`，`True` 表示"新一批已授权写进队列"；`submit(observation, context, instruction, planning) -> None`。窗口另外只提供队列视图与 `consume`，不实现 `StreamGoalSource`、不承担伺服（裁决 R4） |
| `QueueFollower`（`runtime/motion/follower.py`） | 实现 `StreamGoalSource`：队列消耗、航点超时、`pause_when` 谓词。任务层构造它并传给 `run_stream`（裁决 R4），半径暂停仍由任务层判 |
| `ExecutionService.acquire_flight/release_flight/stop_if_owner/checkpoint/set_deadline/request_stop/current_pose/collision_state` | 控制权唯一；`checkpoint()` 只管停止与预算，不管碰撞。碰撞检查挂在 `ControlLoop._guard`，由 `CollisionRecovery.poll` 触发（裁决 R5）；`acquire_flight` 不再有 `collision_check` 参数。碰撞开关只有 `config` 的 `motion.collision.enabled`，默认 `true`，任务层不传（裁决 R8） |
| `MovingObservationSource`（`runtime/moving_observations.py`） | 提供 `begin()`、`request(budget)`、`poll() -> Observation \| None`、`discard()`、`job.busy`、`close()`；取帧线程在 runtime |
| `BackgroundOperation`（`runtime/background_operation.py`） | 目标刷新单飞；结果不自动授权，任务层显式 `accept` |
| `ModelCallService`（`runtime/model_calls.py`）与 `ExecutionBudget` | 单飞模型调用加超时；调用计数由 `common/budget.py` 记 |

本层不 import `assembly/`、`simulators/`、`adapters/`，不出现引擎名，不出现 `MotionModel`/`ACKERMANN`/`HOLONOMIC`。

## 4 错误与失败路径

| 情形 | 谁抛 | 谁捕获 | 结果 |
| --- | --- | --- | --- |
| 用户停止（`request_stop`） | `runtime/execution.py` 的检查点，抛 `ExecutionError`，带 `stop_result` | `TaskLoop.drive` | 任务包映射成 `CANCELLED`；`StageEvidence(completion_confirmed=False)`；`StageResult(False)` |
| 时间预算耗尽 | `ControlLoop`（`ExecutionBudget`）与 `budget.require_time`，抛 `BudgetExceeded` | `TaskLoop.drive` | 映射成 `TIMEOUT`；`budget.remaining_motion` 写进结果与 `StageSummary.final_pose` |
| 模型超时 | `runtime/model_calls.py`，抛 `ModelCallTimeout` | `TaskLoop.drive` | 重试到 `model_retry_count` 用尽后映射成 `TIMEOUT`；计入调用预算 |
| 观测不新鲜 / 未停稳 | 任务包 | 任务包自己 | 重置确认计数，`budget.unreliable_observation()`；累计到 `maximum_unreliable_observations` 抛任务内失败（视觉目标 `PERCEPTION_UNRELIABLE`/`TARGET_AMBIGUOUS`） |
| 参数或规划器输出非法 | `StageAdmission.validate`、`TrajectoryValidator`，抛 `ProtocolError` | 任务包 | `MODEL_ERROR`（参数非法时 `BLOCKED`），不执行任何段 |
| 碰撞接触 | `ControlLoop._guard` 上的 `CollisionRecovery.poll` 取 `CollisionState`（裁决 R5） | 任务层不捕获 | 恢复后继续；次数超限抛 `ExecutionError`，任务包映射成 `EXECUTION_ERROR` |
| 巡航无进展 | `StallWatch.check`，抛 `FreeNavigationStalled` | `TaskLoop.drive` | `STUCK` |
| `close()` 后调用 | 任务包，抛 `ExecutionError` | 装配层 / worker | 会话拆除顺序错误暴露在拆除时，不静默 |

三条固定语义：

1. 任务包不吞异常。`TaskLoop.drive` 只捕获上表四类（`ExecutionError`、`BudgetExceeded`、`ModelCallTimeout`、任务内失败类型），先捕子类再捕父类（`BudgetExceeded` 与 `ModelCallTimeout` 都继承 `ExecutionError`），其余原样上抛，由 worker 记成未预期错误。
2. 失败也带证据。`StageEvidence` 在 `finally` 里构造，至少带 `elapsed`、`execution_mode`、`stop_confirmed` 三项测量；`completion_confirmed` 一律 `False`，`completion_basis` 一律 `NOT_VERIFIED`（`domain/result.py:45` 会拒绝确认与依据不符的组合）。
3. `KeyboardInterrupt` 不转成 `StageResult`。任务包在 `finally` 里 `release_flight` 与 `window.end()`，然后重新抛出，进程级停止由 controller 处理。

完成证据的形状与 `StageResult` 构造逐任务固定，`evidence.result_for` 是唯一构造入口：

| 任务 | `status` 取 | `success` | `completion_basis` | `measurements` | `value` |
| --- | --- | --- | --- | --- | --- |
| 数值动作 | `BasicMotionStatus` | `COMPLETED` 且实测位姿在容差内 | `LOCAL_MEASUREMENT` | `elapsed`、`translation`、`yaw_delta`、`stop_confirmed` | `BasicMotionResult` |
| 目标接近 | `VisualGoalStatus` | `REACHED` 且 `completion_confirmed` | `MODEL_WITH_LOCAL_CHECKS` | `distance`、`elapsed`、`planning_calls`、`stop_confirmed`、`execution_mode` | `VisualGoalNavigationResult` |
| 无终点巡航 | `FreeNavigationStatus` | 恒 `False` | `NOT_VERIFIED` | `planning_calls`、`elapsed`、`traveled`、`stop_confirmed`、`execution_mode` | `FreeNavigationResult` |

`execution_mode` 取 `stopped_segments` 或 `continuous_world_queue`，由本阶段实际用到的通道写，不由任务名推断。`StageResult.summary` 与 `value` 必须同时存在，`summary.final_pose` 取最后一次实测位姿（失败时可以是 `None`，不为它单独取帧）。

## 5 并发与所有权

**线程。** 任务包运行在 worker 线程上，该线程就是载具 owner：`run()` 首行 `execution.acquire_flight(self)`，`finally` 里 `execution.release_flight(self, pending_cleanup=...)`。任务包自己不起线程。三个后台动作都有 runtime 承载者：`runtime/window/` 的模型 worker、`runtime/moving_observations.py` 的取帧线程、`runtime/background_operation.py` 的单飞操作。任务包只轮询。

**控制权交接。** 观测采集与位置保持是临时 owner，由 `runtime/execution.py` 在检查点上做停稳与确认，任务包不感知。切载具只在 idle 发生，且每台载具一个 `VehicleArm`（`application/mission_assembly.py:140`：`VehicleArm` 持有 `vehicle`、`execution`、`application`、`workflows`），所以切换是装配层换 arm，旧的 workflow 不参与交接、不 `close`、不重建。`close()` 只在会话拆除时对全部 arm 各调一次。

**实例寿命。** 一个 arm 一个 workflow 实例，实例跨多个 stage 复用，跨 stage 的可变状态必须清零。三个工作流的 `run()` 首行调用 `budget.start(stage, ...)` 与 `cycle.reset()`；`VisualGoalExitPolicy`、`SearchPolicy`、`StallWatch`、`TargetTracker` 的每次运行状态都在 `run()` 里重建，不留在实例字段上（现在是 `visual_goal_navigation/workflow.py:138` 的写法：每轮 `run` 新建 `VisualGoalSession`）。

**关闭顺序。** `close()` 的调用顺序固定：`TargetTracker.close` → `MovingFeed.close` → `window.close` → `verification.calls.close` → `model_calls.close`，用 `runtime/resource_cleanup.close_resources` 逐个尝试、失败聚合上报。

**故障隔离。** 一次 `run` 结束前必须让所有在途请求落地或作废：`budget` 记录 `remaining_motion`，`feed.discard()`、`targeting.discard()`、`window.end()`。理由写在可测处：`window` 关闭后仍有 worker 在跑，会让下一 stage 读到上一 stage 的轨迹。

## 6 测试点

单测清单，断言对象写具体。

生命周期与契约：

1. `test_workflow_lifecycle.py`：`prepare()` 期间 fake `ControlLoop` 的 `run_segment`/`run_stream` 调用计数为 0，fake `ObservationSource.capture` 计数为 0，fake 规划器调用计数为 0。负例：`prepare()` 时模型调用服务忙 → `ExecutionError`。
2. `test_mission_runner_contract.py`：fake workflow 返回 `success=True` 且 `summary=None` → 被改写为失败，reason 含 `"no confirmed stage completion evidence"`；`stage_id` 不匹配 → 同样失败；`success=False` 且 `summary.completion_confirmed=True` → 被 `StageSummary.__post_init__` 拒绝（`ValueError`）。
3. `test_workflow_close.py`：`close()` 两次不抛；`close()` 后 `run` 抛 `ExecutionError`。

通道选择（判据可检验）：

4. `test_channel_selection.py`：`select_channel` 的 8 组输入（`streaming ∈ {有, 无}` × `moving_capture ∈ {有, 无}` × `sliding_window.enabled ∈ {True, False}`）逐一断言，`CONTINUOUS` 只在三者齐备时出现。
5. `test_channels_are_not_mirrored.py`：AST 扫 `planner/workflows/`——`basic_motion/**` 不出现 `run_stream`，`free_navigation/**` 不出现 `run_segment`；`visual_goal_navigation/workflow.py` 里 `run_segment` 与 `run_stream` 各出现一次；`round()` 函数体内 `if self.window is None` 的分支不超过 3 处。
6. `test_task_packages_no_motion_model.py`：AST 扫三个任务包，不出现 `MotionModel`、`HOLONOMIC`、`ACKERMANN`、`motion_capability`。

零依赖与分层：

7. `test_workflows_layering.py`：每个任务包 import 的模块集合 ⊆ `{planner.domain.*, planner.ports.*, planner.algorithms.*, planner.runtime.*, planner.workflows.common.*}`；任意两个任务包互相 import 计数为 0；`workflows/common/**` 的源码字符串里不出现 `visual_goal`、`free_navigation`、`basic_motion`、`cruise`。
8. `test_task_declaration.py`：三个包 `declaration_of` 返回的 `kind` 与 `required_ports` 等于表值；负例：删掉 `REQUIRED_PORTS` 的 fake 包 → 抛 `ProtocolError`；`required_ports` 首项不是 `"vehicle"` → 抛 `ProtocolError`；`assembly/requirements.py` 生成的表与 `SimulatorPorts` 字段名交叉校验，出现未知槽位名即失败。

证据形状：

9. `test_stage_evidence.py`：三个任务的 `result_for(evidence)` 字段逐一断言；`success=True` 必带非空 `measurements` 与 `completion_basis is not NOT_VERIFIED`；失败时 `final_pose` 可空但 `execution_mode` 与 `stop_confirmed` 必在。
10. `test_free_navigation_never_confirms.py`：巡航返回值恒 `success=False`、`completion_basis is NOT_VERIFIED`，用户停止时 `status` 为 `CANCELLED` 且 `stop_confirmed` 记入测量。

失败路径（负例）：

11. `test_budget_exhaustion.py`：fake clock 推过 deadline → `BudgetExceeded` → `TIMEOUT`，结果里 `remaining_motion.segments` 非空，`StageSummary.final_pose` 等于最后一次实测位姿。
12. `test_unreliable_observation.py`：连续两帧 `capture_id` 相同 → 确认计数清零、`unreliable` 加一；越过上限 → `PERCEPTION_UNRELIABLE`。
13. `test_invalid_planner_output.py`：规划器给出 4 个航点（配置要求 5）→ `TrajectoryValidator` 抛 `ProtocolError` → `MODEL_ERROR`，`ControlLoop` 调用计数为 0。
14. `test_no_task_level_collision_recovery.py`：AST 扫三个任务包，不出现 `backoff`、`maximum_collision_recoveries`、`reverse`。

车道先验删除：

15. `test_no_lane_prior.py`：`planner/workflows/**` 全文不含 `lane`；`FreeNavigationWorkflow` 的构造参数里没有 `lane_reference_provider`；对规划器的调用 kwargs 里 `task_context is None`（用 fake 规划器记录 kwargs 断言）。

## 7 迁移步骤

每步独立可验证，前一步不绿不进下一步。

1. **建 `common/budget.py` 与 `common/evidence.py`。** 把 `session.py:68,84` 的 `require_time`/`model_call` 与 `mission_runner.py:26-42` 的检查搬进去，三个任务包改调新符号，`mission_runner.py` 改为调 `evidence.require_confirmed`。验收：`pytest tests/test_workflow.py tests/test_visual_goal_parameters.py -q` 全绿，`application/mission_runner.py` 降到 20 行以内。
2. **加任务包声明。** 三个 `__init__.py` 写 `TASK_KIND`/`REQUIRED_PORTS`，加 `base.declaration_of`；`assembly/requirements.py` 读声明生成表，与 `simulator/ports.py:44-48` 的旧表并行比对一次（临时断言两表相等），比对通过后删旧表。验收：`pytest tests/test_capability_gating.py tests/test_runtime_catalog.py -q` 全绿。
3. **抽 `common/loop.py`，先接 `basic_motion`。** 数值动作改走 `TaskLoop`，`run_segment` 由任务包调 `ControlLoop`。验收：`pytest tests/test_basic_motion.py -q`；UE5 跑一次数值动作，`/api/trajectory` 输出与重构前一致。
4. **抽 `common/streaming.py` 的 `SegmentChannel`，接目标接近停稳模式。** 删 `approach.py`、`flight.py` 的段执行簿记，`session.py` 剩下的字段并入 `CycleContext`。验收：`pytest tests/test_visual_goal_anchor.py tests/test_visual_goal_collision_recovery.py -q`；UE4 与 UE5 各跑一次目标接近（停稳模式）。
5. **接 `StreamChannel` 与目标接近连续模式。** 前提是 `runtime/collision_recovery.py` 已落地（`docs/REFACTOR.md` 阶段 2），否则第 6 步会留下半套退避。删 `visual_goal_navigation/continuous.py` 的 tick 与 `_recover_from_collision`(`:259`)，目标刷新改调 `runtime` 的 `BackgroundOperation`。验收：连续模式跑一次目标接近，`telemetry` 的控制采样条数与旧版相差不超过 1 拍。
6. **重写 `free_navigation`。** 删停稳批次回退（`workflow.py:237` 的那层 while）、`_lane_context`(`:376`)、`lane_reference_provider`(`:76`)、`continuous.py` 的 `_recover_from_collision`(`:312`) 与 `_lane_context`(`:435`)；`cruise.py` 只保留速度流一条路径；`mission_assembly.py:306` 的 `lane_reference_provider=None` 一并删。验收：`pytest tests/test_free_navigation.py -q` 全绿；UE5 巡航实跑，中途制造一次接触，确认后退—清队列—继续由执行层完成。
7. **收口。** 删 `capabilities/{approach,completion,task_validation}/` 与 `workflows/{visual_goal_navigation/{approach,flight,session,continuous}.py,free_navigation/continuous.py}`；加第 6 节全部测试；`workflows/__init__.py` 去掉 eager import。验收：`pytest tests/ -q` 全绿，用例数不减。

## 8 未决问题

1. **批次模式怎样拿到已授权的航点。** 默认：批次模式完全不用 `PlanningWindow`，任务包直连 `planner.plan` 并发 `ControlLoop.run_segment`，因为"一批确定航点、期间不重规划"没有补点需求。备选：给 `PlanningWindow` 加 `take_batch() -> MotionTrajectory`，让批次模式也走滑窗。取舍：默认方案少一个跨层方法、失败面更小，代价是批次模式没有后台预取，批次之间必然停稳；备选方案能预取，但要往契约里加第七个方法，且批次模式的"停稳取帧"语义与窗口的流式队列混在一起。
2. **`QueueFollower` 由谁构造、暂停判据放哪。** 裁决 R3/R4 已定：`run_stream` 的 deadline 到期抛 `BudgetExceeded`；`QueueFollower` 实现 `StreamGoalSource`，`PlanningWindow` 不承担伺服；任务层把 `pause_when` 交给 `QueueFollower`。剩下的选择是构造点：默认在任务包的 `StreamChannel` 里按 `window.queue_view()` 造，暂停判据与任务策略同处；备选在装配层造好注入，执行层可见全部 hook。取舍：默认方案让 `runtime/motion/follower.py` 不认识任务，代价是构造参数随任务策略增长；备选方案把任务策略搬进装配层，与"任务判定在任务包"冲突。
3. **段执行的暂停粒度。** 默认：段级，`SegmentChannel.send()` 在段与段之间查 `completion.should_pause(pose)`，命中即停，剩余段写进 `budget.remaining_motion`。备选：检查点级，给 `ControlLoop.run_segment` 加 `pause_when` 谓词，可在段中途停。取舍：默认方案零契约改动，代价是最长要跑完一整段才发现进了半径；备选方案暂停更及时，但要把停稳与残余段重锚逻辑搬进执行层。
4. **`moving_capture` 是否进 `REQUIRED_PORTS`。** 默认：不进，`free_navigation` 仍声明 `("vehicle", "observation", "streaming")`，与 `docs/REFACTOR.md` 的表一致；`moving_capture` 缺失在 `FreeNavigationWorkflow.__init__` 抛 `ProtocolError`（构造期失败）。备选：声明四槽位。取舍：默认方案保持与表一致、能力门更宽；备选方案让装配层在路由前就拒绝该任务，但要同步改能力门与 `enabled_tasks` 的既有语义。
5. **`TASK_KIND` 与 `REQUIRED_PORTS` 的位置。** 裁决 R10 已定：写在任务包 `__init__.py`，`assembly/requirements.py` 读它们生成表。剩下的是校验强度：默认 `base.declaration_of` 校验形状，装配期再交叉校验槽位名是否等于 `SimulatorPorts` 字段名；备选只在装配期校验。取舍：默认方案把错误提前到 import 任务包那一刻，代价是 `workflows/base.py` 多一个函数；备选方案少一层，槽位名写错要到填槽时才发现。
6. **`StageSummary.measurements` 的字段集是否统一。** 默认：每任务固定字段，`evidence.MEASUREMENT_NAMES` 给常量名，`basic_motion` 用 `elapsed/translation/yaw_delta/stop_confirmed`，`visual_goal` 用 `distance/elapsed/planning_calls/stop_confirmed/execution_mode`，`free_navigation` 用 `planning_calls/elapsed/traveled/stop_confirmed/execution_mode`。备选：统一成一套字段加 `task_specific` 子表。取舍：默认方案读到的人不必知道字段语义差异，代价是跨任务对账要按名字取；备选方案便于聚合，但把类型差异藏进字典，违背"完成要能复盘"。
7. **数值动作在阿克曼车上要不要保留五次多项式合成。** 默认：保留，段由 `BasicMotionConverter` 算位姿、`ControlLoop.run_segment` 下发，与无人机同一条路径。备选：车用原生驾驶原语（转向角加油门）。取舍：默认方案一条路径、一套限幅，代价是转弯的横向误差靠位置容差吸收；备选方案物理更真，但要在 `runtime/motion` 里加第二套下发分支，且段执行与速度流的限幅不再共用。
