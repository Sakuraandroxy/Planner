# 重构汇总方案

这是六份分层方案的合并结论与裁决记录。各层细节在 `01` 到 `06` 六份文件里，本文件只写层间协议、关键路径、跨层裁决与迁移顺序。

## 一、结论

- 代码分成六类：契约（`domain/`、`ports/`）、计算（`algorithms/`）、执行（`runtime/`）、任务（`workflows/`）、装配（`assembly/`）、入口（`adapters/http/`）；另加端口实现（`adapters/` 其余）、引擎实现（`simulators/<name>/`）与进程栈（`simulator/`）。
- `capabilities` 改 `algorithms`，`services` 改 `runtime`，`application` 改 `assembly`，`domain/runtime.py` 改 `domain/engine_family.py`。
- 运动控制只有一套律：五次多项式、刹车包络、纯跟踪、偏航伺服、限幅都在 `algorithms/`，控制循环与节拍在 `runtime/motion/loop.py`，引擎只实现"读状态、按命令推进一个周期"。
- 两条命令通道按命令性质分工：动作已确定、路径不再变化的走段执行（30 ms 采样）；滑窗持续补点、运动中取帧的走速度流（20 Hz 伺服）。
- 碰撞退避、滑窗、控制权、时间与停止是执行层的被动能力，任务层不编排。
- 可跑任务只由接口推导：任务包声明的必需槽位 ⊆ 引擎填上的槽位，没有第二个人工开关（R18）。
- 层间只经过包入口：每层一个公开入口，跨层只见名字不见实现（R19、R20）。
- 迁移分 8 个阶段，每阶段独立可发布、可回滚，前两阶段不改对外行为。

## 二、包与依赖

```
planner/
  domain/        契约      类型、常量、不变量、引擎能力位、运动模型
  ports/         契约      Protocol
  algorithms/    计算      几何、深度、校验、运动律、补点策略
  runtime/       执行      控制权、控制循环、队列、滑窗、碰撞退避、汇
  workflows/     任务      三类任务包 + 公共编排
  assembly/      装配      装配、路由、能力求交、控制器、配置装配
  adapters/      入口与端口实现
  simulator/     进程栈
  simulators/    引擎实现
```

依赖规则，全部写成测试：

| 编号 | 规则 | 例外 |
| --- | --- | --- |
| D1 | `domain/`、`ports/` 不 import 其他层 | `planner.errors` 是契约层的第三个模块，所有层可 import |
| D2 | `algorithms/` 只 import `domain/`、`ports/` | 当前不需要 `ports/` |
| D3 | `runtime/` 只 import `domain/`、`ports/`、`algorithms/` | 无 |
| D4 | `workflows/` 只 import 上述四层，任务包之间零 import | 无 |
| D5 | `assembly/` 可 import 全部，是唯一同时认识任务与引擎的位置 | 无 |
| D6 | `adapters/` 的端口实现只 import `domain/`、`ports/`、`algorithms/`；`adapters/http/` 只 import `assembly/`、`domain/` | 无 |
| D7 | `simulators/<name>/` 只 import `domain/`、`ports/`、`simulator/` 与自己的客户端库 | 无 |
| D8 | `simulator/` 不 import `simulators/` | 允许经 `assembly` 包入口取 `assemble_missions` 与 `reporting`（会话建立即接线、子进程上报），不许取 `controller` |
| D9 | `server_bootstrap.py` 是唯一同时 import `assembly` 与 `adapters/http` 的组合根 | 无 |
| D10 | 每层只有一个公开入口 `planner/<层>/__init__.py`；跨层 import 只写 `from planner.<层> import <名字>`，深层路径禁止 | 无 |
| D11 | 层的输入只有两类：契约层类型，或本层自己声明的 `Protocol` | 无 |
| D12 | 跨层只传契约类型与本层 Protocol 的实例，不传对方的具体类 | 无 |

## 三、跨层契约（终版）

```python
# domain/motion.py
class MotionModel(str, Enum): HOLONOMIC | ACKERMANN
@dataclass(frozen=True)
class MotionCapability:            # 每台载具构造期算一次，无 IO
    model: MotionModel
    max_lateral_speed_mps: float
    min_turn_radius_m: float | None
    allows_reverse: bool
@dataclass(frozen=True)
class HolonomicCommand: vx, vy, vz, yaw_rate_deg_s      # 世界系
@dataclass(frozen=True)
class CarCommand: forward_speed_mps, yaw_rate_deg_s, reverse
MotionCommand = HolonomicCommand | CarCommand
def command_type_for(capability) -> type
def require_supported(command, capability) -> None      # 不符抛 ValueError

# domain/execution.py
@dataclass(frozen=True) class CollisionContact: counterpart, at_monotonic_s
@dataclass(frozen=True) class CollisionState: in_contact, last_contact
# domain/motion.py
@dataclass(frozen=True)
class CollisionRecoverySettings: backoff_m, maximum_recoveries, reset_after_progress_m

# ports/vehicle.py
class Vehicle(Protocol):
    current_pose() -> WorldPose
    motion_capability() -> MotionCapability
    execute_segment(segment, *, budget, pose_checkpoint) -> None   # 由 GuidedVehicle 兑现
    cancel() / hover()
    collision_state() -> CollisionState

# ports/streaming_vehicle.py
class StreamingVehicle(Protocol):
    stream_state() -> TimedVehicleState
    stream_command(command: MotionCommand, duration_s: float) -> None

# runtime/motion/loop.py
SEGMENT_PERIOD_S = 0.030
class ControlLoop:
    segment_sink() -> Callable[[MotionCommand, float], None]   # vehicle.stream_command 的绑定
    stream_sink()  -> Callable[[MotionCommand, float], None]
    run_segment(segment, sink, *, budget) -> ExecutionReceipt
    run_stream(goals, sink, *, deadline_monotonic) -> None

# runtime/window/window.py
class PlanningWindow:  begin / submit / poll_stream / pause_for_completion / complete / end

# ports/goal_source.py
class StreamGoalSource(Protocol):
    next(state: TimedVehicleState, unwrapped_yaw_deg: float) -> MotionCommand | None

# workflows/<pkg>/__init__.py
TASK_KIND: TaskKind
REQUIRED_PORTS: tuple[str, ...]
```

十四条形状约定：

1. 命令下发只有一个签名 `stream_command(command, duration_s)`。任务层不自己造 sink：它调 `ControlLoop.segment_sink()` 或 `stream_sink()` 拿到可调用对象再传回循环，**不得自行调用**——伺服与限幅都在循环内完成，直接调用会绕过限幅。`SegmentSink`、`StreamSink` 只是执行层内部类型别名，不导出、不是跨层名字（R26）。
2. `execute_segment` 由 `runtime` 的 `GuidedVehicle(streaming, loop)` 兑现，引擎类不实现它（R1）。过渡期允许引擎留三行委派。
3. `run_stream` 的正常返回只有一种：`goals.next` 返回 `None`。deadline 到期抛 `BudgetExceeded`（R3）。
4. 目标源由 `runtime/motion/follower.py` 的 `QueueFollower` 实现，含队列消耗与航点超时；`PlanningWindow` 只提供队列视图、`consume` 与六个方法（R4）。
5. 碰撞只由 `ControlLoop._guard` 触发，`ExecutionService.checkpoint()` 只管停止与预算；`ports/collision_check.py` 删除（R5）。
6. 越点走廊常量的落点是 `algorithms/motion/corridor.py`，按 `MotionModel` 取值，配置可抬高不可压低（R6）。
7. `PositionHoldVehicle.hold_velocity` 保留，实现内部转调 `stream_command`（R7）。
8. 碰撞开关只有 `motion.collision.enabled`，默认 `true`；删掉按任务的 `collision_check_enabled`（R8）。
9. 两个周期常量只写在 `loop.py`，不进配置（R9）。
10. 任务声明在任务包 `__init__.py`，装配层读常量生成需求表，不做自动发现（R10）。
11. 删掉 `enabled_tasks` 与各引擎的 `ENABLED_TASKS`：可跑任务 = `REQUIRED_PORTS` ⊆ 已填槽位。UE4 有 `streaming` 槽位，巡航因此直接对它开放（R18，原 R11 作废）。
12. 控制器里的天气与机位合成一个 `EnvironmentChannel`，生命周期一致（R12）。
13. 提示词只外置 `adapters/trajectory_planner/prompts` 三个文件；其余四个小文件留在 Python（R13）。
14. `StreamGoalSource` 声明在新增的 `ports/goal_source.py`：任务层实现、执行层消费的协议必须落在 `ports/`；两个 Sink 名字不单独建文件（R16）。

## 四、三条关键路径

**段执行（数值动作；路径已确定的目标接近）**

```text
任务包  构造 MotionSegment（或由五个航点转五段）
ControlLoop.run_segment（逐段；execute_with_receipt 已从 ExecutionService 移出）
每 30 ms: _guard（停止/预算/碰撞）-> stream_state -> 采样五次曲线
          -> MotionSynthesizer（按 MotionCapability 分型）-> CommandLimiter
          -> StreamingVehicle.stream_command(cmd, 0.030)
段末    execution.stop() -> PositionHold 停稳验证 -> ExecutionReceipt
```

**速度流（滑窗补点的目标接近；无终点巡航）**

```text
任务包  ControlLoop.run_stream(QueueFollower(window, ...), sink, deadline)
每 50 ms: _guard -> stream_state -> QueueFollower.next（消耗队列、判超时）
          -> 伺服（全向速度或纯跟踪）-> 限幅 -> stream_command(cmd, 0.050)
          -> StreamRecorder（位姿差分、路径长度、控制遥测）
队列空  goals.next 返回 None -> execution.stop() -> 正常返回或继续等补点
```

**切载具（仅 idle，同场景不重启引擎）**

```text
controller  准入：state 必须 IDLE，否则 409；置 connecting
controller  arm_old.execution.request_stop()     # 只置事件
owner       下一检查点抛 ExecutionError -> stop()（cancel + hover）-> release_flight
controller  session.activate_vehicle(new)：引擎侧原子完成 park、hide、recover、home、机位
controller  换 VehicleArm、换机位、重算能力位与控件
controller  position_hold.start() -> IDLE
失败        回滚选择与机位，返回 SwitchRefused -> 409
```

判据：交接期间只有 owner 线程发指令；`request_stop` 到 `release_flight` 之间不允许第三方取控制权；`task_kinds` 随当前载具变化。

## 五、每层的职责、输入与输出

层间解耦靠三条机制，全部可测：

- **入口唯一（D10）。** 每层只从 `planner/<层>/__init__.py` 导出公开名字；**二级包入口**（`planner.algorithms.motion`、`planner.workflows.basic_motion`）也算合法入口。跨层 import 只能指向某个 `__init__.py`，不能指向普通模块文件。深层路径如 `planner.runtime.motion.loop` 只允许同层内部使用。契约层因此扩两个包入口，把新类型与全部 Protocol 导出去。
- **输入只有两类（D11）。** 要么是契约层类型，要么是本层自己声明的 `Protocol`。执行层不接收任务层的具体类，任务层不接收引擎对象。
- **只见名字不见实现（D12）。** 跨层传的是契约类型或本层 Protocol 的实例，构造参数与返回值都不写对方的具体类。

### 契约 `domain/`、`ports/`

- 职责：定义类型、常量、不变量校验与协议。不计算，不开线程，不读时钟。
- 输入：无。`domain/` 与 `ports/` 不 import 任何其他层。
- 输出：约 30 个类型与 14 个 Protocol，经 `planner.domain`、`planner.ports`、`planner.errors` 三个入口导出。
- 不暴露：任何实现类。`GuidedVehicle`、`ControlLoop`、`QueueFollower` 属执行层；`_Slot`、`SimulatorPorts` 属装配与引擎侧。
- 细节：`01-contracts.md` §3.5（输入面与输出面）。

### 计算 `algorithms/`

- 职责：纯变换与纯判定，同输入同输出。
- 输入：`planner.domain` 的 20 余个类型，加 `planner.errors` 的 `ProtocolError`、`ExecutionError`。不 import `ports/`。
- 输出：约 40 个纯函数与不可变设置类：几何 4、坐标变换 3、深度 8、扇面测距 4、序列校验 3、运动律 12（五次曲线、包络、走廊、判达、纯跟踪、偏航、限幅）、滑窗补点策略 1、几何规划器 1。
- 不暴露：任何状态、时钟、线程；有状态包装由调用方在自己的模块声明 `Protocol`。
- 细节：`02-algorithms.md` §3.5。

### 执行 `runtime/`

- 职责：持有状态与时间，不认识任何具体任务。
- 输入：契约层类型；`ports` 的 `Vehicle`、`StreamingVehicle`、`PositionHoldVehicle`、`ObservationSource`、`MovingObservationSource`、`TrajectoryPlanner`、`StreamGoalSource`；`algorithms` 的纯函数；`clock` 与 `sleep` 两个可替换调用。
- 输出（10 个）：`ControlLoop`（两条通道，带 `segment_sink()`、`stream_sink()`）、`ExecutionService`、`PlanningWindow`、`QueueFollower`、`GuidedVehicle`、`PositionHold`、`FrameHub`、`TrajectoryHub`、`LogBuffer`、`BackgroundCall`。`CollisionRecovery`、`StreamTimeline`、`MovingObservations`、`close_resources` 已内部化，不进对外面。
- 不暴露：`WorldWaypointQueue`、`SlowPlannerWorker`、`BackgroundContinuationPlanner`、`StreamRecorder`、`MotionSynthesizer`、`CommandLimiter`、两个周期常量、`_guard`。
- 细节：`03-runtime.md` §3.4。

### 任务 `workflows/`

- 职责：认识任务，不认识引擎。
- 输入：契约层类型与异常；`ports` 的 `Vehicle`、`StreamingVehicle`、`ObservationSource`、`ControlledObservationSource`、`MovingObservationSource`、`GoalObserver`、`TrajectoryPlanner`、`TaskCompletionChecker`、`TaskTrace`；`algorithms` 的纯函数与走廊常量；`runtime` 的对象一律经本层声明的四个 `Protocol` 使用（`MotionControl`、`ExecutionControl`、`PlanningQueue`、`MovingCapture`），不 import 其具体类（D11/D12）。
- 输出：三个任务包各自导出的 `TASK_KIND` 与 `REQUIRED_PORTS`（装配层按二级包入口逐个读）、三个 `Workflow` 实现、三份 `StageAdmission`。`TASK_PACKAGES`、`TaskDeclaration`、`require_confirmed` 不对外。
- 不暴露：`TaskLoop`、`StageEvidence`、`CycleContext`、`Channel` 实现、`TargetTracker`、`CompletionGate`、`StallWatch`、各任务策略类。
- 细节：`04-workflows.md` §3 末（输入面与输出面）。

### 装配 `assembly/`

- 职责：唯一同时认识任务与引擎的位置，负责构造期接线与运行期路由。
- 输入：三个任务包子包入口的 `TASK_KIND` 与 `REQUIRED_PORTS`；`planner.config` 的配置对象；引擎会话交出的 `MissionInputs`（`SimulatorPorts` 与端口对象）；`adapters` 造的解析器、检测器、规划器、完成判定。
- 输出（12 个）：`ServerController`（入口层唯一依赖，能力通过 `controller.capabilities()` 取）、`assemble_missions`、`VehicleArm`、`require_application`、`build_resources`、`SwitchRefused` 等；全表见 `05-assembly.md` §8.3。`capabilities_for`、`PlannerApplication`、`MissionRunner`、`MissionValidator`、`WorkflowRouter` 已内部化。
- 不暴露：六个控制器协作者（`LifecycleState`、`CommandChannel`、`WorldSwitcher`、`ModelSwitcher`、`EnvironmentChannel`、`ProfileSwitcher`）的内部方法；配置包的解析细节。
- 细节：`05-assembly.md` §3.3。

### 入口与适配 `adapters/`

- 职责：把外部请求翻译成装配层命令，把遥测读出来；实现模型端点与检测端点。
- 输入：`assembly` 的 `ServerController`（仅 `adapters/http`）；契约层类型；`algorithms` 的纯函数；模型 HTTP 端点。
- 输出：FastAPI 应用与路由；端口实现对象（`TaskParser`、`TargetDetector`、`GoalObserver`、`TrajectoryPlanner`、`TaskCompletionChecker`、`TaskTrace`）；MJPEG 帧流。
- 不暴露：状态机、命令队列、任务阶段逻辑；提示词模板不跨层可见。
- 细节：`06-adapters-engines.md` §三。

### 引擎 `simulators/`、`simulator/`

- 职责：把命令变成引擎动作，把引擎状态变成端口对象；进程栈负责起停与子进程。
- 输入：契约层类型与异常；`ports` 的协议定义；`simulators.yaml` 与地图配置；`stream_command(command, duration_s)` 的命令。
- 输出：`SimulatorPorts`（已填槽位）、`PORT_SLOTS`、`VEHICLES`、`build_session`、`describe`；`simulator/` 的 worker、探针、进程管理。
- 不暴露：客户端库对象不离开引擎包；端口实现类只经由 `SimulatorPorts` 槽位对外。
- 细节：`06-adapters-engines.md` §三。

### 削面结果（R21–R24）

六层各自按「每个对外名字给出层外运行期调用点」重审了一遍。设计时估的终值是 132 个名字，落地时按实际跨层用量定，最终入口导出：契约 116、端口 18、计算 11（另有 `motion/`、`depth/`、`targeting/` 三个二级入口）、执行 25、任务 5、装配 12。差额来自设计表低估了跨层引用面，削面按「无引用才删」执行，删的是死名字，不是被引用的类型。

| 层 | 对外名字 | 删除与移出 | 主要处置 |
| --- | --- | --- | --- |
| 契约 | 48 → 44 | 约 470 行 | 生命周期 7 类搬进 `assembly/controller/state.py`；`WorkerEvent`、`TaskSubmission` 随 `backend_worker` 进 `simulator/worker_protocol.py`；`LogBatch` 进 `runtime/log_buffer.py`；目标接近的 `approach.py` 整体迁出（三个类型与 `assess` 删）；`SegmentResult` 删；`depth_foreground`、`depth_visualization` 并入 `depth.py`；`MotionLimits` 的四个 heading 字段收成 `HeadingHoldSettings`；`domain/` 26 → 21 个文件 |
| 计算 | 72 → 36 | 约 170 行 | 27 个名字私有化；`plan_segment`、`SegmentSetpoint` 删除；停稳三判合成 `settle_check`；两个偏航函数合一；对外入口拆成层入口 11 个加 `motion/`、`depth/`、`targeting/` 三个二级入口 |
| 执行 | 19 → 10 | 约 710 行 | 删 `segment_motion.py`、`continuous_execution.py`、`collision_check.py`；两个引擎的跟踪器律段删除；两份碰撞退避镜像删除；`StreamTimeline`、`MovingObservations`、`CollisionRecovery`、`close_resources` 内部化 |
| 任务 | 10 → 7 | 约 303 行 | `TaskDeclaration`、`Workflow` 内部化；`require_confirmed` 撤；`TaskLoop`、`StageEvidence`、六个 `Channel` 全部不进对外面 |
| 装配 | 27 → 12 | 约 120 行 | 控制器协作者 6 → 4（`SessionSwitcher` 收地图、外观、配置；`EnvironmentChannel` 收天气与机位）；六个工厂收成四个；`capabilities_for`、`PlannerApplication`、`MissionValidator`、`MissionRunner`、`WorkflowRouter` 内部化 |
| 引擎与适配 | 50 → 23 | 约 2233 行 | 删 `recording.py`、两个 tracker 文件、观测门、走廊常量、车道提示词；`streaming_views.py` 移出到 `scripts/`；提示词正文外置为模板（1411 → 约 490 行 Python 加约 900 行模板）；`TASK_REQUIREMENTS` 迁出 |

三条判据落在测试里：`tests/test_public_surface.py` 把每层的包入口 `__all__` 与该层文档的 A 表逐名比对，多一个少一个都失败；`tests/test_layer_entries.py` 保证跨层 import 只落在 `__init__.py`；`tests/test_architecture.py` 继续守客户端库与能力推导。


## 六、被动能力的落点

| 能力 | 位置 | 触发 | 任务层看到什么 |
| --- | --- | --- | --- |
| 控制权 | `runtime/execution.py` | `acquire_flight`/`release_flight`/检查点 | 什么都不用做，`run()` 首行取、`finally` 还 |
| 碰撞退避 | `runtime/collision_recovery.py` + `ControlLoop._guard` | 每拍或每段检查点 | 只在恢复次数超限时收到 `ExecutionError` |
| 滑窗 | `runtime/window/` | `begin/submit/poll_stream/pause_for_completion/complete/end` | 六个方法，看不到线程与队列 |
| 时间与停止 | `runtime/execution.py` 的 deadline 与 `workflows/common/budget.py` | 检查点 | `BudgetExceeded` / `ExecutionError` 各映射成一个状态 |
| 停稳 | `runtime/position_hold.py` | 观测前、完成确认前、暂停后 | 不感知，取到的观测已保证停稳 |

## 七、失败语义

| 情形 | 抛出方 | 任务层映射 | 附带信息 |
| --- | --- | --- | --- |
| 用户停止 | `runtime/execution.py` | `CANCELLED` | `stop_confirmed` 进测量 |
| 预算耗尽 | `ControlLoop` / `budget.require_time` | `TIMEOUT` | 剩余运动与最后实测位姿 |
| 碰撞超限 | `CollisionRecovery` | `EXECUTION_ERROR` | 对方名称、恢复次数 |
| 模型超时 | `runtime/model_calls.py` | `TIMEOUT` | 调用计数、阶段已用时长 |
| 观测不新鲜 | 任务包策略 | 任务内状态（如 `PERCEPTION_UNRELIABLE`） | 连续不可靠次数 |
| 规划输出非法 | `TrajectoryValidator` | `MODEL_ERROR` | 校验失败原因，零段执行 |
| 端口与命令类型不符 | 适配层 `require_supported` | 构造期或第一拍报错 | 载具模型与命令类型 |
| 切载具失败 | `assembly/controller` | 入口层 409 | `SwitchRefused(vehicle, reason)` |

三条固定语义：异常离开循环前必须先 `stop()`；失败也带证据（`completion_confirmed=False`、`completion_basis=NOT_VERIFIED`）；`KeyboardInterrupt` 不转成 `StageResult`，清理后原样上抛。

## 八、测试与验收

新增测试文件与守的东西：

| 文件 | 守什么 |
| --- | --- |
| `tests/test_layering.py` | D1–D9 依赖规则，含 `simulator/` 白名单与"只有 assembly 同时 import 任务与引擎" |
| `tests/test_layer_entries.py` | D10–D12：跨层 import 只落在包入口；输入参数类型是契约类型或本层 `Protocol`；跨层签名里不出现对方具体类 |
| `tests/test_public_surface.py` | 每层包入口的 `__all__` 与该层文档 A 表中处置为「保留」的名字逐名一致（内部化与删除的行不参与比对）；二级入口只导出本组名字 |
| `tests/test_capability_matrix.py` | R18：可跑任务 = `REQUIRED_PORTS` ⊆ 已填槽位；`SimulatorPorts` 无 `enabled_tasks`；UE4 填了 `streaming` 因此巡航在结果里 |
| `tests/test_motion_contract.py` | 五个新类型的 `__post_init__` 负例、命令分型、`require_supported` |
| `tests/test_vehicle_port.py` | 三个适配层的 `motion_capability()` 取值；给车喂横向命令必须抛错且引擎调用计数为 0 |
| `tests/test_control_loop_segment.py` | 30 ms 节拍、次数、预算预检、重入、停稳验证 |
| `tests/test_control_loop_stream.py` | 20 Hz 节拍、`next` 返回 `None` 时停一次、deadline 抛 `BudgetExceeded` |
| `tests/test_command_limiter.py` | 加速度与 jerk 不超限、`NaN` 整段零下发 |
| `tests/test_collision_recovery.py` | 第 n 次后退距离、超限抛错、进度复位、恢复期间不连乘 |
| `tests/test_task_packages.py` | 三包互不 import、不含 `MotionModel`、不含 `backoff`、`lane` 全无 |
| `tests/test_channel_selection.py` | 8 组输入下的通道判定 |
| `tests/test_server_controller.py` | 既有 68 例只改 import；新增切载具阻塞与回滚、`SwitchRefused` 映射 |
| `tests/test_prompt_render.py` | 模板渲染与旧版逐字一致 |

人工验收，UE5 一条链路：启动场景 → 无人机数值动作 → 不重启切到无人车 → 无人车数值动作与目标接近 → 无人车巡航并制造一次接触（确认后退、清队列、继续）→ 切回无人机（机位与控制权跟随）→ 换图（能力位与控件刷新）。

UE4 侧一次：启动场景后确认巡航出现在能力位里（R18 取消了实测白名单），实跑一段并记录画面与退出条件。

数值底线：数值动作位置误差不劣于现状（直行 5 m 0.05 m、倒车 2 m 0.03 m，`docs/archive/measurements.md:48-49`）；连续链路控制采样条数与重构前差不超过 1 拍。

## 八之二、删除安全核对

对计划中风险最高的几个删除，逐条查了运行期调用方：

| 删除对象 | 运行期调用方 | 结论 |
| --- | --- | --- |
| `segment_motion.plan_segment` | 0；只有 `services/__init__.py` 的再导出与 `tests/test_motion.py` 的 4 个用例 | 删。连带删 4 条用例，已写进验收例外 |
| `domain.trajectory.SegmentResult` | 0；只有 `domain/__init__.py` 的再导出 | 删 |
| `ports` 里的 `TaskCompletion` | 0 出现；实现类由工厂直接构造 | 删 |
| `simulators/projectairsim/streaming_views.py` | 0；只有 `tests/test_streaming_views.py` 导入 | 移到 `scripts/`；`domain/view.py:18` 的注释要同步改路径 |
| `services/continuous_execution.py` | 两个 workflow 的 continuous 与测试 | 拆进 `runtime` 后删，行为由 `ControlLoop.run_stream` 承担 |
| `simulators/*/motion_tracker.py`、`projectairsim/motion.py` 的律段 | 三个载具类 | 律进 `algorithms` 与 `runtime/motion`，载具类只留执行器 |

跨层深层 import 实测 460 条、162 个文件（原估 470 条）。

## 九、迁移阶段

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| 0 | 删除俯视任务 | 已完成（`8273ef7`） |
| 1 | 已完成。改名与搬家：三个包改名、`domain/runtime.py` 改 `engine_family.py`、纯函数进 `algorithms/`、`MotionLimits` 合一份、滑窗线程进 `runtime/window/`、任务判定进任务包、删 `lane_reference`；跨层 import 改走包入口（实测 460 条、162 个文件）；按各层 B 表删除历史残留（合计约 3800 行） | `pytest tests/ -q` 全绿；`test_layering.py` 与 `test_layer_entries.py` 通过；用例数不减 |
| 2 | 已完成。契约扩展：加 `MotionModel`/`MotionCapability`/两个命令/`CollisionState`/`CollisionRecoverySettings`；三个适配层实现 `motion_capability()` 与 `stream_command`（旧名保留一版别名）；`motion.collision` 配置段 | 契约与端口测试全绿；行为不变 |
| 3 | 已完成。循环上移：`ControlLoop` 外壳先委派旧实现，再把律搬进 `runtime/motion/`，删两个 tracker 的律，`GuidedVehicle` 填槽 | UE5 数值动作精度不劣化；连续遥测差 ≤1 拍 |
| 4 | 已完成。被动安全：`CollisionRecovery` 接 `_guard`，删巡航与目标接近里的两处镜像退避与碰撞预检分支；此时才删 `enabled_tasks` 与 `ENABLED_TASKS`，能力矩阵改为纯槽位推导（R18） | 假载具接触三任务都触发；次数超限报错；UE4 巡航出现在能力位里且能跑 |
| 5 | 已完成，视觉目标任务的连续路径仍走自身批次语义（裁决见 `COMMAND.md` 遗留项）。任务层收口：`common/{loop,evidence,budget,streaming}`、三包 `policy.py`、`TASK_KIND`/`REQUIRED_PORTS`、巡航单一路径 | 通道选择与包隔离测试；镜像代码 AST 测试 |
| 6 | 已完成。装配与配置拆分：`controller/` 六协作者、`factories/` 四文件、`planner/config/` 七文件、`requirements.py`、`SwitchRefused` | `/api/status` 与 `/api/world/options` 逐字段一致；68 例控制器测试 |
| 7 | 代码完成。引擎与提示词收口：`session.py` 拆分、提示词模板化、端口自检、`CameraIntrinsics` 断言、`streaming_views` 归位 | 渲染逐字一致（已验）；UE5 三类任务实跑留待有模拟器环境时补 |

每阶段结束跑一次 `pytest tests/ -q`，阶段 3 之后每个阶段加一次 UE5 冒烟。回滚单位是阶段。

## 十、裁决记录

| 编号 | 争点 | 裁决 | 代价 |
| --- | --- | --- | --- |
| R1 | 段采样循环在引擎还是执行层 | 执行层；引擎只实现 `stream_command`；`GuidedVehicle` 填 `Vehicle` 槽 | 装配层多一层组合；过渡期留三行委派 |
| R2 | 命令下发几个签名 | 一个：`stream_command(command, duration_s)` | 旧 `stream_velocity` 的 11 处调用要改 |
| R3 | `run_stream` 到期行为 | 抛 `BudgetExceeded` | 任务层必须捕，否则漏判超时 |
| R4 | 目标源谁实现 | `QueueFollower`；窗口只管队列与六个方法 | 任务层要多传一个对象 |
| R5 | 碰撞挂检查点还是循环 | 循环 `_guard`；删 `CollisionCheck` 端口 | `acquire_flight` 少一个参数，假载具测试要改 |
| R6 | 越点走廊归属 | `algorithms/motion/corridor.py`，按模型取值，配置只能抬高 | 引擎删常量；常量的实测依据仍只有一次标定 |
| R7 | `hold_velocity` 去留 | 保留，内部转调 `stream_command` | 一个同义方法 |
| R8 | 碰撞开关粒度 | 只 `motion.collision.enabled`，删按任务开关 | 视觉目标路径首次真正启用碰撞检测，需回归 |
| R9 | 周期是否可配 | 两个常量写在 `loop.py` | 小机型不能单独调慢 |
| R10 | 任务声明位置 | 任务包 `__init__.py` | 常量分散，要靠测试守形状 |
| R11 | `enabled_tasks` 位置 | **作废**，见 R18 | — |
| R12 | 天气与机位 | 合成 `EnvironmentChannel` | 与 `REFACTOR.md` 原表一字不符，需同步改 |
| R13 | 提示词外置范围 | 只三个大文件 | 四份小提示词仍在 Python |
| R14 | 相机内参半像素差 | 两份都留，加断言 | 长期存在两套主点 |
| R15 | `streaming_views.py` | **作废**，见 R22（移出到 `scripts/`） | — |
| R16 | `StreamGoalSource` 放哪 | 新增 `ports/goal_source.py`；Sink 名不建文件 | `ports/` 最终 13 个文件（含删 `backend_worker.py`） |
| R17 | 越点走廊取值 | 阿克曼 1.0 m（原 0.6 m 实测失败率偏高），配置只能抬高 | 车更早起下一个航点，越点后轨迹更宽松 |
| R18 | 可跑任务由什么决定 | 只由 `REQUIRED_PORTS` ⊆ 已填槽位；删 `enabled_tasks` 与 `ENABLED_TASKS` | 未实跑过的组合也会被承诺，出问题要从引擎查 |
| R19 | 层间怎么解耦 | 层一个公开入口；输入只收契约类型或本层 `Protocol`；跨层不见实现类 | 包入口要导出全部公开名，460 条深层 import 要改 |
| R20 | 契约层包入口导出范围 | `domain/`、`ports/` 两个 `__init__` 导出全部公开名字，`errors.py` 作第三个入口 | 导出面变宽，新增类型必须同步入口 |
| R21 | 对外名字是否都必要 | 逐个给出层外运行期调用点；没有就内部化或删除；找不到就删，只要测试在用的不保留 | 约 144 个名字被内部化或删除 |
| R22 | HTTP 生命周期与目标接近类型留在契约层吗 | 搬出：生命周期进装配层，worker 协议进进程栈，目标接近类型进任务包 | 契约层少 12 个名字、`domain/` 26 → 21 个文件 |
| R23 | 契约层还能不能压到 34 | 压到 34 需要拆掉构造期校验，保留 44 并逐条给 import 行反证 | 契约层仍是六层里对外名字最多的 |
| R25 | 收尾一致性：`approach.py` 与 `COMMAND_FOR_MODEL` | `domain/approach.py` 整体迁出任务包；不新增 `COMMAND_FOR_MODEL`，适配层用 `require_supported` 校验 | 契约层少一个文件 |
| R26 | sink 是不是跨层名字 | 不是。`ControlLoop.segment_sink()`/`stream_sink()` 提供 `stream_command` 的绑定，两个 Sink 类型别名留在执行层内部 | 任务层多两次方法调用 |
| R24 | 一个大入口还是几个二级入口 | 允许二级包入口：层入口只放通用名字，`algorithms.motion`、`algorithms.depth`、`algorithms.targeting` 各自成入口 | 规则稍宽，测试按"只能指向 `__init__.py`"守 |

## 十一、仍然开放的问题

每条已给默认，实施时按默认走；要改只需说一句。

| 问题 | 默认 | 备选与代价 |
| --- | --- | --- |
| 段执行与连续执行是否共用同一个 `MotionSynthesizer` 实例 | 每类载具一个，随 arm 建一次 | 每段重建：状态干净但首拍有阶跃 |
| 批次模式是否也走 `PlanningWindow` | 不走，任务包直连 `planner.plan` | 走窗口可预取，但窗口要加第七个方法，且批次语义混进流式队列 |
| 段内暂停粒度 | 段级（段与段之间判） | 检查点级要加 `pause_when` 谓词，停稳与残余段重锚逻辑搬进执行层 |
| 巡航速度三处上限 | 取三者最小值，`MotionCapability` 只做合法性校验 | 只认载具能力：车的横向上限为 0，语义需要另加"前向上限"字段 |
| `min_turn_radius_m` | 常量 3.20 m（由 `drive.py` 标定推出） | 进配置：多一个没标定过的数 |
| `MovingObservationSource` 是否进 `REQUIRED_PORTS` | 不进；缺端口在构造期抛 `ProtocolError` | 进：能力门更严，没有移动取帧的引擎直接看不到该任务 |
| 扇区网格的视场来源 | 保持现状（rgb 视场） | 改用深度内参：AirSim 上扇面标签会更准，但要重测 |
| `DepthGeometry.estimate` 捕获集 | 保持三类 | 加 `OverflowError`：真溢出会被静默降级 |
| `simulator/` 反向 import 装配层 | 白名单放行两个模块 | 上移到组合根：两处各写三行装配 |

## 十二、文档同步清单

本次裁决使既有文档出现三处不一致，实施前先改：

| 文档 | 要改的段落 |
| --- | --- |
| `docs/REFACTOR.md` | 「被动能力」里"`checkpoint()` 是碰撞退避触发点"改为"`ControlLoop._guard` 是触发点"；装配层表里 `controller/view.py` 合并进 `controller/weather.py`；新增 `GuidedVehicle` 与 `QueueFollower`；错误表补 `SwitchRefused` |
| `docs/TARGET_ARCHITECTURE.md` | 执行层职责已含碰撞退避，保持；把"段采样在适配层"的表述指向 `03-runtime.md` |
| `README.md` | 文档索引补 `docs/refactor/` |
| `docs/ARCHITECTURE.md`、`docs/AGENT.md`、`docs/CODE_INVENTORY.md` | 描述的是当前代码，等对应阶段落地时再改，现在不动 |

## 附：各层文档

| 文件 | 层 | 行数 |
| --- | --- | --- |
| `00-boundaries.md` | 共同约束与初始契约 | 193 |
| `01-contracts.md` | 契约层 `domain/`、`ports/` | 420 |
| `02-algorithms.md` | 计算层 `algorithms/` | 955 |
| `03-runtime.md` | 执行层 `runtime/` | 687 |
| `04-workflows.md` | 任务层 `workflows/` | 391 |
| `05-assembly.md` | 装配层 `assembly/` 与 `config/` | 459 |
| `06-adapters-engines.md` | 端口实现、引擎实现、进程栈 | 454 |
