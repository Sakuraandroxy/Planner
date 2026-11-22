# 重构汇总方案

这是六份分层方案的合并结论与裁决记录。各层细节在 `01` 到 `06` 六份文件里，本文件只写层间协议、关键路径、跨层裁决与迁移顺序。

## 一、结论

- 代码分成六类：契约（`domain/`、`ports/`）、计算（`algorithms/`）、执行（`runtime/`）、任务（`workflows/`）、装配（`assembly/`）、入口（`adapters/http/`）；另加端口实现（`adapters/` 其余）、引擎实现（`simulators/<name>/`）与进程栈（`simulator/`）。
- `capabilities` 改 `algorithms`，`services` 改 `runtime`，`application` 改 `assembly`，`domain/runtime.py` 改 `domain/engine_family.py`。
- 运动控制只有一套律：五次多项式、刹车包络、纯跟踪、偏航伺服、限幅都在 `algorithms/`，控制循环与节拍在 `runtime/motion/loop.py`，引擎只实现"读状态、按命令推进一个周期"。
- 两条命令通道按命令性质分工：动作已确定、路径不再变化的走段执行（30 ms 采样）；滑窗持续补点、运动中取帧的走速度流（20 Hz 伺服）。
- 碰撞退避、滑窗、控制权、时间与停止是执行层的被动能力，任务层不编排。
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
| D1 | `domain/`、`ports/` 不 import 其他层 | 无 |
| D2 | `algorithms/` 只 import `domain/`、`ports/` | 当前不需要 `ports/` |
| D3 | `runtime/` 只 import `domain/`、`ports/`、`algorithms/` | 无 |
| D4 | `workflows/` 只 import 上述四层，任务包之间零 import | 无 |
| D5 | `assembly/` 可 import 全部，是唯一同时认识任务与引擎的位置 | 无 |
| D6 | `adapters/` 的端口实现只 import `domain/`、`ports/`、`algorithms/`；`adapters/http/` 只 import `assembly/`、`domain/` | 无 |
| D7 | `simulators/<name>/` 只 import `domain/`、`ports/`、`simulator/` 与自己的客户端库 | 无 |
| D8 | `simulator/` 不 import `simulators/` | 允许 import `assembly.mission_assembly` 与 `assembly.reporting` 两个模块（会话建立即接线、子进程上报），不许 import `assembly.controller` |
| D9 | `server_bootstrap.py` 是唯一同时 import `assembly` 与 `adapters/http` 的组合根 | 无 |

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

1. 命令下发只有一个签名 `stream_command(command, duration_s)`。`SegmentSink` 与 `StreamSink` 都是它的别名，不再有 `apply` 这个第二名字。
2. `execute_segment` 由 `runtime` 的 `GuidedVehicle(streaming, loop)` 兑现，引擎类不实现它（R1）。过渡期允许引擎留三行委派。
3. `run_stream` 的正常返回只有一种：`goals.next` 返回 `None`。deadline 到期抛 `BudgetExceeded`（R3）。
4. 目标源由 `runtime/motion/follower.py` 的 `QueueFollower` 实现，含队列消耗与航点超时；`PlanningWindow` 只提供队列视图、`consume` 与六个方法（R4）。
5. 碰撞只由 `ControlLoop._guard` 触发，`ExecutionService.checkpoint()` 只管停止与预算；`ports/collision_check.py` 删除（R5）。
6. 越点走廊常量的落点是 `algorithms/motion/corridor.py`，按 `MotionModel` 取值，配置可抬高不可压低（R6）。
7. `PositionHoldVehicle.hold_velocity` 保留，实现内部转调 `stream_command`（R7）。
8. 碰撞开关只有 `motion.collision.enabled`，默认 `true`；删掉按任务的 `collision_check_enabled`（R8）。
9. 两个周期常量只写在 `loop.py`，不进配置（R9）。
10. 任务声明在任务包 `__init__.py`，装配层读常量生成需求表，不做自动发现（R10）。
11. `enabled_tasks` 留在 `SimulatorPorts`，表示实测状态（R11）。
12. 控制器里的天气与机位合成一个 `EnvironmentChannel`，生命周期一致（R12）。
13. 提示词只外置 `adapters/trajectory_planner/prompts` 三个文件；其余四个小文件留在 Python（R13）。
14. `StreamGoalSource` 声明在新增的 `ports/goal_source.py`：任务层实现、执行层消费的协议必须落在 `ports/`；两个 Sink 名字不单独建文件（R16）。

## 四、三条关键路径

**段执行（数值动作；路径已确定的目标接近）**

```text
任务包  构造 MotionSegment（或由五个航点转五段）
runtime/execution.execute_with_receipt -> ControlLoop.run_segment
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
controller  旧载具 park（抬到 1 km 钉住）-> session.activate(new)
controller  换 VehicleArm、换机位、重算能力位与控件
controller  position_hold.start() -> IDLE
失败        回滚选择与机位，返回 SwitchRefused -> 409
```

判据：交接期间只有 owner 线程发指令；`request_stop` 到 `release_flight` 之间不允许第三方取控制权；`task_kinds` 随当前载具变化。

## 五、各层要点

| 层 | 一句话 | 关键决定 | 细节 |
| --- | --- | --- | --- |
| 契约 | 只定义，不计算 | `MotionCapability` 与 `Capabilities` 分文件、不互引；`MotionLimits` 合成一份 18 字段；命令按模型分型 | `01-contracts.md` |
| 计算 | 同输入同输出 | 三套循环里的算式各有唯一落点；纯跟踪给曲率解算；限幅用加速度球投影加 jerk 限 | `02-algorithms.md` |
| 执行 | 有状态、不认识任务 | 一条 `ControlLoop` 跑两条通道；碰撞、队列、滑窗、停稳都在这里；四个对象共用一个 owner 线程 | `03-runtime.md` |
| 任务 | 认任务、不认引擎 | 三类任务包各导出 `TASK_KIND`/`REQUIRED_PORTS`；通道选择写成可测的 8 组判据；不许出现 `MotionModel` | `04-workflows.md` |
| 装配 | 唯一同时认识两边 | 能力求交在 `requirements.py`；控制器拆六个协作者；配置拆成 `planner/config/` 包 | `05-assembly.md` |
| 适配与引擎 | 端口实现与引擎实现 | 引擎只实现 `stream_command` 与三个无 IO 查询；两个 tracker 的律删掉；提示词外置只做三个大文件 | `06-adapters-engines.md` |

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
| `tests/test_layering.py` | D1–D9 九条依赖规则，含 `simulator/` 白名单与"只有 assembly 同时 import 任务与引擎" |
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

数值底线：数值动作位置误差不劣于现状（直行 5 m 0.05 m、倒车 2 m 0.03 m，`docs/archive/measurements.md:48-49`）；连续链路控制采样条数与重构前差不超过 1 拍。

## 九、迁移阶段

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| 0 | 删除俯视任务 | 已完成（`8273ef7`） |
| 1 | 改名与搬家：三个包改名、`domain/runtime.py` 改 `engine_family.py`、纯函数进 `algorithms/`、`MotionLimits` 合一份、滑窗线程进 `runtime/window/`、任务判定进任务包、删 `lane_reference` | `pytest tests/ -q` 全绿；`test_layering.py` 通过；用例数不减 |
| 2 | 契约扩展：加 `MotionModel`/`MotionCapability`/两个命令/`CollisionState`/`CollisionRecoverySettings`；三个适配层实现 `motion_capability()` 与 `stream_command`（旧名保留一版别名）；`motion.collision` 配置段 | 契约与端口测试全绿；行为不变 |
| 3 | 循环上移：`ControlLoop` 外壳先委派旧实现，再把律搬进 `runtime/motion/`，删两个 tracker 的律，`GuidedVehicle` 填槽 | UE5 数值动作精度不劣化；连续遥测差 ≤1 拍 |
| 4 | 被动安全：`CollisionRecovery` 接 `_guard`，删巡航与目标接近里的两处镜像退避与碰撞预检分支 | 假载具接触三任务都触发；次数超限报错 |
| 5 | 任务层收口：`common/{loop,evidence,budget,streaming}`、三包 `policy.py`、`TASK_KIND`/`REQUIRED_PORTS`、巡航单一路径 | 通道选择与包隔离测试；镜像代码 AST 测试 |
| 6 | 装配与配置拆分：`controller/` 六协作者、`factories/` 四文件、`planner/config/` 七文件、`requirements.py`、`SwitchRefused` | `/api/status` 与 `/api/world/options` 逐字段一致；68 例控制器测试 |
| 7 | 引擎与提示词收口：`session.py` 拆分、提示词模板化、端口自检、`CameraIntrinsics` 断言、`streaming_views` 归位 | UE5 三类任务实跑；UE4 数值动作与目标接近；渲染逐字一致 |

每阶段结束跑一次 `pytest tests/ -q`，阶段 3 之后每个阶段加一次 UE5 冒烟。回滚单位是阶段。

## 十、裁决记录

| 编号 | 争点 | 裁决 | 代价 |
| --- | --- | --- | --- |
| R1 | 段采样循环在引擎还是执行层 | 执行层；引擎只实现 `stream_command`；`GuidedVehicle` 填 `Vehicle` 槽 | 装配层多一层组合；过渡期留三行委派 |
| R2 | 命令下发几个签名 | 一个：`stream_command(command, duration_s)` | 旧 `stream_velocity` 的 11 处调用要改 |
| R3 | `run_stream` 到期行为 | 抛 `BudgetExceeded` | 任务层必须捕，否则漏判超时 |
| R4 | 目标源谁实现 | `QueueFollower`；窗口只管队列与六个方法 | 任务层要多传一个对象 |
| R5 | 碰撞挂检查点还是循环 | 循环 `_guard`；删 `CollisionCheck` 端口 | `acquire_flight` 少一个参数，假载具测试要改 |
| R6 | 越点走廊归属 | `algorithms/motion/corridor.py`，按模型取值，配置只能抬高 | 引擎删常量；0.6 m 仍无实测依据 |
| R7 | `hold_velocity` 去留 | 保留，内部转调 `stream_command` | 一个同义方法 |
| R8 | 碰撞开关粒度 | 只 `motion.collision.enabled`，删按任务开关 | 视觉目标路径首次真正启用碰撞检测，需回归 |
| R9 | 周期是否可配 | 两个常量写在 `loop.py` | 小机型不能单独调慢 |
| R10 | 任务声明位置 | 任务包 `__init__.py` | 常量分散，要靠测试守形状 |
| R11 | `enabled_tasks` 位置 | 留 `SimulatorPorts` | 能力位不随载具变化的那部分仍在引擎侧 |
| R12 | 天气与机位 | 合成 `EnvironmentChannel` | 与 `REFACTOR.md` 原表一字不符，需同步改 |
| R13 | 提示词外置范围 | 只三个大文件 | 四份小提示词仍在 Python |
| R14 | 相机内参半像素差 | 两份都留，加断言 | 长期存在两套主点 |
| R15 | `streaming_views.py` | 留引擎包，运行时不 import | 引擎包继续携带 xvfb/xpra 相关代码 |
| R16 | `StreamGoalSource` 放哪 | 新增 `ports/goal_source.py`；Sink 名不建文件 | `ports/` 回到 14 个文件 |

## 十一、仍然开放的问题

每条已给默认，实施时按默认走；要改只需说一句。

| 问题 | 默认 | 备选与代价 |
| --- | --- | --- |
| 段执行与连续执行是否共用同一个 `MotionSynthesizer` 实例 | 每类载具一个，随 arm 建一次 | 每段重建：状态干净但首拍有阶跃 |
| 批次模式是否也走 `PlanningWindow` | 不走，任务包直连 `planner.plan` | 走窗口可预取，但窗口要加第七个方法，且批次语义混进流式队列 |
| 段内暂停粒度 | 段级（段与段之间判） | 检查点级要加 `pause_when` 谓词，停稳与残余段重锚逻辑搬进执行层 |
| 巡航速度三处上限 | 取三者最小值，`MotionCapability` 只做合法性校验 | 只认载具能力：车的横向上限为 0，语义需要另加"前向上限"字段 |
| `min_turn_radius_m` | 常量 3.20 m（由 `drive.py` 标定推出） | 进配置：多一个没标定过的数 |
| `MovingObservationSource` 是否进 `REQUIRED_PORTS` | 不进；缺端口在构造期抛 `ProtocolError` | 进：能力门更严，但要改 `enabled_tasks` 语义 |
| 扇区网格的视场来源 | 保持现状（rgb 视场） | 改用深度内参：AirSim 上扇面标签会更准，但要重测 |
| `DepthGeometry.estimate` 捕获集 | 保持三类 | 加 `OverflowError`：真溢出会被静默降级 |
| `simulator/` 反向 import 装配层 | 白名单放行两个模块 | 上移到组合根：两处各写三行装配 |
| UE4 的巡航何时开放 | 实机飞过一次再进 `enabled_tasks` | 提前开放：能力位承诺未验证的任务 |

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
| `00-boundaries.md` | 共同约束与初始契约 | 192 |
| `01-contracts.md` | 契约层 `domain/`、`ports/` | 268 |
| `02-algorithms.md` | 计算层 `algorithms/` | 785 |
| `03-runtime.md` | 执行层 `runtime/` | 564 |
| `04-workflows.md` | 任务层 `workflows/` | 313 |
| `05-assembly.md` | 装配层 `assembly/` 与 `config/` | 358 |
| `06-adapters-engines.md` | 端口实现、引擎实现、进程栈 | 338 |
