# 重构分工：边界与交付要求

这份文件是六个执行 agent 的共同约束。所有层方案以它为准，冲突时以它为准，改它必须由指挥层改。

## 一、已定决策

以下决策不再讨论，方案必须与之相容。

| 项 | 决定 |
| --- | --- |
| 包名 | `capabilities` 改 `algorithms`，`services` 改 `runtime`，`application` 改 `assembly`，`domain/runtime.py` 改 `domain/engine_family.py` |
| 载具 | 只有两种，固定不变：无人机（全向可横移）、阿克曼无人车（不能横移，只能前后加转向） |
| 任务 | 只有三类：数值动作、视觉目标接近、无终点巡航。两类载具都支持三类 |
| 巡航 | 纯视觉，不读车道数据。删除 `domain/lane_reference.py` 与相关空接线 |
| 碰撞退避 | 被动技能，落在执行层，由检查点触发，任务层不编排 |
| 控制权 | 执行层唯一硬约束：任一时刻每台载具一个 owner |
| 规划器配置 | 全局一段 `trajectory_planner`，换模型不改代码 |
| 命令通道 | 两条都保留，按命令性质分工，见下节 |
| 热切换 | 同场景内切换载具，不重启引擎 |
| 分层 | 契约、计算、执行、任务、装配、入口；外加引擎实现与端口实现 |

## 二、两条命令通道的分工

这是本次重构的运动控制核心决策，所有层必须按它设计。

| 通道 | 适用 | 判据 | 命令单位 | 控制率 |
| --- | --- | --- | --- | --- |
| 段执行（批次） | 数值动作；规划器一次给出一批确定航点且期间不重规划的目标接近 | 动作用户已说清、路径已确定 | 一段：起止位姿加五次多项式时间曲线 | 引擎侧按 30 ms 采样曲线 |
| 速度流（连续） | 滑窗持续补点的目标接近；无终点巡航 | 路径在运动中由规划器不断给出 | 一拍：速度命令加持续时长 | 20 Hz |

分工由任务层选择原语，执行层两条都提供且不判断业务。两条共用同一套限幅、伺服与偏航律，只有目标生成方式不同。

## 三、包与依赖规则

```
planner/
  domain/        契约    类型、常量、不变量校验、引擎能力位、运动模型
  ports/         契约    Protocol
  algorithms/    计算    纯函数与纯策略
  runtime/       执行    控制权、控制循环、队列、滑窗、碰撞退避、汇
  workflows/     任务    三类任务包 + 公共编排
  assembly/      装配    装配、路由、能力求交、控制器、配置装配
  adapters/      入口与端口实现   http/、task_parser/、target_detection/、trajectory_planner/、task_completion/
  simulator/     进程栈    起停、探针、worker、子进程
  simulators/    引擎实现  每个引擎一个包，只有这里 import 客户端库
```

依赖规则，写成测试：

1. `domain/` 与 `ports/` 不 import 其他任何层。
2. `algorithms/` 只 import `domain/`、`ports/`。不许 import `runtime/`、`workflows/`、`assembly/`。
3. `runtime/` 只 import `domain/`、`ports/`、`algorithms/`。不许 import `workflows/`、`assembly/`、`simulators/`。
4. `workflows/` 只 import `domain/`、`ports/`、`algorithms/`、`runtime/`。任务包之间不互相 import。
5. `assembly/` 可以 import 以上全部加 `simulators/`，是唯一同时认识任务与引擎的位置。
6. `adapters/` 的端口实现只 import `domain/`、`ports/`、`algorithms/`；`adapters/http/` 只 import `assembly/` 与 `domain/`。
7. `simulators/<name>/` 只 import `domain/`、`ports/`、 `simulator/`，以及自己的客户端库。
8. `simulator/` 不 import `simulators/`。

## 四、跨层契约

下面这些名字是层间交界处，各层方案必须使用这些名字，不得自行另起。

### 契约层定义

```python
class MotionModel(Enum):
    HOLONOMIC = "holonomic"      # 无人机
    ACKERMANN = "ackermann"      # 无人车

@dataclass(frozen=True)
class MotionCapability:
    model: MotionModel
    max_lateral_speed_mps: float      # 阿克曼为 0
    min_turn_radius_m: float | None   # 阿克曼有下限
    allows_reverse: bool

@dataclass(frozen=True)
class HolonomicCommand:      # 给 HOLONOMIC
    vx: float; vy: float; vz: float       # 世界系
    yaw_rate_deg_s: float

@dataclass(frozen=True)
class CarCommand:            # 给 ACKERMANN
    forward_speed_mps: float              # 有符号，负为倒车
    yaw_rate_deg_s: float
    reverse: bool

MotionCommand = HolonomicCommand | CarCommand

@dataclass(frozen=True)
class CollisionContact:
    counterpart: str | None
    at_monotonic_s: float

@dataclass(frozen=True)
class CollisionState:
    in_contact: bool
    last_contact: CollisionContact | None

@dataclass(frozen=True)
class CollisionRecoverySettings:
    backoff_m: float
    maximum_recoveries: int
    reset_after_progress_m: float
```

### 端口

```python
class Vehicle(Protocol):
    def current_pose(self) -> WorldPose: ...
    def motion_capability(self) -> MotionCapability: ...
    def execute_segment(self, segment: MotionSegment, *, budget: ExecutionBudget | None,
                        pose_checkpoint: Callable[[WorldPose, float], bool] | None) -> None: ...
    def cancel(self) -> None: ...
    def hover(self) -> None: ...
    def collision_state(self) -> CollisionState: ...

class StreamingVehicle(Protocol):
    def stream_state(self) -> TimedVehicleState: ...
    def stream_command(self, command: MotionCommand, duration_s: float) -> None: ...
```

`stream_velocity` 这个名字作废：它暗示全向速度契约。适配层收到与 `MotionCapability` 不符的命令必须抛错，不许静默丢弃分量。

### 执行层提供的两个原语

```python
# runtime/motion/loop.py
class ControlLoop:
    def run_segment(self, segment: MotionSegment, sink: SegmentSink, *,
                    budget: ExecutionBudget | None) -> ExecutionReceipt: ...
    def run_stream(self, goals: StreamGoalSource, sink: StreamSink, *,
                   deadline_monotonic: float | None) -> None: ...
```

- `SegmentSink.apply(command: MotionCommand, duration_s: float)`：引擎侧唯一需要实现的动作接口，由适配层包在 `execute_segment` 内。
- `StreamGoalSource.next(state) -> MotionCommand | None`：任务层提供目标，执行层负责伺服与限幅。
- 30 ms 与 50 ms 两个周期都由 `ControlLoop` 决定，引擎不决定节奏。

### 执行层提供的滑窗

```python
class PlanningWindow:
    def begin(self, *, deadline_monotonic: float | None, maximum_rounds: int | None) -> None: ...
    def submit(self, observation, context, instruction, planning) -> None: ...
    def poll_stream(self, measured, checkpoint) -> bool: ...
    def pause_for_completion(self, checkpoint) -> None: ...
    def complete(self) -> None: ...
    def end(self) -> None: ...
```

任务层只调这六个方法，看不到线程与队列。

### 任务层的对外常量

每个任务包导出 `TASK_KIND: TaskKind` 与 `REQUIRED_PORTS: tuple[str, ...]`。装配层只读这两个常量。

## 五、状态与失败语义

| 情形 | 谁产生 | 谁处理 | 结果 |
| --- | --- | --- | --- |
| 用户停止 | 入口层 | `runtime/execution.py` 检查点 | `ExecutionError` 沿层上抛，任务层转成 `StageResult(False)` |
| 时间预算耗尽 | `ControlLoop` 与 `runtime/execution.py` | 同上 | `BudgetExceeded`，剩余运动写进 `StageResult` |
| 碰撞接触 | 引擎端口 | `runtime/collision_recovery.py` | 后退、清队列、继续；恢复次数超限才抛 `ExecutionError` |
| 观测不新鲜 | 任务层 | 任务层策略 | 重置确认计数，重新取帧 |
| 模型超时 | 端口实现 | 任务层 `common/budget.py` | 计入调用预算，超限终止 |
| 端口与命令类型不符 | 适配层 | 构造期检查 | 构造期报错，不进运行期 |

进程边界不是功能归属：UE4 的任务循环跑在子进程里，是因为客户端库冲突。

## 六、交付物要求

每人交一份 `docs/refactor/<NN>-<层名>.md`，必须含八节：

1. 本层职责一句话，加文件清单表：文件、类与函数签名、估计行数、从哪来。
2. 数据流与时序：关键路径写到帧或段一级。
3. 跨层接口：我提供什么、我要求相邻层提供什么。名字必须用第四节的契约名。
4. 错误与失败路径：谁抛什么，谁捕获，怎么恢复。
5. 并发与所有权：线程、锁、谁持有控制权、切换载具时怎么交接。
6. 测试点：单测清单与断言对象，含负例。
7. 迁移步骤：从现状到目标的最小序列，每步可独立验证。
8. 未决问题：需要指挥层决定的，逐条列出并给两个以上选项与取舍。

## 七、禁止事项

- 不修改任何代码文件。只写自己的 `docs/refactor/<NN>-<层名>.md`。
- 不修改本文件，不修改别人的文档。
- 不运行长时间命令（模拟器、训练、下载）。
- 不引入新的第三方依赖。
- 不在方案里写"视情况而定"这类结论，要给一个默认选择。
- 引用的文件与符号必须真实存在，写完自查一遍路径与行号。
