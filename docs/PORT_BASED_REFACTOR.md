# 端口优先的重构设计

取代 `docs/PLANNER_UE4_PLAN.md` 第 4 节的架构部分。那份文件里的决策记录仍然有效，架构部分以本文为准。

## 0 为什么要重做

上一版把旧分支 `planner/` 整树复制成 `planner/ue4/`，12044 行。测量结果说明这是错的：

| 量 | 值 |
|---|---|
| main 的 `planner/` 总量 | 19393 行 / 178 文件 |
| 其中真正仿真器专有（`adapters/projectairsim` 加 session） | 4382 行，22% |
| 共享层 | 15011 行，77% |
| UE4 的适配层 `adapters/airsim` | 1450 行 |
| 真正 import 客户端库的文件 | 11 个（UE5 五个，UE4 六个） |
| 我复制的量 | 12044 行，其中约 10600 行是纯重复 |

两边同名文件 137 个，70 个只差格式、导入写法与 docstring。控制层 `services/motion/*` 全在这批里，逐字相同。

结论：仿真器差异集中在 11 个文件，不在 206 个文件。

## 1 原则

1. 仿真器之间的差异是**连接与控制**。用一个统一的抽象端口组表达，多个窄接口，不是一个上帝接口。
2. 每个仿真器按自己的能力实现其中一部分。缺的端口就是缺的能力。
3. 共享代码只对端口编程，不知道仿真器叫什么名字。
4. 行为不一致之处——UE4 能俯视、UE5 能切外观模型——各是一个独立端口。这些才是需要单独适配的地方。
5. **能力的唯一真值是"实现了哪些端口"**，不是配置文件里手写的表。

第 5 条删掉了上一版的配置块，见第 4 节。

main 已经部分符合：`services/execution.py`、`continuous_execution.py`、`position_hold.py` 只 import 端口；`workflows/free_navigation/continuous.py:54` 的 `lane_reference_provider=None` 已经是"可选端口加默认空"的写法。本文把这套做法推广到仿真器这一轴。

## 2 端口分组

全部放 `planner/ports/`，按角色分文件。标注 ○ 的是可选能力端口，仿真器可以完全不实现。

### 2.1 控制面：跨进程边界，父侧唯一可见

| 端口 | 方法 | 来源 |
|---|---|---|
| `BackendWorker` | `start`、`submit`、`stop`、`poll`、`shutdown`、`terminate` | main 已有 |
| `ConnectionMonitor` | `check`、`close` | 旧分支有，UE4 的心跳重连靠它，须并回 |
| `SimulatorRuntime` ○ | `home`、`activate`、`set_view_mode`、`set_weather`、`report_frames` | 从 `BackendWorker` 里拆出去，见 2.5 |

### 2.2 载具面

| 端口 | 方法 |
|---|---|
| `Vehicle` | `current_pose`、`execute_segment`、`cancel`、`hover`、`collision_state` |
| `StreamingVehicle` ○ | `stream_state`、`stream_velocity` |
| `PositionHoldVehicle` ○ | `stream_state`、`cancel`、`hold_velocity` |
| `BasicMotionVehicle` ○ | `current_pose`、`execute_basic_motion` |
| `ReturnHome` ○ | `return_home` |

### 2.3 感知面

| 端口 | 方法 |
|---|---|
| `ObservationSource` | `capture` |
| `ControlledObservationSource` ○ | `request_capture` |
| `MovingObservationSource` ○ | `capture_moving`、`close` |
| `CameraView` ○ | `read_mount`、`point`、`point_at_world`、`point_direction_world`、`point_down`、`restore` |
| `LaneReferenceProvider` ○ | `lane_reference` |

`CameraView` 就是 UE4 的俯视能力，六个方法原样来自旧分支 `adapters/airsim/above_camera.py`。UE5 没有实现它，于是俯视任务在 UE5 上不可用——这条结论不再需要写进配置或文档。

### 2.4 模型面：与仿真器无关

`TargetDetector`、`TopDetector` ○、`TaskCompletionChecker`、`TrajectoryPlanner`、`TaskParser`。

这组**不进仿真器目录**。它们对接的是推理服务，换仿真器不影响它们。上一版把它们复制了两遍。

`TopDetector` 与 `TargetDetector` 分开，是因为俯视检测的输入输出类型不同（`AboveDetectionInput` 对 `DetectionInput`），不是因为它属于 UE4。

### 2.5 副作用与仪表

| 端口 | 谁需要 |
|---|---|
| `TaskTrace`、`ControlTrace`、`CollisionCheck` | 已有 |
| `StreamingViews` ○ | UE5 的四个机位循环。实现者同时提供 `view_modes` 枚举 |
| `AppearanceModelControl` ○ | UE5 的外观模型切换，并提供可选型号枚举 |
| `WeatherControl` ○ | 两族都实现：UE5 走 `World.set_weather`，UE4 走 AirSim msgpack |
| `FramePublisher` ○ | UE5 的 MJPEG 帧流。UE4 不实现，画面走 xpra |
| `TrajectoryTelemetry` ○ | UE5 的航点队列上报 |

把这几个从 `BackendWorker` 里拆出来是本轮最重要的一处结构调整：现在 `BackendWorker` 有 10 个方法，其中 `home`、`activate`、`set_view_mode`、`set_weather` 是四种可选能力硬挤进一个协议的结果，实现者只能用 raise 或空函数敷衍。拆开后每个端口都可被真实实现或干净地缺席。

## 3 端口的装配与探测

```python title="planner/simulator/ports.py · 设计稿"
@dataclass(frozen=True)
class SimulatorPorts:
    """One simulator's implemented surface. None means the capability is absent."""
    vehicle: Vehicle
    observation: ObservationSource
    streaming: StreamingVehicle | None = None
    camera_view: CameraView | None = None
    top_detector: TopDetector | None = None
    weather: WeatherControl | None = None
    views: StreamingViews | None = None
    models: AppearanceModelControl | None = None
    frames: FramePublisher | None = None
    telemetry: TrajectoryTelemetry | None = None
    home: ReturnHome | None = None
    monitor: ConnectionMonitor | None = None

    def missing(self, required: tuple[str, ...]) -> tuple[str, ...]:
        return tuple(name for name in required if getattr(self, name) is None)
```

每个仿真器提供一个工厂：

```python title="planner/simulators/airsim/runtime.py · 设计稿"
def build_ports(config, log) -> SimulatorPorts:
    connection = AirSimConnection(config.host, config.port, config.timeout_s)
    vehicle = AirSimVehicle(connection, config.motion, config.drone)
    return SimulatorPorts(
        vehicle=vehicle,
        observation=AirSimObservationSource(connection, config.camera, config.depth),
        streaming=vehicle,                 # 旧分支已实现 stream_velocity
        camera_view=AirSimCameraView(connection, config.camera.camera_id),
        weather=AirSimWeather(connection),  # 移植 flight-console 的 msgpack 写法
        # views / models / frames / telemetry / home 不实现
    )
```

任务类型声明它需要什么，这张表是共享层唯一的分支来源：

```python title="planner/workflows/manifest.py · 设计稿"
TASK_REQUIREMENTS = {
    TaskKind.BASIC_MOTION: ("vehicle",),
    TaskKind.VISUAL_GOAL_NAVIGATION: ("vehicle", "observation"),
    TaskKind.VISUAL_GOAL_ABOVE: ("vehicle", "observation", "camera_view", "top_detector"),
    TaskKind.FREE_NAVIGATION: ("vehicle", "observation", "streaming"),
}
```

## 4 已定的 18 项决策如何自动成立

配置里的 `capabilities:` 整段删掉。下表全部由"实现了哪些端口"推导，`/api/status` 与 `/api/world/options` 输出推导结果。

| 任务类型 | 必需端口 | airsim | projectairsim |
|---|---|---|---|
| `basic_motion` | vehicle | ok | ok |
| `visual_goal_navigation` | vehicle, observation | ok | ok |
| `visual_goal_above` | + camera_view, top_detector | ok | no 缺 camera_view |
| `free_navigation` | + streaming | ok | ok |

| 控件 | 依据端口 | airsim | projectairsim |
|---|---|---|---|
| 载具列表 | 实现了 `vehicle` 的载具名 | drone | rover, drone |
| 回起点 | home | no | ok |
| 机位切换 | views | no 空枚举 | ok 四个 |
| 外观模型 | models | no | ok |
| 天气 | weather | ok msgpack | ok World API |
| 帧流 | frames | no 走 xpra | ok |
| 轨迹层 | telemetry | no | ok |

对照你已定的决策：UE4 纯无人机、UE5 不做俯视、UE4 隐藏回起点与轨迹、UE4 保留天气、机位只有 UE5 有——全部落在表里，一行配置都不用写。

顺带消掉两处配置说谎的可能：`config/base.yaml` 的 `server.vehicles` 与注册表的 `capabilities.vehicles` 现在是两份真相，将来只有装配出来的端口是真相。

## 5 目录

```tree list
planner/
  ports/                       唯一抽象，按第 2 节分组
  simulator/                   共用 xpra 进程栈：一套 argv 渲染加探针注册表
    ports.py                   SimulatorPorts
    stack.py                   EnvironmentService 那套状态机，两族共用
    argv.py                    argv 模板渲染，xpra 启动方式统一
  simulators/                  并列，同深度同形状
    airsim/                    UE4：adapter 1450 行加 runtime.py
    projectairsim/             UE5：现有 4382 行原样迁入
  domain services capabilities task_parser workflows  共享，一份
  application/ adapters/http/  共享，一份
  contracts/                   跨进程 DTO
console/ config/               一份
```

命名不再带引擎版本号：目录用软件名 `airsim` 与 `projectairsim`。地图条目用 `simulator:` 字段显式声明，无默认值。

`planner/ue4/` 整个删掉。

## 6 强制规则

规则不写下来就会被侵蚀，所以每条配一个测试。

| 规则 | 检查方式 |
|---|---|
| `planner/simulators/**` 之外不许 import `airsim`、`msgpackrpc`、`projectairsim` | 泛化现有 `tests/test_import_boundary.py`，遍历 AST 的 import 节点 |
| 工作流与服务的构造函数只接受端口类型，不接受 `PlannerConfig` 或具体 adapter 类 | 同一测试里检查构造函数注解的模块归属 |
| 能力位不许出现在 YAML 里 | 配置校验拒绝 `capabilities` 键 |
| 主进程不许加载任何仿真器客户端库 | 现有测试，已实现 |

## 7 迁移步骤

每步独立提交、可测、可单独回退。

| 步 | 内容 | 量 | 本地可验证 |
|---|---|---|---|
| 1 | 删 `planner/ue4`；把 6 个 airsim 文件与 `above_camera` 迁到 `simulators/airsim/adapter/`，类型换成主干的端口 | 约 1500 行移动 | 可以 |
| 2 | `adapters/projectairsim` 与 session 迁入 `simulators/projectairsim/` | 纯移动加改导入 | 可以，816 测试兜底 |
| 3 | `ports/` 补齐第 2 节的可选端口定义；`BackendWorker` 拆分 | 新增约 300 行 | 可以 |
| 4 | `SimulatorPorts` 与任务需求表；删 YAML 的 `capabilities`；HTTP 改读推导 | 约 250 行改 | 可以，重写门控测试 |
| 5 | 统一 xpra 进程栈：一套驱动两套 argv 模板 | 约 350 行 | 可以，假 runner |
| 6 | 并入共享任务层的旧独有文件：`workflows/visual_goal_above`、`capabilities/viewing` 与 `targeting/top_*`、`domain/visual_goal_above` 与 `above_polygon`，与 48 处分歧逐层裁决 | 约 2600 行并入，48 处要读 | 可以，逐层跑测试 |
| 7 | `simulators/airsim/runtime.py` 装配 UE4 任务链路 | 约 400 行 | 假客户端可以，真机不行 |

顺序有讲究：1 到 4 做完，UE4 与 UE5 就已经是并列形状，共享层唯一。第 6 步最贵但只是往共享层加能力，不再动形状。

## 8 这版设计没有解决的问题

- 第 6 步的 48 处实现分歧仍需逐个读。端口化不消除语义差异，只是把它暴露出来：其中约 20 处在模型客户端与提示词，与仿真器无关，统一取 main 版再把旧规则作为新任务类型加入。
- UE4 真机行为无法本地验证。旧 `README.md:435` 自己写明实际 AirSim 仍需联调。
- 跨进程后端口对象不能直接暴露给父侧。父侧只见 `BackendWorker` 与握手上报的能力清单，任务循环整个跑在子进程里。这与 UE5 目前"进程内线程 worker"的形态不同，第 3 与第 4 步要决定 UE5 是否也移进子进程。

## 9 本轮落地状态

已做完（第 7 节的 1、2、3 与部分 4、5）：

| 项 | 结果 |
|---|---|
| 删 `planner/ue4` | 完成。git 记为重命名，旧分支仍可单文件取回 |
| `simulators/{airsim,projectairsim}` 并列 | 完成，同深度同形状 |
| 端口层 | `planner/ports/capabilities.py` 八个可选端口；`SimulatorPorts` 与 `TASK_REQUIREMENTS` |
| 能力位改推导 | 完成。`config/simulators.yaml` 写 `capabilities` 会被拒绝 |
| 命名去偏心 | `family` 改 `simulator`，`ue4`/`ue5` 改 `airsim`/`projectairsim`，去掉 `default_family` 偏心 |
| airsim 适配层接干线类型 | 14 模块导入通过，`Vehicle`、`StreamingVehicle`、`PositionHoldVehicle`、`ObservationSource` 零缺失方法 |
| 俯视支持层进共享层 | 13 文件，含 `CameraMountPose` 上移到 `domain/pose.py` |
| xpra argv 统一 | 一套渲染路径，两份模板；supervisor 测试断言命令串逐字不变 |
| 架构规则可执行 | `tests/test_architecture.py` 用 AST 扫全仓 |

测试 851 passed（起点 816）。

## 10 本轮没做，以及为什么

| 剩项 | 为什么停 |
|---|---|
| `xpra_start` 进程驱动 | 需要真机验证窗口就绪与 `/proc` 所有权判据，写出来也无法本地证明 |
| UE4 会话装配（`build_ports` 真实调用方） | 上一条不做，装配没有消费者；留桩会腐烂 |
| `visual_goal_above` 端到端接线 | 要改 `domain/mission.py` 的 `TaskKind` 与 `task_parser` 注册，那属于 48 处分歧。这些改动同时影响 UE5 行为，而我这边跑不了任何一侧仿真器，把未验证的行为改动塞进共享层违背"逻辑保真" |
| UE5 移进子进程 | 保留进程内 worker 是既定取舍；`projectairsim` 因此仍在主进程被加载，`test_architecture.py` 的注释记下了这条边界 |
