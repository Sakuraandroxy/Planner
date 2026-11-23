> 上级：[项目架构](../ARCHITECTURE.md)
> 同层兄弟：[ports](./ports.md)、[algorithms](./algorithms.md)、[runtime](./runtime.md)、[workflows](./workflows.md)、[assembly](./assembly.md)、[adapters](./adapters.md)、[simulator](./simulator.md)、[simulators](./simulators.md)

# domain 层

## 1 一句话职责

`planner/domain/` 是全栈的公共词汇表：25 个模块 2678 行，导出 117 个名字（76 个类、23 个函数、18 个常量与别名），只依赖标准库与 numpy。

## 2 边界

管：

- 值类型的定义与构造期不变量校验，全部放在 `__post_init__`，抛 `ValueError`。
- 枚举与字符串常量：`TaskKind`、`MotionModel`、`VisualGoalStatus`、`MODE_*`、`WEATHER_*`、`VIEW_*`。
- 纯函数换算与校验：角度绕回（`planner/domain/pose.py:23`）、坐标旋转（`pose.py:54`）、深度协议（`planner/domain/depth.py:10`）、天气值校验（`planner/domain/weather.py:116`）。
- 一份配置 schema：`config/simulators.yaml` 的 `SimulatorFamily`、`FamilyCatalog`、`parse_family_catalog`（`planner/domain/engine_family.py:90`、`:189`、`:242`）。

不管：

- 端口 Protocol → [ports](./ports.md)。
- 异常类型层级 → `planner/errors.py`，契约层的第三个入口，单文件不成包。`planner/domain/` 不 import 它，全包只抛 `ValueError`。
- 规划、跟踪、伺服、碰撞退避的实现 → [algorithms](./algorithms.md) 与 [runtime](./runtime.md)。
- HTTP 响应体结构 → [adapters](./adapters.md)。domain 只提供 `to_payload` 级的中转数据（`WeatherCatalog.to_payload`）；剩余轨迹快照的构造与编解码在 [runtime](./runtime.md) 的 `telemetry.py`。
- 配置文件的读取时机与路径拼接 → `planner/config/loader.py`（adapters 与 assembly 的边界）。

## 3 输入与输出

- 入口：`planner/domain/__init__.py`，`__all__` 117 个名字。`MotionModel` 被入口 import 且被 `planner/algorithms/motion/corridor.py:5`、`planner/runtime/motion/tracking.py:17` 按入口路径取用，已补进 `__all__`。`tests/test_public_surface.py:149` 的 `HARDCODED["domain"]` 与入口逐名比对（`:273` 把它登记为 `CALIBRATED`，`:362` 执行比对），多一个或少一个都失败。
- 契约类型形态：`@dataclass(frozen=True)` 与 `class X(str, Enum)`。可变字段一律在 `__post_init__` 里用 `object.__setattr__` 归一（`planner/domain/observation.py:110`、`:137`）。
- 返回：纯值对象，无副作用，无 I/O。例外是 `parse_family_catalog` 接收已解析的 YAML 字典（`planner/domain/engine_family.py:242`）。
- 异常：只有 `ValueError`。ctor 校验失败、`require_supported` 命令与能力不符（`planner/domain/motion.py:102`）、`validate_view_mode` 未知视图（`planner/domain/view.py:66`）都是 `ValueError`。
- 层外引用面：`planner/` 内 113 个文件 import `planner.domain`，分布为 adapters 18、algorithms 12、assembly 8、ports 10、runtime 21、simulator 3、simulators 20、workflows 21。

## 4 对外名字表

调用点写层外文件；`—` 表示只有 `tests/` 引用，产品代码无调用点。

### 运动与命令（`motion.py`）

| 名字 | 类型 | 用途 | 层外调用点 |
| --- | --- | --- | --- |
| `MotionModel` | `str, Enum` | 全向 / 阿克曼两种运动模型（入口 import 但不在 `__all__`，见 Qdomain13） | `simulators/airsim/vehicle.py:41`、`simulators/projectairsim/drone.py:89`、`rover.py:133` |
| `MotionCapability` | 冻结 dataclass | 一台载具的运动学上限，构造期拒绝矛盾组合 | `runtime/motion/tracking.py`、三个引擎实现 |
| `HolonomicCommand` | 冻结 dataclass | 一拍世界系速度与偏航角速度 | `runtime/continuous_execution.py:142`、`runtime/motion/tracking.py:61` |
| `CarCommand` | 冻结 dataclass | 一拍有符号前向速度与偏航角速度 | `runtime/motion/tracking.py:60`、`simulators/projectairsim/drive.py` |
| `MotionCommand` | 类型别名 | `HolonomicCommand \| CarCommand`，只用于标注 | 三个引擎的 `stream_command` |
| `command_type_for` | 函数 | 按能力给出命令类 | 只被 `require_supported` 调用 |
| `require_supported` | 函数 | 命令与载具能力不符时抛错 | `simulators/airsim/vehicle.py:223`、`drone.py:292`、`rover.py:268` |
| `MotionLimits` | 冻结 dataclass | 运动包络与跟踪容差，配置与规划共用一份 | `config/loader.py:223`、`runtime/motion/loop.py`、`workflows/*/workflow.py` |
| `HeadingHoldSettings` | 冻结 dataclass | 位置保持航向环的四个参数 | `config/loader.py:231` |
| `ContinuousTrackingSettings` | 冻结 dataclass | 连续跟踪前瞻与加加速度上限 | `config/loader.py:260`、`runtime/motion/continuous.py`、`runtime/position_hold.py` |
| `ContinuousReference` | 冻结 dataclass | 一拍参考点：位置、剩余距离、对齐标志 | `runtime/motion/continuous_reference.py` |
| `CollisionRecoverySettings` | 冻结 dataclass | 退避距离、恢复次数、进度清零阈值 | `config/sections.py:359`、`runtime/collision_recovery.py` |

### 位姿与轨迹（`pose.py`、`trajectory.py`）

| 名字 | 类型 | 用途 | 层外调用点 |
| --- | --- | --- | --- |
| `WorldPose` | 冻结 dataclass | 世界系位姿，z 向下，yaw 顺时针为正 | 27 个 `planner/` 文件，`runtime/segment_motion.py`、`simulators/projectairsim/coordinates.py` 等 |
| `RelativePoseDelta` | 冻结 dataclass | 机体系位移请求，轨迹模型输出的单位 | `algorithms/trajectory_transform.py`、`adapters/trajectory_planner/response_parser.py` |
| `wrap_deg` | 函数 | 角度绕回到 (-180, 180] | `runtime/segment_motion.py`、`simulators/projectairsim/coordinates.py` |
| `wrap_yaw_deg` | 函数 | 流式变换用的绕回，吸附亚毫度噪声 | 12 个文件，`runtime/motion/tracking.py`、`workflows/basic_motion/workflow.py` 等 |
| `resolve_yaw_delta_deg` | 函数 | 保留显式转向方向的偏航增量 | `runtime/motion/loop.py`、`runtime/window/queue.py`、`workflows/visual_goal_navigation/completion.py` |
| `relative_to_world` | 函数 | 机体系偏移旋到世界系 | `algorithms/trajectory_transform.py`、`runtime/window/inflight.py`、`workflows/visual_goal_navigation/approach.py` |
| `MotionSegment` | 冻结 dataclass | 一段可执行运动，起止位姿加 profile | `runtime/guided_vehicle.py`、`simulators/airsim/vehicle.py:74`、`workflows/basic_motion/workflow.py` |
| `MotionTrajectory` | 冻结 dataclass | 有序段集合，`remaining` 字段用它表达 | `runtime/motion/loop.py`、`runtime/window/window.py`、`workflows/visual_goal_navigation/flight.py` |
| `RelativeTrajectory` | 冻结 dataclass | 机体系航点序列，必须是 tuple | `algorithms/trajectory_validation.py`、`runtime/window/queue.py`、`adapters/trajectory_planner/response_parser.py:40` |
| `WorldTrajectory` | 冻结 dataclass | 世界系位姿序列，可选逐段偏航增量 | `algorithms/trajectory_transform.py`、`runtime/window/window.py`、`workflows/visual_goal_navigation/completion.py` |

### 执行（`execution.py`）

| 名字 | 类型 | 用途 | 层外调用点 |
| --- | --- | --- | --- |
| `ExecutionBudget` | 冻结 dataclass | 单调钟截止时刻，`remaining(now)` 截到 0 | 11 个文件，`runtime/motion/loop.py`、`workflows/common/budget.py` 等 |
| `ExecutionReceipt` | 冻结 dataclass | 一批段的执行回执：完成段数、末位姿、实测弦长 | `runtime/motion/loop.py:278`、`workflows/visual_goal_navigation/flight.py` |
| `ExecutionRecord` | 冻结 dataclass | 一批的实测端点记录，带位移与偏航累计属性 | `runtime/motion/record.py:70`、`runtime/window/window.py:151`、`workflows/free_navigation/workflow.py:267` |
| `StopResult` | 冻结 dataclass | 停机确认与失败原因元组 | `runtime/execution.py`、三个任务包的 workflow |
| `CollisionContact` | 冻结 dataclass | 一次已捕获接触的对手名与读取时刻 | `runtime/collision_recovery.py`、`simulators/projectairsim/drone.py`、`rover.py` |
| `CollisionState` | 冻结 dataclass | 一次碰撞端口读取的结果 | `runtime/collision_recovery.py`、三个引擎实现 |
| `PoseCheckpoint` | 类型别名 | `Callable[[WorldPose, float], bool]`，返回 True 表示请求暂停 | `simulators/airsim/vehicle.py` |

### 任务、结果与参数（`mission.py`、`result.py`、`basic_motion.py`、`free_navigation.py`）

| 名字 | 类型 | 用途 | 层外调用点 |
| --- | --- | --- | --- |
| `TaskKind` | `str, Enum` | 三类任务：数值动作、视觉接近、无终点巡航 | 15 个 `planner/` 文件，`assembly/workflow_router.py`、`adapters/task_parser/task_factory.py` 等 |
| `TaskParameters` | 类型别名 | 三种参数 dataclass 的联合 | `adapters/task_parser/tasks/__init__.py` |
| `MissionStage` | 冻结 dataclass | 一个阶段：id、类型、参数 | 11 个文件，`workflows/base.py`、`assembly/mission_validator.py` 等 |
| `MissionPlan` | 冻结 dataclass | 带 `protocol_version` 的阶段序列 | `assembly/application.py`、`assembly/mission_runner.py`、`adapters/task_parser/openai_task_parser.py` |
| `MissionResult` | 冻结 dataclass | 整次任务的成功标志与阶段结果 | `assembly/reporting.py`、`simulators/airsim/session.py`、`simulators/projectairsim/session.py` |
| `StageResult` | 泛型冻结 dataclass | 单阶段结果，失败时不得声称完成确认 | 6 个 workflow 文件，`assembly/mission_runner.py` |
| `StageSummary` | 冻结 dataclass | 与任务类型无关的阶段摘要，带完成依据枚举 | `workflows/common/evidence.py`、三个任务包 workflow |
| `CompletionBasis` | `str, Enum` | 完成判据来源：未验证 / 本地实测 / API 执行 / 模型加本地校验 | 三个任务包 workflow |
| `ResultMeasurement` | 冻结 dataclass | 一条具名测量值，带单位 | 三个任务包 workflow |
| `BasicMotionParameters` | 冻结 dataclass | 无目标相对动作：前进、上升、转向 | `algorithms/basic_motion_conversion.py`、`workflows/basic_motion/policy.py`、`adapters/task_parser/tasks/basic_motion.py` |
| `BasicMotionStatus` | `str, Enum` | 完成 / 执行错误 | `workflows/basic_motion/workflow.py` |
| `BasicMotionResult` | 冻结 dataclass | 起止位姿、耗时、停机结果 | `workflows/basic_motion/workflow.py` |
| `FreeNavigationParameters` | 冻结 dataclass | 只有指令文本的无界任务参数 | `workflows/free_navigation/workflow.py`、`adapters/task_parser/tasks/free_navigation.py` |
| `FreeNavigationStatus` | `str, Enum` | 六种结束状态，没有「完成」 | `workflows/free_navigation/workflow.py` |
| `FreeNavigationResult` | 冻结 dataclass | 规划次数、行驶距离、末位姿 | `workflows/free_navigation/workflow.py` |

### 视觉目标（`visual_goal.py`、`visual_goal_limits.py`、`visual_goal_completion.py`、`target_detection.py`、`target_state.py`、`depth_foreground.py`）

| 名字 | 类型 | 用途 | 层外调用点 |
| --- | --- | --- | --- |
| `ArrivalRelation` | `str, Enum` | 只有 `NEAR` 一个成员 | `adapters/task_parser/tasks/visual_goal_navigation.py` |
| `VisualGoalNavigationParameters` | 冻结 dataclass | 指令、目标描述、到达关系、是否保持可见 | `workflows/visual_goal_navigation/policy.py`、`adapters/task_parser/tasks/visual_goal_navigation.py` |
| `GoalVisibility` | `str, Enum` | 可见 / 不可见 / 歧义 | 6 个文件，`algorithms/targeting/association.py`、`workflows/visual_goal_navigation/targeting.py` 等 |
| `GoalObservation` | 冻结 dataclass | 一帧的检测结论：框、表面点、证据、同目标标志 | `algorithms/targeting/depth_geometry.py`、`workflows/visual_goal_navigation/targeting.py` |
| `SurfacePointEstimate` | 冻结 dataclass | 单个表面点的反投影结果与失效原因 | `algorithms/targeting/depth_geometry.py` |
| `GoalGeometry` | 冻结 dataclass | 距离、世界点、样本与深度滤波结果 | `algorithms/targeting/depth_geometry.py`、`workflows/visual_goal_navigation/session.py` |
| `GoalReference` | 冻结 dataclass | 阶段起始的外观与位置锚点，不随最新估计漂移 | `algorithms/targeting/association.py`、`workflows/visual_goal_navigation/targeting.py` |
| `VisualGoalState` | `str, Enum` | 搜索 / 接近 / 核验 | `workflows/visual_goal_navigation/approach.py`、`continuous.py` |
| `VisualGoalStatus` | `str, Enum` | 12 种结束状态，含 `REACHED` | 5 个 workflow 文件，`workflows/visual_goal_navigation/termination.py` 等 |
| `VisualGoalPlanningContext` | 冻结 dataclass | 给规划器的任务上下文，装配层原样透传 | `algorithms/planning/geometric_planner.py`、`adapters/trajectory_planner/prompts/visual_goal_navigation.py` |
| `VisualGoalNavigationResult` | 冻结 dataclass | 距离、路径、确认次数、剩余运动 | `workflows/visual_goal_navigation/workflow.py` |
| `VisualGoalLimits` | 冻结 dataclass | 视觉接近的全部策略阈值，21 个字段 | `config/loader.py:453`、`workflows/visual_goal_navigation/policy.py` |
| `VisualGoalTriggerRules` | 冻结 dataclass | 完成检查半径、重触发条件、检查上限 | `config/loader.py:458`、`workflows/visual_goal_navigation/completion.py` |
| `VisualGoalTargetSnapshot` | 冻结 dataclass | 阶段绑定的目标位置与捕获时刻 | `workflows/visual_goal_navigation/targeting.py`、`completion.py` |
| `VisualGoalCompletionInput` | 冻结 dataclass | 一次语义完成检查的全部输入 | `workflows/visual_goal_navigation/completion.py`、`adapters/task_completion/prompts/visual_goal_completion.py` |
| `NormalizedBoundingBox` | 冻结 dataclass | 归一化到全图、左上原点的框 | `adapters/target_detection/client.py`、`manual.py` |
| `TargetDetection` | 冻结 dataclass | 检测结论：找到标志、框、至多 5 个表面点 | `adapters/target_detection/client.py`、`manual.py` |
| `TargetDetectionInput` | 冻结 dataclass | 一次检测请求：观测、指令、目标描述、可选参考图 | `adapters/target_detection/client.py`、`workflows/visual_goal_navigation/targeting.py` |
| `TargetObservationEvidence` | 冻结 dataclass | 最新检测声明与被拒原因，与已接受状态分开 | `workflows/visual_goal_navigation/targeting.py`、`session.py` |
| `TargetPositionState` | 冻结 dataclass | 阶段内的锚点、当前世界点与表面点集合 | `algorithms/targeting/association.py`、`workflows/visual_goal_navigation/completion.py` |
| `DepthForegroundSettings` | 冻结 dataclass | 深度前景滤波开关、半径、跳变比 | `config/loader.py:460`、`algorithms/targeting/depth_foreground.py` |
| `DepthForegroundResult` | 冻结 dataclass | 滤波是否生效、拒绝点数、点的分层 | `algorithms/targeting/depth_foreground.py` |
| `DepthPointLayer` | 冻结 dataclass | 单点的近层与远层深度区间 | `algorithms/targeting/depth_foreground.py` |

### 观测与深度显示（`observation.py`、`depth.py`）

| 名字 | 类型 | 用途 | 层外调用点 |
| --- | --- | --- | --- |
| `Observation` | 冻结 dataclass | 停稳同步的 RGB 加深图，含内参、相机矩阵、时间戳 | 15 个文件，`algorithms/targeting/depth_geometry.py`、`adapters/trajectory_planner/qwen_vl_planner.py` 等 |
| `CameraIntrinsics` | 冻结 dataclass | 针孔内参，`from_horizontal_fov` 由水平视场反算 | `simulators/airsim/camera_model.py`、`simulators/projectairsim/camera.py`、`algorithms/targeting/depth_surface_sampling.py` |
| `SHARED_DEPTH_MAPPING` | 字符串常量 | 当前显示协议 `log1p_inverse_gray_v2` | — |
| `INVALID_DEPTH_SENTINEL_M` | 浮点常量 | 65500.0，引擎侧无效深度哨兵 | `algorithms/free_space.py` |
| `FAR_VALID_PIXEL_VALUE` | 整型常量 | 16，最远有效深度对应的灰度值 | — |
| `LEGACY_DEPTH_MAPPING` | 字符串常量 | 旧显示协议 `log_inverse_gray_v1`，`render_depth` 的默认值 | `algorithms/depth/render.py` |
| `validate_depth_protocol` | 函数 | 校验映射名与上下限是否匹配 | `config/sections.py`、`algorithms/depth/render.py` |
| `depth_display_description` | 函数 | 生成给模型提示词的深度说明文本 | `adapters/trajectory_planner/prompts/base.py`、`adapters/task_completion/prompts/visual_goal_completion.py` |

### 滑窗与流式（`sliding_window.py`、`planning_window.py`、`streaming.py`）

| 名字 | 类型 | 用途 | 层外调用点 |
| --- | --- | --- | --- |
| `SlidingWindowSettings` | 冻结 dataclass | 补点数量、时延余量、任务时长、观测年龄上限 | `config/loader.py:368`、`runtime/window/window.py`、两个连续任务 workflow |
| `QueuedWaypoint` | 冻结 dataclass | 队列里的一段：起止位姿与偏航增量 | `runtime/window/queue.py`、`algorithms/motion/reached.py` |
| `PlanningSnapshot` | 冻结 dataclass | 冻结的一次续规划请求，含未执行前缀 | `runtime/window/window.py`、`queue.py`、`inflight.py` |
| `ContinuationCandidate` | 冻结 dataclass | 候选快照与对应世界系轨迹 | `runtime/window/inflight.py` |
| `PlanningOutput` | 冻结 dataclass | 一次规划的相对轨迹与时延 | `runtime/window/worker.py`、`inflight.py` |
| `PlanningWindowInput` | 冻结 dataclass | 窗口的冻结输入：初始位姿、年龄上限、请求超时 | `runtime/window/window.py`、两个连续任务 workflow |
| `TimedVehicleState` | 冻结 dataclass | 实测位姿、时间戳、速度、姿态、偏航角速度 | 7 个文件，`runtime/position_hold.py`、`simulators/projectairsim/coordinates.py` 等 |
| `QueueObservationAnchor` | 冻结 dataclass | 队列修订号与锚点快照，用于图像与队列对齐 | `runtime/stream_timeline.py`、两个连续任务 workflow |

### 引擎族与能力位（`engine_family.py`）

| 名字 | 类型 | 用途 | 层外调用点 |
| --- | --- | --- | --- |
| `Capabilities` | 冻结 dataclass | 控制台可提供的载具、视图、动作、任务类型、天气字段 | `assembly/requirements.py:97`、`assembly/controller/__init__.py` |
| `SimulatorFamily` | 冻结 dataclass | `simulators.yaml` 里一个引擎的启动参数、端口、相机、连接信息 | `adapters/http/schemas.py`、`config/loader.py` |
| `FamilyCatalog` | 冻结 dataclass | 引擎条目集合，带 `entry` / `require` / `ids` / `labels` | `config/loader.py:127`、`config/vehicles.py` |
| `parse_family_catalog` | 函数 | 校验并解析 `simulators.yaml`，未知键报错 | `config/loader.py:127` |

### 视图与天气（`view.py`、`weather.py`）

| 名字 | 类型 | 用途 | 层外调用点 |
| --- | --- | --- | --- |
| `VIEW_MODES` | 元组常量 | 四路相机环的固定顺序 | `config/maps.py`、`config/sections.py`、`simulators/projectairsim/session.py`、`scripts/streaming_views.py` |
| `VIEW_LABELS` | 字典常量 | 视图中文名 | — |
| `VEHICLE_FRONT_VIEW` | 字典常量 | 载具到前视相机名 | — |
| `VEHICLE_DEFAULT_VIEW` | 字典常量 | 载具到默认追尾相机名 | — |
| `front_view_for` | 函数 | 取前视相机名，未知载具抛错 | — |
| `default_view_for` | 函数 | 取默认相机名 | 6 个文件，`assembly/controller/state.py`、`adapters/http/server_api.py` 等 |
| `validate_view_mode` | 函数 | 校验视图名在环内 | `simulators/projectairsim/session.py`、`assembly/controller/weather.py` |
| `view_index` | 函数 | 视图名在环里的下标 | — |
| `cycle_delta` | 函数 | 从当前下标到目标下标的前向步数 | `simulators/projectairsim/session.py:357` |
| `WEATHER_FIELDS` | 元组常量 | 八个天气字段，顺序对应引擎枚举 1..8 | `simulators/airsim/weather.py`、`simulators/projectairsim/weather.py`、`simulators/*/runtime.py` |
| `WEATHER_LABELS` | 字典常量 | 字段中文名 | — |
| `WEATHER_CATALOG` | `WeatherCatalog` 实例 | 字段与六个预设的静态目录 | `simulators/projectairsim/session.py`、`simulators/projectairsim/runtime.py` |
| `WeatherCatalog` | 冻结 dataclass | 天气目录，`to_payload` 是跨进程线格式 | `simulator/mission_child.py`、`simulators/projectairsim/session.py` |
| `default_weather` | 函数 | 八字段全零 | — |
| `merge_weather` | 函数 | 部分更新叠加到当前状态 | `assembly/controller/weather.py` |
| `validate_weather_values` | 函数 | 校验完整八字段且值在 [0, 1] | `simulators/airsim/session.py`、`simulators/projectairsim/weather.py` |
| `weather_catalog_from_payload` | 函数 | 由线格式还原目录 | `simulator/subprocess_worker.py` |

## 5 关键流程

1. 配置落到契约对象。`config/core/<profile>/base.yaml` 的 `motion` 段在 `planner/config/loader.py:223` 转成 `MotionLimits`，其中航向环在 `:231` 单独构造 `HeadingHoldSettings`；`motion.collision` 段在 `planner/config/sections.py:359` 转成 `CollisionRecoverySettings`。构造即校验，YAML 里写错类型或矛盾值在启动期抛 `ValueError`。
2. 观测进入契约。引擎侧造 `Observation`，`planner/domain/observation.py:107` 起校验：`rgb` 必须 HxWx3、`depth_m` 必须匹配 `depth_intrinsics`、`camera_matrix` 必须 4x4、`depth_kind` 只能是两种取值；缺省字段在这里补齐（`depth_intrinsics`、`rotation_camera_to_world`、`camera_position_world`）。
3. 规划输出落到 `RelativeTrajectory`。`planner/adapters/trajectory_planner/response_parser.py:40` 构造，构造期只校验是 tuple 且元素类型正确（`planner/domain/trajectory.py:16`）；数值合法性由 `planner/algorithms/trajectory_validation.py` 负责。
4. 命令下发前过能力门。`planner/runtime/motion/tracking.py:60` 与 `:61` 按 `MotionCapability.model` 造 `CarCommand` 或 `HolonomicCommand`；引擎侧 `stream_command` 第一行调 `require_supported`（`planner/simulators/airsim/vehicle.py:223`、`planner/simulators/projectairsim/drone.py:292`、`rover.py:268`），实例类型与模型不符或倒车不被允许时抛 `ValueError`。
5. 遥测跨进程。`planner/runtime/trajectory_hub.py` 调 `build_queue_snapshot` 造快照；`planner/simulator/mission_child.py` 调 `snapshot_to_payload` 塞进管道；`planner/simulator/subprocess_worker.py` 用 `snapshot_from_payload` 还原后交给 `adapters/http`。三个函数都在 `planner/runtime/telemetry.py`。
6. 结果上行。三个任务包各自造 `StageSummary` 与 `StageResult`（`planner/workflows/common/evidence.py`），`planner/assembly/mission_runner.py` 汇总成 `MissionResult`。

## 6 失败模式

| 报错类型 | 触发条件 | 调用方处理 |
| --- | --- | --- |
| `ValueError`（`MotionCapability`） | 阿克曼给了横向速度或非正转弯半径；全向给了 `min_turn_radius_m`（`planner/domain/motion.py:24`） | 引擎构造函数里抛出，启动即失败，不进运行期 |
| `ValueError`（`CarCommand`） | `reverse` 与 `forward_speed_mps` 符号不一致（`planner/domain/motion.py:79`） | 命令构造函数抛出，属于编程错误 |
| `ValueError`（`require_supported`） | `HolonomicCommand` 发给阿克曼车，或 `allows_reverse` 为假时收到倒车命令（`planner/domain/motion.py:102`） | 引擎 `stream_command` 直接抛，往上到 `runtime` 的 `ExecutionError` 路径 |
| `ValueError`（`Observation`） | 图像形状、内参、`depth_kind`、矩阵尺寸不符（`planner/domain/observation.py:107`） | 适配层捕获转 `ObservationError`；`domain` 自己不捕获 |
| `ValueError`（`validate_depth_protocol`） | 映射名未知，或 `log1p_inverse_gray_v2` 的区间不是 0-200（`planner/domain/depth.py:10`） | 配置加载期抛；显示期由 `planner/algorithms/depth/render.py:24` 开头再校验一次 |
| `KeyError`（`weather_catalog_from_payload`） | 线格式里字段缺 `id` 或 `label`（`planner/domain/weather.py:93`） | 未包装成 `ValueError`，与快照解码不一致，见 Qdomain11 |
| `ValueError`（`parse_family_catalog`） | `capabilities` 块出现、未知键、缺 `argv`、端口越界、`driver: script`（`planner/domain/engine_family.py:242` 起） | 启动期抛，报错文本带引擎 id 与期望键集合 |
| `ValueError`（`VisualGoalLimits`） | 任一字段非有限数、`confirmation_count < 2`、`search_step > search_half_span`、扫描步数超过 12（`planner/domain/visual_goal_limits.py:53`） | 配置加载期抛 |

配置层的转换失败没有异常包装：`planner/config/loader.py` 里 `float(...)` / `int(...)` 的 `TypeError`、`ValueError` 直接冒到启动路径，报错文本是 Python 原生格式，不带字段路径。

## 7 测试位置

跑法：`~/miniconda3/envs/vln/bin/python -m pytest -q tests/test_motion_contract.py tests/test_weather_domain.py tests/test_view_catalog.py tests/test_depth_visualization.py tests/test_intrinsics_validation.py tests/test_target_detection.py tests/test_public_surface.py tests/test_layer_entries.py tests/test_layering.py tests/test_runtime_catalog.py tests/test_motion.py`，本轮结果 158 passed。

覆盖到的：`tests/test_motion_contract.py` 逐条打 `MotionCapability`、`CarCommand`、`require_supported`、`HeadingHoldSettings`、`CollisionState` 的拒绝分支与冻结性；`tests/test_weather_domain.py` 覆盖八字段顺序、预设、合并与拒绝；`tests/test_view_catalog.py` 覆盖环下标与前向步数；`tests/test_depth_visualization.py` 覆盖两套映射的灰度区间；`tests/test_intrinsics_validation.py` 覆盖内参边界；`tests/test_target_detection.py` 覆盖框与表面点约束；`tests/test_runtime_catalog.py:43` 起覆盖 `parse_family_catalog` 的未知键、`capabilities` 块、`script` driver、空目录；`tests/test_trajectory_hub.py` 与 `tests/test_subprocess_worker.py` 覆盖快照编解码；`tests/test_public_surface.py` 锁定 117 个导出名；`tests/test_layer_entries.py` 与 `tests/test_layering.py` 锁定 import 只能指向包入口。

没覆盖的：

- `ExecutionRecord.signed_yaw_deg`（`planner/domain/execution.py:107`）无调用点、无测试。
- `WeatherField`、`WeatherPreset` 没有 `__post_init__`，错误线格式只在 `weather_catalog_from_payload` 处暴露为 `KeyError`，无负例测试。
- `WeatherCatalog.to_payload` 与 `weather_catalog_from_payload` 没有往返测试；`tests/test_weather_domain.py:48` 只测字段与标签。
- `VisualGoalLimits` 的复合约束（`search_half_span_deg / search_step_deg > 12`）与 `maximum_vertical_offset_m` 跳过校验的分支没有负例。

本轮没有跑覆盖率：环境里没有 `pytest-cov`，也没有 `coverage`（`~/miniconda3/envs/vln/bin/python -c "import coverage"` 报 `ModuleNotFoundError`）。覆盖面结论来自逐个测试文件的断言对象，不是行覆盖率数字。

## 8 与其他层的联动

- 上游：无。`planner/domain/*` 只 import 标准库、numpy 与包内相对模块，`tests/test_architecture.py:79` 起把「共享层不 import 引擎客户端」写成测试。
- 下游：`planner/` 内 113 个文件 import `planner.domain`。契约改字段名会同时打到 adapters 18 个文件、algorithms 12、assembly 8、ports 10、runtime 21、simulator 3、simulators 20、workflows 21。
- 数据形态：全部是冻结 dataclass 与枚举，没有身份（identity）语义，可以安全跨线程传引用；带 numpy 数组的 `Observation` 是唯一例外，数组内部可变。
- 与 ports 的关系：ports 只 import `planner.domain`，10 个端口模块的签名全部用本层类型（`ExecutionBudget`、`Observation`、`WorldPose`、`MotionSegment`、`MotionCapability`、`TimedVehicleState`、`MotionCommand`、`CollisionState`、`GoalObservation`、`GoalGeometry`）。

## 9 疑问与争议

- Qdomain01 层：domain 结论：契约层里放了渲染、编解码与 YAML 解析的实现，不只是类型。依据：`planner/domain/depth_visualization.py:16`（34 行 numpy 渲染）、`planner/domain/trajectory_telemetry.py:71`/`:116`（快照编解码约 90 行）、`planner/domain/engine_family.py:242`（YAML 解析）。影响：是
- Qdomain02 层：domain 结论：`engine_family.py` 306 行是 `config/simulators.yaml` 的配置 schema，不是跨层运行期契约，层外只有 `config/loader.py:127` 与 `adapters/http/schemas.py` 用它。依据：`planner/domain/engine_family.py:229` 的 `KNOWN_FAMILY_KEYS` 与 `:90` 的 `SimulatorFamily`。影响：是
- Qdomain03 层：domain 结论：`CameraMountPose` 与 `Point3` 已在本轮删除，全仓无引用。依据：提交 `refactor(domain): 删除 CameraMountPose 与 Point3`。影响：否
- Qdomain04 层：domain 结论：`HeadingHoldSettings` 的 docstring 指向 `services/position_hold.py`，该包已改名 `runtime`。依据：`planner/domain/motion.py:175`。影响：否
- Qdomain05 层：domain 结论：`pose.py` 与 `observation.py` 用 CARLA 解释坐标系与内参，仓库里没有 CARLA 引擎实现，只有 airsim 与 projectairsim。依据：`planner/domain/pose.py:13`、`planner/domain/observation.py:46`、`:94`。影响：否
- Qdomain06 层：domain 结论：`MotionLimits` 的回落值在 `config/loader.py:224` 与 `planner/domain/motion.py:217` 各写一份，两处都无出处注释，且与 `config/core/default/base.yaml:29` 的 5.0 不一致。依据：`planner/config/loader.py:224`、`planner/domain/motion.py:217`。影响：否
- Qdomain07 层：domain 结论：`BasicMotionResult.completion_basis` 是字符串，与 `result.CompletionBasis` 枚举表达同一概念，两种表示并存。依据：`planner/domain/basic_motion.py:58`、`planner/domain/result.py:13`。影响：是
- Qdomain08 层：domain 结论：`ArrivalRelation` 只有 `NEAR` 一个成员，构造函数拒绝其他值，参数等于常量。依据：`planner/domain/visual_goal.py:11`、`:28`。影响：否
- Qdomain09 层：domain 结论：`ExecutionRecord.signed_yaw_deg` 无调用点、无测试。依据：`planner/domain/execution.py:107`。影响：否
- Qdomain10 层：domain 结论：`snapshot_to_payload` 不带 `updated_monotonic`，跨进程还原后该字段恒为 0，编解码不是往返等价；`tests/test_trajectory_hub.py:132` 记录了原因，但类型注释没写。依据：`planner/domain/trajectory_telemetry.py:71`、`:64`。影响：是
- Qdomain11 层：domain 结论：`weather_catalog_from_payload` 缺字段时抛 `KeyError` 且不包装，与 `snapshot_from_payload` 统一转 `ValueError` 的做法不一致。依据：`planner/domain/weather.py:93`、`planner/domain/trajectory_telemetry.py:162`。影响：否
- Qdomain12 层：domain 结论：`VIEW_LABELS`、`VEHICLE_FRONT_VIEW`、`VEHICLE_DEFAULT_VIEW`、`front_view_for`、`view_index`、`FAR_VALID_PIXEL_VALUE`、`SHARED_DEPTH_MAPPING`、`WEATHER_LABELS`、`default_weather`、`VehicleSnapshot`、`TrajectoryPointSnapshot` 只有测试引用，产品代码无调用点，是否该留在入口需要判断。依据：`planner/domain/view.py:32`、`planner/domain/trajectory_telemetry.py:32`。影响：否
- Qdomain13 层：domain 结论：`MotionModel` 曾被入口 import 却不在 `__all__`，已补进导出与测试期望集。依据：`planner/domain/__init__.py:48`、`:192`。影响：否

## 10 已知缺口

契约层不是纯类型层：一个模块带着实现，另外绝大多数数值默认值是没有标定记录的经验值。两项都按现状写在下面，逐条给调用点与影响。

### 10.1 带实现的一个模块（M1）

`planner/domain/engine_family.py`（306 行）：`SimulatorFamily`（`:90`）、`FamilyCatalog`（`:189`）、`parse_family_catalog`（`:242`）是 `config/simulators.yaml` 的 schema 与解析器。层外调用点：`planner/config/loader.py:127`、`planner/config/vehicles.py`、`planner/adapters/http/schemas.py`。风险：改键名等于改配置文件格式，`parse_family_catalog` 对未知键直接报错，旧配置会启动失败。

搬迁待办已按裁决记入 `docs/TODO.md`（由指挥层写，本层不代办）。

### 10.2 阈值是经验值（M7）

除航向环以外，本层所有数值默认值都没有标定记录，取值来自试飞观察，标注为「经验值，未标定」。

| 类型 | 默认取值 | 影响 |
| --- | --- | --- |
| `MotionLimits`（`motion.py:209`） | 限速 3.0 m/s、加速度 1.5 m/s²、偏航角速度 30 °/s、偏航角加速度 60 °/s²、`control_hz` 20、位置增益 1.0、偏航增益 2.0、位置容差 0.3 m、偏航容差 2.0°、超车走廊 0.3 m、停稳速度 0.15 m/s、停稳角速度 2.0 °/s、稳定时间 0.3 s、最短段时长 1.0 s | 决定段时长下限、停稳判据与「已驶过端点」走廊；走廊过窄会让阿克曼车在端点反复倒车 |
| `HeadingHoldSettings`（`motion.py:174`） | 增益 0.4、阻尼 0.0、死区 0.5°、角速度死区 1.5 °/s | 唯一有出处的组：`config/core/default/base.yaml` 注释指向 `docs/archive/measurements.md` 的「偏航执行链」 |
| `CollisionRecoverySettings`（`motion.py:114`） | 退避 3.0 m、最多 3 次、累计前进 20.0 m 清零 | 退避距离按次数线性放大，超过 3 次转 `ExecutionError` |
| `ContinuousTrackingSettings`（`motion.py:144`） | 前瞻 0.6 s、0.3-1.5 m、平移加加速度 3.0 m/s³、偏航加加速度 60 °/s³ | 前瞻决定跟线超前量，加加速度决定速度变化率上限 |
| `SlidingWindowSettings`（`sliding_window.py:11`） | 补点 3、时延余量 1.5 s、任务上限 600 s、连续失败 2、观测年龄 120 s、规划间隔 5 s、缓冲 10 | 规划间隔决定模型调用频率，观测年龄决定何时丢弃旧帧 |
| `VisualGoalLimits`（`visual_goal_limits.py:7`） | 21 个字段，关键值：距离余量 0.3 m、确认 2 次、确认间隔 0.5 s、搜索步长 30°、半幅 90°、任务上限 600 s、模型超时 20 s、观测年龄 45 s、锚点漂移 0.3 m、目标刷新 5 s | 搜索步数与半幅决定扫视覆盖，确认次数决定误判率 |
| `VisualGoalTriggerRules`（`visual_goal_completion.py:10`） | 完成半径 4.0 m、重触发位移 0.5 m、重触发偏航 10°、检查上限 10 次 | 完成半径直接决定停在哪里 |
| `DepthForegroundSettings`（`depth_foreground.py:6`） | 半径 3 px、跳变比 4.0 | 决定哪些像素被判为背景，影响表面点数量 |

生效方式：这些默认值只在配置文件缺键时生效。`planner/config/loader.py:224` 起对 `MotionLimits` 的字段逐个给回落值，`planner/config/loader.py:260`、`:368` 用 `**raw` 构造，缺键即取 dataclass 默认；引擎侧 `motion=None` 时也取默认（`planner/simulators/projectairsim/drone.py:88`）。因此改默认值会静默改变缺键配置的行为，也可能与 `config/core/*/base.yaml` 的值不一致，见 Qdomain06。

标定待办已按裁决记入 `docs/TODO.md`。
