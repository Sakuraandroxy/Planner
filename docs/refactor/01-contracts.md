# 01 契约层：`planner/domain/` 与 `planner/ports/`

契约层最终 21 个文件（`domain/`）加 13 个（`ports/`）。载具运动类型与命令联合全部落在 `domain/motion.py`，碰撞类型落在 `domain/execution.py`，两份 `MotionLimits` 合成 `domain/motion.py` 一份 15 字段（四个纬度字段收成 `HeadingHoldSettings`）。端口改两个文件、删一个、加一个：`vehicle.py` 加 `motion_capability()` 与新的 `collision_state()` 返回类型，`streaming_vehicle.py` 用 `stream_command` 替掉 `stream_velocity`（裁决 R2），`collision_check.py` 删除（裁决 R5），新增 `goal_source.py` 声明 `StreamGoalSource`（裁决 R16）。`Vehicle` 槽由 runtime 的 `GuidedVehicle(streaming, loop)` 组合对象兑现，引擎只实现 `stream_command` 一个数据面动作（裁决 R1）。`Capabilities` 与 `MotionCapability` 靠文件、字段与测试三重隔离，不共享命名空间。

## 一 职责与文件清单

一句话：本层只定义类型、常量、不变量校验与协议声明，不含函数逻辑、不读时钟、不开线程、import 任何其他层。

### 1.1 `planner/domain/`（21 个文件，估约 2260 行）

| 文件 | 类与函数签名 | 估行 | 从哪来 |
| --- | --- | --- | --- |
| `motion.py` | `MotionModel(str, Enum)`；`MotionCapability`；`HolonomicCommand`；`CarCommand`；`MotionCommand = HolonomicCommand \| CarCommand`；`require_supported(command, capability) -> None`；`HeadingHoldSettings`（四个纬向参数）；`MotionLimits`（15 字段）；`ContinuousTrackingSettings`；`ContinuousReference`（模块内私有，不导出）；`CollisionRecoverySettings` | 205 | 现 `domain/motion.py` 89 行 + `services/segment_motion.py:21-93` 的 `MotionLimits` 合入 + 新增 6 个类型与 1 个校验函数 + `HeadingHoldSettings`（裁决 R22） |
| `execution.py` | `PoseCheckpoint`；`ExecutionBudget`；`ExecutionReceipt`；`StopResult`；`ExecutionRecord`；新增 `CollisionContact`；新增 `CollisionState` | 95 | 现 69 行 + 26 行 |
| `engine_family.py` | `KNOWN_ACTIONS`；`KNOWN_DRIVERS`；`Capabilities`；`SimulatorFamily`；`FamilyCatalog`；`KNOWN_FAMILY_KEYS`；`NON_FAMILY_SECTIONS`；`parse_family_catalog(raw)` | 306 | 现 `domain/runtime.py` 原样改名，内容一字不改 |
| `sliding_window.py` | `SlidingWindowSettings`；`QueuedWaypoint`；`PlanningSnapshot`；`ContinuationCandidate`；`PlanningOutput`；并入 `PlanningWindowInput` | 135 | 现 99 行 + `domain/planning_window.py:9-35` 的 `PlanningWindowInput` |
| `pose.py` | `WorldPose`、`RelativePoseDelta`、`CameraMountPose`、`wrap_yaw_deg`、`resolve_yaw_delta_deg`、`relative_to_world` | 152 | 不动 |
| `observation.py` | `Observation`、`CameraIntrinsics` | 123 | 不动 |
| `trajectory.py` | `RelativeTrajectory`、`WorldTrajectory`、`MotionSegment`、`MotionTrajectory`、`SegmentResult` | 75 | 不动 |
| `mission.py` | `TaskKind`、`MissionStage`、`MissionPlan`、`TaskParameters` | 36 | 不动 |
| `result.py` | `CompletionBasis`、`ResultMeasurement`、`StageSummary`、`StageResult`、`MissionResult` | 66 | 不动 |
| `streaming.py` | `TimedVehicleState`（导出）；`ControlTelemetry`、`QueuePauseSnapshot`、`QueueObservationAnchor`（不导出，仅执行层消费） | 80 | 不动；`QueuePauseSnapshot` 内部化（裁决 R22） |
| `trajectory_telemetry.py` | 剩余航点快照与序列化 | 213 | 不动 |
| `server.py` | `ServerState`、`WorkerEvent`、`TaskSubmission`、`ServerStatus`、`LogBatch`、`BackendUnavailable` | 115 | 不动 |
| `weather.py` | `WEATHER_CATALOG`、`WEATHER_FIELDS`、`merge_weather` | 185 | 不动 |
| `view.py` | `VIEW_MODES`、`front_view_for`、`default_view_for`、`cycle_delta` | 82 | 不动 |
| `basic_motion.py` | 数值动作参数与结果 | 57 | 不动 |
| `free_navigation.py` | 自由导航参数与结果 | 41 | 不动 |
| `visual_goal.py` | `GoalObservation`、`GoalGeometry`、`GoalReference`、`VisualGoalState` 等 | 134 | 不动 |
| `visual_goal_limits.py` | 到达半径、`VisualGoalNavigationParameters` | 63 | 不动 |
| `visual_goal_completion.py` | `VisualGoalTriggerRules`、`VisualGoalCompletionInput` | 79 | 不动 |
| `approach.py` | 已删除：`ApproachSettings` 与 `TargetApproach.distance_within_radius` 随能力迁进 `workflows/visual_goal_navigation/policy.py`；`ApproachTarget`、`ApproachAssessment`、`ApproachStatus` 与 `assess` 删掉（裁决 R22、R23、R25） | 0 | 现 49 行删光，同 `capabilities/approach/target_approach.py` |
| `target_detection.py` | `NormalizedBoundingBox`、`TargetDetection`、`TargetDetectionInput` | 100 | 不动 |
| `target_state.py` | `TargetPositionState`、`TargetObservationEvidence` | 49 | 不动 |
| `depth.py` | 深度协议、前景过滤设置、深度渲染三个小文件合并为一个 | 95 | 现 `depth.py` 31 + `depth_foreground.py` 35 + `depth_visualization.py` 34 合入（裁决 R22） |
| `__init__.py` | 包入口：现有 7 个再导出，加 `MotionModel`、`MotionCapability`、`HolonomicCommand`、`CarCommand`、`MotionCommand`、`command_type_for`、`require_supported`、`CollisionContact`、`CollisionState`、`CollisionRecoverySettings`、`Capabilities`、`MotionLimits`、`ContinuousTrackingSettings`、`ExecutionBudget`、`ExecutionReceipt`、`StopResult`、`PoseCheckpoint`、`TimedVehicleState`、`PlanningWindowInput`、`SlidingWindowSettings` | 45 | 现有 14 行按裁决 R20 扩成完整导出；跨层只走这个入口 |

删除：`lane_reference.py`（99 行，决定纯视觉）、`planning_window.py`（36 行，并入 `sliding_window.py`）、`server.py`（115 行，7 个生命周期类型搬到 `assembly/`，裁决 R22）、`approach.py`（49 行：`ApproachSettings` 与 `distance_within_radius` 迁进 `workflows/visual_goal_navigation/policy.py`，其余三个类型与 `assess` 删掉，裁决 R25）。`depth_foreground.py` 与 `depth_visualization.py` 并入 `depth.py`（裁决 R22）。

任务相关类型（`basic_motion.py`、`free_navigation.py`、`visual_goal*.py`）留在 `domain/` 而不是迁进 `workflows/<task>/`：`adapters/task_parser/tasks/*` 与 `adapters/task_completion/prompts/visual_goal_completion.py` 必须引用这些类型，而 `adapters/` 只允许 import `domain/`、`ports/`、`algorithms/`。留在这里的只有数据结构，判定函数按 `docs/REFACTOR.md` 进各任务包 `policy.py`。`approach.py` 不在此列：`adapters/` 不引用它，所以整体迁出（裁决 R25）。

### 1.2 `planner/ports/`（13 个文件，裁决 R23）

| 文件 | 协议与签名 | 估行 | 从哪来 |
| --- | --- | --- | --- |
| `vehicle.py` | `Vehicle`：`current_pose() -> WorldPose`；`motion_capability() -> MotionCapability`；`execute_segment(segment: MotionSegment, *, budget: ExecutionBudget \| None = None, pose_checkpoint: PoseCheckpoint \| None = None) -> None`；`cancel() -> None`；`hover() -> None`；`collision_state() -> CollisionState` | 31 | 现 26 行，加 `motion_capability`，改 `collision_state` 返回类型。`execute_segment` 由 `runtime/motion/loop.py` 的 `GuidedVehicle(streaming, loop)` 兑现，引擎类不实现它（裁决 R1） |
| `streaming_vehicle.py` | `StreamingVehicle`：`stream_state() -> TimedVehicleState`；`stream_command(command: MotionCommand, duration_s: float) -> None` | 14 | 现 9 行，`stream_velocity` 删。这是唯一的命令下发签名，`SegmentSink`/`StreamSink` 都是它的别名（裁决 R2） |
| `goal_source.py` | `StreamGoalSource`：`next(state: TimedVehicleState, unwrapped_yaw_deg: float) -> MotionCommand \| None` | 10 | 新建。任务层实现（`runtime/motion/follower.py` 的 `QueueFollower`）、执行层消费（`ControlLoop.run_stream` 的 `goals`）的协议，按依赖规则必须落在 `ports/`（裁决 R16） |
| `position_hold_vehicle.py` | `PositionHoldVehicle`：`stream_state()`、`cancel()`、`hold_velocity(velocity_world, yaw_rate_deg_s, duration_s)` | 10 | 不动，签名保持；实现内部转调 `stream_command`（裁决 R7） |
| `observation_source.py` | `ObservationSource`、`ControlledObservationSource`、`MovingObservationSource` | 24 | 不动 |
| `backend_worker.py` | 已搬出：`simulator/worker_protocol.py` | 28 | 进程栈内部协议，连同 `WorkerEvent`、`TaskSubmission` 离开契约层（裁决 R23） |
| `capabilities.py` | `ReturnHome`、`WeatherControl`、`StreamingViews`、`AppearanceModelControl`、`FramePublisher` | 70 | 不动 |
| `goal_observer.py`、`target_detector.py`、`task_parser.py`、`task_completion.py`、`trajectory_planner.py`、`task_trace.py` | 各一个 `Protocol` | 12 / 12 / 10 / 7 / 37 / 12 | 不动 |
| `__init__.py` | 包入口：全部 `Protocol` 与 `SegmentSink`/`StreamSink` 两个别名、`StreamGoalSource` | 25 | 现 6 行，按裁决 R20 扩成完整导出 |

两个周期常量只写在 `runtime/motion/loop.py`：`SEGMENT_PERIOD_S = 0.030` 与 `1 / MotionLimits.control_hz`，不进配置（裁决 R9）。`duration_s` 的类型仍是秒的 `float`，周期由执行层决定，不由适配层。

### 1.3 与现有 `Capabilities` 的隔离

三处含 "capabilit" 的符号，各自边界固定：

| 符号 | 位置 | 含义 | 字段 |
| --- | --- | --- | --- |
| `Capabilities` | `domain/engine_family.py:37` | 引擎能给控制台提供哪些控件 | `vehicles`、`view_modes`、`actions`、`task_kinds`、`weather_fields`、`weather_presets` |
| `MotionCapability` | `domain/motion.py` | 一台载具的运动学 | `model`、`max_lateral_speed_mps`、`min_turn_radius_m`、`allows_reverse` |
| `ports/capabilities.py` | `ports/` 模块 | 五个可选端口的模块名 | 无数据 |

- `Capabilities` 内容一字不改，不获得任何运动学字段。
- `MotionCapability` 不含 `actions`、`task_kinds`、`vehicles`，只回答"这台载具能怎么动"。
- 两者都从 `planner/domain` 入口导出，靠名字与字段区分（裁决 R20）：`Capabilities` 说引擎控件，`MotionCapability` 说载具运动学，同名不混用。
- 新增测试守住：`MotionCapability` 只出现在 `domain/motion.py`，`Capabilities` 只出现在 `domain/engine_family.py`。

### 1.4 两份 `MotionLimits` 合并：保留 15 个字段

四个纬向参数收成嵌套对象 `HeadingHoldSettings`，它们只被 `services/position_hold.py` 读，平铺在物理包络里只是历史（裁决 R22）。

现 `domain/motion.py:38` 与 `services/segment_motion.py:21` 各一份。合并后落在 `domain/motion.py`，字段来源与默认值：

| 字段 | 来源 | 默认值 |
| --- | --- | --- |
| `max_speed_mps` | 配置侧独有（`segment_motion.py:22`） | 3.0 |
| `max_acceleration_mps2` | 配置侧 `max_accel_mps2` 改名为 domain 名 | 1.5 |
| `max_yaw_rate_deg_s` | 配置侧 `max_yaw_rate_dps` 改名 | 30.0 |
| `max_yaw_acceleration_deg_s2` | 配置侧 `max_yaw_accel_dps2` 改名 | 60.0 |
| `control_hz` | 两侧同名 | 20.0 |
| `min_duration_s` | 两侧同名 | 1.0 |
| `passed_lateral_tolerance_m` | 两侧同名，改成配置覆盖项 | `None` 表示不覆盖；生效值取模型默认与配置值的较大者，配置只能抬高不能压低（裁决 R17） |
| `heading: HeadingHoldSettings` | 两侧同名，收成嵌套对象 | `HeadingHoldSettings()`；内含 `heading_hold_gain=0.4`、`heading_rate_damping=0.0`、`heading_deadband_deg=0.5`、`heading_rate_deadband_dps=1.5`（裁决 R22） |
| `position_gain` | domain 独有 | 1.0 |
| `yaw_gain` | domain 独有 | 2.0 |
| `position_tolerance_m` | domain 独有 | 0.3 |
| `yaw_tolerance_deg` | domain 独有 | 2.0 |
| `stopped_speed_mps` | domain 独有 | 0.15 |
| `stopped_yaw_rate_deg_s` | domain 独有 | 2.0 |
| `settle_time_s` | domain 独有 | 0.3 |

删除：`to_tracking_limits()`（合并后无转换对象）、配置侧的 `OPTIONAL`/`POSITIVE` 两套校验。`__post_init__` 只保留 domain 版规则（有限、非负、非零项为正、`1 <= control_hz <= 50`），`OPTIONAL` 变成四个可零项：`heading_rate_damping`、`heading_deadband_deg`、`heading_rate_deadband_dps`，外加允许 `None` 的 `passed_lateral_tolerance_m`（裁决 R6、R17：`None` 表示不覆盖）。

`max_yaw_acceleration_deg_s2` 的默认值从 domain 的 30.0 改成配置侧的 60.0。生产路径一直由 `planner/config.py:1504` 与两份 `config/core/*/base.yaml` 传 60.0，所以这个改动只影响测试里的裸构造：`tests/test_motion.py:27`、`tests/test_airsim_server_vehicle.py:111`、`tests/test_batch3_services.py:72`、`tests/test_basic_motion.py:154`、`tests/test_projectairsim_motion.py:13`、`tests/test_position_hold.py:12`、`tests/test_workflow.py:201`、`tests/test_free_navigation.py:764`。合并那一步要重跑这些文件，若某条时长断言依赖 30.0，改断言而不是改默认值。

`config/core/<profile>/base.yaml` 的键名不变（`max_accel_mps2`、`max_yaw_rate_dps`、`max_yaw_accel_dps2`），改名发生在 `planner/config.py:1500-1512` 的关键字参数上。

## 二 数据流与时序

### 2.1 段执行（批次原语）

`docs/refactor/00-boundaries.md` 第二节定义它用于数值动作与"路径已确定、期间不重规划"的目标接近。

1. 任务层构造 `MotionSegment`（两个 `WorldPose` 加 `profile`），交 `runtime/execution.py` 的 `ExecutionService.execute_with_receipt`。
2. `checkpoint()` 只校验停止请求与时间预算，不读碰撞（裁决 R5）。
3. `Vehicle.execute_segment(...)` 落到 runtime 的 `GuidedVehicle(streaming, loop)`，由 `runtime/motion/loop.py` 的 `ControlLoop.run_segment` 采样五次多项式时标，每 `SEGMENT_PERIOD_S = 0.030` 一拍：`require_supported(command, capability)` → `streaming.stream_command(command, SEGMENT_PERIOD_S)` → 引擎推进 30 ms。引擎里没有采样循环，也没有 tracker（裁决 R1、R9）。
4. 每拍读 `streaming.current_pose()`，回调 `pose_checkpoint(pose, unwrapped_yaw)`；`ControlLoop._guard` 顺带调 `CollisionRecovery.poll`，接触即按第五节的退避流程处理（裁决 R5）。
5. 段结束返回 `None`。执行层读 `current_pose()`，写 `ExecutionReceipt`（`completed_segments`、`measured_chord_length_m`、`measured_actions`）。

节拍只在适配层内，`MotionSegment` 不带采样周期。

### 2.2 速度流（拍原语）

用于"路径由规划器持续给出"的目标接近与无终点巡航。

1. `runtime/motion/loop.py` 的 `ControlLoop.run_stream(goals, sink, deadline_monotonic)` 每 `1 / MotionLimits.control_hz` 一拍（默认 50 ms，裁决 R9）。
2. `goals` 是 `runtime/motion/follower.py` 的 `QueueFollower`，它实现 `StreamGoalSource.next(measured_state) -> MotionCommand | None`，负责队列消耗与 waypoint 超时。`PlanningWindow` 只提供队列视图、`consume` 与六个公开方法，不实现 `StreamGoalSource`，也不承担伺服（裁决 R4）。
3. `require_supported(command, capability)` 通过后调 `sink(command, duration_s)`，`sink` 就是 `stream_command` 签名，一次下发并请求引擎推进一拍（裁决 R2）。
4. 每拍 `streaming.stream_state()` 读实测，写 `runtime/stream_timeline.py`。
5. `PlanningWindow.poll_stream(measured, checkpoint)` 按 `SlidingWindowSettings.planning_interval_s`（5.0 s）提交续规划；返回 `True` 表示有新航点进队列，下一拍 `QueueFollower.next` 才能看到。
6. `deadline_monotonic` 到期抛 `BudgetExceeded`，不静默正常返回；任务层捕获后映射 `TIMEOUT`（裁决 R3）。

20 Hz 取自 `MotionLimits.control_hz`（默认 20.0，允许 1 到 50）。30 ms 与 `1/control_hz` 都是执行层决定的值，适配层只接受 `duration_s`（裁决 R9）。

### 2.3 帧级

观测只在停稳后取，锚点由位置保持给出（`docs/ARCHITECTURE.md` 第 11 节）。速度流路径取帧走 `MovingObservationSource.capture_moving()`，帧与 `TimedVehicleState` 同拍配对。契约层为此只保证 `TimedVehicleState` 同时带 `timestamp_ns` 与 `observed_monotonic`（`domain/streaming.py:12-15`），不引入新的观测类型。

### 2.4 `CollisionContact.at_monotonic_s` 的取值

三个适配层的引擎事件带的是仿真时间戳（`simulators/projectairsim/collision.py:27` 的 `timestamp_ns`），与 `time.monotonic()` 之间没有已标定的映射。字段名叫 monotonic 就必须是单调钟：适配层在读到事件那一刻取 `time.monotonic()` 填进去。仿真时间戳继续留在 `last_collision` 属性里给诊断用。

## 三 跨层接口

### 3.1 我提供什么

- 全部数据类型与 `Protocol`：`MotionModel`、`MotionCapability`、`HolonomicCommand`、`CarCommand`、`MotionCommand`、`CollisionContact`、`CollisionState`、`CollisionRecoverySettings`、`MotionLimits`、`MotionSegment`、`ExecutionBudget`、`PoseCheckpoint`、`Capabilities`，以及 `StreamGoalSource`（裁决 R16）。
- 三个校验入口：各 frozen 类型的 `__post_init__`；`command_type_for(capability)`；`require_supported(command, capability)`。
- 零逻辑承诺：`domain/` 与 `ports/` 不 import `algorithms/`、`runtime/`、`workflows/`、`assembly/`、`adapters/`、`simulator/`、`simulators/`。
- 不提供第二个下发名字：`SegmentSink`、`StreamSink` 都是 `stream_command(command, duration_s)` 的别名，`apply` 这个名字不出现（裁决 R2）。两个 Sink 名字不单独建文件，`ports/` 下新增的只有 `goal_source.py`（裁决 R16）。

### 3.2 我要求相邻层提供什么

| 相邻层 | 要求 |
| --- | --- |
| `simulators/projectairsim/{rover,drone}.py`、`simulators/airsim/vehicle.py` | 实现 `stream_command(command, duration_s)` 与 `current_pose`、`motion_capability`、`collision_state`、`cancel`、`hover`。不实现 `execute_segment`，不保留 tracker，不做段采样（裁决 R1）。过渡期允许三行委派，最终删除。`stream_command` 第一句 `require_supported`。`collision_state()` 改返回 `CollisionState`，`last_collision` 作为过渡别名保留，供 `scripts/smoke_projectairsim_{adapter,rover}.py` 与现有测试继续读 |
| `runtime/execution.py` | `checkpoint()` 只管停止与预算，不读碰撞、不锁存接触；删掉 `collision_state()`、`check_safety()` 与 `_collision_check`，`acquire_flight()` 去掉 `collision_check` 参数（裁决 R5） |
| `runtime/motion/loop.py` | `ControlLoop` 构造参数只收 `MotionCapability`，由它选律（全向律或纯跟踪律），不收独立的"模式"参数。`GuidedVehicle(streaming, loop)` 兑现 `Vehicle` 槽，`execute_segment` 由 `run_segment` 驱动逐拍调 `stream_command`（裁决 R1）。碰撞检查挂在 `ControlLoop._guard` 上，由 `CollisionRecovery.poll` 触发（裁决 R5）。`SEGMENT_PERIOD_S = 0.030` 与 `1/control_hz` 是这里的模块常量（裁决 R9） |
| `algorithms/motion/corridor.py` | 越点走廊常量按 `MotionModel` 取值：`ACKERMANN_PASSED_LATERAL_TOLERANCE_M = 1.0`，`HOLONOMIC` 取 `MotionLimits.position_tolerance_m`；配置 `motion.passed_lateral_tolerance_m` 只能抬高不能压低，不从 `min_turn_radius_m` 推导（裁决 R6、R17）。引擎里删掉 `ROVER_PASSED_LATERAL_TOLERANCE_M`（`simulators/projectairsim/rover.py:48`）与 `rover_tracking_limits()`（同文件 51 行） |
| `algorithms/motion/quintic.py` | 接收 `services/segment_motion.py` 的 `SegmentSetpoint`、`quintic`、`duration_for`、`plan_segment` 与 `_QUINTIC_PEAK_*` 常量，签名不变，入参类型换成合并后的 `MotionLimits` |
| `assembly/` | `is_drone = vehicle_id == "drone"`（`application/mission_assembly.py:214`）改成 `motion.model is MotionModel.HOLONOMIC`；用 `GuidedVehicle(streaming, loop)` 填 `vehicle` 槽并保留底层 `streaming` 对象；删掉 `collision_check=vehicle.collision_state`（同文件 252 行）接线；`tracking_limits=` 四处（222、251、266、296 行）直接传 `config.motion` |
| `planner/config.py` | `MotionLimits` 改从 `.domain.motion` 导入（现 `config.py:20`），关键字改三个新名；`motion.collision` 子段构造 `CollisionRecoverySettings`，新增 `motion.collision.enabled`（默认 `true`），删掉 `visual_goal_navigation.collision_check_enabled`（`config.py:345`），碰撞开关只剩这一个（裁决 R8） |
| `adapters/task_parser/` | 地面与空中的分支依据改成 `MotionCapability.model`，不再读 `aerial: bool` 之外的字符串判据；横向与竖直指令的拒绝发生在解码处，不进契约层 |

### 3.3 每个类型由谁写入、谁读取

| 类型 | 写入者 | 读取者 |
| --- | --- | --- |
| `MotionModel`、`MotionCapability` | 三个载具适配层构造期各算一次；`motion_capability()` 返回同一个对象 | `assembly/`（选解码器、构造 `ControlLoop`）、`algorithms/`（运动律分支、越点走廊默认值）、`runtime/motion/`（限幅与偏航律）、`adapters/task_parser/`（拒绝载具做不到的指令） |
| `HolonomicCommand` | `runtime/motion/loop.py`（限幅后）、适配层的段采样 | `StreamingVehicle.stream_command` 实现、`adapters/diagnostics/`、`runtime/window/` 遥测 |
| `CarCommand` | 同上，阿克曼律产出 | 同上 |
| `MotionCommand` | 上述两个联合 | `require_supported`、适配层 `_apply` |
| `CollisionState`、`CollisionContact` | `Vehicle.collision_state()` 三个适配层 | `runtime/collision_recovery.py` 的 `poll`，由 `runtime/motion/loop.py` 的 `ControlLoop._guard` 调用；`runtime/execution.py` 不读（裁决 R5）；诊断记录 |
| `CollisionRecoverySettings` | `assembly/`（读 `config` 的 `motion.collision`，默认 `backoff_m=3.0`、`maximum_recoveries=3`、`reset_after_progress_m=20.0`，现值在 `config/core/default/base.yaml:138-143`）；`motion.collision.enabled=false` 时不构造（裁决 R8） | `runtime/collision_recovery.py`；任务层不读 |
| `MotionLimits` | `planner/config.py:1500` 一处，测试若干 | `algorithms/`（段规划时长）、`runtime/motion/`（跟踪与保持）、`workflows/basic_motion/workflow.py` 等把 limits 传给规划器 |
| `ExecutionBudget`、`PoseCheckpoint` | `runtime/execution.py` | `Vehicle.execute_segment` 实现 |
| `MotionSegment`、`MotionTrajectory` | `algorithms/` 段规划、`workflows/basic_motion/workflow.py` | `Vehicle.execute_segment` 实现 |
| `Capabilities` | `simulator/ports.py:107` 的推导 | `adapters/http/server_api.py`、`console/`。`task_kinds` 纯粹由 `REQUIRED_PORTS ⊆ 已填槽位` 推出；`SimulatorPorts.enabled_tasks` 字段与 `simulators/projectairsim/runtime.py` 的 `ENABLED_TASKS` 删除，可跑任务没有第二个开关（裁决 R18，R11 作废） |

### 3.4 两种原语怎么共存

- 引擎只有一个数据面动作 `stream_command(command, duration_s)`（裁决 R1）。段采样、限幅、伺服全在 `runtime/motion/loop.py`，引擎里既没有 tracker 也没有 `execute_segment` 的采样循环。过渡期允许引擎保留三行委派，最终删除。
- 两种原语共用同一签名：`SegmentSink`、`StreamSink` 都是 `stream_command(command, duration_s)` 的别名，`apply` 这个第二名字不出现（裁决 R2）。`ControlLoop.run_segment` 与 `run_stream` 都逐拍调 `sink`，差别只在谁生成命令。
- 能力拒绝仍只有一个地方：引擎 `stream_command` 的第一句 `require_supported`。段执行与速度流都在到达引擎前经过它，所以"适配层静默丢掉横向分量"（现 `simulators/projectairsim/drive.py:156-161`）没有存活空间。
- 互斥不靠新增标志位。`ExecutionService` 已有 owner 线程与 flight owner 两层校验（`services/execution.py:46-77`），`_executing` 拦住嵌套段执行（同文件 133-136）。`GuidedVehicle` 是这两层之下唯一的 `Vehicle` 实现者，两条路径不会同时在同一载具上活动。
- 一个任务只选一种原语。选哪一种由任务层按 `docs/refactor/00-boundaries.md` 第二节的判据决定，执行层两条都提供且不判断业务。

### 3.5 输入面与输出面（裁决 R19）

两条规则，都能写成测试。

- 入口唯一：`planner/<层>/__init__.py` 是该层唯一公开入口，只导出该层公开名字。跨层 import 只能指向包入口，`from planner.runtime.motion.loop import ControlLoop` 这类深层路径禁；同层内部随便 import。
- 输入只有两类：契约层的类型（`planner.domain`、`planner.ports` 导出的名字），或本层自己声明的 `Protocol`。禁止把另一层的具体类当构造参数或返回值。

输入：

| 名字 | 形状 | 由谁提供 | 是否可选 |
| --- | --- | --- | --- |
| `WorldPose`、`MotionSegment`、`MotionTrajectory` | frozen dataclass | `planner.domain` | 必需 |
| `MotionLimits`、`ContinuousTrackingSettings` | frozen dataclass | `planner.domain` | 必需（可取默认值） |
| `MotionModel`、`MotionCapability` | enum 与 frozen dataclass | `planner.domain` | 必需 |
| `HolonomicCommand`、`CarCommand`、`MotionCommand` | frozen dataclass 与联合 | `planner.domain` | 必需 |
| `CollisionState`、`CollisionContact` | frozen dataclass | `planner.domain` | 必需（无接触时 `last_contact` 为 `None`） |
| `CollisionRecoverySettings` | frozen dataclass | `planner.domain` | 可选，`motion.collision.enabled=false` 时不传 |
| `ExecutionBudget`、`PoseCheckpoint` | frozen dataclass 与 `Callable` 别名 | `planner.domain` | 可选 |
| `TimedVehicleState` | frozen dataclass | `planner.domain` | 必需 |
| `Capabilities` | frozen dataclass | `planner.domain` | 必需（只给入口层读） |

输出：

| 名字 | 形状 | 由谁消费 | 稳定性 |
| --- | --- | --- | --- |
| `MotionModel`、`MotionCapability` | 见上 | `assembly`、`algorithms`、`runtime`、`adapters/task_parser` | 稳定 |
| `MotionCommand`（含两个成员） | 见上 | 引擎适配层 `stream_command`、`adapters/diagnostics` | 稳定 |
| `MotionLimits` | 见上 | `algorithms`、`runtime` | 稳定 |
| `HeadingHoldSettings` | frozen dataclass | `runtime`（位置保持） | 稳定（裁决 R22） |
| `CollisionState`、`CollisionContact` | 见上 | `runtime` | 稳定 |
| `CollisionRecoverySettings` | 见上 | `runtime` | 稳定 |
| `Vehicle`、`StreamingVehicle` | `Protocol` | 引擎适配层实现，`runtime` 消费 | 稳定 |
| `StreamGoalSource` | `Protocol` | 任务层实现，`runtime/motion/loop.py` 消费（裁决 R16） | 稳定 |
| `PositionHoldVehicle` | `Protocol` | 引擎适配层实现，`runtime` 消费（裁决 R7） | 内部，阶段 5 再定 |
| `require_supported` | 纯函数 | 引擎适配层、`assembly` 构造期、`runtime` | 稳定 |

不暴露什么：`GuidedVehicle`、`ControlLoop`、`QueueFollower`、`PlanningWindow`、`WorldWaypointQueue` 属执行层内部，不进任何跨层表；`SegmentSink`、`StreamSink` 只是 `stream_command` 的别名，不单独建文件也不导出（裁决 R2）；`SimulatorPorts`、`TASK_REQUIREMENTS`、`_Slot` 等占位类属装配与引擎侧，不进契约；任何引擎实现类（`ProjectAirSimRover`、`AirSimVehicle`）不跨层可见。

冲突已由 R20 解决：`planner/domain/__init__.py` 导出全部公开名字（含 `Capabilities`），`planner/ports/__init__.py` 导出全部 `Protocol` 与 `SegmentSink`/`StreamSink` 两个别名加 `StreamGoalSource`，`planner/errors.py` 保持单文件，算契约层的第三个入口。1.1 与 1.3 已按此口径改写。

## 四 错误与失败路径

| 触发 | 谁抛 | 异常类型 | 谁捕获 | 结果 |
| --- | --- | --- | --- | --- |
| 命令类型与 `MotionModel` 不符 | 引擎适配层 `stream_command` 第一句调 `require_supported`（裁决 R1） | `ValueError` | `ControlLoop.run_segment` / `run_stream` | 包成 `ExecutionError`，任务层转 `StageResult(False)`，不进重试 |
| 全向命令带横向分量而能力为阿克曼 | 同上 | `ValueError` | 同上 | 同上；这条是第一拍就炸，不是跑到航点超时 |
| `CarCommand.reverse` 与速度符号矛盾 | `CarCommand.__post_init__` | `ValueError` | 无人捕获 | 构造期失败，不进运行期 |
| `MotionCapability` 字段与 `model` 自相矛盾（阿克曼有横向速度、全向有转弯半径） | `MotionCapability.__post_init__` | `ValueError` | 无人捕获 | 适配层构造期失败，场景起不来 |
| `MotionLimits`、`CollisionRecoverySettings` 字段非法 | 各自 `__post_init__` | `ValueError` | `planner/config.py` 的加载路径不捕获 | 服务启动失败，带字段名 |
| 时间预算耗尽 | `ControlLoop.run_segment` 或 `run_stream` 抛 `BudgetExceeded`（`ExecutionError` 子类，`errors.py:28`）；`run_stream` 的 `deadline_monotonic` 到期必须抛，不静默返回（裁决 R3） | `BudgetExceeded` | 任务层捕获后映射 `TIMEOUT`；`ExecutionService` 记回执 | 剩余运动写进 `ExecutionReceipt.remaining` |
| 用户停止 | `ExecutionService.check_deadline` | `ExecutionError("flight stop requested")`（`services/execution.py:96`） | 任务层 | `StageResult(False)` |
| 碰撞接触 | 端口不抛，返回 `CollisionState(in_contact=True)` | — | `runtime/motion/loop.py` 的 `ControlLoop._guard` 调 `CollisionRecovery.poll`（裁决 R5） | 后退、清队列、继续；恢复次数超 `maximum_recoveries` 才抛 `ExecutionError` |
| 端口读失败（客户端库异常） | 适配层 | `SimulatorConnectionError` 或库异常 | 任务层 | 阶段失败，原异常进 `StageResult.error` |

新增异常类：零个。`require_supported` 抛 `ValueError`，不加 `UnsupportedMotionCommand`，因为段执行路径的兜底已经把任何 `Exception` 包成带回执的 `ExecutionError`，多一个类只是多一处映射。

## 五 并发与所有权

- 控制权不新增机制。`ExecutionService` 持 owner 线程标识与 flight owner（`services/execution.py:31-33`、`acquire_flight`/`release_flight`），契约层的类型不参与。
- 契约层全是 frozen dataclass，切换载具只是换对象引用，不需要加锁、不需要重建类型。
- `collision_state()` 是消费式读。`ProjectAirSimRover.collision_state`（`rover.py:219-226`）在读到事件后清空监视器，`AirSimVehicle.collision_state`（`airsim/vehicle.py:192`）读的是电平。锁存点放在 `runtime/collision_recovery.py`，由 `ControlLoop._guard` 每拍调一次 `poll`，同一拍里不得重复读端口（裁决 R5）。ProjectAirSim 侧重复读会丢事件，AirSim 侧会重复触发退避。
- `motion_capability()` 必须无 IO、无阻塞，装配层在构造期调用一次并把结果放进 `VehicleArm`，之后不再问。
- 位置保持与观测采集是临时持有者：`PositionHoldVehicle.hold_velocity` 保留，实现内部转调 `stream_command`（裁决 R7）；它不可用时执行层退回 `Vehicle.hover()`。
- 切载具的顺序固定为：`ExecutionService.stop()` → `release_flight(owner)` → 换 `execution.vehicle` 引用与新的 arm → 新载具已停稳。契约层不参与交接，也不持有载具引用。

## 六 测试点

新增 `tests/test_motion_contract.py`：

| 断言对象 | 内容 |
| --- | --- |
| `MotionCapability.__post_init__` | 负例：`ACKERMANN` 带 `max_lateral_speed_mps=0.6` 抛 `ValueError`；`HOLONOMIC` 带 `min_turn_radius_m=3.2` 抛；`HOLONOMIC` 带 `max_lateral_speed_mps=0.0` 抛；`ACKERMANN` 带 `min_turn_radius_m=None` 抛 |
| `CarCommand.__post_init__` | 负例：`CarCommand(1.0, 0.0, reverse=True)` 抛；`CarCommand(0.0, 3.0, reverse=True)` 抛 |
| `command_type_for` | `HOLONOMIC` → `HolonomicCommand`；`ACKERMANN` → `CarCommand` |
| `require_supported` | 正例：`CarCommand(1.0, 20.0, False)` 配阿克曼通过、`HolonomicCommand(1.0, 0.3, 0.0, 0.0)` 配全向通过；负例：`HolonomicCommand(0.0, 0.3, 0.0, 0.0)` 配阿克曼抛、`HolonomicCommand(1.0, 0.0, 0.0, 0.0)` 配阿克曼因类型不符抛、`CarCommand(-1.0, 0.0, True)` 配 `allows_reverse=False` 抛 |
| 冻结 | 对五个新类型分别赋值抛 `FrozenInstanceError` |
| `CollisionRecoverySettings` | 负例：`backoff_m=0.0`、`maximum_recoveries=0`、`reset_after_progress_m` 小于等于 `backoff_m` 各抛 |

新增 `tests/test_vehicle_port.py`：

| 断言对象 | 内容 |
| --- | --- |
| 三个适配层的 `motion_capability()` | `ProjectAirSimRover` 返回 `ACKERMANN`、`max_lateral_speed_mps == 0.0`、`min_turn_radius_m == pytest.approx(3.20, abs=0.05)`；`ProjectAirSimDrone` 与 `AirSimVehicle` 返回 `HOLONOMIC`、`min_turn_radius_m is None`、`max_lateral_speed_mps > 0` |
| 适配层拒收 | 负例：`rover.stream_command(HolonomicCommand(0.0, 0.3, 0.0, 0.0), 0.05)` 抛 `ValueError`，且引擎调用计数保持 0（拒绝发生在下发之前） |
| `collision_state` 含义 | 有事件时 `in_contact is True` 且 `last_contact.counterpart == "Building_3"`、`at_monotonic_s` 单调递增；无事件时 `in_contact is False` 且 `last_contact is None`；AirSim 电平式：`has_collided=True` 得 `in_contact is True`、`last_contact is None` 合法 |
| `stream_velocity` 已消失 | `assert not hasattr(StreamingVehicle, "stream_velocity")`；反向断言 `stream_command` 在三个适配层上存在 |
| `GuidedVehicle` 满足 `Vehicle` | `runtime/motion/loop.py` 的 `GuidedVehicle(streaming, loop)` 具备 `Vehicle` 的六个公开方法，`execute_segment` 逐拍调 `stream_command`（裁决 R1） |
| 走廊取值（裁决 R17） | `algorithms/motion/corridor.py` 的测试断言 `ACKERMANN_PASSED_LATERAL_TOLERANCE_M == 1.0`、`HOLONOMIC` 得 `MotionLimits.position_tolerance_m`、配置只能抬高（传 0.4 仍得 1.0，传 1.2 得 1.2）；本层只断言 `passed_lateral_tolerance_m is None` 表示不覆盖 |
| 内参两份（裁决 R14） | `CameraIntrinsics` 的两份实现都保留，加断言两份主点差不超过 0.5 px |

`tests/test_architecture.py` 里比对协议成员与适配层的那段循环从 `AirSimVehicle` 扩到 `ProjectAirSimRover` 与 `ProjectAirSimDrone`：`Vehicle` 的六个公开方法与 `StreamingVehicle` 的两个公开方法在每个适配层上都能找到。这条守的是 `motion_capability` 与 `stream_command` 不会只在协议里存在。

`tests/test_layering.py` 增两条：`domain/` 与 `ports/` 的 import 集合里不出现其他层包名；`MotionCapability` 只出现在 `domain/motion.py`、`Capabilities` 只出现在 `domain/engine_family.py`。

负例清单（必须有）：可零字段取负值、`CarCommand` 符号矛盾、全向命令投给阿克曼、协议方法在适配层缺失、`stream_velocity` 复活。

## 七 迁移步骤

每步独立可验证，出问题退回上一步。前四步只增不改行为。

1. 改名。`domain/runtime.py` 移到 `domain/engine_family.py`，改 5 个引用：`planner/simulator/ports.py:26`、`planner/config.py:15`、`tests/test_runtime_catalog.py:15`、`tests/test_xpra_start_driver.py:23`、`tests/test_argv.py:6`。验收：`pytest tests/test_runtime_catalog.py tests/test_xpra_start_driver.py tests/test_argv.py tests/test_architecture.py -q`。
2. 合并 `MotionLimits`。`domain/motion.py` 加 6 个配置侧字段、改 3 个字段名、默认 `max_yaw_acceleration_deg_s2=60.0`；`planner/config.py:20` 与 `1500-1512` 切换；`services/segment_motion.py` 删掉类与 `to_tracking_limits`，只留四个纯函数；`simulators/projectairsim/motion.py:222` 的 `tracking_limits()` 改成恒等函数（保留调用点）。验收：`pytest tests/test_motion.py tests/test_passed_corridor.py tests/test_projectairsim_motion.py tests/test_position_hold.py -q`，断言时长的地方按新默认值校准。
3. 加碰撞类型。`domain/execution.py` 加 `CollisionContact`、`CollisionState`；三个适配层 `collision_state()` 改返回类型，`last_collision` 保留为别名；删 `ports/collision_check.py`，`acquire_flight()` 去掉 `collision_check` 参数，`ExecutionService` 的碰撞读取与锁存改由 `runtime/collision_recovery.py` 持有（裁决 R5）；`application/mission_assembly.py:252` 的接线删除。验收：`pytest tests/test_visual_goal_collision_recovery.py tests/test_projectairsim_rover.py tests/test_projectairsim_drone.py tests/test_free_navigation.py -q`。
4. 加载具运动类型。`domain/motion.py` 加 `MotionModel`、`MotionCapability`、两个命令类、`MotionCommand`、两个校验函数；三个适配层实现 `motion_capability()`；`application/mission_assembly.py:214` 的字符串判据换成 `motion.model`；走廊常量进 `algorithms/motion/corridor.py`，引擎里删 `ROVER_PASSED_LATERAL_TOLERANCE_M` 与 `rover_tracking_limits()`（裁决 R6）。验收：`pytest tests/test_motion_contract.py tests/test_capability_gating.py tests/test_passed_corridor.py tests/test_task_parser.py -q`。
5. 换下发原语并上提控制循环。三个适配层把 `stream_velocity` 改为 `stream_command` 并接 `require_supported`；`runtime/motion/loop.py` 新增 `ControlLoop` 与 `GuidedVehicle`，引擎删 tracker、删 `execute_segment` 的采样循环（过渡期留三行委派，裁决 R1）；`services/continuous_execution.py:125`、`workflows/visual_goal_navigation/continuous.py:70`、`workflows/free_navigation/continuous.py:126`、11 处测试同步改；协议里的 `stream_velocity` 删除（裁决 R2）。验收：`pytest tests/test_free_navigation.py tests/test_projectairsim_rover.py tests/test_projectairsim_drone.py tests/test_vehicle_port.py -q`，并 `grep -rn stream_velocity planner tests` 为空。
6. 加 `CollisionRecoverySettings`。`domain/motion.py` 加三字段类型；`config/core/<profile>/base.yaml` 的 `free_navigation` 段三个键迁到 `motion.collision`，并新增 `motion.collision.enabled`（默认 `true`）；删 `visual_goal_navigation.collision_check_enabled`（裁决 R8）；`planner/config.py` 的 `FreeNavigationSettings`（373-393 行）减三个字段、新增 `CollisionSettings`。验收：`pytest tests/test_config.py tests/test_reference_config.py tests/test_profiles.py -q`。
7. 删文件。`domain/lane_reference.py` 与其两处接线删除；`domain/planning_window.py` 的 `PlanningWindowInput` 并入 `domain/sliding_window.py` 后删文件，改 4 个引用（`services/planning_window.py:18`、`workflows/visual_goal_navigation/session.py:8`、`workflows/free_navigation/continuous.py:17`、`tests/test_batch3_services.py:12`）。验收：`pytest tests/ -q` 全绿，用例数不减少。
8. 跨层 import 改走包入口（裁决 R20）。`from planner.domain.pose import WorldPose` → `from planner.domain import WorldPose`，`planner.ports` 同理。现状估算：指向 `planner.domain.<模块>` 的深层 import 约 444 条（绝对路径 368，相对两三级点 76），指向 `planner.ports.<模块>` 约 25 条，合计约 470 条、涉 162 个文件。按包分批：`runtime`（现 `services/`）116 处、`adapters` 52 处，其余为 `workflows`、`tests`、`scripts` 各一批，每批 `pytest tests/ -q`。验收：`grep -rn "from planner\.domain\.\|from \.\.domain\.\|from \.\.\.domain\.\|from planner\.ports\.\|from \.\.ports\." planner tests scripts` 只剩 `planner/domain/` 与 `planner/ports/` 包内部及两个包入口。

## 八 削面复核（裁决 R21）

口径：引用数 = `grep -rn "\bName\b" planner --include=*.py` 去掉 `planner/domain/__init__.py` 的转发行后的计数。测试、`scripts/`、包入口转发不计；同层内部引用不计入"层外调用点"。

### A 对外名字逐条

| 名字 | 现引用数 | 层外运行期调用点 | 处置 | 依据 |
| --- | --- | --- | --- | --- |
| `WorldPose` | 165 | `services/trajectory-validation.py`、所有载具适配层 | 保留 | 调用（全栈输入） |
| `RelativePoseDelta` | 29 | `services/trajectory_transform.py:5` | 保留 | 调用 |
| `Observation` | 68 | `services/free_space.py:21` | 保留 | 调用与构造期校验 |
| `CameraIntrinsics` | 15 | `capabilities/targeting/depth_surface_sampling.py:21` | 保留 | 调用；两份实现按 R14 不合并 |
| `CameraMountPose` | 1 | 无 | 删 | 全仓仅定义一处，无人使用 |
| `relative_to_world` | 21 | `services/segment_motion.py:11` | 保留 | 调用 |
| `wrap_deg` | 8 | `services/segment_motion.py:151` | 保留 | 调用 |
| `SegmentResult` | 1 | 无 | 删 | 仅定义加包入口导出，无生产调用方 |
| `MotionModel` | 0 | 目标：`simulators/projectairsim/rover.py` 的 `motion_capability()`（R1 落地后） | 保留 | 对外契约：引擎必须声明的运动学类型 |
| `MotionCapability` | 0 | 同上 | 保留 | 对外契约：引擎必须实现的 `motion_capability()` 返回类型 |
| `HolonomicCommand` | 0 | 目标：`runtime/motion/loop.py` 产出，引擎 `stream_command` 消费 | 保留 | 对外契约：`stream_command` 的命令类型 |
| `CarCommand` | 0 | 同上 | 保留 | 同上 |
| `MotionCommand` | 0 | 同上 | 保留 | 同上 |
| `command_type_for` | 0 | 无 | 内部化 | 仅 `require_supported` 内部需要，外部直接调 `require_supported` |
| `require_supported` | 0 | 目标：引擎 `stream_command` 第一句（R1） | 保留 | 构造期不变量校验 |
| `CollisionContact` | 0 | 目标：三个载具适配层的 `collision_state()` | 保留 | 对外契约：`CollisionState` 字段类型，适配层构造 |
| `CollisionState` | 0 | 目标：`runtime/collision_recovery.py` 的 `poll`（R5） | 保留 | 对外契约：`Vehicle.collision_state()` 返回类型 |
| `CollisionRecoverySettings` | 0 | 目标：`runtime/collision_recovery.py` 构造参数 | 保留 | 配置字段 |
| `Capabilities` | 4 | `simulator/ports.py:93` → `/api/status` | 保留 | HTTP 字段 |
| `MotionLimits` | 42 | `services/motion/synchronized.py:5` 等 | 保留 | 配置字段；字段审计见 A.2 |
| `HeadingHoldSettings` | 0 | 目标：`services/position_hold.py`（现四处纬向字段的唯一读者） | 保留 | 配置字段；从 `MotionLimits` 收成嵌套对象（裁决 R22） |
| `ContinuousTrackingSettings` | 32 | `services/motion/continuous.py:32` | 保留 | 配置字段 |
| `ExecutionBudget` | 37 | `services/execution.py:127` | 保留 | 调用 |
| `COMMAND_FOR_MODEL` | 0 | 不采纳 | 不新增常量 | 06 提的“按模型取命令类型”用 `require_supported(command, capability)` 覆盖：适配层收到命令后校验，类型不符就抛 `ValueError`，不需要映射表（裁决 R25） |
| `ExecutionReceipt` | 9 | 仅 `runtime` 内部产生与消费 | 内部化 | 只有执行层用，不进契约面（裁决 R22） |
| `StopResult` | 18 | 仅 `services/execution.py:79` 自身与任务层取错误字符串 | 内部化 | 同上（裁决 R22） |
| `PoseCheckpoint` | 9 | `simulators/projectairsim/motion.py:72` | 保留 | 对外契约：`execute_segment` 参数类型 |
| `TimedVehicleState` | 18 | `services/position_hold.py:31` | 保留 | 调用与 `stream_state()` 返回类型 |
| `PlanningWindowInput` | 10 | 仅 `services/planning_window.py:121` | 内部化，随滑窗进 `runtime/window/` | 无第二消费者（裁决 R22） |
| `SlidingWindowSettings` | 12 | `services/planning_window.py:66` | 保留 | 配置字段 |
| `Vehicle` | 17 | `simulator/ports.py:40` 槽位类型 | 保留 | 对外契约：引擎必须实现的 Protocol |
| `StreamingVehicle` | 4 | `simulator/ports.py:39` | 保留 | 同上 |
| `StreamGoalSource` | 0 | 目标：`runtime/motion/follower.py` 的 `QueueFollower` 实现（R4/R16） | 保留 | 对外契经：任务层实现、执行层消费 |
| `SegmentSink`、`StreamSink` | 0 | 目标：`runtime/motion/loop.py` 的 `sink`（R2） | 内部化 | 是 `stream_command` 的签名别名，留在 runtime（裁决 R22） |
| `PositionHoldVehicle` | 2 | `services/position_hold.py:12` | 保留 | 对外契约：引擎必须实现的 Protocol（R7） |
| `ObservationSource` | 7 | `simulator/ports.py:37` | 保留 | 对外契约 |
| `ControlledObservationSource` | 2 | `simulator/ports.py:35` | 保留 | 对外契约 |
| `MovingObservationSource` | 6 | `simulator/ports.py:36` | 保留 | 对外契约 |
| `ReturnHome` | 2 | `simulator/ports.py:30` | 保留 | 对外契约与 `home` 动作门 |
| `WeatherControl` | 3 | `simulator/ports.py:32` | 保留 | 对外契约与天气 HTTP 字段 |
| `StreamingViews` | 2 | `simulator/ports.py:31` | 保留 | 对外契约与 `view_modes` 字段 |
| `AppearanceModelControl` | 2 | `simulator/ports.py:28` | 保留 | 对外契约与 `/api/models` |
| `FramePublisher` | 2 | `simulator/ports.py:29` | 保留 | 对外契约与 `/api/video` 门 |
| `BackendWorker` | 4 | 搬到 `simulator/worker_protocol.py`，由 `simulator/worker.py:38` 与两个引擎的 `session.py` 从 `simulator` 取（裁决 R23） | 搬走 | 进程栈协议，不是载具能力 |
| `GoalObserver` | 2 | `workflows/visual_goal_navigation/workflow.py:54` | 保留 | 对外契约：检测器实现必须满足 |
| `TargetDetector` | 2 | `capabilities/targeting/bbox_goal_observation.py:9` | 保留 | 对外契约 |
| `TaskCompletion` | 0 | 无任何 import 行；唯一引用是 `application/factories.py:19` 导入实现类 | 删（不导出） | 无层外 import，按 R21 规则 4 处理（裁决 R23） |
| `TaskParser` | 4 | `application/application.py:7` | 保留 | 对外契约 |
| `TrajectoryPlanner` | 10 | `services/planning_window.py:32` | 保留 | 对外契约 |
| `TaskTrace` | 3 | `workflows/visual_goal_navigation/workflow.py:57` | 保留 | 对外契约 |

### A.2 `MotionLimits` 15 个字段逐个读者

| 字段 | 引用数 | 层外运行期首处读者 | 处置 |
| --- | --- | --- | --- |
| `max_speed_mps` | 5 | `services/segment_motion.py`（段时长）、`application/factories.py` | 保留 |
| `max_acceleration_mps2` | 11 | `services/motion/continuous.py` | 保留 |
| `max_yaw_rate_deg_s` | 7 | `services/motion/continuous.py` | 保留 |
| `max_yaw_acceleration_deg_s2` | 8 | `services/motion/continuous.py` | 保留 |
| `control_hz` | 15 | `services/position_hold.py` | 保留（亦定流量节拍） |
| `min_duration_s` | 4 | `services/motion/synchronized.py` | 保留 |
| `passed_lateral_tolerance_m` | 5 | `services/trajectory_queue.py`（越点判据） | 保留（R6/R17 覆盖项） |
| `heading: HeadingHoldSettings` | 0（现为四个平铺字段，各 2） | `services/position_hold.py` | 收成嵌套对象；四子字段仍是不同轴，不再合（裁决 R22） |
| `position_gain` | 4 | `services/motion/continuous.py` | 保留 |
| `yaw_gain` | 3 | `services/motion/continuous.py` | 保留 |
| `position_tolerance_m` | 21 | `services/motion/continuous.py` | 保留 |
| `yaw_tolerance_deg` | 14 | `services/motion/continuous_reference.py` | 保留 |
| `stopped_speed_mps` | 6 | `services/continuous_execution.py` | 保留；与 `stopped_yaw_rate_deg_s` 是两个轴 |
| `stopped_yaw_rate_deg_s` | 3 | `simulators/projectairsim/motion.py` | 保留；合并到速度阈值会把"原地旋转可接受"误判成已停 |
| `settle_time_s` | 11 | `services/position_hold.py` | 保留；它是时间窗，与 `position_tolerance_m`（几何半径）不同量纲 |

不合并的三对：`settle_time_s` 与 `position_tolerance_m`（时间与距离）；`stopped_speed_mps` 与 `stopped_yaw_rate_deg_s`（平移与旋转两个轴）；`heading_deadband_deg` 与 `heading_rate_deadband_dps`（误差与速度两个判据）。

### B 历史残留删除清单

| 名字 | 位置 | 现引用 | 处置与理由 | 风险 |
| --- | --- | --- | --- | --- |
| `SegmentResult` | `domain/trajectory.py:71-76` | 3（定义、包入口导入与 `__all__`） | 删。无生产调用方，与 `ExecutionReceipt` 职责重叠 | 无；若有外部脚本反序列化该名字会报 `ImportError` |
| `CameraMountPose` | `domain/pose.py:143` 起 | 1（仅定义） | 删。docstring 声称 AirSim 读回，全仓无一处读取或构造 | 无 |
| `ApproachTarget`、`ApproachAssessment`、`ApproachStatus` 与 `TargetApproach.assess` | `domain/approach.py`、`capabilities/approach/target_approach.py:23` | 5、5、4；`assess` 0 个运行期调用方 | 删。`ApproachSettings` 与 `distance_within_radius` 随能力迁进 `workflows/visual_goal_navigation/policy.py`（裁决 R25），`distance_within_radius` 的现调用点是 `capabilities/completion/visual_goal_request.py:42` | 目前只有 `tests/test_capability_completion.py` 与 `tests/test_reference_contracts.py` 在用，两个测试随之删 |
| 第二份 `MotionLimits` | `services/segment_motion.py:21-93` | 73 行 | 已按合并裁决移入 `domain/motion.py` | 无 |
| `CollisionCheck` | `ports/collision_check.py` | 已删 | R5 删除 | 无 |
| 车道先验 | `domain/lane_reference.py` | 99 行 | 已定删除 | 无 |
| `PlanningWindowInput` 的第二个宿主 | `domain/planning_window.py` | 36 行 | 并入 `domain/sliding_window.py` | 无 |
| `CameraIntrinsics` 两份实现 | `domain/observation.py` 与 `ports/` 相机视图侧 | 15 | 保留两份，加主点差 ≤ 0.5 px 的断言（R14） | 两份漂移只能靠断言发现 |
| `ports/position_hold_vehicle.py` 的 `hold_velocity` | 同文件 | 2 | 保留，实现内部转调 `stream_command`（R7）；全向签名对阿克曼无效，但该端口不由车实现 | 车没有速度保持，只能 `hover()` |

### C 结论数字（裁决 R23 终值）

- 基线 48 → 终值 44。明细：R21 −2（`SegmentResult`、`command_type_for`）；R22 −1（`PlanningWindowInput`）、−2（`SegmentSink`、`StreamSink`）、+1（`HeadingHoldSettings`）；R23 −1（`TaskCompletion`）；补齐 R20 清单漏列 +2（`MotionSegment`、`MotionTrajectory`）。
- `ExecutionReceipt`、`StopResult` 留成契约类型（裁决 R23 第 5 条）：`workflows/visual_goal_navigation/workflow.py:20` 导入 `StopResult`，同文件 189 行读 `exc.receipt.remaining`，`flight.py:7` 导入 `ExecutionReceipt`。
- 搬出契约层、不占总名额：`ServerState`、`ServerStatus`、`BackendUnavailable` 等 7 个生命周期类型进 `assembly/controller/state.py`；`WorkerEvent`、`TaskSubmission` 随 `ports/backend_worker.py` 进 `simulator/worker_protocol.py`；`LogBatch` 进 `runtime/log_buffer.py`（唯一使用点 `services/log_buffer.py:9`）；四个目标接近类型进 `workflows/visual_goal_navigation/policy.py`（其中三个类型与 `assess` 删掉）。`domain/` 26 → 21 个文件（裁决 R25）。
- 终值 44 仍高于 34，差值 10，逐条给 import 行反证（拆掉就不能在构造期报错或推导能力位）：
  - `GoalObserver`：`planner/workflows/visual_goal_navigation/workflow.py:54`
  - `TargetDetector`：`planner/capabilities/targeting/bbox_goal_observation.py:9`
  - `TaskParser`：`planner/application/application.py:7`
  - `TrajectoryPlanner`：`planner/services/planning_window.py:32`
  - `TaskTrace`：`planner/workflows/visual_goal_navigation/workflow.py:57`
  - `FramePublisher`：`planner/simulator/ports.py:29`
  - `ReturnHome`：`planner/simulator/ports.py:30`
  - `StreamingViews`：`planner/simulator/ports.py:31`
  - `WeatherControl`：`planner/simulator/ports.py:32`
  - `AppearanceModelControl`：`planner/simulator/ports.py:28`
  后五条的 import 行都在 `planner/simulator/ports.py` 的 `SimulatorPorts` 字段声明上，拆掉后槽位只能写 `object`，`tests/test_architecture.py` 的“声明槽位有真实实现”检查失效；前五条拆掉后，`adapters/` 与 `application/` 的参数只剩鸭子类型，少接一根线不再在构造期报错。
- 删代码行：本次实删 164 行（`server.py` 115、`approach.py` 49），另有 `ports/backend_worker.py` 28 行搬去 `simulator/`；累计（含前几次裁决）约 420 行。

## 九 未决问题

1. `MotionCapability` 是否进 `SimulatorPorts` 槽位。
   - 选项 A（默认）：只走 `Vehicle.motion_capability()`，`SimulatorPorts` 不加槽位。代价：`SimulatorPorts.from_slots` 的 `_Slot` 占位（`planner/simulator/ports.py`）没有这个方法，控制台不能据能力渲染"横移"控件。
   - 选项 B：加 `motion` 槽并与 `PORT_SLOTS` 同步。代价：两个引擎的 `PORT_SLOTS` 与 `_Slot` 都要改，`tests/test_architecture.py:141` 的槽位一致检查跟着改，收益是一个暂时没人用的控件开关。
2. `min_turn_radius_m` 取常量还是取配置。
   - 选项 A（默认）：常量 3.20 m，由 `RoverDriveModel.yaw_rate_per_steer_mps = 17.9`（`simulators/projectairsim/drive.py:40`）与 `max_steering = 1.0` 推出，即 `180 / (π · 17.9)`。代价：换车模型要改代码。
   - 选项 B：加 `motion.min_turn_radius_m` 配置键。代价：多一个没人标定过的数，且与 `drive.py` 的标定值可能不一致。
3. `MotionModel` 的基类。
   - 选项 A（默认）：`class MotionModel(str, Enum)`，与 `TaskKind`（`domain/mission.py:10`）一致，配置与日志能直接写出枚举值。
   - 选项 B：纯 `Enum`，与 `docs/refactor/00-boundaries.md` 第四节的字面一致。代价：序列化时要额外转换，`/api/status` 里会出现 `MotionModel.HOLONOMIC` 这种字符串。
4. `CarCommand.reverse` 与速度符号的校验强度。
   - 选项 A（默认）：严格相等，`reverse is (forward_speed_mps < 0)`，否则构造期抛错。
   - 选项 B：只在两者矛盾时报错、允许 `reverse=False` 配负速度。代价：`reverse` 变成可被忽略的字段，适配层还得自己判符号。
5. 已由 R23 第 4 条消除：四个目标接近类型只被 `capabilities/approach/target_approach.py` 与 `capabilities/completion/visual_goal_request.py` 引用，两个文件随类型一起迁进 `workflows/visual_goal_navigation/policy.py`，`adapters/` 不 import 它们，不存在依赖冲突。
