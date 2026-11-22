# Planner 代码清单

按一次任务从进来到结束的顺序排列，每个文件一行。

规模：`planner/` 下 220 个 Python 文件、约 2.6 万行；控制台 9 个文件；配置 34 个；脚本 8 个；测试 72 个。

## 0 一条链路

```
浏览器 console/
   │ HTTP
adapters/http/server_api.py            路由、状态码、能力门
   │
application/server_controller.py       状态机、命令队列、换图换模型
   │ worker 线程或子进程
simulators/<name>/session.py           连仿真器、拥有载具
   │ mission_inputs() ⇄ application/mission_assembly.py 的 attach_arms()
application/application.py             解析 → 校验 → 跑
   │
adapters/task_parser/                  自然语言 → MissionPlan
application/mission_validator.py       参数检查
application/workflow_router.py         查表选工作流
   │
workflows/<task>/                      阶段编排
   ├─ capabilities/                     纯算法
   ├─ services/                         有状态的部件
   └─ adapters/{trajectory_planner, target_detection, task_completion}
   │
ports/vehicle.py → simulators/<name>/  真正发指令
```

读这份清单时记住一件事：**下面各章的顺序是数据流的方向，不是依赖的方向。** `domain/` 和 `ports/` 谁都不依赖，`adapters/` 被所有人依赖。

---

## 1 契约：数据长什么样，谁能做什么

### 1.1 `planner/domain/`（28 个文件，2511 行）

纯数据和不变量校验。不 import 任何人，也不会有网络、线程、设备。

| 文件                            | 做什么                                                                                                                                                                                                                |
| ------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `__init__.py`                   | 包说明：纯领域类型，无仿真器与网络依赖                                                                                                                                                                                |
| `pose.py`                       | 全栈共用的位姿约定。`WorldPose`（世界系，x 北 y 东 z 下，yaw 顺时为正）、`RelativePoseDelta`（机体系增量）、`CameraMountPose`（相机挂点，可保存可恢复）、`wrap_yaw_deg`、`resolve_yaw_delta_deg`、`relative_to_world` |
| `observation.py`                | 停稳后的一帧同步观测：RGB、`depth_m`、深度语义（光轴深度还是射线距离）、`CameraIntrinsics.from_horizontal_fov`                                                                                                        |
| `trajectory.py`                 | 五种轨迹类型。`RelativeTrajectory`（机体系相对航点）、`WorldTrajectory`、`MotionSegment`（两个世界位姿之间的一段）、`MotionTrajectory`（可执行的整条）、`SegmentResult`                                               |
| `motion.py`                     | `MotionLimits`（速度、加速度、jerk、位置与航向容差）、`ContinuousTrackingSettings`（流式跟踪与平滑）、`ContinuousReference`                                                                                           |
| `execution.py`                  | `ExecutionBudget`（任务期限）、`ExecutionReceipt`（实测回执）、`StopResult`、`ExecutionRecord`（一次执行的实测端点历史）                                                                                              |
| `mission.py`                    | `TaskKind`（三种任务）、`MissionStage`、`MissionPlan`、`TaskParameters` 联合类型                                                                                                                                      |
| `result.py`                     | `StageResult`、`MissionResult`、`StageSummary`、`ResultMeasurement`、`CompletionBasis`                                                                                                                                |
| `errors.py`（在 `planner/` 根） | 全栈共用的异常族：`ProtocolError`、`ObservationError`、`ExecutionError`、`SimulatorConnectionError`、`ModelCallTimeout`、`BudgetExceeded`、`MotionPaused`、`FreeNavigationStalled`                                    |
| `visual_goal.py`                | 视觉目标导航的不可变契约：`GoalObservation`、`GoalGeometry`、`GoalReference`、`GoalVisibility`、`VisualGoalState`、`VisualGoalPlanningContext`、结果类型                                                              |
| `visual_goal_limits.py`         | 任务策略值：到达半径、最大垂直偏移、`approach_settings()`                                                                                                                                                             |
| `visual_goal_completion.py`     | `VisualGoalTriggerRules`（半径暂停上限）、`VisualGoalTargetSnapshot`、`VisualGoalCompletionInput`                                                                                                                     |
| `basic_motion.py`               | 数值动作参数（前后、横移、升降、转向）、状态枚举、结果                                                                                                                                                                |
| `free_navigation.py`            | 自由导航参数（只有一个自然语言指令）、状态、结果                                                                                                                                                                      |
| `approach.py`                   | 到达半径契约：`ApproachSettings`、`ApproachTarget`、`ApproachAssessment`、`ApproachStatus`                                                                                                                            |
| `sliding_window.py`             | 滑窗的不可变锚：`SlidingWindowSettings`、`QueuedWaypoint`、`PlanningSnapshot`、`ContinuationCandidate`、`PlanningOutput`                                                                                              |
| `planning_window.py`            | `PlanningWindowInput`：一次续规划请求的冻结输入                                                                                                                                                                       |
| `streaming.py`                  | 流式实测状态：`TimedVehicleState`、`ControlTelemetry`、`QueuePauseSnapshot`、`QueueObservationAnchor`                                                                                                                 |
| `trajectory_telemetry.py`       | 控制台读的剩余航点快照，JSON 安全，含 `build_queue_snapshot`、序列化与反序列化                                                                                                                                        |
| `target_detection.py`           | 面向模型的检测结果：`NormalizedBoundingBox`（全图归一化）、`TargetDetection`、`TargetDetectionInput`                                                                                                                  |
| `target_state.py`               | 阶段内的目标位置估计 `TargetPositionState`，以及检测器最新主张 `TargetObservationEvidence`                                                                                                                            |
| `depth.py`                      | 深度显示协议的版本校验与描述文字                                                                                                                                                                                      |
| `depth_visualization.py`        | 把米制深度渲染成可视化图（给模型和诊断用）                                                                                                                                                                            |
| `depth_foreground.py`           | 深度层过滤的设置与证据类型                                                                                                                                                                                            |
| `lane_reference.py`             | 车道先验：车道中心、切向、边界，来自仿真器 HD 地图而非图像模型                                                                                                                                                        |
| `weather.py`                    | 天气目录：8 个参数的顺序与校验、预设、`merge_weather`、payload 互转                                                                                                                                                   |
| `view.py`                       | 机位目录：`front_view_for`、`default_view_for`、`cycle_delta`、模式校验                                                                                                                                               |
| `runtime.py`                    | 仿真器家族与能力位：`Capabilities`、`SimulatorFamily`、`FamilyCatalog`、`parse_family_catalog`                                                                                                                        |
| `server.py`                     | HTTP 后端生命周期类型：`ServerState`、`WorkerEvent`、`WorkerEventKind`、`TaskSubmission`、`ServerStatus`、`LogBatch`、`BackendUnavailable`                                                                            |
### 1.2 `planner/ports/`（14 个文件，270 行）

只有 `Protocol` 定义，没有实现。仿真器实现了哪个端口，就代表它能做哪件事。

| 文件                       | 做什么                                                                                                                                                                 |
| -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `__init__.py`              | 包说明：端口是结构化接口，仿真器适配层只需形状匹配                                                                                                                     |
| `vehicle.py`               | 最核心的端口：`current_pose`、`execute_segment`、`cancel`、`hover`、`collision_state`                                                                                  |
| `observation_source.py`    | `ObservationSource.capture()`；`ControlledObservationSource.request_capture()`（暂停轨迹后按需采一帧）；`MovingObservationSource.capture_moving()`（运动中取帧，只读） |
| `streaming_vehicle.py`     | `stream_state()` 与 `stream_velocity()`：20 Hz 的速度控制                                                                                                              |
| `position_hold_vehicle.py` | 位置保持：`hold_velocity` 加 `cancel`                                                                                                                                  |
| `capabilities.py`          | 五个可选能力端口：`ReturnHome`、`WeatherControl`、`StreamingViews`、`AppearanceModelControl`、`FramePublisher`                                                         |
| `backend_worker.py`        | worker 端口：`start`、`submit`、`stop`、`home`、`activate`、`set_view_mode`、`set_weather`、`poll`、`shutdown`、`terminate`                                            |
| `collision_check.py`       | 碰撞查询，控制线程用；没有实现就没有碰撞证据                                                                                                                           |
| `goal_observer.py`         | 视觉目标观测（带参考帧的目标识别）                                                                                                                                     |
| `target_detector.py`       | 通用目标检测                                                                                                                                                           |
| `task_completion.py`       | 语义完成判定的泛型端口，请求类型由各任务自己定义                                                                                                                       |
| `task_parser.py`           | 指令解析                                                                                                                                                               |
| `trajectory_planner.py`    | 轨迹规划：输入当前观测与指令，输出五个新航点                                                                                                                           |
| `task_trace.py`            | 诊断记录端口                                                                                                                                                           |

---

## 2 启动：配置读进来，所有东西接起来

| 文件                                                | 做什么                                                                                                                                                                                                                                                                                                                                                                                                    |
| --------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `planner/config.py`（1775 行）                      | 全家配置。`app.yml`、`maps.yaml`、`simulators.yaml`、场景 JSONC、各 profile 的 yaml 都从这里进内存。二十多个 dataclass（`DroneSettings`、`CameraSettings`、`TrajectoryPlannerSettings`、`VisualGoalNavigationSettings`、`ServerSettings`、`MapCatalog`、`MapEntry`、`VehicleModelCatalog`…），加上 profile 目录发现、profile 漂移检查、地图目录加载、场景默认值展平、相机设置构造。入口是 `load_config()` |
| `planner/server_bootstrap.py`（139 行）             | HTTP 服务的装配：模型适配、帧汇、轨迹汇、controller、worker 工厂，一次接完。`build_resources()` 与 `build_server()`                                                                                                                                                                                                                                                                                       |
| `planner/application/mission_assembly.py`（359 行） | 整个仓库唯一同时认识仿真器和任务管线的地方。`MissionInputs` 是仿真器交出来的东西，`assemble_missions()` 还它 `VehicleArm`。少接一根线在构造期报错                                                                                                                                                                                                                                                         |
| `planner/application/factories.py`（251 行）        | 按配置造适配器：`build_detector`、`build_planner`、`build_task_parser`、`build_completion_checker`、`build_geometry`、`build_trace_factory`。CLI 和 HTTP 共用同一套解析规则                                                                                                                                                                                                                               |

---

## 3 HTTP 边界与控制台

| 文件                                                  | 做什么                                                                                                                                                                                        |
| ----------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `planner/adapters/http/server_api.py`（512 行）       | 全部路由与请求模型。`POST /api/tasks` 只在 idle 接受；`POST /api/stop` 在运动点之间中断；能力不够返回 501。仿真器和模型调用都不在 handler 里                                                  |
| `planner/adapters/http/video.py`（71 行）             | MJPEG：`encode_jpeg`、`mjpeg_chunks`。每个观看者一个异步生成器，只短暂轮询帧汇                                                                                                                |
| `planner/adapters/http_errors.py`（66 行）            | 把模型端点的 httpx 异常翻译成人能看懂的一句话，带上状态码、请求 URL 和响应片段                                                                                                                |
| `planner/application/server_controller.py`（1132 行） | 后端生命周期的唯一拥有者。监督线程持有 worker；`disconnected → connecting → idle ⇄ running → stopping` 状态机；提交、停止、回出生点、切载具、切地图、切机位、切外观、切天气、切 profile、退出 |
| `console/index.html`（212 行）                        | 单页控制台的骨架：状态行、任务卡、世界卡、设置卡、三维视图                                                                                                                                    |
| `console/styles.css`（567 行）                        | 全部样式，无构建步骤                                                                                                                                                                          |
| `console/presets.json`（8 行）                        | 预设指令下拉框的数据源，加一条只改这个文件                                                                                                                                                    |
| `console/src/core.js`（280 行）                       | 骨架层：常量、共享状态、DOM 工具、渲染入口、能力门控、tab、HTTP 请求                                                                                                                          |
| `console/src/app.js`（429 行）                        | 入口：状态轮询、任务提交与停止、载具切换、事件绑定                                                                                                                                            |
| `console/src/world.js`（288 行）                      | 场景侧：地图启动/切换/停止、天气                                                                                                                                                              |
| `console/src/settings.js`（206 行）                   | 这一趟用什么跑：载具外观模型与规划器选择                                                                                                                                                      |
| `console/src/flight.js`（351 行）                     | 执行侧显示：剩余航点列表、轨迹喂数据、画面通道（xpra 与 MJPEG）、机位                                                                                                                         |
| `console/src/scene3d.js`（204 行）                    | 三维轨迹视图，three.js 从 CDN 动态 import，取不到就只降级这一块                                                                                                                               |

---

## 4 指令进站：解析 → 校验 → 路由 → 跑

| 文件                                                                    | 做什么                                                                            |
| ----------------------------------------------------------------------- | --------------------------------------------------------------------------------- |
| `planner/adapters/task_parser/openai_task_parser.py`（178 行）          | OpenAI 兼容的任务解析器：发指令，收 JSON，交出 `MissionPlan`                      |
| `planner/adapters/task_parser/prompt.py`（84 行）                       | 系统提示词与用户提示词，列出当前支持的任务类型                                    |
| `planner/adapters/task_parser/response_schema.py`（63 行）              | 严格 JSON：拒绝 `NaN`、重复字段，校验外层信封                                     |
| `planner/adapters/task_parser/task_factory.py`（55 行）                 | 字典 → `MissionPlan`，按任务类型分发到各解码器                                    |
| `planner/adapters/task_parser/tasks/__init__.py`（74 行）               | 内置任务解码器的显式注册表，import 不产生副作用                                   |
| `planner/adapters/task_parser/tasks/basic_motion.py`（109 行）          | 数值动作的线上格式与严格解码，地面与空中两套                                      |
| `planner/adapters/task_parser/tasks/free_navigation.py`（46 行）        | 自由导航的线上格式与解码                                                          |
| `planner/adapters/task_parser/tasks/visual_goal_navigation.py`（66 行） | 视觉目标的线上格式与解码                                                          |
| `planner/application/application.py`（41 行）                           | `PlannerApplication`：`prepare()` 解析、`execute_plan()` 跑、`execute()` 两步合一 |
| `planner/application/mission_validator.py`（68 行）                     | 通用准入，各任务的检查显式注入；`StageValidator` 协议                             |
| `planner/application/mission_runner.py`（48 行）                        | 按顺序跑计划中的阶段，首个失败即停，汇总结果                                      |
| `planner/application/workflow_router.py`（21 行）                       | 任务类型 → 工作流的查表                                                           |
| `planner/application/reporting.py`（70 行）                             | 同一份结果文本给 CLI 和 HTTP 日志窗用；`mission_report`、`task_finish_events`     |
| `planner/workflows/base.py`（11 行）                                    | 工作流生命周期协议：`prepare`、`run`、`close`，prepare 不发指令                   |

---

## 5 三条任务线

### 5.1 数值动作 `basic_motion`

| 文件                                           | 做什么                                                                                             |
| ---------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| `workflows/basic_motion/workflow.py`（162 行） | 直接构造同步运动段执行，不做观测门。有原生动作原语的载具直接用，没有的（运动学车）用五次多项式合成 |
| `workflows/basic_motion/__init__.py`（3 行）   | 导出                                                                                               |

### 5.2 视觉目标导航 `visual_goal_navigation`

| 文件                                                        | 做什么                                                                                                         |
| ----------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `workflows/visual_goal_navigation/workflow.py`（415 行）    | 主流程：识别 → 靠近 → 验证。依赖全部注入，`prepare` 只准备资源                                                 |
| `workflows/visual_goal_navigation/approach.py`（73 行）     | 直接用模型轨迹执行，并准备不可变的续规划上下文                                                                 |
| `workflows/visual_goal_navigation/continuous.py`（386 行）  | 连续模式：20 Hz 队列跟随、后台滑窗、碰撞后退与清队列重规划                                                     |
| `workflows/visual_goal_navigation/flight.py`（126 行）      | 暂停、语义验证、保留飞行恢复                                                                                   |
| `workflows/visual_goal_navigation/session.py`（104 行）     | 单次运行的局部状态与控制流小工具，`model_call` 包装                                                            |
| `workflows/visual_goal_navigation/termination.py`（110 行） | 每轮的退出策略：`VisualGoalExitPolicy`（运动记录、规划调用计数、锚点新鲜度）、`SearchPolicy`（找目标时往哪转） |
| `workflows/visual_goal_navigation/__init__.py`（5 行）      | 导出                                                                                                           |

### 5.3 自由导航 `free_navigation`

| 文件                                                | 做什么                                                               |
| --------------------------------------------------- | -------------------------------------------------------------------- |
| `workflows/free_navigation/workflow.py`（404 行）   | 沿路无界行驶，无目标无幅度。滑窗开启时走连续模式                     |
| `workflows/free_navigation/continuous.py`（445 行） | 连续跟随主体：与视觉目标连续模式共用队列与后台规划，碰撞后倒车重规划 |
| `workflows/free_navigation/__init__.py`（6 行）     | 导出                                                                 |

### 5.4 `workflows/__init__.py`（11 行）

导出工作流。

---

## 6 算法层 `capabilities/`（27 个文件，1074 行）

给数据出结果，原则上不碰设备。实际有例外，见第 16 章。

### 6.1 规划 `planning/`

| 文件                                      | 做什么                                                                                         |
| ----------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `planning/geometric_planner.py`（110 行） | 确定性几何规划器。没配模型端点时的兜底：从目标表面算五个机体航点，收水平距离、修航向、有界抬降 |
| `planning/__init__.py`（5 行）            | 导出                                                                                           |

### 6.2 目标定位 `targeting/`

| 文件                                             | 做什么                                                                                                |
| ------------------------------------------------ | ----------------------------------------------------------------------------------------------------- |
| `targeting/depth_geometry.py`（107 行）          | 相机基与深度反投影：`validate`、`ray`、`project_camera`、`estimate`。约定 +X 光轴前、+Y 图右、+Z 图下 |
| `targeting/depth_surface_sampling.py`（39 行）   | 精确读一个标定过的原始深度像素，不做邻域处理                                                          |
| `targeting/depth_foreground.py`（74 行）         | 逐点的深度不连续过滤，不建全局直方图、不造新点                                                        |
| `targeting/association.py`（77 行）              | 首帧参考不可变；`accept_position`、`bound_target_geometry`、`bound_target_goal`、`make_reference`     |
| `targeting/visual_goal_refresh.py`（133 行）     | 阶段内的目标刷新：只有 owning 线程接受只带数据的结果，`begin`/`accept`/`submit`/`poll`/`close`        |
| `targeting/bbox_goal_observation.py`（34 行）    | 框选式观测：调检测器端口，表面像素由模型挑                                                            |
| `targeting/__init__.py`（5 行）                  | 导出                                                                                                  |

### 6.3 到达与完成 `approach/` `completion/`

| 文件                                                 | 做什么                                                 |
| ---------------------------------------------------- | ------------------------------------------------------ |
| `approach/target_approach.py`（29 行）               | 实测距离的到达评估，不带路由授权                       |
| `approach/__init__.py`（3 行）                       | 导出                                                   |
| `completion/visual_goal.py`（25 行）                 | 停稳距离准入；可见性交给模型判断                       |
| `completion/visual_goal_request.py`（51 行）         | 新鲜深度或阶段绑定证据的显式准入，两者都不构成语义完成 |
| `completion/visual_goal_trigger.py`（48 行）         | 半径暂停策略：多久采一次、什么时候该停                 |
| `completion/visual_goal_verification.py`（39 行）    | 在冻结的本地证据上做语义校验，带预算与重试             |
| `completion/visual_goal_retained_motion.py`（17 行） | 已执行增量不重放的世界目标重锚                         |
| `completion/__init__.py`（1 行）                     | 包说明                                                 |

### 6.4 参数准入 `task_validation/`

| 文件                                                 | 做什么                             |
| ---------------------------------------------------- | ---------------------------------- |
| `task_validation/basic_motion.py`（12 行）           | 只校验接线，数值动作用不到飞行准入 |
| `task_validation/free_navigation.py`（14 行）        | 只校验接线，自由导航没有数值参数   |
| `task_validation/visual_goal_navigation.py`（27 行） | 静态视觉目标飞行的预检             |
| `task_validation/__init__.py`（1 行）                | 包说明                             |

### 6.5 滑窗原语 `sliding_window/`

| 文件                                   | 做什么                                                                                                 |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| `sliding_window/planning.py`（136 行） | owning 侧持有模型 future 的集合，结果永不自动授权。`begin`、`submit`、`poll`、`wait`、`discard`、`end` |
| `sliding_window/scheduler.py`（16 行） | 纯策略：航点数量加预估平滑执行时长，决定要不要提前补                                                   |
| `sliding_window/worker.py`（77 行）    | 一个惰性模型 worker，带 `threading.Condition`，无回调、不动队列、不碰飞行                              |
| `sliding_window/__init__.py`（1 行）   | 包说明：授权归各工作流                                                                                 |

### 6.6 `capabilities/basic_motion_conversion.py`（19 行）

无状态的语义轴转换：参数 → 目标位姿。

---

## 7 运行时部件 `services/`（29 个文件，2710 行）

有状态、有线程、要排队、占资源的都在这里。

### 7.1 执行链

| 文件                                | 做什么                                                                                                                                                                |
| ----------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `execution.py`（263 行）            | 执行服务。拿一条轨迹，逐段下发、轮询、检查点、超时、停止。`execute`、`execute_with_receipt`、`acquire_flight`、`request_stop`、`check_safety`、`bind_hold_checkpoint` |
| `continuous_execution.py`（170 行） | 单 owner 的队列跟随器：`begin`、`sample`、`tick`、`pause`、`resume`、`drain_telemetry`                                                                                |
| `position_hold.py`（171 行）        | 位置保持：观测期间、任务结束、空闲等待共用。`start`、`release`、`tick`、`wait`、`climb_by`                                                                            |
| `segment_motion.py`（155 行）       | 五次多项式运动段规划，`MotionLimits`（与 `domain/motion.py` 同名不同类）、`quintic`、`duration_for`、`plan_segment`                                                   |

### 7.2 运动律 `motion/`

| 文件                                      | 做什么                                                               |
| ----------------------------------------- | -------------------------------------------------------------------- |
| `motion/base.py`（7 行）                  | `MotionPlanner` 协议：`create_motion`                                |
| `motion/synchronized.py`（75 行）         | 航点处停稳、平移与偏航同一周期完成的同步规划器                       |
| `motion/continuous.py`（105 行）          | 折线跟随，速度连续、偏航独立。`reached` 含"已驶过端点"的横向走廊判据 |
| `motion/continuous_reference.py`（81 行） | 有界路径预览与锁存航向对齐，不改队列端点                             |
| `motion/continuous_speed.py`（96 行）     | 整条队列的速度预览、转弯限速、jerk 感知的刹车距离                    |
| `motion/heading_servo.py`（97 行）        | 航向误差 → 角速度的唯一一条指令律，带阻尼、死区、制动曲线            |
| `motion/command_smoothing.py`（33 行）    | 有状态的加速度与 jerk 限幅                                           |
| `motion/hold_handoff.py`（25 行）         | 停稳前的纯平移交接                                                   |
| `motion/__init__.py`（4 行）              | 导出                                                                 |

### 7.3 队列与续规划

| 文件                                | 做什么                                                                                                                                                      |
| ----------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `trajectory_queue.py`（149 行）     | owner 线程的世界航点队列：消费前缀不改修订号；`load_batch`、`pending`、`consume`、`append_continuation`、`consume_stream`、`pause_stream`、`restore_stream` |
| `planning_window.py`（332 行）      | 任务无关的滑窗桥：接实测观测与可执行运动，转发模型续规划，从不判定完成、不调载具                                                                            |
| `trajectory_hub.py`（118 行）       | 控制台剩余航点面板的发布器；工作流线程写、HTTP 读、子进程 `adopt`                                                                                           |
| `trajectory_validation.py`（56 行） | 轨迹准入：执行锚点、单步长度、单步转角                                                                                                                      |
| `trajectory_transform.py`（52 行）  | 三种轨迹坐标系互转：`relative_to_world`、`world_to_relative`、`world_pose_delta_in_reference_frame`                                                         |

### 7.4 观测与数据汇

| 文件                               | 做什么                                                                                   |
| ---------------------------------- | ---------------------------------------------------------------------------------------- |
| `frame_hub.py`（70 行）            | 线程安全的最新帧：写入侧是相机回调（限流），读取侧是 MJPEG 处理器                        |
| `moving_observations.py`（47 行）  | 有界只读取帧泵，拿不到控制器                                                             |
| `observation_snapshot.py`（18 行） | 交给模型 worker 前冻结图像与深度                                                         |
| `stream_timeline.py`（32 行）      | 按仿真器时间记录的队列历史，用于延迟到达的图像                                           |
| `free_space.py`（281 行）          | 从标定米制深度算出的无障碍扇面：每个方位扇区最近的高于地面障碍，供提示词用               |
| `target_coordinates.py`（34 行）   | 世界点转机体水平系：`horizontal_distance`、`nearest_surface`、`point_in_horizontal_body` |

### 7.5 后台与资源

| 文件                               | 做什么                                                     |
| ---------------------------------- | ---------------------------------------------------------- |
| `background_operation.py`（96 行） | 一个受限的纯数据后台操作，迟到结果隔离，显式关闭           |
| `model_calls.py`（92 行）          | 单飞模型调用，带超时；操作里只能有模型 I/O，检查点在调用方 |
| `log_buffer.py`（44 行）           | 有界日志与共享游标，多个 HTTP 读者各读各的                 |
| `resource_cleanup.py`（18 行）     | 每个资源都尝试关一次，失败聚合上报                         |
| `__init__.py`（17 行）             | 导出运动段规划的几个函数                                   |

---

## 8 模型端点适配 `adapters/`（HTTP 之外的 12 个文件）

| 文件                                                                | 做什么                                                                                         |
| ------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `multimodal_chat.py`（146 行）                                      | 共享的无状态 HTTP 传输：OpenAI 兼容 `/chat/completions`、deepseek 方言、Anthropic Messages API |
| `chat_request_options.py`（32 行）                                  | 按显式选择的 HTTP 方言给生成参数，不猜 URL 和模型                                              |
| `image_encoding.py`（62 行）                                        | RGB 数组 → data URL；`resize_longest_side` 控长边                                              |
| `model_visual_inputs.py`（52 行）                                   | 给模型准备的深度图：内参校验、与 RGB 对齐                                                      |
| `target_detection/client.py`（62 行）                               | OpenAI 兼容的 grounding 适配器；`parse_target_detection` 严格解析                              |
| `target_detection/manual.py`（50 行）                               | 人工检测器：直接给框，用于没有视觉端点时跑通深度反投影与轨迹环                                 |
| `target_detection/prompts/system.py`（44 行）                       | 只做检测的 bbox 与表面像素协议                                                                 |
| `target_detection/prompts/visual_goal_target_detection.py`（33 行） | 视觉目标任务的识别消息                                                                         |
| `task_completion/client.py`（27 行）                                | 布尔完成判定的传输层，协议只有一个真假                                                         |
| `task_completion/prompt.py`（16 行）                                | 适配层本地的消息类型与 builder 协议                                                            |
| `task_completion/prompts/visual_goal_completion.py`（151 行）       | 只做完成判定的消息，含绑定证据摘要，从不给轨迹规划器复用                                       |
| `trajectory_planner/qwen_vl_planner.py`（111 行）                   | OpenAI 兼容的 VLM 轨迹规划器，输入 RGB 加对齐后的深度可视化                                    |
| `trajectory_planner/response_parser.py`（40 行）                    | 解析模型的 JSON 航点数组，数量必须对得上                                                       |
| `trajectory_planner/prompts/base.py`（511 行）                      | 协议与通用系统提示词、地面与空中两套用户提示词构造                                             |
| `trajectory_planner/prompts/free_navigation.py`（597 行）           | 自由导航提示词：系统提示、车道先验格式化、扇面格式化、方位标签                                 |
| `trajectory_planner/prompts/visual_goal_navigation.py`（306 行）    | 视觉目标提示词：目标向量、待执行前缀、地面扇面注入、目标侧规则                                 |
| `diagnostics/visual_goal_trace.py`（140 行）                        | 文件系统诊断记录：观测、目标几何、控制采样；从不写模型原文与密钥                               |

---

## 9 仿真器进程栈 `simulator/`（12 个文件，2474 行）

两套仿真器共用的进程、协议与注册。这里不许 import 任何仿真器客户端库。

| 文件                             | 做什么                                                                                                      |
| -------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `__init__.py`（5 行）            | 包说明：进程栈与 worker，不含仿真器专用代码                                                                 |
| `xpra_start.py`（591 行）        | `xpra start --start-child` 起停一个仿真器：渲染 argv、检查端口、读窗口、判归属、停栈、重启                  |
| `argv.py`（78 行）               | argv 模板渲染，两套家族走同一条代码路径                                                                     |
| `processes.py`（284 行）         | 进程原语：端口存活、监听者 PID、进程身份与后代、验证后终止、display 文件回收                                |
| `marker.py`（35 行）             | 地图标记文件：驱动写，路由与 HTTP 读                                                                        |
| `router.py`（113 行）            | 把 controller 的调用分发给各家族驱动；`FamilyRouter` 是唯一的双家族门面                                     |
| `registry.py`（88 行）           | 仿真器 id → 端口槽位 → 能力位的解析，含 `build_session`                                                     |
| `ports.py`（200 行）             | `SimulatorPorts`：任务逻辑与仿真器之间唯一的缝。两张表 `TASK_REQUIREMENTS`、`ACTION_PORTS` 是能力的唯一来源 |
| `worker.py`（277 行）            | 进程内 worker 线程：自己造会话、一次跑一个任务、发生命周期事件                                              |
| `subprocess_worker.py`（336 行） | 子进程 worker：为 AirSim 的依赖冲突而存在，父侧收发命令与事件                                               |
| `mission_child.py`（392 行）     | 子进程侧：读命令、拥有会话、写事件，日志经协议转发                                                          |
| `telemetry_relay.py`（76 行）    | 给子进程用的轨迹汇替身：造同形状快照，按 0.4 秒合并后送回父进程                                             |

---

## 10 各仿真器实现 `simulators/`（32 个文件，6499 行）

### 10.1 `simulators/__init__.py`（6 行）

包说明：每个引擎一个包，只有这里 import 客户端库。

### 10.2 ProjectAirSim（UE5）`simulators/projectairsim/`（16 个文件）

| 文件                           | 做什么                                                                                                        |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------- |
| `runtime.py`（92 行）          | 端口槽位声明与 `build_session`                                                                                |
| `connection.py`（360 行）      | 连接、场景加载、仿真时钟推进、天气、切机位；一个连接独占 pynng 通信，串行化所有请求                           |
| `session.py`（790 行）         | 双载具任务会话：拥有连接、场景和两台车；`mission_inputs`、`attach_arms`、`run`、`idle_tick`、天气、机位、退出 |
| `drone.py`（741 行）           | 无人机适配：位姿、起降、速度飞行、观测、回出生点、传送、碰撞、流式状态                                        |
| `rover.py`（410 行）           | 无人车适配：位姿、驾驶、观测、接触；`rover_tracking_limits` 把走廊放宽到车宽                                  |
| `motion.py`（235 行）          | 世界位姿 profile 的定时跟踪：参考采样、速度与偏航反馈、加速度限幅、预算与位姿检查                             |
| `drive.py`（170 行）           | 参考速度与偏航 → 车的执行器量：转向角与油门，含前馈模型与速度 PID                                             |
| `collision.py`（145 行）       | 原生碰撞接触，按法线区分地面与墙                                                                              |
| `observation.py`（199 行）     | 相机传感器的同步观测；停稳采集时小步推进暂停时钟直到渲染出图                                                  |
| `frames.py`（212 行）          | 单相机 RGB 与深度的同步 topic 流，按仿真时间配对                                                              |
| `moving_source.py`（28 行）    | 非拥有式的运动取帧代理                                                                                        |
| `coordinates.py`（206 行）     | 纯 numpy 的 NED 几何：四元数、RPY、机体位姿、速度、偏航角速度                                                 |
| `camera.py`（125 行）          | 从场景 JSONC 校验相机（纯函数，不 import 仿真器）：尺寸、FOV、挂点、内参                                      |
| `async_bridge.py`（69 行）     | 同步控制线程里跑 asyncio 命令的桥                                                                             |
| `streaming_views.py`（543 行） | 机位循环的运行时标定：xpra 截图、签名比对、指派求解、校验                                                     |
| `__init__.py`（44 行）         | 惰性导出，避免 import 时就拉进客户端库                                                                        |

### 10.3 AirSim（UE4）`simulators/airsim/`（13 个文件）

| 文件                              | 做什么                                                                 |
| --------------------------------- | ---------------------------------------------------------------------- |
| `runtime.py`（114 行）            | 端口槽位与 `build_ports`、`build_session`                              |
| `session.py`（317 行）            | UE4 任务会话：跑任务、关会话、空闲心跳、天气                           |
| `connection.py`（270 行）         | 连接与 RPC 串行化：状态 RPC 与长时运动 RPC 分开，起飞、准备、关闭      |
| `vehicle.py`（222 行）            | 载具适配：位姿、执行段、取消、原生数值动作、悬停、碰撞、流式速度与保持 |
| `observation_source.py`（258 行） | 观测源：RGB 与深度解码、配对校验、各类采集入口                         |
| `observation_gate.py`（151 行）   | 暂停轨迹、保持位置、按需服务采集请求                                   |
| `server_vehicle.py`（143 行）     | 仿真器就绪判定与固定位置保持，独占执行线程                             |
| `motion_tracker.py`（158 行）     | 定时世界位姿 profile 跟踪，航向与速度解耦                              |
| `camera_model.py`（61 行）        | 内参、相机矩阵、四元数转旋转                                           |
| `weather.py`（61 行）             | AirSim 的 8 个天气参数与规范名映射                                     |
| `recording.py`（190 行）          | 后台线程持续录制相机观测到磁盘                                         |
| `rpc_loop.py`（54 行）            | msgpack-rpc 与 Tornado 4.5 的可重启事件循环兼容层                      |
| `__init__.py`（7 行）             | 导出                                                                   |

---

## 11 配置数据 `config/`（32 个文件）

| 文件                                             | 做什么                                                                                        |
| ------------------------------------------------ | --------------------------------------------------------------------------------------------- |
| `app.yml`                                        | 服务主配置：模型端点、超时、运动限制、任务参数                                                |
| `maps.yaml`                                      | 可切换地图清单，每个地图归属某个仿真器家族                                                    |
| `simulators.yaml`                                | 各家族的启动 argv、端口、探针、display                                                        |
| `airsim_settings.json`                           | UE4 客户端自己的设置（载具名、相机）                                                          |
| `core/default/base.yaml`                         | default 配置档的基线                                                                          |
| `core/default/tasks/visual_goal_navigation.yaml` | 视觉目标导航的任务规则                                                                        |
| `core/vlm/base.yaml`                             | VLM 配置档基线                                                                                |
| `core/vlm/tasks/visual_goal_navigation.yaml`     | VLM 档的视觉目标规则                                                                          |
| `scenes/<map>/scene_*.jsonc`                     | 场景文件：地图、时间、天气、载具摆放。每个地图 4 个（单车无人机、单车无人车、双车、双六旋翼） |
| `scenes/<map>/robot_*.jsonc`                     | 机器人配置：相机与深度传感器、轮子与摩擦、飞行参数、外观模型                                  |

三个地图：`city`、`jiaoliucenter`、`neighborhood`，各 8 个 JSONC。

---

## 12 脚本 `scripts/`（8 个文件）

| 文件                             | 做什么                                             |
| -------------------------------- | -------------------------------------------------- |
| `serve_planner.py`               | 一条命令起后端与控制台                             |
| `smoke_projectairsim.py`         | M1 冒烟：加载双载具场景并取帧                      |
| `smoke_projectairsim_adapter.py` | M2 验收：实机检查适配层                            |
| `smoke_projectairsim_rover.py`   | M3 验收：实机跑无人车工作流                        |
| `smoke_projectairsim_drone.py`   | M4 验收：实机跑无人机工作流                        |
| `calibrate_streaming_views.py`   | 在实机上记录机位循环顺序，写回标定结果             |
| `check_profiles.py`              | 配置档之间只允许差"切换时能生效"的东西，违反就失败 |
| `probe_ue4_env.py`               | 首次 UE4 起来之前检查机器环境                      |

---

## 13 测试 `tests/`（72 个文件）

| 文件                                     | 覆盖什么                                                                                        | 测试数 |
| ---------------------------------------- | ----------------------------------------------------------------------------------------------- | ------ |
| `support.py`                             | 手搭观测与配置目录的共用工具，不被收集                                                          | —      |
| `test_architecture.py`                   | 三条架构不变量：只有 `simulators/` 能 import 客户端库、共享层不依赖具体仿真器、能力位由端口推导 | 6      |
| `test_config.py`                         | 配置加载、校验、目录发现                                                                        | 30     |
| `test_reference_config.py`               | 参考配置的固定值                                                                                | 7      |
| `test_profiles.py`                       | 配置档只差可切换项                                                                              | 6      |
| `test_runtime_catalog.py`                | 家族目录：只有连接数据，能力由端口推导                                                          | 18     |
| `test_capability_gating.py`              | 控制台控件从端口槽位推导                                                                        | 9      |
| `test_console_gating.py`                 | 能力门控不得藏掉首要面板                                                                        | 9      |
| `test_console_presets.py`                | 预设指令文件形状、静态路由、选中落进输入框                                                      | 3      |
| `test_console_world_stop.py`             | 停止场景按钮真的打到端点                                                                        | 2      |
| `test_server_api.py`                     | 全部路由与状态码                                                                                | 56     |
| `test_server_controller.py`              | 状态机、命令队列、换图换模型换 profile                                                          | 68     |
| `test_http_errors.py`                    | 请求失败要说出原因                                                                              | 7      |
| `test_video.py`                          | MJPEG 帧与 JPEG 编码                                                                            | 7      |
| `test_family_router.py`                  | 哪个家族的驱动接哪一次调用                                                                      | 3      |
| `test_argv.py`                           | argv 渲染不泄漏字面量                                                                           | 5      |
| `test_xpra_start_driver.py`              | 起停驱动：端口、PID、`/proc` 全是替身                                                           | 46     |
| `test_map_marker.py`                     | 地图标记谁写谁读                                                                                | 3      |
| `test_subprocess_worker.py`              | 父子协议编解码与一次真实往返                                                                    | 11     |
| `test_mission_child.py`                  | 任务飞行中仍能应答 stop                                                                         | 8      |
| `test_threaded_worker.py`                | 进程内 worker 生命周期                                                                          | 17     |
| `test_mission.py`                        | 任务计划类型与构造                                                                              | 16     |
| `test_task_parser.py`                    | 自然语言 → 计划，严格解码                                                                       | 21     |
| `test_factories.py`                      | 按配置造出的适配器类型正确                                                                      | 15     |
| `test_workflow.py`                       | 工作流骨架与依赖注入                                                                            | 10     |
| `test_basic_motion.py`                   | 数值动作工作流                                                                                  | 10     |
| `test_free_navigation.py`                | 自由导航全链路                                                                                  | 39     |
| `test_visual_goal_parameters.py`         | 视觉目标参数准入                                                                                | 4      |
| `test_visual_goal_anchor.py`             | 锚点新鲜度与失败命名                                                                            | 6      |
| `test_visual_goal_collision_recovery.py` | 碰撞 → 后退 → 清队列 → 重规划                                                                   | 4      |
| `test_visual_goal_trace.py`              | 诊断记录写入                                                                                    | 6      |
| `test_passed_corridor.py`                | 冲过端点的横向走廊                                                                              | 5      |
| `test_motion.py`                         | 运动段规划与执行                                                                                | 8      |
| `test_heading_servo.py`                  | 航向指令律：形状、阻尼、死区                                                                    | 11     |
| `test_position_hold.py`                  | 位置保持                                                                                        | 9      |
| `test_streaming_state.py`                | 流式状态带实测偏航角速度                                                                        | 2      |
| `test_trajectory_hub.py`                 | 剩余航点汇的并发读写                                                                            | 7      |
| `test_frame_hub.py`                      | 最新帧的写入与等待                                                                              | 5      |
| `test_log_buffer.py`                     | 环形日志与游标                                                                                  | 4      |
| `test_batch3_services.py`                | 后台操作与模型调用的受限行为                                                                    | 11     |
| `test_batch3_chat.py`                    | 多模态请求的组装与两种方言                                                                      | 10     |
| `test_model_visual_inputs.py`            | 给模型的深度图准备                                                                              | 4      |
| `test_depth_geometry.py`                 | 反投影与相机基                                                                                  | 12     |
| `test_depth_visualization.py`            | 深度渲染                                                                                        | 4      |
| `test_free_space.py`                     | 深度扇面测距                                                                                    | 10     |
| `test_target_detection.py`               | 检测结果解析与人工检测器                                                                        | 13     |
| `test_capability_targeting.py`           | 目标定位能力                                                                                    | 12     |
| `test_capability_completion.py`          | 完成判定能力                                                                                    | 10     |
| `test_geometric_planner.py`              | 几何兜底规划器                                                                                  | 10     |
| `test_trajectory_parser.py`              | 航点 JSON 解析                                                                                  | 8      |
| `test_trajectory_prompts.py`             | 三种提示词的形状与内容约束                                                                      | 21     |
| `test_task_completion.py`                | 布尔完成判定端点                                                                                | 5      |
| `test_weather_domain.py`                 | 天气参数校验与预设                                                                              | 10     |
| `test_view_catalog.py`                   | 机位目录与循环                                                                                  | 10     |
| `test_projectairsim_connection.py`       | 仿真时钟推进钩子                                                                                | 6      |
| `test_projectairsim_geometry.py`         | NED 几何与位姿换算                                                                              | 16     |
| `test_projectairsim_drive.py`            | 转向与油门模型                                                                                  | 16     |
| `test_projectairsim_motion.py`           | 定时跟踪器                                                                                      | 9      |
| `test_projectairsim_drone.py`            | 无人机适配                                                                                      | 34     |
| `test_projectairsim_rover.py`            | 无人车适配                                                                                      | 16     |
| `test_projectairsim_session.py`          | 双载具会话路由，无实机                                                                          | 34     |
| `test_projectairsim_streaming.py`        | topic 流与配对                                                                                  | 25     |
| `test_streaming_views.py`                | 机位标定，无实机                                                                                | 19     |
| `test_scene_spawns.py`                   | 双车场景同一出生点                                                                              | 4      |
| `test_vehicle_models.py`                 | 六旋翼变体与默认场景同步                                                                        | 2      |
| `test_airsim_connection.py`              | 每次调用都落到设置文件点名的载具                                                                | 8      |
| `test_airsim_port_names.py`              | UE4 适配层用到的属性名真的存在                                                                  | 3      |
| `test_airsim_observation.py`             | 观测符合共享契约                                                                                | 7      |
| `test_airsim_server_vehicle.py`          | 起飞判定不能只看 `landed_state`                                                                 | 5      |
| `test_airsim_vehicle.py`                 | 动作由状态决定，不信会撒谎的 ack                                                                | 9      |
| `test_airsim_session.py`                 | UE4 会话装配成它宣称的端口                                                                      | 13     |
| `test_reference_contracts.py`            | 参考实现的契约对齐                                                                              | 14     |

---

## 14 文档 `docs/`（9 个文件）

| 文件                                       | 做什么                                                 |
| ------------------------------------------ | ------------------------------------------------------ |
| `ARCHITECTURE.md`                          | 分层、模块职责、端口与能力推导、任务管线、进程生命周期 |
| `MOTION_MODEL_AND_CONTINUOUS_EXECUTION.md` | 连续执行、运动模型、载具端口差异，越点卡死的完整因果链 |
| `DEPLOYMENT.md`                            | 机器准备、打包产物、启动步骤                           |
| `AGENT.md`                                 | 给编码助手的项目约定                                   |
| `TODO.md`                                  | 待办                                                   |
| `archive/README.md`                        | 归档说明                                               |
| `archive/decisions.md`                     | 历史决策记录                                           |
| `archive/measurements.md`                  | 历史实测数据                                           |
| `archive/pitfalls.md`                      | 踩过的坑                                               |

---

## 15 附录：包入口的 `__init__.py`（32 个）

这些小文件只做导出与命名空间，没有逻辑。列出它们的行数，方便判断哪个已经不是纯转发了。

| 文件                                                      | 行数 | 做什么                                                          |
| --------------------------------------------------------- | ---- | --------------------------------------------------------------- |
| `planner/__init__.py`                                     | 0    | 空                                                              |
| `planner/adapters/__init__.py`                            | 5    | 包说明：故意没有 eager import，纯模块可以脱离客户端库测试       |
| `planner/adapters/diagnostics/__init__.py`                | 2    | 导出诊断记录                                                    |
| `planner/adapters/http/__init__.py`                       | 0    | 空                                                              |
| `planner/adapters/target_detection/__init__.py`           | 9    | 导出检测器两种实现                                              |
| `planner/adapters/target_detection/prompts/__init__.py`   | 3    | 导出提示词                                                      |
| `planner/adapters/task_completion/__init__.py`            | 2    | 导出完成判定                                                    |
| `planner/adapters/task_completion/prompts/__init__.py`    | 2    | 导出提示词                                                      |
| `planner/adapters/task_parser/__init__.py`                | 2    | 导出解析器                                                      |
| `planner/adapters/task_parser/tasks/__init__.py`          | 73   | 任务解码器注册表与 `builtin_task_codecs()`，**有逻辑**          |
| `planner/adapters/trajectory_planner/__init__.py`         | 5    | 导出规划器                                                      |
| `planner/adapters/trajectory_planner/prompts/__init__.py` | 45   | 把三个提示词模块的全部常量与类汇总导出，**41 个名字**           |
| `planner/application/__init__.py`                         | 2    | 导出 `PlannerApplication`                                       |
| `planner/capabilities/__init__.py`                        | 0    | 空                                                              |
| `planner/capabilities/approach/__init__.py`               | 2    | 导出 `TargetApproach`                                           |
| `planner/capabilities/completion/__init__.py`             | 0    | 空                                                              |
| `planner/capabilities/planning/__init__.py`               | 4    | 导出几何规划器                                                  |
| `planner/capabilities/sliding_window/__init__.py`         | 0    | 空                                                              |
| `planner/capabilities/targeting/__init__.py`              | 4    | 导出定位能力                                                    |
| `planner/capabilities/task_validation/__init__.py`        | 0    | 空                                                              |
| `planner/domain/__init__.py`                              | 14   | 包说明与少量再导出                                              |
| `planner/ports/__init__.py`                               | 6    | 包说明：端口是结构化接口                                        |
| `planner/services/__init__.py`                            | 16   | 导出运动段规划五个函数                                          |
| `planner/services/motion/__init__.py`                     | 3    | 导出运动律                                                      |
| `planner/simulator/__init__.py`                           | 5    | 包说明：进程栈与 worker，不许 import 客户端库                   |
| `planner/simulators/__init__.py`                          | 6    | 包说明：每个引擎一个包，互为平级                                |
| `planner/simulators/airsim/__init__.py`                   | 7    | 导出 UE4 适配                                                   |
| `planner/simulators/projectairsim/__init__.py`            | 43   | 惰性 `__getattr__` 导出，避免 import 时拉进客户端库，**有逻辑** |
| `planner/workflows/__init__.py`                           | 10   | 导出三个工作流                                                  |
| `planner/workflows/basic_motion/__init__.py`              | 2    | 导出                                                            |
| `planner/workflows/free_navigation/__init__.py`           | 5    | 导出                                                            |
| `planner/workflows/visual_goal_navigation/__init__.py`    | 4    | 导出                                                            |

---

## 16 重构时会先动的地方

按"改一处收益多大 / 风险多大"排。

**1. 同名不同类的 `MotionLimits`**

`domain/motion.py` 和 `services/segment_motion.py` 各有一个 `MotionLimits`，字段大半重合，`to_tracking_limits()` 做转换。最近加一个 `passed_lateral_tolerance_m` 就要在两处都改、都要写注释。合成一个，或者让配置层只构造一种。

**2. 三套运动跟踪器**

`services/motion/continuous.py`（105 行，共享的连续跟随）、`simulators/projectairsim/motion.py`（235 行，`SteppedMotionTracker`）、`simulators/airsim/motion_tracker.py`（158 行）。三处都在做"按定时 profile 跟踪世界位姿、速度与偏航反馈、限幅"，差别只是执行器。抽一个共享的跟踪律，仿真器只提供执行器接口。

**3. 两个 500 行以上的巨型文件**

`config.py`（1775 行）混合了 20 多个配置段、目录发现、profile 漂移检查、场景展平。`application/server_controller.py`（1132 行）混合了状态机、命令队列、地图切换、天气、外观、profile、退出。两个都能按现有职责切开而不改任何调用语义。

**4. `services/` 里的纯函数该搬家**

`target_coordinates.py`（34 行）、`trajectory_transform.py`（52 行）、`free_space.py`（281 行）里没有一个类，全是纯函数，被 `capabilities` 和 `adapters` 双向 import。按内容它们是几何能力，放在 `services/` 只是因为历史。搬走之后 `capabilities` 就不必依赖 `services`，分层才真正单向。

**5. `capabilities/sliding_window/` 与 `services/planning_window.py` 交错**

前者有线程（`worker.py` 里的 `threading.Condition`），后者把前者包一层。名字、职责、文件都成对出现，看目录分不清谁主管。合并成一个模块，或者把策略（无状态）与执行（有状态）真正分开。

**6. 提示词文本 1414 行混在 Python 里**

`adapters/trajectory_planner/prompts/` 三个文件：511 + 597 + 306 行，几乎全是字符串拼接。改成独立模板文件加参数渲染，改提示词就不用读代码。

**7. `simulators/projectairsim/streaming_views.py`（543 行）位置不对**

它在适配层里做 xpra 截图标定，而这件事和 `scripts/calibrate_streaming_views.py` 是同一件事。标定属于工具链，运行时应只读标定结果。

**8. 双载具会话两套实现**

`simulators/projectairsim/session.py`（790 行，双车）与 `simulators/airsim/session.py`（317 行，单车）。`mission_assembly` 已经把任务管线抽出来了，会话侧还剩载具生命周期、空闲心跳、天气、机位这四块各自一份。
