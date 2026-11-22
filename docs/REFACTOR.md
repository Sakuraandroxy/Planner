# 重构：目标结构与迁移路径

重构把现有 205 个 Python 文件按四类归位：契约、计算、执行、任务，另加装配、入口、引擎实现、端口实现四块外部代码。完成后同一件事只有一处实现，新增任务只写一个包，碰撞退避与滑窗执行是所有任务自动获得的执行层能力。

## 功能目标

- 无人机与无人车在同一场景内运行，直接切换，不重启引擎。切换时停稳、交接控制权、换机位。
- 两类载具都支持三类任务：数值动作、移动到视觉目标附近、无终点巡航。
- 巡航只用视觉，不读车道数据。无人机与无人车走同一条规划路径，差别只在运动律。
- 碰撞退避、滑窗轨迹执行、控制权交接属于执行层，任务层不感知。
- 规划器与模型端点由一段全局配置选择，换模型不改代码，也不改任务包。

## 不做的事

- 不新增任务种类，仍是三类。
- 不引入车道先验。删除现有 `domain/lane_reference.py` 与相关空接线。
- 不支持第三种载具。载具固定为无人机（可横移）与阿克曼无人车（不能横移）。
- 不改 HTTP 契约：路由、字段、状态码保持现状。
- 不改模型 wire 格式：`mission_plan_v1` 与五点轨迹协议不动。

## 分层与目标目录

判据见 `docs/TARGET_ARCHITECTURE.md`，这里只列落地结果。包名保留现有名字，改的是每个包装什么。

```
planner/
  domain/        契约       类型、不变量，无逻辑
  ports/         契约       Protocol 声明
  capabilities/  计算       纯函数与纯策略，同输入同输出
  services/      执行       控制权、队列、滑窗、运动律、汇
  workflows/     任务       三类任务包 + 公共编排
  application/   装配       唯一同时认识任务与引擎的位置
  adapters/      入口与端口实现  http/、task_parser/、target_detection/、trajectory_planner/、task_completion/
  simulator/     进程栈     起停、worker、子进程、地图标记
  simulators/    引擎实现   每个引擎一个包，只有这里 import 客户端库
```

| 包 | 层 | 装什么 | 不许装什么 |
| --- | --- | --- | --- |
| `domain/` | 契约 | 数据类型、常量表、不变量校验 | 任何函数逻辑、import 其他包 |
| `ports/` | 契约 | `Protocol` 声明 | 实现、默认参数逻辑 |
| `capabilities/` | 计算 | 几何、深度、运动律、序列校验、补点策略、几何规划器 | 线程、时钟、设备、任务名 |
| `services/` | 执行 | 控制权、执行器、队列、滑窗、位置保持、碰撞退避、汇 | 任务名、引擎名、模型调用 |
| `workflows/` | 任务 | 每个任务一个包，加公共编排 | 引擎名、其他任务包 |
| `application/` | 装配 | 装配、路由、能力求交、控制器 | 具体任务的阶段逻辑 |
| `adapters/http/` | 入口 | 路由、状态码、MJPEG、静态页 | 状态机、命令队列 |
| `adapters/` 其他子包 | 端口实现 | 解析器、检测器、规划器、完成判定与提示词 | 任务阶段逻辑 |
| `simulator/` | 进程栈 | 起停、探针、worker、子进程 | 任务与模型 |

## 文件级清单

### 契约 `planner/domain/`、`planner/ports/`

| 目标文件 | 现状 | 写什么 |
| --- | --- | --- |
| `domain/motion.py` | 已有，另有同名类在 `services/segment_motion.py` | `MotionLimits` 只留这一份；`ContinuousTrackingSettings`；新增 `CollisionRecoverySettings`（后退距离、恢复次数上限、进度重置距离） |
| `domain/execution.py` | 已有 | 加 `CollisionContact`（对方名称、发生时刻）与 `CollisionState`（是否接触、最近一次接触） |
| `domain/mission.py` | 已有 | `TaskKind` 三类、`MissionStage`、`MissionPlan`、`TaskParameters` 联合 |
| `domain/lane_reference.py` | 已有 | 删除 |
| `ports/vehicle.py` | 已有 | `current_pose`、`execute_segment`、`cancel`、`hover`，`collision_state()` 返回 `CollisionState`，替掉现在靠 `getattr(vehicle, "last_collision")` 的探针 |
| `ports/` 其余 13 个 | 已有 | 不动 |

### 计算 `planner/capabilities/`

| 目标文件 | 现状 | 写什么 |
| --- | --- | --- |
| `capabilities/target_coordinates.py` | 在 `services/` | 水平距离、最近表面、点在机体内判定 |
| `capabilities/trajectory_transform.py` | 在 `services/` | 机体系与世界系互转、参考系增量 |
| `capabilities/free_space.py` | 在 `services/` | 扇面测距与空地高度带过滤 |
| `capabilities/trajectory_validation.py` | 在 `services/` | 航点数、单步位移与偏航上限校验，只读构造参数 |
| `capabilities/depth/` | 现 `targeting/{depth_geometry,depth_surface_sampling,depth_foreground}` | 深度反投影、单点读数、前景过滤 |
| `capabilities/targeting/association.py` | 已有 | 首帧参考不可变、目标绑定与几何绑定 |
| `capabilities/planning/geometric_planner.py` | 已有 | 几何规划器，配置里 `planner.mode: geometric` 时选它 |
| `capabilities/sliding_window/scheduler.py` | 已有 | 补点策略：航点数与预估平滑时长 |
| `capabilities/sliding_window/{planning,worker}.py` | 已有，持线程 | 移到 `services/window/` |
| `capabilities/{approach,completion,task_validation}/*` | 已有 | 移到对应任务包的 `policy.py`，这三个目录消失 |

### 执行 `planner/services/`

| 目标文件 | 现状 | 写什么 |
| --- | --- | --- |
| `services/execution.py` | 已有 262 行 | 控制权（线程 owner 与 flight owner）、检查点、时间预算、停止。`checkpoint()` 是碰撞退避的触发点 |
| `services/collision_recovery.py` | 新建 | 被动技能。见「被动能力的落点」 |
| `services/window/window.py` | 现 `services/planning_window.py` 331 行 | 滑窗编排：begin、submit、poll、commit、pause_for_completion、end |
| `services/window/queue.py` | 现 `services/trajectory_queue.py` 148 行 | 世界航点队列：装载、消费、追加、流式位点 |
| `services/window/worker.py` | 现 `capabilities/sliding_window/worker.py` 77 行 | 惰性模型 worker，带 `threading.Condition` |
| `services/window/inflight.py` | 现 `capabilities/sliding_window/planning.py` 135 行 | 在途请求集合，结果永不自动授权 |
| `services/motion/tracking.py` | 新建 | 世界位姿跟踪律：定时 profile、速度与偏航反馈、限幅。引擎只提供执行器 |
| `services/motion/` 其余 | 已有 | 五次多项式段规划、连续跟随、航向伺服、位置交接 |
| `services/{frame_hub,trajectory_hub,log_buffer,model_calls,observation_snapshot,moving_observations,position_hold,continuous_execution,stream_timeline,background_operation,resource_cleanup}.py` | 已有 | 保持职责不变 |
| `services/{target_coordinates,trajectory_transform,free_space,trajectory_validation}.py` | 已有 | 移到 `capabilities/`，原文件删除 |

### 任务 `planner/workflows/`

| 目标文件 | 现状 | 写什么 |
| --- | --- | --- |
| `workflows/common/loop.py` | 新建 | 主循环骨架：取观测 → 问策略 → 执行 → 检查点。不含任务名 |
| `workflows/common/evidence.py` | 新建 | `StageResult`、`StageSummary`、`CompletionBasis` 的构造与校验，吸收 `mission_runner.py` 里的结果检查 |
| `workflows/common/budget.py` | 新建 | 时限、模型调用计数、停止请求、部分执行的剩余运动 |
| `workflows/common/streaming.py` | 新建 | 连续执行的编排：何时取帧、何时补点、何时暂停。实际下发速度的是 `services/continuous_execution.py` |
| `workflows/basic_motion/workflow.py` | 已有 161 行 | 数值动作：构造运动段、执行、报实测 |
| `workflows/basic_motion/policy.py` | 现 `capabilities/task_validation/basic_motion.py` | 参数校验与完成证据形状 |
| `workflows/visual_goal_navigation/workflow.py` | 已有 414 行 | 阶段编排，走 `common/loop.py` |
| `workflows/visual_goal_navigation/targeting.py` | 现 `capabilities/targeting/visual_goal_refresh.py` 与 `bbox_goal_observation.py` | 目标刷新与检测，持线程的部分调 `services/` |
| `workflows/visual_goal_navigation/completion.py` | 现 `capabilities/completion/*` 5 个文件 | 半径暂停、确认计数、语义校验、完成证据 |
| `workflows/visual_goal_navigation/policy.py` | 现 `capabilities/task_validation/visual_goal_navigation.py` 与 `approach/` | 参数准入、到达评估 |
| `workflows/free_navigation/workflow.py` | 已有 403 行 | 无终点巡航的阶段编排 |
| `workflows/free_navigation/cruise.py` | 现 `free_navigation/continuous.py` 444 行 | 连续跟随循环，碰撞处理交给执行层 |
| `workflows/free_navigation/policy.py` | 现 `capabilities/task_validation/free_navigation.py` | 参数准入、停滞判定 |

每个任务包的 `__init__.py` 导出两个常量：`TASK_KIND` 与 `REQUIRED_PORTS`（必需槽位名）。装配层只读这两个常量，不认识任务内部，新增任务因此不改装配逻辑。

### 装配 `planner/application/`

| 目标文件 | 现状 | 写什么 |
| --- | --- | --- |
| `application/mission_assembly.py` | 已有 359 行 | 唯一装配：端口 → 任务管线。删掉 `lane_reference_provider=None` 参数 |
| `application/requirements.py` | 新建，从 `simulator/ports.py` 迁出 | 汇总各任务包声明的必需槽位，得出可跑任务集合 |
| `application/controller/state.py` | 现 `server_controller.py` 1132 行的状态机部分 | 状态迁移、事件泵、提交与停止的准入 |
| `application/controller/commands.py` | 同上，命令队列部分 | 命令队列与 worker 投递 |
| `application/controller/world.py` | 同上，换图部分 | 选图、起停引擎、换图后的会话重建 |
| `application/controller/models.py` | 同上，外观部分 | 车型清单与切换 |
| `application/controller/weather.py` | 同上，天气部分 | 天气参数合并与下发 |
| `application/controller/profiles.py` | 同上，配置档部分 | 配置档列举与切换 |
| `application/factories/` | 现 `factories.py` 250 行 | `planners.py`、`detectors.py`、`parsers.py`、`completion.py` 四个文件，同一套解析规则 |
| `application/{application,mission_runner,mission_validator,workflow_router}.py` | 已有 | 解析 → 校验 → 路由 → 执行，保持 |

### 入口与端口实现 `planner/adapters/`

| 目标文件 | 现状 | 写什么 |
| --- | --- | --- |
| `adapters/http/server_api.py` | 已有 511 行 | 路由、状态码、能力门，改动限于装配接口 |
| `adapters/task_parser/` | 已有 | 指令解析与三类任务的解码器，按载具给 `aerial` 开关 |
| `adapters/target_detection/`、`adapters/task_completion/` | 已有 | 目标检测与完成判定端点 |
| `adapters/trajectory_planner/{qwen_vl_planner,response_parser}.py` | 已有 | 视觉规划器与响应解析 |
| `adapters/trajectory_planner/prompts/templates/*.md` | 现三个提示词文件共 1411 行 | 提示词正文外置为模板，`render.py` 填实测数据与规则文本 |
| `adapters/diagnostics/` | 已有 | 任务轨迹留档 |

### 引擎 `planner/simulator/`、`planner/simulators/`

| 目标文件 | 现状 | 写什么 |
| --- | --- | --- |
| `simulator/ports.py` | 已有 200 行 | 只留 `SimulatorPorts` 槽位与能力推导，`TASK_REQUIREMENTS` 迁走 |
| `simulator/{registry,router,worker,subprocess_worker,mission_child,processes,argv,marker,xpra_start,telemetry_relay}.py` | 已有 | 起停、探针、两套 worker、地图标记，协议不变 |
| `simulators/projectairsim/session.py` | 已有 790 行 | 只留会话生命周期与端口装配 |
| `simulators/projectairsim/vehicles.py` | 拆分自 `session.py` | 双载具持有、`activate` 切换、停稳与位置保持交接 |
| `simulators/projectairsim/observation.py` | 拆分自 `session.py` | 观测源、深度解码、运动中取帧 |
| `simulators/projectairsim/{rover,drone,motion,streaming_views,weather,camera_model}.py` | 已有 | 载具实现、执行器、机位、天气 |
| `simulators/airsim/*` | 已有 | UE4 侧实现，跑在子进程里，协议不变 |

### 配置数据 `config/`

| 目标位置 | 现状 | 写什么 |
| --- | --- | --- |
| `config/core/<profile>/base.yaml` 的 `motion` 段 | 现碰撞参数在 `free_navigation` 段 | 新增 `collision` 子段：后退距离、恢复次数上限、进度重置距离，三类任务共用 |
| `config/core/<profile>/base.yaml` 的 `trajectory_planner` 段 | 已有，全局一段 | 不动。新增 `mode` 字段区分视觉规划器与几何规划器 |
| `config/core/<profile>/tasks/*.yaml` | 已有 | 只留三类任务的规则文件，无能力开关 |
| `config/maps.yaml`、`config/simulators.yaml` | 已有 | 不动，不增加车道字段 |

## 三类任务的最终行为

| 任务 | 无人机 | 无人车 | 必需端口 | 结束条件 |
| --- | --- | --- | --- | --- |
| 数值动作 | 前后、转向、升降 | 前后、转向 | `vehicle` | 走完并按实测报告 |
| 目标接近 | 前后、转向、升降 | 前后、转向 | `vehicle`、`observation` | 停稳且语义确认 |
| 无终点巡航 | 纯视觉沿路 | 纯视觉沿路 | `vehicle`、`observation`、`streaming` | 用户停止或本地检查失败 |

有 `streaming` 端口时，目标接近与巡航都按控制率连续执行；没有该端口时退化为停稳后分批执行，判定逻辑不变。

载具差异只有两处，落在解码层与计算层，任务层不分支：

- 升降。地面解码器没有竖直字段，aerial 解码器多一个 `up_m`。下发竖直位移给车在解码处就被拒绝。
- 横移。阿克曼车不能横移。横向数值指令在解码处拒绝，不换算成转弯。要改成换算，动 `adapters/task_parser/tasks/basic_motion.py` 的规则与 `services/motion/` 的分支，任务包不动。

## 被动能力的落点

四件事所有任务都获得，任务包里不出现相关代码。

**控制权。** `services/execution.py`。任一时刻每台载具一个 owner，owner 是线程标识；取用、释放、停止都校验 owner。观测采集与位置保持临时取用控制权。破坏表现为后台循环在暂停期间继续发速度指令。

**碰撞退避。** `services/collision_recovery.py`，由 `ExecutionService.checkpoint()` 触发。接触发生时依次做：记录对方名称与位姿、用运动律后退（距离按次数递增）、清空队列、把控制权交还任务、继续执行。恢复次数超上限才抛 `ExecutionError`。参数在 `domain/motion.py` 的 `CollisionRecoverySettings`，值来自 `config`。开车与飞行共用一套，因为退避只依赖载具端口。

**滑窗执行。** `services/window/` 持有状态，`capabilities/sliding_window/scheduler.py` 提供补点策略。任务层只调 `begin / submit / poll / commit / pause_for_completion / end`，看不到线程与队列。

**时间与停止。** `services/execution.py` 的 deadline 加 `workflows/common/budget.py` 的调用计数。停止请求从入口层到 owner 只有一条路：`request_stop()` 置事件，owner 在检查点抛 `ExecutionError`。

## 要删除的东西

| 删除对象 | 位置 | 理由 |
| --- | --- | --- |
| 车道先验空接线 | `domain/lane_reference.py`、`prompts/free_navigation.py` 的 `format_lane_reference`、两处 `lane_reference_provider` 参数、`mission_assembly.py:306` 的 `None` | 决定纯视觉 |
| 第二份 `MotionLimits` | `services/segment_motion.py` | 同名不同类，一个事实两处定义 |
| 两个引擎各自的跟踪律 | `simulators/projectairsim/motion.py`、`simulators/airsim/motion_tracker.py` 的律部分 | 律进 `services/motion/tracking.py`，引擎只留执行器 |
| 任务专属判定在计算层 | `capabilities/{approach,completion,task_validation}/` | 移到任务包 `policy.py` |
| 线程在计算层 | `capabilities/sliding_window/{planning,worker}.py` | 移到 `services/window/` |
| `capabilities` 反向依赖 `services` | 10 处 import | 纯函数搬走后自然消失 |
| 提示词正文混在 Python | `adapters/trajectory_planner/prompts/` 三个文件 1411 行 | 外置模板 |
| 俯视任务整条线 | 已在 `8273ef7` 删除 | 未接线 |

## 迁移阶段

每阶段独立可发布，出问题退回上一阶段。前两阶段只搬位置与下沉，不改对外行为。

| 阶段 | 做什么 | 验收 | 风险 |
| --- | --- | --- | --- |
| 0 | 删除俯视任务 | 已完成 | 无 |
| 1 | 位置归位：纯函数进 `capabilities/`，`MotionLimits` 合一份，滑窗线程进 `services/window/`，任务判定进任务包 `policy.py`，删车道先验 | `pytest tests/ -q` 全绿；新增分层测试通过；UE5 三类任务各跑一次 | import 面大，按包分批提交 |
| 2 | 被动安全下沉：加 `CollisionState` 端口、`services/collision_recovery.py`、配置段；删 `free_navigation` 里的退避与 `visual_goal` 的碰撞预检分支 | 单测用假载具模拟接触，三类任务都触发退避；恢复次数超上限时任务失败并报原因 | 改变失败语义，需要人工回归 |
| 3 | 公共编排：抽 `workflows/common/`，合并两个连续执行实现，收敛目标接近的批处理与连续分支 | 行为不变，`telemetry` 输出对比一致；删除镜像代码 | 合并循环最容易引入时序差 |
| 4 | 装配与配置拆分：`controller/`、`factories/`、`config/`；`REQUIRED_PORTS` 下沉到任务包，装配层只汇总 | `/api/status` 与 `/api/world/options` 输出与重构前逐字段一致 | 状态机拆分要保事件顺序 |
| 5 | 引擎侧收敛：跟踪律合并、`session.py` 拆分、提示词外置 | UE5 跑完三类任务；UE4 跑数值动作与目标接近，巡航补测后加入 `enabled_tasks`；提示词渲染结果与旧版逐字一致 | 需要引擎环境，最慢 |

## 验收标准

自动检查，全部进 `tests/`：

- `tests/test_layering.py` 五条规则：`domain/` 与 `ports/` 不 import 其他包；`capabilities/` 不 import `services/`、`workflows/`、`application/`；`services/` 不 import `workflows/`、`application/`；任务包之间不互相 import；只有 `application/` 同时 import 任务与 `simulators/`。
- `tests/test_architecture.py` 现有规则保留：只有 `simulators/<name>/` import 客户端库；共享层不 import `planner.simulators`；能力位由槽位推导。
- 能力矩阵：`tests/test_capability_gating.py` 与 `tests/test_runtime_catalog.py` 守住任务与控件的推导结果。
- `pytest tests/ -q` 全绿，不减少用例数。

人工验收，UE5 一条链路走完：

1. 启动场景，选无人机，跑数值动作。
2. 不重启场景，`/api/vehicle` 切到无人车，跑数值动作与目标接近。
3. 无人车跑无终点巡航，中途制造一次碰撞，确认后退、清队列、继续。
4. 切回无人机，确认机位与控制权回到无人机。
5. 换一张图，确认能力位与控件跟着变。
