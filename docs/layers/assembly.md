> 上级：[项目架构](../ARCHITECTURE.md)
> 同层兄弟：[domain](domain.md)、[ports](ports.md)、[algorithms](algorithms.md)、[runtime](runtime.md)、[workflows](workflows.md)、[adapters](adapters.md)、[simulator](simulator.md)、[simulators](simulators.md)

# 装配层 assembly

## 1 一句话职责

把引擎会话交出的端口对象装成任务管线，并成为后端生命周期（worker、地图、模型、配置、天气、机位）的唯一所有者。

## 2 边界

管这些：

- 每台载具一条管线：`ControlLoop`、`ExecutionService`、三个工作流、三个阶段校验器、解析器、`PlannerApplication` 的构造与接线（`planner/assembly/mission_assembly.py:181-347`）。
- 能力求交：把任务包的 `REQUIRED_PORTS` 与 `SimulatorPorts` 的已填槽位求交，生成 `task_kinds` 与控制台动作位（`planner/assembly/requirements.py:41-118`）。
- 任务执行的顺序编排：解析 → 校验 → 路由 → 逐阶段执行 → 首败即停（`planner/assembly/application.py:25-41`、`planner/assembly/mission_runner.py:13-30`）。
- 后端生命周期与状态机、命令队列、事件泵、换图/换模型/换配置/天气/机位（`planner/assembly/controller/`）。
- 配置到端点的构造：detector、planner、completion checker、trace factory、task parser（`planner/assembly/factories/`）。

不管这些：

- 运动学、控制率、队列、规划窗、模型单飞：[runtime](runtime.md)。
- 任务判定与 `StageResult` 内容：[workflows](workflows.md)。
- HTTP 路由、状态码、JSON 形状：[adapters](adapters.md)。
- 引擎会话与端口实现：[simulators](simulators.md)。
- worker 进程、子进程、仿真栈启停脚本：[simulator](simulator.md)。

## 3 输入与输出

| 入口 | 签名 | 文件 |
| --- | --- | --- |
| `assemble_missions` | `(config, *, inputs: MissionInputs, resources: MissionResources, log) -> dict[str, VehicleArm]` | `planner/assembly/mission_assembly.py:85` |
| `require_application` | `(arm: VehicleArm) -> PlannerApplication`，解析器缺失抛 `ProtocolError` | `planner/assembly/mission_assembly.py:350` |
| `build_mission_common` | `(config, *, planner, detector, log) -> MissionCommon` | `planner/assembly/mission_assembly.py:151` |
| `build_vehicle_arm` | 21 个关键字参数，返回 `VehicleArm` | `planner/assembly/mission_assembly.py:181` |
| `ServerController` | `(worker_factory, settings, *, maps, simulator=None, initial_map=None, vehicle_models=None, restart_attempts=2, worker_shutdown_timeout_s=60.0, autostart_session=True, profiles=(), initial_profile="", session_reset=None, clock=time.monotonic)` | `planner/assembly/controller/__init__.py:43` |
| `PlannerApplication` | `(task_parser, validator, runner, *, parse_call=None)`，方法 `prepare/execute_plan/execute` | `planner/assembly/application.py:11` |
| `task_finish_events` | `(result, task_id) -> (tuple[str, ...], WorkerEvent)` | `planner/assembly/reporting.py:48` |

- 输入契约：`MissionInputs`（`planner/assembly/mission_assembly.py:66-82`）、`MissionResources`（`planner/assembly/mission_assembly.py:50-63`）、`SimulatorPorts`（`planner/simulator/ports.py:42`）、`AppConfig`、`WorkerFactory`（闭包，`planner/server_bootstrap.py:58`）、`SimulatorSupervisor`（`FamilyRouter` 实例，`planner/server_bootstrap.py:110`）、`BackendWorker`。
- 输出契约：`dict[str, VehicleArm]`；`VehicleArm` 字段 `vehicle_id/vehicle/is_drone/execution/application/model_calls/workflows/loop`（`planner/assembly/mission_assembly.py:137-148`）；`PlannerApplication` 的 `prepare/execute_plan`；`ServerController` 的公开方法。
- 构造期异常：`ProtocolError`（`resources.trajectory` 缺失 `planner/assembly/mission_assembly.py:96-100`、解析器缺失 `planner/assembly/mission_assembly.py:352-356`、阶段校验器缺失 `planner/assembly/mission_validator.py:21-36`）、`ValueError`（端点模式未知、`url/model` 缺失、地图/模型/配置名未知）。
- 运行期异常：`BackendUnavailable`（状态不允许，`planner/assembly/controller/state.py:72-75`）、`ValueError`（参数不可用）、`ProtocolError`（协议或任务种类不支持）、`RuntimeError`（监督线程关不掉，`planner/assembly/controller/commands.py:350-353`）。

## 4 对外名字表

| 名字 | 类型 | 用途 | 层外调用点 |
| --- | --- | --- | --- |
| `ServerController` | 类 | 后端生命周期唯一门面 | `planner/server_bootstrap.py:112`；`planner/adapters/http/wiring.py`（`build_app` 入参） |
| `ServerState` | `str` Enum | 五个对外状态 | `planner/adapters/http/wiring.py:19,28`；`planner/adapters/http/server_api.py:19,60,91,99` |
| `BackendUnavailable` | `RuntimeError` 子类 | 状态拒绝的载体 | `planner/adapters/http/wiring.py:19,24,45` |
| `MapSelection` | frozen dataclass | 会话工厂的场景身份 | `planner/server_bootstrap.py:17,59` |
| `MissionInputs` | frozen dataclass | 会话交出的引擎对象 | `planner/simulators/projectairsim/session.py:30,238`；`planner/simulators/airsim/session.py:30,157` |
| `MissionResources` | frozen dataclass | 与会话无关的资源 | `planner/server_bootstrap.py:9,38`；`planner/simulator/mission_child.py:32,228` |
| `VehicleArm` | dataclass | 一台载具的管线 | `planner/simulators/projectairsim/session.py:30,60` |
| `assemble_missions` | 函数 | 端口集合 → 管线字典 | `planner/simulator/registry.py:74,77` |
| `require_application` | 函数 | 拿不到解析器时的拒单 | `planner/simulators/projectairsim/session.py:266`；`planner/simulators/airsim/session.py:200` |
| `task_finish_events` | 函数 | 结果 → 日志行 + FINISHED 事件 | `planner/simulator/worker.py:272`；`planner/simulator/mission_child.py:285` |
| `capabilities_for` | 函数 | `SimulatorPorts` → `Capabilities` | `planner/simulator/registry.py:38-42`（经 registry 转发） |
| `build_detector` | 函数 | 检测器构造 | `planner/server_bootstrap.py:39`；`planner/simulator/mission_child.py:229`；`scripts/smoke_projectairsim_drone.py:268` |
| `build_geometry` | 函数 | 深度几何构造 | `planner/server_bootstrap.py:46`；`planner/simulator/mission_child.py:236`；`scripts/smoke_projectairsim_drone.py:125` |
| `build_planner` | 函数 | 轨迹规划器构造 | `planner/server_bootstrap.py:45`；`planner/simulator/mission_child.py:235`；`scripts/smoke_projectairsim_drone.py:271` |
| `build_free_navigation_planner` | 函数 | 巡航规划器构造 | `scripts/smoke_projectairsim_rover.py:150`；生产路径在 `planner/assembly/mission_assembly.py:158-159` |

`planner/assembly/__init__.py:30-46` 是完整清单。`ServerStatus` 已从层入口移除，测试改从 `planner/assembly/controller/state.py` 取。`MissionCommon`、`build_mission_common`、`build_vehicle_arm`、`capabilities_for` 之外的工厂函数（`build_task_parser`、`build_trace_factory`、`build_completion_checker`、`api_key`）、六个控制器协作者（`LifecycleState`、`CommandChannel`、`SessionSwitcher`、`ModelSwitcher`、`ProfileSwitcher`、`EnvironmentChannel`）、`PlannerApplication`/`MissionValidator`/`MissionRunner`/`WorkflowRouter` 都不在层入口，同层内部按模块路径取。

## 5 关键流程

### 5.1 装配顺序（一次会话）

1. 组合根造共享资源：`planner/server_bootstrap.py:52` 起 `FrameHub`/`TrajectoryHub`，`planner/server_bootstrap.py:55` 调 `build_resources`。
2. `registry.build_session` 按地图的 `simulator` 字段 `import_module` 引擎包并调其 `build_session`：`planner/simulator/registry.py:53-69`。
3. 同一函数内 `session.attach_arms(assemble_missions(...))`：`planner/simulator/registry.py:74-83`。
4. `assemble_missions` 先拒掉缺轨迹 sink 的装配：`planner/assembly/mission_assembly.py:96-100`；再建每服务共享对象 `build_mission_common`：`planner/assembly/mission_assembly.py:101-106`。
5. `build_mission_common` 建免费的巡航规划器与航拍变体、按 `server.planner` 选无人机规划器、建 completion checker 与 trace factory，巡航规划器缺失只记 WARNING：`planner/assembly/mission_assembly.py:158-178`。
6. 逐载具 `build_vehicle_arm`：`planner/assembly/mission_assembly.py:107-122`。每臂的顺序是：`limits` ← `config.visual_goal_navigation.limits`（`planner/assembly/mission_assembly.py:208-209`）；`is_drone = vehicle_id == "drone"`（`planner/assembly/mission_assembly.py:210`）；规划器按机型二选一（`planner/assembly/mission_assembly.py:211-212`）；`ExecutionService`（`planner/assembly/mission_assembly.py:215`）；`ControlLoop`（`planner/assembly/mission_assembly.py:216-225`，碰撞开关取 `config.collision.enabled`）；`GuidedVehicle`（`planner/assembly/mission_assembly.py:226`）；观测源缺省回落到载具（`planner/assembly/mission_assembly.py:230`）；`SynchronizedMotionPlanner`（`planner/assembly/mission_assembly.py:231`）；`BBoxGoalObserver`（`planner/assembly/mission_assembly.py:232-235`）；`TrajectoryValidator`（`planner/assembly/mission_assembly.py:236-240`）。
7. 移动取帧源只在 `moving_source_factory` 非空且 `config.sliding_window.enabled` 时创建，同一个对象交给视觉与巡航两个工作流：`planner/assembly/mission_assembly.py:241-245`、`planner/assembly/mission_assembly.py:262`、`planner/assembly/mission_assembly.py:300`。
8. 视觉工作流无条件构造：`planner/assembly/mission_assembly.py:246-268`，`completion_checker` 来自 `MissionCommon`。
9. 数值动作工作流接 `GuidedVehicle`：`planner/assembly/mission_assembly.py:269-276`。
10. `workflows` 与 `stage_validators` 两张表先登记前两类：`planner/assembly/mission_assembly.py:277-286`；巡航仅在 `free_planner is not None` 时登记，并同时把 `TaskKind.FREE_NAVIGATION` 加进 `terminal_kinds`：`planner/assembly/mission_assembly.py:289-310`。
11. `WorkflowRouter` → `MissionValidator` → `build_task_parser(kinds=router.supported_kinds, aerial=is_drone)`：`planner/assembly/mission_assembly.py:311-313`。
12. 解析器缺失只记 WARNING，`application` 留 `None`：`planner/assembly/mission_assembly.py:314-320`；存在则包一层 `ModelCallService` 并把 `parse_call` 注入 `PlannerApplication`：`planner/assembly/mission_assembly.py:321-337`。
13. 返回 `VehicleArm`，`closed` 是 `workflows` 元组：`planner/assembly/mission_assembly.py:338-347`。

### 5.2 能力求交

1. 允许出现在 `REQUIRED_PORTS` 里的槽位名 = `dataclasses.fields(SimulatorPorts)` 减去 `vehicles`：`planner/assembly/requirements.py:41-43`。
2. `build_table()` 逐个 `import_module` 三个任务包，读 `TASK_KIND`/`REQUIRED_PORTS`，校验类型、槽位名、`TaskKind` 合法性：`planner/assembly/requirements.py:46-73`；结果固化在模块级 `TASK_REQUIREMENTS`：`planner/assembly/requirements.py:76`。
3. 求交公式是"必需槽位全部在已填槽位里"，没有第二个开关：`available_kinds` `planner/assembly/requirements.py:79-84`、`missing_slots` `planner/assembly/requirements.py:87-94`。
4. `capabilities_for(ports)` 把 `ports.implemented()` 换成 `task_kinds`，并按 `ACTION_PORTS` 映射出 `home/models/frames/weather` 四个动作位：`planner/assembly/requirements.py:97-118`。
5. 调用链：`/api/status` 与 `/api/world/options` 走 `planner/adapters/http/wiring.py:60` → `ServerController.capabilities()`（`planner/assembly/controller/__init__.py:208-219`）→ `registry.capabilities_for(entry.simulator)`（`planner/simulator/registry.py:38-42`）→ 本模块的 `capabilities_for`。目录里查不到地图条目时返回 `None`。
6. 控制台的 501 门只挡动作位，不挡任务种类：`planner/adapters/http/wiring.py:63-81`。

### 5.3 任务提交与执行

1. `ServerController.submit` → `CommandChannel.submit`：校验指令非空、载具名在 `settings.vehicles` 内，然后**在锁内**把 `_sequence`+1、`_active`/`_instruction`/`_vehicle` 直接改写、`_state = RUNNING`、投递 `("task", generation, task)`：`planner/assembly/controller/commands.py:40-66`。这一步不发 `activate`。
2. 监督线程 `supervise` 每 50 ms 取一条命令（`planner/assembly/controller/commands.py:281`），`deliver` 校代数后按 kind 分派：`planner/assembly/controller/commands.py:127-175`；`task` 分支只在 `state is RUNNING` 且 `task_id == _active` 时 `worker.submit`：`planner/assembly/controller/commands.py:149-157`。
3. worker 侧 `session.run(task)` 决定 park/hide 另一台载具并 `return_home`，这条逻辑在 [simulators](simulators.md)（`planner/simulators/projectairsim/session.py:256-280`），装配层只保证投递时 `_vehicle` 已是请求值。
4. `require_application(arm)` 拿 `PlannerApplication`：`planner/assembly/mission_assembly.py:350-356`。
5. `PlannerApplication.prepare` = 解析（可走 `parse_call` 的模型单飞）+ `MissionValidator.validate` + `MissionRunner.prepare`：`planner/assembly/application.py:25-33`。
6. `MissionRunner.prepare` 只对计划里出现的 kind 各调一次 `workflow.prepare()`：`planner/assembly/mission_runner.py:13-19`。
7. `MissionRunner.run` 逐 stage 调 `workflow.run`，结果先过 `require_confirmed`，首个 `success=False` 即返回 `MissionResult(False, ...)`：`planner/assembly/mission_runner.py:21-30`。
8. `MissionValidator.validate` 检查协议版本、非空阶段、`stage_id` 唯一、kind 已登记、终结类任务必须在最后，再交任务自己的校验器：`planner/assembly/mission_validator.py:42-68`。
9. 完成事件：`task_finish_events` → `mission_report` 生成日志行与退出码，`cancelled` 前缀额外补一行停止确认：`planner/assembly/reporting.py:6-70`。

### 5.4 事件泵与状态转移

1. `supervise` 每轮调 `c.lifecycle.on_event()`：`planner/assembly/controller/commands.py:298`。
2. `on_event` 最多取 100 条事件，按 kind 分派：LOG、天气/机位（交给 `EnvironmentChannel.on_event`）、READY、FINISHED、ACTIVE、ERROR、CLOSED：`planner/assembly/controller/state.py:127-235`。
3. READY：置 `_worker_ready`、`_state = IDLE`，把载具与机位强制回到默认载具，重放天气：`planner/assembly/controller/state.py:140-173`。
4. FINISHED：只接受 `task_id` 与 `_active` 匹配且在 `RUNNING`/`STOPPING` 的事件，回到 IDLE，必要时补发被延迟的机位：`planner/assembly/controller/state.py:174-199`。
5. ACTIVE：成功则按新载具设机位，失败则按 `_pending_activation` 回滚载具与机位：`planner/assembly/controller/state.py:200-228`。
6. ERROR/CLOSED：转 `commands.failed`，换代、置 CONNECTING、安排重连：`planner/assembly/controller/state.py:229-236`。

### 5.5 换图、换模型、换配置

1. `request_world`：校验地图存在、`_simulator` 非空，只有 DISCONNECTED/IDLE 允许；IDLE 下要求 worker 就绪，同图直接返回；DISCONNECTED 且同图已在跑则只重连不重启栈；否则 `preflight` 后置 CONNECTING 并投递 `("world", ...)`：`planner/assembly/controller/world.py:78-149`。
2. `switch_world`（监督线程）：先 `close_worker(skip_landing=True)` 等 CLOSED，失败则只置 `_world_retry_at` 不碰栈；成功则 `restart_attempts` 次 `apply_simulator`；全失败回 DISCONNECTED 并写 `_sim_error`；成功则更新 `_map_id`/`_selection` 再 `connect()`：`planner/assembly/controller/world.py:196-252`。
3. `request_models`：合并选择、置 CONNECTING、投递 `("models", ...)`：`planner/assembly/controller/models.py:21-77`；`switch_models` 只重建 worker 不动引擎进程：`planner/assembly/controller/models.py:79-98`。
4. `request_profile`：DISCONNECTED 只记名字，IDLE 才重建会话：`planner/assembly/controller/profiles.py:32-84`；`switch_profile` 同模型路径：`planner/assembly/controller/profiles.py:86-101`。
5. 换配置后的配置读取发生在 `worker_factory` 闭包里：先读 `controller.profile`，不同则 `load_config` 并重建资源，再按 family 决定线程 worker 还是子进程 worker：`planner/server_bootstrap.py:58-108`。
6. `stop_world`/`stop_world_command` 与 `stop_simulator` 走同一条"先关 worker 再按所有权停栈"的阶梯：`planner/assembly/controller/world.py:151-174`、`planner/assembly/controller/world.py:254-290`、`planner/assembly/controller/world.py:292-336`。

### 5.6 关闭

`ServerController.close` → `CommandChannel.close`：置 `_state = STOPPING`、`_closing`，join 监督线程（超时 `startup_timeout_s + 8.0`），最后 `world.stop_simulator()`：`planner/assembly/controller/commands.py:336-357`；监督线程的 `finally` 再走一次 `shutdown_worker`：`planner/assembly/controller/commands.py:313-314`、`planner/assembly/controller/commands.py:316-334`。

## 6 失败模式

| 报错类型 | 触发条件 | 调用方处理 |
| --- | --- | --- |
| `ProtocolError` | 轨迹 sink 缺失（`planner/assembly/mission_assembly.py:96-100`）；解析器未配置（`planner/assembly/mission_assembly.py:352-356`）；协议版本不符、阶段为空、`stage_id` 重复、kind 未登记、终结阶段不在末尾（`planner/assembly/mission_validator.py:43-67`）；工作流回包未带确认证据（`planner/assembly/mission_runner.py:24-26`） | 构造期：会话建不起来；提交期：任务被拒，日志写原因；入口层不单独映射，落到 500 前的错误日志 |
| `ValueError` | 未知地图（`planner/assembly/controller/world.py:90-93`）、部署不管理仿真栈（`planner/assembly/controller/world.py:94-95`）、未知载具（`planner/assembly/controller/commands.py:47-51`）、未知模型/配置/目录为空（`planner/assembly/controller/models.py:30-49`、`planner/assembly/controller/profiles.py:43-52`）、端点模式未知或 `url/model` 缺失（`planner/assembly/factories/planners.py:43-73`、`planner/assembly/factories/detectors.py:26-57`、`planner/assembly/factories/completion.py:14-17`）、天气/机位取值非法（`planner/assembly/controller/weather.py:36,52`，校验函数在 `planner.domain`） | 入口层映射 400（`planner/adapters/http/wiring.py:31-40`） |
| `BackendUnavailable` | 状态不是 IDLE 或 worker 未就绪：提交（`planner/assembly/controller/commands.py:52-54`）、`home`（`planner/assembly/controller/commands.py:80-86`）、`activate`（`planner/assembly/controller/commands.py:104-110`）、模型（`planner/assembly/controller/models.py:50-57`）、配置（`planner/assembly/controller/profiles.py:65-70`）、天气（`planner/assembly/controller/weather.py:29-35`）、换图（`planner/assembly/controller/world.py:98-102`）、停场景（`planner/assembly/controller/world.py:162-164`） | 入口层映射 503（CONNECTING/DISCONNECTED）或 409（其余），`detail.state` 带当前态（`planner/adapters/http/wiring.py:24-33`） |
| `RuntimeError` | 监督线程在 `startup_timeout_s + 8.0` 内没退出（`planner/assembly/controller/commands.py:350-353`） | 进程退出路径，`finally` 仍会停栈 |
| `SimulatorNotOwned` / `SimulatorRestartError` | 仿真栈非本服务启动，或启停失败（`planner/assembly/controller/world.py:34-36,51-63,125-128,269-278,325-330`） | 只告警，栈继续运行；`stop_world` 把原因写进 `sim_error` |
| `ValueError`（地图切换） | `preflight` 抛任意异常时包装成 `ValueError("cannot switch maps: ...")`（`planner/assembly/controller/world.py:124-128`） | 入口层 400，状态未改动 |
| 解析器缺失（非异常） | `build_task_parser` 返回 `None`（`planner/assembly/factories/parsers.py:27-28`） | 装配写 WARNING（`planner/assembly/mission_assembly.py:316-320`），提交时由 `require_application` 抛 `ProtocolError`。服务仍可起、可看状态 |

静默路径三处，调用方无法从返回值区分：代数不符的命令被丢弃（`planner/assembly/controller/commands.py:130-133`）；`TaskKind` 未登记的路由（`planner/assembly/workflow_router.py:15-21` 抛 `ProtocolError`，但解析器已被限制在该集合内，实际不可达）；`drain` 未拿到 CLOSED 时 `shutdown_worker` 只写一条 ERROR（`planner/assembly/controller/commands.py:331-334`）。

## 7 测试位置

| 文件 | 用例数 | 覆盖 |
| --- | --- | --- |
| `tests/test_server_controller.py` | 68 | 状态机全路径：断开启动、启动清残留栈、attach、栈启动失败重试、READY/任务/停止/回 IDLE、home、activate（含 ACTIVE 失败回滚与 `_view_pending`）、机位延迟与失败回滚、天气合并与重放、换图（等 CLOSED、重试、失败回 DISCONNECTED）、换模型、换配置、worker 错误换代、重复 FINISHED、`close` 关 worker |
| `tests/test_server_api.py` | 56 | 端点与状态码、`capabilities` 载荷、日志与轨迹接口；用 `FakeController` |
| `tests/test_mission.py` | 16 | 校验器（协议版本、终结阶段、显式支持）、路由、按序执行、首败即停、确认证据与 stage_id 不符、`PlannerApplication` 三条路径、`mission_report` 三种输出 |
| `tests/test_requirements.py` | 6 | 三个包的声明与 `TASK_REQUIREMENTS` 一致、交集三组输入、`SimulatorPorts` 无 `enabled_tasks`、缺槽位清单 |
| `tests/test_runtime_catalog.py` | 18 | 每台仿真器声明的槽位与 `build_ports` 一致、能力矩阵 |
| `tests/test_capability_gating.py` | 9 | 控制台控件从能力位推导 |
| `tests/test_factories.py` | 13 | 四个工厂的端点、提示词、`request_format`、`kinds` 过滤、`api_key` 优先级、trace 配置写入 |
| `tests/test_airsim_session.py` | 13 | `assemble_missions` 在 AirSim 会话上的接线、`moving_source_factory` 调用 |
| `tests/test_layer_entries.py` / `tests/test_layering.py` / `tests/test_public_surface.py` | 3 / 10 / 4 | 跨层 import 只指包入口、任务包互不 import、层入口导出面与文档表一致 |

空白：

- `build_vehicle_arm` 的连续分支（`moving_source_factory` + `sliding_window.enabled`）无断言；`assemble_missions` 只在 AirSim 会话测试里被驱动，`ProjectAirSim` 路径无对应用例。
- `capabilities()` 返回 `None` 时的行为（目录无条目）没有测试；`tests/test_server_api.py` 的 `FakeController` 直接给现成能力位。
- `CommandChannel.deliver` 的代数丢弃、`shutdown_worker` 的未确认 CLOSED 分支、`wait_closed` 超时分支各缺负例。
- `ServerStatus` 不再是层入口名字，只有 `tests/test_server_api.py:259` 构造过它，字段集合变化不会被测试发现。
- `mission_report` 的 `success=True` 但无阶段分支（`planner/assembly/reporting.py:29-43`）未覆盖。

## 8 与其他层的联动

| 层 | 方向 | 数据形态 |
| --- | --- | --- |
| [domain](domain.md) | 上游 | `TaskKind`、`MissionPlan`、`MissionStage`、`StageResult`、`MissionResult`、`Capabilities`、`default_view_for`、`merge_weather`、`validate_view_mode`、`WEATHER_CATALOG`、`MotionLimits`/`ContinuousTrackingSettings`/`SlidingWindowSettings`/`VisualGoalLimits`/`CollisionRecoverySettings` |
| [ports](ports.md) | 上游 | `TaskParser`、`BackendWorker`、`Vehicle`、`ObservationSource`、`MovingObservationSource`（经会话） |
| [algorithms](algorithms.md) | 上游 | `TrajectoryValidator`、`GeometricTargetPlanner`、`DepthGeometry` |
| [runtime](runtime.md) | 上游 | `ControlLoop`、`ExecutionService`、`GuidedVehicle`、`SynchronizedMotionPlanner`、`ModelCallService`、`LogBuffer`、`FrameHub`、`TrajectoryHub` |
| [workflows](workflows.md) | 上游（构造期） | 三个工作流类、三个阶段校验器、`TargetApproach`、`VisualGoalCompletion`、`BBoxGoalObserver`、`TASK_KIND`、`REQUIRED_PORTS`、`require_confirmed` |
| [simulator](simulator.md) | 双向 | 上游：`registry.build_session`、`WorkerFactory`、`FamilyRouter`、`ThreadedMissionWorker`/`SubprocessMissionWorker`、`WorkerEvent`/`WorkerEventKind`、`TaskSubmission`、`SimulatorPorts`、`WorkerEvent` 载荷；下游：`assemble_missions`、`require_application`、`task_finish_events`、`VehicleArm.workflows`/`.model_calls`/`.loop` |
| [simulators](simulators.md) | 双向 | 上游：`SimulatorPorts`、`describe()`、`build_session`、`build_ports`；下游：`MissionInputs`/`MissionResources`/`VehicleArm` 与 `PlannerApplication` |
| [adapters](adapters.md) | 下游 | `ServerController` 全部公开方法、`BackendUnavailable`、`ServerState`、`Capabilities` |

## 9 疑问与争议

- Qdocc13 层：assembly 结论：`task_kinds` 只按端口槽位求交，但管线的实际成员还取决于 `free_planner`、`build_task_parser`、`build_completion_checker` 三个配置，能力位可能广告 `free_navigation` 而该臂的 router 没有它 依据：`planner/assembly/requirements.py:79-84`、`planner/assembly/mission_assembly.py:289-313`、`planner/assembly/mission_assembly.py:316-320` 影响：是（adapters 与 simulator 文档的能力位与拒绝文案）
- Qdocc14 层：assembly 结论：装配层与 `planner.simulator` 存在双向依赖，`requirements.py` 在模块顶层 import `..simulator`，`planner/simulator/registry.py:38-42` 再反向惰性 import 本层 依据：`planner/assembly/requirements.py:20`、`planner/simulator/registry.py:38-42` 影响：是（simulator 文档的层依赖图）
- Qdocc15 层：assembly 结论：`capabilities_for` 在两层各有一个同名入口，层入口导出的是 `requirements` 版，控制器与配置加载实际走 `registry` 版 依据：`planner/assembly/__init__.py:29`、`planner/assembly/requirements.py:97`、`planner/simulator/registry.py:38`、`planner/assembly/controller/__init__.py:219`、`planner/config/vehicles.py:181` 影响：是（simulator 与适配层文档）
- Qdocc16 层：assembly 结论：能力位为空时网关整段放行，地图没有目录条目会让四个动作位全部可点 依据：`planner/assembly/controller/__init__.py:216-219`、`planner/adapters/http/wiring.py:63-68` 影响：是（adapters 文档的门控说明）
- Qdocc17 层：assembly 结论：`submit` 在锁内直接把 `_vehicle` 改成请求值并置 RUNNING，切载具的 park/hide/homing 全在会话 `run` 里，失败时没有回滚路径 依据：`planner/assembly/controller/commands.py:52-66`、`planner/simulators/projectairsim/session.py:256-280` 影响：是（simulator 文档的提交时序）
- Qdocc18 层：assembly 结论：六个协作者都直接读写控制器私有字段，`state.py` 自称"唯一读状态机的地方"，`models.py`/`profiles.py`/`world.py`/`weather.py` 各自直接改 `_state` 依据：`planner/assembly/controller/state.py:1-6`、`planner/assembly/controller/models.py:50-71`、`planner/assembly/controller/world.py:129-145`、`planner/assembly/controller/weather.py:53-64` 影响：否
- Qdocc19 层：assembly 结论：`shutdown_worker` 的告警说"超时未确认"，实际只跑一趟非阻塞 `drain`，与 `wait_closed` 的超时等待不是同一套 依据：`planner/assembly/controller/commands.py:331-334`、`planner/assembly/controller/commands.py:247-274` 影响：否
- Qdocc20 层：assembly 结论：代数不符的命令在 `deliver` 里静默 return，而调用方已经改过状态与 `_pending_activation`，没有日志也没有回执 依据：`planner/assembly/controller/commands.py:127-133` 影响：否
- Qdocc21 层：assembly 结论：`MissionInputs.execution` 由所有 arm 共用，而 `ExecutionService` 绑定单个 `vehicle`；目前只有 AirSim 传该字段且只报一台载具，多载具家族一旦照传就会串车 依据：`planner/assembly/mission_assembly.py:215`、`planner/simulators/airsim/session.py:157-164`、`planner/simulators/projectairsim/session.py:238-241` 影响：是（simulator 文档的 `mission_inputs` 契约）
- Qdocc22 层：assembly 结论：`TASK_REQUIREMENTS` 在 import 时构造并 `import_module` 三个任务包，任何导入 `planner.assembly` 的进程都会加载全部工作流与其依赖 依据：`planner/assembly/requirements.py:76`、`planner/workflows/__init__.py:3-7` 影响：否
- Qdocc23 层：assembly 结论：`PlannerApplication.execute` 会对同一份计划跑两次 `validate` 与两次 `runner.prepare`（`prepare()` 里一次，`execute_plan()` 里再一次） 依据：`planner/assembly/application.py:25-41` 影响：否
- Qdocc24 层：assembly 结论：`ServerStatus` 曾从层入口导出，层外只有测试引用；已按 M2 移除导出，`ServerState` 才是 HTTP 适配层真正需要的类型 依据：`planner/assembly/__init__.py:30-46`、`planner/assembly/controller/state.py:39-69`、`tests/test_server_api.py:16` 影响：否
- Qdocc25 层：assembly 结论：`mission_report` 在 `success=True` 但阶段为空时返回退出码 0 且文案写"未验证"，退出码与文案矛盾 依据：`planner/assembly/reporting.py:29-43` 影响：否

## 10 已知缺口

1. 与 `planner.simulator` 双向依赖，现存两条边。装配取引擎槽位：`planner/assembly/requirements.py:20`；引擎栈取装配的名字：`planner/simulator/registry.py:40` 惰性 import `capabilities_for`，`planner/simulator/worker.py:14` 与 `planner/simulator/mission_child.py:33` 取 `task_finish_events`。风险：把 `SimulatorPorts` 移出 `planner/simulator`，或让 registry 在模块顶层 import 装配层，都会变成导入环。`tests/test_layering.py:155` 只断言"装配是唯一同时 import 任务与仿真器的层"，不覆盖 adapters，因此 `planner/adapters/task_completion/prompts/visual_goal_completion.py:6` 从任务层二级入口取 `require_visual_completion_evidence` 这条边没有测试盯着。
2. `capabilities_for` 有两份位置。派生实现在 `planner/assembly/requirements.py:97`，转发包装在 `planner/simulator/registry.py:38`。层入口导出的只有派生版（`planner/assembly/__init__.py:43`）；控制器与配置加载走转发版（`planner/assembly/controller/__init__.py:219`、`planner/config/vehicles.py:181`）。风险：换入口时容易只改一处。
3. `submit` 在锁内把 `_vehicle` 改成请求值并置 RUNNING，切载具的 park、hide、homing 要到会话 `run` 才发生，失败没有回滚路径（`planner/assembly/controller/commands.py:52-66`、`planner/simulators/projectairsim/session.py:256-280`）。触发条件：提交的目标载具不是当前活动载具，且该载具切换失败。
4. 代数不符的命令在 `deliver` 里静默 return，而调用方已经改过 `_state` 与 `_pending_activation`，没有日志也没有回执（`planner/assembly/controller/commands.py:127-133`）。触发条件：换代发生在同一轮投递与消费之间。
5. `mission_report` 在 `success=True` 且阶段为空时返回退出码 0，文案写"task completion not verified"，退出码与文案矛盾（`planner/assembly/reporting.py:29-43`）。
6. `require_confirmed` 定义在任务层，唯一调用点是装配层（`planner/assembly/mission_runner.py:24`）。放错层的代价是本层新增一个跨层名字，行为没有差异。
7. `PlannerApplication.execute` 对同一份计划跑两次 `validate` 与两次 `runner.prepare`（`planner/assembly/application.py:25-41`）。影响：`prepare` 必须保持幂等，否则第二次会抛。
8. 状态码决定权在适配层，装配层只给异常类型与状态枚举。503 与 409 由 `planner/adapters/http/wiring.py:24-33` 按 `BackendUnavailable.state` 二分，400 由同文件 `:31-40` 映射 `ValueError`，501 由同文件 `:63-81` 的能力门给出。`ServerController.capabilities()` 返回 `None` 时能力门整段跳过（`planner/adapters/http/wiring.py:66-68`），四个动作位全部放行。
9. 能力位与可跑任务不一致。`task_kinds` 只按端口槽位求交（`planner/assembly/requirements.py:79-84`），管线成员还受三个配置约束：巡航规划器是否配置（`planner/assembly/mission_assembly.py:289`）、任务解析器是否配置（`planner/assembly/mission_assembly.py:316`）、完成校验器在构造期直接抛 `ValueError`（`planner/assembly/factories/completion.py:14-17`）。触发条件：巡航规划器未配置而载具槽位满足 `REQUIRED_PORTS`，控制台显示可点，提交后 `MissionValidator` 抛 `unsupported task kind`（`planner/assembly/mission_validator.py:61-63`）。
10. 常量未走配置：巡航航点超时在两处写死 `60.0`（`planner/assembly/mission_assembly.py:264`、`planner/assembly/mission_assembly.py:302`），任务包同名参数的默认值也是 `60.0`（`planner/workflows/free_navigation/workflow.py:68`），三处没有单一出处。
