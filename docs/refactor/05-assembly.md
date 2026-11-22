# 装配层 `planner/assembly/`：文件清单与内容

装配层是唯一同时认识任务包与引擎实现的位置：它把端口接成任务管线，把任务包声明的必需端口与仿真器实际填上的槽位求交，持有后端状态机与命令队列，并按配置造外部端点。状态码映射不在本层，留在 `planner/adapters/http/`。

## 1. 职责与文件清单

一句话：装配层决定本次跑什么、能不能跑、谁来跑，并在构造期把缺线的地方直接报错。

行数为估计值，"从哪来"里的行号是现状文件的行号，已核对。目标文件不存在的名字在当前代码里没有对应物，属于本次新建。

### 1.1 装配顶层

| 文件 | 类与函数签名 | 行 | 从哪来 |
| --- | --- | --- | --- |
| `assembly/__init__.py` | 导出 12 个跨层名字：`assemble_missions`、`MissionInputs`、`MissionResources`、`VehicleArm`、`require_application`、`task_finish_events`、`ServerController`、`SwitchRefused`、`StageValidator`、`WorkerFactory`、`SimulatorSupervisor`、`build_resources` | 30 | 新写，转发现同级模块。跨层 import 只能指向这个文件（裁决 R19a）；名字集由 §8 削面复核定 |
| `assembly/mission_assembly.py` | `MissionResources`、`MissionInputs`、`VehicleArm`、`assemble_missions(config, *, inputs, resources, log) -> dict[str, VehicleArm]`、`build_vehicle_arm(config, *, common, vehicle, vehicle_id, detector, geometry, telemetry, moving_source_factory, execution, observation_source, log) -> VehicleArm`、`require_application(arm) -> PlannerApplication` | 320 | `application/mission_assembly.py`（359 行）。删 `lane_reference_provider=None`（:306）；工作流集合计算（:281-290）移进 `requirements.py`，可跑任务只由槽位交集推导（裁决 R18）；新增 `Vehicle` 槽填充：`GuidedVehicle(streaming, loop)`（裁决 R1） |
| `assembly/requirements.py` | `EXECUTABLE_PACKAGES`、`PortsView`、`declared_requirements() -> dict[TaskKind, tuple[str, ...]]`、`ports_view(*, slots, vehicles) -> PortsView`、`missing_slots(view, kind) -> tuple[str, ...]`、`available_kinds(view) -> frozenset[TaskKind]`、私有 `capabilities_for(view, *, view_modes, weather_fields, weather_presets) -> Capabilities`、`assert_constructible(view, vehicle_id) -> None` | 120 | 新建。从一张显式元组 `EXECUTABLE_PACKAGES` 逐个 import 任务包 `__init__`，读 `TASK_KIND` 与 `REQUIRED_PORTS`（与 `docs/refactor/04-workflows.md` 同名）；吸收 `simulator/ports.py:44` 的 `TASK_REQUIREMENTS` 与 `:85` 的 `available_tasks()`；没有 `enabled_tasks` 参数（裁决 R18） |
| `assembly/application.py` | `PlannerApplication.__init__(task_parser, validator, runner, *, parse_call)`、`prepare(instruction)`、`execute_plan(plan)`、`execute(instruction)` | 41 | `application/application.py`，不拆 |
| `assembly/mission_runner.py` | `MissionRunner.__init__(router)`、`prepare(plan)`、`run(plan) -> MissionResult` | 48 | `application/mission_runner.py`，不拆 |
| `assembly/mission_validator.py` | `StageValidator`、`MissionValidator.__init__(validators, *, terminal_kinds)`、`supported_kinds`、`validate(plan)` | 68 | `application/mission_validator.py`，不拆 |
| `assembly/workflow_router.py` | `WorkflowRouter.__init__(workflows)`、`supported_kinds`、`workflow_for(stage)` | 21 | `application/workflow_router.py`，不拆 |
| `assembly/reporting.py` | `task_finish_events(result, task_id) -> tuple[tuple[str, ...], WorkerEvent]`、私有 `_report(result)` | 70 | `application/reporting.py`。`simulator/worker.py:14` 与 `simulator/mission_child.py:37` 继续 import `task_finish_events`，见 §9 第 4 条 |

### 1.2 控制器子包

公开入口是 `assembly/controller/__init__.py::ServerController`，构造签名与 `application/server_controller.py:44` 逐字保持（新增仅 `handoff_timeout_s` 落到 `ServerSettings`），`simulator` 与 `worker_factory` 两个参数的类型换成装配层自己声明的 `SimulatorSupervisor` 与 `WorkerFactory` Protocol（裁决 R19b）。现有 `tests/test_server_controller.py` 的 68 个用例只改 import 路径。

| 文件 | 类与函数签名 | 行 | 从哪来（现文件行号） |
| --- | --- | --- | --- |
| `assembly/controller/__init__.py` | `ServerController`：门面，持四个协作者；`state`、`status`、`logs`、`submit`、`stop`、`home`、`activate`、`request_weather`、`request_view`、`request_world`、`stop_world`、`request_models`、`request_profile`、`start`、`close`、`maps`、`vehicle_models`、`profiles`、`profile`、`weather_catalog`、`simulator_owned`、`capabilities()`；`SwitchRefused(Exception)` 带 `vehicle` 与 `reason`；`WorkerFactory` 与 `SimulatorSupervisor` 两个 Protocol 也声明在这里（裁决 R19b） | 175 | 新建门面；`__init__` 44-143 拆成字段分派 |
| `assembly/controller/state.py` | `LifecycleState`：`state`、`status`、`admit_idle(operation)`、`admit_running_or_idle()`、`transition(state, reason)`、`on_event(event)`（事件泵）、`reset_session_state()` | 260 | `state` 145、`status` 150、`_log` 591、`_events` 892、`_reset_session_state` 830 |
| `assembly/controller/commands.py` | `CommandChannel`：`put(kind, value)`、`deliver(command)`（`_command` 648 的分派）、`connect()` 607、`failed(reason)` 625、`close_worker(*, skip_landing)` 837、`wait_closed(worker)` 863、`drain(worker)` 875、`supervise()` 1015、`shutdown_worker()` 1054、`close()` 1073；`Handoff`：`request(target, *, blocking, timeout_s)`、`resolve(event)` | 310 | `_command` 648、`_connect` 607、`_failed_worker` 625、`_close_worker` 837、`_wait_closed` 863、`_drain` 875、`_run` 1015、`_shutdown_worker` 1054、`close` 1073；`Handoff` 从 `submit` 266 与 `_events` 的 ACTIVE 分支抽出 |
| `assembly/controller/world.py` | `SessionSwitcher`：`maps`、`simulator_owned`、`start()`、`stop_leftover_simulator()`、`request_world(map_id)`、`stop_world()`、`selection_for(map_id)`、`apply_simulator(entry, restart)`、`switch_world(entry)`、`stop_world_command()`、`stop_simulator()`、`vehicle_models`、`request_models(updates)`、`switch_models()`、`profile`、`profiles`、`request_profile(name)`、`switch_profile()` | 380 | `maps` 196、`simulator_owned` 200、`start` 215、`_stop_leftover_simulator` 229、`request_world` 386、`stop_world` 458、`_selection_for` 595、`_apply_simulator` 692、`_switch_world` 699、`_stop_world` 756、`_stop_simulator` 1091；并入 `vehicle_models` 177、`request_models` 482、`_switch_models` 793、`profile` 182、`profiles` 192、`request_profile` 539、`_switch_profile` 813。地图、外观、配置三条命令共用 `close_worker → 必要时动栈 → connect` 骨架（§8） |
| `assembly/controller/weather.py` | `EnvironmentChannel`：`weather_catalog`、`request_weather(values)`、`request_view(mode)`、`on_event(event)`（WEATHER/VIEW 两支）、`reapply(worker)` | 115 | `weather_catalog` 211、`request_weather` 347、`request_view` 370，加 `_events` 里的 WEATHER（`drain` 875 与 `_events` 892）与 VIEW 分支。天气与机位同生命周期：跨会话保留、READY 后重放、RUNNING 中延迟，合并成一个协作者（裁决 R12），见 §9 第 6 条 |
| `assembly/controller/profiles.py` | `ProfileSwitcher`：`profile`、`profiles`、`request_profile(name)`、`switch_profile()`、`reset()` | 100 | `profile` 182、`profiles` 192、`request_profile` 539、`_switch_profile` 813 |

### 1.3 工厂子包

| 文件 | 类与函数签名 | 行 | 从哪来 |
| --- | --- | --- | --- |
| `assembly/factories/__init__.py` | 只导出 `build_resources` | 12 | 新写 |
| `assembly/factories/resources.py` | `build_resources(config, *, frame_sink, trajectory) -> MissionResources`；私有 `_detector(config)`、`_geometry(config)`、`_completion_checker(config)` | 130 | 合并 `application/factories.py:59-99`、`:150-166`、`:203-210`，加 `server_bootstrap.py:29-49` 与 `simulator/mission_child.py:232-252` 两份资源构造 |
| `assembly/factories/planners.py` | `build_planner(config, mode, *, aerial=False, purpose="visual_goal")`；私有 `api_key(settings, label)`、`_trajectory_planner(config, prompt_builder)` | 125 | `application/factories.py:38-56`、`:102-148`；`build_free_navigation_planner` 合并进来 |
| `assembly/factories/parsers.py` | `build_task_parser(config, *, kinds, aerial)`、`build_trace_factory(config, *, detector, planner)` | 110 | `application/factories.py:169-200`、`:213-251` |

### 1.4 配置包 `planner/config/`

数据目录仍是仓库根 `config/`。`_project_root()` 现在是 `Path(__file__).resolve().parent.parent`（`config.py:844`，`__file__` 为 `planner/config.py`），文件搬进 `planner/config/profiles.py` 后要改成 `parents[2]`。

| 文件 | 内容 | 行 | 从哪来 |
| --- | --- | --- | --- |
| `config/__init__.py` | 再导出全部公开名字，外部 import 路径不变；跨层只能 import `planner.config`（裁决 R19a） | 90 | 新写；保住 `tests/test_config.py`、`scripts/*.py`、`simulator/mission_child.py:38` 的现有 import |
| `config/profiles.py` | `profile_root()`、`default_profile()`、`default_config_path()`、`profile_of(path)`、`profile_config_path(profile)`、`available_profiles()`、`shared_or_profile(path, rel)`、`ProfileInfo`、`PROFILE_SWITCHABLE_SECTIONS`、`profile_info(raw, path)`、`profile_raw(profile)`、`profile_drift(reference)`、`profile_catalog()` | 200 | `config.py:775-788`、`:843-889`、`:891-977`、`:979-1001` |
| `config/sections.py` | `ProjectAirSimSettings`、`DroneSettings`、`CameraSettings`、`TargetDetectionSettings`、`TrajectoryPlannerSettings`、`DepthSettings`、`TaskCompletionSettings`、`TaskParserSettings`、`VisualGoalNavigationSettings`、`FreeNavigationSettings`、`ServerSettings`、`SimulatorSettings`、`build_camera_settings(raw)`、`triple(values, *, where)` | 400 | `config.py:23-454`、`:738-774`、`:1312-1363`。`ServerSettings` 新增 `handoff_timeout_s: float = 15.0` |
| `config/maps.py` | `MapScene`、`MapEntry`、`MapCatalog`、`VehicleModelOption`、`VehicleModelGroup`、`VehicleModelCatalog`、`default_vehicle_models()`、`load_map_catalog(...)`、`load_vehicle_models(raw, *, maps, vehicles, families)`、`family_vehicle_models(families)`、`parse_map_scene(...)`、`flatten_scene_defaults(...)`、`binary_root(...)`、`resolve_binary(...)` | 500 | `config.py:455-737`、`:600-702`、`:1003-1218`、`:1219-1311`、`:1364-1382`。外观模型与地图的场景变体是同一份数据，合并一文件；删掉 `load_vehicle_models` 里的 `from .simulator import registry`（`:1280-1286`），见 §4.3 |
| `config/loader.py` | `AppConfig`、`PROFILE_SECTIONS`、`section(raw, key)`、`load_config(path) -> AppConfig`、`load_completion_rules(config_path, filename)` | 420 | `config.py:789-835`、`:836-842`、`:1383-1774` |

### 1.5 组合根

`planner/server_bootstrap.py` 留在原位，不属于六层中的任何一层，是进程组合根：`build_resources(config, *, frames, trajectory) -> MissionResources`、`build_server(config) -> FastApi`。它是唯一同时 import `assembly` 与 `adapters/http` 的模块（`scripts/serve_planner.py:41` 调 `build_server`），`adapters/http/` 反过来不 import 它。

## 2. 数据流与时序

### 2.1 会话构造与能力求交

```
build_server(config)                                    server_bootstrap.py
  ├─ build_resources → MissionResources(detector, planner, geometry, trajectory, frame_sink)
  └─ ServerController(worker_factory, settings, maps=..., simulator=<SimulatorSupervisor>, ...)   构造参数用本层 Protocol（裁决 R19b）
        │  worker_factory(selection)        组合根内的闭包
        ▼
registry.build_session(simulator, config, map_entry, family, resources, log)   registry.py:41
  ├─ import simulators/<name>.runtime，调其 build_session
  ├─ session.mission_inputs() → MissionInputs(vehicles, execution, observation_source, moving_source_factory)
  ├─ assemble_missions(config, inputs=..., resources=..., log=...)              mission_assembly.py:86
  │    └─ 每台载具：build_vehicle_arm(...)                mission_assembly.py:182
  │          ├─ 按 is_drone 选规划器与提示词（无共享容器，§8.2）
  │          ├─ Vehicle 槽 ← GuidedVehicle(streaming, loop)    （裁决 R1）
  │          ├─ view = requirements.ports_view(slots=该载具实际拿到的槽位)
  │          ├─ kinds = requirements.available_kinds(view)      ← 能力求交唯一计算点
  │          ├─ 按 kinds 建 workflows / stage_validators / parser codecs
  │          └─ requirements.assert_constructible(view, vehicle_id)
  └─ session.attach_arms(arms)，缺一台载具的 arm 即 ProtocolError      projectairsim/session.py:276
```

求交公式：`available_kinds(view) = {kind for kind, ports in declared_requirements() if view.slot(name) is not None for name in ports}`。`declared_requirements()` 读每个任务包的 `REQUIRED_PORTS`（`docs/refactor/00-boundaries.md` 第四节），这是任务侧唯一的事实来源；`SimulatorPorts` 只回答"这个槽位有没有对象"。可跑任务不叠加任何白名单（裁决 R18）：UE4 的 `streaming` 槽有值，无终点巡航对它自动开放。

### 2.2 任务提交与切载具（提交期阻塞路径）

```
POST /api/tasks                                          server_api.py:211
 → ServerController.submit(instruction, vehicle)          controller/state.py 准入
     closing? state is IDLE? vehicle in self._vehicles?  否则 BackendUnavailable / ValueError
     vehicle != self._vehicle
       └─ Handoff.request(target, blocking=True, timeout_s=server.handoff_timeout_s)
            put ("activate", generation, target)         controller/commands.py
            ┌ 等待 ACTIVE 事件，期间不持 state 锁 ┐
            supervisor 线程 → CommandChannel.deliver → worker.activate(target)     simulator/worker.py:60
            worker 循环 → session.activate_vehicle(target)                          projectairsim/session.py:386
                 1  _park(other)       无人机 park / 无人车 hover                 projectairsim/session.py:658
                 2  _hide(other)       传送到 1 km 上方并钉住                     projectairsim/session.py:574
                 3  _recover(target)   清掉该载具的隐藏记录                       projectairsim/session.py:647
                 4  active = target
                 5  return_home(target)  传送到出生点                             projectairsim/session.py:356
                 6  set_view_mode(default_view_for(target))                      projectairsim/session.py:487
                 7  任一步异常：active 回滚为 previous，重新 hide target，上抛
            worker 发 ACTIVE(success, vehicle)                                        simulator/worker.py:189/198
            事件泵 KeyedHandler.on_event → Handoff.resolve
       ACTIVE 成功 → self._vehicle = target，view_mode = default_view_for(target)，
                     _view_pending = False，capabilities 重算，Handoff 放行
       ACTIVE 失败 / CLOSED / ERROR / 超时 → Handoff 抛 SwitchRefused(target, reason)
                     self._vehicle 保持原值（提交期不做乐观改写）
     sequence += 1
     TaskSubmission(task_id=f"task_{sequence}", instruction, target)   domain/server.py:44
     state = RUNNING
     put ("task", generation, task)
 → worker 循环 → session.run(task)                        projectairsim/session.py:286
     vehicle_id == self._active_vehicle，所以 park/hide 分支（:299-313）不走
     arm.execution.bind_hold_checkpoint(stop_event)        services/execution.py:43
     application.prepare(instruction) → execute_plan(plan)
 → FINISHED(task_id, message, success)                    simulator/worker.py:272 / mission_child.py:289
 → 事件泵：task_id 与 active 匹配且 state ∈ {RUNNING, STOPPING} 才接受（:943-947）
```

顺序不变量三条：ACTIVE 必须在 task 投递之前被本进程消费；命令里的 `generation` 必须等于当前代数；FINISHED 的 `task_id` 必须等于 `self._active`。三条都写成断言，见 §6。

### 2.3 空闲期切载具（不阻塞）

`POST /api/vehicle` → `ServerController.activate`（现 `:317-346`）：乐观改写 `_vehicle`、`_view_mode`、`_view_pending`，快照 `_pending_activation`，投递 `("activate", generation, target)`，立即返 202。ACTIVE 成功则确认 view（`:972-981`），失败则按快照回滚（`:982-984`）。与 2.2 的差别只有两处：不阻塞 HTTP 线程，失败不改状态码而写 `WARNING` 日志并回滚。

### 2.4 换图、换模型、换配置

三者共用一条骨架，差别在会话重建后要不要动引擎进程：

| 命令 | 序号 | 动作 | 结束时状态 |
| --- | --- | --- | --- |
| `world` | 1 | `close_worker(skip_landing=True)`，等 CLOSED（现 `server_controller.py:837`/`:863`） | 失败→ `_world_retry_at` 延后重试，不碰栈 |
| | 2 | `preflight()`；`apply_simulator(entry, restart)` 循环 `restart_attempts` 次（`world.py`） | 全失败→ DISCONNECTED + `sim_error`，等操作员重选 |
| | 3 | 更新 `map_id`/`selection`，`connect()` 新 worker | CONNECTING → READY → IDLE |
| `models` | 1 | `close_worker(skip_landing=True)` | 失败→ `failed(reason)` |
| | 2 | `selection_for(map_id)` 按车型选 scene variant | — |
| | 3 | `connect()`，引擎进程不动 | CONNECTING → IDLE |
| `profile` | 1 | `close_worker(skip_landing=True)` | 失败→ `failed(reason)` |
| | 2 | `connect()`；`worker_factory` 读 `controller.profile` 重载该档 `base.yaml` 并重建 `MissionResources`（`server_bootstrap.py:59-73`） | CONNECTING → IDLE |

`profile` 在 DISCONNECTED 下只记名字（现 `server_controller.py:560-566`），不重建任何东西。

### 2.5 能力求交的计算位置与时机

| 时机 | 输入 | 输出 | 代码位置 |
| --- | --- | --- | --- |
| 构造期，无会话 | 目标地图 `simulator` → `simulators/<name>.runtime.describe()` 的 `PORT_SLOTS`（`simulators/airsim/runtime.py:17`、`simulators/projectairsim/runtime.py:12`） | `/api/world/options` 的 `capabilities` | `assembly/requirements.py` + 组合根把结果挂到 controller |
| 会话构造期 | 每台载具实际拿到的槽位（`MissionInputs`） | 该载具的 `workflows`、`stage_validators`、解析器 codec 集合（写进 `VehicleArm`） | `mission_assembly.py:182` 内调 `requirements` |
| READY | 会话上报的 `vehicles` 与已填槽位 | `/api/status.capabilities` 覆盖声明值 | `controller/state.py` 的 READY 分支（现 `:906-939`） |
| ACTIVE（切载具） | 新载具 arm 的交集 | `/api/status.capabilities` 立即变 | `controller/commands.py` 的 Handoff 成功回调 |
| 换图 / 换模型 / 换配置 | 新地图的 `PORT_SLOTS` 或新会话 | 整组能力位与控件 | READY 分支复用 |

`server_api.py:159` 现在自己调 `registry.capabilities_for`，入口层因此 import 了 `simulator/registry`（`:25`）。目标形态是 `ServerController.capabilities()`，入口层只读它，删掉那条 import。

## 3. 跨层接口

### 3.1 我提供什么

| 名字 | 使用者 | 语义 |
| --- | --- | --- |
| `assemble_missions(config, *, inputs: MissionInputs, resources: MissionResources, log)` | `simulator/registry.py:70`（import）、`:73`（调用） | 端口集合 → `{vehicle_id: VehicleArm}`；缺线即 `ProtocolError` |
| `MissionInputs` / `MissionResources` / `VehicleArm` | 会话侧（`mission_inputs` / `attach_arms`） | 会话交出的引擎对象；装配交回的管线 |
| `require_application(arm)` | `simulators/projectairsim/session.py:297`、`simulators/airsim/session.py:204` | 拿不到解析器时的拒单理由，两个会话共用 |
| `build_resources(config, *, frame_sink, trajectory)` | `server_bootstrap.py:57`、`simulator/mission_child.py`（改后） | 端点与几何的唯一构造入口，替掉两份实现 |
| `StageValidator` | 任务包的 `policy.py` 按形状实现，不 import | 任务阶段准入的形状 |
| `WorkerFactory` / `SimulatorSupervisor` | 组合根传闭包与 `FamilyRouter`，结构满足 | `ServerController` 的两类输入 |
| `PlannerApplication` / `MissionValidator` / `WorkflowRouter` / `MissionRunner` | 层内互用；对外不导出 | 解析 → 校验 → 路由 → 执行，替换物是 `require_application` 与 `ServerController` |
| `task_finish_events(result, task_id)` | `simulator/worker.py:272`、`simulator/mission_child.py:289` | `MissionResult` → 日志行 + 一条 FINISHED 事件 |
| `ServerController` | `adapters/http/server_api.py`、`server_bootstrap.py` | 后端生命周期唯一所有者：`submit`、`stop`、`home`、`activate`、`request_weather`、`request_view`、`request_world`、`stop_world`、`request_models`、`request_profile`、`capabilities()`、`status`、`logs`、`start`、`close` |
| `SwitchRefused(vehicle, reason)` | 入口层映射 409 | 提交期切载具失败，任务未投递 |
| `build_planner` / `build_task_parser` / `build_trace_factory` | 层内调用；输出面只有 `build_resources` | 按配置造端点；巡航规划器并入 `build_planner`，detector/geometry/completion 内联进 `build_resources` |

### 3.2 我要求相邻层提供什么

| 层 | 名字 | 用途 |
| --- | --- | --- |
| 任务层 `workflows/` | `TASK_KIND: TaskKind`、`REQUIRED_PORTS: tuple[str, ...]` | 能力求交的唯一来源。`workflows/__init__.py` 再导出三个包的这两个常量，`requirements.py` 只 import 层入口（裁决 R19a）。缺这两个常量或 `REQUIRED_PORTS` 里的名字不是 `SimulatorPorts` 的槽位 → 构造期 `ProtocolError` |
| 任务层 | `Workflow.prepare() / run(stage) / close()`（`workflows/base.py:8-11`）、各任务 `policy.py` 的 `StageValidator` 实现 | 路由与准入 |
| 执行层 `runtime/` | `ExecutionService(vehicle)`、`bind_hold_checkpoint(checkpoint)`、`ExecutionError`、`StopResult` | 每台载具一个执行服务；停止请求经检查点上抛 |
| 执行层 | `ControlLoop(...)`（构造期收 `MotionCapability`，由它选律）、`ControlLoop.run_segment(segment, sink, *, budget)`、`run_stream(goals, sink, *, deadline_monotonic)`、`PlanningWindow.begin/submit/poll_stream/pause_for_completion/complete/end`、`GuidedVehicle(streaming, loop)` | `ControlLoop` 与 `GuidedVehicle` 都在 `build_vehicle_arm` 里构造；`GuidedVehicle` 拿去填 `Vehicle` 槽（裁决 R1），`ControlLoop` 注入任务包；`run_segment`/`run_stream` 与滑窗六个方法的调用方是任务包。`SegmentSink`/`StreamSink` 都是 `stream_command(command, duration_s)` 的别名，没有第二个下发名字（裁决 R2）。这些名字只从 `runtime/__init__.py` 取（裁决 R19a） |
| 引擎层 `simulators/<name>/` | 数据面只实现 `stream_command(command, duration_s)`，加 `current_pose()`、`motion_capability()`、`collision_state()`、`cancel()`、`hover()` | 引擎不再实现 `execute_segment` 的采样循环，也没有 tracker；过渡期保留三行委派（裁决 R1） |
| 契约层 `ports/` | `Vehicle`（含 `collision_state() -> CollisionState`；槽位由 `runtime` 的 `GuidedVehicle(streaming, loop)` 兑现，裁决 R1）、`StreamingVehicle`（`stream_state()`、`stream_command(command, duration_s)`）、`BackendWorker`（`start/submit/stop/home/activate/set_view_mode/set_weather/poll/shutdown/terminate`，`ports/backend_worker.py:10`） | 注入与事件源 |
| 契约层 `domain/` | `MotionModel`、`MotionCapability`、`HolonomicCommand`、`CarCommand`、`MotionCommand`、`TaskKind`、`TaskSubmission`、`WorkerEvent`、`ServerState`、`ServerStatus`、`Capabilities`、`CollisionRecoverySettings` | 类型与状态 |
| 引擎层 `simulators/<name>/` | `PORT_SLOTS`、`describe() -> SimulatorPorts`、`build_session(...)`、`VEHICLES`、`build_ports(...)` | 声明式能力与真实端口。`ENABLED_TASKS` 删除（裁决 R18） |
| 进程栈 `simulator/` | `registry.build_session`；`ThreadedMissionWorker`、`SubprocessMissionWorker` 只经 `WorkerFactory` 返回；`FamilyRouter` 只经 `SimulatorSupervisor` 注入（裁决 R19b） | 会话与引擎进程 |
| 配置 | `AppConfig`（`motion`、`trajectory_planner`、`visual_goal_navigation`、`free_navigation`、`sliding_window`、`continuous`、`server`、`simulator`、`maps`、`families`、`vehicle_models`）；`motion.passed_lateral_tolerance_m` 只能抬高走廊默认值，默认 1.0 m（裁决 R17） | 构造参数 |

### 3.3 输入面与输出面

跨层可见的名字才有资格进这两张表；内部实现类不出现在表里。

输入：

| 名字 | 形状 | 由谁提供 | 是否可选 |
| --- | --- | --- | --- |
| `AppConfig` | 冻结数据类 | 数据包 `planner/config` 入口 | 否 |
| `MissionInputs` | 装配层自己的冻结数据类，字段一律声明为 `object` | 会话（引擎侧） | 否；`execution`、`observation_source`、`moving_source_factory` 三个字段可为 `None` |
| `MissionResources` | 同上 | 组合根 `server_bootstrap` | `trajectory` 不可是 `None`（缺即报错），`frame_sink` 可选 |
| `SimulatorPorts` | 契约层冻结数据类 | 引擎包 `describe()` | 否 |
| `WorkerFactory` | 本层 Protocol：`__call__(MapSelection) -> BackendWorker` | 组合根 | 否 |
| `SimulatorSupervisor` | 本层 Protocol：`running()`、`preflight()`、`start(entry)`、`restart(entry)`、`stop()` | 组合根（`FamilyRouter` 的实例） | 否；传 `None` 时地图命令直接 `ValueError` |
| `BackendWorker` | 契约层 Protocol | 进程栈 | 否 |
| `TASK_KIND`、`REQUIRED_PORTS` | `TaskKind` 与 `tuple[str, ...]` | `workflows` 层入口 | 否 |
| `ExecutionService`、`ControlLoop`、`GuidedVehicle` | runtime 层入口导出的类，装配只构造不收 | `runtime` 层入口 | 否 |
| `clock` | 本层 Protocol：`__call__() -> float` | 组合根 / 测试 | 是；默认 `time.monotonic` |

输出：

| 名字 | 形状 | 由谁消费 | 稳定性 |
| --- | --- | --- | --- |
| `assemble_missions` | 函数，返回 `dict[str, VehicleArm]` | `simulator/registry.build_session` | 稳定 |
| `MissionInputs`、`MissionResources`、`VehicleArm` | 数据类 | 会话侧、组合根 | 稳定；`VehicleArm.model_calls` 字段被引擎会话 `close()` 读（`projectairsim/session.py:734`） |
| `require_application` | 函数，返回一个只被调 `prepare` / `execute_plan` 的对象（`PlannerApplication` 是内部名） | 两个会话的 `run` | 稳定 |
| `build_resources` | 函数，返回 `MissionResources` | `server_bootstrap`、`mission_child` | 稳定 |
| `ServerController` | 类，公开方法见 §1.2 | `adapters/http`、组合根 | 稳定 |
| `SwitchRefused` | 异常类型 | `adapters/http` 映射 409 | 稳定 |
| `StageValidator` | Protocol | 任务包结构实现 | 稳定 |
| `WorkerFactory`、`SimulatorSupervisor` | Protocol | 组合根结构实现 | 稳定 |
| `task_finish_events` | 函数，返回 `(lines, WorkerEvent)` | `simulator/worker`、`mission_child` | 稳定 |

不暴露什么：四个控制器协作者（`LifecycleState`、`CommandChannel`、`SessionSwitcher`、`EnvironmentChannel`）、`Handoff`、`PortsView`、`EXECUTABLE_PACKAGES`、`build_vehicle_arm`、`PlannerApplication`、`MissionValidator`、`MissionRunner`、`WorkflowRouter`、`mission_report`、六个工厂私有函数都不出 `assembly/__init__.py`，同层内部随便 import。反例：runtime 的 `WorldWaypointQueue` 不在本层表里，装配也不认识 `PlanningWindow` 的内部队列。

## 4. 错误与失败路径

### 4.1 运行期

| 情形 | 谁抛 | 谁捕获 | 怎么恢复 |
| --- | --- | --- | --- |
| 提交期切载具失败或超时 | `Handoff` → `SwitchRefused` | `submit` | 状态保持 IDLE，`_vehicle` 未改写，任务不投递；入口映射 409 |
| 空闲期切载具失败 | worker 发 `ACTIVE(success=False)`（`simulator/worker.py:189`） | 事件泵 | 按 `_pending_activation` 回滚载具与机位，`view_pending=False`，写 WARNING |
| 非 IDLE / 非 DISCONNECTED 下收到命令 | `BackendUnavailable(state)`（`domain/server.py:113`） | 入口层 | 503（CONNECTING/DISCONNECTED）或 409（IDLE 之外的其他态），映射在 `server_api.py:42-51` |
| 未知载具 / 未知地图 / 未知车型 / 未知配置 | `ValueError` | 入口层 | 400 |
| 动作背后的端口不存在 | `ServerController.capabilities()` 不含该 action | 入口层 `require()`（`server_api.py:167`） | 501，`detail.unsupported` + `detail.simulator` |
| worker 构造或会话启动超时 | `CommandChannel.failed(reason)` | 监督循环 | 换代、置 CONNECTING、`reconnect_interval_s` 后重建 |
| 任务中 worker 报 ERROR 或 CLOSED | 事件泵（`:1006-1013`） | 监督循环 | 同上；`_active` 清空，`last_result` 保留最后一条消息 |
| 换图时 worker 不确认 CLOSED | `wait_closed` 返回 False | `world.py` | 不碰引擎栈，置 `_world_retry_at`，等下一轮重试 |
| 引擎栈非本服务启动 | `SimulatorNotOwned`（`errors.py:22`） | `world.py` | 只告警，栈继续运行；`stop_world` 把原因写进 `sim_error` |
| 引擎启动失败 | `simulator.restart/start` 抛任意异常 | `switch_world` 循环 `restart_attempts` | 全失败 → DISCONNECTED + `sim_error`，操作员重选场景 |
| 任务返回无完成证据 | `MissionRunner.run`（`mission_runner.py:24-44`） | 装配自身 | 转 `StageResult(False)`，任务失败，状态回 IDLE |
| 解析器未配置 | 无异常；`require_application` 在提交时抛 `ProtocolError`（`mission_assembly.py:352`） | worker | 任务被拒，日志说明缺哪个配置项 |
| `session_reset` 抛错 | 任意异常 | `reset_session_state`（`:830`） | 只告警，不阻塞重启 |

### 4.2 构造期失败清单

构造期报错指服务起不来或会话建不起来，不留到运行期。

| 触发 | 抛什么 | 位置 |
| --- | --- | --- |
| `MissionResources.trajectory is None` | `ProtocolError` | `mission_assembly.py:97-101`（保留） |
| 装配返回的 arm 少于会话声明的载具 | `ProtocolError` | `simulators/projectairsim/session.py:276-280`（保留） |
| 某台载具的槽位交集为空 | `ProtocolError`（"没有任务能在这台载具上跑"） | `requirements.assert_constructible`，新增 |
| `REQUIRED_PORTS` 里出现 `SimulatorPorts` 没有的槽位名 | `ProtocolError` | `requirements.declared_requirements`，新增；用 `dataclasses.fields(SimulatorPorts)` 比对，写法与 `tests/test_architecture.py:137-140` 一致 |
| 两个任务包声明同一个 `TASK_KIND` | `ProtocolError` | `requirements.declared_requirements`，新增 |
| `MotionCapability.model` 与本臂注入的 `StreamingVehicle` 接受的命令类型不符 | `ProtocolError` | `command_type_for(capability)` 与 `require_supported(command, capability)`（`domain/motion.py`，见 `docs/refactor/01-contracts.md`）在 `build_vehicle_arm` 构造臂时比对；对应 `00-boundaries.md` 第五节"端口与命令类型不符" |
| 准入表缺某个 `TaskKind` 的 `StageValidator`，或终结类任务没有准入器 | `ValueError` | `mission_validator.py:21-36`（保留） |
| 检测器 / 规划器 mode 未知 | `ValueError` | `factories/resources.py`、`factories/planners.py`（`factories.py:99`、`:134`） |
| 选了 `vlm` / 补全判定却没配 `url`/`model` | `ValueError` | `factories.py:80-84`、`:105-108`、`:152-155`（保留） |
| 轨迹规划器 `timeout_s`/`max_tokens` 非正 | `ValueError` | `config/loader.py`（`config.py:1574-1576`） |
| 配置分节不属该 profile | `ValueError` | `config/loader.py`（`config.py:1390-1396`） |
| 地图引用了未配置的仿真器 | `ValueError` | `loader.py`（`config.py:1468-1473`） |

解析器缺失不算构造期失败：`build_task_parser` 返回 `None`，装配写 WARNING（`mission_assembly.py:319-323`），提交时由 `require_application` 拒绝。这是现状行为，保留，因为无模型端点仍要能起服务、看状态。

### 4.3 配置包拆出后要收掉的一处反向依赖

`config.py:1280-1286` 在 `load_vehicle_models` 里 `from .simulator import registry`，只为筛出"声明了 `models` 能力的地图"。拆包后改为：候选地图取"`maps.yaml` 里声明了 `scene.vehicle_scenes` 的条目"。判据自洽，同时删掉配置包对引擎包的依赖。代价是非模型类家族的图如果写了 scene variant，现在会在加载期报错而不是被跳过；这类错误本来就该在加载期暴露。

## 5. 并发与所有权

### 5.1 线程

| 线程 | 数量 | 持有的东西 |
| --- | --- | --- |
| HTTP 线程池 | 每请求一个 | 只调 `ServerController` 的公开方法，不碰 worker 内部 |
| 监督线程 `planner-supervisor` | 1 | worker 引用、命令队列消费、事件泵、重试计时（`server_controller.py:215-227`、`:1015`） |
| worker 线程（UE5） | 1 | 会话、载具、`ExecutionService._owner`（`services/execution.py:32`） |
| 子进程（UE4） | 1 | 同上，跑在 `planner.simulator.mission_child` 里 |

### 5.2 同步原语与不变量

- 一把 `threading.Lock` 保护状态与所有开关位（`_state`、`_worker_ready`、`_map_switching`、`_models_switching`、`_profile_switching`、`_view_pending`、`_pending_activation`、`_weather`）。读状态一律在锁内取快照再出锁，`status` 属性就是这么写的（`:150-176`）。
- 命令队列是 HTTP 线程与监督线程之间唯一的通道。每条命令带 `generation`，监督线程丢弃代数不符的命令（`:648-656`），换代发生在 `connect`、`failed`、`close_worker` 之后。
- 事件只由监督线程消费。任何协作者的状态改写都必须发生在 `on_event` 内，不在 HTTP 线程直接改。
- 提交期切载具的等待用单次 `threading.Event`，不用条件变量：一个 Handoff 对应一次请求，超时即弃，晚到的 ACTIVE 被代数或目标载具校验丢弃。等待期间不持 state 锁，否则 `on_event` 无法写回。

### 5.3 控制权

- 控制权的物理落点在执行层：`ExecutionService` 用线程标识判 owner，`acquire_flight`/`release_flight` 校验（`services/execution.py:53-75`）。装配层不参与运动控制，只保证同一时刻只有一台载具的 arm 在接受指令。
- 一台载具一个 `ExecutionService`（`mission_assembly.py:220`），因此切载具不共享执行服务，也不存在两个 owner 争同一条控制链。
- 切载具时的控制权交接只有三步，都在引擎侧：`_park` 让出车停稳（无人机停飞、无人车踩住刹车），`_hide` 把非活动载具移出画面并钉在固定位，`return_home` 把目标载具传送到出生点。任务层的 `bind_hold_checkpoint` 在 `session.run` 里重新绑定（`simulators/projectairsim/session.py:314-321`），旧任务的停止请求不会带到新任务。
- 位置保持、观测采集只在 worker 线程内临时取用控制权，与任务执行串行。

### 5.4 会话与引擎进程

- 换图必须重建 worker，因为 cooked 地图不能在运行中的进程里替换；换模型与换配置只重建 worker，引擎进程不动；切载具两者都不动。
- 子进程家族（`runtime_python` 非空）里，任务循环与 `ExecutionService` 都在子进程，父进程只有 `SubprocessMissionWorker` 与 `TrajectoryHub` 的 adopt 通道（`subprocess_worker.py:101`、`:212`）。`WorkerEventKind.TRAJECTORY` 在 worker 层被消费，不进控制器状态机。

## 6. 测试点

### 6.1 新增单测

| 文件 | 断言对象 | 负例 |
| --- | --- | --- |
| `tests/test_assembly_requirements.py` | `declared_requirements()` 等于三个任务包的 `REQUIRED_PORTS`；`available_kinds` 在 `{vehicle, observation}` 上给出 `{basic_motion, visual_goal_navigation}`，加 `streaming` 给出三个；两台载具给不同槽位得到不同集合；`SimulatorPorts` 不再有 `enabled_tasks` 字段（裁决 R18） | 槽位名不在 `SimulatorPorts` 里 → `ProtocolError`；交集为空 → `ProtocolError`；两包同 `TASK_KIND` → `ProtocolError`；`REQUIRED_PORTS` 缺失 → `ProtocolError`；给 UE4 的声明槽位集（含 `streaming`）断言 `free_navigation` 在结果里（R18：无白名单） |
| `tests/test_assembly_handoff.py` | 提交期切载具的调用序：`FakeWorker.activations == ["rover"]` 且 `tasks` 在 `activations` 之后；同载具提交不产生 activate；ACTIVE 成功后才 `state == RUNNING` | ACTIVE 失败：`tasks == []`、`state == IDLE`、`_vehicle` 未变、抛 `SwitchRefused`；ACTIVE 永不返回：超时后同上；RUNNING 中 `activate` → `BackendUnavailable` |
| `tests/test_assembly_capabilities.py` | `ServerController.capabilities()` 在无会话时来自 `PORT_SLOTS`，READY 后被会话上报值覆盖，ACTIVE 后被新载具交集覆盖 | 入口层不再 import `planner.simulator`（AST 断言）；端口缺失时该 action 不在 `Capabilities.actions` 里 |
| `tests/test_assembly_controller_split.py` | 六个协作者的公开方法名与 §1.2 表一致；`ServerController` 的构造签名不含已删除的字段 | 在 `LifecycleState` 之外写 `_state`、`_weather`、`_view_mode` 抛 `AttributeError`（属性只读），确保状态改写只能经 `on_event` |
| `tests/test_config_package.py` | `planner.config` 的公开名字集合与拆分前一致（现 `tests/test_config.py` 的 import 清单）；`profiles_root()` 指向仓库根 `config/core` | `load_vehicle_models` 在没有任何 scene variant 时的行为；地图声明了不存在的 variant → `ValueError` |
| `tests/test_layer_entries.py` | 每层入口 `__init__.py` 导出的名字集合与 §3.3 输出表一致；跨层 import 的模块路径停在包名（函数参数或返回值里出现别层具体类也算违规）（裁决 R19a、R19b） | `from planner.runtime.motion.loop import ControlLoop` 这类深层路径 → 失败；`ServerController` 的 `simulator`/`worker_factory` 参数换成别层具体类 → 失败 |

### 6.2 事件顺序如何被钉住

`tests/test_server_controller.py:96` 的 `FakeWorker` 已按调用记录字段（`tasks`、`homes`、`activations`、`views`、`weathers`、`stops`），`FakeSupervisor:160` 记录 `launches`/`restarts`/`stops`。事件顺序测试用这两个替身，断言列表而非断言最终状态：

| 不变量 | 现有守卫 | 新增守卫 |
| --- | --- | --- |
| READY 之后才允许提交 | `test_ready_then_task_lifecycle`（`:271`） | — |
| stop 先投递再回 IDLE | `test_stop_requests_worker_stop_then_returns_idle`（`:560`） | — |
| ACTIVE 失败回滚载具与机位 | `test_active_event_failure_restores_the_previous_selection`（`:693`） | 提交期同一失败路径（`test_assembly_handoff.py`） |
| 换图必须先等 CLOSED 再重启栈 | `test_world_switch_waits_for_worker_closed_before_restart`（`:970`） | worker 不确认 CLOSED（`SilentWorker:150`）时不调 `restart` |
| 任务结束后才应用被延迟的机位 | `test_deferred_view_marks_pending_after_the_task_finishes`（`:800`） | ACTIVE 之后同样只允许在 IDLE 应用 |
| READY 重放天气但不重放机位 | `test_weather_is_replayed_but_the_view_resets_on_the_next_ready`（`:862`） | — |
| 重复 FINISHED 被忽略 | `test_duplicate_finished_event_is_ignored`（`:1287`） | 代数不符的 ACTIVE 同样被丢弃 |
| 能力位随 ACTIVE 变化 | — | 新：`capabilities()` 在 ACTIVE 前后返回不同 `task_kinds` |

### 6.3 起到结构作用的既有测试

- `tests/test_architecture.py:126`（声明槽位与 `build_ports` 一致）与 `:137-140`（`SimulatorPorts` 字段名）继续守住"槽位名是唯一词表"，`REQUIRED_PORTS` 的校验复用同一份字段集合。
- `tests/test_runtime_catalog.py:60`、`:71`：能力位由端口推导，不读配置文件。删 `enabled_tasks` 后两张能力矩阵的期望值跟着改：UE4 的 `streaming` 槽有值，`task_kinds` 从两项变三项（裁决 R18）。
- `tests/test_capability_gating.py`：控制台控件从能力位推导，通过 `create_api(controller)` 打到入口层。能力位改由 controller 提供后，`tests/test_server_api.py` 的 `FakeController` 需要实现同一个 `capabilities()`，断言不变。
- `tests/test_factories.py`：拆分后六个构建函数同名同签名，测试不改。

## 7. 迁移步骤

每步独立可发布、独立可验证。

| 步 | 做什么 | 验证 |
| --- | --- | --- |
| 1 | `planner/application/` 改 `planner/assembly/`，包内 import 全改；`tests/` 与 `scripts/` 同步改 import 路径。不动任何函数体 | `pytest tests/ -q` 用例数不变、全绿 |
| 2 | 新增 `assembly/requirements.py`；三个任务包补 `TASK_KIND` 与 `REQUIRED_PORTS`；删 `simulator/ports.py:44` 的 `TASK_REQUIREMENTS` 与 `:85` 的 `available_tasks()`；`mission_assembly.py:281-290` 的过滤改调 `requirements.available_kinds` | `tests/test_assembly_requirements.py` 新绿；`test_runtime_catalog.py`、`test_capability_gating.py` 不改断言通过 |
| 3 | 拆控制器：先 `commands.py`（队列 + worker 生命周期），再 `world.py`，再 `models.py`/`weather.py`/`profiles.py`，最后 `state.py` 与 `controller/__init__.py` 门面。每步 `ServerController` 公开签名不变 | 每步跑 `tests/test_server_controller.py` 68 个用例，只改 import 行，断言不动 |
| 4 | 提交期切载具改阻塞交接 + `SwitchRefused`；`ServerSettings` 加 `handoff_timeout_s`；入口层加一条 `except SwitchRefused → 409` | `tests/test_assembly_handoff.py` 新绿；`test_server_api.py` 56 个用例不改通过 |
| 5 | 能力位改由 `ServerController.capabilities()` 提供，`/api/world/options` 的 `simulator_labels` 一并改由 controller 提供（现读 `registry.labels()`，`server_api.py:322`）；删 `server_api.py:25` 的 `from ...simulator import registry` | 两个端点的 JSON 与重构前逐字段比对（golden 快照），只有 `task_kinds` 在切载具后收窄属预期差异；`tests/test_architecture.py` 全绿 |
| 6 | 拆 `assembly/factories/` 为四个文件（`__init__`、`resources`、`planners`、`parsers`）；`build_detector`/`build_geometry`/`build_completion_checker` 内联成 `resources.py` 的私有函数，`build_free_navigation_planner` 并入 `build_planner`；`server_bootstrap` 与 `mission_child` 两边改调 `build_resources` | `tests/test_factories.py` 不改通过；两份资源构造的字段逐项一致 |
| 7 | 拆 `planner/config/` 包；`_project_root()` 层级改为 `parents[2]`；`load_vehicle_models` 去掉 `registry` 依赖 | `tests/test_config.py`、`test_profiles.py`、`test_reference_config.py` 不改通过；`scripts/check_profiles.py` 输出为空 |
| 8 | 加 `tests/test_layering.py` 的五条规则、§6.1 的构造期负例，以及 `tests/test_layer_entries.py` 的包入口与输入面规则（裁决 R19a、R19b） | 新测试绿；`pytest tests/ -q` 用例数只增不减 |

## 8. 削面复核

复核口径：只认层外、运行路径上的调用点（文件:行号）。测试引用、同层内部引用、`__init__` 转发都不算。留一个名字要落进三类豁免之一：被外层运行路径调用；对外契约（HTTP 字段、配置字段、引擎或任务必须实现的 Protocol）；构造期不变量校验。引用数是 `grep -rn "\b<name>\b" --include=*.py planner scripts tests | wc -l`，含测试。

### 8.1 表 A 对外名字逐条

| 名字 | 现引用数 | 层外运行期调用点 | 处置 | 依据 |
| --- | --- | --- | --- | --- |
| `assemble_missions` | 8 | `simulator/registry.py:70`（import）、`:73`（调用） | 保留 | 会话工厂拿它接线 |
| `MissionInputs` | 10 | `simulators/projectairsim/session.py:271`、`simulators/airsim/session.py:163` | 保留 | 会话构造它并交给装配 |
| `MissionResources` | 9 | `server_bootstrap.py:38`、`simulator/mission_child.py:232` | 保留 | 组合根造它，两个进程各一份 |
| `VehicleArm` | 7 | `simulators/projectairsim/session.py:84`（`_VehicleArm` 别名） | 保留 | `attach_arms` 的元素类型 |
| `require_application` | 5 | `projectairsim/session.py:297`、`airsim/session.py:204` | 保留 | 两个会话共用的拒单路径 |
| `task_finish_events` | 5 | `simulator/worker.py:272`、`simulator/mission_child.py:289` | 保留 | 两种 worker 共用同一份完成文本 |
| `ServerController` | 16 | `server_bootstrap.py:117` | 保留 | 组合根唯一后端门面 |
| `StageValidator` | 2 | 0（任务包按形状实现，不 import） | 保留（需裁决） | 对外契约：任务阶段准入的形状 |
| `WorkerFactory` | 0 | 0（组合根传闭包，结构满足） | 保留（需裁决） | `ServerController` 的输入契约 |
| `SimulatorSupervisor` | 0 | 0（组合根传 `FamilyRouter`） | 保留（需裁决） | 同上 |
| `SwitchRefused` | 0 | 0（入口层尚未落地 `except`） | 保留（需裁决） | 409 的唯一语义载体 |
| `build_resources`（新） | — | `server_bootstrap.py:57`、`mission_child.py:232`（改后） | 新增对外 | 替掉两份资源构造实现 |
| `capabilities_for` | 3 | 0（入口读 `ServerController.capabilities()`） | 内部化 | 只在 `ServerController.capabilities()` 里调，不出层 |
| `PlannerApplication` | 11 | 0 | 内部化 | 只在本层构造，外部只用 `require_application` |
| `MissionValidator` | 13 | 0 | 内部化 | 只被 `PlannerApplication` 用 |
| `MissionRunner` | 13 | 0 | 内部化 | 同上 |
| `WorkflowRouter` | 9 | 0 | 内部化 | 同上 |
| `MissionCommon` | 4 | 0 | 删 | 只是建臂中间数据，并入 `VehicleArm` 构造 |
| `build_vehicle_arm` | 3 | 0（1 处只是 docstring） | 内部化 | 只有本层调 |
| `build_mission_common` | 2 | 0 | 删 | 与 `MissionCommon` 同时消失 |
| `build_detector` | 7 | `mission_child.py:233`、`server_bootstrap.py:39`、`scripts/smoke_projectairsim_drone.py:280` | 删（内联进 `build_resources`） | 三层调用都改为 `build_resources` |
| `build_geometry` | 7 | `mission_child.py:240`、`server_bootstrap.py:46`、`smoke_projectairsim_drone.py:137` | 删（内联） | 同上 |
| `build_planner` | 13 | `mission_child.py:239`、`server_bootstrap.py:45`、`smoke_projectairsim_drone.py:283` | 内部化 | 同上 |
| `build_free_navigation_planner` | 10 | `scripts/smoke_projectairsim_rover.py:152` | 合并进 `build_planner` | 两函数只差 prompt 与“未配置返 None”语义 |
| `build_task_parser` | 12 | 0（`mission_assembly.py:316` 在本层） | 内部化 | 建臂时才知道 kinds 与 aerial |
| `build_completion_checker` | 6 | 0 | 删（内联） | 只有 `build_mission_common` 用 |
| `build_trace_factory` | 5 | 0 | 内部化 | 同上 |
| `mission_report` | 7 | 0（`reporting.py:56` 与 5 处测试） | 内部化 | 私有化为 `_report`；测试改读 `task_finish_events` |

### 8.2 表 B 历史残留删除清单

| 对象 | 位置 | 处理 | 理由 |
| --- | --- | --- | --- |
| 两份资源构造 | `server_bootstrap.py:29-49` 与 `simulator/mission_child.py:232-252` | 合并为 `assembly/factories/resources.py::build_resources` | 同一构造写两遍，只能靠注释维持一致；`frame_sink` 漏传曾让控制台掉面板（`mission_child.py:242-246`） |
| `TASK_REQUIREMENTS` | `simulator/ports.py:44`，引用 3 | 删 | 唯一读者是同类 `:111`；交集改由 `assembly/requirements.py` 算（R18） |
| `SimulatorPorts.available_tasks()` | `simulator/ports.py:85`，引用 2（均同类） | 删 | 同上 |
| `SimulatorPorts.enabled_tasks` | `simulator/ports.py:77`、`:90`、`:122`、`:133`、`:157`，加 `mission_assembly.py:83`、`:118`、`:192`、`:215`，共 8 处 | 删 | 能力只由接口决定（R18） |
| `lane_reference_provider` 与 `format_lane_reference` | `mission_assembly.py:306`；`workflows/free_navigation/workflow.py:76`、`:125`、`:164`、`:383`、`:386`；`continuous.py:54`、`:91`、`:437`、`:440`；`domain/lane_reference.py`；`prompts/free_navigation.py:404`、`:410`，共 13 处 | 删 | 纯视觉巡航（R1） |
| `application/__init__.py` 只导出 `PlannerApplication` | `application/__init__.py:1-3`，层外调用 0 | 入口换成 §3.3 的 12 个名字 | 旧入口导出的是测试才用的名字 |
| `mission_report` | `application/reporting.py:6`，引用 7（5 处在测试） | 私有化 `_report` | 运行期只有 `task_finish_events` 一个读者 |
| `MissionCommon.aerial_planner` / `aerial_free_planner` | `mission_assembly.py:132-134`、`:159-176` | 随 `MissionCommon` 删 | 建臂时按 `is_drone` 选 prompt，少一个每臂共享的容器 |
| `config.py` 里的引擎依赖 | `config.py:1280-1286` 的 `from .simulator import registry` | 删 | 判据改为“地图声明了 `scene.vehicle_scenes`”（§4.3） |

### 8.3 合并结论

- 控制器：6 文件/6 协作者 → 4 文件/4 协作者（§1.2 已按此更新）。`SessionSwitcher` 收地图、外观、配置三条命令，共用 `close_worker → 必要时动栈 → connect` 骨架；`EnvironmentChannel` 收天气与机位（R12）。
- 工厂：7 个构建函数 → 4 个（§1.3 已按此更新）：`build_resources`（吸收 detector、geometry、completion checker）、`build_planner`（吸收巡航 prompt 与 aerial 变体）、`build_task_parser`、`build_trace_factory`。后三个不导出；代价是三个 `scripts/smoke_*.py` 改调 `build_resources`。
- 配置包：7 模块 → 5 文件（§1.4 已按此更新）：`__init__.py`、`profiles.py`（路径 + 配置档元数据）、`sections.py`（含相机）、`maps.py`（含外观模型）、`loader.py`。

### 8.4 结论数字

对外名字 27 → 12：删 5 个（`MissionCommon`、`build_mission_common`、`build_detector`、`build_geometry`、`build_completion_checker`），内部化 10 个（`capabilities_for`、`PlannerApplication`、`MissionValidator`、`MissionRunner`、`WorkflowRouter`、`build_vehicle_arm`、`build_planner`、`build_task_parser`、`build_trace_factory`、`mission_report`），合并 1 个（`build_free_navigation_planner`），新增 1 个（`build_resources`）。5+10+1=16，27-16+1=12。删除代码约 120 行：`MissionCommon` 与 `build_mission_common` 45 行、`mission_child` 重复资源构造 22 行、detector/geometry/completion 三个公开包装 20 行、巡航规划器合并 12 行、`enabled_tasks` 过滤与泳道接线 6 行、入口旧导出 3 行、`capabilities_for` 层外导出 2 行。

### 8.5 需指挥层裁决

1. `SwitchRefused`：目前无运行期调用点，入口层 `except` 分支尚未落地。保留理由是它是“切载具失败 → 409”的唯一载体，复用 `BackendUnavailable` 会把 `detail.message` 写成 "planner is idle"。
2. `StageValidator`、`WorkerFactory`、`SimulatorSupervisor`：三者都没有 import 调用点，靠结构类型被任务包与组合根满足。保留理由是对外契约（R19b 的输入面）。若要求“无调用点即删”，三个名字要降为文档约定，删掉后构造签名只剩 `object`。
3. `build_resources`：新增名字，无历史引用可数；它的依据是 8.2 表第一行的两份实现必须收成一份。
4. 三个 smoke 脚本直调 `build_detector`/`build_geometry`/`build_planner`：把它们算作工具路径（已内部化）还是运行路径（则三个工厂必须保留对外）需定口径。本表按“工具路径不算运行路径”处理。

## 9. 未决问题

### 9.1 提交期切载具是否阻塞 HTTP 请求

选项：阻塞到 ACTIVE（默认），或立即 202 再在任务里切（现状）。

取默认阻塞，上限 `server.handoff_timeout_s = 15.0`。理由：交接失败时目标载具可能已经传送或切过机位，若此时已经返 202，回滚只能表现为任务失败，操作员看到的是“任务失败”而不是“切载具失败”。阻塞把失败留在准入阶段，对应 409。代价是 `/api/tasks` 最坏多等 `handoff_timeout_s`；正常路径只走 park、传送、切机位三步，三步都在 worker 线程同步完成。

### 9.2 `task_kinds` 的语义从"仿真器支持"变成"当前载具支持"

选项：字段值改为当前载具的交集（默认），或保持仿真器级并新增 `task_kinds_by_vehicle` 字段。

取默认，HTTP 字段不增不减。代价是同一个地图下切换载具时 `task_kinds` 会变，控制台必须按新值重渲染按钮（已有能力门控逻辑，数据源换掉即可）。选另一条路会让 `/api/status` 出现两份任务清单，控制台得决定读哪一份。

### 9.3 实测白名单已删除（原 R11 作废）

裁决 R18：删 `SimulatorPorts.enabled_tasks` 字段，删 `simulators/projectairsim/runtime.py` 与 `simulators/airsim/runtime.py` 的 `ENABLED_TASKS`。可跑任务 = `REQUIRED_PORTS` ⊆ 已填槽位，没有第二个开关。原 R11 定的“白名单留在槽位包”不再成立；代码里唯一残留是提交时日志不再区分“引擎不支持”与“未实测”两种拒绝理由。

### 9.4 `simulator/` 反向 import 装配层

`simulator/registry.py:70` 与 `simulator/worker.py:14`、`mission_child.py:37` 需要装配层的名字。选项：只从 `assembly/__init__.py` 取（默认），或把这些名字上移到组合根。

取默认：`assembly/__init__.py` 导出 `assemble_missions` 与 `task_finish_events`，`simulator/` 不得 import `assembly.mission_assembly` 这类深层路径（裁决 R19a）。`registry.build_session` 仍是“会话建好就接线”的唯一缝，两个调用点（`server_bootstrap.py:98` 的闭包与 `mission_child.py:254`）共用它。`tests/test_layering.py` 写成“`planner.simulator` 的 import 只能指向 `planner.assembly` 包本身，出现子模块路径即失败”。

### 9.5 切载具失败用新异常还是复用 `BackendUnavailable`

选项：新 `SwitchRefused` + 入口层新增一条 409 映射（默认），或复用 `BackendUnavailable(ServerState.IDLE)`（现状会落到 409，无需改入口）。

取新异常。复用会让 `detail.message` 写成 "planner is idle"，而真实原因是出车没停稳或目标载具传送失败；操作员看不出要修什么。代价是入口层多一个 `except`。

### 9.6 天气与机位合并成一个协作者

已裁决：合并，一个文件（裁决 R12）。`controller/weather.py` 放 `EnvironmentChannel`，同时管天气与机位。两者生命周期一致：跨会话保留、READY 后重放、RUNNING 中延迟、失败回滚到 `_applied_view`/上一份天气。拆开会让"READY 之后重放哪些状态"这件事散在两个文件里。

### 9.7 `api_key` 放哪个工厂文件

选项：放 `factories/planners.py`，另三个文件从它 import（默认），或新建 `factories/credentials.py`（第五个文件）。

取默认。四类端点共用同一套"配置里的 key 优先、环境变量兜底、缺失时告警一次"的规则（`factories.py:38-56`），为它多开一个文件只是多一层 import；但要接受 `detectors.py`、`parsers.py`、`completion.py` 依赖 `planners.py` 这个方向，工厂之间没有别的交叉依赖。
