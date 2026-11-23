> 上级：[项目架构](../ARCHITECTURE.md)
> 同层兄弟：[domain](./domain.md)、[algorithms](./algorithms.md)、[runtime](./runtime.md)、[workflows](./workflows.md)、[assembly](./assembly.md)、[adapters](./adapters.md)、[simulator](./simulator.md)、[simulators](./simulators.md)

# ports 层

## 1 一句话职责

`planner/ports/` 用 13 个文件 291 行（含入口）声明 18 个 `Protocol`，12 个模块各管一组，是引擎实现与共享逻辑之间唯一的形状约定，本身不含任何代码路径。

## 2 边界

管：

- 13 个必需或可选端口的结构签名：`Vehicle`、`StreamingVehicle`、`ObservationSource`、`ControlledObservationSource`、`MovingObservationSource`、`PositionHoldVehicle`、`GoalObserver`、`StreamGoalSource`、`TargetDetector`、`TaskCompletionChecker`、`TrajectoryPlanner`、`TaskParser`、`TaskTrace`。
- 5 个可选能力端口：`ReturnHome`、`WeatherControl`、`StreamingViews`、`AppearanceModelControl`、`FramePublisher`，它们的存在与否就是能力位本身（`planner/ports/capabilities.py:1`）。
- 端口方法签名里引用的类型全部来自 `planner.domain`，`planner/ports/*` 只 import `planner.domain` 与标准库（`task_trace.py:1` 的 `pathlib.Path` 是唯一额外 import）。

不管：

- 端口实例怎么打包给共享层 → `planner/simulator/ports.py` 的 `SimulatorPorts`，那属于装配与进程边界，不是本层。
- 哪个槽位解锁哪个按钮 → `planner/assembly/requirements.py:32` 的 `ACTION_PORTS`。
- 端口实现 → `planner/simulators/<name>/`，见 [simulators](./simulators.md)。
- 端口实现与命令类型是否相符的判断函数 `require_supported` → [domain](./domain.md)。

## 3 输入与输出

- 入口：`planner/ports/__init__.py`，`__all__` 18 个名字，与 `tests/test_public_surface.py:90` 的 `CALIBRATED["ports"]` 逐名比对。
- 形态：`typing.Protocol`，无基类，无 `@runtime_checkable`（全仓库 grep `runtime_checkable` 无命中），因此 `isinstance` 检查在运行期不可用。
- 返回：方法全部标注具体返回类型，绝大多数来自 domain；`ControlledObservationSource.request_capture` 返回 `concurrent.futures.Future[Observation]`（`planner/ports/observation_source.py:18`）。
- 异常：本层不定义、不抛异常。端口契约要求的失败行为写在 docstring 里，需要抛出时用 `planner/errors.py` 的类型（如 `TrajectoryPlanner.plan` 的 `ProtocolError`，`planner/ports/trajectory_planner.py:19`）。
- 层外引用面：`planner/` 内 14 个文件 import `planner.ports`，`tests/` 内只有 `tests/test_architecture.py`。

## 4 对外名字表

| 名字 | 定义处 | 方法 | 已存在的实现 | 层外调用点 |
| --- | --- | --- | --- | --- |
| `Vehicle` | `vehicle.py:18` | `current_pose`、`tracking_limits`（属性）、`motion_capability`、`execute_segment`、`cancel`、`hold`、`collision_state` | `AirSimVehicle`（`planner/simulators/airsim/vehicle.py:23`）、`ProjectAirSimDrone`（`projectairsim/drone.py`）、`ProjectAirSimRover`（`projectairsim/rover.py`）；`GuidedVehicle`（`planner/runtime/guided_vehicle.py:13`）兑现 `execute_segment`（`planner/runtime/guided_vehicle.py:37`）并转发读侧与命令侧方法（`current_pose`、`motion_capability`、`collision_state`、`cancel`、`hold`、`stream_state`、`stream_command`），`tracking_limits` 由引擎对象直接提供 | `assembly/mission_assembly.py:226`、`runtime/execution.py:104`、`simulator/ports.py:45` |
| `StreamingVehicle` | `streaming_vehicle.py:7` | `stream_state`、`stream_command` | 同上三个引擎类 | `runtime/continuous_execution.py`、`simulator/ports.py:47` |
| `PositionHoldVehicle` | `position_hold_vehicle.py:7` | `stream_state`、`cancel`、`hold_velocity` | `ProjectAirSimDrone`（`projectairsim/drone.py:298`）、`AirSimVehicle`（`airsim/vehicle.py:230`）；`ProjectAirSimRover` 没有 `hold_velocity` | `runtime/position_hold.py:22` |
| `ObservationSource` | `observation_source.py:12` | `capture(*, budget)` | `AirSimObservationSource`（`airsim/observation_source.py:76`）、`ProjectAirSimObservationSource`（`projectairsim/observation.py:107`） | `assembly/mission_assembly.py:230`、`workflows/visual_goal_navigation/workflow.py` |
| `ControlledObservationSource` | `observation_source.py:16` | `request_capture(timeout_s)` | `AirSimObservationSource`（`airsim/observation_source.py:96`） | `simulator/ports.py:48` |
| `MovingObservationSource` | `observation_source.py:21` | `capture_moving(*, budget)`、`close` | 上述两个观测源，加 `BorrowedMovingSource`（`projectairsim/moving_source.py:14`） | `runtime/moving_observations.py`、`workflows/visual_goal_navigation/workflow.py` |
| `GoalObserver` | `goal_observer.py:9` | `observe(observation, target_description, reference, *, budget, initial_pose, instruction)` | `BBoxGoalObserver`（`planner/workflows/visual_goal_navigation/targeting.py:150`） | `workflows/visual_goal_navigation/workflow.py:41` |
| `StreamGoalSource` | `goal_source.py:8` | `next(state, unwrapped_yaw_deg)` | `QueueFollower` 一族（`runtime/motion/follower.py:75`） | `runtime/motion/loop.py:531` |
| `TargetDetector` | `target_detector.py:10` | `detect(request, *, timeout_s)` | `client.py:52`、`manual.py:43`（`planner/adapters/target_detection/`） | `workflows/visual_goal_navigation/targeting.py` |
| `TaskCompletionChecker` | `task_completion.py:7` | `check(request, *, timeout_s)` | `planner/adapters/task_completion/client.py:21` | `workflows/visual_goal_navigation/completion.py`、`workflow.py` |
| `TaskParser` | `task_parser.py:7` | `parse(instruction)` | `planner/adapters/task_parser/openai_task_parser.py` | `assembly/application.py` |
| `TaskTrace` | `task_trace.py:8` | 属性 `path: Path`，方法 `event`、`observation`、`target_geometry` | `VisualGoalTrace`（`adapters/diagnostics/visual_goal_trace.py:36`） | `workflows/visual_goal_navigation/workflow.py` |
| `TrajectoryPlanner` | `trajectory_planner.py:11` | `plan` 带 12 个参数，最后一个 `timeout_s` | `QwenVlPlanner`（`adapters/trajectory_planner/qwen_vl_planner.py:36`）、`GeometricTargetPlanner`（`algorithms/planning/geometric_planner.py:45`） | `runtime/window/window.py:45`、`runtime/window/inflight.py` |
| `ReturnHome` | `capabilities.py:20` | `return_home(vehicle: str \| None = None) -> WorldPose` | 三个实现都是这一形状：`VehicleRoster.return_home`（`projectairsim/vehicles.py:77`）、`ProjectAirSimDrone.return_home`（`drone.py:178`）、`ProjectAirSimRover.return_home`（`rover.py:308`）；AirSim 无实现 | `simulator/ports.py:54` 作能力位；`/api/home` 走 worker 协议不走本端口 |
| `WeatherControl` | `capabilities.py:26` | `fields`、`presets`、`apply` | `AirSimWeather`（`airsim/weather.py:26`）；`ProjectAirSimMissionSession` 只有 `weather_catalog`/`set_weather`（`projectairsim/session.py:326`、`:338`），名字不同 | `simulator/ports.py:50` |
| `StreamingViews` | `capabilities.py:42` | `view_modes`、`set_view_mode(mode) -> str` | `ProjectAirSimMissionSession.set_view_mode` 返回切换后的视图名（`projectairsim/session.py:344`）；没有 `view_modes`，视图表来自配置的 `VIEW_MODES` | `simulator/ports.py:51` |
| `AppearanceModelControl` | `capabilities.py:51` | `options`、`current`、`request` | 无实现 | `simulator/ports.py:52` 占位 |
| `FramePublisher` | `capabilities.py:66` | `publish(name, jpeg, timestamp_ns)` | 无实现；真实帧通路是 `frame_sink: Callable[[np.ndarray, float], None]` → `runtime/frame_hub.py:29` | `simulator/ports.py:53` 占位 |

## 5 关键流程

端口从配置到运行的解析顺序如下，中途有两套不同的构造方式。

1. 读配置。`planner/config/loader.py:127` 调 `parse_family_catalog`，得到 `FamilyCatalog`；地图到引擎的映射在 `planner/config/maps.py`。
2. 解析引擎模块。`planner/simulator/registry.py:28` 的 `ports_for(simulator)` 按 `RUNTIME_MODULES` 表 import `planner.simulators.<name>.runtime`，调它的 `describe()`。
3. AirSim 走真实对象。`planner/simulators/airsim/runtime.py:28` 的 `build_ports` 把已构造的观测源、车辆、天气对象填进 `SimulatorPorts`（`:45` 起），`PORT_SLOTS`（`:17`）与之一致。
4. ProjectAirSim 走占位对象。`planner/simulators/projectairsim/runtime.py:112` 的 `describe()` 调 `SimulatorPorts.from_slots`（`planner/simulator/ports.py:86`），每个已声明槽位填 `_Slot` 占位（`:106`），只有 `views` 与 `weather` 换成能答 `view_modes`/`fields`/`presets` 的 `_Views`/`_Weather`（`:136` 起）。
5. 能力位推导。`planner/assembly/requirements.py:97` 的 `capabilities_for(ports)` 读 `ports.implemented()` 与 `ACTION_PORTS`（`:32`），产出 `Capabilities`，由 `adapters/http` 写给控制台。缺失槽位对应的动作在 `planner/adapters/http/wiring.py:66` 变成 HTTP 501。
6. 运行期取用。`planner/assembly/mission_assembly.py:229` 用 `GuidedVehicle(vehicle, loop)` 把 `Vehicle` 端口接到控制循环，`:231` 读 `vehicle.tracking_limits`；`runtime/position_hold.py:22` 收 `PositionHoldVehicle`；`runtime/motion/loop.py:530` 的 `run_stream` 收 `StreamGoalSource`；`runtime/continuous_execution.py` 收 `StreamingVehicle`。
7. 实现落在引擎包。三个载具类的 `stream_command` 第一行是 `require_supported`（`airsim/vehicle.py:223`、`drone.py:292`、`rover.py:268`），命令与能力不符即抛 `ValueError`。

## 6 失败模式

| 情形 | 报错或表现 | 调用方处理 |
| --- | --- | --- |
| 槽位为空 | 能力位不出现，`planner/adapters/http/wiring.py:66` 抛 `HTTPException(501)`，`detail.message` 写明该引擎没有实现该端口 | 控制台隐藏按钮，接口返回 501 |
| 端口声明了但实现签名不同 | Python 不做结构检查，调用行才抛 `TypeError`（例如给 `return_home` 传 `vehicle`） | 目前无防护，只有 `tests/test_architecture.py:134` 查方法存在，见 Qports01 |
| 槽位是 `_Slot` 占位 | 只能答 `view_modes`、`fields`、`presets` 三个方法（`planner/simulator/ports.py:108` 起），访问其他属性抛 `AttributeError` | `describe()` 只走 `capabilities_for`，不调用其他方法，因此当前不触发 |
| 端口方法内部失败 | 端口自身不兜底，引擎实现抛 `planner/errors.py` 的 `ExecutionError`/`ProtocolError` 或 `ValueError` | 由 runtime 与 workflows 转换为 `StageResult` |
| 端口类型与命令不符 | `require_supported` 抛 `ValueError`（见 [domain](./domain.md) 第 6 节） | 引擎侧直接抛，属于编程错误，运行期不重试 |

本层没有超时与重试策略：`timeout_s` 只是参数，由实现方（`TrajectoryPlanner`、`TargetDetector`、`TaskCompletionChecker`）决定怎么用。

## 7 测试位置

跑法：`~/miniconda3/envs/vln/bin/python -m pytest -q tests/test_architecture.py tests/test_capability_gating.py tests/test_public_surface.py tests/test_runtime_catalog.py`，本轮 52 passed。

覆盖到的：

- `tests/test_public_surface.py:90` 锁定 18 个导出名，入口多一个少一个都失败。
- `tests/test_architecture.py:120` 要求每个已注册引擎的 `vehicle` 槽非空、`task_kinds` 非空。
- `tests/test_architecture.py:134` 的 `test_airsim_slots_are_backed_by_real_code` 要求 `PORT_SLOTS` 的每个名字都是 `SimulatorPorts` 的字段，并对 `Vehicle`、`StreamingVehicle` 逐个方法做 `hasattr` 检查。
- `tests/test_capability_gating.py` 从 HTTP 侧验证能力位随引擎变化。
- `tests/test_runtime_catalog.py:59`、`:75` 验证两个引擎推导出的能力位内容。

没覆盖的：

- 剩下 16 个端口（`ReturnHome`、`WeatherControl`、`StreamingViews`、`AppearanceModelControl`、`FramePublisher`、`ObservationSource`、`ControlledObservationSource`、`MovingObservationSource`、`GoalObserver`、`PositionHoldVehicle`、`StreamGoalSource`、`TargetDetector`、`TaskCompletionChecker`、`TaskParser`、`TaskTrace`、`TrajectoryPlanner`）没有任何与实现的一致性测试，签名漂移只能靠读代码发现，见 Qports01、Qports02。
- `tests/` 里只有 `tests/test_architecture.py` 直接 import `planner.ports`，其余测试都通过引擎对象间接使用端口。

## 8 与其他层的联动

- 上游：[domain](./domain.md)。端口签名里的每一个类型都来自那里，`planner/ports/*` 不 import 其他层。
- 下游实现方：[simulators](./simulators.md) 提供 `Vehicle`、`StreamingVehicle`、`ObservationSource`、`PositionHoldVehicle`、`WeatherControl` 的实现；[adapters](./adapters.md) 提供 `TaskParser`、`TargetDetector`、`TaskCompletionChecker`、`TrajectoryPlanner`、`TaskTrace`；[runtime](./runtime.md) 提供 `StreamGoalSource` 的实现；[workflows](./workflows.md) 提供 `GoalObserver` 的实现。
- 中介：[assembly](./assembly.md) 通过 `planner/simulator/ports.py` 的 `SimulatorPorts` 打包端口、通过 `requirements.py` 把槽位翻译成能力位。
- 数据形态：端口方法收发的都是 domain 值对象；唯一跨进程的是 `SimulatorPorts` 槽位名（字符串元组），经 worker 协议传到 HTTP 进程。

## 9 疑问与争议

- Qports01 层：ports 结论：`ReturnHome.return_home(self, vehicle: str) -> None` 与三个实现都不符，ProjectAirSim 返回 `str` 且参数可选，两个载具类不接参数、返回 `WorldPose`。依据：`planner/ports/capabilities.py:23`、`planner/simulators/projectairsim/vehicles.py:77`、`planner/simulators/projectairsim/drone.py:178`、`planner/simulators/projectairsim/rover.py:308`。影响：是
- Qports02 层：ports 结论：`StreamingViews.set_view_mode` 声明返回 `bool`（True 立即生效、False 延后），实现返回视图名字符串，`planner/simulator/worker_protocol.py:61` 又是返回 `None`。依据：`planner/ports/capabilities.py:47`、`planner/simulators/projectairsim/session.py:344`。影响：是
- Qports03 层：ports 结论：`Vehicle` 端口早期缺 `tracking_limits`，装配层直接按引擎属性访问。现持有 `Vehicle.tracking_limits` 声明与三个引擎实现一致；`attach_loop` 已从端口与引擎一并删除，段执行改由 `GuidedVehicle` 兑现。依据：`planner/ports/vehicle.py:12`、`planner/assembly/mission_assembly.py:216-226`。影响：否
- Qports04 层：ports 结论：`hover` 对阿克曼车是刹车，端口名与语义不符。依据：`planner/ports/vehicle.py:27`、`planner/simulators/projectairsim/rover.py:178`。影响：否
- Qports05 层：ports 结论：`AppearanceModelControl` 没有任何实现，ProjectAirSim 仍声明 `models` 槽位，能力位因此为真，实际操作走 `assembly/controller` 的 `ModelSwitcher`。依据：`planner/ports/capabilities.py:51`、`planner/simulators/projectairsim/runtime.py:20`、`planner/simulator/ports.py:95`、`planner/assembly/controller/commands.py:139`。影响：是
- Qports06 层：ports 结论：`FramePublisher` 没有实现，真实帧通路的签名是 `publish(bgra, timestamp_s)`，与端口的 `publish(name, jpeg, timestamp_ns)` 不同，`/api/video` 的门禁靠槽位名而不是对象能力。依据：`planner/ports/capabilities.py:69`、`planner/runtime/frame_hub.py:29`、`planner/simulators/projectairsim/frames.py:53`。影响：是
- Qports07 层：ports 结论：18 个端口里只有 `Vehicle` 与 `StreamingVehicle` 有一致性测试，且只查方法存在、不查签名，其余 16 个端口的实现没有校验。依据：`tests/test_architecture.py:152` 起到 `:159`。影响：是
- Qports08 层：ports 结论：`WeatherControl.fields()`/`presets()` 与 domain 的 `WEATHER_FIELDS`/`WEATHER_CATALOG` 是同一份数据的两个来源，ProjectAirSim 直接把 domain 常量回传，AirSim 回传配置里的拼写映射。依据：`planner/ports/capabilities.py:35`、`planner/domain/weather.py:15`、`planner/simulators/projectairsim/runtime.py:28`。影响：是
- Qports09 层：ports 结论：`PositionHoldVehicle` 只有两个无人机实现，阿克曼车没有 `hold_velocity`，端口注释没有说明车不支持该能力。依据：`planner/ports/position_hold_vehicle.py:7`、`planner/simulators/projectairsim/drone.py:298`、`planner/simulators/projectairsim/rover.py`（无该方法）。影响：是

## 10 已知缺口

能力矩阵里另有四个端口在两套引擎上都没有实现。按现状列出。

### 10.1 声明与实现已收口（M3）

`ReturnHome.return_home`、`StreamingViews.set_view_mode`、`Vehicle.tracking_limits`、`Vehicle.hover`（改名为 `hold`）四处已统一成一份，`tests/test_architecture.py::test_port_signatures_match_their_implementations` 逐个端口比对参数名、默认值与两侧都标注了的返回类型。`Vehicle.attach_loop` 已从端口与引擎一并删除，`execute_segment` 对引擎类不再比对，只由 `tests/test_architecture.py::test_airsim_slots_are_backed_by_real_code` 查 `GuidedVehicle` 上有这个方法。未覆盖的仍是第 10.2 节列出的无实现端口。

### 10.2 能力矩阵（18 个端口 × UE5 / UE4）

「有实现」指存在实现了端口方法的对象；「无实现」指没有对象提供这些方法；「仅测试」指除 `planner/simulator/ports.py` 的声明与 `tests/` 的名单检查外没有调用点。`capabilities.py` 里的五个端口分五行列出。

| 端口 | 文件 | UE5（projectairsim） | UE4（airsim） | 备注 |
| --- | --- | --- | --- | --- |
| `Vehicle` | `vehicle.py` | 有实现 | 有实现 | 无人机与车各一个类 |
| `StreamingVehicle` | `streaming_vehicle.py` | 有实现 | 有实现 | 同上三个类 |
| `PositionHoldVehicle` | `position_hold_vehicle.py` | 有实现 | 有实现 | 只无人机实现，车没有 `hold_velocity` |
| `ObservationSource` | `observation_source.py` | 有实现 | 有实现 | — |
| `ControlledObservationSource` | `observation_source.py` | 无实现 | 有实现 | UE5 声明了 `controlled_capture` 槽位，但观测源没有 `request_capture` |
| `MovingObservationSource` | `observation_source.py` | 有实现 | 有实现 | UE5 另有 `BorrowedMovingSource` 包装 |
| `GoalObserver` | `goal_observer.py` | 有实现 | 有实现 | 由 workflows 提供，与引擎无关 |
| `StreamGoalSource` | `goal_source.py` | 有实现 | 有实现 | 由 runtime 的 `QueueFollower` 一族提供 |
| `TargetDetector` | `target_detector.py` | 有实现 | 有实现 | 由 adapters 提供 |
| `TaskCompletionChecker` | `task_completion.py` | 有实现 | 有实现 | 由 adapters 提供 |
| `TaskParser` | `task_parser.py` | 有实现 | 有实现 | 由 adapters 提供 |
| `TaskTrace` | `task_trace.py` | 有实现 | 有实现 | 由 adapters 的诊断轨迹提供 |
| `TrajectoryPlanner` | `trajectory_planner.py` | 有实现 | 有实现 | 模型与几何两个实现都在共享层 |
| `ReturnHome` | `capabilities.py` | 有实现 | 无实现 | 方法名存在、签名不符；端口本身无调用点 |
| `WeatherControl` | `capabilities.py` | 无实现 | 有实现 | UE5 只有 `weather_catalog`/`set_weather`，名字与端口不同 |
| `StreamingViews` | `capabilities.py` | 有实现 | 无实现 | 有 `set_view_mode`、无 `view_modes` |
| `AppearanceModelControl` | `capabilities.py` | 无实现 | 无实现 | 仅测试：`options`/`current`/`request` 无人调用，实际切换走 `planner/assembly/controller/models.py` |
| `FramePublisher` | `capabilities.py` | 无实现 | 无实现 | 仅测试：真实帧通路是 `frame_sink` 回调，不经过本端口 |
