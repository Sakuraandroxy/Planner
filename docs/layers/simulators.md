> 上级：[项目架构](../ARCHITECTURE.md)
> 同层兄弟：[domain](domain.md)、[ports](ports.md)、[algorithms](algorithms.md)、[runtime](runtime.md)、[workflows](workflows.md)、[assembly](assembly.md)、[adapters](adapters.md)、[simulator](simulator.md)

# simulators：引擎实现（UE5 ProjectAirSim / UE4 AirSim）

## 1 一句话职责

本层是全项目唯一允许 import 仿真器客户端库的地方，把 `planner.ports` 的端口协议落到两套引擎上：UE5 的 `projectairsim` 包（双载具 rover+drone）与 UE4 的 `airsim` 包（单无人机）。

## 2 边界

管这些：

- 引擎连接、时钟推进、场景加载：`planner/simulators/projectairsim/connection.py:72`（pynng 单消费者链路 + 步进/自由时钟）、`planner/simulators/airsim/connection.py:51`（msgpack RPC）。
- 载具端口：位姿、运动能力、`stream_command`、碰撞、停稳。
- 观测端口：相机取帧、深度标定、成对时间戳校验。
- 天气、切换视图、载具激活、返回原点：只有 UE5 声明这些槽位。
- 端口声明与能力推导的输入：`planner/simulators/airsim/runtime.py:17-81`、`planner/simulators/projectairsim/runtime.py:12-81`。

不管这些：

- 进程、xpra、worker 协议、子进程：见 [simulator](simulator.md)。
- 任务流程与运动律（`ControlLoop`、`PositionHold`、队列跟随）：见 [runtime](runtime.md)、[workflows](workflows.md)。
- 端口 Protocol 的形状：见 [ports](ports.md)。
- 能力位怎么由槽位推出来：`planner/assembly/requirements.py:97`，见 [assembly](assembly.md)。

## 3 输入与输出

### 入口

| 入口 | 位置 | 说明 |
| --- | --- | --- |
| `airsim.runtime.build_session(*, config, map_entry, family, resources, log)` | `airsim/runtime.py:55` | 建 `AirSimMissionSession`，不读 `map_entry`/`resources` |
| `projectairsim.runtime.build_session(...)` | `projectairsim/runtime.py:33` | 用 `map_entry.scene` 覆盖 config 后建 `ProjectAirSimMissionSession` |
| `describe()` | `airsim/runtime.py:73`、`projectairsim/runtime.py:74` | 声明式端口 bundle |
| `AirSimMissionSession.run/idle_tick/close/set_weather/mission_inputs/attach_arms` | `airsim/session.py:188`、`:232`、`:246`、`:240`、`:148`、`:176` | worker 的鸭子协议 |
| `ProjectAirSimMissionSession.run/idle_tick/close/set_weather/set_view_mode/commit` 等 | `projectairsim/session.py:255`、`:316`、`:446`、`:338`、`:344` | 同上，另加视图与载具切换 |
| `<Vehicle>.stream_command(command, duration_s)` | `projectairsim/rover.py:267`、`projectairsim/drone.py:291`、`airsim/vehicle.py:222` | 引擎侧唯一数据面动作 |
| `VehicleRoster.activate_vehicle/return_home` | `projectairsim/vehicles.py:107`、`:77` | 控制台的载具切换与回原点 |

### 契约类型

读：`MotionCommand`、`MotionTrajectory`、`MotionSegment`、`MotionLimits`、`MotionCapability`、`ExecutionBudget`、`ContinuousTrackingSettings`、`PoseCheckpoint`、`WorldPose`、`TimedVehicleState`、`Observation`、`CollisionState`、`MissionResult`、`TaskSubmission`、`MissionInputs`、`VehicleArm`。

写：`Observation`、`TimedVehicleState`、`CollisionState`、`MissionResult`、`SimulatorPorts`。

### 返回与异常

| 异常 | 抛出点 | 语义 |
| --- | --- | --- |
| `ExecutionError` | `rover.py:143-148`、`:286`、`drone.py:135-140`、`:161`、`airsim/vehicle.py:79`、`:89`、`:173`、`:184`、`projectairsim/flight.py:76`、`:97` | 引擎拒绝、未 `start()`、端点没走到、命令类型不符 |
| `SimulatorConnectionError` | `projectairsim/connection.py:119-124`、`:281`、`airsim/connection.py:96`、`airsim/server_vehicle.py:40` | 连接或传输失败、时钟停滞 |
| `ProtocolError` | `projectairsim/weather.py:29`、`:33`、`:40`、`projectairsim/session.py:246`、`:373`、`airsim/session.py:181` | 引擎拒绝天气/视图、装配缺臂 |
| `ObservationError` | `projectairsim/observation.py:218`、`airsim/observation_source.py:229` | 图像对时间戳或相机位姿不配 |
| `BudgetExceeded` | `projectairsim/observation.py:112`、`airsim/observation_source.py:80`、`airsim/observation_gate.py:118` | 预算在取帧期间耗尽 |
| `RuntimeError` | `projectairsim/connection.py:89`、`:196-206`、`projectairsim/session.py:345-346` | 连接已开、没加载过场景、会话已关 |

## 4 对外名字表

| 名字 | 类型 | 一句话用途 | 层外调用点 |
| --- | --- | --- | --- |
| `PORT_SLOTS` | 常量元组 | 声明槽位 | `planner/simulator/registry.py:35` 间接经 `describe()` |
| `describe()` | 函数 | 不建客户端的能力报告 | `planner/simulator/registry.py:35` |
| `build_session()` | 函数 | 建会话 | `planner/simulator/registry.py:67` |
| `AirSimMissionSession` | 类 | UE4 会话 | `airsim/runtime.py:70` |
| `ProjectAirSimMissionSession` | 类 | UE5 会话 | `projectairsim/runtime.py:66` |
| `AirSimVehicle` | 类 | UE4 载具端口 | `airsim/server_vehicle.py:33` |
| `ProjectAirSimRover` / `ProjectAirSimDrone` | 类 | UE5 两个载具端口 | `projectairsim/session.py:139`、`:153` |
| `AirSimObservationSource` | 类 | UE4 取帧 | `airsim/session.py:108`、`:119` |
| `ProjectAirSimObservationSource` | 类 | UE5 取帧 | `projectairsim/rover.py:110`、`drone.py:104` |
| `AirSimWeather` | 类 | UE4 天气 | `airsim/session.py:145` |
| `apply_weather` | 函数 | UE5 天气 | `projectairsim/session.py:340` |
| `VehicleRoster` | mixin | 隐藏/恢复/激活 | `projectairsim/session.py:63` |
| `ObservationGate` | 类 | UE4 取帧闸门 | `airsim/session.py:85`、`runtime/motion/loop.py:419` |
| `ServerVehicle` | 类 | UE4 就绪与位置保持绑定 | `airsim/session.py:64` |
| `CameraTopicStream` | 类 | UE5 topic 帧缓存 | `projectairsim/observation.py:96` |
| `CollisionMonitor` | 类 | UE5 碰撞订阅 | `projectairsim/rover.py:117`、`drone.py:109` |
| `AsyncBridge` | 类 | UE5 协程执行桥 | `projectairsim/drone.py:101` |
| `CameraApiRecorder` | 类 | UE4 录像 | 无生产调用者，见 Qeng2 |

## 5 关键流程

### 5.1 UE5 会话建立（`projectairsim/session.py:64-227`）

1. 建 `ProjectAirSimConnection` 并 `connect(wait_s=max(10, timeout_s))`，失败重试到超时（`connection.py:87-126`）。
2. `add_advance_hook(self._pin_hook)`：每次推进后钉住隐藏载具（`session.py:120`、`vehicles.py:265-293`）。
3. `load_scene()`：场景超过普通请求超时，临时把 `recv_timeout` 抬到 ≥60 s，然后 `world.pause()`（`connection.py:129-163`）。
4. 建 `ProjectAirSimRover` 与 `ProjectAirSimDrone`，各自 `start()`（enable_api_control + arm；`rover.py:142-150`、`drone.py:133-143`）。
5. `_hide_quiet("drone")`：场景开局两台机器人同点，必须先让一台离场（`session.py:170`）。
6. 预热取一帧给空闲观众，然后 `measure_resting_pose()` 量出车的静止高度并 `set_home_pose`（`session.py:174-200`、`rover.py:195-226`）。
7. `_hide_inactive_at_startup()` 按默认载具复位可见性，`_apply_initial_view()` 把视图切到默认载具的 chase 相机（`session.py:204-207`、`:383-397`）。
8. `resume_clock()`：先试自由时钟，1 s 内没推进就退回步进并打 WARNING（`connection.py:165-194`）。
9. 任一步抛异常都 `self.close()` 后重抛（`session.py:224-226`）。

### 5.2 UE4 会话建立（`airsim/session.py:54-147`）

1. 建 `AirSimConnection(host, ports["rpc"], timeout_s, motion_timeout_s=move_timeout_s+2)`（`session.py:71-78`）。
2. 建 `ServerVehicle`，它内部建 `AirSimVehicle` 与 `PositionHold`，并把 hold 挂回载具（`server_vehicle.py:23-36`）。
3. `ExecutionService(control)` 绑定位置保持检查点与取帧闸门的 `service_requests`/`cancel_requests`（`session.py:93-101`）。
4. 建 `AirSimObservationSource`，带 `observation_gate`、`verify_image_pair=True`（`session.py:103-116`）。
5. `build_capture_source()` 每次新建一条只读 `AirSimConnection` + 观测源，登记进 `_capture_connections`（`session.py:119-139`）。
6. `AirSimWeather` + `build_ports(...)` 得到端口 bundle（`session.py:145-150`）。
7. `run()` 里才第一次飞：`prepare(takeoff_if_landed=True, ground_z=…)`，地面高度取会话见过的最大 z（UE 坐标 z 向下）（`session.py:213-227`）。

### 5.3 段执行一拍

- UE5：`stream_command` 校验命令类型（`require_supported`）→ 有限性 → 算控制量 → `advance_for_sim_time(duration_s)`（`rover.py:267-296`、`drone.py:291-296`）。rover 走 `controls_from_command` 把速度与角速度换成 engine/steering/brake（`drive.py:175-199`）；drone 走 `commands.move_by_velocity`，由 `AsyncBridge` 在独立线程跑一个 asyncio 事件循环（`commands.py:48-99`、`async_bridge.py:21-56`）。
- UE4：`AirSimVehicle.stream_command` → `hold_velocity` → `connection.call_streaming("moveByVelocityAsync", …)`，只提交 Future，不推进时钟，靠后续 state RPC 泵 I/O（`airsim/vehicle.py:222-239`、`airsim/connection.py:119-137`）。

### 5.4 停稳取帧

- UE5 rover：`capture` 在暂停态下按 50 ms 步进推进时钟，直到收到新的一对帧（`observation.py:107-132`）。
- UE5 drone：`capture` → `capture_settled` → `_settle_capture`，先等 `PositionHold.wait()` 再取帧（`drone.py:310-322`、`observation.py:134-149`）。
- UE4：`AirSimObservationSource.capture` → `ObservationGate.capture` 上下文（`begin_observation_hold` → `wait_for_observation`）→ `_capture_held`；`_capture_held` 把取图丢进 `BackgroundOperation`，在等待期间反复 `keep_holding()`，并检查 `holding.observable` 与 `holding.contains_pose(result.pose)`，不满足就 recapture（`airsim/observation_source.py:76-136`、`observation_gate.py:76-96`）。

### 5.5 载具切换（只有 UE5）

`activate_vehicle`：`_park(other)` → `_hide(other)` → `_recover(target)` → 先把 `_active_vehicle` 改到目标 → `return_home` → `set_view_mode(default_view_for(target))`；任一步失败回滚活动载具并把目标重新藏起来（`vehicles.py:107-156`）。隐藏是向上平移 `HIDE_ALTITUDE_GAIN_M=1000.0`，并由 `_pin_hidden` 每 `HIDE_PIN_PERIOD_S=2.0` 仿真秒重贴一次（`vehicles.py:38-42`、`:211-293`）。

### 5.6 天气

- UE5：`validate_weather_values` → `reset_weather_effects` → 全零则 `disable_weather_effects`，否则 `enable` 后逐字段 `set_weather_visual_effects_param`（`weather.py:17-51`）。
- UE4：`simEnableWeather(any(v>0))` 后逐字段 `simSetWeatherParameter(PARAMETER_IDS[spelling], v)`（`airsim/weather.py:40-61`）。

### 5.7 两引擎兑现差异对照

同一组端口槽位在两套引擎上的实际能力不同，控制台靠 `Capabilities` 区分，代码里没有任何 `if simulator == ...`。

| 维度 | UE5 `projectairsim` | UE4 `airsim` | 依据 |
| --- | --- | --- | --- |
| 声明槽位 | vehicle、observation、streaming、controlled_capture、moving_capture、weather、views、models、frames、home | vehicle、observation、streaming、moving_capture、weather | `projectairsim/runtime.py:12-23`、`airsim/runtime.py:17-23` |
| 载具 | `rover` + `drone` 双载具，可切换 | 只有 `drone` | `projectairsim/runtime.py:25`、`airsim/runtime.py:25`、`airsim/session.py:194-199` |
| 运动模型 | rover `ACKERMANN`（最小转弯半径由标定反算）、drone `HOLONOMIC` | `HOLONOMIC` | `rover.py:56-59`、`:228-231`、`drone.py:80-87`、`airsim/vehicle.py:41-47` |
| 时钟 | 可自由也可步进；`stream_command` 末尾自己推进 | 引擎自走；`stream_command` 只提交 Future | `connection.py:165-252`、`rover.py:295`、`airsim/vehicle.py:230-239` |
| 流式命令落地 | rover `SetRoverControls`（engine/steering/brake）；drone `MoveByVelocity` 走 `AsyncBridge` | `moveByVelocityAsync`，`drivetrain=MaxDegreeOfFreedom`，`is_rate=True`，时长取 `max(duration, 0.2)` | `rover.py:387-398`、`drive.py:175-199`、`commands.py:48-99`、`airsim/vehicle.py:230-239` |
| 相机来源 | topic 订阅（`CameraTopicStream`，RGB `BGR` + 深度 `16FC1`） | 拉取式 `simGetImages`（Scene + DepthPerspective） | `projectairsim/frames.py:29-30`、`:94-124`、`airsim/observation_source.py:156-180` |
| 相机数量 | 三机位 + 流式视图环（`VIEW_MODES`） | 单相机 id，无视图切换 | `projectairsim/observation.py:44-60`、`session.py:344-373`、`airsim/runtime.py:17-23` |
| 取帧并发 | 单连接、单订阅，rover 与 drone 各一条 topic | 每个 worker 一条只读 `AirSimConnection`，像素不进控制锁 | `projectairsim/connection.py:337-345`、`airsim/session.py:119-139` |
| 停稳保证 | `capture_settled` + `PositionHold` | `ObservationGate` + `PositionHold.observable`/`contains_pose` | `projectairsim/observation.py:134-149`、`airsim/observation_gate.py:76-96`、`airsim/observation_source.py:103-136` |
| 深度内参 | 与 RGB 同一内参，`depth_intrinsics=self.intrinsics` | 单独按深度响应尺寸算 `depth_intrinsics`，可配 `depth_horizontal_fov_deg` | `projectairsim/observation.py:200-205`、`airsim/observation_source.py:203-212` |
| 天气字段名 | canonical 与引擎同名（8 个） | `road_wetness` → `roadwetness`，靠 `WEATHER_FIELDS` + `PARAMETER_IDS` | `projectairsim/runtime.py:28-30`、`airsim/runtime.py:87-96`、`airsim/weather.py:14-23` |
| 天气开关 | `reset` → `enable`/`disable` + 逐字段 `set` | `simEnableWeather(any>0)` + 逐字段 `simSetWeatherParameter` | `projectairsim/weather.py:26-41`、`airsim/weather.py:47-60` |
| 返回原点 | 有：teleport 到测得的地面位姿 | 无 home 槽位；关闭时降落 | `projectairsim/vehicles.py:77-105`、`airsim/runtime.py:17-23` |
| 载具激活 | 有：park → hide → home → 切视图 | 无 | `projectairsim/vehicles.py:107-156` |
| 碰撞 | topic 订阅 + 拉取式 latch（`consume`） | 每次 `collision_state()` 现查 | `projectairsim/collision.py:86-141`、`airsim/vehicle.py:205-207` |
| 着陆判定 | 控制器 flag 或 `z >= ground_z-0.1` | 控制器 flag 或 `z >= ground_z-0.25`，注释明确 `landed_state` 不可信 | `projectairsim/drone.py:339-350`、`airsim/session.py:49-52`、`airsim/server_vehicle.py:15-21` |
| 子进程 | 主进程内（线程 worker） | 独立解释器（`runtime_python`），见 [simulator](simulator.md) | `planner/simulator/subprocess_worker.py:1-20` |

### 5.8 三个引擎实现里实际没被调用过的方法

静态扫描 `planner/` 全域（排除定义行）的结果，以下公开方法在生产代码里零调用点：

| 方法 | 依据 | 说明 |
| --- | --- | --- |
| `AirSimVehicle.execute_basic_motion` | `airsim/vehicle.py:123`，仅 `tests/test_airsim_vehicle.py` 调用 | 段执行已改由 `ControlLoop` 走 `stream_command`；`_await_arrival`（`:89`）只被它调用 |
| `ProjectAirSimRover.execute_segment` | `projectairsim/rover.py:164`，仅测试 | 过渡期委派残留，`attach_loop` 注入了 `_loop` 但没人调这个入口 |
| `ProjectAirSimDrone.execute_segment` | `projectairsim/drone.py:198`，仅测试 | 同上 |
| `AirSimVehicle.execute_segment` | `airsim/vehicle.py:74`，仅测试 | 同上 |
| `ProjectAirSimDrone.hold_tick` | `projectairsim/drone.py:238` | 只有 UE4 的 `ObservationGate` 会用 `getattr(vehicle,"hold_tick")`；UE5 不走那个闸门 |
| `ProjectAirSimDrone.begin_observation_hold` | `projectairsim/drone.py:254` | 同上 |
| `ProjectAirSimDrone.wait_hold` | `projectairsim/drone.py:261` | 无调用者（UE4 侧用的名字是 `wait_for_observation`） |
| `ProjectAirSimConnection.get_weather_parameters` | `projectairsim/connection.py:323` | 从未用于校验"天气真的设上了" |
| `CameraTopicStream.last_delivered_ns` | `projectairsim/frames.py:89` | 无调用者 |
| `CameraApiRecorder`（整个类） | `airsim/recording.py:18`，只在 `airsim/__init__.py:3` 导出 | 生产路径无实例化；`last_error`（`:40`）同样没用 |
| `AirSimObservationSource.request_capture` | `airsim/observation_source.py:96` | 它转调 `ObservationGate.request_capture`，而 `planner/ports/observation_source.py:18` 声明的 `request_capture` 没有任何上层调用者 |
| `ProjectAirSimMissionSession.view_mode` | `projectairsim/session.py:331` | 控制台读的是控制器自己的 `_view_mode`，这个属性没被任何地方读 |
| `ProjectAirSimRover.last_collision` / `ProjectAirSimDrone.last_collision` | `rover.py:243`、`drone.py:279` | 只在诊断里可能有用，当前无读取者 |

## 6 失败模式

| 报错 | 触发条件 | 调用方怎么处理 |
| --- | --- | --- |
| `ExecutionError("rover/drone API control is not enabled; call start() first")` | 会话构造未完就下发命令（`rover.py:381-385`、`drone.py:325-329`） | 装配顺序错误，不是运行时抖动 |
| `ExecutionError("rover refused API control"/"refused arm")` | 引擎拒绝启用 API 或拒绝 arm（`rover.py:143-148`、`drone.py:135-140`） | 会话构造失败，整个栈要重启 |
| `ValueError` from `require_supported` | 命令类型与 `motion_capability()` 不符（`domain` 侧实现，调用点在 `rover.py:268`、`drone.py:292`、`airsim/vehicle.py:223`） | 上游算错了命令类型，一拍就该炸 |
| `ExecutionError("non-finite streaming command or invalid duration")` | 速度或时长非有限、`duration_s<=0`（`rover.py:270-286`、`commands.py:56-70`） | 同上 |
| `ExecutionError("ProjectAirSim velocity command was interrupted")` | `MoveByVelocity` 返回非 `True`（`commands.py:92-95`） | 一拍失败；上层按节拍继续或终止 |
| `SimulatorConnectionError("simulator clock did not advance …")` | 自由时钟 2 s 内没动（`connection.py:253-286`） | 视为引擎卡死 |
| `SimulatorConnectionError("AirSim transport failed (…)")` | msgpack-rpc 传输异常（`airsim/connection.py:84-98`） | 置 `connection_lost`，会话不可复用 |
| `ExecutionError("AirSim takeoff did not lift the vehicle …")` | 爬升 < `MIN_CLIMB_M=0.5`（`airsim/connection.py:20`、`:184-199`） | 引擎侧真实失败；消息带实测爬升 |
| `ObservationError("RGB/depth … not registered")` | 两路图像的相机位姿或时间戳不对（`airsim/observation_source.py:226-241`、`projectairsim/observation.py:209-220`） | 取帧失败，阶段重试 |
| `ObservationError("no synchronized RGB/depth frame arrived …")` | 超时内没等到成对帧（`projectairsim/observation.py:126-131`） | 取帧超时；相机或时钟有问题 |
| `ProtocolError("simulator refused to switch the streaming view")` | `switch_streaming_view()` 返回假（`projectairsim/session.py:367-373`） | 视图切换失败，`/api/view` 回错误 |
| `RuntimeError("another ProjectAirSim connection is active in this process")` | 上一个连接没释放（`connection.py:344-359`） | 进程级单消费者限制，只能重启进程 |
| `RuntimeError("the ProjectAirSim session is closed")` | 关闭后仍调天气/视图（`weather.py:24-25`、`session.py:345-346`） | 控制器状态错误 |

会话 `close` 的路径把异常降级为 WARNING 并继续释放传输：`session.py:446-508`（UE5）、`session.py:246-287`（UE4）。UE4 的 `close` 先降落并等高度回到地面记录，拿这个测量作为 `landed_on_ground` 传给 `connection.shutdown`（`airsim/session.py:256-287`、`:288-310`）。

## 7 测试位置

覆盖到的：

- UE5：`tests/test_projectairsim_session.py`（构造顺序、载具切换回滚）、`tests/test_projectairsim_rover.py`（转向标定、teleport 容差、静止高度）、`tests/test_projectairsim_drone.py`（起飞/降落/`_is_landed`）、`tests/test_projectairsim_connection.py`（重连、时钟两种模式）、`tests/test_projectairsim_streaming.py`、`tests/test_projectairsim_geometry.py`、`tests/test_projectairsim_drive.py`。
- UE4：`tests/test_airsim_session.py`、`tests/test_airsim_vehicle.py`、`tests/test_airsim_connection.py`、`tests/test_airsim_observation.py`、`tests/test_airsim_server_vehicle.py`、`tests/test_airsim_port_names.py`（用 AST 校验 `getattr` 用到的属性名真在目标类上）。
- 横切：`tests/test_architecture.py:134-160`（`PORT_SLOTS` 名字必须是 `SimulatorPorts` 的字段；`Vehicle`/`StreamingVehicle` 的方法必须在三个载具类上存在）、`tests/test_camera*`/`test_intrinsics_validation.py`、`tests/test_motion_contract.py`。

没覆盖的空白：

- `CameraTopicStream`（`projectairsim/frames.py`，211 行）没有专门测试文件，只在 `test_projectairsim_streaming.py` 里被间接覆盖。
- `AsyncBridge` 的超时与 `close` 路径没有直接测试。
- `airsim/recording.py`（190 行）没有任何测试，也没有生产调用者。
- `ProjectAirSimCommands.run_native`（`commands.py:100-137`）的 pump 循环只在测试里用替身跑过。
- 第 5.8 节列出的未接线方法都没有测试断言其"不该被调用"。

## 8 与其他层的联动

| 方向 | 对象 | 形态 |
| --- | --- | --- |
| 上游 | `planner.assembly` | `MissionInputs`、`VehicleArm`、`require_application`（`projectairsim/session.py:30`、`airsim/session.py:30`）；`require_application` 决定任务真的跑不跑 |
| 上游 | `planner.simulator` | `SimulatorPorts`、`TaskSubmission`（`airsim/session.py:32`、`projectairsim/session.py:33`） |
| 上游 | `planner.runtime` | `PositionHold`、`ExecutionService`、`SMOOTH_PROFILE`、`BackgroundOperation`、`TrajectoryHub` 形状的 sink |
| 下游 | `planner.domain` | 位姿/状态/观测/碰撞/能力位全部用 domain 类型，引擎不自定义契约 |
| 下游 | `planner.algorithms` | rover 用 `passed_lateral_corridor`，提示词层用 depth 算法（后者不在本层） |
| 引擎依赖 | `projectairsim`（pynng，单消费者 topic 链路） | `connection.py:36-40`、`:129-163` |
| 引擎依赖 | `airsim`（msgpack-rpc 0.5.6 + tornado） | `airsim/connection.py:54-72`、`airsim/rpc_loop.py:40-54` |

## 9 疑问与争议

- Qeng1 层：simulators 结论：`airsim` 与 `projectairsim` 的 `describe()` 与真实 `build_ports` 之间没有比对；UE4 有 `build_ports`（`airsim/runtime.py:28`），UE5 完全由 `session.py` 手工填端口、没有任何构造期比对，`PORT_SLOTS` 是唯一声明。依据：`airsim/runtime.py:28-52`、`projectairsim/runtime.py:74-81`、`projectairsim/session.py:79-227`。影响：是（simulator 与 assembly 文档）。
- Qeng2 层：simulators 结论：`airsim/recording.py` 190 行整模块无生产调用者，只在包 `__init__` 里导出。依据：`planner/simulators/airsim/recording.py:18`、`planner/simulators/airsim/__init__.py:3-6`。影响：否。
- Qeng3 层：simulators 结论：三个载具类都还留着 `execute_segment` + `_loop` 这套过渡委派，生产零调用，`attach_loop` 注入的 loop 只被测试的 `execute_segment` 用到。依据：`airsim/vehicle.py:70-85`、`projectairsim/rover.py:160-173`、`projectairsim/drone.py:194-215`、`planner/assembly/mission_assembly.py:226-229`。影响：是（assembly 的注入点）。
- Qeng4 层：simulators 结论：`AirSimVehicle.execute_basic_motion` 与它的 `_await_arrival` 已被 `ControlLoop` 取代，`airsim/vehicle.py` 里仍有 60 行无人调用的运动实现，包含 `airsim.DrivetrainType` 的硬编码。依据：`airsim/vehicle.py:89-170`。影响：否。
- Qeng5 层：simulators 结论：`ProjectAirSimDrone` 上有三个方法（`hold_tick`、`begin_observation_hold`、`wait_hold`）是为 UE4 那套 `ObservationGate` 协议准备的，UE5 不走闸门，方法永久悬空。依据：`projectairsim/drone.py:238-262`、`airsim/observation_gate.py:84-88`、`:145-151`。影响：否。
- Qeng6 层：simulators 结论：`ObservationSource.request_capture` 这条契约（停稳采集 + Future 语义）在两套引擎里都实现了，但没有任何上层调用者；`controlled_capture` 槽位只有在 `ports.py` 与 `from_slots` 里出现，UE4 的 `build_ports` 没有填它。依据：`planner/ports/observation_source.py:18`、`airsim/observation_source.py:96-101`、`airsim/runtime.py:45-52`、`planner/simulator/ports.py:48`。影响：是（ports 与 runtime 文档）。
- Qeng7 层：simulators 结论：`ProjectAirSimMissionSession.view_mode` 属性没有任何读取者，控制台的视图状态由控制器自己维护，同一个"当前视图"存在两份真相。依据：`projectairsim/session.py:331-333`、`planner/assembly/controller/state.py:50`、`:188-218`。影响：是（assembly 文档）。
- Qeng8 层：simulators 结论：UE5 的隐藏机制是"向上平移 1000 m + 每 2 仿真秒重贴"，两个常量没有配置入口，且注释里的量测（水平 300 m 会插进墙里、约 700 次/分钟接触）在仓库里没有对应脚本或数据。依据：`projectairsim/vehicles.py:36-42`、`:265-293`。影响：否。
- Qeng9 层：simulators 结论：`ProjectAirSimConnection` 的"进程内只允许一个连接"用模块级全局变量实现，测试若并行建两个会话会互相干扰，且异常信息让调用方以为需要重启进程而不是先 `close`。依据：`projectairsim/connection.py:44-46`、`:337-352`、`:196-206`。影响：否。
- Qeng10 层：simulators 结论：`airborne`/着陆判定在两套引擎里各有一套阈值（0.1 m 与 0.25 m），且都是"常量写死在类里"，没有共享常量也没有出处记录，两处注释都只说"实测"。依据：`projectairsim/drone.py:339-350`、`airsim/session.py:49-52`、`airsim/server_vehicle.py:15-21`。影响：否。
- Qeng11 层：simulators 结论：UE4 的 `AirSimVehicle.hold_velocity` 把命令时长取 `max(duration_s, 0.2)`，而上层按 `1/control_hz` 传 0.05 s 量级的值，实际下发的时长被静默放大 2-20 倍，拍长与下发时长不一致。依据：`airsim/vehicle.py:34-36`、`:230-239`。影响：是（runtime 的拍长语义）。
- Qeng12 层：simulators 结论：`projectairsim/session.py` 在 `run()` 里会 `_park`+`_hide` 切换活动载具，但 `close()` 不恢复隐藏状态；同一进程内重建会话时，上一会话隐藏的载具状态只存在于已销毁对象里，靠新会话的 `_hide_inactive_at_startup` 兜。依据：`projectairsim/session.py:262-279`、`:446-508`。影响：否。
- Qeng13 层：simulators 结论：`airsim/session.py:288-310` 的 `_land_and_confirm` 用最多 60 s 的 0.5 s 轮询等高度回落，期间没有任何 stop event 检查，任务取消后关闭会话仍要阻塞几十秒。依据：`airsim/session.py:288-310`、`planner/simulator/worker.py:154-165`。影响：是（simulator 的关闭超时）。
