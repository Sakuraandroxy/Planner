# 端口实现与引擎实现：文件清单与内容

本层只做两件事：把模型 wire 格式变成 `domain/` 的契约对象，把契约层的 `MotionCommand` 变成引擎的一条原生命令。任务判断、运动律、控制权、控制率都不在这里。引擎侧在重构后只剩执行器与读侧，两个 `tracker` 的控制循环进 `runtime/motion/`。

## 一、本层职责与文件清单

估计行数按目标状态，来源标注现状行数。

### `planner/adapters/`

| 文件 | 类与函数签名 | 估计行数 | 从哪来 |
| --- | --- | --- | --- |
| `adapters/task_parser/openai_task_parser.py` | `OpenAITaskParser.parse(instruction: str) -> MissionPlan` | 177 | 现有，不改 |
| `adapters/task_parser/prompt.py` | `task_parser_system_prompt(supported_kinds, *, codecs=None, aerial=False) -> str` | 83 | 现有，不改 |
| `adapters/task_parser/response_schema.py` | `parse_json_object`、`validate_mission_envelope` | 62 | 现有，不改 |
| `adapters/task_parser/task_factory.py` | `mission_from_dict(data, factory) -> MissionPlan` | 54 | 现有，不改 |
| `adapters/task_parser/tasks/{__init__,basic_motion,free_navigation,visual_goal_navigation}.py` | `TaskCodec`、`builtin_task_codecs(*, aerial=False)` | 73/108/45/65 | 现有，不改；载具差异（`aerial` 开关、地面解码器无竖直字段）留在这里 |
| `adapters/target_detection/client.py` | `OpenAICompatibleTargetDetector.detect(request, *, timeout_s) -> TargetDetection`、`parse_target_detection` | 61 | 现有，不改 |
| `adapters/target_detection/manual.py` | `ManualTargetDetector.detect`、`from_corners` | 49 | 现有，不改 |
| `adapters/target_detection/prompts/{system,visual_goal_target_detection}.py` | `TARGET_DETECTION_SYSTEM_PROMPT`、`VisualGoalTargetDetectionPrompt.build` | 43/32 | 现有，保持 Python（见第 8 节第 4 条） |
| `adapters/task_completion/client.py` | `OpenAICompatibleTaskCompletionChecker.check(request, *, timeout_s) -> bool` | 26 | 现有，不改 |
| `adapters/task_completion/prompt.py` | `CompletionMessages`、`CompletionPromptBuilder` | 15 | 现有，不改 |
| `adapters/task_completion/prompts/visual_goal_completion.py` | `VisualGoalCompletionPrompt.prepare`、`build_user_prompt` | 150 | 现有，保持 Python |
| `adapters/trajectory_planner/qwen_vl_planner.py` | `QwenVLPlanner.plan(observation, instruction, *, …, task_context, budget, timeout_s) -> RelativeTrajectory` | 110 | 现有，改为调 `prompts/render.py` 取正文 |
| `adapters/trajectory_planner/response_parser.py` | `parse_trajectory(text, expected_points=5)` | 39 | 现有，不改 |
| `adapters/trajectory_planner/prompts/render.py` | `render(template_name: str, **values) -> str`、`compose(base: str, part: str) -> str`、`template_names()` | 新建约 60 | 新建 |
| `adapters/trajectory_planner/prompts/templates/*.md` | 12 个模板：`base_ground_system`、`base_aerial_system`、`free_navigation_rules_ground`、`free_navigation_rules_aerial`、`visual_goal_rules_ground`、`visual_goal_rules_aerial`、`visual_goal_view_guidance`、`visual_goal_output_ground`、`visual_goal_output_aerial`、`base_user_ground`、`base_user_aerial`、`free_space_profile_head` | 合计约 900 | 从三个提示词模块抽出 |
| `adapters/trajectory_planner/prompts/base.py` | `base_system_prompt(*, aerial=False)`、`build_base_user_prompt(...)`、`build_aerial_base_user_prompt(...)`、`BaseTrajectoryPrompt`、`TrajectoryPromptBuilder` | 510 → 约 190 | 现有，正文外置 |
| `adapters/trajectory_planner/prompts/free_navigation.py` | `free_navigation_system_prompt(*, aerial=False)`、`FreeNavigationTrajectoryPrompt.build_user_prompt`、`format_free_space_profile` | 596 → 约 150 | 现有；删 `format_lane_reference` 与 `LaneReference` 参数 |
| `adapters/trajectory_planner/prompts/visual_goal_navigation.py` | `visual_goal_system_prompt(*, aerial=False)`、`VisualGoalTrajectoryPrompt.build_user_prompt`、`_target_side_rule`、`_new_trajectory_reference` | 305 → 约 150 | 现有，正文外置 |
| `adapters/trajectory_planner/prompts/__init__.py` | 41 个名字的再导出 | 45 | 现有，名字不变 |
| `adapters/http/server_api.py` | `create_api(controller, settings, frames=None, trajectory_hub=None) -> FastAPI` | 511 → 约 505 | 现有，能力位改读 `ServerController.capabilities()`，删 `simulator_label` 直取（裁决 R27） |
| `adapters/http/video.py`、`adapters/http_errors.py`、`multimodal_chat.py`、`chat_request_options.py`、`image_encoding.py`、`model_visual_inputs.py`、`diagnostics/visual_goal_trace.py` | 现状签名 | 不变 | 现有，不改 |

`adapters/http/` 当前从 `planner.simulator` import `registry`（`capabilities_for` 与 `labels`），违反 00 第六节规则 6。改动：能力位改读 `ServerController.capabilities()`，simulator 标签也由 controller 给，该 import 整行删除（裁决 R27）。路由、字段、状态码、`/api/video` 的门都不动。

### `planner/simulators/projectairsim/`

| 文件 | 类与函数签名 | 估计行数 | 从哪来 |
| --- | --- | --- | --- |
| `session.py` | `ProjectAirSimMissionSession(config, *, log, frame_sink, trajectory)`；`mission_inputs() -> MissionInputs`、`attach_arms(arms)`、`run(task, *, stop_event, log) -> MissionResult`、`idle_tick()`、`close(*, skip_landing=False)`、`verify_ports()` | 789 → 约 330 | 拆自现 `session.py` |
| `vehicles.py` | `DualVehicleWorld(rover, drone, *, active, hide_gain_m, pin_period_s, log)`：`active_vehicle`、`activate_vehicle(vehicle=None) -> str`、`return_home(vehicle=None) -> str`、`park(vehicle_id) -> bool`、`settle(vehicle_id)`、`hide/unhide/recover/pin_hidden`、`frame_sink_for(vehicle_id)`、`set_publish(enabled)` | 新建约 270 | 拆自现 `session.py` 的 `activate_vehicle`、`return_home`、`_hide`、`_unhide`、`_pin_hidden`、`_recover`、`_park`、`_settle`、`_sink_for`、`_hide_inactive_at_startup`、`HIDE_ALTITUDE_GAIN_M`、`HIDE_PIN_PERIOD_S` |
| `weather.py` | `ProjectAirSimWeather(connection)`：`fields() -> dict[str,str]`、`presets() -> tuple[str,...]`、`apply(values) -> dict[str,float]` | 新建约 70 | 拆自现 `session.py` 的 `weather_catalog`、`set_weather` |
| `views.py` | `StreamingViewCycle(connection, *, declared_start, modes=VIEW_MODES)`：`view_modes()`、`set_view_mode(mode) -> bool`、`current()` | 新建约 80 | 拆自现 `session.py` 的 `set_view_mode`、`_switch_view_once`、`_view_positions`、`_view_index`、`_apply_initial_view` |
| `rover.py` | `ProjectAirSimRover`：`stream_command(command, duration_s)`、`stream_state()`、`current_pose()`、`motion_capability()`、`cancel()`、`hover()`、`collision_state()`、`start()`、`settle()`、`measure_resting_pose()`、`teleport(pose)`、`return_home()`、`set_home_pose(pose)`、`idle_tick()`；`execute_segment` 只在过渡期保留三行委派（裁决 R1） | 409 → 约 300 | 现有；删 `_tracker`、`_apply_reference`、`stream_velocity`；`ROVER_PASSED_LATERAL_TOLERANCE_M` 与 `rover_tracking_limits()` 删除，走廊改由 `algorithms/motion/corridor.py` 给出（裁决 R6），常量 `ACKERMANN_PASSED_LATERAL_TOLERANCE_M = 1.0`，配置只能抬高不能压低（裁决 R17） |
| `drone.py` | `ProjectAirSimDrone`：`stream_command`、`hold_velocity`（转调 `stream_command`，裁决 R7）、`motion_capability()`、`prepare`、`takeoff`、`land`、`park`、`hold`、`hover`、`cancel`、`collision_state`、`stream_state`、`capture`、`capture_moving`、`return_home`、`teleport`、`idle_tick`、`close`；`execute_segment` 只在过渡期保留三行委派（裁决 R1） | 740 → 约 630 | 现有；删 `_tracker`、`_apply_reference`、`stream_velocity` |
| `motion.py` | 只留 `tracking_limits(motion) -> MotionLimits`（配置包络转 domain 类型） | 234 → 约 30 | 现有；`SteppedMotionTracker`、`limit_vector` 删除 |
| `drive.py` | `RoverDriveModel`、`SpeedPid`、`heading_vector`、`controls_from_reference`、`RoverControls` | 170 | 现有，不改（纯标定模型） |
| `observation.py` | `ProjectAirSimObservationSource.capture`、`capture_moving`、`close` | 198 | 现有，不改 |
| `frames.py` | `CameraTopicStream`、`FramePair` | 211 | 现有，不改 |
| `moving_source.py` | `BorrowedMovingSource` | 27 | 现有，不改 |
| `connection.py` | `ProjectAirSimConnection`、`ProjectAirSimConnectionConfig`、`advance_for_sim_time`、`add_advance_hook`、`resume_clock`、`switch_streaming_view`、`set_weather_parameter` | 359 | 现有，不改 |
| `coordinates.py` | `pose_from_kinematics`、`velocity_from_kinematics`、`yaw_rate_deg_from_kinematics`、`quaternion_to_rotation`、`camera_matrix` | 206 | 现有，不改 |
| `collision.py` | `CollisionMonitor`、`CollisionEvent` | 144 | 现有，不改 |
| `camera.py` | 场景 JSONC 相机校验与内参 | 124 | 现有，不改 |
| `async_bridge.py` | `AsyncBridge.run(awaitable, *, timeout_s)` | 68 | 现有，不改 |
| ~~`streaming_views.py`~~ | 已移出引擎包 | 543 → 0 | 移为仓库根 `scripts/streaming_views.py`，只给标定脚本与测试用（裁决 R22，R15 作废） |
| `runtime.py` | `PORT_SLOTS`、`VEHICLES`、`describe()`、`build_ports(...) -> SimulatorPorts`、`verify_ports(ports) -> None` | 86 → 约 115 | 现有；`build_ports` 与 `verify_ports` 新增，与 UE4 对称；`ENABLED_TASKS` 删除（裁决 R18） |
| `__init__.py` | 惰性导出 | 43 → 约 55 | 现有，增加 `DualVehicleWorld`、`ProjectAirSimWeather`、`StreamingViewCycle` |

### `planner/simulators/airsim/`

| 文件 | 类与函数签名 | 估计行数 | 从哪来 |
| --- | --- | --- | --- |
| `session.py` | `AirSimMissionSession(config, family, *, log)`；`mission_inputs`、`attach_arms`、`run`、`idle_tick`、`set_weather`、`close`、`verify_ports()` | 314 → 约 300 | 现有，加构造期端口自检 |
| `vehicle.py` | `AirSimVehicle`：`stream_command(command, duration_s)`、`hold_velocity`（转调 `stream_command`，裁决 R7）、`motion_capability()`、`current_pose`、`cancel`、`hover`、`hold_tick`、`begin_observation_hold`、`wait_for_observation`、`collision_state`、`stream_state`、`execute_basic_motion`、`tracking_limits`；`execute_segment` 只在过渡期保留三行委派（裁决 R1） | 222 → 约 190 | 现有；删 `stream_velocity` 与 `AirSimMotionTracker` 构造 |
| `motion_tracker.py` | 删除 | 158 → 0 | 律进 `runtime/motion/tracking.py` |
| `observation_gate.py` | `ObservationGate`：留 `request_capture`、`cancel_requests`、`service_requests`、`capture`、`wait_for_observation`、`keep_holding`；删 `step()`、`generation`、`_generation` 展开 | 151 → 约 105 | 现有；暂停/恢复交 `PoseCheckpoint` |
| `server_vehicle.py` | `ServerVehicle.prepare`、`hold`、`climb_after_takeoff` | 143 | 现有，不改 |
| `observation_source.py` | `AirSimObservationSource.capture`、`request_capture`、`capture_moving`、`close` | 258 | 现有，不改 |
| `camera_model.py` | `camera_matrix`、`quaternion_to_rotation`、`intrinsics_from_horizontal_fov` | 61 | 现有；见第 8 节第 3 条，默认保留三个函数 |
| `weather.py` | `AirSimWeather`、`PARAMETER_IDS` | 61 | 现有，不改 |
| `connection.py` | `AirSimConnection.call`、`call_streaming`、`call_async_and_wait`、`prepare_vehicle`、`takeoff`、`shutdown`、`close_transport` | 270 | 现有，不改 |
| `recording.py`、`rpc_loop.py` | `CameraApiRecorder`、`OwnerRpcLoop`、`configure_rpc_loop` | 190/54 | 现有，不改 |
| `runtime.py` | `PORT_SLOTS`、`VEHICLES`、`build_ports`、`describe()`、`verify_ports(ports)`、`WEATHER_FIELDS`、`WEATHER_PRESETS` | 110 → 约 115 | 现有；`verify_ports` 新增；`ENABLED_TASKS` 删除（裁决 R18） |
| `__init__.py` | 导出 | 7 | 现有，不改 |

### `planner/simulator/`（进程栈）

| 文件 | 改动 | 行数 |
| --- | --- | --- |
| `registry.py` | `build_session` 的 import 前缀 `application` → `assembly`；会话构造后调 `session.verify_ports()`（未实现该方法视为错误） | 88 |
| `ports.py` | 删 `TASK_REQUIREMENTS`（迁 `assembly/requirements.py`，读各任务包 `__init__.py` 的 `TASK_KIND` 与 `REQUIRED_PORTS`，裁决 R10）；`SimulatorPorts.capabilities(*, task_kinds)` 改由调用方传入可跑任务集；`enabled_tasks` 字段删除（裁决 R18，原 R11 作废）；保留 `ACTION_PORTS`、`implemented()`、`from_slots`、`_Slot`/`_Views`/`_Weather` | 196 → 约 140 |
| `worker.py` | 不改（对 session 是 `getattr` 鸭子调用：`idle_tick`、`activate_vehicle`、`set_view_mode`、`set_weather`） | 277 |
| `subprocess_worker.py` | 不改 | 336 |
| `mission_child.py` | import 前缀改名；`_build_session` 的 `MissionResources` 构造不变 | 392 |
| `xpra_start.py`、`processes.py`、`argv.py`、`marker.py`、`router.py`、`telemetry_relay.py` | 不改 | 591/284/78/35/113/76 |

UE4 子进程协议不改。理由见第三节末。

## 二、数据流与时序

### 段执行一拍（30 ms）

```
任务层 → ExecutionService.execute_with_receipt
  → ControlLoop.run_segment(segment, sink=streaming)                 # runtime，裁决 R1
      Vehicle 槽由 GuidedVehicle(streaming, loop) 兑现，装配层填槽；
      过渡期引擎保留三行 execute_segment 委派，转调同一 run_segment，最终删除
      每 SEGMENT_PERIOD_S = 0.030 s（裁决 R9）：
        sample_segment(segment, elapsed) → 参考位姿与速度
        tracked_heading_rate_deg_s(...) → yaw_rate
        速度限幅、加速度限幅、位置增益
        sink.stream_command(MotionCommand, 0.03)                     # 同一签名，裁决 R2
        引擎：一条原生命令 + 时钟推进 0.03 s
      settle 判定：位姿与偏航在容差内且实测速度、角速度低于停稳阈值
```

引擎侧一拍只做三件事：读一次 `current_pose`；把算好的命令发下去；把世界推进 `duration_s`。ProjectAirSim 的推进是 `ProjectAirSimConnection.advance_for_sim_time(duration_s)`，每次推进后跑 `add_advance_hook` 注册的隐藏载具钉位钩子。UE4 的时钟自己跑，`stream_command` 不推进，只发 `moveByVelocityAsync`。

### 流执行一拍（50 ms，`1/motion.control_hz`）

```
QueueFollower.next(state)          # runtime/motion/follower.py，含队列消耗与 waypoint 超时（裁决 R4）
  → ControlLoop.run_stream 每 1/MotionLimits.control_hz（默认 50 ms）：
      state = vehicle.stream_state()                             # 引擎读侧
      command = QueueFollower.next(state)
      vehicle.stream_command(command, duration_s)                # 同一签名（裁决 R2）
```

段拍长与流拍长都由 `ControlLoop` 决定，引擎不读 `control_hz`，只按下发时长执行。现状的段拍长是 `1/motion.control_hz`（20 Hz，50 ms），目标改为固定 30 ms；每拍位移与加速度限幅随之变小，配置不用改。

### 观测一帧

ProjectAirSim：`capture()` → `ProjectAirSimObservationSource.capture` → `CameraTopicStream.poll_next` 取一对 RGB 与深度（按仿真时间戳配对）→ `Observation`。停稳采集先把位置保持稳住再取帧。

UE4：`capture()` → `ObservationGate.capture()`（自称 owner、`begin_observation_hold`、`wait_for_observation`）→ `AirSimObservationSource.capture`，图像走每 worker 一条只读连接（`build_capture_source`），不进控制锁。

### 提示词渲染

`QwenVLPlanner.plan` → `prompt_builder.system_prompt()`（读模板，一次拼接后缓存）→ `build_user_prompt(...)`（Python 算数值，模板只放固定标签与说明行，`${name}` 占位）→ `MultimodalChatClient.request`。渲染一次约 6 次文件读取，`lru_cache` 后只在首轮。

## 三、跨层接口

### 我提供

| 契约名 | 实现位置 | 说明 |
| --- | --- | --- |
| `Vehicle` | 最终由 `runtime.motion.loop.GuidedVehicle(streaming, loop)` 兑现，装配层填槽；过渡期投影到引擎载具 | `current_pose`、`motion_capability`、`execute_segment`、`cancel`、`hover`、`collision_state`（裁决 R1） |
| `StreamingVehicle` | `ProjectAirSimRover`、`ProjectAirSimDrone`、`AirSimVehicle` | `stream_state`、`stream_command` |
| `stream_command(command: MotionCommand, duration_s: float) -> None` | 同上三处 | 引擎侧唯一数据面动作；sink 由 `ControlLoop.segment_sink()`/`stream_sink()` 提供，就是 `vehicle.stream_command` 的绑定，适配层不认识任何 sink 名字（裁决 R2、R26） |
| `ObservationSource` / `ControlledObservationSource` / `MovingObservationSource` | `ProjectAirSimObservationSource`、`AirSimObservationSource`、`BorrowedMovingSource` | |
| `WeatherControl` | 新增 `ProjectAirSimWeather`、现有 `AirSimWeather` | |
| `StreamingViews` | 新增 `StreamingViewCycle` | 只有 UE5 有 |
| `ReturnHome` | `ProjectAirSimRover.return_home`、`ProjectAirSimDrone.return_home` | 只有 UE5 声明该槽位 |
| `SimulatorPorts` | `simulators/*/runtime.py::build_ports` + `verify_ports` | 声明与实物在构造期比对 |
| 端口实现 | `adapters/{task_parser,target_detection,trajectory_planner,task_completion}` | `TaskParser`、`TargetDetector`、`TrajectoryPlanner`、`TaskCompletionChecker` |

### `MotionCapability` 由谁给出、取值多少

由引擎载具类给出，写在 `motion_capability()` 里，值不来自配置：

| 载具 | `model` | `max_lateral_speed_mps` | `min_turn_radius_m` | `allows_reverse` |
| --- | --- | --- | --- | --- |
| `ProjectAirSimDrone` | `MotionModel.HOLONOMIC` | `config.drone.speed_mps`（默认 2.0） | `None` | `True` |
| `AirSimVehicle` | `MotionModel.HOLONOMIC` | 构造参数 `speed_mps`（`config.drone.speed_mps`，默认 2.0） | `None` | `True` |
| `ProjectAirSimRover` | `MotionModel.ACKERMANN` | `0.0` | `180/(yaw_rate_per_steer_mps·π·max_steering)`，由 `drive.py` 的 `RoverDriveModel` 常量算出，默认 ≈3.20 m | `True` |

无人机取 `drone.speed_mps` 而不是 `motion.max_speed_mps`：前者是规划器实际使用的平移速度上限（段规划与流跟随都用它），横向分量不该超过这个数。阿克曼车 `max_lateral_speed_mps=0.0` 是硬约束，不是标定值。最小转弯半径由转向模型反算，标定一改就跟着改。

### `stream_command` 实现

三个实现同一个骨架，差异只在落地那一行：

```python
# ProjectAirSimRover
def stream_command(self, command: MotionCommand, duration_s: float) -> None:
    self._require_started()
    self._require_command(command, CarCommand)          # 不符抛 ValueError
    _require_finite(command, duration_s)                 # 失败抛 ExecutionError
    state = self.stream_state()
    forward = heading_vector(state.pose.yaw_deg)        # drive.py 的 NED 车头轴
    controls = controls_from_reference(
        self._drive_model, self._pid,
        (forward[0] * command.forward_speed_mps, forward[1] * command.forward_speed_mps, 0.0),
        command.yaw_rate_deg_s, state.velocity_world, state.pose.yaw_deg, duration_s,
    )
    self._send_controls(controls)
    self._connection.advance_for_sim_time(duration_s)
```

```python
# ProjectAirSimDrone / AirSimVehicle
def stream_command(self, command: MotionCommand, duration_s: float) -> None:
    self._require_started()
    self._require_command(command, HolonomicCommand)     # 不符抛 ValueError
    _require_finite(command, duration_s)
    ...  # drone: _bridge.run(_velocity_exchange(...))；airsim: connection.call_streaming("moveByVelocityAsync", ...)
```

校验规则，三条，全部在适配层：

1. 命令类型必须与载具的 `motion_capability()` 相符，校验用 `require_supported(command, self._capability)`（适配层自己的函数，不建模型→命令类型的映射表），不符抛 `ValueError`，不新增异常类（裁决 R27）。无人机收 `CarCommand`、车收 `HolonomicCommand` 都在这一句上炸。
2. 所有字段与 `duration_s` 有限且 `duration_s > 0`，否则 `ExecutionError`（沿用现有文案）。
3. `command.reverse is True` 且 `allows_reverse is False` 时 `ValueError`。

阿克曼车收到 `HolonomicCommand` 一律拒收，不检查横向分量大小：横向分量的存在本身就说明调用方按全向载具算的，这一拍就该炸，而不是让 `drive.py` 把分量投影掉再跑到超时。`hold_velocity(velocity_world, yaw_rate_deg_s, duration_s)` 保留，作为 `PositionHoldVehicle` 端口实现，函数体转调 `stream_command(HolonomicCommand(*velocity_world, yaw_rate_deg_s), duration_s)`。

### 我要求相邻层提供

| 提供方 | 内容 | 用途 |
| --- | --- | --- |
| `runtime`（包入口导出 `GuidedVehicle`、`ControlLoop`、`QueueFollower`、`CollisionRecovery`；文件可在子包内，跨层只认包入口，裁决 R19a） | `ControlLoop.run_segment(segment, sink, *, budget, pose_checkpoint)`、`ControlLoop.run_stream(goals, sink, *, deadline_monotonic)`；`SEGMENT_PERIOD_S = 0.030` 与 `1/MotionLimits.control_hz` 都在 `runtime/motion/loop.py`，不进配置（裁决 R9）；`run_stream` 的 deadline 到期抛 `BudgetExceeded`，任务层映射 `TIMEOUT`（裁决 R3） | 引擎不 import `runtime`（00 第三节规则 7），最终由 `GuidedVehicle` 组合，过渡期以对象注入 |
| `runtime`（`QueueFollower`，实现 `runtime/motion/follower.py`，裁决 R19a） | 实现 `StreamGoalSource`，含队列消耗与 waypoint 超时；`PlanningWindow` 只提供队列视图、`consume` 与六个公开方法（裁决 R4） | 任务层把 `QueueFollower` 传给 `run_stream` |
| `runtime`（`CollisionRecovery`） | `poll` 挂在 `ControlLoop._guard` 上，读引擎 `collision_state()`；`ExecutionService.checkpoint()` 只管停止与预算，`ports/collision_check.py` 删除（裁决 R5） | 引擎只提供 `collision_state`，不决定接触后怎么退 |
| `runtime/motion/tracking.py` | 现 `SteppedMotionTracker` 与 `AirSimMotionTracker` 的律：参考采样、速度与偏航反馈、`limit_vector`、加速限幅、settle 判定、"从实测位姿恢复"（现只 UE4 有） | 引擎删律，行为由这里统一 |
| `runtime`（`ExecutionService` 经包入口暴露） | 控制权唯一性、检查点、`request_stop`（裁决 R19a） | 引擎不判 owner |
| `runtime`（位置保持） | 位置保持走 `PositionHoldVehicle`（`stream_state`/`cancel`/`hold_velocity`） | 引擎只提供端口 |
| `assembly`（`requirements`，裁决 R19a） | `TASK_REQUIREMENTS`、`available_tasks(ports)`；能力位与 simulator 标签由 `ServerController.capabilities()` 提供，不再导出 `capabilities_for`/`simulator_labels`（裁决 R27） | `adapters/http/` 与 `SimulatorPorts.capabilities()` 的来源 |
| `assembly`（`mission_assembly`） | 按 `REQUIRED_PORTS` 装配，并构造 `GuidedVehicle` 填 `vehicle` 槽（裁决 R1、R19b） | 引擎不认识任务 |
| `domain/` + `ports/` | `MotionModel`、`MotionCapability`、`HolonomicCommand`、`CarCommand`、`MotionCommand`、`CollisionState`、`CollisionContact`；`ports/vehicle.py` 增加 `motion_capability()`；`ports/streaming_vehicle.py` 用 `stream_command` 替掉 `stream_velocity`；不新增 `COMMAND_FOR_MODEL`，契约层不加名字。`ports/` 最终 13 个文件：`backend_worker.py` 迁 `simulator/worker_protocol.py`、新增 `goal_source.py`、删 `collision_check.py`（裁决 R27） | 00 第四节已定，这里只实现 |

### 构造期端口自检

两个引擎在会话构造末尾调 `verify_ports`，比对三件事，不一致抛 `ProtocolError`，不进运行期：

1. `PORT_SLOTS` 与 `SimulatorPorts.implemented()` 相等（UE5 因此必须新增 `build_ports` 造一个活包，现在只有声明）。
2. 槽位 `vehicle`、`streaming` 是同一个对象（UE4 的 `build_ports` 已经把 `vehicle` 同时填进两个槽）。
3. 每台载具：`require_supported` 对它自己能力相符的命令放行、对另一种命令拒绝（构造期各跑一次 `HolonomicCommand` 与 `CarCommand` 的典型实例，能接受的那一支必须与类属性 `ACCEPTS` 一致）。

### UE4 子进程协议

不改。父进程发给子进程的命令集是 `task`、`stop`、`weather`、`shutdown`、`terminate`、`home`、`activate`、`view`,事件经 fd 3 单行 JSON 回传;`MotionCommand` 只在子进程内的控制回路里活着,不过管道。子进程的会话构造与端口自检与进程内 worker 完全一致,所以父进程看不到差异。唯一改动是子进程模块的 `application` → `assembly` import 前缀。

### 输入面与输出面（裁决 R19c）

只列跨层可见的名字。同层内部实现类不进表。跨层 import 一律指向包入口（`planner.runtime`、`planner.assembly`、`planner.simulator`、`planner.simulators.<name>`、`planner.adapters.<name>`），禁止 `planner.runtime.motion.loop` 这类深层路径（裁决 R19a）。

输入：

| 名字 | 形状 | 由谁提供 | 是否可选 |
| --- | --- | --- | --- |
| `Observation` | `domain.observation.Observation` | 契约层 | 否 |
| `MotionSegment` | `domain.trajectory.MotionSegment` | 契约层 | 否 |
| `MotionCommand` | `domain` 的 `HolonomicCommand` \| `CarCommand` | 契约层 | 否 |
| `MotionCapability` / `MotionModel` | `domain.motion` | 契约层 | 否 |
| `CollisionState` / `CollisionContact` | `domain.execution` | 契约层 | 否 |
| `ExecutionBudget` / `PoseCheckpoint` | `domain.execution` | 契约层 | 是 |
| `TaskSubmission` / `MissionResult` / `WorkerEvent` | `domain.server`、`domain.result` | 契约层 | 否 |
| `TimedVehicleState` | `domain.streaming` | 契约层 | 否 |
| `StreamGoalSource` | 契约层 Protocol | 契约层（runtime 侧实现） | 仅流执行路径 |
| `SimulatorPorts` | 槽位包，`simulator.ports` 声明 | 本层自己声明 | 否 |
| `AppConfig` / `MapEntry` / `SimulatorFamily` | `config`、`domain.engine_family` | 契约层与装配层 | `build_session` 必给，`map_entry` 对 UE4 不读 |

输出：

| 名字 | 形状 | 由谁消费 | 稳定性 |
| --- | --- | --- | --- |
| 载具实现 | `Vehicle` + `StreamingVehicle` 端口（9 个方法） | `runtime`（经 `GuidedVehicle`） | 稳定 |
| 观测源实现 | `ObservationSource` / `ControlledObservationSource` / `MovingObservationSource` | `runtime`、任务层装配 | 稳定 |
| `ProjectAirSimWeather` / `AirSimWeather` | `WeatherControl` | `assembly` 的 `EnvironmentChannel` | 稳定 |
| `StreamingViewCycle` | `StreamingViews` | `assembly` 的 `EnvironmentChannel` | 稳定 |
| `describe() -> SimulatorPorts` / `build_ports(...) -> SimulatorPorts` | 声明包与活包 | `simulator.registry`、`assembly`、测试 | 稳定 |
| `verify_ports(ports)` | 构造期自检，失败抛 `ProtocolError` | `registry`、测试 | 稳定 |
| 四个端口实现 | `TaskParser`、`TargetDetector`、`TrajectoryPlanner`、`TaskCompletionChecker` | `assembly`（经 `factories`） | 稳定 |

不暴露什么：

- 载具内部件：`RoverDriveModel`、`SpeedPid`、`CollisionMonitor`、`CameraTopicStream`、`ObservationGate`、`AsyncBridge`、`ProjectAirSimConnection`、`AirSimConnection` 不进任何跨层签名。
- 已删除的 `SteppedMotionTracker`、`AirSimMotionTracker`、`ROVER_PASSED_LATERAL_TOLERANCE_M`、`rover_tracking_limits()` 不再是名字。
- 标定常量（`SETTLE_DURATION_S`、`HIDE_ALTITUDE_GAIN_M`、`HIDE_PIN_PERIOD_S`、`TELEPORT_TOLERANCE_M`）只在引擎包内使用。
- 机位标定的测量函数（`calibrate_streaming_views`、`verify_streaming_views`、`signature_bgr`、`xpra_screenshot`）已整块移出到 `scripts/streaming_views.py`；引擎包不导出它们，运行时只读 `config` 里的标定结果（裁决 R22）。
- 提示词模板名与 `render()` 是 `adapters/trajectory_planner` 的内部，不进端口签名。
- 跨层不传具体类：引擎构造参数只收契约类型与本层声明的端口实现，不收另一层的具体对象（裁决 R19b）。

## 四、错误与失败路径

| 情形 | 谁抛 | 谁捕获 | 结果与恢复 |
| --- | --- | --- | --- |
| 命令类型与载具运动模型不符 | `stream_command` 的 `_require_command`（即 `require_supported`） | `ControlLoop` 上抛，`ExecutionService` 记一次失败 | `ValueError`（裁决 R27）；这是编程错误，不重试 |
| 非有限数值或 `duration_s<=0` | `stream_command` | 同上 | `ExecutionError`，任务转 `StageResult(False)` |
| 声明槽位与实物不符 | `verify_ports` | 无（构造期） | `ProtocolError`，会话不成立，`/api/status` 停在 `disconnected` |
| 连接已关闭 | `ProjectAirSimConnection.request` 的 `_require_client` | `worker` 或 `mission_child` | `RuntimeError`，子进程按任务失败上报，进程存活等下一条命令 |
| 传输层失败（UE4） | `AirSimConnection._transport_call` | `worker`/`mission_child` | `SimulatorConnectionError`，`connection_lost` 置位，需重建会话 |
| 原生速度命令被中断 | 引擎 `stream_command` | `ControlLoop` → `ExecutionService` | `ExecutionError`，任务失败；位置保持在 `finally` 里接管 |
| 姿态/偏航未收敛 | `runtime/motion/loop.py` 的 settle 判定 | 同上 | `ExecutionError`，报实测距离与偏航误差 |
| 碰撞接触 | 引擎 `collision_state()`（`CollisionMonitor.consume`） | `runtime/collision_recovery.py` | 后退、清队列、继续；超次数抛 `ExecutionError` |
| 停稳采集超时 | `ObservationGate.capture` / 保持等待 | 任务层 | `ExecutionError`，重取帧 |
| 图像配对失败或时间戳不连续 | `CameraTopicStream._validate_message`、`AirSimObservationSource._validate_pair` | 任务层 | `ObservationError`，重置确认计数 |
| 天气被引擎拒绝 | `ProjectAirSimWeather.apply` / `AirSimWeather.apply` | `assembly/controller/environment.py` 的 `EnvironmentChannel`（天气与机位合并成一个文件，裁决 R12） | `ProtocolError`，202 已发出，事件里带原因 |
| 热切换时停不住原载具 | `DualVehicleWorld.activate_vehicle` | `worker` 的 `activate` 分支 | `ExecutionError`，`active_vehicle` 回滚到切换前 |
| 热切换时隐藏失败 | 同上 | 同上 | `ExecutionError`，目标载具不启用，避免"两台都看得见" |
| 模板文件缺失或占位符缺失 | `prompts/render.py` | `QwenVLPlanner` 的调用方 | `ProtocolError`；缺占位符带模板名与变量名 |
| 模板 CSS 式 `{}` 误解析 | 不适用 | 不适用 | 渲染器只认 `${name}`，正文里的 `{` 原样输出 |

任务边界上的收敛规则不变：引擎抛出的异常不跨任务存活；`finally` 里做停稳或落地，失败只记日志不覆盖原异常。

## 五、并发与所有权

### 线程

| 线程 | 谁起 | 碰什么 |
| --- | --- | --- |
| worker 线程（UE5，进程内） | `simulator/worker.py` | 拥有会话，串行执行 `run`、`idle_tick`、`activate`、`weather`、`view` |
| 子进程主线程（UE4） | `simulator/mission_child.py` | 同上，会话在子进程里 |
| 控制回路 | 调用 `execute_segment` 的那个线程 | 就是 worker 线程；`ControlLoop` 不另起线程 |
| asyncio 桥线程 | `AsyncBridge._serve` | 跑 `move_by_velocity_async`，只被 `stream_command` 用 |
| 碰撞订阅线程 | `CollisionMonitor.start` | 只写 `deque`，`consume()` 由控制线程调 |
| 帧 topic 线程 | `CameraTopicStream.start` | 只写帧槽；`FramePublisher` 汇在 `session` 侧按活动载具过滤 |
| 捕获 worker | `ObservationGate.request_capture` 的调用方 | UE4 每 worker 一条只读连接，不进控制锁 |

ProjectAirSim 的所有服务请求经 `ProjectAirSimConnection` 的一把锁串行；同一进程只允许一个连接（`_claim_active`）。

### 控制权

引擎不判控制权，只保证"谁调 `stream_command` 就发一条命令"。owner 在 `runtime/execution.py`，通过检查点与停止事件表达。引擎侧有两处会自己动载具，都不在任务期间：

- `idle_tick()`：worker 空闲时调用，UE5 让活动载具保持或推进一拍时钟，UE4 只在位置保持活动时 `tick`。
- `capture()`：UE5 先等位置保持稳住；UE4 走 `ObservationGate`，自称 owner，非 owner 线程调 `capture` 直接抛。

### 切换载具的交接

切换载具由引擎侧原子完成（裁决 R27）：controller 不单独 park、不单独 hide，也不分步调用，唯一入口是 `DualVehicleWorld.activate_vehicle(new)`；它内部依次 park、hide、recover、home、切机位，中途失败就回滚 `active_vehicle`，不存在“controller 已经 park 过、引擎又 park 一次”的双重步骤。

顺序固定，任一步失败即中止并回滚 `active_vehicle`：

1. `park(other)`：无人机 `park()`（释放位置保持、`cancel_last_task`），车 `hover()`（抱刹）。返回 `False` 直接中止。
2. `hide(other)`：向上平移 1000 m 并记下停放位姿。
3. `recover(target)`：丢弃目标载具的停放记录，让它能自由移动。
4. 置 `active_vehicle = target`（必须在 `return_home` 之前，否则它会被当成非活动载具再藏一次）。
5. `return_home(target)`：传送到测得的停放位姿（车的 home 位姿由 `measure_resting_pose` 标定，不是场景作者给的出生点）。
6. `set_view_mode(default_view_for(target))`。

隐藏载具靠 `pin_hidden` 每 2 s 仿真时间重新贴一次位姿，因为世界仍在施加重力。只有活动载具的相机帧进帧汇（`frame_sink_for`），否则预览会在天空与地面之间闪。

UE4 只有一台载具，`AirSimMissionSession.run` 对非 `drone` 的请求抛 `ProtocolError`，`vehicles` 恒为 `("drone",)`，控制台因此只出一个按钮。

## 六、测试点

| 测试文件 | 断言对象 |
| --- | --- |
| `tests/test_engine_command_contract.py`（新） | 三台载具的 `motion_capability()` 取值；`require_supported` 的接受/拒绝两支与 `ACCEPTS` 一致；`min_turn_radius_m` 由 `RoverDriveModel` 常量算出（改标定则断言值跟着变） |
| 同上，负例 | 车收 `HolonomicCommand` → `ValueError`；无人机收 `CarCommand` → `ValueError`；`duration_s=0`、`nan` 字段 → `ExecutionError`；`allows_reverse=False` 的假载具收 `reverse=True` → `ValueError` |
| `tests/test_ports_selfcheck.py`（新） | 两个 `verify_ports` 在真端口上通过；少填 `weather` 槽位 → `ProtocolError`；`PORT_SLOTS` 多声明一个槽 → `ProtocolError`；`vehicle` 与 `streaming` 不是同一对象 → `ProtocolError` |
| `tests/test_airsim_session.py`（现有 13 例） | 增加 `session.ports.capabilities() == runtime.describe().capabilities()`；`AirSimVehicle` 有 `stream_command` 与 `motion_capability` |
| `tests/test_projectairsim_session.py`（现有 34 例） | 增加活包与 `describe()` 的能力相等；`mission_inputs()` 的 `vehicles` 键与 `VEHICLES` 一致 |
| `tests/test_vehicle_switch.py`（新） | `activate_vehicle` 的六步顺序；`park` 返回 `False` 时 `active_vehicle` 不变；`hide` 抛异常时抛 `ExecutionError` 且不置新活动载具；对未知载具名 → `ProtocolError` |
| `tests/test_hidden_pin.py`（新） | 空集合直接返回；2 s 内重复调用只贴一次；某台载具贴失败只记一次日志、其余继续 |
| `tests/test_engine_actuator.py`（新） | 用假 connection 断言 `stream_command` 恰好发一条命令、`advance_for_sim_time` 收到 `duration_s`；车的 `engine` 只由前向分量与 PID 决定（无横向投影路径） |
| `tests/test_prompt_rendering.py`（新） | 12 个模板渲染结果与提交的基线逐字一致；多余变量报错；缺变量报错带模板名；正文里的 `{`、`[[`、`]` 原样保留 |
| `tests/test_trajectory_prompts.py`（现有 21 例） | 形状与内容约束不减少用例；`format_lane_reference` 相关用例随车道先验一起删除 |
| `tests/test_architecture.py`（现有） | 增加三条：`planner/simulators/**` 不 import `planner.runtime`；`planner/adapters/http/**` 不 import `planner.simulator`；跨层 import 只指向包入口（裁决 R19a） |
| `tests/test_capability_derivation.py`（新） | 可跑任务 = 任务包 `REQUIRED_PORTS` ⊆ 已填槽位，没有第二个开关：`hasattr(SimulatorPorts, "enabled_tasks")` 为假；引擎 `runtime.py` 无 `ENABLED_TASKS`；UE4 因实现了 `streaming` 而开放 `free_navigation`（裁决 R18） |
| `tests/test_streaming_views.py`（现有 19 例） | 改 import 路径到 `scripts/streaming_views.py`；新增断言：`planner/simulators/` 下不出现 `xpra_screenshot`、`signature_bgr` 两个名字，且 `projectairsim/runtime.py`、`session.py` 不 import 标定模块（裁决 R22） |
| `tests/test_layer_inputs.py`（新） | 每层入口 `__init__.py` 导出名与应用层构造参数只包含契约类型或本层 Protocol（裁决 R19b） |
| `tests/test_airsim_port_names.py`（现有 3 例） | 扩展为 `Vehicle` 与 `StreamingVehicle` 的全部成员都在 `AirSimVehicle` 上 |
| `tests/test_projectairsim_drive.py`（现有 16 例） | 不改；`controls_from_reference` 签名不动 |
| `tests/test_projectairsim_motion.py`（9 例）、`tests/test_motion.py`（8 例） | 迁到 `runtime` 的律上；同样的场景断言同样的命令序列 |

## 七、迁移步骤

每步独立可回退，验收写在最后一行。

| 步 | 做什么 | 验收 |
| --- | --- | --- |
| 1 | 提示词外置：先用一次性脚本把三个模块的系统提示词与用户提示词按固定输入导出成基线文件，再写 `templates/*.md` 与 `render.py`，三个模块正文改为渲染 | `test_prompt_rendering.py` 与 `test_trajectory_prompts.py` 全绿，基线逐字一致 |
| 2 | 契约上线：`ports/vehicle.py` 加 `motion_capability()`，不加映射表；三台载具实现 `motion_capability()` 与 `require_supported`；`stream_command` 以新名增加，`stream_velocity` 暂时保留为转调 | 给车喂 `HolonomicCommand` 必须抛错（测试断言 `ValueError`，裁决 R27）；`pytest tests/ -q` 全绿（此步不改行为） |
| 3 | 律进 `runtime`：新建 `runtime/motion/tracking.py` 与 `runtime/motion/loop.py`，载具构造接收 `segment_runner`，`execute_segment` 改三行委派，删 `SteppedMotionTracker`、`AirSimMotionTracker`、`_apply_reference` | `test_motion.py`、`test_projectairsim_motion.py`、`test_airsim_vehicle.py` 通过；UE5 数值动作实机一次 |
| 4 | 命令类型化落地：删 `stream_velocity`；车拒收 `HolonomicCommand`；`hold_velocity` 转调 `stream_command` | 负例测试通过；UE5 巡航与目标接近各一次，UE4 数值动作一次 |
| 5 | `session.py` 拆分：`vehicles.py`、`weather.py`、`views.py`；UE5 新增 `build_ports` 与 `verify_ports`，UE4 加自检 | `test_projectairsim_session.py`、`test_airsim_session.py`、`test_vehicle_switch.py`、`test_hidden_pin.py` 全绿，无实机 |
| 6 | 进程栈与入口：`TASK_REQUIREMENTS` 迁 `assembly/requirements.py`；`SimulatorPorts.capabilities` 收参数；`adapters/http` 删 `simulator.registry` import，能力位与标签改读 `ServerController.capabilities()`（裁决 R27）；`mission_child`/`registry` 改 import 前缀 | `test_server_api.py`(56)、`test_capability_gating.py`(9)、`test_runtime_catalog.py`(18)、`test_family_router.py`(3) 全绿；`test_layer_entries.py` 断言 `adapters/http` 不 import `planner.simulator` |
| 7 | UE4 实机回归：数值动作、目标接近；`free_navigation` 登开跑一次（它的 `streaming` 槽位已经存在，按裁决 R18 已开放） | 三类任务在 UE4 上各跑一次，`/api/status` 的 `task_kinds` 由槽位交集自动得出，不靠白名单 |

第 1 步与引擎无关，可以最先合。第 3 步是唯一会影响实机时序的一步，出问题退回第 2 步。

## 八、削面复核（裁决 R21）

### A 对外名字逐条

计数口径：`grep -rF --include=*.py NAME planner tests scripts | wc -l`（匹配行数）。层外运行期调用点只列 `planner/` 下的非测试、非本层引用。

| 名字 | 现引用数 | 层外运行期调用点 | 处置 | 依据 |
| --- | --- | --- | --- | --- |
| `OpenAITaskParser` | 5 | `application/factories.py:21,189` | 保留 | 被装配调用 |
| `builtin_task_codecs` | 6 | `application/factories.py:22` | 保留 | 被装配调用 |
| `TaskCodec` | 9 | 无（只有 `task_parser/prompt.py` 同层用） | 内部化 | 同层引用不算 |
| `ManualTargetDetector` | 6 | `application/factories.py:78` | 保留 | 被装配调用 |
| `OpenAICompatibleTargetDetector` | 5 | `application/factories.py:95` | 保留 | 被装配调用 |
| `parse_target_detection` | 4 | 无（只有 `target_detection/client.py:62` 同层） | 内部化 | 同上 |
| `TARGET_DETECTION_SYSTEM_PROMPT` | 4 | 无（`client.py:60` 同层） | 内部化 | 同上 |
| `VisualGoalTargetDetectionPrompt` | 3 | 无（`client.py:49` 默认参数） | 内部化 | 同上 |
| `OpenAICompatibleTaskCompletionChecker` | 4 | `application/factories.py:19,157` | 保留 | 被装配调用 |
| `VisualGoalCompletionPrompt` | 4 | `application/factories.py:20,162` | 保留，但要改由 `adapters.task_completion` 入口导出 | 现在走深层路径（R19a） |
| `QwenVLPlanner` | 6 | `application/factories.py:23,118` | 保留 | 被装配调用 |
| `BaseTrajectoryPrompt`、`FreeNavigationTrajectoryPrompt`、`VisualGoalTrajectoryPrompt` | 7 / 18 / 18 | `application/factories.py:24-27`，`qwen_vl_planner.py:34` 默认值 | 保留 | 被装配注入 |
| `TrajectoryPromptBuilder` | 5 | 无（`qwen_vl_planner.py:17` 类型注解） | 内部化 | 只是层内 Protocol |
| `parse_trajectory` | 16 | 无（只有 `qwen_vl_planner.py:106` 同层） | 内部化 | 同层引用不算 |
| 12 个 `*_SYSTEM_PROMPT` / `*_GUIDANCE` 常量 | 3～31 | 无（只在 `prompts/` 内部拼装） | 全部内部化，`prompts/__init__.py` 从 18 名收到 3 名 | 层外零调用 |
| `base_system_prompt`、`build_base_user_prompt`、`build_aerial_base_user_prompt`、`free_navigation_system_prompt`、`visual_goal_system_prompt`、`format_free_space_profile` | 3～8 | 无 | 前 6 个内部化 | 层外零调用 |
| `format_lane_reference` | 2 | 无 | 删 | 车道先验已定删 |
| `VisualGoalTrace` | 4 | `application/factories.py:13,222` | 保留 | 被装配调用 |
| `resize_longest_side`、`generation_options`、`aligned_depth_image` | 3 / 3 / 21 | 无（只在 `adapters/` 内部互相调用） | 保持模块级，不进包入口 | 层内工具 |
| `ProjectAirSimMissionSession` / `AirSimMissionSession` | 3 / 3 | `simulator/registry.py:41` 经 `runtime.build_session` | 保留 | 会话工厂被进程栈调用 |
| `PORT_SLOTS`（两个引擎） | 4 / 3 | `simulator/registry.py:27` → `describe` | 保留，上提到包入口 | 能力推导的输入 |
| `describe()`（`projectairsim/runtime.py:78`、`airsim/runtime.py:84`） | 2 定义 | `registry.py:27,38` | 保留 | 同上 |
| `build_ports()`（`airsim/runtime.py:38`） | 1 定义 | `airsim/session.py:110` 附近 | 保留，UE5 新增同名 | 构造期自检需要活包 |
| `build_session()`（两个引擎） | 2 定义 | `mission_child.py:254`、`server_bootstrap.py:98`，经 `registry.py:41` | 保留 | 唯一造会话入口 |
| `VEHICLES`（两个引擎） | 3 | `registry.py` → `describe` | 保留，上提到包入口 | 载体清单来源 |
| `SimulatorPorts` | 18 | `registry.py:27,41`、`server_bootstrap.py`、`simulator/worker.py` | 保留 | 构造期不变量校验与接口契约 |
| `capabilities_for` | 6 | `config.py:1285`（HTTP 侧不再读，改读 `ServerController.capabilities()`，裁决 R27） | 保留 | 配置校验在读；HTTP 能力门经 controller |
| `ports_for` | 4 | 无（只有 `registry.py:38` 同文件用） | 内部化 | 同层引用不算 |
| `ACTION_PORTS` | 2 | 无（`ports.py:52` 定义 + `capabilities()` 同文件） | 内部化 | 同上 |
| `TASK_REQUIREMENTS` | 2 | 无（`ports.py:44` 定义 + `available_tasks()` 同文件） | 迁出 `assembly/requirements.py` | 裁决 R10 |
| `ENABLED_TASKS`（两个引擎） | 各 3～4 | `describe()`、`build_ports()`、测试 | 删 | 裁决 R18 |
| `CameraApiRecorder` | 3 | 无（`airsim/__init__.py:3,6` + 定义） | 删，连同 `recording.py` 190 行 | 零运行期调用 |
| `CameraTopicStream`、`ProjectAirSimConnection`、`ProjectAirSimConnectionConfig`、`ProjectAirSimObservationSource`、`ProjectAirSimRover`（包入口惰性导出） | 各 1～3 | 无（外部一律 import 子模块或 `runtime`） | 删入口导出，子模块内的类保留 | 入口导出无人消费 |
| `verify_ports()` | 新增 | 计划：`registry.py:41` 的 `build_session` 后调一次 | 保留 | 构造期不变量校验 |

### B 历史残留删除清单

| 名字 / 位置 | 现引用数 | 删或合并的理由与替换物 | 风险 |
| --- | --- | --- | --- |
| `simulators/airsim/recording.py`（`CameraApiRecorder`，190 行） | 3 | 无人使用；`airsim/__init__.py:3` 还在包导入时把它拉进来 | 无；附带去掉一条导入期依赖 |
| `simulators/projectairsim/motion.py`（`SteppedMotionTracker` 170 行 + `limit_vector` + `tracking_limits`） | 3 | 律进 `runtime/motion/tracking.py`；`tracking_limits` 换成 `config.motion.to_tracking_limits()`，session 直接调 | `test_projectairsim_motion.py` 改指向 runtime；UE5 段执行要实机回归 |
| `simulators/airsim/motion_tracker.py`（158 行） | 2 | 同上（裁决 R1） | `test_airsim_vehicle.py` 断言改指向 runtime |
| `observation_gate.py::step()`、`generation`（约 25 行） | 3 | 唯一调用方是 `motion_tracker.py`；暂停改用 `PoseCheckpoint`（R1、R5） | UE4 观测打断段的恢复行为要靠 runtime 的 resume 兼容 |
| `rover.py::ROVER_PASSED_LATERAL_TOLERANCE_M`、`rover_tracking_limits()` | 3 | 迁 `algorithms/motion/corridor.py`，`ACKERMANN_PASSED_LATERAL_TOLERANCE_M = 1.0`（R6、R17） | 车越点行为变了，重跑一次车的目标接近 |
| `prompts/free_navigation.py::format_lane_reference`（约 130 行） | 2 | 车道先验已决定删除，无替换物 | 无 |
| `moving_source.py::BorrowedMovingSource`（27 行，整文件） | 3 | 并入 `projectairsim/observation.py`，仍由 `session.py:273` 当 `moving_source_factory` 交出 | 无 |
| `session.py::_VehicleArm` 别名 | 1 | 历史兼容名，直接用 `VehicleArm` | 无 |
| `simulator/ports.py::TASK_REQUIREMENTS`、`available_tasks()` | 2 | 迁 `assembly/requirements.py`（R10） | 能力矩阵测试改断言入口 |
| `simulators/*/runtime.py::ENABLED_TASKS` | 各 3～4 | 删（R18）；UE4 巡航因 `streaming` 槽位存在而自动开放 | 巡航首次在 UE4 上跑，失败会直接暴露 |
| `adapters/trajectory_planner/prompts/__init__.py`（18 名） | 18 | 收到 3 名（`BaseTrajectoryPrompt`、`FreeNavigationTrajectoryPrompt`、`VisualGoalTrajectoryPrompt`） | 基线逐字对比测试守住 |
| `airsim/camera_model.py::intrinsics_from_horizontal_fov`（与 domain 重复） | 3 | 保留（R14），只加主点差 ≤ 0.5 px 断言 | 无 |
| `camera_matrix`、`quaternion_to_rotation` 两份实现（`airsim/camera_model.py`、`projectairsim/coordinates.py`） | 各 3～5 | 只能去 `domain/` 或 `algorithms/`，两者都不在引擎允许的 import 表里（00 规则 7），保留两份 + 等价性断言 | 两份漂移无自动发现 |
| `simulators/projectairsim/streaming_views.py`（543 行） | 15 | 移为仓库根 `scripts/streaming_views.py`，只给 `scripts/calibrate_streaming_views.py` 与 `tests/test_streaming_views.py` 用；引擎包删除该文件与它的 import，运行时只读 `config` 里的标定结果（裁决 R22，R15 作废） | 测试与脚本改 import 路径；另加一条名字级断言防止回流 |

### 结论数字

本层对外名字从 **50 降到 23**（适配层 31→11，引擎包入口 9→6，进程栈 10→6）；**内部化 22 个**（适配层 20，进程栈 `ports_for`、`ACTION_PORTS`），**删 7 个**（`CameraApiRecorder`、projectairsim 包入口 5 个惰性导出、`ENABLED_TASKS` 计 1），**迁出 1 个**（`TASK_REQUIREMENTS`）；`50 − 7 − 22 − 1 + 3 = 23`，+3 是 `PORT_SLOTS`、`VEHICLES`、`verify_ports` 上提到包入口。

删除代码约 **2233 行**：`recording.py` 190 + `projectairsim/motion.py` 234 + `motion_tracker.py` 158 + 观测门 25 + 走廊 12 + 车道提示词 130 + 提示词正文外置 920（1411→约 490）+ 开关与别名 21 + `streaming_views.py` 543（移出引擎包为 `scripts/streaming_views.py`，不算净删，算离开本层）。模板新增约 900 行 md，不计入代码行数。

只有测试在用、但要求保留的名字：无。每个保留项都能指出层外运行期调用点；`verify_ports` 是新增名字，调用点靠本方案要求 `registry.build_session` 加一行，当前尚未存在。

## 九、未决问题

原方案与裁决冲突的条目已按裁决改写（R1、R6、R7、R9、R13、R14、R15、R17、R18、R19），本节目前没有仍未决的条目；后续新出现的按同样格式补写。

裁决 R1（不再是开放项）：`execute_segment` 的采样循环在 `runtime/motion/loop.py`，引擎没有 tracker。引擎只实现 `stream_command(command, duration_s)` 加 `current_pose`/`motion_capability`/`collision_state`/`cancel`/`hover`；`Vehicle` 槽由 `runtime` 的 `GuidedVehicle(streaming, loop)` 组合对象兑现，装配层填槽。过渡期允许引擎保留三行委派，最终删除。

裁决 R9（不再是开放项）：`SEGMENT_PERIOD_S = 0.030` 与 `1/MotionLimits.control_hz` 都写在 `runtime/motion/loop.py`，不进配置。正文第二、七节里的“写死/进配置”对比作废。

裁决 R14（不再是开放项）：两份 `CameraIntrinsics` 实现都保留，只加一条主点差不超过 0.5 px 的断言。`domain/observation.py::CameraIntrinsics.from_horizontal_fov` 取 `w/2-0.5`，`simulators/airsim/camera_model.py::intrinsics_from_horizontal_fov` 取 `w/2`，不做统一。

裁决 R13（不再是开放项）：只外置 `adapters/trajectory_planner/prompts/` 的三个文件。`task_parser/prompt.py`、`task_completion/prompts/visual_goal_completion.py`、`target_detection/prompts/*` 继续留在 Python，不进模板。

裁决 R15 已被 R22 覆盖：`streaming_views.py` 移出引擎包到 `scripts/streaming_views.py`，引擎包不再有该文件；`views.py` 只做机位循环本身，不做测量。

裁决 R6 + R17（不再是开放项）：越点走廊常量落 `algorithms/motion/corridor.py`，`ACKERMANN_PASSED_LATERAL_TOLERANCE_M = 1.0`（原 0.6 m 是手工挑的，实测失败率偏高），`HOLONOMIC` 取 `MotionLimits.position_tolerance_m`；配置 `motion.passed_lateral_tolerance_m` 可覆盖，但只能抬高不能压低；不从 `min_turn_radius_m` 推导。引擎删除 `ROVER_PASSED_LATERAL_TOLERANCE_M` 与 `rover_tracking_limits()`。

裁决 R7（不再是开放项）：`PositionHoldVehicle.hold_velocity` 保留，实现内部转调 `stream_command`，不给位置保持第二个下发入口。

裁决 R18（该条关闭）：能力只由接口决定。`SimulatorPorts.enabled_tasks` 字段与两个引擎的 `ENABLED_TASKS` 删除，可跑任务 = 任务包 `REQUIRED_PORTS` ⊆ 已填槽位，没有第二个开关。UE4 实现了 `streaming` 端口，因此巡航自动对它开放；本节原先的“白名单等实测”选项作废。
