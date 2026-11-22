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
| `adapters/http/server_api.py` | `create_api(controller, settings, frames=None, trajectory_hub=None) -> FastAPI` | 511 → 约 505 | 现有，改 `capabilities()` 与 `simulator_labels` 的取法 |
| `adapters/http/video.py`、`adapters/http_errors.py`、`multimodal_chat.py`、`chat_request_options.py`、`image_encoding.py`、`model_visual_inputs.py`、`diagnostics/visual_goal_trace.py` | 现状签名 | 不变 | 现有，不改 |

`adapters/http/` 当前从 `planner.simulator` import `registry`（`capabilities_for` 与 `labels`），违反 00 第六节规则 6。改动只有一处：改成 `from ...assembly.requirements import capabilities_for, simulator_labels`。路由、字段、状态码、`/api/video` 的门都不动。

### `planner/simulators/projectairsim/`

| 文件 | 类与函数签名 | 估计行数 | 从哪来 |
| --- | --- | --- | --- |
| `session.py` | `ProjectAirSimMissionSession(config, *, log, frame_sink, trajectory)`；`mission_inputs() -> MissionInputs`、`attach_arms(arms)`、`run(task, *, stop_event, log) -> MissionResult`、`idle_tick()`、`close(*, skip_landing=False)`、`verify_ports()` | 789 → 约 330 | 拆自现 `session.py` |
| `vehicles.py` | `DualVehicleWorld(rover, drone, *, active, hide_gain_m, pin_period_s, log)`：`active_vehicle`、`activate_vehicle(vehicle=None) -> str`、`return_home(vehicle=None) -> str`、`park(vehicle_id) -> bool`、`settle(vehicle_id)`、`hide/unhide/recover/pin_hidden`、`frame_sink_for(vehicle_id)`、`set_publish(enabled)` | 新建约 270 | 拆自现 `session.py` 的 `activate_vehicle`、`return_home`、`_hide`、`_unhide`、`_pin_hidden`、`_recover`、`_park`、`_settle`、`_sink_for`、`_hide_inactive_at_startup`、`HIDE_ALTITUDE_GAIN_M`、`HIDE_PIN_PERIOD_S` |
| `weather.py` | `ProjectAirSimWeather(connection)`：`fields() -> dict[str,str]`、`presets() -> tuple[str,...]`、`apply(values) -> dict[str,float]` | 新建约 70 | 拆自现 `session.py` 的 `weather_catalog`、`set_weather` |
| `views.py` | `StreamingViewCycle(connection, *, declared_start, modes=VIEW_MODES)`：`view_modes()`、`set_view_mode(mode) -> bool`、`current()` | 新建约 80 | 拆自现 `session.py` 的 `set_view_mode`、`_switch_view_once`、`_view_positions`、`_view_index`、`_apply_initial_view` |
| `rover.py` | `ProjectAirSimRover`：`stream_command(command, duration_s)`、`stream_state()`、`current_pose()`、`motion_capability()`、`cancel()`、`hover()`、`collision_state()`、`start()`、`settle()`、`measure_resting_pose()`、`teleport(pose)`、`return_home()`、`set_home_pose(pose)`、`idle_tick()`；`execute_segment` 只在过渡期保留三行委派（裁决 R1） | 409 → 约 300 | 现有；删 `_tracker`、`_apply_reference`、`stream_velocity`；`ROVER_PASSED_LATERAL_TOLERANCE_M` 与 `rover_tracking_limits()` 删除，走廊改由 `algorithms/motion/corridor.py` 按 `MotionModel` 给出（裁决 R6） |
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
| `streaming_views.py` | `calibrate_streaming_views`、`verify_streaming_views`、`signature_bgr`、`signature_score`、`xpra_screenshot` | 543 | 现有；只被 `scripts/calibrate_streaming_views.py` 与 `tests/test_streaming_views.py` 引用，声明为工具模块，运行时路径不许 import |
| `runtime.py` | `PORT_SLOTS`、`ENABLED_TASKS`、`VEHICLES`、`describe()`、`build_ports(...) -> SimulatorPorts`、`verify_ports(ports) -> None` | 86 → 约 120 | 现有；`build_ports` 与 `verify_ports` 新增，与 UE4 对称 |
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
| `runtime.py` | `PORT_SLOTS`、`ENABLED_TASKS`、`VEHICLES`、`build_ports`、`describe()`、`verify_ports(ports)`、`WEATHER_FIELDS`、`WEATHER_PRESETS` | 110 → 约 125 | 现有；`verify_ports` 新增 |
| `__init__.py` | 导出 | 7 | 现有，不改 |

### `planner/simulator/`（进程栈）

| 文件 | 改动 | 行数 |
| --- | --- | --- |
| `registry.py` | `build_session` 的 import 前缀 `application` → `assembly`；会话构造后调 `session.verify_ports()`（未实现该方法视为错误） | 88 |
| `ports.py` | 删 `TASK_REQUIREMENTS`（迁 `assembly/requirements.py`，读各任务包 `__init__.py` 的 `TASK_KIND` 与 `REQUIRED_PORTS`，裁决 R10）；`SimulatorPorts.capabilities(*, task_kinds)` 改由调用方传入可跑任务集；`enabled_tasks` 留在 `SimulatorPorts`（裁决 R11）；保留 `ACTION_PORTS`、`implemented()`、`from_slots`、`_Slot`/`_Views`/`_Weather` | 196 → 约 150 |
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
| `stream_command(command: MotionCommand, duration_s: float) -> None` | 同上三处 | 引擎侧唯一数据面动作；runtime 的 `SegmentSink`、`StreamSink` 都是这个签名的别名，不出现第二个名字（裁决 R2） |
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
    self._require_command(command, CarCommand)          # 失败抛 ProtocolError
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
    self._require_command(command, HolonomicCommand)     # 失败抛 ProtocolError
    _require_finite(command, duration_s)
    ...  # drone: _bridge.run(_velocity_exchange(...))；airsim: connection.call_streaming("moveByVelocityAsync", ...)
```

校验规则，三条，全部在适配层：

1. 命令类型必须等于该载具 `motion_capability().model` 对应的类型（表 `COMMAND_FOR_MODEL = {HOLONOMIC: HolonomicCommand, ACKERMANN: CarCommand}`，放 `ports/vehicle.py`），否则 `ProtocolError`。
2. 所有字段与 `duration_s` 有限且 `duration_s > 0`，否则 `ExecutionError`（沿用现有文案）。
3. `command.reverse is True` 且 `allows_reverse is False` 时 `ProtocolError`。

阿克曼车收到 `HolonomicCommand` 一律拒收，不检查横向分量大小：横向分量的存在本身就说明调用方按全向载具算的，这一拍就该炸，而不是让 `drive.py` 把分量投影掉再跑到超时。`hold_velocity(velocity_world, yaw_rate_deg_s, duration_s)` 保留，作为 `PositionHoldVehicle` 端口实现，函数体转调 `stream_command(HolonomicCommand(*velocity_world, yaw_rate_deg_s), duration_s)`。

### 我要求相邻层提供

| 提供方 | 内容 | 用途 |
| --- | --- | --- |
| `runtime/motion/loop.py` | `ControlLoop.run_segment(segment, sink, *, budget, pose_checkpoint)`、`ControlLoop.run_stream(goals, sink, *, deadline_monotonic)`；`SEGMENT_PERIOD_S = 0.030` 与 `1/MotionLimits.control_hz` 都在这个文件里，不进配置（裁决 R9）；`run_stream` 的 deadline 到期抛 `BudgetExceeded`，任务层映射 `TIMEOUT`（裁决 R3） | 引擎不 import `runtime`（00 第三节规则 7），最终由 `GuidedVehicle` 组合，过渡期以对象注入 |
| `runtime/motion/follower.py` | `QueueFollower` 实现 `StreamGoalSource`，含队列消耗与 waypoint 超时；`PlanningWindow` 只提供队列视图、`consume` 与六个公开方法（裁决 R4） | 任务层把 `QueueFollower` 传给 `run_stream` |
| `runtime/collision_recovery.py` | `CollisionRecovery.poll` 挂在 `ControlLoop._guard` 上，读引擎 `collision_state()`；`ExecutionService.checkpoint()` 只管停止与预算，`ports/collision_check.py` 删除（裁决 R5） | 引擎只提供 `collision_state`，不决定接触后怎么退 |
| `runtime/motion/tracking.py` | 现 `SteppedMotionTracker` 与 `AirSimMotionTracker` 的律：参考采样、速度与偏航反馈、`limit_vector`、加速限幅、settle 判定、"从实测位姿恢复"（现只 UE4 有） | 引擎删律，行为由这里统一 |
| `runtime/execution.py` | 控制权唯一性、检查点、`request_stop` | 引擎不判 owner |
| `runtime/position_hold.py` | 位置保持走 `PositionHoldVehicle`（`stream_state`/`cancel`/`hold_velocity`） | 引擎只提供端口 |
| `assembly/requirements.py` | `TASK_REQUIREMENTS`、`available_tasks(ports)`、`capabilities_for(simulator) -> Capabilities`、`simulator_labels() -> dict[str, str]` | `adapters/http/` 与 `SimulatorPorts.capabilities()` 的唯一来源 |
| `assembly/mission_assembly.py` | 按 `REQUIRED_PORTS` 装配，并把 `ControlLoop` 注入载具构造 | 引擎不认识任务 |
| `domain/` + `ports/` | `MotionModel`、`MotionCapability`、`HolonomicCommand`、`CarCommand`、`MotionCommand`、`CollisionState`、`CollisionContact`；`ports/vehicle.py` 增加 `motion_capability()` 与 `COMMAND_FOR_MODEL`；`ports/streaming_vehicle.py` 用 `stream_command` 替掉 `stream_velocity` | 00 第四节已定，这里只实现 |

### 构造期端口自检

两个引擎在会话构造末尾调 `verify_ports`，比对三件事，不一致抛 `ProtocolError`，不进运行期：

1. `PORT_SLOTS` 与 `SimulatorPorts.implemented()` 相等（UE5 因此必须新增 `build_ports` 造一个活包，现在只有声明）。
2. 槽位 `vehicle`、`streaming` 是同一个对象（UE4 的 `build_ports` 已经把 `vehicle` 同时填进两个槽）。
3. 每台载具：`COMMAND_FOR_MODEL[vehicle.motion_capability().model] is type(vehicle).ACCEPTS`，`ACCEPTS` 是类属性常量。

### UE4 子进程协议

不改。父进程发给子进程的命令集是 `task`、`stop`、`weather`、`shutdown`、`terminate`、`home`、`activate`、`view`，事件经 fd 3 单行 JSON 回传；`MotionCommand` 只在子进程内的控制回路里活着，不过管道。子进程的会话构造与端口自检与进程内 worker 完全一致，所以父进程看不到差异。唯一改动是子进程模块的 `application` → `assembly` import 前缀。

## 四、错误与失败路径

| 情形 | 谁抛 | 谁捕获 | 结果与恢复 |
| --- | --- | --- | --- |
| 命令类型与载具运动模型不符 | `stream_command` 的 `_require_command` | `ControlLoop` 上抛，`ExecutionService` 记一次失败 | `ProtocolError`；这是编程错误，不重试 |
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

`DualVehicleWorld.activate_vehicle(vehicle)` 的顺序固定，任一步失败即中止并回滚 `active_vehicle`：

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
| `tests/test_engine_command_contract.py`（新） | 三台载具的 `motion_capability()` 取值；`ACCEPTS` 与 `COMMAND_FOR_MODEL` 一致；`min_turn_radius_m` 由 `RoverDriveModel` 常量算出（改标定则断言值跟着变） |
| 同上，负例 | 车收 `HolonomicCommand` → `ProtocolError`；无人机收 `CarCommand` → `ProtocolError`；`duration_s=0`、`nan` 字段 → `ExecutionError`；`allows_reverse=False` 的假载具收 `reverse=True` → `ProtocolError` |
| `tests/test_ports_selfcheck.py`（新） | 两个 `verify_ports` 在真端口上通过；少填 `weather` 槽位 → `ProtocolError`；`PORT_SLOTS` 多声明一个槽 → `ProtocolError`；`vehicle` 与 `streaming` 不是同一对象 → `ProtocolError` |
| `tests/test_airsim_session.py`（现有 13 例） | 增加 `session.ports.capabilities() == runtime.describe().capabilities()`；`AirSimVehicle` 有 `stream_command` 与 `motion_capability` |
| `tests/test_projectairsim_session.py`（现有 34 例） | 增加活包与 `describe()` 的能力相等；`mission_inputs()` 的 `vehicles` 键与 `VEHICLES` 一致 |
| `tests/test_vehicle_switch.py`（新） | `activate_vehicle` 的六步顺序；`park` 返回 `False` 时 `active_vehicle` 不变；`hide` 抛异常时抛 `ExecutionError` 且不置新活动载具；对未知载具名 → `ProtocolError` |
| `tests/test_hidden_pin.py`（新） | 空集合直接返回；2 s 内重复调用只贴一次；某台载具贴失败只记一次日志、其余继续 |
| `tests/test_engine_actuator.py`（新） | 用假 connection 断言 `stream_command` 恰好发一条命令、`advance_for_sim_time` 收到 `duration_s`；车的 `engine` 只由前向分量与 PID 决定（无横向投影路径） |
| `tests/test_prompt_rendering.py`（新） | 12 个模板渲染结果与提交的基线逐字一致；多余变量报错；缺变量报错带模板名；正文里的 `{`、`[[`、`]` 原样保留 |
| `tests/test_trajectory_prompts.py`（现有 21 例） | 形状与内容约束不减少用例；`format_lane_reference` 相关用例随车道先验一起删除 |
| `tests/test_architecture.py`（现有） | 增加两条：`planner/simulators/**` 不 import `planner.runtime`；`planner/adapters/http/**` 不 import `planner.simulator` |
| `tests/test_airsim_port_names.py`（现有 3 例） | 扩展为 `Vehicle` 与 `StreamingVehicle` 的全部成员都在 `AirSimVehicle` 上 |
| `tests/test_projectairsim_drive.py`（现有 16 例） | 不改；`controls_from_reference` 签名不动 |
| `tests/test_projectairsim_motion.py`（9 例）、`tests/test_motion.py`（8 例） | 迁到 `runtime` 的律上；同样的场景断言同样的命令序列 |

## 七、迁移步骤

每步独立可回退，验收写在最后一行。

| 步 | 做什么 | 验收 |
| --- | --- | --- |
| 1 | 提示词外置：先用一次性脚本把三个模块的系统提示词与用户提示词按固定输入导出成基线文件，再写 `templates/*.md` 与 `render.py`，三个模块正文改为渲染 | `test_prompt_rendering.py` 与 `test_trajectory_prompts.py` 全绿，基线逐字一致 |
| 2 | 契约上线：`ports/vehicle.py` 加 `motion_capability()` 与 `COMMAND_FOR_MODEL`；三台载具实现 `motion_capability()`；`stream_command` 以新名增加，`stream_velocity` 暂时保留为转调 | `test_engine_command_contract.py` 通过；`pytest tests/ -q` 全绿（此步不改行为） |
| 3 | 律进 `runtime`：新建 `runtime/motion/tracking.py` 与 `runtime/motion/loop.py`，载具构造接收 `segment_runner`，`execute_segment` 改三行委派，删 `SteppedMotionTracker`、`AirSimMotionTracker`、`_apply_reference` | `test_motion.py`、`test_projectairsim_motion.py`、`test_airsim_vehicle.py` 通过；UE5 数值动作实机一次 |
| 4 | 命令类型化落地：删 `stream_velocity`；车拒收 `HolonomicCommand`；`hold_velocity` 转调 `stream_command` | 负例测试通过；UE5 巡航与目标接近各一次，UE4 数值动作一次 |
| 5 | `session.py` 拆分：`vehicles.py`、`weather.py`、`views.py`；UE5 新增 `build_ports` 与 `verify_ports`，UE4 加自检 | `test_projectairsim_session.py`、`test_airsim_session.py`、`test_vehicle_switch.py`、`test_hidden_pin.py` 全绿，无实机 |
| 6 | 进程栈与入口：`TASK_REQUIREMENTS` 迁 `assembly/requirements.py`；`SimulatorPorts.capabilities` 收参数；`adapters/http` 改从 assembly 取能力与标签；`mission_child`/`registry` 改 import 前缀 | `test_server_api.py`(56)、`test_capability_gating.py`(9)、`test_runtime_catalog.py`(18)、`test_family_router.py`(3) 全绿 |
| 7 | UE4 实机回归：数值动作、目标接近；巡航补测通过后把它加进 `simulators/airsim/runtime.py` 的 `ENABLED_TASKS` | 三类任务在 UE4 上各跑一次，`/api/status` 的 `task_kinds` 多一项 |

第 1 步与引擎无关，可以最先合。第 3 步是唯一会影响实机时序的一步，出问题退回第 2 步。

原方案与裁决冲突的七条已按裁决改写（R1、R6、R7、R9、R13、R14、R15），不再是开放项；剩下第 8 条是唯一未决项。

裁决 R1（不再是开放项）：`execute_segment` 的采样循环在 `runtime/motion/loop.py`，引擎没有 tracker。引擎只实现 `stream_command(command, duration_s)` 加 `current_pose`/`motion_capability`/`collision_state`/`cancel`/`hover`；`Vehicle` 槽由 `runtime` 的 `GuidedVehicle(streaming, loop)` 组合对象兑现，装配层填槽。过渡期允许引擎保留三行委派，最终删除。

裁决 R9（不再是开放项）：`SEGMENT_PERIOD_S = 0.030` 与 `1/MotionLimits.control_hz` 都写在 `runtime/motion/loop.py`，不进配置。正文第二、七节里的“写死/进配置”对比作废。

裁决 R14（不再是开放项）：两份 `CameraIntrinsics` 实现都保留，只加一条主点差不超过 0.5 px 的断言。`domain/observation.py::CameraIntrinsics.from_horizontal_fov` 取 `w/2-0.5`，`simulators/airsim/camera_model.py::intrinsics_from_horizontal_fov` 取 `w/2`，不做统一。

裁决 R13（不再是开放项）：只外置 `adapters/trajectory_planner/prompts/` 的三个文件。`task_parser/prompt.py`、`task_completion/prompts/visual_goal_completion.py`、`target_detection/prompts/*` 继续留在 Python，不进模板。

裁决 R15（不再是开放项）：`streaming_views.py` 留在引擎包，运行时路径不 import 它；加测试守住这条。`views.py` 只做机位循环本身，不做测量。

裁决 R6（不再是开放项）：越点走廊常量落 `algorithms/motion/corridor.py`，按 `MotionModel` 取值，`ACKERMANN` 用 0.6 m，`HOLONOMIC` 取 `MotionLimits.position_tolerance_m`；配置 `motion.passed_lateral_tolerance_m` 可覆盖；不从 `min_turn_radius_m` 推导。引擎删除 `ROVER_PASSED_LATERAL_TOLERANCE_M` 与 `rover_tracking_limits()`。

裁决 R7（不再是开放项）：`PositionHoldVehicle.hold_velocity` 保留，实现内部转调 `stream_command`，不给位置保持第二个下发入口。

**8. UE4 的 `free_navigation` 什么时候打开**
默认：维持现状，不在 `ENABLED_TASKS` 里，直到实机飞过一次。
备选：直接打开，能力矩阵与 UE5 对齐。取舍明确：打开后 `/api/status` 会承诺一个从没飞过的任务，失败时问题落在引擎与配置上，而不是"没承诺过"。
