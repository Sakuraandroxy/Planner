# UE4 剩余工作清单

本文件由脚本对 `origin/low-altitude-level-view-visual-goal-navigation` 与 `HEAD` 的 `planner/` 逐个 AST 比对生成，不要手改数字。

- 两版同名 py 文件：148
- 只差格式、导入与 docstring：69 个，无需处理
- **旧版有定义而新版缺失的共享文件：7 个**，见第 2 节

第 2 节是共享层合并的真实工作量。口径是"UE4 会失去什么"，不是"文本有没有差异"，
所以比早期估的 48 个小一个量级。

## 1 已落地（不用重做）

| 项 | 位置 |
|---|---|
| AirSim 适配层接到共享类型 | `planner/simulators/airsim/` 14 个模块，对 `Vehicle`、`StreamingVehicle`、`PositionHoldVehicle`、`ObservationSource` 零缺失方法 |
| 俯视支持层进入共享层 | `domain/visual_goal_above.py`、`domain/above_polygon.py`、`ports/camera_view.py`、`ports/top_detector.py`、`capabilities/viewing/*`、`capabilities/targeting/` 的 top 与 above 系列、`capabilities/completion/visual_goal_above.py`、`capabilities/task_validation/visual_goal_above.py`、`config/tasks/visual_goal_above.yaml` |
| 端口束与能力推导 | `planner/simulator/ports.py`、`planner/simulator/registry.py`、`planner/ports/capabilities.py` |
| 两族共用 argv 渲染 | `planner/simulator/argv.py` |
| 架构规则测试 | `tests/test_architecture.py` |

## 2 待裁决的共享文件

这些文件 UE5 也在用，改动同时影响两侧，必须两侧都验证。

### planner/adapters（2 个）

- `planner/adapters/model_visual_inputs.py` 旧 69 / 新 52 行 → 旧有而新缺：`_normalized_intrinsics`

- `planner/adapters/trajectory_planner/qwen_vl_planner.py` 旧 131 / 新 111 行 → 旧有而新缺：`_chat_url`, `_image_url`

### planner/application（1 个）

- `planner/application/server_controller.py` 旧 251 / 新 905 行 → 旧有而新缺：`ServerController._monitor`, `ServerController._reconnect`, `ServerController._replace`, `ServerController._retry_preparation`

### planner/domain（2 个）

- `planner/domain/pose.py` 旧 58 / 新 152 行 → 旧有而新缺：`WorldPose.__post_init__`

- `planner/domain/server.py` 旧 68 / 新 105 行 → 旧有而新缺：`WorkerMode`

### planner/errors.py（1 个）

- `planner/errors.py` 旧 56 / 新 63 行 → 旧有而新缺：`ConfigurationError`, `PlannerError`

### planner/services（1 个）

- `planner/services/position_hold.py` 旧 159 / 新 144 行 → 旧有而新缺：`PositionHold._wait`, `PositionHold.observation_ready`, `PositionHold.wait_for_observation`


## 3 还没搬进共享层的旧分支文件

| 文件 | 为什么还没搬 |
|---|---|
| `planner/workflows/visual_goal_above/` 共 8 个 | 要配 `TaskKind.VISUAL_GOAL_ABOVE` 与 task_parser 注册才跑得起来，见第 4 节 |
| `planner/adapters/task_parser/tasks/visual_goal_above.py` | 新版 task_parser 被重写过（旧 348 行 / 新 613 行）。这里是合并，不是复制 |
| `planner/adapters/target_detection/above_client.py` | 俯视检测的模型客户端，与 task_parser 一起接 |
| `planner/visual_goal_above_bootstrap.py`、`bootstrap.py`、`server_worker.py`、旧 `server_bootstrap.py`、旧 `application/server_controller.py` | 旧装配层。不要整棵搬：把心跳与停止接管的语义挑出来并入，见第 5 节 |
| 旧 `planner/adapters/http/server_api.py`、旧 `config/` 装配、旧 `planner/ports/backend_runtime.py` | 功能已被新版覆盖，不搬 |

取回任意文件：

```bash
git show origin/low-altitude-level-view-visual-goal-navigation:planner/workflows/visual_goal_above/workflow.py
```

## 4 俯视任务接线顺序

1. `planner/domain/mission.py` 的 `TaskKind` 加 `VISUAL_GOAL_ABOVE`，`TaskParameters` 并上 `VisualGoalAboveParameters`。
2. `planner/adapters/task_parser/tasks/` 加 codec（从旧分支取），在 `tasks/__init__.py` 注册。
3. `planner/adapters/task_parser/prompt.py` 增加该类型规则文本。注意 `planner/simulator/ports.py` 的 `TASK_REQUIREMENTS` 已保证 UE5 拿不到该类型（缺 `camera_view`）。提示词必须先按能力位过滤再拼，否则 UE5 会解析出一个跑不了的任务。
4. `planner/workflows/visual_goal_above/` 移入 `planner/workflows/`，导入指向共享层。
5. `planner/application/workflow_router.py` 注册该工作流。
6. `planner/simulators/airsim/runtime.py` 的 `ENABLED_TASKS` 已含 `visual_goal_above`，无需改。

第 2、3 步会改到 UE5 行为，两侧都要验证。每步之后跑 `pytest -q`。

## 5 UE4 运行时还缺的两块

| 块 | 位置 | 要点 |
|---|---|---|
| `xpra_start` 进程驱动 | `planner/simulator/` | argv 模板已在 `config/simulators.yaml` 的 `airsim` 段就位。要实现：渲染并起进程、按 `probes: [window, ports, rpc]` 等就绪、写 pidfile、用 `/proc/<pid>/exe` 加 argv 地图名判所有权、拒绝停非自己拉起的进程。抄 `airsim-flight-console` 的 `scene_launcher.py:33-75`、`scene_processes.py:180-200`、`environment_service.py` 的 phase 状态机 |
| AirSim 会话装配 | `planner/simulators/airsim/runtime.py` | 现在只有 `describe()` 与 `PORT_SLOTS` 供能力推导。`build_ports` 曾因无调用方被删。接上驱动时一并补回：`AirSimVehicle`、`AirSimObservationSource`、`AirSimCameraView`、`AirSimWeather`（天气从 `airsim-flight-console/weather_service.py` 移植，走 msgpack RPC） |

心跳重连（旧 `ConnectionMonitor`、`heartbeat_interval_s`、`readiness_stable_s`）与停止接管（旧 `README.md:428` 描述的 `rotateByYawRateAsync` 交接）都在旧 `ServerController` 里，不在 worker 协议中。旧 worker 的 IPC 只认 `task` 与 `shutdown` 两条命令。装配时只看 worker 接口会漏掉这两块。

## 6 已定的边界，不要重新讨论

| 边界 | 内容 |
|---|---|
| 命名 | 用软件名 `airsim` / `projectairsim`，不用引擎版本号。地图字段叫 `simulator`，无默认偏向 |
| 能力位 | 只能由端口推导。`config/simulators.yaml` 写 `capabilities` 会被配置校验拒绝 |
| UE4 不提供 | `free_navigation`（旧分支没有）、回起点、机位切换、外观模型、`/api/video`、轨迹层 |
| UE5 不做 | `visual_goal_above`，因为缺 `camera_view` 端口，由推导自动成立 |
| 俯视属于谁 | 任务逻辑属共享层，只有 `CameraView` 端口属 AirSim |
| UE5 进程 | 留在服务进程内，不搬进子进程。因此 `projectairsim` 会被主进程加载，`tests/test_architecture.py` 只禁 AirSim 栈进主进程 |
| xpra | 两族共用一套 argv 渲染，模板在 YAML |
| 环境 | UE4 运行时独立 conda 环境，靠子进程隔离依赖冲突 |
| 桩 | 不写无人调用的桩与开关。未实现的路径一律显式报错 |

## 7 无法本地验证的事

- AirSim 与 ProjectAirSim 的依赖能否共存一个解释器。旧分支专写 `rpc_loop.py` 绕 msgpack-rpc 0.4.1 与 Tornado 4.5 的循环重启，这条链本就脆。
- `landed_state` 在现场 AirSim 构建里的实际取值。旧 `adapters/airsim/connection.py:126-155` 用连续两次相同读数判落地。
- `settings.json` 相机名与深度内参 对 旧 config 的 `airsim.camera_id: front_center`、`depth_horizontal_fov_deg`。已定以 settings.json 为准并启动时校验，`scripts/probe_ue4_env.py` 会读两边报差异。
- UE5 是否有相机位姿写入 API。全仓 `git grep setCameraPose` 零命中，`scripts/calibrate_streaming_views.py:4` 自己写着 ProjectAirSim has no free-spectator camera。但 `switch_streaming_view` 同样不在官方 API 文档里却可用，说明文档非全集。确认命令见 `docs/UE4_BRINGUP.md` 第 5 节。
- 旧分支自述实机未联调：旧 `README.md:435`。旧分支没有 `tests/`，UE4 保真来自代码照抄，不来自测试通过。

