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

## 7 实测结论与残留未知

环境安装、依赖版本、自检命另的完整记录在 `docs/ENVIRONMENT.md`。这里只记结论。

### 7.1 已验证（从本节原来的"无法本地验证"移出）

**两族依赖不能共存于一个解释器——确定，不是猜测。**

- `airsim` → `msgpack-rpc-python` 0.4.1 → `msgpack-python` 0.5.6，提供模块 `msgpack`（1.2.4 也叫这个名字）。
- `msgpackrpc/transport/tcp.py:12-13` 调 `msgpack.Packer(encoding=...)` 与 `Unpacker(encoding=...)`，`encoding` 在 msgpack 1.0 已删。
- 对照实验：`vln-ue4`（msgpack 0.5.6）下 `client.ping()` 报 `TransportError`（ECONNREFUSED，序列化链路是通的）；
  混装（msgpack 1.2.4 抢到模块名）下同样的调用报 `TypeError: unexpected keyword argument 'encoding'`。
  两边的 `import airsim` 与 `MultirotorClient()` 都成功，只在真正发包时分岔。
- **副作用：`AGENT_PROMPT.md` 第 1 步第 3 条的判据给假绿灯。** `import msgpackrpc, pynng, msgpack`
  不碰打包 API，混装下照样打印 `coexist ok`。判共存要用 `client.ping()`。
- 第 6 节"环境：UE4 运行时独立 conda 环境"从预防措施升级为实测约束。

**3.11 能装整套 UE4 客户端，不必降级 Python。** `msgpack-python` 0.5.6、`tornado` 4.5.3、
`msgpack-rpc-python` 0.4.1 三个 sdist 在 3.11.18 上均构建成功。代价：`msgpack-python` 的 wheel 只冇 10 KB，
`msgpack._cmsgpack` 不存在，打包走纯 Python fallback，取图延迟上这是一个已知开销点。
`BRINGUP` 第 1 节那两条 `pip install` 照抄会失败，原因与正确命令见 `ENVIRONMENT.md` 第 3 节。

**UE5 有相机位姿写入 API。**

```
Drone.set_camera_pose(camera_id: str, pose: Transform, wait_for_pose_update: bool = True) -> bool
Drone.reset_camera_pose(camera_id: str, wait_for_pose_update: bool = True) -> bool
Drone.camera_look_at_object(camera_id: str, object_name: str, ...) -> bool
```

`set_camera_pose` 写的是载具相机的相对位姿；`World` 另有 `spawn_object` / `set_object_pose` / `get_object_pose`。
按 `AGENT_PROMPT.md` 第 1 步的判读规则，这命中"UE5 可以做俯视"。此前认为"零命中"是因为只 `git grep`
了 C++ 名 `setCameraPose`，没查 Python 端的 `set_camera_pose`。

**建议重开第 6 节一条边界（按硬规则 8 的方式提出，不默改）：** "UE5 不做 `visual_goal_above`" 的前提
是缺 `camera_view` 端口，而实测 `set_camera_pose` 存在，前提已不成立。改成"做"要动
`planner/simulators/projectairsim/runtime.py` 的 `PORT_SLOTS` 与提示词过滤顺序，**等你拍板**。

**`settings.json` 与配置的对应关系已读到。** `/data/airsim/settings.json`：`SimMode: Multirotor`，
载具 `Drone_1`，相机 `front_center` + `down_center`（俯视用的下视相机已经在），无 `ApiServerPort`，
即默认 41451，与 `config/simulators.yaml` 的 `airsim.ports.rpc` 一致。旧 config 的 `airsim.camera_id`
随 `planner/ue4/` 一同删掉了，这顶"两边对不上"只剩 settings.json 一侧可核。
`scripts/probe_ue4_env.py` 在 `vln` 下实测 `25 ok, 1 warn, 1 FAIL`，FAIL 就是 airsim 没装在主环境——
按本节上面的结论，这是正确状态，不要为把它跑绿而往 `vln` 里装 airsim。

**基线数字。** `pytest -q` 实测 **851 passed**（本文件上面那个由脚本生成的计数与本节无关）。
`BRINGUP` 第 3 节的 `847 passed` 过时。另：`projectairsim` 已在 PyPI 上（`0.2.0/0.3.0/1.0.1/1.0.2/1.1.0`），
`AGENT_PROMPT.md` 第 0 步要求的"wheel 路径"可以直接换成 `pip install --no-deps projectairsim==1.0.2`。

### 7.2 仍然未知

- `landed_state` 在现场 AirSim 构建里的实际取值。旧 `adapters/airsim/connection.py:126-155` 用连续两次相同读数判落地。
- 深度图的实际内参。`settings.json` 里 `front_center` 的深度通道是 256x256 / FOV 90，与 planner 期望是否一致要取一次真帧才能判。
- UE4 应用在本机能否真的渲染出图（无桌面会话 + `DISPLAY` 为空）。xpra / Xvfb / `libGL.so.1` 已确认存在，
  但还没起过 UE4 进程。另：可见 3 张 RTX 3080 Ti，另有一张 `0000:68:00.0` 报 `Unknown Error`，要避用。
- **UE5 启动脚本在本仓库路径下起不来**（只读了脚本，未实跑）：`scripts/launch_projectairsim_xpra.sh:47,52,57`
  把二进制路径写死为 `$PROJECT_DIR/sim/...`，`PROJECT_DIR` 由脚本自身位置推得 = 仓库根，而本仓库没有 `sim/`；
  打包好的 UE5 在 `/roxy/sakura/XV3DGS-airsim-vln/sim/`。要建软链还是改脚本支持环境变量，**待定**。
- 旧分支自述实机未联调：旧 `README.md:435`。旧分支没有 `tests/`，UE4 保真来自代码照抄，不来自测试通过。

