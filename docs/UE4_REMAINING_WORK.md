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
| `xpra_start` 进程驱动 | **已落地：`planner/simulator/xpra_start.py` + `router.py`** | 渲染 argv、起进程、三个探针逐个等就绪、按 `/proc` 里的 exe 真路径 + `-settings=` 判归属、只停自己拉起过的栈。生命周期那套（`identity` 快照、`descendants`、`terminate_verified_tree`、`reclaim_stale_display`、`window_ready`、先校验后动作的 stop 阶梯）逐字参照 `airsim-flight-console/scene_processes.py` 与 `environment_service.py`。真机实测见第 7.1 节 |
| AirSim 会话装配 | `planner/simulators/airsim/runtime.py` | 现在只有 `describe()` 与 `PORT_SLOTS` 供能力推导。`build_ports` 曾因无调用方被删。补回时必须同一个提交里带上调用方：`AirSimVehicle`、`AirSimObservationSource`、`AirSimCameraView`、`AirSimWeather`（天气从 `airsim-flight-console/weather_service.py` 移植，走 msgpack RPC） |

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

**`landed_state` 在现场构建里的实际取值已测到。** 字段名就是 `landed_state`（不是 `land_state`），
枚举类是 `airsim.LandedState`，取值 `Landed=0` / `Flying=1`；Blocks 与 Campus 冷启动后稳定报
`Landed`，`z` 分别为 0.67 / 3.56。`connection.py:149` 与 `server_vehicle.py` 里的
`state.landed_state == airsim.LandedState.Landed` 写法与构建一致，不用改。

**UE4 打包应用在本机能渲染、能取图、能切图（驱动真机实测）。**

用线上配置经 `FamilyRouter` 跑完整周期，`vln-ue4` 取帧：

| 动作 | 结果 |
|---|---|
| `restart(blocks)` | 就绪 5s；取图 0.1s、1920x1080、非零率 98.8%；`z=0.67`、`landed_state=0` |
| `restart(campus)` | 就绪 8s；取图 0.2s、1920x1080、非零率 93.1%；`z=3.56`、`landed_state=0` |
| `stop()` | 1.4s，端口释放，**`/tmp/.X104-lock` 无残留** |

两个坑都是 `airsim-flight-console` 已经踩过并写在注释里的，我这版实现照它做才避开：

1. **未 map 的窗口。** seamless xpra 在客户端接上之前不 map 窗口，UE4 就不渲染：RPC 应答正常，
   `simGetImages` 永不返回。解法是 `xpra info` 找到 `windows.<id>.pid`、校验归属、
   再 `xpra control map` + `focus`。我先前误判成"窗口模式不可用"，改用 `-RenderOffscreen`
   把症状盖掉了——那个参数连桌面画面一起去掉，已从 `config/simulators.yaml` 撤掉。
2. **stale 显示锁。** Xvfb 只在正常退出时删 `/tmp/.X<n>-lock`；被强杀后锁留着，下一次同 display
   启动直接失败，而且引擎日志一个字节都不会写（子进程根本没起来）。表现是"切图超时 180 秒、
   没有任何日志"，真因是一个锁文件。启动前必须走 `reclaim_stale_display`：锁里的 PID 已死才删，
   还活着就报错并给出可执行建议，绝不删别人的显示号。

顺带一条：campus 从来不是"图太大启动慢"，8 秒就好。之前那次失败只是撞上了上面第 2 条。

### 7.2 仍然未知

- 深度图的实际内参。`settings.json` 里 `front_center` 的深度通道是 256x256 / FOV 90，
  真帧已能取到（读数 0.3m–16.3km，中位 22m），但与 planner `camera_model` 的期望是否一致还没逐项核。
- **UE5 启动脚本在本仓库路径下起不来**（只读了脚本，未实跑）：`scripts/launch_projectairsim_xpra.sh:47,52,57`
  把二进制路径写死为 `$PROJECT_DIR/sim/...`，`PROJECT_DIR` 由脚本自身位置推得 = 仓库根，而本仓库没有 `sim/`；
  打包好的 UE5 在 `/roxy/sakura/XV3DGS-airsim-vln/sim/`。用户已定方向：不走脚本，直接驱动二进制。
  UE5 那族要不要一并换成 `xpra_start` 驱动，**待定**。
- GPU：可见 3 张 RTX 3080 Ti 正常，另有一张 `0000:68:00.0` 报 `Unknown Error`，要避用。
  UE4 目前没显式指定适配器也能跑。
- 旧分支自述实机未联调：旧 `README.md:435`。旧分支没有 `tests/`，UE4 保真来自代码照抄，不来自测试通过。

