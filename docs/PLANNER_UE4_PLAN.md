# UE4 与 UE5 双仿真器支持方案 v2

目标分支：`main`。旧分支指 `low-altitude-level-view-visual-goal-navigation`。

本文件未被 git 跟踪，切分支不会丢，确认后请提交到 `main`。

v1 到 v2 的变化：吸收 12 项决策，并把架构从"两个分支加 if"改为声明式 runtime 注册表。第 3 节记录被推翻的 v1 结论。

## 1 目标

1. 废弃 `airsim-flight-console`，前端与后端都在本仓库。
2. 保留 UE5（ProjectAirSim）能力，新增 UE4（AirSim）能力，可随时切换两族场景。
3. UE4 这条链的功能逻辑以旧分支为准，整树复制，不做适配式重写。
4. 仿真器进程生命周期以 `airsim-flight-console` 的 `EnvironmentService` 为准。
5. 架构按 N 个仿真器设计，不按两个设计。

第 5 条不是审美要求。`planner/__init__.py:1` 仍是 "CARLA vision-language navigation skeleton"，`planner/domain/pose.py:13-14` 还把 CARLA 写成坐标转换的对端，而仓库里已无任何 CARLA 代码。这套 planner 已经被换过一次后端，注释是那次换代的残留。按两个仿真器设计的下场已经发生过一次。

## 2 决策记录

| 编号 | 决定 |
|---|---|
| D1 | 功能逻辑抄旧分支；仿真器生命周期抄 `airsim-flight-console` |
| D2 | 旧 `planner/` 整树复制为 `planner/ue4/`，只改导入前缀 |
| D3 | UE5 不做俯视任务。俯视只在 UE4 上提供，UE5 留待后续验证，见 7.1 |
| D4 | 6 张 UE4 图全登记，不加验证标记 |
| D5 | UE4 不提供 `free_navigation` |
| D6 | 端口不做两族统一：UE4 用 41451，UE5 用现有的 8991 与 8992 |
| D7 | `settings.json` 全局一份，写在 `simulator.ue4.settings_file` |
| D8 | 相机参数以 `settings.json` 为准，启动时校验两边一致 |
| D9 | UE4 隐藏回起点与轨迹层，后端返回不支持 |
| D10 | UE4 提供天气，移植 `weather_service.py` |
| D11 | UE4 worker 跑独立进程与环境，保留旧子进程模型 |
| D12 | 不加总开关，回退靠 git 提交切分 |

## 3 被推翻的 v1 结论

| v1 | v2 | 原因 |
|---|---|---|
| 生命周期以 main 为准 | 以 flight-console 为准 | D1 |
| UE5 保留 `--sim-only` 换图不断流 | 失去，见第 7 节 | D1 的代价 |
| `SimulatorSupervisor` 扩成按族取脚本 | 删掉，换成通用的 `SimulatorStack` | D1 加可扩展性 |
| 天气字段名在 UE4 适配器里写 if 映射 | 别名表进注册表，成为数据 | D8 |
| 前端按 `family == "ue4"` 隐藏控件 | 前端只读能力位，不认识族名 | 可扩展性 |
| 新增 `scripts/launch_airsim_xpra.sh` | 不新增脚本，argv 模板写在注册表 | D1 |

## 4 架构

```
主进程（零仿真器依赖）
  console/ 前端            只读能力位渲染
  adapters/http/           路由，沿用 main
  application/             ServerController，沿用 main
      |                       |
  simulator/registry     simulator/stack
  声明式族注册表          flight-console 状态机，实现一次
      |
  ---------------- 进程边界 ----------------
      |                       |
  runtimes/ue5            runtimes/ue4
  子进程入口               子进程入口
      |                       |
  adapters/projectairsim   planner/ue4/**  旧分支整树复制
  projectairsim client     airsim + msgpack-rpc client
```

### 4.1 为什么必须有进程边界

D4 要求运行时随时切族，D11 说明两族的 Python 依赖可能不能共存于一个解释器。两条同时成立，只有一种解法：族专属代码不在主进程加载。主进程只加载契约与注册表。

主进程若直接 import 两族适配器，切族就是把 `msgpack-rpc` 的老 `tornado` 与 `pynng` 装进同一个 `sys.modules`。旧分支专门写了 `adapters/airsim/rpc_loop.py` 绕开 msgpack-rpc 0.4.1 与 Tornado 4.5 的循环重启问题，这条链本来就脆。

顺带的好处：UE4 段错误、旧 tornado 循环崩掉都带不垮 HTTP，前端仍能看日志与状态。旧分支已经是 `SubprocessPlannerWorker` 模型，这条边界几乎免费。

### 4.2 契约层 `planner/contracts/`

跨进程唯一允许共享的模块。纯数据，无仿真器依赖，无第三方依赖。

- `TaskSubmission`、`WorkerEvent`、`ServerStatus` 的传输形态（沿用 main 的 dataclass，去掉仿真器相关字段）
- `Command`：`submit`、`stop`、`home`、`activate`、`set_view`、`set_weather`、`shutdown`、`terminate`
- `Capabilities`：见 4.4
- 编解码：长度前缀 JSON，一行一条

`terminate()` 的语义照抄旧 `ports/backend_runtime.py` 的注释：返回后该 worker 不可能再发飞行指令。这是安全属性，写进测试。

### 4.3 栈层 `planner/simulator/`

把 flight-console 那套验证过的逻辑实现一次，供所有族共用。

| 来源 | 去处 | 说明 |
|---|---|---|
| `environment_service.py:31-347` 的 phase 状态机 | `simulator/stack.py` | `idle`→`stopping`→`starting`→`initializing`→`ready`，失败进 `error` |
| 同文件 `_wait`、`snapshot(refresh)` 自愈、`instance_id` 防串话 | `simulator/stack.py` | 原样搬 |
| `scene_processes.py:180-200` 的 `/proc/<pid>/exe` 与 argv 标记比对 | `simulator/ownership.py` | 原样搬 |
| `scene_launcher.py:33-75` 的 xpra 命令 | `simulator/argv.py` | 拆成模板加占位符，不再是 UE4 专用 |
| `weather_service.py` 的 msgpack 天气 RPC | `runtimes/ue4/` | 属族实现，不进栈层 |

探针做成小函数注册表，族差异只是列表内容：`window` 查窗口标题、`ports` 查监听、`rpc` 探 AirSim RPC。

栈层不含任何 `if family`。加第四个仿真器时这一层零改动。

### 4.4 能力位 `Capabilities`

前端与后端分支的唯一依据。字段：

```yaml
vehicles: [rover, drone]        # 载具白名单
view_modes: [...]               # 机位，空列表则隐藏机位控件
actions: [home, models, trajectory, video]   # 支持的动作
task_kinds: [...]               # 可解析的任务类型
weather: {fields: {canonical: family_name}, presets: [...]}
```

`GET /api/world/options` 与 `GET /api/status` 带上当前生效的 `capabilities`。前端只做 `if (!caps.view_modes.length)`，代码里不出现 `ue4` 或 `ue5` 字样。

不在 `actions` 里的调用返回 501 与 `{unsupported: "home"}`，不静默成功。

`weather.fields` 是"规范名到族内名"的映射表。D8 的 `road_wetness` 与 `roadwetness` 差异因此成为配置，适配器里没有 if。

`task_kinds` 同时喂给任务解析：`prompt.py` 按族拼规则文本。UE4 上 `free_navigation` 根本不进提示词，D5 的"解析到就返回不支持"由构造保证，不需要运行时拒绝。

### 4.5 注册表 `config/simulators.yaml`

```yaml
ue5:
  runtime: planner.runtimes.ue5
  stack:
    binary: /data/projectairsim/...
    ports: {xpra: 10010, topics: 8991, services: 8992}
    argv: ["{binary}", "{map.launch_map}", "-windowed", "-ResX=1280", "-ResY=720",
           "-topicsport={ports.topics}", "-servicesport={ports.services}"]
    probes: [window, ports]
  capabilities:
    vehicles: [rover, drone]
    view_modes: [rover_front, rover_chase, drone_front, drone_chase]
    actions: [home, models, trajectory, video]
    task_kinds: [basic_motion, visual_goal_navigation, free_navigation]
    weather: {fields: {rain: rain, road_wetness: road_wetness}, presets: [clear, rain, fog, snow, dust]}
ue4:
  runtime: planner.runtimes.ue4
  stack:
    ports: {xpra: 10010, rpc: 41451}
    argv: ["{binary}", "{map.map_name}", "-settings={settings_file}", "-abslog={abslog}",
           "-windowed", "-ResX=1280", "-ResY=720"]
    probes: [window, ports, rpc]
  capabilities:
    vehicles: [drone]
    view_modes: []
    actions: [weather]
    task_kinds: [basic_motion, visual_goal_navigation, visual_goal_above]
    weather: {fields: {rain: rain, road_wetness: roadwetness}, presets: [clear, rain, fog, snow]}
```

`config/maps.yaml` 每条只加一个 `family` 引用与族内字段：

```yaml
  - id: airsimnh
    family: ue4
    label: AirSim 郊区 NH
    binary: /data/airsim/AirSimNH/LinuxNoEditor/AirSimNH/Binaries/Linux/AirSimNH
    map_name: AirSimNH
```

## 5 加第三个仿真器的成本

假设要接回 CARLA：

| 要做 | 量 |
|---|---|
| `config/simulators.yaml` 加一段 | 配置 |
| `config/maps.yaml` 加若干条 `family: carla` | 配置 |
| `planner/runtimes/carla/` 子进程入口，装配现有 workflows | 约 150 行 |
| `planner/adapters/carla/` 实现 `Vehicle`、`StreamingVehicle`、`ObservationSource` | 真正的活，约 1000 行 |

零改动：前端、HTTP 层、`ServerController`、栈层、注册表加载、天气、任务解析、配置校验。

对照 if-else 方案：`server_api.py` 5 处分支、`console/app.js` 6 处、supervisor 4 处、prompt 2 处、天气 1 处，且每加一族要全量回归。

## 6 实施顺序

分支 `feat/ue4-dual-runtime` 上已落地（本地 847 测试通过）：

| 步 | 内容 | 提交 |
|---|---|---|
| 1 | 旧分支整树复制到 `planner/ue4` | `fed3373` |
| 2 | 族注册表、能力位、maps 扩字段 | `5cb0565` |
| 6 | 前端按能力位渲染 | `df7f3f3` |
| 7b | 导入边界测试 | `5cb0565` |
| — | 上机环境自检 | `ca2ed71` |
| — | 未实现驱动的 501 拦截 | `d03cf77` |

未落地：步 4（`xpra_start` 进程驱动）、步 5、步 6 的 UE4 运行时入口、步 8 依赖拆环境。
详见 `docs/UE4_BRINGUP.md` 第 0 与第 4 节。

原计划的完整顺序，供后续对照：

D12 不加开关，所以提交粒度就是回退单位。

1. `chore: 复制旧分支规划器到 planner/ue4`。纯机械，无行为变化。
2. `feat(contracts): 抽出跨进程契约与 Capabilities`。带编解码单测。
3. `feat(simulator): 声明式注册表与配置校验`。带校验单测。
4. `feat(simulator): SimulatorStack 移植 EnvironmentService 状态机`。带假 runner 单测。此提交起 UE5 换图行为变化，见 7.2。
5. `feat(runtime): ue5 子进程入口`，主进程改为走 IPC。
6. `feat(runtime): ue4 子进程入口加天气`。带假客户端单测。
7. `feat(console): 按 capabilities 渲染`。
7b. `test: 导入边界测试`。
8. `chore(deps): UE4 依赖独立环境加锁定版本`。

第 7b 条是护栏，成本约 20 行：断言导入 `planner.server_bootstrap` 与 `planner.adapters.http.server_api` 后，`sys.modules` 不含 `airlab`、`airsim`、`msgpackrpc`、`projectairsim`。这条测试失败就是有人在主进程里直接 import 了族代码。

## 7 已知障碍

### 7.1 D3：UE5 没有机体相机位姿 API

查了 ProjectAirSim 官方 API 文档（iamaisim.github.io/ProjectAirSim/api.html）。结论是这里不是"适配一下"的问题。

| 旧版俯视用到的调用 | UE5 对应 | 情况 |
|---|---|---|
| `getMultirotorState` 读机体位姿 | `Drone.get_ground_truth_pose()`、`get_ground_truth_kinematics()` | ok 有 |
| `simGetCameraInfo(camera_id)` 读相机安装位姿 | 无 | err 缺 |
| `simSetCameraPose(camera_id, pose)` 写机体相机位姿 | 无 | err 缺 |

UE5 的 Drone 传感器里 Camera API 只有一项：`get_images(camera_id, image_type_ids)`。没有任何相机位姿读写。API 文档开头也写明 NOT a goal to maintain strict backward compatibility with GitHub AirSim project APIs，客户端已从单一 client 对象改成 Client、World、Robot 实体三层。适配层解决的是同名不同参，这里是根本没有那个调用。

三个可选绕过路径：

| 路径 | 做法 | 风险 |
|---|---|---|
| A | 用 `World.spawn_object(asset_path, object_pose)` 生成一个带相机的 actor，再用 `World.set_object_pose` 移动和转向 | 文档未说明 spawn 出的 actor 能否挂相机并出图，必须上机验 |
| B | 场景里预置一架只做云台的载具，从上方用它的 `get_images` 取俯视图 | 不改 API，风险最低，但改变任务语义：俯视由另一台载具提供 |
| C | 放弃 UE5 俯视，只在 UE4 上做 | 无风险，回到 v1 建议 |

一个必须记住的不确定性：main 用的 `switch_streaming_view` 并不在那份公开 API 清单里（main 通过 `_call_world("switch_streaming_view")` 调它）。说明文档不是全集，最终判据是在你机器上跑一次：

```bash title="上机确认 UE5 是否有相机位姿 API"
python -c "from projectairsim import Drone, World; \
print('drone:', [m for m in dir(Drone) if 'camera' in m.lower() or 'sensor' in m.lower()]); \
print('world:', [m for m in dir(World) if 'camera' in m.lower() or 'pose' in m.lower()])"
```
`scripts/calibrate_streaming_views.py:4` 也已写明 ProjectAirSim has no free-spectator camera，与文档一致。

**已定：选路径 C，UE5 不做俯视，留待后续验证。**

落地的形式只有两件事：

1. 注册表里 `ue5.capabilities.task_kinds` 不含 `visual_goal_above`。这是唯一生效的门：提示词不拼该规则，任务解析不会产出它，前端不会收到它。UE4 不受影响，旧版三个调用在 AirSim 上全部可用。
2. `docs/UE4_BRINGUP.md` 里保留上面那条 `dir()` 命令与判读方式。

不预埋任何东西。不写 UE5 俯视的桩代码、不加 `camera_pose` 能力位、不在 `projectairsim_session.py` 里留空方法。理由：桩无法被测试，无人跑过的桩会腐烂；而 `task_kinds` 已经是声明式的开关，再加一层就是冗余。
### 7.1b 日后验证通过后的启用步骤

三条路径都不改控制逻辑，差别只在"俯视图从哪来"。旧版的感知下游（`top_geometry`、`above_target_association`、`visual_goal_above` 的完成判定）与 `point_at_world` 那套向量计算都能直接用。

1. 跑 `dir()` 命令，确认走路径 A（`World.spawn_object` 加 `set_object_pose`）还是路径 B（预置云台载具）。
2. 在 `planner/adapters/projectairsim/` 里实现一个俯视观测源，输出旧 `AboveCamera` 同构的接口：一个能存还原位姿、能把视线指向世界点的方法，加一路图像与深度。
3. 注册表里给 UE5 的 `task_kinds` 加上 `visual_goal_above`。到这一步为止前端与 HTTP 层零改动。

### 7.2 UE5 换图会退化

D1 选 flight-console 的 `xpra start --start-child --exit-with-children`，代价明确：

| | 现在 main | v2 |
|---|---|---|
| UE5 换图 | 只重启 UE 进程，Xvfb 与 xpra 保留，画面不断流（`run.txt:27`） | 整套 xpra 会话重启，画面断一次 |
| 结构 | 先起 bare Xvfb，再 `xpra shadow` 挂已有显示 | `xpra start` 自带 `--xvfb` |
| 就绪判据 | 窗口标题加三端口 | 探针列表，两族不同 |

换来的是你验证过的状态机、超时、`instance_id` 与 error 自愈。前端有恢复路径：`console/app.js:329-334` 在 worker 就绪后重建 xpra 帧，断流期显示"xpra 未连接"。

### 7.3 端口

已定不做两族统一：UE4 用 AirSim 默认的 41451，UE5 沿用现有代码的 topics 8991 与 services 8992。两族的端口集由注册表的 `stack.ports` 声明，栈层探针按族读取，代码里没有硬编码端口。两族仍共用 display `:104`、xpra 10010 与后端 8000，同一时刻只跑一个仿真器。

### 7.4 UE4 实机未验证过

旧 `README.md:435` 原文：浏览器接口和真实子进程已用离线替身验证，实际 AirSim、真实模型和前端地图切换仍需联调。旧分支没有 `tests/` 目录。所以 UE4 的行为保真来自代码照抄，不来自测试通过。

## 8 交付物

你本地无环境，一次联调，所以除了代码：

1. `scripts/probe_ue4_env.py`：上机第一步。检查 6 个二进制存在可执行、`settings.json` 可读、`ApiServerPort` 与注册表一致、xpra 与 RPC 端口空闲、`xpra` 可执行、UE4 依赖环境里 `airlab` 与 `msgpackrpc` 可导入。每项失败给出具体命令。
2. `docs/UE4_BRINGUP.md`：上机顺序，每条命令配期望输出与出错看哪个日志。含 7.1 的客户端 API 确认命令。
3. `scripts/smoke_airsim_drone.py`：参照现有 `scripts/smoke_projectairsim_drone.py`。只连、读位姿、悬停、停，不跑任务。
4. 全套假客户端单测，不需要 UE 与仿真器即可跑。

## 9 决策状态

12 项全部关闭。本文件可直接开工，无需再确认。

| 编号 | 结论 |
|---|---|
| D1 | 功能逻辑抄旧分支，生命周期抄 flight-console |
| D2 | 旧 `planner/` 整树复制为 `planner/ue4/`，只改导入前缀 |
| D3 | UE5 不做俯视，用 `task_kinds` 做唯一开关，不预埋桩 |
| D4 | 6 张 UE4 图全登记，不设验证标记 |
| D5 | UE4 不提供 `free_navigation` |
| D6 | 端口不统一：UE4 用 41451，UE5 用 8991 与 8992 |
| D7 | `settings.json` 全局一份 |
| D8 | 相机参数以 `settings.json` 为准，启动校验两边一致 |
| D9 | UE4 隐藏回起点与轨迹层，返回 501 |
| D10 | UE4 提供天气，移植 `weather_service.py` |
| D11 | 每族一个 runtime 子进程，独立环境 |
| D12 | 不加总开关，回退靠第 6 节的提交切分 |
| 监听 | 默认 `127.0.0.1:8000`，部署时传 `--host 0.0.0.0` |

## 10 free_navigation 的适用范围

摘要：无终点、无距离的沿路巡航，只有人工停才结束。`workflows/free_navigation/workflow.py:1-10` 写明循环没有完成条件。依赖 main 新增的 `services/free_space.py`（283 行）与 `capabilities/planning/geometric_planner.py`，旧分支没有这两个模块。

一处纠正：它不是无人车专属。`adapters/task_parser/tasks/free_navigation.py` 里同时有 `FREE_NAVIGATION_RULES`（地面）与 `AERIAL_FREE_NAVIGATION_RULES`（空中），无人机版本是已实现的。所以 UE4 上不做的决定不变，但 UE5 上不要把它当无人车残留删掉，现在 main 的无人机任务集里有它。
