# XV3DGS-AirSim VLN 实施方案

目标：用 ProjectAirSim（UE5）替代 CARLA/Blocks，在一个仿真器里同时支持
**无人机 + 无人车**的视觉语言导航闭环，前端展示、后端 VLN、xpra 交互画面与
参考项目保持一致。

参考项目（只读，不修改）：

- `/roxy/sakura/carla-vln`：无人车 VLN 服务、控制台、xpra 闭环、滑窗执行栈。
- `/roxy/zhusai/Planner`：无人机 AirSim 适配器、工作流、任务系统。
- `/roxy/kedior/airsim-flight-console`：控制台交互参考。

所有新代码在 `/roxy/sakura/XV3DGS-airsim-vln`。

## 1. 已验证事实

在本机（Linux，Xvfb + 窗口化 UE5）上实测：

| 环境 | 启动方式 | 端口 | 加载场景 | 结论 |
|---|---|---|---|---|
| Blocks | `sim/Blocks-Linux-1.0.1/Blocks/Binaries/Linux/Blocks-Linux-Shipping` | 8989/8990 | ~5 s | 现有冒烟环境，保留 |
| **CityEnviron** | `sim/CityEnviron/Binaries/Linux/CityEnviron-Linux-Shipping` | 8991/8992 | 5.7–6.8 s | CARLA 风格城市，作为默认地图 |
| **Neighborhood** | `sim/Neighborhood/AirSimNH/Binaries/Linux/AirSimNH-Linux-Shipping` | 8993/8994 | ~6.9 s | AirSim 风格郊区，作为备选地图 |

- CityEnviron：无人机航拍可见高楼、主干道、红绿灯、中心环岛；Rover 在
  `(50, 0, -1)` 出生、`set_rover_controls(engine=0.2)` 3 秒行驶 13.1 m，
  车头相机拍到大街与车道线。地图原点环岛中心是花园，出生点必须选在道路上。
- Neighborhood：无人机在 `(0, 0, -40)` 航拍可见独栋住宅、道路、树木、停放车辆
  （经典 AirSimNH 郊区）；Rover 在 `(0, 0, -2)` 出生、3 秒行驶 13.0 m，
  物理/控制正常。原点附近是公园草地，出生点同样需要标注。
- 两个环境都自带 Rover 插件资产与 `/Drone/Quadrotor1` 等无人机资产；无需
  Unreal 编辑器，`-topicsport/-servicesport` 指定端口即可与 Python 客户端通信。
- `CustomGIS` 模式自带的 mini 瓦片集视觉上仍然像 Blocks；真正的 GIS 城市需要
  外部 GLB/3D Tiles（`tiles-dir` 或 `-gltfDir`）或自行编译 UE 环境，暂不采用。

已有资源说明：服务器上 `/data/airsim/` 还有一套**经典 AirSim（UE4）**环境
（AirSimNH、Blocks、Campus、Shanghai、AbandonedPark 等，供
`/roxy/kedior/airsim-flight-console` 使用，启动入口
`/data/airsim/AirSimNH/LinuxNoEditor/AirSimNH.sh`）。它们与本次下载的
ProjectAirSim v1.0.1 环境是**同一张地图的不同引擎版本/通信协议**：
经典版走 AirSim RPC（41451），ProjectAirSim 版走 topic/service 双端口。
`projectairsim` 客户端不能驱动经典版，反之亦然，因此本项目需要 v1.0.1 的
预编译环境，不复用 `/data/airsim/` 的二进制。

## 2. 地图选型

- **默认地图：CityEnviron。** 道路网络完整，最接近现有 carla-vln 的城市场景，
  适合无人车 VLN 与无人机-无人车协同任务。
- **备选地图：Neighborhood。** 对应参考 Planner 的 AirSimNH 场景，用于
  郊区/低密度任务，以及视觉差异测试。
- **Blocks：** 只用于冒烟测试与回归，不作为展示地图。
- LandscapeMountains：按需下载，暂不纳入里程碑。
- 场景配置文件按地图分目录存放，每张地图标注验证过的出生点与朝向。

## 3. 控制映射（方案 A）

参考 carla-vln 的 `ContinuousExecution` 输出的是 20 Hz 速度指令。Rover 没有
速度流接口，只提供 `set_rover_controls(engine, steering_angle, brake)`。
采用方案 A：

- 保留参考的连续执行栈不修改（队列跟随、后台续规划、滑窗）。
- 新增无人车控制适配器：把速度指令转换为
  - 纯追踪（pure pursuit）：由当前位姿与前方路径点计算转向角；
  - 速度 PID：由目标速度与当前速度计算油门/刹车；
  - 输出 `set_rover_controls(engine, steering_angle, brake)`。
- 保留真实物理与碰撞检测；碰撞恢复继续使用已批准的「原生碰撞传感器 +
  有界倒车 + 清空队列 + 重新规划」。
- 无人机继续使用参考 Planner 的速度流控制（ProjectAirSim `move_by_velocity` /
  `move_by_velocity_async` 等价接口，具体以适配器实测为准）。

## 4. 目标架构

在 carla-vln 的六边形结构上替换仿真适配层：

```
XV3DGS-airsim-vln/
├── config/                 # 服务配置、地图/车辆/场景配置
├── console/                # 控制台前端（双载具）
├── planner/
│   ├── adapters/
│   │   ├── projectairsim/  # 连接、异步桥、坐标、观测、深度、相机、
│   │   │                   # 碰撞、场景、天气、视角
│   │   └── runtime/        # 会话装配（双载具）
│   ├── application/        # 任务解析、阶段分发
│   ├── capabilities/       # 滑窗规划、目标检测、完成判定
│   ├── domain/ ports/ services/ workflows/
│   ├── workflows/          # basic_motion、visual_goal_navigation、
│   │                       # free_navigation（批处理 + 连续）
│   └── ...
├── scripts/                # Xvfb + UE + xpra shadow 启动脚本
├── sim/                    # CityEnviron / Neighborhood / Blocks 可执行环境
└── tests/
```

关键差异：

- `planner/adapters/carla/` → `planner/adapters/projectairsim/`。
- ProjectAirSim 使用 `projectairsim` Python 客户端（topic/service 双端口、
  异步任务返回 `Task` 对象），与 Planner 使用的 `airsim` RPC 客户端 API 不同；
  无人机部分按接口语义移植，而不是复制调用代码。
- 一个场景 JSON 可同时生成无人机与无人车 Actor，M5 验证同屏双载具。

## 5. 里程碑

### M1 骨架与地图

- 从 carla-vln 复制 `planner/`、`config/`、`console/`、`scripts/`、`tests/`
  骨架，删除 CARLA 专有依赖。
- 新建 CityEnviron/Neighborhood/Blocks 三套场景 JSON 与 Robot 配置：
  无人机（nonphysics 与 physics）、Rover、相机（前视/下视/追尾/顶视）。
- 标注各地图验证过的出生点、朝向、相机安装参数。
- 用一次性冒烟脚本验证场景可加载、双载具可生成、相机可出图。
- 本阶段不接模型、不跑 VLN、不改算法。
- 验收：目录、配置与场景就绪；冒烟脚本能在 CityEnviron 中同时生成
  无人机 + Rover 并取到图像。

### M2 ProjectAirSim 适配器

- 连接与生命周期：双端口连接、场景加载/卸载、进程内单例客户端。
- 异步桥：`projectairsim` 的 `Task`/回调 → asyncio，超时与取消。
- 坐标系：NED ↔ ENU/世界坐标、姿态四元数与 rpy 转换、与 carla-vln 的
  `Pose`/`Transform` 语义对齐。
- 观测：RGB、深度（16FC1 float16，+inf=65504）、相机内参、时间戳。
- 碰撞：`collision_info` 主题 → 原生碰撞事件。
- 场景/天气/视角：场景切换、时间/光照、追尾/车载/自由相机。
- 验收：适配器单测 + 真实 CityEnviron 下取图/取位姿/碰撞事件通过。

### M3 无人车工作流（方案 A）

- 速度指令 → 纯追踪 + 速度 PID → `set_rover_controls` 适配器。
- 移植 carla-vln 的滑窗连续执行、任务系统、视觉目标、基础动作、
  `free_navigation`（含碰撞恢复）。
- CityEnviron 没有 CARLA 的车道 API；`free_navigation` 的车道参考改为
  地图/相机前视点或简化版，并在本阶段记录与 carla-vln 的差异。
- 验收：CityEnviron 上完成「直行 N 米」「沿道路一直前进」；停止接口生效；
  碰撞触发倒车后继续。
- M3 实施记录（2026-11-08）：
  - 驱动模型在 CityEnviron 实测标定：`engine` 线性映射到有符号地速
    （23.3 m/s 每单位，正反向一致）；`steering_angle` 产生的偏航率与有符号
    前向速度成正比（约 17.9 °/s 每 (m/s·单位)，|steering|>0.5 轻微饱和）；
    `brake=1` 可从约 8 m/s 在一个控制周期内停住。
  - 速度指令投影到车头方向作为参考速度，速度 PID 只修正模型误差；
    yaw rate 换算成 steering。brake 只在车辆仍向前滚动且超过指令速度时
    介入，否则从静止发出的倒车指令会被刹车锁死。
  - CityEnviron 没有 CARLA HD 车道 API：`free_navigation` 传入
    `lane_reference_provider=None`，沿用提示词的自由空间规则回退。
  - 物理越野车是非完整约束载具：纯 yaw、无平移的原地旋转基础动作在实车
    不可执行；直行、倒车、平移+转向组合和连续路径跟踪均已验证。
  - 实车验收：直行 5 m（终点误差 0.05 m）、倒车 2 m（误差 0.03 m）；
    连续 `free_navigation` 30 s 行驶 60.3 m，途中两次碰撞分别倒车 3 m/6 m
    后继续；共享 hold checkpoint（`POST /api/stop` 使用的停止路径）生效。

### M4 无人机工作流

- 移植 Planner 的无人机工作流与速度控制、起飞/降落、视觉目标闭环。
- 验收：CityEnviron/Neighborhood 上起飞、按速度飞行、取图、目标接近。
- M4 实施记录（2026-11-08）：
  - `ProjectAirSimDrone` 移植 `PositionHold`（位置/偏航增益 + 速度阻尼，
    以仿真步进时钟为观测窗口），每个控制周期发送一个
    `MoveByVelocity(duration=1/control_hz)` 并恰好推进一次仿真时钟，
    与暂停的可步进世界保持锁步。
  - 原生 `takeoff_async` 在有实测爬升（1.38 m）且 `landed_state=FLYING`
    时仍返回 `False`；参考 Planner 同样容忍该怪癖。适配器以
    「未落地 + 爬升超过位置容差」确认起飞，仅在原生返回失败时记 WARNING。
  - 原生 `land_async` 返回 `True` 且飞机降到地面高度后，
    `GetLandedState` 仍长期保持 FLYING（实测 10 s 仿真时间不变）。
    适配器在起飞前记录地面高度，落地后以实测高度（地面 ± 0.1 m）确认
    触地；该语义已写入适配器文档字符串。
  - 话题 socket 是单消费者链路，参考项目为移动取帧单独开只读连接的
    做法不可行：新增 `BorrowedMovingSource` 代理，工作流借用车辆适配器
    已有的相机订阅，`close()` 为空操作，避免工作流关闭时连带关闭
    车辆的异步桥。
  - 测试：`tests/test_projectairsim_drone.py` 22 项、
    `tests/test_position_hold.py` 6 项；全量 612 项通过。
  - 实车验收（CityEnviron 双载具场景，Xvfb :104，未重启仿真器）：
    起飞爬升 1.38 m、位置保持确认；取图 1280×720 RGB+深度
    （深度中位数 27.3 m）；基础前进 1 m 与 3 m 均在停车确认后完成；
    视觉目标闭环（手动 bbox 检测 + 几何规划器 + 连续滑窗执行）
    「前方路面」5.9 s 到达（路径 5.8 m），右侧砖楼「0.80,0.45,0.90,0.55」
    一次 72.5 s / 86.8 m 到达；降落由实测高度确认
    `landed=True pose=(62.26,0.00,0.85) settle=0.0s`。
  - 与参考项目的差异：视觉目标 smoke 使用本地恒真完成检查器替代语义
    模型检查器（M5 接入生产会话的 `config.task_completion`）；
    `Neighborhood` 验收待 M7 补做（当前场景为 CityEnviron）。

### M5 会话、API 与控制台（双载具）

- 双载具会话装配：一个 World 内同时管理 Drone 与 Rover，任务级路由。
- FastAPI 端点与控制台：载具切换、任务提交/停止/状态、实时画面。
- 验收：浏览器控制台可分别或同时操作两台载具。
- M5 实施记录（2026-11-08）：
  - 新增 `ProjectAirSimMissionSession`：一次建连、一个暂停 `World`、
    同时创建 `Rover1` 与 `Drone1`。每个载具拥有独立的工作流栈
    （`BasicMotionWorkflow`、`VisualGoalNavigationWorkflow`、
    `FreeNavigationWorkflow`、解析器与 `ModelCallService`），任务按
    `TaskSubmission.vehicle`（`rover` / `drone`）路由。
  - 载具切换语义：非活动载具先停放——无人机降落并确认触地，无人车刹车；
    无人机仅在任务真正路由到它时执行参考起飞/位置保持。任务结束后无人机
    保持位置、无人车保持刹车。`idle_tick` 在命令队列空闲时推进暂停世界
    （活动无人机 tick 当前位置保持，否则无人车 tick 或裸推进一个 20 Hz 周期），
    保证 xpra 场景持续渲染。
  - `ServerSettings.vehicles/default_vehicle` 与 `/api/status` 的
    `vehicle`/`vehicles` 字段；`POST /api/tasks` 接受 `vehicle`（缺省用
    `default_vehicle`），未知载具返回 400；`POST /api/home` 仅无人车可用，
    无人机活动时返回 400（工作线程侧仍保留 `ProtocolError` 防御）。
  - 控制台：新增「载具」分段按钮与「当前载具」；提交时携带 `vehicle`；
    移除 CARLA 专属的世界/天气/视角 UI（对应 API 端点保留，M7 统一清理）。
  - 任务解析器按载具选择提示词：无人车沿用 carla-vln 的地面载具提示词；
    无人机改用参考 Planner 的 UAV 提示词，并使用航空版规则文本。当前运行时
    的无人机基础动作只支持前向/后退平移与偏航，规则明确拒绝横移与高度变化；
    视觉目标与道路跟随规则改为飞行措辞。
  - 无人机适配器补充 `idle_tick()` 与公开 `landed` 属性；会话关闭顺序为
    工作流/模型调用 → 无人机落地并关闭 → 无人车关闭 → 连接关闭。
  - 测试：新增 `tests/test_projectairsim_session.py` 11 项，补充
    API/控制器/解析器/适配器用例；全量 634 项通过。
  - 实车验收（CityEnviron 双载具场景，未重启仿真器）：
    无人车「向前直行 1 米」2.49 s 完成；无人机「向前飞行 1 米」起飞并
    6.46 s 完成（日志 `Drone airborne; position hold confirmed.`）；
    切回无人车时日志 `Drone parked on the ground.`；30 m 任务中途
    `POST /api/stop` → `cancelled: task was cancelled` + `Stop confirmed`；
    `POST /api/home` 无人车返回 202、无人机活动时返回 400；
    控制台提供载具选择且 `/api/trajectory` 持续返回快照。
  - 待办：xpra 交互画面启动脚本与闭环属 M6；CARLA 会话/适配器及
    世界/天气/视角端点在 M7 清理或明确保留。

### M6 Xvfb + xpra 闭环

- 沿用已验证模式：裸 `Xvfb :N` → 窗口化 UE → `xpra shadow`。
- 启动/停止脚本、端口分配、日志与进程守护。
- 验收：xpra 交互画面可操作 CityEnviron 场景，控制台闭环可用。
- M6 实施记录（2026-11-08）：
  - 新增 `scripts/launch_projectairsim_xpra.sh`：参数
    `[MAP] [DISPLAY] [XPRA_PORT] [TOPICS_PORT] [SERVICES_PORT]`，默认
    `cityenviron :104 10010 8991 8992`。显示号已有对应仿真窗口时（例如手动
    启动的 CityEnviron）只启动 `xpra shadow` 复用窗口；否则依次启动裸
    `Xvfb`、窗口化 UE、`xpra shadow`。启动前检查 xpra 端口占用、失效 X
    锁文件与仿真端口冲突；窗口/端口等待带超时。
  - 进程守护：Xvfb、UE、shadow 均写 pidfile 到 `output/xpra/`，日志写到
    `output/logs/`（UE、Xvfb）与 `output/xpra/logs/`（xpra）。重复运行启动
    脚本会被 xpra 端口检查拒绝。
  - 新增 `scripts/stop_projectairsim_xpra.sh [DISPLAY] [--view-only]`：默认
    先 `xpra stop`，再按 pidfile + `/proc/<pid>/cmdline` 归属校验停止**本脚本
    启动的** UE 与 Xvfb；手动启动的仿真不写 pidfile，绝不会被停止。
    `--view-only` 只停交互画面。停止后端口、锁文件与 pidfile 均清理。
  - 交互画面用 `xpra shadow :DISPLAY,multi-window=no`，单窗口 1280×720，
    HTML5 客户端在 `10010`（控制台默认嵌入同一地址）。
  - 验收（CityEnviron，未重启已有仿真器 PID 994426 / Xvfb :104）：attach
    模式启动 shadow，`http://127.0.0.1:10010/` 返回 HTML 200，
    `xpra info` 报 `server.type=Python/bindings/x11-shadow`、
    `state.windows=1`，截图 1280×720 为真实城市场景（97% 非黑）。
  - 控制台闭环（另开 Xvfb :107 + Firefox 打开 `http://127.0.0.1:8000/`）：
    iframe 自动连接 shadow，画面显示 CityEnviron 实时场景；iframe 内点击和
    按键到达 xpra 服务端（`input.count.button-action/key-action` 递增），
    键盘事件经 XTest 转发到 :104 的焦点窗口（探针 xev 记录到
    `KeyPress` keysym `a`）。UE5 窗口持有指针 grab，因此指针事件由 UE 窗口
    接收，符合交互预期。
  - 从零启动路径用 Blocks 在 `:108 / 10012 / 8995 / 8996` 验证：
    `Xvfb → Blocks → shadow` 全部就绪，HTML 200、截图可见 Blocks 场景；
    `stop_projectairsim_xpra.sh :108` 停掉 shadow、UE、Xvfb 并释放三个端口。
  - 说明：控制台只默认嵌入 `10010`；其它显示号/端口可直接打开 xpra URL 或
    用控制台 `?xpra=host:port` 覆盖。Neighborhood 的脚本路径已就绪，实车
    验收放到 M7。

### M7 测试、文档与验收

- 单元测试、适配器集成测试（真实仿真器，不 mock 物理）。
- README/run.txt、配置说明、已知限制。
- 验收清单：双载具端到端任务、停止语义、碰撞恢复、地图切换。
- M7 实施记录（2026-11-08）：
  - **清理 CARLA 专属代码。** 删除 `planner/adapters/carla/`（11 个文件）、
    `planner/adapters/runtime/carla_session.py`、`planner/domain/weather.py`；
    删除脚本 `calibrate_depth.py`、`launch_carla.sh`、`launch_carla_xpra.sh`、
    `run_visual_goal.py`、`smoke_carla.py`、`stop_carla_xpra.sh`；删除测试
    `test_coordinates.py`、`test_depth_camera.py`、`test_lane_reference.py`、
    `test_map_selection.py`、`test_weather.py`；删除过时的
    `README.carla-vln.md`、`run.carla-vln.txt`。
  - **保留通用能力并移到中立位置。** 深度渲染
    `render_depth` 移到 `planner/domain/depth_visualization.py`；手工观测矩阵
    测试助手 `ue_camera_matrix` 移到 `tests/support.py`，7 个通用测试改用该
    助手。保留 worker 生命周期、深度/相机适配、`FrameHub` 帧流与控制台
    xpra 交互画面。
  - **移除世界/天气/视角端点与配套状态。** 删除
    `GET /api/world/options`、`POST /api/world`、`POST /api/view`、
    `GET/POST /api/weather`；删除 `ViewMode`、`WorldSelection`、
    `WorldCatalog`、`server.view_mode`、`WorkerEventKind.WORLD/WEATHER`、
    控制器的世界/天气/视角状态、会话的 `selection`/`world_catalog`/
    `set_view_mode`、worker 的 `set_view_mode`/`set_weather`。ProjectAirSim
    不能运行中换图，也没有 CARLA 的天气 spectator API。保留的任务端点：
    `/api/tasks`、`/api/status`、`/api/logs`、`/api/trajectory`、
    `/api/stop`、`/api/home`、`/`、`/console/*`，以及装配 `FrameHub`
    时才注册的 `/api/video`。
  - **清理未使用的连接包装方法。** 删除 `ProjectAirSimConnection` 的
    `list_actors`、`set_time_of_day`、`set_sun_position_from_date_time`、
    天气方法组和 `switch_streaming_view`；保留 `scene_configuration`
    （相机内参）与 `get_time_of_day`（冒烟脚本）。
  - **文档。** 新增 `README.md`（架构、环境、三终端启动、地图与配置、
    任务语义、HTTP API、配置要点、测试、已知限制）与 `run.txt`（端口、
    命令、任务语义、地图切换步骤、诊断产物）。`serve_planner.py` 与
    `smoke_*.py` 的启动说明改为 M6 的 xpra 启动脚本，修正
    `smoke_projectairsim.py` 的环境名笔误。
  - **测试。** `conda run -n carla-vln python -m pytest -q`：
    **555 passed, 1 warning**（Starlette 的 httpx 弃用警告）。相比 M5
    记录的 634 项，减少的 79 项全部来自删除的 CARLA 坐标/深度/车道/
    地图选择/天气测试及世界/天气/视角端点用例；通用工作流、任务系统、
    滑窗、双载具会话与控制台测试保留。
  - **Neighborhood 实机验收（新做，未重启 CityEnviron PID 994426）。**
    用 `scripts/launch_projectairsim_xpra.sh neighborhood :105 10011 8993 8994`
    从零启动 `Xvfb → AirSimNH → xpra shadow`；`http://127.0.0.1:10011/`
    返回 200，`xpra info` 报 `server.type=Python/bindings/x11-shadow`、
    `state.windows=1`。双载具场景 `scene_neighborhood_dual.jsonc` 正常
    生成 Rover1 + Drone1，前视 RGB/深度与 Chase 帧可取。
  - **Neighborhood 出生点修正。** 原双载具场景无人机出生在 `(5,0,-3)`，
    正前方约 2 m 是一棵树：基础前进在 x≈6.39 处超时
    （`execution_error: synchronized motion timed out`），降落也无法确认。
    将双载具场景 Drone1 原点改为 `(0,0,-40)`（与已验证的
    `scene_neighborhood_drone.jsonc` 一致；ProjectAirSim 会把载具落到
    地面高度附近），前方道路/草坪通畅。修正后：无人车前进 3 m 实测
    3.01 m（3.9 s）、倒车 1.5 m 实测 1.52 m（3.0 s）、`return_home`
    回到 `(0,0,-1.00)`；无人机起飞爬升 1.287 m（原生返回未确认，
    M4 已记录的怪癖）、位置保持 z=-4.19、取图 1280×720（深度中位数
    11.37 m），前进 0.5 m 3.4 s 完成，降落 `landed=True z=-2.76
    settle=0.0s`。另一次前进 2 m 也成功（7.7 s，x=1.82），但该点正下方
    是树篱/树冠，原生降落未完成并记录 warning；这是落点净空问题，
    不是控制环问题。
  - **地图切换验收。** 用临时配置把 `port_topics/port_services` 指向
    8993/8994、`scene_file` 指向 `scene_neighborhood_dual.jsonc`、
    `sim_config_path` 指向 `config/scenes/neighborhood`，在 8001 端口
    启动第二个规划服务：`/api/status` 从 `connecting` 进入 `idle`，
    worker 日志 `ProjectAirSim ready: scene=scene_neighborhood_dual.jsonc`；
    SIGINT 后 `Application shutdown complete` 并断开连接。原 8000
    规划服务与 CityEnviron 未受影响。结论：换图流程 = 另起一张图的
    仿真器（独立显示/端口）+ 改 `config/base.yaml` + 重启规划服务；
    ProjectAirSim 不支持热切换。
  - **Blocks 启动链路复验。** `:106 / 10012 / 8995 / 8996` 从零启动
    `Xvfb → Blocks → xpra`，HTTP 200、`state.windows=1`；
    `stop_projectairsim_xpra.sh :106` 释放仿真、xpra 与显示号。
    Blocks 没有双载具任务场景配置，只用于启动与回归冒烟。
  - **验收清单状态。** 双载具端到端任务：CityEnviron 的 M5 实机记录 +
    Neighborhood 的 M7 实机记录；停止语义：M3/M5 实机记录（控制环
    检查点 `Stop confirmed`）；碰撞恢复：M3 实机记录（3/6/9 m 倒车后
    继续）；地图切换：M7 Neighborhood + Blocks 记录。Neighborhood 的
    任务级 VLM 闭环（解析器/目标检测/完成判定）未重复跑，沿用
    CityEnviron 的 M4/M5 证据；实机脚本只验证适配器与工作流控制环。
  - **操作提示。** M7 清理后，正在运行的 8000 规划服务仍是旧进程，
    `/api/status` 仍会返回已删除的 `map/vehicle_model/vehicle_color/
    view_mode` 字段；需要用户自行重启该服务才能加载新代码。本次
    验收没有重启或停止 CityEnviron、8000 规划服务与 10010 xpra。

### M8 控制台运维：地图、天气、视角、双载具回原点

用户要求补齐 carla-vln 同类的运维控制：前端点击切地图、选择无人机/无人车、
切天气、改视角、让任一载具回原点。M7 曾删除世界/天气/视角端点，M8 按
ProjectAirSim 的实际能力重新实现。

- **实现文件。** `planner/config.py`（`SimulatorSettings`/`MapEntry`/
  `MapCatalog`）、`config/maps.yaml`、`planner/domain/weather.py`、
  `planner/domain/view.py`、`planner/adapters/runtime/simulator_supervisor.py`
  （脚本托管与 preflight/stop/launch/restart）、`projectairsim/connection.py`
  （天气与 `switch_streaming_view` 包装）、`projectairsim/drone.py`
  （SetPose 回出生点 + 实测贴地确认）、`runtime/projectairsim_session.py`、
  `runtime/worker.py`、`application/server_controller.py`、
  `server_bootstrap.py`、`adapters/http/server_api.py`、
  `console/{index.html,app.js,styles.css}`、
  `scripts/calibrate_streaming_views.py`。4 个 robot JSONC 打开恰好 4 个
  `streaming-enabled` 相机（Rover Front/Chase、Drone Front/Chase）。
- **设计决定。** 换图 = 整套重启（ProjectAirSim 无进程内换图）；无人机
  回原点用瞬时 SetPose，与 carla-vln 无人车语义一致；视角为 4 个固定
  机位循环（无自由观察者相机）；天气为 8 参数 + 预设。换图前必须等
  worker CLOSED（topic socket 单消费者），失败时有界重试并保持
  `connecting` + `sim_error`。
- **实机验收（CityEnviron + Neighborhood，:104 / 10010 / 8991 / 8992）。**
  - `GET /api/world/options` 返回两张地图、标签、当前选择与
    `simulator_owned=true`（启动脚本托管）。
  - 天气：应用 `heavy_rain` 后截图出现雨丝、雾与湿暗路面；恢复
    `clear` 后关闭全部特效。
  - 视角：`scripts/calibrate_streaming_views.py` 在两张地图各跑一轮，
    截图确认顺序为 rover_front → rover_chase（画面出现无人车）→
    drone_front（无人机视角看到前方无人车）→ drone_chase（画面出现
    黑色四旋翼），4 步后回到初始画面。`INITIAL_STREAM_INDEX=0` 一致。
  - 回原点：无人车日志 `Rover returned to start position: x=0.00 y=0.00
    yaw=0.0`；无人机 `Drone returned home: pose=(-0.00,0.00,-2.77)
    landed=True ground_z=-2.77`。
  - 换图：`POST /api/world {"map":"neighborhood"}` → 202 → `connecting`
    → 新 AirSimNH 进程（同显示/端口）→ idle，`status.map=neighborhood`，
    worker 加载 `scene_neighborhood_dual.jsonc`，天气自动重放；
    再切回 `cityenviron` 同样成功，`output/xpra/current_map` 同步更新。
    重启规划服务后按 marker 恢复到 cityenviron。
  - 已知环境限制：本机 `task_parser` 端点返回 403，任务级闭环（含
    “运行中提交换图返回 409”）未能在本次实机复验，改由单元测试覆盖。
- **与 carla-vln 的差异（需在文档中保留）。**
  1. 没有自由 spectator 相机，视角只能在场景的 `streaming-enabled`
     相机间循环；
  2. 任务运行中的视角切换延迟到任务结束后生效（API 返回
     `deferred=true`），不是任意时刻立即切换；
  3. 换图需要整套重启仿真栈，期间服务短暂不可用（carla-vln 是进程内
     `load_world`）；
  4. 天气/视角保存在规划服务内存，换图与重连会重放，但重启服务后重置。
- **测试。** `conda run -n carla-vln python -m pytest -q`：
  **662 passed, 1 warning**（Starlette httpx 弃用）。新增
  `test_simulator_supervisor.py`、`test_weather_domain.py`、
  `test_view_catalog.py`，扩展 config/controller/worker/session/drone/API/
  console 断言。

### M8 补充：载具切换即归位与郊区出生点错开（2026-11-08）

用户确认：两个场景默认同时生成无人机和无人车；控制台点击载具按钮应
**切换即归位**——停放另一台、把选中载具传回出生点、把画面切到它的前视；
同时把 Neighborhood 的出生点显式错开。

- **会话原子操作。** `ProjectAirSimMissionSession.activate_vehicle()`：
  `_park(另一台)`（无人机 cancel+land 并确认触地；无人车 hover 刹车）→
  `return_home(选中载具)` → `set_view_mode(front_view_for(载具))` →
  更新活动载具。`_park()` 改为返回 bool，落地未确认时
  `ExecutionError`，整次激活失败、活动载具不变。新增 `active_vehicle`。
  `return_home()` 语义不变（不切活动载具）。
- **worker/控制器。** worker 新增 `activate(vehicle)` 命令，成功后发
  `WorkerEventKind.ACTIVE`，失败只记 WARNING 并发 `ACTIVE(success=False)`，
  worker 不退出。控制器 `activate()` 仅 idle 可用，乐观更新
  `_vehicle`/`_view_mode`；ACTIVE 成功时把 `_applied_view` 设为该类载具
  前视，失败时回滚到点击前的值。READY/ERROR/换图/关闭清空待定激活。
- **HTTP 与控制台。** 新增 `POST /api/vehicle {"vehicle":"rover"|"drone"}`，
  202 返回 `{state, vehicle, view_mode}`；未知载具 400、运行/停止中 409、
  连接中 503、请求体错误 422。控制台点载具调用该端点，允许重复点当前
  载具（再次归位），失败时回滚选中载具与视角高亮。
- **出生点。** Neighborhood `Drone1` 从 `0 0 -40` 改为 `-5 0 -40`
  （Rover 保持 `0 0 -2`）；避开 M7 记录的 x≈+1.8 树篱/树冠与 x≈+6.4 树。
  CityEnviron 原本就是 Rover `50 0 -1` / Drone `55 0 -2`，未改。
- **测试。** 新增 `tests/test_scene_spawns.py`（两场景都存在双载具且水平
  距离 ≥ 3 m）；扩展 view/session/worker/controller/API/console 断言。
  全量 **692 passed, 1 warning**（Starlette httpx 弃用）。
- **实机验收：待做。** 需要重启规划服务加载新代码，并在 Neighborhood 上
  确认 `-5 0 -40` 的起飞、位置保持与降落，以及点击切换的停放/归位/前视
  闭环。按约定不自行重启用户正在运行的服务。

### M8 补充 2：非活动载具传送隐藏（2026-11-08）

用户要求场景里同一时刻只看到一台载具。ProjectAirSim 客户端没有运行时
删除/新建机器人的 API（机器人由场景 JSONC 声明），因此采用「传送走」：
非活动载具停放后瞬移到 300 m 外，画面里只剩当前载具；切换或任务路由到
它时再传回原位。

- **适配器。** `ProjectAirSimRover.teleport()` /
  `ProjectAirSimDrone.teleport()` 用 `SetPose(reset_kinematics=true)`
  瞬移并清空遥测/PID/碰撞缓存；`rover.return_home()` 改为复用
  `teleport()`，对外语义不变。
- **会话。** 新增 `_hidden_poses` 与 `_hide()` / `_unhide()`：会话启动时
  隐藏非默认载具；`activate_vehicle()` 在停放另一台后把它移出画面；
  `run()` 在任务路由到另一台时隐藏旧载具、恢复目标载具；
  `return_home()` 对被隐藏载具会再次隐藏（显式回起点不会让它重新出现在
  出生点）。隐藏失败在启动时只记 WARNING；在激活时使整次激活失败并回滚。
- **测试。** 新增适配器 `teleport` 用例与会话隐藏/恢复/启动告警用例；
  全量 **699 passed, 1 warning**（Starlette httpx 弃用）。
- **已知限制。** 「只剩一台」是画面效果：另一台仍在物理世界中，只是位于
  300 m 外；把视角切到被隐藏载具的前视/追尾相机会看到那处远处场景。
- **实机验收：待做。** 需要重启规划服务加载新代码；在 CityEnviron /
  Neighborhood 上确认点击切换后画面里只剩一台、再次切换能恢复原位。

### M8 补充 3：流式视角运行时自标定（2026-11-08）

用户实测反馈：控制台点“无人机”出现的是无人车视角，四个机位（车前/车后/
机前/机后）整体错位。定位到两条根因：

1. `planner/domain/view.py` 的 `VIEW_MODES`/`INITIAL_STREAM_INDEX` 是写死的
   静态顺序，但 ProjectAirSim 服务端的 `switch_streaming_view` 循环顺序和
   起始锚点**并不跨进程/场景加载稳定**。本次运行实测的锚点顺序是
   `rover_chase → rover_front → drone_front → drone_chase`，与旧的静态表
   不同；M8 早先的“标定”只记录了截图，并没有在运行时使用实测结果。
2. 客户端从不检查服务端当前机位；`set_view_mode` 只按静态索引算步进差，
   所以每一步都可能落到错误相机。控制器在 READY 时也假定服务端在
   `rover_front`，与服务端实际位置可能不一致。

修复：

- **新模块 `planner/adapters/projectairsim/streaming_views.py`。**
  `calibrate_streaming_views` 走一整圈：每个位置取 xpra 截图，与各相机
  `GetImages` 参考帧做 160×90 零均值灰度特征的相关性匹配。实测正确机位
  0.92 左右，其余 ≤0.28，阈值 0.55 + 最小间隔 0.10。无人车 Chase 相机
  `capture-enabled: false`、取不到帧，用“恰好一个位置匹配不上”的排除法
  确认。任何不确定（两个以上取帧失败、同一机位匹配两个位置、屏幕无法
  匹配、切换被拒）都返回 `None`，由调用方回退。函数结束时把服务端切回
  进入时的锚点位置。
- **取帧不能占用主连接。** `GetImages` 会阻塞到服务端在请求时间之后拍下
  新帧，而暂停世界必须由主连接 `ContinueForSimTime` 推进；同一个连接会
  死锁。取帧改用 `ProjectAirSimClient.connect_services()` 的服务专用客户端
  （不占 topic socket）在后台线程阻塞，主线程同时推进时钟；该客户端超时
  会自行断开，代码在下一次取帧时重连。
- **会话集成。** `ProjectAirSimMissionSession._calibrate_views()` 在场景
  加载、双载具 start、预热取帧之后、隐藏非活动载具之前执行。成功后
  `_view_positions` 记录实测顺序、`_view_index=0`，并立即
  `set_view_mode(front_view_for(active_vehicle))`；失败只记 WARNING，保持
  内置 `VIEW_MODES`。`set_view_mode` 改为用 `_view_positions.index` 计算
  目标位置。
- **控制器初始机位。** 初始 `_view_mode/_applied_view` 以及 READY 后的
  `_applied_view` 改为 `front_view_for(default_vehicle)`，与会话标定后
  立即切到的位置一致，不再假定固定索引。
- **顺带修复无人机回原点。** `ProjectAirSimDrone.start()` 在会话把非活动
  无人机传送隐藏**之前**记录 `_home_pose`；此前是懒记录，首次激活时会把
  300 m 外的隐藏位姿当“出生点”，且每次激活再漂移 -300 m。

实机验证（未重启用户正在运行的规划服务/仿真器）：

- 用生产函数直接连运行中的 CityEnviron（8991/8992、xpra :104），实测
  `RESULT ('rover_chase', 'rover_front', 'drone_front', 'drone_chase')`；
  各位置最佳匹配 0.918 / 0.929 / 0.934，次优 ≤0.244。截图证据在
  `output/calibration/live/`：`mode_rover_chase.png` 画面中能看到无人车
  本体，`mode_rover_front.png` 是贴近地面的车前视角。
- 首次标定约 13.8 s，其中约 8 s 是无人车 Chase 的取帧超时；每次场景
  加载只做一次。
- 单测：新增 `tests/test_streaming_views.py` 9 项（合成循环/任意锚点/
  排除法/全可取帧/两种失败/切换被拒/截图异常后归位）；会话补充实测顺序
  生效、标定成功存储、失败回退、无场景话题四项；无人机补充 start 记录
  home 并验证隐藏后回原点一项。全量 **713 passed, 1 warning**。
- **待办：** 控制台点击闭环需要重启规划服务加载新代码；按约定本次没有
  重启用户正在运行的服务。

### M8 补充 4：默认尾视角、前视相机抬高、郊区出生点改到路心（2026-11-09）

用户实测反馈四点：①初始化时应默认只有无人车（控制台默认选中无人车）；
②无人车前视画面在地下；③点无人机/无人车都应默认尾视角；④无人车初始位置
不在道路中央。逐一定位并修复：

1. **初始化画面错位（①③）。** 控制器 READY 时会把内存里的 `_view_mode`
   回放到新会话；该值可能是上一轮选的 `drone_chase`，于是 status 写着
   `vehicle=rover`、画面却是无人机机位。现在 READY 一律重置为默认载具 +
   默认机位：`_vehicle = default_vehicle`、`_view_mode/_applied_view =
   default_view_for(default_vehicle)`，不再回放旧机位；天气仍照旧重放。
   控制台在 `connecting → idle` 时清掉 `vehicleTouched`，等下一个状态轮询
   把载具按钮同步回服务端实际值（换图/重连都适用）。
2. **默认机位改为尾视角（③）。** `planner/domain/view.py` 新增
   `VEHICLE_DEFAULT_VIEW = {"rover": "rover_chase", "drone": "drone_chase"}`
   与 `default_view_for()`；`front_view_for()` 保留给控制台“车前/机前”
   显式按钮。会话 `activate_vehicle()` 与流式视角标定结束都改切默认尾视角；
   控制器初始值、激活乐观更新、ACTIVE 成功、`POST /api/vehicle` 响应
   （`{state, vehicle, view_mode: "*_chase"}`）全部改用
   `default_view_for()`；控制台初始 `taskView=rover_chase`，激活时乐观切
   到 `*_chase`，成功提示改为“画面切换到它的尾视角”。
3. **前视相机沉到路面下（②）。** `FrontCamera` 原挂 `1.8 0.0 0.2`
   （传感器原点 NED，z 向下为正，即车架原点下方 0.2 m）。用下视深度相机
   （挂在 `0 0 0`）实测车架离地：`(0,-16)` 约 1.17 m、`(20,-16)` 约
   0.38 m，因此原前视相机离路面只有 0–0.2 m，基本贴着/埋进路面或车底——
   这就是实机看到的“前视在地下”（下视是沥青、前视下半屏是草、上半屏发黑）。
   修复：用传感器运行时 `SetPose` 在 `(0,-16)` 实测候选安装点，选定
   `1.8 0.0 -0.8`（原点上方 0.8 m、pitch -3°）：前视深度中位数约 14 m、
   `<3 m` 像素 0%，画面是干净笔直的街道、车身不入镜。两张地图的
   `robot_rover.jsonc`、`config/base.yaml camera.offset_m` 与
   `planner/config.py` 的两个默认值同步改为该值；运行时
   `assert_camera_matches_scene` 会校验三者一致。
4. **郊区出生点不在路心（④）。** 原 `Rover1 (0, 0, -2)` 实际停在
   `(0,0)` 的混凝土人行道上、正对树篱。用只连服务的临时客户端
   （`connect_services()`，不占 planner 的 topic socket）在候选点
   `SetPose + GetImages` 实测：`y=-16` 一条街是沥青路；`x≈-30` 一带下视
   是草地（此前的“砖路”判断有误）。先选 `(20,-16)`，但该点上方有树冠、
   前视被近处树叶挡死（`<1.5 m` 像素 44–87%）。最终 `Rover1` 改为
   `0 -16 -2`、`rpy 0 0 0`（沥青路中心，朝 +x 沿街），下视为路面、
   前视为笔直街景；与 `Drone1 (-5, 0, -40)` 水平相距约 17 m。探针结束后
   恢复原位，未影响正在运行的规划服务。
5. **测试与文档。** 更新 view/config/session/controller/API 既有断言并新增
   `default_view_for` 用例；全量 **718 passed, 1 warning**。
   README/run.txt 的“切前视”“视角跨重连重放”“两车相距约 5 m”等描述同步
   改写。
- **实机验收：待做。** 场景 JSONC 与机器人 JSONC 都改了，需要重启规划服务
  并让仿真器重新加载场景（换图或整套重启）才能看到新出生点与抬高后的前视。
  按约定不自行重启用户正在运行的服务。验收点：启动后画面只有无人车且是
  尾视角；点无人机 → 无人机尾视角（rover 被传送走）；点回无人车 → 尾视角；
  切 Neighborhood 后无人车在沥青路中心、前视不再在地下。

### M8 补充 5：两车同点出生、城市路心出生点、标定重复匹配修复（2026-11-09）

用户实测反馈：初始应默认无人车 + 车尾视角；点无人机只显示无人机 + 机尾
视角；郊区要和 AirSim 原场景一样放在道路中央，无人机与无人车起始位置
相同但一次只出现一台；城市场景无人车必须初始化在道路正中央，而不是花坛。
同时要求不再把“全量测试通过”当验收，必须实际跑出画面来看。

1. **两车共用出生点。** Neighborhood：`Rover1 0 -16 -2` /
   `Drone1 0 -16 -40`（水平重合，无人机在正上方 38 m）；CityEnviron：
   `Rover1 -45 -22 -1` / `Drone1 -45 -22 -40`。此前 City 的
   `(50,0)/(55,0)` 实测落在花坛/不可行驶区域，用户反馈“初始化到花坛里”。
   新点用航拍 + 下视相机确认：西侧路口沥青带横跨 y≈-36..-8，路心
   y≈-22，无人机正下方就是人行横道。同一时刻只显示一台仍由启动时的
   “传送 300 m 外”实现，与同点出生不冲突。
2. **启动顺序调整。** `_hide_inactive_at_startup()` 提前到
   `_calibrate_views()` 之前：两车同点出生时，两个前视相机拍的是同一条街、
   画面几乎一样，先移走非活动载具再做相机匹配。`_calibrate_views()`
   无论成功、失败、无场景话题还是异常，最后都调用 `_apply_initial_view()`
   把画面切到活动载具的尾视角，避免标定循环停在任意机位上。
3. **标定重复匹配修复。** 旧逻辑逐位置挑最高分，Neighborhood 实测里
   无人车 Chase 所在位置的截图与 `rover_front` 相关 0.75，同时真正的
   `rover_front` 位置得分 0.998，于是被误判成“同一相机匹配两个位置”并
   放弃标定（画面停在错误机位）。`_solve_assignment` 改为按整圈做全局
   排列求解：遍历 4! 种“位置→机位”排列，取匹配分之和最大且每个可匹配
   位置都 ≥ 0.55 的解；不可取帧的 Chase 相机由排除法获得剩余位置，且该
   位置对其余相机的最大相关度必须 < 0.90（`MAX_UNMATCHED_SCORE`）。
   用三组实测分数矩阵（郊区同点、郊区旧布局、城市）复核，均解出
   `drone_front → drone_chase → rover_chase → rover_front`。
4. **测试（回归检查，不作为验收）。** `tests/test_scene_spawns.py` 改为
   断言两车水平重合（< 0.5 m）且无人机 z 在无人车上方；
   `tests/test_streaming_views.py` 新增真实分数矩阵的全局求解用例与
   0.90 排除阈值用例；会话标定失败用例改为断言最终停在 `rover_chase`。
   受影响 65 项测试通过。
5. **实机画面（xpra 截图，标定修复前的进程 + 新场景文件）。**
   - `/tmp/nb_new.png`：Neighborhood 启动只有无人车、尾视角、在沥青路
     中心 `(0, -16)`。
   - `/tmp/drone_chase_zoom.png`：点无人机后只剩无人机、尾视角，仍在该
     路心。
   - `/tmp/city_new.png`：CityEnviron 启动只有无人车、尾视角，在西侧路口
     沥青路中央 `(-45, -22)`。
   - `/tmp/city_drone.png`：点无人机后只剩无人机、尾视角，在同一路口
     人行横道处。
6. **待做。** 新的标定代码只在源码里，重启规划服务后才会生效；按约定
   不自行重启用户正在运行的服务。重启后需在 xpra 画面上复核：初始
   无人车 + 尾视角；点无人机 → 只剩无人机 + 尾视角；点回无人车 → 只剩
   无人车 + 尾视角；Neighborhood 无人车前视不再在地下。

### M8 补充 6：隐藏位置改为高空并钉住（2026-11-10）

1. **现象。** Neighborhood 上输入“沿道路行驶，不要发生碰撞”，无人车连续
   后退（3/6/9 m 的碰撞退避），随后报
   `collision recovery budget exhausted`，轨迹队列为空。xpra 截图显示车
   停在开阔沥青路上，前方没有可见障碍。
2. **服务端证据。** `projectairsim_server.log` 从启动隐藏无人机的时刻
   （08:00:33 UTC）起，持续 4 小时输出
   `Collision detected between 'Frame' and 'Outer_Wall_Quart_Win_10' at
   z=276.997437`，累计 93970 条；其余接触只在两台车出生时各出现一次。
   隐藏无人机的旧位姿是水平偏移 `(-300, -300)` m，正好落在那面墙里。
3. **原因。** 隐藏用的水平传送把无人机嵌进了地图几何体，物理求解器每帧都
   报接触；两台机器人的碰撞 link 都叫 `Frame`，服务端日志无法区分来源，
   而无人车的自由导航在任务开始后连续消费到碰撞并触发退避，直到 3 次
   预算用完。
4. **修复。**
   - `_hide()` 改为只抬高 `HIDE_ALTITUDE_GAIN_M = 1000 m`，不再横向移动，
     因此不会再嵌进任何建筑。
   - `ProjectAirSimConnection` 新增 `add_advance_hook()` /
     `remove_advance_hook()`：每次 `advance_for_sim_time()` 后触发钩子，
     异常只记日志、不打断推进。
   - 会话注册 `_pin_hidden()`，按 `HIDE_PIN_PERIOD_S = 2 s` 仿真时间节流
     重新 `SetPose` 到隐藏位姿。被隐藏的载具仍在受重力的世界里，两次钉住
     之间只下落约 20 m，离地 1 km 仍远高于两张地图的所有建筑。没有隐藏
     载具时钩子零开销。
   - `_unhide()`、`return_home()`、`close()` 同步清理钉住记录与钩子；
     钉住失败只在首次记录 WARNING，恢复后自动清除标记。
5. **测试。** 新增 `tests/test_projectairsim_connection.py`（3 项：钩子收到
   仿真时间、异常不打断推进、移除后不再执行）与会话隐藏/钉住/钉住失败
   告警用例；全量 **728 passed, 1 warning**。
6. **待做。** 新代码需要重启规划服务才会生效；按约定不自行重启用户正在
   运行的服务。重启后实机复核：启动即只有无人车、尾视角；输入自由导航
   指令不再连续后退；服务端日志不再出现成片的 `Outer_Wall_Quart_Win_10`
   接触；点无人机/无人车切换后仍只有一台可见，且无人机能正常降落。

### M8 补充 7：郊区路面与自身接触不再触发碰撞恢复（2026-11-10）

1. **现象。** Neighborhood 输入“沿着道路行驶，不要发生碰撞”，车仍连续
   倒车 3/6/9 m，随后 `collision recovery budget exhausted`，轨迹队列为空。
   补充 6 修的是隐藏无人机嵌墙，这次的接触来源不同。
2. **证据。** 规划服务终端 21:38:00–21:38:17 连续锁存
   `object=Road_89`、`Road_88`，`penetration=0.000`，接触位置依次为
   `(-1.15, …)`、`(-4.12, …)`、`(-10.06, …)`、`(-19.01, …)`，正好跟随
   车辆前进/倒车路径；服务端同期没有任何建筑、树木或墙体的接触。
3. **原因。** `CollisionMonitor` 只把 `Landscape` 前缀当作地面。城市图的
   路面属于地形，郊区图的路面是独立的 `Road_82/84/88/89` 模型，不在忽略
   名单里，于是每次车轮压上路面都被当成碰撞并触发倒车恢复。出生与传送
   还会产生 `Rover1`/`Drone1` 自身接触，同样不是障碍物。
4. **修复。** 默认忽略前缀扩展为 `("Landscape", "Road_")`；新增
   `ignored_names` 精确名字集合，无人车/无人机适配器传入各自 `robot.name`，
   只忽略自身、不影响 `Rover10` 这类其他对象。
5. **测试。** `tests/test_projectairsim_streaming.py` 新增郊区路面忽略与
   自身精确匹配两项；碰撞、无人车、无人机三组 61 项通过。
6. **实机复核（2026-11-10 21:57）。** 重启后同一句“沿着道路行驶，不要发生
   碰撞”不再锁存任何 `Road_*` 接触，车行驶 171.8 m、13 次规划后撞上真实
   的 `Fence_250`（截图 `/tmp/now_rover.png` 车头正对白色栅栏），3/6/9 m
   退避后每次重规划仍朝同一栅栏前进，最终预算耗尽。说明修复生效，剩下的是
   真实碰撞。

### M8 补充 8：道路跟随缺少车道参考，暂不修改（2026-11-10）

1. **提示词对比。** `planner/adapters/trajectory_planner/prompts/` 三个文件
   与 carla-vln 逐字节相同；任务解析提示词的地面车规则也相同，只额外多了
   无人机措辞。
2. **真正差异。** carla-vln 每次规划附带 CARLA HD 地图测得的车道参考
   （车道中心偏移、宽度、左右边界距离、5/10/20/30/40 m 前视点、航向误差，
   见 `carla-vln/planner/adapters/carla/lane_reference.py`）。
   `projectairsim_session.py` 的 `lane_reference_provider=None`，提示词因此
   回退为“看 RGB 和深度自行判断”。ProjectAirSim 没有暴露车道地图接口。
3. **后果。** 视觉方案在直路上能沿路，但路口、尽头、栅栏前缺少硬约束，会
   把车带出车道并反复撞同一障碍。
4. **决定（用户 2026-11-10）。** 暂不实现道路中心线参考，保持现状。若以后
   要做，方案是逐图实车采集道路中心线，实现与 carla-vln 同接口的
   `LaneReferenceProvider`；提示词与工作流无需改动。

### M8 补充 9：无人机轨迹提示词对齐参考 Planner（2026-11-10）

1. **问题。** 无人车逻辑与 carla-vln 一致，无人机适配器是 Planner 的移植，
   但两条轨迹提示词仍共用 carla-vln 的“地面车、dz 恒为 0”版本，无人机
   无法规划升降；这与本项目既定意图（交通载具改地面措辞、无人机保留飞行
   措辞）不一致。
2. **移植内容（用户 2026-11-10 选择“对齐 Planner”）。**
   `prompts/base.py` 增加 `AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT`（逐字取自
   `/roxy/zhusai/Planner`，UAV 措辞、NED 坐标、允许垂直运动）；
   `visual_goal_navigation.py` 增加 `AERIAL_VISUAL_GOAL_SYSTEM_PROMPT` 与带
   `dz` 非零示例的 `AERIAL_VISUAL_GOAL_OUTPUT_GUIDANCE`；
   `free_navigation.py` 增加 `AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT`（Planner
   无对应实现，故沿用同一条道路跟随流程，只把措辞改为飞行）。
3. **按载具选择。** `VisualGoalTrajectoryPrompt(aerial=...)`、
   `FreeNavigationTrajectoryPrompt(aerial=...)` 由构造参数决定提示词；
   `build_planner(..., aerial=...)` 与
   `build_free_navigation_planner(..., aerial=...)` 透传；
   `projectairsim_session._build_arms` 为 rover 用原 carla-vln 提示词，为
   drone 单独构建 aerial 规划器。无人车提示词逐字未改。
4. **测试。** 新增无人机基础/视觉目标/道路跟随提示词断言与工厂透传断言；
   全套 735 项通过。
5. **生效条件。** 需要重启规划服务加载新代码（按约定不自行重启）。

### M8 补充 10：仿真器切换加速（2026-11-11）

1. **实测耗时（原实现，点击换图 → idle）。** 约 27 s：停 worker 与仿真器约
   3 s；拉起 UE 约 6 s；连接并加载场景约 7 s（其中固定
   `delay_after_load_s` 占 5 s）；机位标定约 13 s（其中无人车 Chase 的
   `GetImages` 取帧超时占 8 s）。
2. **A 收紧固定等待（部分回退）。** 启动脚本拉起 UE 后的静置 2 s → 0.5 s
   保留。`delay_after_load_s` 曾从 5.0 降到 2.0，但实机日志显示这会让机位
   标定整体失败：场景加载期间仿真时钟在跑，无人机从 38 m 高空下落约 2.8 s
   才落地；2 s 时它还在半空，标定抓的无人机参考帧与随后截图对不上，四个
   机位只能回退到内置顺序（用户看到“车前/车尾/机前/机尾都不一致”）。现
   恢复为 5.0，等无人机落地后再标定。这一项不再省时间。（该等落地方案已在
   补充 11 中被“暂停时钟 + 悬停标定”取代，加载等待回到 2.0。）
3. **B 跳过不可取帧机位。** 新增 `projectairsim.uncapturable_views`（默认
   `[rover_chase]`，对应 robot JSONC 的 `capture-enabled: false`）。标定与
   锚点校验都不再对它调用 `GetImages`，改由全局排列 + 排除法定位。约省 8 s。
4. **C 标定结果按场景文件缓存。** `server_bootstrap` 维护进程内
   `view_cache`，会话按 `projectairsim.scene_file` 记录测得的循环顺序；同一
   场景再次加载只用一张 xpra 截图做锚点校验（`CACHE_ANCHOR_MIN_SCORE=0.90`、
   `CACHE_ANCHOR_MIN_MARGIN=0.15`——Chase 画面与相邻前视的相关度可达约
   0.75，阈值必须比完整标定更严），通过约 2 s，不通过再完整标定。首次加载
   仍走完整标定。
5. **D 换图只重启 UE。** `launch/stop_projectairsim_xpra.sh` 增加
   `--sim-only`：只停/启 UE 进程，保留 Xvfb 与 xpra，控制台 iframe 不断流；
   `SimulatorSupervisor.restart()` 先用 `_view_stack_alive()` 判断显示栈，
   在则 sim-only（日志 `restarting only the simulator`），否则回退整套重启。
6. **跳过降落等待。** 换图/换模型会销毁场景，`worker.shutdown(skip_landing=
   True)` → `session.close(skip_landing=True)`：无人机未落地时不再等
   `land_timeout_s`（最多可省约 30 s），只记
   `Skipping drone landing: the scene is being replaced.`。手动停止任务与回
   起点不受影响。
7. **预期效果。** 点击 → idle 由约 27 s 降到约 18–21 s；同一场景文件再次
   加载约 15–17 s；换图期间 Xvfb/xpra 保持，画面不断流。主要收益来自 B
   （省 8 s Chase 取帧超时）与 C（同场景省约 4 s）。
8. **测试。** 新增标定跳过/缓存锚点、supervisor sim-only 重启、worker
   跳过降落等用例；全套 **791 passed, 1 warning**；两个 shell 脚本
   `bash -n` 通过。
9. **生效条件。** 需要重启规划服务加载新代码（按约定不自行重启）。实机
   复核：换图日志出现 `restarting only the simulator` 且 iframe 不重载；
   首次标定约 5 s、同场景再次加载约 2 s；无人机在空中时换图不再停等降落。

### M8 补充 11：暂停时钟 + 悬停标定，四机位与加载提速（2026-11-11）

1. **问题根因。** 补充 10 把 `delay_after_load_s` 降回 5 s 只是用等待掩盖
   了下坠：两个双载具场景的时钟在加载时就已运行（`pause-on-start: false`），
   无人机出生在半空（城市 -1 m 之外还有高度差、郊区原为 38 m），加载等待
   期间它一直在落。标定若在它落地前抓帧，参考帧与随后截图对不上，四个
   机位就被错误标注——用户看到“车前/车尾/机前/机尾都不一致”。
2. **出生点改为完全同点 + 暂停时钟。** 四份双载具场景（city/neighborhood ×
   default/hexarotor）把 `Drone1` 的 origin 改为与 `Rover1` 完全相同的坐标
   （CityEnviron `-45 -22`，Neighborhood `0 -16`），并把
   `clock.pause-on-start` 改为 `true`。场景加载后仿真时钟不自行推进，
   服务请求（`GetGroundTruthKinematics` / `SetPose`）仍可用。
3. **标定前隐藏非活动载具。** 会话在 `_rover.start()` / `_drone.start()` 之后、
   任何时钟推进之前调用 `_hide_inactive_at_startup()`，把非活动载具沿垂直
   方向传送到地图上方 1000 m 并钉住。这样两台车的碰撞体不会重叠，四个流式
   相机看到的是互不相同的画面。此前一版曾改成“先在同点上方 15 m 悬停、
   标定完再隐藏”，城市地图的地面机位匹配从约 0.99 掉到 0.39/0.47，标定
   失败后回退内置顺序，默认画面落到隐藏无人机的追尾相机上——该顺序已回退，
   恢复“先隐藏再标定”的已验证行为。标定后不再移动载具；`_unhide` 记录的
   仍是出生点。
4. **加载等待回归 2 s。** `delay_after_load_s` 从 5.0 改为 2.0：现在它只用于
   等渲染器出首帧，不再需要等无人机落地。连接后还会以 8 s 上限抓一帧
   无人车预览，强制渲染器发布传感器画面。
5. **视角语义不变。** 四个机位、标定算法、缓存锚点校验、默认尾视角、
   `POST /api/vehicle` 的“停放另一台 → 归位 → 切尾视角”都与补充 10 一致；
   本次只修正出生/加载时序，让标定拿到的参考帧稳定可信。
6. **测试。** 场景测试断言两车坐标完全相同、`pause-on-start` 为 `true`；
   会话测试覆盖启动时从出生点隐藏非活动载具、隐藏位姿钉住及其失败只记一次、
   启动隐藏记录出生点而非隐藏位。全套 **794 passed, 1 warning**。
7. **生效条件。** 需要重启规划服务加载新代码（按约定不自行重启），并且
   场景文件已改，实机需重新加载地图后观察首次标定是否稳定给出四个正确机位。

### M8 补充 12：整圈证据判定、影子帧等待与连接占用释放（2026-11-11）

1. **两台无人机相机画面接近时的误判。** 补充 11 上线后，Neighborhood 实测
   位置 1 只比位置 0 高 0.086（低于 0.10 的逐位置差距阈值），标定被放弃并
   回退内置顺序，默认画面落到隐藏无人机的追尾相机。`_solve_assignment`
   增加整圈证据规则：逐位置差距不足时，只有“另一台相机被分到别的位置、且
   它在该位置得分更高”才接受这组排列（实测
   `0:drone_front -> 1:drone_chase -> 2:rover_chase -> 3:rover_front`）。
   同时增加整圈歧义保护：两组排列总分差小于 0.10 时判为
   `ambiguous cycle assignment` 并放弃，不猜顺序。
2. **xpra 影子帧滞后。** CityEnviron 冷加载后第一次完整标定里，位置 1 和
   位置 2 读到同一张旧画面（drone_chase 都是 0.984），位置 3 的
   rover_front 掉到 0.384，整轮仍然失败。新增 `_settled_signature()`：
   每次切机位后最多重试 5 次、间隔 0.5 s，连续两张截图相关度 ≥ 0.9995 且
   与上一机位画面不同才采样，否则用最后一帧。标定循环改为把上一机位的
   签名传入下一位置。
3. **会话关闭时释放连接占用。** 上一轮复现中发现 `close()` 读取
   `self._drone.landed` 抛 `Object closed` 时会跳过后面的
   `connection.close()`，进程级 `_ACTIVE_CONNECTION` 未释放，之后所有重连
   都报 `another ProjectAirSim connection is active in this process`，服务
   永久卡在 `connecting`。`close()` 改为整段 `try/finally`：落地状态读取
   失败只记 WARNING 并当作已落地继续收尾，臂/载具/连接的清理各自容错，
   连接释放放在 `finally` 里保证执行。
4. **测试。** 新增 `LaggingShadowServer`（切机位后第一次截图返回旧机位）与
   稳定帧用例、`PARKED_DRONE_SCORE_ROWS` 实测分数用例、两组同分排列判
   歧义用例、`UnreadableLandedDrone` 关闭释放连接用例。全套
   **798 passed, 1 warning**。
5. **实机验证（重启规划服务后，单进程）。**
   - CityEnviron 冷加载 → `idle`，日志
     `Streaming views calibrated: 0:drone_front -> 1:drone_chase ->
     2:rover_chase -> 3:rover_front`、`Streaming view: rover_chase.`；
     xpra 截图（`/tmp/verify_city2.png`）为无人车尾视、贴地停在路心。
   - 控制台换图到 Neighborhood → `idle`，同样给出四个正确机位与
     `rover_chase`；截图（`/tmp/verify_nbhd2.png`）为无人车尾视、地面视角。
   - 整轮日志无 `ambiguous`、无 `another ProjectAirSim connection is
     active`、无 `session close failed`。
6. **文档。** README 的视角标定/已知限制、`run.txt` 的视角段同步补充上述
   规则与关闭语义。

### M8 补充 13：无人车归位悬空与未定机位黑屏（2026-11-11）

1. **切到无人机再切回无人车后，车悬在半空。** 第一次修复（暖机首帧取
   `observation.pose` 作为归位目标）没有生效：实测日志
   `Rover home pose measured: ... z=-1.99` 与作者坐标 `-2.0` 几乎相同，
   切回后车仍在空中。用独立探针（规划服务停掉后直接连仿真器）逐帧记录
   `GetGroundTruthKinematics` / `GetObjectPoses` / 相机消息三者，找到真实
   原因：作者出生点 `z=-2.0` 比郊区路面高约 1 米，快速物理车在时钟推进后
   **落到 `z=-1.001`**（0.5 s 到 -0.91，约 1.0 s 后稳定在 -1.001，直到
   3 s 不变）；而暖机 `capture()` 只推进了 0.05 s，读到的还是下落前的
   `-1.99`。修复：暖机取帧后 `ProjectAirSimRover.settle()` 按住刹车推进
   `SETTLE_DURATION_S = 2.0` 秒仿真时间，再取一帧实测沉降后的姿态调用
   `set_home_pose()`。涉及
   `planner/adapters/projectairsim/rover.py`（`settle` / `SETTLE_DURATION_S`）、
   `planner/adapters/runtime/projectairsim_session.py`（暖机帧 → 沉降帧 →
   归位目标、日志）。探针还确认：`SetPose` 后渲染 actor 会在下一次时钟
   推进时同步，`SetPose + SetObjectPose` 到与当前几乎相同的位姿时服务端
   会报 `Unable to move object ... not movable`（保持目标位姿不同即可，
   切载具/隐藏场景不受影响）。
2. **未确定机位时画面来回切换。** 标定、切换载具、切换机位都靠
   `World.switch_streaming_view()` 逐个循环四个相机，控制台嵌入的 xpra
   会把中间机位全部显示出来。新增后端状态位与事件：
   `WorkerEventKind.VIEW`（worker 执行完机位切换后回传成功/失败）、
   `ServerStatus.view_pending`（控制器在 activate / idle 下的 request_view /
   任务结束补切机位期间置位，收到 `ACTIVE` 或 `VIEW` 事件清除）。控制台
   `#view-shield` 用不透明纯黑遮罩盖住 iframe：状态为 `connecting`、
   `unknown` 或 `view_pending=true` 时显示，机位确认后隐藏；遮罩上不显示
   任何文字或状态说明（2026-11-11 按要求去掉标题与提示行）。
3. **测试。** `set_home_pose` 改变归位目标；`settle` 刹车并推进
   `SETTLE_DURATION_S`、未 `start()` 时拒绝；worker 的 VIEW 成功/失败事件；
   控制器 `view_pending` 的置位与清除（activate、立即切机位、延迟切机位、
   失败回滚）；`/api/status` 的 `view_pending`；控制台静态文件包含
   `view-shield` / `renderViewShield`；`teleport` 目标与当前位姿一致时
   不发送 `SetObjectPose`（无人车与无人机各一例）。全套 **811 passed,
   1 warning**。
4. **实机验证（2026-11-11 17:25，重启规划服务 + Neighborhood 冷启动）。**
   日志 `Rover home pose measured: ... z=-1.00`（修复前为 `-1.99`）、
   标定顺序正确、默认 `rover_chase`。点无人机 → `Rover moved out of view`、
   `Vehicle activated: drone`；再点无人车 → `Rover returned to start
   position: ... z=-1.00`、`Vehicle activated: rover`。xpra 截图
   `/tmp/v5_rover_back.png` 与初始 `/tmp/v5_start.png` 高度一致，车轮贴地，
   不再出现修复前的悬空阴影（对照 `/tmp/now_rover_back.png`）。全程无
   `view change failed`、无 WARNING。
5. **切到被隐藏的载具时归位失败（验证中发现的连带问题）。** 激活载具时
   目标通常还停在地图上方 1000 m（`_hidden_teleports` 记录），而
   `_pin_hidden` 会在每次时钟推进后把它钉回去；原来的 `activate_vehicle`
   直接对隐藏目标调用 `return_home`，于是 CityEnviron 日志出现
   `Drone home attempt 1 failed: drone home pose did not converge:
   distance=999.99m`，无人机要用满 5 秒在 ~1000 m 高度“降落”。修复：归位前
   先 `self._unhide(vehicle_id)` 恢复停放前的位姿并清掉隐藏记录，再
   `return_home` / 切前视。新增回归测试
   `test_activate_vehicle_restores_a_hidden_target_before_homing_it`
   （断言先 `teleport` 回停放位姿、再 `return_home`，且隐藏记录已清空）。
6. **切回无人车时激活失败（`SetObjectPose` 拒绝对同一位置传送）。** 第 5 项
   的修复让激活时先把目标 `_unhide` 回停放点，紧接着 `return_home` 又传送到
   同一个出生点——ProjectAirSim 对“原地传送”报
   `SetObjectPose failed. Unable to move object Rover1, check if object state
   is movable!`，激活整体失败并回滚。修复：
   `ProjectAirSimRover.teleport` 与 `ProjectAirSimDrone._set_pose` 在调用
   `SetPose` 前先读当前位姿，目标与当前水平/垂直距离 ≤
   `TELEPORT_TOLERANCE_M = 0.01` m、偏航差 ≤ `TELEPORT_TOLERANCE_DEG = 0.5`
   度时只发 `SetPose`、跳过 `SetObjectPose`（快速物理状态是权威的，渲染
   actor 会在下一次时钟推进时同步）。真实移动仍照常发送 actor 传送，所以
   隐藏/恢复不受影响。新增
   `test_teleport_to_the_current_pose_skips_the_actor_move`（无人车、无人机
   各一条）。全套 **811 passed, 1 warning**。

### M8 补充 14：两车不再共用轨迹用户消息（2026-11-11）

1. **问题。** 补充 9 只分了系统提示词；两条轨迹任务的**用户消息**仍由同一个
   `build_base_user_prompt` 生成（carla-vln 地面车措辞：`Current vehicle
   pose`、`Down:`、`Guidance reference vehicle pose`）。参考 Planner 的用户
   消息用的是 UAV 措辞，无人机因此没有拿到与 `/roxy/zhusai/Planner` 完全
   一致的消息。
2. **修改。** `prompts/base.py` 新增 `build_aerial_base_user_prompt`，正文
   逐字取自参考 Planner 的 `build_base_user_prompt`，只把参考项目的
   `observation.vehicle_pose` 换成本项目的同义字段 `observation.pose`
   （用 AST 提取函数体逐行比对，结果 `IDENTICAL`）。
   `VisualGoalTrajectoryPrompt` 与 `FreeNavigationTrajectoryPrompt` 按
   `self.aerial` 选择用户消息构造器；无人车分支继续调用原 carla-vln 的
   `build_base_user_prompt`，逐字未改。`prompts/__init__.py` 导出新构造器。
3. **测试。** 新增视觉目标用例断言：地面消息含 `Current vehicle pose:` /
   `Down:` / `Guidance reference vehicle pose`，无人机消息含
   `Current UAV pose:` / `NED down:` / `Guidance reference UAV pose`，且
   两套措辞互不混入；道路跟随的无人机用例同样断言用户消息为 UAV 措辞。
   全套 **812 passed, 1 warning**。
4. **生效条件。** 需要重启规划服务加载新代码（按约定不自行重启）。

### M8 补充 15：目标侧向与偏航方向约束（2026-11-11）

1. **问题。** 基础系统提示词第 15 条只定义符号（正值右转、负值左转），
   用户消息里只写了目标方位角的符号，没有规定“目标在哪一侧就不能往哪边
   转”。2026-11-11 的三次“行驶到红色汽车旁”里，目标在正前方偏右
   +4.45°，模型却连续给出 `dyaw_deg = -1`（左转）的航点；提示词里没有
   一条规则能判它错。
2. **修改。** 视觉目标提示词增加目标侧向约束（车、机两版一致）：
   - 系统提示词（`VISUAL_GOAL_SYSTEM_PROMPT` /
     `AERIAL_VISUAL_GOAL_SYSTEM_PROMPT`）：目标在左侧时禁止输出正
     `dyaw_deg`，在右侧时禁止输出负 `dyaw_deg`；`dyaw_deg = 0` 和朝目标
     方向的转弯仍然允许。
   - 每轮的“新轨迹规划参考”（`_new_trajectory_reference`）：按当前
     参考向量（有待执行时用 pending 尾部）判断左右，动态写入一句明确的
     禁止项；正前方/正后方不设侧向限制，水平方向未定义时同样不限制。
   这是对两个参考项目的有意偏离（参考项目没有这条规则），代码注释已注明。
3. **测试。** 新增 4 个用例：目标在右 → 禁负 dyaw、目标在左 → 禁正 dyaw、
   正前方无侧向限制、两套系统提示词都含静态规则。全套
   **816 passed, 1 warning**。
4. **边界。** 约束只作用于提示词（模型仍可能在极端情况下违反），没有加
   代码侧的确定性校验；目标方位角接近 0 时规则按严格符号生效（0.1° 偏左
   也会禁右转），如需死区需另行确认。
5. **生效条件。** 需要重启规划服务加载新代码（按约定不自行重启）。

## 6. 风险与说明

- 无 Unreal 编辑器时无法新增地图；可用地图限于官方预编译环境。自定义 GIS
  城市需外部 3D Tiles/GLB 或自行编译 UE5.2 工程。
- ProjectAirSim 话题/服务协议与 AirSim RPC 不同，Planner 的无人机代码需要
  接口级移植，测试覆盖要更细。
- UE5 窗口化渲染在 Xvfb 下依赖 GLX；已在本机验证可运行，但 GPU 内存有限，
  同时运行 CityEnviron + Neighborhood + Blocks 时需按需启停。
- 出生点、相机安装参数、道路跟随质量需在 M1–M3 中逐图校准。
- 配置文件中可能包含明文 API key，不得写入日志或提交。