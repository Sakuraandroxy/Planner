# XV3DGS-AirSim VLN

在 **ProjectAirSim v1.0.1（UE5）** 里同时运行 **四旋翼无人机 + 地面无人车**，
用自然语言指令完成视觉语言导航（VLN）闭环：浏览器控制台提交任务 → 后端
VLM 规划 → 仿真器执行 → xpra 交互画面实时显示。

本项目从两个只读参考项目移植：

- `/roxy/sakura/carla-vln`：无人车 VLN 服务、控制台、xpra 闭环、滑窗执行栈。
- `/roxy/zhusai/Planner`：无人机 AirSim 适配器、工作流、任务系统。
- `/roxy/kedior/airsim-flight-console`：控制台交互参考。

所有新代码在 `/roxy/sakura/XV3DGS-airsim-vln`。3DGS 重建/渲染暂不接入，
仿真器画面直接来自 ProjectAirSim。

## 目录

```
XV3DGS-airsim-vln/
├── config/
│   ├── base.yaml                 # 服务、模型端点、运动与滑窗参数
│   ├── maps.yaml                 # 可切换地图目录（id/标签/场景/配置目录）
│   ├── scenes/city/              # CityEnviron 场景与机器人 JSONC
│   ├── scenes/neighborhood/      # Neighborhood 场景与机器人 JSONC
│   └── tasks/visual_goal_navigation.yaml
├── console/                      # 控制台（双载具、剩余轨迹、xpra 画面）
├── planner/
│   ├── adapters/
│   │   ├── projectairsim/        # 连接、异步桥、坐标、观测、相机、碰撞、天气/视角
│   │   ├── http/                 # FastAPI 路由、MJPEG 帧流
│   │   ├── runtime/              # 任务线程、双载具会话与仿真器进程托管
│   │   └── ...                   # 任务解析 / 目标检测 / 轨迹规划端点
│   ├── application/              # 任务解析、阶段分发、完成校验
│   ├── capabilities/             # 滑窗规划、目标检测、完成判定
│   ├── domain/ ports/ services/ workflows/
│   └── server_bootstrap.py
├── scripts/
│   ├── launch_projectairsim_xpra.sh   # Xvfb + UE + xpra shadow
│   ├── stop_projectairsim_xpra.sh
│   ├── serve_planner.py               # 规划服务 + 控制台
│   ├── calibrate_streaming_views.py   # 4 机位循环顺序实机标定
│   └── smoke_*.py                     # M1–M4 实机验收脚本
├── sim/                          # CityEnviron / Neighborhood / Blocks 预编译环境
├── tests/
└── docs/PLAN.md                  # 实施方案与 M1–M7 记录
```

## 环境

```bash
conda activate carla-vln        # 环境名沿用参考项目，Python >= 3.10
```

Python 依赖见 `pyproject.toml`。`projectairsim` 客户端来自 ProjectAirSim
v1.0.1 的 wheel（不在 PyPI），已安装在 `carla-vln` 环境中。

仿真器二进制（均可执行）：

| 地图 | 二进制 |
|---|---|
| CityEnviron | `sim/CityEnviron/Binaries/Linux/CityEnviron-Linux-Shipping` |
| Neighborhood | `sim/Neighborhood/AirSimNH/Binaries/Linux/AirSimNH-Linux-Shipping` |
| Blocks | `sim/Blocks-Linux-1.0.1/Blocks/Binaries/Linux/Blocks-Linux-Shipping` |

## 快速开始

启动后端与控制台只需要一条命令；**服务启动时不会连接仿真器**，也不加载任何
场景（状态 `disconnected`，画面空白）。如果上一轮由启动脚本拉起的仿真栈还在
跑，服务启动时会先把它停掉（只停 pidfile 与命令行都校验属于本项目的进程；
手工启动的外部进程不会被误杀）。在控制台「世界设置」里选择地图，点「启动
场景」后，后台才会拉起仿真栈并连接。

```bash
conda activate carla-vln
python scripts/serve_planner.py --host 0.0.0.0
# 浏览器打开 http://<本机IP>:8000/
```

- 初始状态是 `disconnected`（未连接场景），任务、载具、天气、模型控件都不可用。
- 选择地图 →「启动场景」→ 状态变 `connecting`（后台停/启仿真栈，显示号与
  端口保持不变）→ 就绪后自动变 `idle`，交互画面自动重试。
- 如果此时有一台**非脚本托管**的仿真器在监听端口：选同一张地图会直接连接，
  不重启它；选另一张地图会拒绝（不能替你杀掉外部进程）。
- 脚本托管的仿真栈正常都已被启动清理停掉，所以「启动场景」通常是全新拉起；
  规划服务运行中换图只重启 UE 进程（Xvfb/xpra 保留、画面不断流；两者不在
  时才整套重启）。一个 UE 进程只能加载一张 cooked 地图。
- 启动失败会回到 `disconnected`，`/api/status` 的 `sim_error` 写明原因；
  操作员修好仿真器后再点一次「启动场景」即可重试。

**（可选）手动启动仿真器 + xpra 交互画面**

```bash
scripts/launch_projectairsim_xpra.sh cityenviron :104 10010 8991 8992
```

参数：`[MAP] [DISPLAY] [XPRA_PORT] [TOPICS_PORT] [SERVICES_PORT]`。

脚本在空闲显示号上依次启动 `Xvfb` → 窗口化 UE → `xpra shadow`；如果该
显示号已有同名仿真窗口（例如手动启动的 CityEnviron），则只启动 xpra 并
复用现有窗口。pidfile 与服务日志在 `output/xpra/`、`output/logs/`。
交互画面：`http://<host>:10010/`。

这适合只想直接看 xpra 画面、或调试启动脚本的场合。注意：这样拉起的仿真栈
同样带 pidfile，**之后启动规划服务时会把它当作上一轮残留停掉**。想在控制台里
用这张图，请先启动规划服务，再在「世界设置」里点「启动场景」。

控制台默认嵌入 `http://<host>:10010/` 的 xpra 画面；换端口时用
`http://127.0.0.1:8000/?xpra=<host>:<port>` 覆盖。点击画面后可操作
CityEnviron 场景。控制台右侧“世界设置”可切地图、切载具外观模型、调天气、
切 4 个机位，“回起点”对当前选中载具生效。点“载具”里的无人机/无人车 =
**切换即归位**：停放另一台（无人机落地确认、无人车刹车）、把选中载具传回出生点、
抬升到地图上方 1000 m 并保持高度（画面里只剩一台），并把画面切到选中载具的
**尾视角**（追尾相机）。规划服务启动后默认就是无人车 + 尾视角，无人机已在画面外。

**（可选）实机冒烟**

```bash
python scripts/smoke_projectairsim.py            # M1 双载具场景与取图
python scripts/smoke_projectairsim_adapter.py    # M2 适配器
python scripts/smoke_projectairsim_rover.py      # M3 无人车工作流
python scripts/smoke_projectairsim_drone.py      # M4 无人机工作流
```

冒烟脚本会自己建连、加载场景并在结束时恢复/关闭，不需要规划服务，
但**同一仿真器实例同一时间只允许一个客户端占用**（topic socket 是
单消费者链路）。规划服务只在控制台点「启动场景」时才拉起仿真器，所以
先停掉规划服务（或让它保持在 `disconnected`），冒烟结束后再在控制台启动场景。

关闭：

```bash
scripts/stop_projectairsim_xpra.sh :104            # 停 shadow + 本脚本启动的 UE/Xvfb
scripts/stop_projectairsim_xpra.sh :104 --view-only  # 只停交互画面
```

脚本只会停止自己 pidfile 记录的进程；手动启动的仿真不会被误杀。

## 地图与配置

可切换地图定义在 `config/maps.yaml`（`id / label / launch_map / scene_file /
sim_config_path`），当前开放两张：

- **cityenviron（默认）**：城市道路场景，`scene_city_dual.jsonc`。
- **neighborhood**：郊区场景，`scene_neighborhood_dual.jsonc`。
- **Blocks**：只用于启动链路与回归冒烟，没有对应的双载具任务场景配置，
  不在目录中。

ProjectAirSim 一个进程只加载一张 cooked UE 地图，**不能运行中换图**。
因此控制台/API 的换图 = 后台重启仿真：停 worker（topic socket 是单消费者
链路；场景马上被销毁，无人机不再等落地）→ 停止自己启动的 UE 进程 → 用
`scripts/launch_projectairsim_xpra.sh` 在同一显示上拉起目标地图 → 重连并
重建双载具会话。Xvfb 与 xpra 通常保持
运行（脚本带 `--sim-only`），所以控制台画面不断流、iframe 无需重载；只有
Xvfb 或 xpra 已不在运行时才整套重启。显示号、端口与 topic/service 端口在
切换前后保持不变（默认 `:104 / 10010 / 8991 / 8992`），天气与视角状态会在
新会话 READY 后重放。

安全边界：后台只停止 `output/xpra/` 下 pidfile 记录且命令行属于本项目的
进程。若仿真器是手工启动的（没有 pidfile 或进程不属于本项目），后台会拒绝
换图并给出重新启动命令，**不会误杀外部进程**。

当前地图记录在 `output/xpra/current_map`（只在换图成功后写入）。规划服务
重启时读取该 marker；内容不在目录里才回退到 `maps.yaml` 的 `default_map`，
避免把城市场景加载进郊区仿真器。

### 载具外观模型

`config/base.yaml` 的 `vehicle_models` 选择仿真器里显示哪一款载具网格
（不是 VLM 服务端点）。可选值只来自引擎打包资源：

- **无人车**：`offroad`（越野车）。
- **无人机**：`quadrotor`（四旋翼，默认）或 `hexarotor`（六旋翼）。

控制台“世界设置 → 载具模型”有两个下拉框，点“应用载具模型”后后台只重启
worker 并在**同一个仿真器进程**里重新加载场景（不重启 UE / 不换 cooked 地图）：
两台载具回到出生点，已保存的天气会在新会话 READY 后重放。六旋翼使用
`scene_*_dual_hexarotor.jsonc` 场景变体与 `robot_drone_hexarotor.jsonc`
（官方 hexarotor 布局，`simple-flight-api` 六电机顺序）。切换期间状态为
`connecting`，任务与其它设置返回 503/409，完成后回到 idle。

模型选择不会跨规划服务重启保存：重启后按 `vehicle_models` 的 `default`
重新选择。切换失败（场景文件缺失或网格加载失败）会留在 connecting 并写日志，
修好后重启规划服务即可。

手动启动 Neighborhood（不经控制台）：
`scripts/launch_projectairsim_xpra.sh neighborhood :105 10011 8993 8994`，
同时让 `config/base.yaml` 的 `projectairsim.port_topics/port_services` 与
`scene_file/sim_config_path` 匹配，再重启规划服务。不要让两张 UE 地图
同时占用 GPU。

## 任务语义

控制台/API 只提交自然语言指令：

```json
{"instruction": "向前直行 5 米，然后左转 90 度，再开到前方的红色汽车", "vehicle": "rover"}
```

`task_parser` 模型把指令解析成 `mission_plan_v1` 的有序阶段，逐阶段执行：

- **basic_motion**：无目标的数值动作（直行/倒车 N 米、左/右转 N 度），
  构造同步五次多项式运动段执行；不启用碰撞门。`forward_m` 为负表示倒车，
  `yaw_right_deg` 为负表示左转。
- **visual_goal_navigation**：带静态目标描述（“开到前方的红色汽车”）。
  目标检测 → 5 航点轨迹规划 → 滑窗连续执行 → 本地到达条件 + 独立
  `task_completion` 模型语义确认，两者都通过才报告 reached。完成规则见
  `config/tasks/visual_goal_navigation.yaml`。
- **free_navigation**：没有目标、也没有数值动作的道路跟随
  （“沿着道路一直前进，不要碰到障碍物”）。**连续滑窗执行**：队列常驻
  5–10 个世界航点，后台续规划，20 Hz 连续跟随，不再每 5 个航点停车。
  任务**无总时长上限、永不自动完成**，只能由 `POST /api/stop` 或本地失败结束。
  CityEnviron 没有 CARLA 车道 API，该工作流按提示词的自由空间规则运行
  （`lane_reference_provider=None`）。原生碰撞传感器触发有界倒车
  （3/6/9 m）+ 清空队列 + 重新规划；累计前进 20 m 后清零恢复预算。
  `free_navigation` 必须是最后一个阶段。
- 严格解析：`task_parser` 未配置或端点不可用时服务仍可启动、画面仍可看，
  但所有任务都会被明确拒绝，没有确定性回退；解析失败整单拒绝，不会执行
  部分阶段。

把 `sliding_window.enabled` 设为 `false` 会回退为停车分批执行
（视觉目标与自由导航都适用），用于对比和排障。

双载具语义：一个会话同时持有 `Rover1` 与 `Drone1`。任务按 `vehicle`
路由；切换载具时先停放非活动载具——无人机降落并确认触地，无人车刹车——
再把非活动载具抬升到地图上方 1000 m 并在每次时钟推进后钉住高度，
画面里只保留当前载具。
无人机只在任务真正路由到它时才起飞并进入位置保持。命令队列空闲时，
`idle_tick` 以 20 Hz 推进暂停世界，xpra 场景持续渲染。

控制台点击载具按钮会调用 `POST /api/vehicle`，一次完成“停放另一台 →
把另一台移出画面 → 选中载具传回出生点 → 切到它的尾视角 → 成为活动载具”；
重复点击当前载具等于再次归位。停放失败（例如无人机落地未确认）时整次
激活失败，状态回滚到点击前的载具与视角。新会话就绪时一律回到默认载具 +
尾视角，不回放上一次选中的机位。

无人车采用方案 A：参考滑窗控制环输出的速度/偏航率指令，经纯追踪 +
速度 PID 转换为 `set_rover_controls(engine, steering_angle, brake)`，
物理与接触保持在环内。无人机使用 ProjectAirSim 的
`MoveByVelocity`/`MoveByVelocityAsync` 速度控制。

## 控制台运维控制

- **切换载具**：点“无人机/无人车”即切换并归位：停放另一台、把另一台抬升到
  地图上方 1000 m 并保持高度、把选中载具瞬时传回出生点、画面切到它的
  **尾视角**（追尾相机）。
  仅 idle 可用；重复点当前载具会再次归位。停放失败时状态回滚并显示错误。
- **启动/切换地图**：世界设置卡片选择地图并点「启动场景」（`disconnected`）
  或「应用世界设置」（`idle`）。提交后状态变为 `connecting`
  （`map_switching=true`）：仿真栈不存在时直接拉起，同图已运行时只连接，
  换图时只重启 UE 进程（Xvfb/xpra 保留；两者不在时才整套重启）；完成后回到
  idle、status 的 `map` 更新，控制台自动重置
  xpra iframe。运行/停止中提交返回 409，连接中返回 503，仿真器非脚本托管
  返回 400。
- **天气**：预设（晴朗/阴天/小雨/大雨/雪/雾/扬尘）+ 8 个 0–1 滑块
  （rain / road_wetness / snow / road_snow / maple_leaf / road_leaf / dust /
  fog），映射到 `WeatherParameter` 1–8。仅 idle 可提交；全 0 等价于关闭天气
  特效。换图后自动重放已保存的天气。
- **视角**：4 个机位——无人车前视（rover_front）、无人车追尾
  （rover_chase）、无人机前视（drone_front）、无人机追尾（drone_chase）。
  ProjectAirSim 没有自由观察者相机，只能通过 `World.switch_streaming_view()`
  在 robot JSONC 里 `streaming-enabled` 的相机之间循环，而且服务端既不暴露
  当前机位，也不保证循环顺序跨启动稳定。因此会话在加载场景后**运行时自标定**：
  先把非活动载具移出画面（两车同点出发，两个前视相机画面几乎一样，会干扰
  匹配），再走一整圈，把 xpra 截图与各相机 `GetImages` 帧做匹配（正确机位
  相关度约 0.92，其余约 0.28），得到本轮真实的“位置→机位”映射。匹配按整圈
  做全局排列求解，而不是逐位置挑最高分——无人车 Chase 无法取帧
  （`capture-enabled: false`），它在某个位置的画面可能与相邻前视高度相关
  （实测约 0.75），全局求解后由排除法确认，并要求该位置对其余相机的相关度
  低于 0.90，否则判为不可信。两个无人机相机画面接近时（实测位置 1 只比
  位置 0 高 0.086），用整圈证据判定：只有当另一台相机被分到别的位置、且在该
  位置得分更高时才接受这组排列；两组排列总分差小于 0.10 判为歧义并放弃。
  配置 `uncapturable_views` 会跳过这条取帧，不再等满 8 秒超时。每次
  `World.switch_streaming_view()` 后还要等 xpra 影子帧追上新机位（连续两次
  截图相关度 ≥ 0.9995、且与上一机位不同才采样），避免读到切换前的旧画面。
  标定成功后立即切到默认载具的尾视角；标定失败只记 WARNING、回退到内置顺序，
  同样切到默认尾视角。测得的结果按场景文件缓存：同一场景再次加载时只用一张
  截图做锚点校验（约 2 秒），校验不通过再完整标定。
  任务运行中切换会接受但延迟到任务结束后生效（响应 `deferred=true`）。
  标定、切换载具或切换机位期间，仿真器会逐个循环四个相机；控制台在这段时间
  用不透明黑色遮罩盖住画面（状态接口的 `view_pending=true`，或状态为
  `connecting`），机位确认后才显示画面，避免看到中间机位来回闪。
  逐机位截图证据可用 `scripts/calibrate_streaming_views.py` 复现。
- **回原点**：对当前选中载具（rover 或 drone）生效，仅 idle 可用。无人机
  与无人车语义一致——瞬时 SetPose 传送回出生点；无人机传送后以实测高度
  确认贴地并记录 `landed=True`，不依赖原生 `landed_state` 枚举。
  无人车的“出生点”是**沉降后实测的贴地姿态**：场景文件里的作者坐标比路面
  高约 1 米，车会在一秒内落到路面；会话在暖机取帧后按住刹车推进 2 秒仿真
  时间，再取一帧前视画面记录姿态（日志
  `Rover home pose measured: ... z=...`），归位时也打印 `z=`，便于核对是否
  贴地。

## HTTP API

服务默认 `127.0.0.1:8000`，完整 schema 见 `http://127.0.0.1:8000/docs`。

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/api/tasks` | `{"instruction": "...", "vehicle": "rover"\|"drone"}`；仅 idle 接受，返回 202 |
| GET | `/api/status` | 状态、当前指令、最近结果、当前/可用载具、`map/maps/view_mode/view_pending/weather/map_switching/sim_error/models/models_switching` |
| GET | `/api/logs?cursor=N` | 增量读取内存日志环形缓冲 |
| GET | `/api/trajectory` | 当前剩余世界航点、队列占用、里程、位姿 |
| POST | `/api/stop` | 请求停止；执行中返回 202，空闲返回 200 |
| POST | `/api/home` | `{"vehicle": "rover"\|"drone"}`（可省略，缺省当前载具）；仅 idle，202 |
| POST | `/api/vehicle` | `{"vehicle": "rover"\|"drone"}`；停放另一台并移出画面、选中载具回出生点、切到它的尾视角；仅 idle，202 `{state, vehicle, view_mode}`（`view_mode` 为 `*_chase`） |
| GET | `/api/world/options` | 地图目录、标签、默认地图、当前选择、`simulator_owned`；目录缺失 503 |
| POST | `/api/world` | `{"map": "..."}`；`disconnected` 时启动场景、`idle` 时换图；202 已接受（拉起/同图连接/换图重拉 UE）、200 无变化（仅 idle 同图）、409 运行/停止中、503 连接中、400 未知地图或仿真器非脚本托管 |
| GET | `/api/models` | 每台载具的当前模型、可选模型（id + 标签）与 `switching`；目录缺失 503 |
| POST | `/api/models` | `{"models": {"drone": "hexarotor"}}`（可只提交一台）；仅 idle，202 已接受重载、200 无变化、409 运行/停止中、503 连接中、400 未知载具/模型 |
| GET | `/api/weather` | 8 个字段定义与预设（每个预设都是完整 8 字段）；目录未就绪 503 |
| POST | `/api/weather` | 任意子集，值必须是 0–1 有限数；202 返回合并后的完整 8 字段 |
| POST | `/api/view` | `{"mode": "rover_front"\|"rover_chase"\|"drone_front"\|"drone_chase"}`；200 `{view_mode, deferred}`，未知机位 422 |

状态码：提交/回起点在连接中返回 503、运行/停止中返回 409；未知载具返回
400；请求体校验失败返回 422。`POST /api/stop` 在解析、采集、模型调用、
航点执行和自由导航控制环检查点均生效，日志出现 `Stop confirmed`，
`last_result` 为 `cancelled: ...`。

## 配置要点（`config/base.yaml`）

- `projectairsim.*`：地址、双端口、场景文件、配置目录、载具/相机名。
- `simulator`：启动/停止脚本、显示号、xpra 与 topic/service 端口、运行目录、
  地图 marker、启动/停止/端口释放超时与重启次数。topic/service 端口必须与
  `projectairsim.*` 一致。
- `config/maps.yaml`：可切换地图目录与默认地图；scene/配置路径必须存在。
- `vehicle_models`：每台载具的外观模型（`default` + `options`）。带
  `scene_files` 的模型必须为每张目录地图提供场景变体，文件必须存在于对应
  `sim_config_path`；同一地图不允许两个载具声明不同的场景文件。
- `camera` / `depth`：前视相机分辨率、视场角、安装偏移、深度语义。
- `motion` / `motion.continuous`：速度、加速度、偏航率上限与控制频率。
- `sliding_window`：队列容量、补点阈值、续规划周期、失败/时长上限。
- `free_navigation`：碰撞倒车距离、最大恢复次数、恢复预算清零里程。
- `visual_goal_navigation`：完成规则文件、确认次数、搜索与接近半径。
- `server`：监听地址、载具列表、默认载具、日志缓冲、CORS、视频帧率。
- `task_parser` / `target_detection` / `trajectory_planner` /
  `task_completion`：VLM/LLM 端点。`api_key` 直接写在配置里时优先于
  `api_key_env`；**填入真实密钥的配置文件不要提交到版本库**，密钥也不要
  写入日志。

## 测试

```bash
conda run -n carla-vln python -m pytest -q
# 812 passed
```

单元测试不启动仿真器（不 mock 物理公式，适配器的数学与协议转换用构造
数据验证）；实机验收通过 `scripts/smoke_*.py` 与
`scripts/calibrate_streaming_views.py` 在真实 ProjectAirSim 上完成，结果
记录在 `docs/PLAN.md` 的 M3–M8 实施记录中。

## 已知限制

- **换图有短暂停机**：ProjectAirSim 一个进程只加载一张 cooked UE 地图；控制台
  换图只重启 UE 进程（Xvfb/xpra 保留，画面不断流），通常约 18–20 秒（同一
  场景再次加载约 15–17 秒），视机器冷热缓存而定，期间服务为 `connecting`、
  任务返回 503。Xvfb/xpra 不在时才会整套重启。仿真器必须由启动脚本拉起，
  否则后台拒绝换图。
- **启动不连接仿真器**：规划服务启动时状态为 `disconnected`，没有场景也没有
  画面；启动时若发现上一轮由启动脚本拉起的仿真栈还在跑，会先停掉它（手工
  启动的外部进程不动，此时画面可能仍显示旧场景）。必须在控制台选择地图并
  点击「启动场景」（或 `POST /api/world`）才会拉起/连接仿真栈。在此之前任务
  与其它设置返回 503。启动失败会回到 `disconnected` 并记录 `sim_error`，
  可修正后再次启动。
- **视角不是自由观察者**：只能在 4 个 `streaming-enabled` 相机间循环；循环
  顺序由会话在加载场景后自标定（首次约 5 秒，无人车 Chase 不取帧），同一场景
  再次加载时用单帧锚点校验复用缓存（约 2 秒）。标定按整圈做全局排列求解，
  并复用“另一台相机在别处得分更高”的整圈证据，避免两台无人机相机画面接近时
  被误判；两组排列总分差小于 0.10 视为歧义，放弃本轮结果。每次切机位后等待
  xpra 影子帧稳定（连续两帧相关度 ≥ 0.9995 且不同于上一机位）再采样。
  标定失败时回退到 `planner/domain/view.py` 的内置顺序并只记 WARNING。
  标定、切载具、切机位期间控制台用不透明黑色遮罩盖住画面，机位确认后
  才显示，不会看到相机循环的中间帧。
  任务运行中的视角切换延迟到任务结束后生效。
- **天气跨重连保留，视角不保留**：天气保存在规划服务内存，换图/重连会重放；
  视角在每次新会话就绪时统一重置为默认载具的尾视角（不回放旧机位），重启
  `serve_planner.py` 后天气也清空。
- **一个仿真器实例只接一个客户端**：topic socket 是单消费者链路，规划服务
  与冒烟脚本不能同时连接同一实例。会话关闭时即使无人机落地状态查询失败
  （例如对象已关闭），也会释放进程内连接占用，不会让后续重连永久卡在
  “another ProjectAirSim connection is active”。
- **free_navigation 是无界任务**：没有 `max_task_duration_s` 上限，不会自动
  报告完成；只能停止或等待本地失败（碰撞恢复预算耗尽、航点超时等）。
- **视觉目标默认不启用碰撞门**（`collision_check_enabled: false`），与参考
  项目一致；自由导航始终启用原生碰撞恢复。
- **无人车是非完整约束载具**：纯 yaw、无平移的原地旋转不可执行；基础动作
  只支持直行、倒车和带平移的转向组合。
- **两车共用同一出生点**：两张地图里 `Rover1` 与 `Drone1` 的坐标完全相同
  （Neighborhood `(0, -16)`，CityEnviron `(-45, -22)`，都在沥青路中心），
  场景时钟在加载时暂停（`pause-on-start`），所以无人机不会在加载等待里下坠。
  会话在任何时钟推进之前先把非活动载具传送到地图上方 1000 m 并隐藏，再做
  机位标定，因此四个相机看到的是互不相同的画面。点载具切换时同样先停放
  另一台；同一时刻只看得到一台。无人车的出生点归位用的是暖机刹车沉降后
  实测的贴地姿态（`Rover home pose measured`），而不是场景文件里的作者
  坐标，后者比路面高约 1 米，归位会让车悬空。
- **“只剩一台”是画面效果**：ProjectAirSim 客户端没有运行时删除/新建机器人
  的 API，机器人由场景文件声明。切换时另一台只是被抬升到地图上方 1000 m
  并钉住高度，物理世界里仍然存在；把视角切到被隐藏载具的前视/追尾相机会
  看到高空俯视的场景，而不是它原来的位置。
- **无人机降落确认依赖落点净空**：Neighborhood 上若正下方是树篱/树冠，
  原生降落可能无法完成；会话切换载具时会记录 warning 并继续，下一次
  无人机任务会重新进入位置保持。
- **VLM/LLM 端点是硬依赖**：解析、目标检测、轨迹规划、完成判定都没有确定性
  回退；端点不可用时任务被明确拒绝。
- **xpra 画面无鉴权**：`10010` 端口只应暴露在可信局域网；控制台默认只嵌入
  该端口，其它端口用 `?xpra=host:port` 覆盖。
- **GPU/显示资源有限**：同时运行多张 UE5 地图会争用显存；每张地图用独立
  Xvfb 显示号，按需启停。
- **3DGS 未接入**：当前画面与感知来自 ProjectAirSim 原生相机；3DGS 重建/
  渲染留作后续工作。
- M8 已按 carla-vln 的运维语义补回地图/天气/视角/回原点端点与控件；差异是
  视角只能循环固定相机、换图依赖脚本托管的重拉 UE 进程（Xvfb/xpra 保留，
  详见 `docs/PLAN.md`）。

## 参考项目边界

`/roxy/sakura/carla-vln`、`/roxy/zhusai/Planner`、
`/roxy/kedior/airsim-flight-console` 是只读参考，本项目不修改它们。
实施方案、里程碑与各阶段实机验收记录见 `docs/PLAN.md`。