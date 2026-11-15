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
│   │   ├── http/                 # FastAPI 路由、MJPEG 帧流
│   │   └── ...                   # 任务解析 / 目标检测 / 轨迹规划端点
│   ├── simulator/                # 两族共用：进程驱动、argv 渲染、端口装配
│   ├── simulators/
│   │   ├── projectairsim/        # UE5：连接、异步桥、坐标、观测、相机、碰撞、天气/视角
│   │   └── airsim/               # UE4：AirSim RPC 适配层
│   ├── application/              # 任务解析、阶段分发、完成校验
│   ├── capabilities/             # 滑窗规划、目标检测、完成判定
│   ├── domain/ ports/ services/ workflows/
│   └── server_bootstrap.py
├── scripts/
│   ├── serve_planner.py               # 规划服务 + 控制台
│   ├── calibrate_streaming_views.py   # 手动测 4 机位循环起点（运行时不再调用）
│   ├── probe_ue4_env.py               # UE4 上机环境自检
│   └── smoke_*.py                     # M1–M4 实机验收脚本
├── tests/
└── docs/PLAN.md                  # 实施方案与 M1–M7 记录
```

## 环境

两个 conda 环境，各自对应一个仿真器客户端（完整记录在 `docs/ENVIRONMENT.md`）：

```bash
/home/kedior/miniconda3/envs/vln/bin/python        # 主服务、UE5 projectairsim 1.0.2、测试
/home/kedior/miniconda3/envs/vln-ue4/bin/python    # UE4 airsim 1.8.1，只跑任务子进程
```

两个客户端不能在同一个解释器里共存：`msgpack-python` 0.5.6 与 `msgpack` 1.2.4 抢同一个
模块名，混装后 `client.ping()` 直接报 `TypeError: __init__() got an unexpected keyword
argument 'encoding'`。因此 UE4 会话跑在 `vln-ue4` 拉起的子进程里，主进程不加载 AirSim
（`tests/test_architecture.py` 守这条）。`projectairsim` 不在 PyPI，靠 `--no-deps` 装。

仿真器不在仓库里。两个引擎都只用已经打包好的产物，不编译、不开编辑器；
每张图的路径写在 `config/maps.yaml` 的 `binary` 字段，那里就是唯一出处：

| 地图 | 引擎 | 二进制 |
|---|---|---|
| cityenviron | ProjectAirSim (UE5) | `/roxy/sakura/XV3DGS-airsim-vln/sim/CityEnviron/Binaries/Linux/CityEnviron-Linux-Shipping` |
| neighborhood | ProjectAirSim (UE5) | `/roxy/sakura/XV3DGS-airsim-vln/sim/Neighborhood/AirSimNH/Binaries/Linux/AirSimNH-Linux-Shipping` |
| blocks / campus / … | AirSim (UE4) | `/data/airsim/<图>/LinuxNoEditor/<图>/Binaries/Linux/<图>` |

## 快速开始

启动后端与控制台只需要一条命令；**服务启动时不会连接仿真器**，也不加载任何
场景（状态 `disconnected`，画面空白）。如果上一轮拉起的仿真栈还在跑，服务启动
时会先把它停掉（只停归属能对上本项目配置的进程；手工启动的外部进程不会被误杀）。
在控制台「世界设置」里选择地图，点「启动场景」后，后台才会拉起仿真栈并连接。

```bash
/home/kedior/miniconda3/envs/vln/bin/python scripts/serve_planner.py --host 0.0.0.0
# 浏览器打开 http://<本机IP>:8000/
```

- 初始状态是 `disconnected`（未连接场景），任务、载具、天气、模型控件都不可用。
- 选择地图 →「启动场景」→ 状态变 `connecting`（后台停/启仿真栈，显示号与
  端口保持不变）→ 就绪后自动变 `idle`，交互画面自动重试。
- 如果此时有一台**本驱动没拉起过**的仿真器在监听端口：选同一张地图且归属能
  对上时直接连，不重启它；对不上（手工启动、或跑的是另一张图）则拒绝换图并给出
  可直接执行的建议——后台不会替你杀掉不属于本项目的进程。
- 换图是停整套再起整套（xpra 会话、引擎、Xvfb 一起）：一个 UE 进程只能加载一张
  cooked 地图，而画面端口与会话绑在一起，所以浏览器里的画面会断开一次再自动重连。
  两个时间尺度分开看：仿真栈本身 UE5 `restart(cityenviron)` 7.8s、`restart(neighborhood)`
  7.0s，UE4 `restart(blocks)` 5–8s；而操作员在控制台上等到 `idle` 还要叠上场景与会话
  装配，实测 UE5 约 40s、UE4 约 8–16s。跨族切换（UE5↔UE4）真机验证见
  `docs/UE4_BRINGUP.md` 5.3。
- 启动失败会回到 `disconnected`，`/api/status` 的 `sim_error` 写明原因；
  操作员修好仿真器后再点一次「启动场景」即可重试。

**（可选）手动拉起仿真器 + xpra 交互画面**

启动与停止的命令在 `docs/UE4_BRINGUP.md` 第 3 节，两族同一条。启动前请先确认
规划服务没在跑：仿真器起来后它会把这个栈认作上一轮残留并在启动时停掉（前提是
归属对得上本项目的配置）。

想在控制台里用某张图，仍然是先启动规划服务、再在「世界设置」里点「启动场景」。

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

关闭：停掉整套仿真栈的命令同样在 `docs/UE4_BRINGUP.md` 第 3 节。后台只停
本项目拉起过的进程，归属靠引擎二进制真路径与 xpra 会话目录判定。

## 地图与配置

可切换地图定义在 `config/maps.yaml`，当前开放八张。每张图先声明它属于哪个引擎
（`simulator`），再带自己那组字段：

```yaml
- id: cityenviron
  simulator: projectairsim        # 驱动这张图的适配层包
  binary: /roxy/sakura/.../CityEnviron-Linux-Shipping   # 引擎二进制（也是归属判据）
  map_name: CityEnviron           # 交给二进制的 argv[1]，窗口标题与它同字
  scene_file: scene_city_dual.jsonc
  sim_config_path: config/scenes/city
```

两个引擎都是一个进程只加载一张 cooked UE 地图，**不能运行中换图**。因此
控制台/API 的换图 = 后台重启仿真：停 worker（topic socket 是单消费者链路；
场景马上被销毁，无人机不再等落地）→ 停掉本项目拉起的整套栈（xpra + 引擎 +
Xvfb）→ 按新图的 `binary`/`map_name` 重新拉起 → 重连并重建会话。显示号与
端口在切换前后保持不变（默认 `:104 / 10010`，UE5 另加 `8991 / 8992`），
天气与视角状态会在新会话 READY 后重放。

安全边界：后台只停止本项目拉起过的进程，判据是引擎二进制的真路径在
`config/maps.yaml` 里、且 xpra 会话目录在配置的 `runtime_dir` 下。若仿真器是
手工启动的，后台会拒绝换图并给出重新启动命令，**不会误杀外部进程**。

当前地图记录在 `output/xpra/current_map`（只在拉起成功后写入）。规划服务重启时
读取该 marker；内容不在目录里才回退到 `maps.yaml` 的 `default_map`，避免把
城市场景的配置加载进郊区仿真器。

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

手动拉起仿真器（不经控制台）的命令在 `docs/UE4_BRINGUP.md` 第 3 节，两族同一条：
驱动读 `config/simulators.yaml` 渲染 argv，没有 shell 脚本。想让两张图不挤同一张卡，
改 `config/base.yaml` 的 `simulator.display` 与家族的端口后重启服务。不要让两张 UE 地图
同时占用 GPU。

## 任务语义

控制台/API 只提交自然语言指令：

```json
{"instruction": "向前直行 5 米，然后左转 90 度，再开到前方的红色汽车", "vehicle": "rover"}
```

`task_parser` 模型把指令解析成 `mission_plan_v1` 的有序阶段，逐阶段执行：

- **basic_motion**：无目标的数值动作（直行/倒车 N 米、左/右转 N 度、上升/下降
  N 米），构造同步五次多项式运动段执行；不启用碰撞门。`forward_m` 为负表示倒车，
  `yaw_right_deg` 为负表示左转，`up_m` 为正表示上升（世界坐标 z 轴向下，转换器
  做 `z - up_m`）。
  规划按载具分开：无人机和无人车各有自己的 prompt 与解码器，但解码结果是同一个
  抽象对象。区别在**线上格式**：车的载荷里根本没有竖直字段，带 `up_m` 的包在解码
  阶段就被 `ProtocolError` 拒掉——不是在送仿真器那一层事后发现，也不是被翻译成
  一个前进量。合理性判断只发生在“模型输出变成我们的数据”这个边界上。
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

两族能跑的任务类型不一样，这是能力推导的结果而不是开关：

| 类型 | UE5 cityenviron / neighborhood | UE4 blocks / campus |
|---|---|---|
| `basic_motion` | ok 实测 39s 两阶段 | ok 实测 16s 单阶段 |
| `visual_goal_navigation` | ok （M3 起就跑这个） | ok 实测 60s、9 次 grounding、`basis=model_assessment_with_local_checks` |
| `free_navigation` | ok | no 旧分支没有，仍不提供 |
| `visual_goal_above` | no（缺 `camera_view`） | 被用户明确推迟 |

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
  当前机位，也不能按 id 选中某个相机。所以机位位置由程序记账：起点写在配置
  `projectairsim.view_cycle_start`（本机四次冷启动实测都停在 `drone_front`），
  之后每次切换在环上前进一格，当前格 = 起点 + 已切换次数。
  运行时**不再做机位标定**：整圈标定要截图、要另开一条连接取四张参考帧，每个
  机位还要等 2–4 秒画面才跟上，实测把 UE5 起图从 13.6 秒拖到 60–90 秒。需要
  重新确认起点时手动跑 `scripts/calibrate_streaming_views.py`，它仍然可用。
  起点写错时四个按钮会整体错同样多的格（不会随机错），日志
  `Streaming view: rover_chase (3 switch(es) from drone_front)` 里的起点就是
  定位依据，改配置那一行即可。每次切换后仍要等 xpra 影子帧追上新机位
  （实测 2–4 秒），等待期间继续推进时钟——场景时钟是 `steppable` 且
  `pause-on-start`，不推进就没有新渲染帧，画面会停在切换前。
  会话就绪后切到默认载具的尾视角；任务运行中切换会接受但延迟到任务结束后
  生效（响应 `deferred=true`）。切机位期间控制台用不透明黑色遮罩盖住画面
  （状态接口 `view_pending=true`，或状态为 `connecting`），机位到位后才显示，
  避免看到循环的中间帧。
- **回原点**：对当前选中载具（rover 或 drone）生效，仅 idle 可用。无人机
  与无人车语义一致——瞬时 SetPose 传送回出生点；无人机传送后以实测高度
  确认贴地并记录 `landed=True`，不依赖原生 `landed_state` 枚举。
  无人车的“出生点”是**沉降后实测的贴地姿态**：场景文件里的作者坐标比路面
  高约 1 米，车会在一秒内落到路面；会话在暖机取帧后按住刹车推进 2 秒仿真
  时间，再取一帧前视画面记录姿态（日志
  `Rover home pose measured: ... z=...`），归位时也打印 `z=`，便于核对是否
  贴地。

## 分层与依赖

规划器（`adapters/`）、仿真器（`simulators/` + `simulator/`）、服务层（`adapters/http`、
`application/server_controller`）是**并列的中间件**。它们不互相构造，只由
`application/mission_assembly.py` 接线。三条不变量：

1. **并列件不构造并列件。** 仿真器会话只交出端口（`vehicle`、`observation`、
   `camera_view`、`weather`），不再接收 `detector`/`planner`/`geometry`/遥测汇；
   它报 `mission_inputs()`，装配层还它 `attach_arms()`。
2. **只有装配层知道全部。** 少接一根线在构造期报错：`assemble_missions` 要求
   轨迹汇存在，`attach_arms` 要求每台载具都有 arm。以前是默认参数 `=None`，
   工作流遇到 `None` 就静默不发，于是一个通用面板在 UE4 上默默消失。
3. **能力门只描述引擎能不能做这个动作。** `models`、`views`、`weather`、
   `camera_view`、`home`、`video` 进门；遥测、日志、任务列表、指令卡不进。

进程边界与功能归属无关：UE4 的 AirSim 客户端不能和 HTTP 进程的依赖同解释器共存，
所以任务循环跑在子进程；子进程拿不到父进程的 `TrajectoryHub`，用
`simulator/telemetry_relay.py` 里的转发实现（同一接口、按 0.4s 合并），快照走已有的
工作者事件管道，父进程 `adopt` 进真汇。`/api/trajectory` 读的一直是同一个对象。

## HTTP API

服务默认 `127.0.0.1:8000`，完整 schema 见 `http://127.0.0.1:8000/docs`。

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/api/tasks` | `{"instruction": "...", "vehicle": "rover"\|"drone"}`；仅 idle 接受，返回 202 |
| GET | `/api/status` | 状态、当前指令、最近结果、当前/可用载具、`map/maps/view_mode/view_pending/weather/map_switching/sim_error/models/models_switching` |
| GET | `/api/logs?cursor=N` | 增量读取内存日志环形缓冲 |
| GET | `/api/trajectory` | 当前剩余世界航点、队列占用、里程、位姿。**不设能力门**：这些数据由共享工作流写进本服务的 hub，两族都有。根基础动作任务报“无剩余航点”，两族行为一致，不是缺功能 |
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
| GET | `/api/video` | MJPEG 流（`multipart/x-mixed-replace`）。服务启动时没给出帧枢纽则不注册该路径；
当前家族没有 `video` 端口时返回 **501**，详情里带 `unsupported` 与 `simulator`，不会返回一个空流 |
| GET | `/api/status` 中的 `capabilities` | 由已实现的端口推导，不是配置项；前端按它决定控件可用性。**判定原则**：端口只回答“这台引擎能不能做这个动作”。服务/规划层共有的东西（遥测、日志、任务列表、指令卡）不得进端口包，否则通用功能会变成按仿真器授权的开关 |

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
/home/kedior/miniconda3/envs/vln/bin/python -m pytest -q
# 926 passed
```

单元测试不启动仿真器（不 mock 物理公式，适配器的数学与协议转换用构造
数据验证）；实机验收通过 `scripts/smoke_*.py` 与
`scripts/calibrate_streaming_views.py` 在真实 ProjectAirSim 上完成，结果
记录在 `docs/PLAN.md` 的 M3–M8 实施记录中。

## 已知限制

- **换图有短暂停机**：两个引擎都是一个进程只加载一张 cooked UE 地图，不能运行中换图。
  控制台换图的真机耗时分两个尺度：仿真栈本身 UE4 5–8s、UE5 7–9s；等到控制台上一个可
  下发任务的就绪会话，UE5 约 40s（场景装配在主进程内），UE4 约 8–16s。期间服务为
  `connecting`、任务返回 503。Xvfb/xpra 不在时才会整套重启。
- **UE4 的会话在主进程外**：主进程不得加载 AirSim 客户端，所以 UE4 的任务跑在
  `vln-ue4` 拉起的子进程里，经 `config/simulators.yaml` 的 `runtime_python` 选定解释器。
  没写这个字段的家族（UE5）仍用线程 worker。
- **启动不连接仿真器**：规划服务启动时状态为 `disconnected`，没有场景也没有
  画面；启动时若发现上一轮由本项目驱动拉起的仿真栈还在跑，会先停掉它（手工
  启动的外部进程不动，此时画面可能仍显示旧场景）。必须在控制台选择地图并
  点击「启动场景」（或 `POST /api/world`）才会拉起/连接仿真栈。在此之前任务
  与其它设置返回 503。启动失败会回到 `disconnected` 并记录 `sim_error`，
  可修正后再次启动。
- **视角不是自由观察者**：只能在 4 个 `streaming-enabled` 相机间循环；服务端
  不报告当前在第几格，位置由 `projectairsim.view_cycle_start` 起点加切换次数
  记账，运行时不做标定（原因见「视角」一节）。四个机位里属于不在场载具的那
  两个，看到的是 1000 m 高空——不在场的车被抬出去了，这是设计不是故障。
  切载具、切机位期间控制台用不透明黑色遮罩盖住画面，机位到位后才显示，
  不会看到相机循环的中间帧。
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
  会话在任何时钟推进之前先把非活动载具传送到地图上方 1000 m 并隐藏，再按配置
  的起点把机位切到在场载具的尾视角，因此四个相机看到的是互不相同的画面。点载具切换时同样先停放
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