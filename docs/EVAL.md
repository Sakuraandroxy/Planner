# 评测

本文说明 `eval/` 子系统：它怎么把数据集里的一个 episode 变成一次真实飞行、复用规划器的哪条链路、结果落盘成什么、以及仪表盘怎么读这些结果。

评测**没有独立的规划器**。每个 episode 都在进程内构造与主服务相同的 `AirSimMissionSession`，复用同一份 `config/core/<profile>/base.yaml`、同一套工厂与 visual-goal 工作流。评测只负责三件主服务不管的事：拉起并布置 TravelUAV 场景、按频率采集无人机真实位姿、用真值距离与官方脚本打分。

## 1 一次批量评测的流程

```
数据集 episode                       eval/run.py
  scene / uuid / instruction          run_batch()
  start_pose / target / asset           └─ 按场景分组，open_scene() 拉起 AirSim
        │                               └─ run_episode() → run_episode_real()
        ▼                                         │
  evalcore/sim.py Scene                            │  每集：
  启动 TravelUAV 进程                              │   1. clear_objects → set_pose(起点) → spawn(目标物)
  注册 FrontCamera + DownCamera                    │   2. simPause(False) → enableApiControl → takeoff → hover
  绑定 rpc 端口                                     │   3. 起 5Hz 位姿采样线程写 log/NNNNNN.json
        │                                          │   4. session.run(TaskSubmission(instruction))
        ▼                                          │        → 任务解析 → mission plan → visual-goal 工作流
  evalcore/planner_runtime.py                      │   5. 末点距目标 ≤ success_radius 且无碰撞 → success
  _RealStack：仿真器端口 + 真实规划器栈             │   6. 按 success 与否改名 success_<uuid>/
        │                                          ▼
        └────────────────────────────── evalcore/metrics.py → vendor/metric.py
                                         SR / OSR / NE / SPL
```

要点：

- **场景进程与规划器进程内共存的一侧只有评测**。主服务把 UE4 会话放进子进程是为了隔离 `msgpack`；评测本身就跑在 `vln-ue4` 解释器里，所以 AirSim 客户端可以直接加载（[planner_runtime.py](../eval/evalcore/planner_runtime.py) 顶部有说明）。
- **瞬移后必须重新起飞**。`simSetKinematics` 会把飞控置于失控状态，恢复仿真后无人机会自由落体撞地。`run_episode_real` 在 `simPause(False)` 之后显式执行 `enableApiControl → armDisarm → takeoffAsync → hoverAsync`，再交给规划器。
- **成功率不采信模型自判**。模型判定记在 `result.json.mission_success`，评测的 `success` 只由真值距离与碰撞决定；目录名 `success_` 前缀由评测写，官方 metric.py 以它为 SR 口径。
- **专家轨迹顺带记录**。数据集原 episode 的 `log/` 每帧位置即专家轨迹，构造 episode 时读入 `gt_positions`（[dataset.py:59-68](../eval/evalcore/dataset.py#L59-L68)），写进 `result.json.expert_positions` 供可视化。

## 2 模块职责

| 文件 | 职责 |
| --- | --- |
| `eval/run.py` | CLI 入口：`one` / `batch` / `serve`；分组、开场景、跑集、调打分、写报告 |
| `eval/evalcore/config.py` | 评测配置（`config/eval.yml` + 命令行覆盖）、场景启动脚本表 |
| `eval/evalcore/dataset.py` | 读 `UAV-VLN-FOV/<split>/`，复刻官方 `env_uav.py` 的 episode 构造（起点、目标 spawn-area、指令、专家轨迹） |
| `eval/evalcore/sim.py` | 拉起/关闭 TravelUAV、生成 AirSim settings、瞬移、刷物、取状态/碰撞/RGB；`collision_detail()` 只读诊断 |
| `eval/evalcore/planner_runtime.py` | 把活着的仿真器端口装成规划器的 `SimulatorFamily`，构建 `MissionResources`、装配 mission arm、跑任务；后台位姿采样 |
| `eval/evalcore/writer.py` | 磁盘契约：`log/NNNNNN.json`、`ori_info.json`、`result.json`，成功时目录加 `success_` 前缀 |
| `eval/evalcore/metrics.py` | 调仓库内置的 `eval/vendor/metric.py` 并解析其 stdout（SR/OSR/NE/SPL） |
| `eval/vendor/metric.py` | 官方打分脚本，取自 `3DG-VLN-main/utils/metric.py` 的原样副本，不随 `config/eval.yml` 配置 |
| `eval/evalcore/report.py` | `per_episode.csv`、`metrics.json`、`progress.json` |
| `eval/evalcore/runner.py` | 单集执行器（薄封装，指向 `run_episode_real`） |
| `eval/evalcore/dashboard.py` | 仪表盘 HTTP 服务：结果浏览、发跑/停止、删除、保存记录 |
| `eval/web/` | 无构建步骤的静态页面（`index.html` / `app.js` / `style.css`） |

## 3 磁盘契约（metric.py 能直接吃的格式）

```
eval/output/run_YYYYMMDD_HHMMSS/
  <uuid>/                          # 飞行中；成功时改名为 success_<uuid>/
    log/000000.json ...            # 5Hz 真实位姿帧（frame / position / orientation / linear_velocity）
    frame_front.png                # 每 5 帧采一张，供相机画面
    frame_down.png
    ori_info.json                  # {"ori_traj_dir": 数据集 episode 绝对路径}
    result.json                    # 评测附加元数据（metric.py 忽略）
  metrics.json                     # 聚合：SR/OSR/NE/SPL、逐集明细、profile、数据集信息
  per_episode.csv                  # uuid, scene, success, collided, final_ne, ...
  progress.json                    # 仪表盘轮询用的轻量进度，state 取 running|done|stopped（写完原子替换）
```

`result.json` 关键字段：`success`、`collided`、`final_ne`、`elapsed_s`、`target_position`、`planner_profile`、`mission_success`、`mission_error`、`collision_diag`（瞬移/刷物前后的 `simGetCollisionInfo` 快照）、`expert_positions`。

staged 管线还会在 `eval/output/visual_goal/<profile>/<时间戳>_<id>/` 写诊断 trace：`events.jsonl`（模型、运动参数、执行模式等运行时快照）、逐轮 RGB/深度/目标图、规划请求图。bare 管线不写 trace。

主服务控制台「保存执行记录」将任务记录写入 `eval/saved/<时间戳>_<slug>/record.json`，作为仪表盘「执行记录」页签的数据源。

## 4 配置与启动

评测配置只有 `config/eval.yml`（相对路径、输出目录、端口），其余全部继承规划器配置：

| 键 | 含义 |
| --- | --- |
| `root` / `root_path` | 仓库根与 TravelUAV 环境根（场景启动脚本相对它） |
| `dataset_root` | `UAV-VLN-FOV`，含 `test/`、`unscene/`、`unobject/`、`meta/` |
| `output_dir` / `saved_dir` | 批量结果与执行记录目录 |
| `dashboard_port` | 仪表盘端口（默认 8010，可被 `AVLN_EVAL_PORT` 覆盖） |

运行环境：`vln-ue4`（带 `airsim`、`numpy`、`httpx`、`PIL`、`opencv`），并与规划器共用 `VLN_API_KEY`。主服务拉起仪表盘时会继承自己的环境，所以密钥跟着主服务走；从 CLI 直接跑则需自行导出。

```bash
export VLN_API_KEY=...            # 模型端点密钥
cd Planner/eval

python run.py serve               # 只起仪表盘
python run.py one   --scenes TropicalIsland --limit-episodes 1 --planner-profile default
python run.py batch --scenes TropicalIsland --planner-profile vlm
```

常用参数：`--scenes`（逗号分隔；缺省=数据集全部）、`--uuids`、`--limit-episodes`、`--success-radius`、`--planner-profile`、`--dashboard-port`、`--gpus`。数据集切分由 `--config` 之外的 `split` 字段选择（默认 `test`）。

## 5 规划配置：profile 决定哪条执行管线

仪表盘可临时选 profile；未指定时读主服务 `/api/profile` 的当前档，读不到回退 `default`。注意评测的 profile 与主控台是**只读镜像**，不是另一套规划器。

| | `default`（世界模型档） | `vlm`（普通 VLM 档） |
| --- | --- | --- |
| 执行管线 | `staged`：停稳分段、航点校验、滑窗续规划（`continuous_world_queue`） | `bare`：一次出整批平移偏移，**不校验**，整条路径交引擎 `follow_path` |
| 航点格式 | 4 维（dx, dy, dz, dyaw） | 3 维平移 |
| 滑窗/完成确认 | 有 | 无（bare 刻意不含 pending 队列、平滑、碰撞检查、语义完成确认） |
| 典型参数 | 5 m/s、`max_step_m=10`、缓冲 10 点 | 3 m/s、`max_step_m=6`、缓冲 6 点 |
| 诊断 trace | 写 | 不写 |

**口径提醒**：两档的检测与轨迹模型目前都是 `deepseek-flash`（见 `config/core/*/base.yaml`）。`default` 的“世界模型”指这套 staged 导航核心的定位，**不是**接入世界模型权重；控制台链接的世界模型 Studio 是另一个 8765 服务，planner 与 eval 都不调用它。要在评测里对比真正的世界模型，需要先确定它在导航链路里替换的是检测、规划还是执行环节。

## 6 成功判定与指标

评测自己的成功口径：末帧距目标 spawn-area 中心 ≤ `success_radius_m`（默认 10 m）**且**本次采样期间未发生碰撞。目标位置与官方脚本一致，取 `mark.target.position` 最近的 spawn-area `[9:12]`。

官方 `metric.py`（仓库内副本 [eval/vendor/metric.py](../eval/vendor/metric.py)，取自 `3DG-VLN-main/utils/metric.py`）：

| 指标 | 口径 |
| --- | --- |
| SR | 目录名以 `success_` 开头的比例（即评测的真值判定） |
| OSR | 轨迹任一点进入成功半径的比例（oracle，不要求停住） |
| NE | 末帧到目标距离的均值；只统计 ≤ `max_ne_distance`（500 m）的集 |
| SPL | `S·L*/max(L*, P)`。`P`=实测折线长，`L*`=专家 log 路径长 − 半径；失败集记 0；与 SR 一样按集平均后 ×100 |

仪表盘对**批次卡片**的 SR 会按 `episodes[]` 的 `success` 重新统计，避免读到过期 `metrics.json`；删过集的批次，SR/NE 按剩余集重算，OSR/SPL 缺逐集中间量显示 `—`。

## 7 仪表盘

- 主控台「评测系统」按钮通过 `POST /api/eval/start` 按需启动并打开仪表盘。后续点击复用当前主服务启动的实例，子进程退出后可再次启动。端口被其他进程占用时，接口返回 `409`。也可直接运行 `python run.py serve`。改 Python 后端需重启，改 `eval/web` 静态文件刷新即可（`Cache-Control: no-store`）。
- 主服务退出时，由它启动的仪表盘也会关闭。手动启动的仪表盘不受影响。
- 「批量测试结果」：每次 run 一张批次汇总卡（SR/OSR/NE/SPL、成功/碰撞计数、逐集表），点击行切换样例；支持整批删除。轨迹图绘制实测轨迹与**专家轨迹（蓝色虚线）**，相机页签切换前视/下视。被停止的批次卡片标「已停止」。
- 演示批次（城市核心区，「世界模型」与「普通 VLM」两档）常驻批次列表末尾，有无真实数据、是否在跑评测都不隐藏；点开后不会被轮询自动切回真实批次。
- 评测进行中，进度行右侧出现「停止」按钮。`POST /api/stop` 终止评测子进程，并把该 run 的 `progress.json` 置为 `stopped`；仪表盘重启后丢了子进程句柄时，同一接口把残留的 `running` 收尾为 `stopped`。
- 「场景对比」按场景聚合同场景的多个批次，演示批次一并列出；没有 `metrics.json` 的未完成批次不参与对比。
- 「执行记录」：主控台保存的单次任务记录，含指令、成功率、轨迹、地图与 profile。
- HTTP 接口：

| 方法 | 路径 | 作用 |
| --- | --- | --- |
| GET | `/api/runs` | 所有 run 与批次汇总 |
| GET | `/api/state?run=` | 单 run 进度/指标/集列表 |
| GET | `/api/traj?run=&id=` | 某集实测轨迹 + 专家轨迹 + 目标 + 半径 |
| GET | `/api/frame?run=&id=&cam=` | 前视/下视缩略图 |
| GET | `/api/history`、`/api/dataset`、`/api/episodes`、`/api/runstatus`、`/api/eval/profile` | 历史、数据集、待测集、运行状态、profile 镜像 |
| POST | `/api/start`、`/api/stop` | 发跑/停止；`/api/stop` 在没有活进程时把该 run 残留的 `running` 收尾为 `stopped` |
| POST | `/api/runs/delete`、`/api/episode/delete`、`/api/saved/delete` | 删除批次/单集/保存记录 |

## 8 已知限制

- **停止或中断的 run 没有 `metrics.json`**。仪表盘读不到它就不生成报告（点「生成评测报告」提示「未完成的评测无法生成报告」），该批次也不进场景对比；批次卡片与逐集明细仍按已有的 `result.json` 显示。
- **中段碰撞仍是规划器侧问题**。评测只记录碰撞并判失败，`visual_goal_navigation.collision_check_enabled` 默认 `false`，工作流不会因碰撞中断。同一 episode 具有随机性（同集单独重跑可能不撞），原因在航点本身无障碍约束，以及位置保持阶段不感知碰撞，单集失败不应直接归因于评测器。
- **位姿采样 5 Hz** 决定 `P` 的折线粒度，急弯路径长度会略微低估；所有集同口径。
- **世界模型档未接入世界模型权重**，见第 5 节。
- 数据集场景必须同时出现在 `UAV-VLN-FOV/meta/map_spawnarea_info.json` 与 `evalcore/config.py` 的启动脚本表里，否则该场景会被跳过。
