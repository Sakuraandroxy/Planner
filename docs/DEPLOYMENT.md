> 上级：[项目架构](ARCHITECTURE.md) · [项目说明](../README.md) · [开发文档](DEVELOPMENT.md)
> 层文档：[domain](layers/domain.md) · [ports](layers/ports.md) · [algorithms](layers/algorithms.md) · [runtime](layers/runtime.md) · [workflows](layers/workflows.md) · [assembly](layers/assembly.md) · [adapters](layers/adapters.md) · [simulator](layers/simulator.md) · [simulators](layers/simulators.md)

# 部署文档

本机单机部署：一个 HTTP 服务、一个浏览器控制台、一套按需拉起的仿真器进程栈。所有路径与端口都取自仓库内的配置文件，改机器只需改 `config/maps.yaml` 的 `root` 与 `config/app.yml` 的 `server` 段。

## 1 目标环境

| 项 | 要求 | 出处 |
| --- | --- | --- |
| 操作系统 | Linux，具备 X11 与 `xpra` | `planner/simulator/xpra_start.py` |
| Python | 3.10 以上，实测 3.11.18 | `requirements.txt` 首行注释 |
| 主环境名 | `vln` | `scripts/serve_planner.py` 文档串 |
| UE4 环境名 | `vln-ue4`，独立于主环境 | `config/simulators.yaml` 的 `runtime_python` |
| 引擎包根目录 | `/roxy/vln` | `config/maps.yaml` 的 `root` |
| 显示号 | `:104` | `config/app.yml` 的 `process.display` |
| 服务地址 | `127.0.0.1:8000` | `config/app.yml` 的 `server` 段 |

两个环境必须分开：UE5 的 `projectairsim` 依赖 `msgpack 1.2.4`，UE4 客户端依赖 `msgpack-python 0.5.6`，同名不同包，装不进同一环境。

## 2 安装

```bash
conda create -n vln python=3.11 -y
conda activate vln
pip install -r requirements.txt
pip install --no-deps projectairsim==1.0.2
```

`projectairsim` 单独装的原因写在 `requirements.txt` 里：它声明 GUI 版 opencv，会与 `opencv-python-headless` 抢同一个 `cv2` 目录。它的运行时依赖已列在该文件末尾。

提示词模板由 `jinja2` 渲染，`requirements.txt` 里已声明 `jinja2>=3.1`，上面的安装命令会一并装上，不需要另装。

UE4 支持是可选的：

```bash
conda create -n vln-ue4 python=3.11 -y
~/miniconda3/envs/vln-ue4/bin/pip install -r requirements-ue4.txt
```

`config/simulators.yaml` 的 `airsim.runtime_python` 必须指向这个环境，否则 UE4 任务会话起不来。

前端控制台是静态文件，不需要构建步骤，由服务直接托管 `console/`。

## 3 配置

配置文件分四层，加载顺序由 `planner/config/loader.py` 决定。

| 文件 | 管什么 |
| --- | --- |
| `config/app.yml` | 服务端口、日志缓冲、默认载具、进程栈的显示号与超时 |
| `config/simulators.yaml` | 每套仿真器一段：驱动、端口、相机、场景约定、就绪探针 |
| `config/maps.yaml` | 可切换的 cooked 地图清单，一条对应一个引擎二进制 |
| `config/core/<档>/base.yaml` | 任务与模型端点。`default` 不带视觉模型，`vlm` 带 |
| `config/core/<档>/tasks/*.yaml` | 任务级参数，例如视觉目标任务的接近阈值 |

换机器只需改两处：`config/maps.yaml` 的 `root` 指向引擎包根目录，`config/app.yml` 的 `server.host` 与外网可达性按需调整。

### 3.1 模型端点

四段模型都在 `config/core/<档>/base.yaml`：`target_detection`、`trajectory_planner`、`task_completion`、`task_parser`。共同字段是 `url`、`model`、`api_key`、`api_key_env`、`request_format`、`timeout_s`。

密钥用环境变量传：

```bash
export VLN_API_KEY=<key>
```

`api_key` 直接写在配置里会覆盖环境变量，填真实密钥后不要提交该文件。

`request_format` 取 `openai`、`deepseek`、`anthropic`，决定请求体形状与思考开关。`anthropic` 只接受 `reasoning_effort: none`。

模型端点留空或不可用时，服务照常启动，控制台能看画面，但任务请求会被明确拒绝，没有确定性回退。

## 4 启动

```bash
conda activate vln
cd /roxy/kedior/Planner
python scripts/serve_planner.py
```

常用参数：

| 参数 | 作用 |
| --- | --- |
| `--profile <名>` | 选 `config/core/` 下的档，默认 `default` |
| `--config <路径>` | 直接指定一个 `base.yaml` |
| `--host` / `--port` | 覆盖 `config/app.yml` 的 `server` 段 |

服务启动后处于未连接状态：不加载场景，不打开仿真器 socket。浏览器打开 `http://127.0.0.1:8000/`，在世界设置页选地图，进程栈才被拉起。

启动时只清理本项目自己拉起的残留进程。归属判定依据引擎二进制的真实路径与 xpra 会话目录，别人启动的栈不动。

## 5 进程栈

一次选图会拉起三层进程：

```
xpra start --start-child <UE 二进制> --bind-tcp ...
  └─ Xvfb（xpra 自带）
  └─ UE5 或 UE4 引擎，加载一张 cooked 地图
```

要点：

- 一个 UE 进程只加载一张地图，运行中换不了。切图等于停栈、按新图拉起、重连。
- 就绪判据是三个探针：`window` 找引擎窗口并校验归属、`ports` 确认端口监听、`rpc` 向 AirSim 发一次真实调用。
- 启动超时 `process.launch_timeout_s` 默认 180 秒，停止超时 40 秒，端口释放等待 30 秒，崩溃重启 2 次。
- 日志写 `output/logs/`，xpra 运行时目录 `output/xpra/`，当前地图标记在 `output/xpra/current_map`。

端口：

| 引擎 | 端口 |
| --- | --- |
| UE5 `projectairsim` | xpra 10010、topics 8991、services 8992 |
| UE4 `airsim` | xpra 10010、rpc 41451 |

## 6 端点

| 方法 | 路径 | 作用 |
| --- | --- | --- |
| GET | `/` | 控制台页面 |
| POST | `/api/tasks` | 提交任务，体为 `{"instruction": "...", "vehicle": "rover"\|"drone"}` |
| GET | `/api/status` | 服务、场景、载具、任务状态 |
| GET | `/api/logs?cursor=N` | 增量日志 |
| GET | `/api/trajectory` | 已授权航点与轨迹；服务未接轨迹汇时返回 503（`detail.reason = trajectory_hub_missing`） |
| GET | `/api/video` | MJPEG 画面流，运行中实时，空闲时冻结；缺帧源或缺 `video` 能力位时返回 501，原因写在 `detail.reason` |
| POST | `/api/stop` | 停止当前任务 |
| POST | `/api/home` | 回起点，仅空闲时可用 |
| GET/POST | `/api/world` | 世界设置与选图 |
| GET | `/api/world/options` | 可选地图清单 |
| POST | `/api/world/stop` | 停栈 |
| GET/POST | `/api/weather` | 天气读取与下发 |
| GET | `/api/models` | 车型清单，UE5 独有 |
| GET/POST | `/api/profile` | 规划档读取与切换 |
| GET/POST | `/api/view` | 机位循环，UE5 独有 |
| GET | `/api/vehicle` | 当前载具 |

`/api/status` 与 `/api/world/options` 恒带 `capabilities` 字段，能力位不可得时为 `null`，控制台按 `null` 判定控件不可用。

UE4 不支持 `/api/video` 的帧源，该端点返回 501（`detail.reason = capability_missing`）；服务构造时没接帧汇则是 501 与 `frame_hub_missing`。前端按 `reason` 停止重试并在状态行上给原因。

## 7 首次标定

换引擎包或换地图后需要标定一次相机机位，否则越点走廊与表面点采样会偏。

```bash
conda activate vln
python scripts/calibrate_streaming_views.py
```

标定结果写回配置里的起图机位。运行时不做标定，这一步是人工的。

`uncapturable_views` 字段用于记录采不到帧的机位，标定靠排除法定位它们。

阈值出处

- 段拍 `motion.segment_period_s`（默认 0.03 s）与流拍 `1 / motion.control_hz`（默认 20 Hz）是两个独立配置项，不共用一个值：段执行每一拍要判停稳、算剩余距离并重锚，采样要细；流执行只是命令下发节拍。两项都在 `config/core/<档>/base.yaml` 的 `motion` 段，校验为正有限数。
- 越点走廊 1.0 m、`FreeSpaceProfileSettings` 的九个阈值、三套停滞判据都是经验值、未标定：影响因素写在各自代码注释里，仓库里没有标定脚本与回归基线。改动前按 `tests/test_free_space.py`、`tests/test_algorithms_motion.py`、`tests/test_free_navigation.py` 的期望值一起改。

## 8 自检

部署完成后按顺序跑：

```bash
python -m pytest tests/ -q                  # 全量用例
python scripts/check_profiles.py           # 配置档自检
python scripts/smoke_projectairsim.py      # UE5 连通性
python scripts/smoke_projectairsim_adapter.py
python scripts/smoke_projectairsim_drone.py
python scripts/smoke_projectairsim_rover.py
python scripts/probe_ue4_env.py            # 仅在部署 UE4 时
```

前两条不依赖仿真器，可以在没有引擎的机器上跑。

## 9 故障排查

| 现象 | 判断 | 处理 |
| --- | --- | --- |
| 服务起来但选图卡住 | 引擎没出窗口，`window` 探针一直失败 | 看 `output/logs/` 下该次启动的日志；确认 `maps.yaml` 的 `root` 与 `binary` 拼出的路径存在 |
| 选图报端口占用 | 上一套栈没退干净 | 等 `process.port_release_timeout_s`；仍占用就手工确认残留进程归属 |
| `/api/video` 返回 501 | 该引擎没有帧源，或服务没接帧汇 | 看 `detail.reason`：`capability_missing` 对 UE4 属预期；`frame_hub_missing` 检查 `create_api(frames=...)` 的传参 |
| `/api/trajectory` 返回 503 | 服务构造时漏传轨迹汇 | 看 `detail.reason`；按 `docs/layers/assembly.md` 检查 resources 构造 |
| 任务被拒绝，提示模型不可用 | 四段端点里有一段留空或不可达 | 检查 `config/core/<档>/base.yaml` 的 `url`、`model`、`VLN_API_KEY` |
| 载具不动但轨迹在增长 | 执行层没拿到控制权或命令被校验拒绝 | 看日志里的命令拒绝原因；对照 `docs/layers/runtime.md` |
| 换图后机型或相机名对不上 | 场景 JSONC 与 `simulators.yaml` 的 `scene_defaults` 不一致 | 按场景文件改 `rover_name`、`drone_name`、`cameras` |
| UE4 任务起不来 | `runtime_python` 指向的环境缺依赖 | 按第 2 节装 `requirements-ue4.txt` |

## 10 停机

正常停机：控制台点停止场景，或 `POST /api/world/stop`，再 Ctrl-C 停服务。服务退出时会关闭 MJPEG 流，避免连接排空阻塞。

异常停机后再启动，服务会按归属判定清理自己拉起的残留栈。

## 11 与各层的关系

部署动作主要落在三处：

- 进程栈由 [simulator 层](layers/simulator.md) 负责，配置由 `planner/config/` 读取。
- 引擎实现由 [simulators 层](layers/simulators.md) 提供，两套引擎的能力差异见该文档的对照表。
- HTTP 端点由 [adapters 层](layers/adapters.md) 提供，端点与字段契约见该文档。
