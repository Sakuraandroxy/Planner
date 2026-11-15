# 部署

本文覆盖从零把服务跑起来、以及换一台机器后要动哪些东西。README 里的快速开始是它的摘要，细节以本文为准。

## 1 前置条件

| 项 | 要求 | 说明 |
|---|---|---|
| 系统 | Linux，有图形库 | 引擎以窗口模式跑在虚拟显示上 |
| Python | 3.10 以上 | 代码使用 `str \| None` 语法 |
| 显卡 | NVIDIA，显存足够跑一张 UE 地图 | 同时跑两张会争显存 |
| 显示与画面 | `Xvfb`、`xpra`、`xdpyinfo` | xpra 负责把引擎窗口搬到浏览器 |
| 引擎 | 两族的打包产物，不编译 | 只用 Shipping / NoEditor 二进制，不碰引擎源码 |

服务进程不加载任何仿真器客户端。UE4 的客户端库在独立解释器、独立子进程里跑，
这一条由 `tests/test_architecture.py` 守着。

## 2 两个解释器环境

必须分开，原因是一个确定的模块名撞车，不是谨慎：

- `airsim` 依赖 `msgpack-rpc-python`，后者依赖 `msgpack-python`，提供的模块名是 `msgpack`
- `projectairsim` 依赖 `msgpack>=1.0.5`，模块名同样是 `msgpack`

两个发行版往同一个 `site-packages/msgpack/` 写文件，后装的覆盖前一个。
`msgpackrpc` 用 `msgpack.Packer(encoding=...)` 构造打包器，而 `encoding` 参数在 msgpack 1.0 已删除。
结果是 `import airsim` 成功、`MultirotorClient()` 成功，第一次发包才炸：

```
TypeError: __init__() got an unexpected keyword argument 'encoding'
```

判断环境是否被污染，唯一可靠的检查是发包：

```bash
python -c "import airsim; airsim.MultirotorClient().ping()"
```

- 报 `TransportError`（连接被拒）：环境正确，只是没起仿真器
- 报 `TypeError`：`msgpack` 被 1.x 覆盖了，环境已坏

`import msgpackrpc, pynng, msgpack` 这类检查不碰打包 API，会给出假绿灯。

环境职责：

| 环境 | 用途 | 装的客户端 |
|---|---|---|
| 主环境（示例名 `vln`） | HTTP 服务、控制台、UE5 会话、测试 | `projectairsim` |
| 运行时环境（示例名 `vln-ue4`） | UE4 任务子进程 | `airsim` |

## 3 依赖安装

### 3.1 主环境

```bash
conda create -n vln python=3.11 -y && conda activate vln
pip install numpy pyyaml httpx fastapi "uvicorn[standard]" pydantic \
  opencv-python-headless Pillow pytest
pip install --no-deps projectairsim==1.0.2
pip install pynng "msgpack>=1.0.5" matplotlib commentjson "jsonschema>=4.4.0" \
  inputs pqdict cryptography pykml junit-xml jsonlines Shapely azure-storage-blob
```

三个要点：

- **`projectairsim` 用 `--no-deps`，基础依赖手工补齐。** 它声明依赖 `opencv-python`（GUI 版），
  会把第二份 `cv2/` 写进 `site-packages`，和 `opencv-python-headless` 抢同一个目录。
  上面第三条命令就是它 wheel 元数据里的基础依赖，去掉了 `opencv-python`。
- **`Pillow` 是主环境的硬需求。**
  `planner/simulators/airsim/observation_source.py` 在模块顶层 `from PIL import Image`，缺它会有测试失败，
  失败点看着与 UE4 无关。
- **`pip check` 会报 `opencv-python is not installed`。** 满足它的是 headless 变体，属预期噪音。

### 3.2 运行时环境

```bash
conda create -n vln-ue4 python=3.11 -y && conda activate vln-ue4
pip install numpy requests Pillow pyyaml pydantic \
  opencv-contrib-python-headless "msgpack-rpc-python>=0.4.1"
pip install --no-deps --no-build-isolation airsim==1.8.1
```

- **先装 numpy 再装 airsim。** `airsim` 只有 sdist，`setup.py` 开头就 `import numpy`，构建隔离环境里没有它，
  报 `Getting requirements to build wheel did not run successfully`。`--no-build-isolation` 让它用当前解释器。
- `airsim` 声明的是 `opencv-contrib-python`（GUI 版），用 headless 变体替代，同样有 `pip check` 噪音。
- `msgpack-python` 在这个 Python 上没有 C 扩展，打包走纯 Python 实现。功能正确，取图慢；
  换 msgpack 1.x 会破坏 `msgpackrpc`，不要在依赖上动它。深度分辨率因此是配置项（见 7 节）。
- 这个环境需要 `httpx`：任务解析与轨迹规划在子进程侧发 HTTP。

## 4 配置总览

四个配置来源，各管一件事，不要在第五处重复：

| 文件 | 管什么 | 换机器常改 |
|---|---|---|
| `config/maps.yaml` | 有哪些地图、每张图的二进制与地图名、属于哪个仿真器 | 每条的 `binary` |
| `config/simulators.yaml` | 每个仿真器怎么起停、监听哪些端口、用哪个解释器跑子进程 | `runtime_python`、`ports`、`argv` 里的 xpra 路径 |
| `config/base.yaml` | 服务、相机、运动、滑窗、四个模型端点、显示号、超时 | 模型端点与密钥来源、`simulator.display` |
| `config/airsim_settings.json` | UE4 引擎设置：载具名、相机名、分辨率、RPC 端口 | 载具名、相机名、分辨率 |

场景与任务规则：

| 路径 | 内容 |
|---|---|
| `config/scenes/<map>/*.jsonc` | UE5 场景与机器人文件（双载具、相机、流式开关） |
| `config/tasks/*.yaml` | 视觉目标的完成判定规则 |

`config/simulators.yaml` 缺失时配置加载直接报错，没有内置回落。

## 5 `config/base.yaml`

`simulator` 段只放与进程无关的东西：

| 字段 | 含义 |
|---|---|
| `display` | X 显示号，形如 `:104`。端口在 `simulators.yaml` 的家族段里 |
| `runtime_dir` | pidfile、xpra 会话与日志目录 |
| `map_catalog_file` | 地图目录文件名，相对 `config/` |
| `map_marker_file` | 当前地图标记文件；服务重启时按它恢复 |
| `launch_timeout_s` / `stop_timeout_s` / `port_release_timeout_s` | 起、停、端口释放的超时 |
| `restart_attempts` | 换图失败时整套重启的次数 |

其余常用段：

| 段 | 内容 |
|---|---|
| `projectairsim` | 地址、topic/service 端口、默认场景文件、相机与载具名、`view_cycle_start` |
| `camera` / `depth` | 前视相机分辨率与视场角、安装偏移、深度语义与可视化范围 |
| `motion` / `motion.continuous` | 速度与加速度上限、控制频率、前瞻参数 |
| `sliding_window` | 队列容量、补点阈值、续规划周期、失败与时长上限 |
| `free_navigation` | 碰撞倒车距离、最大恢复次数、恢复预算清零里程 |
| `visual_goal_navigation` | 完成规则文件、确认次数、搜索与接近半径、诊断输出目录 |
| `server` | 监听地址、载具列表、默认载具、日志缓冲、CORS、视频帧率 |
| `target_detection` / `trajectory_planner` / `task_completion` / `task_parser` | 四个模型端点 |

## 6 `config/simulators.yaml`

每个仿真器一段。`projectairsim` 段的 `argv` 用 YAML 锚点定义，`airsim` 段引用同一份，
两族因此共用一套 xpra 会话参数。测试断言两处相等，改任何一族前先看那条测试。

| 字段 | 含义 | 注意 |
|---|---|---|
| `driver` | 进程驱动名，目前只有 `xpra_start` | 写成别的会被配置校验拒绝 |
| `runtime` | 该族的 Python 包 | 由注册表按地图的 `simulator` 字段解析 |
| `ports` | 该族监听的端口 | 键名同时是 `{ports.<键>}` 占位符；UE4 的 `rpc` 必须等于引擎 `ApiServerPort` |
| `probes` | 就绪判据：`window` / `ports` / `rpc` | UE5 用前两个，UE4 三个都要 |
| `child_argv` | 引擎命令行模板 | 端口与分辨率走占位符 |
| `argv` / `stop_argv` | xpra 起停参数 | 两族共用一份 |
| `settings_file` | 引擎 settings 文件 | UE5 没有这一项；它同时是 UE4 的归属指纹 |
| `runtime_python` | 跑任务子进程的解释器 | 只有 UE4 有；没有这一项的家族用线程 worker |
| `host` / `vehicle_name` / `camera_id` / `timeout_s` | 连接数据 | `vehicle_name` 留空会让控制类 RPC 静默失效 |
| `res_x` / `res_y` | 画面尺寸 | 同时决定 Xvfb 分辨率与引擎窗口参数 |
| `abslog_dir` | 引擎日志目录模板 | 只对会写日志的构建有意义 |

能力位不在这里：写 `capabilities` 会被配置校验拒绝，能力由端口推导（见架构文档）。

`argv` 里是 `xpra` 的绝对路径。发行版把它装在别处时改模板，不要在代码里加分支。

## 7 `config/maps.yaml`

```yaml
default_map: cityenviron
maps:
  - id: cityenviron              # 控制台与 API 使用的地图标识
    simulator: projectairsim     # 由哪个仿真器驱动，对应 simulators.yaml 的一段
    label: 城市 CityEnviron       # 控制台显示名
    binary: /path/to/CityEnviron-Linux-Shipping
    map_name: CityEnviron        # 传给二进制的参数，窗口标题与它同字
    scene_file: scene_city_dual.jsonc
    sim_config_path: config/scenes/city
```

- `binary` 同时是进程归属判据：停止时按 `/proc/<pid>/exe` 的真路径判断"这是不是我们的进程"。
- `map_name` 不一定等于目录名。Neighborhood 的目录叫 `Neighborhood`，交给引擎的地图名是 `AirSimNH`，这是引擎自己的命名。
- 地图引用的 `simulator` 必须在 `simulators.yaml` 里存在，否则配置加载报错。
- UE5 的图还带 `scene_file` 与 `sim_config_path`；UE4 的图只带 `binary` 与 `map_name`。

## 8 `config/airsim_settings.json`

UE4 引擎读取的设置，规划器自己保存一份，不动引擎目录里的那份。

| 字段 | 要求 |
|---|---|
| `SimMode` | `Multirotor` |
| `ApiServerPort` | 必须等于 `simulators.yaml` 的 `airsim.ports.rpc`；缺省是 41451 |
| `Vehicles.<名字>` | 载具名必须等于 `simulators.yaml` 的 `airsim.vehicle_name` |
| `Vehicles.<名字>.Cameras.<名字>` | 相机名必须等于 `airsim.camera_id` |
| `ImageType: 0` | RGB，分辨率与 `base.yaml` 的 `camera.width/height` 一致 |
| `ImageType: 2` | 深度，允许独立分辨率；几何用同一 FOV |

深度分辨率取 256×144 的原因是传输：`pixels_as_float=True` 把深度当 float 数组发回，
纯 Python 的 msgpack 解包 36 864 个 float 约 0.24 秒，921 600 个要 25 秒以上。两个 ImageType 用同一个 FOV，
深度几何才能直接用 `depth_intrinsics` 反投射。

相机级覆盖写在 `Vehicles.<名字>.Cameras.<相机>.CaptureSettings`；写在 `CameraDefaults` 不会生效。

## 9 UE5 场景与机器人文件

`config/scenes/<map>/` 里每个场景一套：

| 文件 | 作用 |
|---|---|
| `scene_*_dual.jsonc` | 双载具场景：同时生成 Rover 与 Drone，时钟 `pause-on-start: true` |
| `robot_rover.jsonc` / `robot_drone_flight.jsonc` | 载具外观、相机安装、采集与流式开关 |

约束：

- 两台载具出生点完全相同，同一时刻只显示一台。时钟在加载时暂停，无人机不会在加载等待里下坠。
- 恰好四个相机打开 `streaming-enabled`（车前、车尾、机前、机尾）。服务端只能在它们之间循环。
- 无人车追尾相机 `capture-enabled: false`，标定工具不尝试对它取帧。
- 六旋翼需要单独的场景变体，`base.yaml` 的 `vehicle_models` 里为每张可切换地图指定文件。

## 10 模型端点与密钥

四段配置各有一个端点，都接受 OpenAI 兼容协议：

| 段 | 用途 | 超时 |
|---|---|---|
| `task_parser` | 自然语言 → 任务阶段 | 60 s |
| `target_detection` | 视觉目标定位，给出框与表面点 | 30 s |
| `trajectory_planner` | 5 航点轨迹 | 60 s |
| `task_completion` | 到达后的语义确认，只接受 true/false | 20 s |

- `request_format` 取 `openai`、`deepseek` 或 `anthropic`，决定请求字段与鉴权头。
  `anthropic` 只接受 `reasoning_effort: none`。
- `api_key` 直接写在配置里时优先于 `api_key_env`；留空则读环境变量，默认 `VLN_API_KEY`。
  填了真实密钥的配置文件不要提交到版本库。
- 端点不可用时任务被明确拒绝，服务本身照常启动、画面照常可看。拒绝信息里带状态码、 URL、
  该改哪个字段与 provider 原话。

## 11 启动、验证、关闭

### 启动

```bash
python scripts/serve_planner.py --host 0.0.0.0 --port 8000
```

- 服务启动时状态是 `disconnected`：不加载场景，不连仿真器，控制台没有画面。
- 启动时若发现上一轮由本项目拉起的仿真栈还在跑，会先停掉它。别人手工起的栈不动，这时画面可能还显示旧场景。
- 场景由控制台「启动场景」或 `POST /api/world` 拉起。

### 验证

```bash
python scripts/probe_ue4_env.py                 # 按角色判，两个环境都应 0 failed
python -m pytest -q                             # 单元测试
curl -s localhost:8000/api/status | head -c 400  # 看 state / map / capabilities
```

实机冒烟脚本只覆盖 UE5，且要求没有别的客户端占用仿真器：

```bash
python scripts/smoke_projectairsim.py            # 双载具场景与取图
python scripts/smoke_projectairsim_adapter.py    # 连接、取帧、碰撞、重载场景
python scripts/smoke_projectairsim_rover.py      # 直线、连续导航、停止
python scripts/smoke_projectairsim_drone.py      # 起飞、位置保持、基础飞行、视觉目标
```

### 手动起停仿真栈

控制台之外的唯一入口，两族同一条命令：

```bash
PYTHONPATH=. python -c "
from pathlib import Path
from planner.config import load_config
from planner.simulator.router import FamilyRouter
cfg = load_config('config/base.yaml')
router = FamilyRouter(cfg.families, cfg.simulator, Path('.').resolve(), cfg.maps)
router.restart(cfg.maps.entry('cityenviron'))
print('running:', router.running())
"
```

停止把 `restart` 换成 `stop()`。已有图在跑时 `start` 会拒绝，这是有意的。画面地址是 `http://<主机>:10010/`。

### 关闭

`Ctrl-C` 停服务会带走本服务拉起的仿真栈：引擎、xpra 会话与 Xvfb 一起停。不属于本服务的栈只告警不动。

## 12 换机器要改什么

| 项 | 位置 | 说明 |
|---|---|---|
| 引擎二进制路径 | `config/maps.yaml` 每条的 `binary` | 逃不掉的一项；`binary` 也是所有权判据 |
| UE4 子进程解释器 | `config/simulators.yaml` 的 `airsim.runtime_python` | 指向装了 `airsim` 的环境 |
| 引擎 settings | `config/simulators.yaml` 的 `airsim.settings_file` | 相对路径以仓库根为基准 |
| 显示号 | `config/base.yaml` 的 `simulator.display` | 形如 `:104` |
| 端口 | `config/simulators.yaml` 各族的 `ports` | 改 UE4 的 `rpc` 时要同步改引擎 settings |
| xpra 路径 | `config/simulators.yaml` 的 `argv` / `stop_argv` | 发行版装在别处才需要改 |
| 模型端点与密钥 | `config/base.yaml` 的四段 | 换 provider 时改 `request_format` |
| UE4 载具名与相机名 | `airsim_settings.json` 与 `simulators.yaml` | 两处必须一致 |
| UE5 机位起点 | `config/base.yaml` 的 `projectairsim.view_cycle_start` | 换了包或图之后需要重新确认 |
| GPU 选择 | 环境变量或引擎启动参数 | 有坏卡时显式绑定健康卡 |

`config/maps.yaml` 至少有 `default_map` 一张图；`default_map` 必须在列表里。

示例值（可直接照抄再替换）：

```yaml
# config/simulators.yaml · airsim.runtime_python
runtime_python: /home/<user>/miniconda3/envs/vln-ue4/bin/python

# config/maps.yaml
binary: /opt/simulators/CityEnviron/Binaries/Linux/CityEnviron-Linux-Shipping
binary: /data/airsim/Blocks/LinuxNoEditor/Blocks/Binaries/Linux/Blocks
```

## 13 故障排查

| 现象 | 原因 | 处理 |
|---|---|---|
| 启动场景超时，且引擎一个字节日志都没有 | 显示号被残留的 X 锁占着，子进程根本没起来 | 检查 `/tmp/.X<n>-lock`；驱动只回收持有者已死的锁，别人的不删 |
| `another ProjectAirSim connection is active` | 同一进程里还有未关闭的连接 | 会话关闭时用 `connection.close()`，不是 `client.disconnect()` |
| 取帧一直不返回 | 窗口没 map。seamless xpra 在客户端接上前不 map 窗口，引擎对未 map 窗口暂停渲染 | 驱动会 `xpra control map` + `focus`；不要用 `-RenderOffscreen` 绕，它把桌面画面一起去掉 |
| `TypeError: unexpected keyword argument 'encoding'` | `msgpack` 被 1.x 覆盖 | 重建运行时环境，见 3.2 |
| 任务全被拒绝，日志提到 401/403 | 模型端点凭据不对 | 看日志里的状态码与字段提示；补 `VLN_API_KEY` 或配置 `api_key` |
| 换图返回 400 | 仿真器不是本服务拉起的，归属判据对不上 | 用 11 节的手动起停命令接管，或先停掉外部栈 |
| 控制台没有某些控件 | 当前地图的仿真器没有对应端口 | 这是设计：能力由端口推导。`/api/status` 的 `capabilities` 可见 |
| 接口返回 501 并带 `unsupported` | 该动作背后的端口当前家族没有实现 | 同上，不是故障 |
| 无人机第二个任务贴地前飞 | `landed_state` 落地后停在 Flying 不回 0 | 起飞判据已改为按地面高度比较；会话重建后仍复现时检查高度参考 |
| 切换载具后车悬空 | 归位目标用了场景文件里的作者坐标 | 归位用暖机刹车沉降后实测的贴地姿态，日志里有 `Rover home pose measured` |
| 自由导航反复撞同一障碍 | 没有车道参考，提示词只能靠画面判断 | 属已知限制，不是崩溃 |

## 14 运行目录与日志

| 路径 | 内容 |
|---|---|
| `output/xpra/` | xpra 会话、pidfile、`current_map` 标记 |
| `output/xpra/logs/` | xpra 与引擎子进程的输出 |
| `output/logs/` | 引擎 `abslog` 目录 |
| `output/visual_goal/<run_id>/` | 视觉任务的观测帧、深度、事件、控制遥测 |
| `output/calibration/` | 机位标定截图 |

服务日志进内存环形缓冲，通过 `GET /api/logs?cursor=N` 增量读取，进程退出即丢失。
Python 日志与 uvicorn 输出走 stderr。

`current_map` 只在拉起成功后写入。服务重启时读它恢复地图；内容不在目录里才回退到 `default_map`，
避免把城市地图的配置加载进郊区引擎。