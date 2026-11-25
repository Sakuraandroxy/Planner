# 部署

本文覆盖从零把服务跑起来、以及换一台机器后要动哪些东西。README 里的快速开始是它的摘要，细节以本文为准。

## 1 前置条件

| 项         | 要求                               | 说明                                          |
| ---------- | ---------------------------------- | --------------------------------------------- |
| 系统       | Linux，有图形库                    | 引擎以窗口模式跑在虚拟显示上                  |
| Python     | 3.10 以上                          | 代码使用 `str \| None` 语法                   |
| 显卡       | NVIDIA，显存足够跑一张 UE 地图     | 同时跑两张会争显存                            |
| 显示与画面 | `Xvfb`、`xpra`、`xdpyinfo`         | xpra 负责把引擎窗口搬到浏览器                 |
| 引擎       | 用两套仿真器现成的打包产物，不编译 | 只用 Shipping / NoEditor 二进制，不碰引擎源码 |

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

| 环境                           | 用途                              | 装的客户端      |
| ------------------------------ | --------------------------------- | --------------- |
| 主环境（示例名 `vln`）         | HTTP 服务、控制台、UE5 会话、测试 | `projectairsim` |
| 运行时环境（示例名 `vln-ue4`） | UE4 任务子进程                    | `airsim`        |

## 3 依赖安装

两个环境的依赖各一份清单，分别在 `requirements.txt` 与 `requirements-ue4.txt`。

### 3.1 主环境

```bash
conda create -n vln python=3.11 -y && conda activate vln
pip install -r requirements.txt
pip install --no-deps projectairsim==1.0.2
```

三个要点：

- **`projectairsim` 用 `--no-deps`，基础依赖由清单末尾那几行补齐。** 它声明依赖 `opencv-python`（GUI 版），
  会把第二份 `cv2/` 写进 `site-packages`，和 `opencv-python-headless` 抢同一个目录。清单里已列出它
  wheel 元数据里的基础依赖，只去掉了 `opencv-python`。
- **`Pillow` 是主环境的硬需求。**
  `planner/simulators/airsim/observation_source.py` 在模块顶层 `from PIL import Image`，缺它会有测试失败，
  失败点看着与 UE4 无关。
- **`pip check` 会报 `opencv-python is not installed`。** 满足它的是 headless 变体，属预期噪音。

### 3.2 运行时环境

```bash
conda create -n vln-ue4 python=3.11 -y && conda activate vln-ue4
pip install -r requirements-ue4.txt
pip install --no-deps --no-build-isolation airsim==1.8.1
```

- **先装清单再装 airsim，顺序不能反。** `airsim` 只有 sdist，`setup.py` 开头就 `import numpy`，构建隔离环境里
  没有它，报 `Getting requirements to build wheel did not run successfully`。
  `--no-build-isolation` 让它用当前解释器。
- `airsim` 声明的是 `opencv-contrib-python`（GUI 版），用 headless 变体替代，同样有 `pip check` 噪音。
- `msgpack-python` 在这个 Python 上没有 C 扩展，打包走纯 Python 实现。功能正确，取图慢；
  换 msgpack 1.x 会破坏 `msgpackrpc`，不要在依赖上动它。深度分辨率因此是配置项（见 7 节）。
- 清单里有 `httpx`：任务解析与轨迹规划在子进程侧发 HTTP 请求。

## 4 配置总览

五个配置来源，各管一件事，不要在第六处重复：

| 文件                              | 管什么                                                                                          | 换机器常改                                                 |
| --------------------------------- | ----------------------------------------------------------------------------------------------- | ---------------------------------------------------------- |
| `config/app.yml`                  | 程序级共用：HTTP 服务（监听、载具、CORS、帧率）与仿真器进程托管（显示号、运行时目录、起停超时） | `server.host` / `server.port`、`process.display`           |
| `config/maps.yaml`                | 有哪些地图、包根目录、每张图的相对二进制与地图名、属于哪个仿真器、该图的场景身份                | `root` 一行、`scene` 段                                    |
| `config/simulators.yaml`          | 每个仿真器怎么起停、监听哪些端口、图像参数、可选车型                                            | `runtime_python`、`ports`、`camera`、`argv` 里的 xpra 路径 |
| `config/core/<profile>/base.yaml` | 导航与控制参数、四个模型端点                                                                    | 模型端点与密钥来源                                         |
| `config/airsim_settings.json`     | UE4 引擎设置：载具名、相机名、分辨率、RPC 端口                                                  | 载具名、相机名、分辨率                                     |

一份配置（profile）包含 `config/core/<profile>/base.yaml` 和它引用的判定规则。服务默认加载 `default`，也可用 `python scripts/serve_planner.py --profile vlm` 启动普通 VLM 档。

| 配置      | 控制台名称 | 视觉目标执行方式                         |
| --------- | ---------- | ---------------------------------------- |
| `default` | 世界模型   | 规划器执行，由规划器校验航点并控制载具   |
| `vlm`     | 普通 VLM   | 仿真器路径跟随，每轮把一批航点交给仿真器 |

`app.yml`、`maps.yaml`、`simulators.yaml`、`scenes/`、`airsim_settings.json` 属于部署，所有配置共用一份；某份配置需要自己的副本时，放进它自己的目录里就优先。

装好的配置可以列出来，同时检查它们有没有在不该不同的段上写歪：

```bash
python scripts/check_profiles.py
```

切换配置在控制台的「规划设置」标签页。场景已启动且空闲时，切换会重建会话、重载场景、载具回到出生点，
约 8 到 10 秒。场景未启动时，切换只记下名字，下一次启动场景就按它装配。
切换不动仿真器进程，所以两份配置只在导航与模型相关段上不同，上面那条检查守的就是这一点。

场景与任务规则：

| 路径                                 | 内容                                           |
| ------------------------------------ | ---------------------------------------------- |
| `config/scenes/<map>/*.jsonc`        | UE5 场景与机器人文件（双载具、相机、流式开关） |
| `config/core/<profile>/tasks/*.yaml` | 视觉目标的完成判定规则，跟着配置走             |

`config/simulators.yaml` 缺失时配置加载直接报错。

## 5 `config/app.yml`

程序级设置，所有配置共用，与地图和导航逻辑分开：

```yaml
server: # HTTP 服务：监听地址、可接任务的载具、CORS、日志缓冲、视频帧率
  host: 127.0.0.1
  port: 8000
  vehicles: ["rover", "drone"]
  default_vehicle: drone
process: # 仿真器进程托管：显示号、运行时目录、起停超时、地图目录与标记文件
  display: ":104"
  runtime_dir: output/xpra
  launch_timeout_s: 180.0
```

配置文件里出现不属于配置的段时，配置加载直接报错，并指出该段应该在哪份文件里。

## 6 `config/core/<profile>/base.yaml`

一份配置只写影响导航与控制的东西：运动包线、感知与导航参数、四个模型端点。
与地图、场景或仿真器包绑定的值放在这些位置：

| 内容                               | 位置                                              |
| ---------------------------------- | ------------------------------------------------- |
| 场景文件、场景里的名字与机位       | `config/maps.yaml` 每条的 `scene` 段              |
| 地址、端口、超时                   | `config/simulators.yaml` 每套仿真器一段           |
| 相机分辨率、视场角、挂点、深度解码 | `config/simulators.yaml` 每套仿真器的 `camera` 段 |
| 显示号、运行时目录、起停超时       | `config/simulators.yaml` 顶部的 `process` 段      |

常用段：

| 段                                                                            | 内容                                                                        |
| ----------------------------------------------------------------------------- | --------------------------------------------------------------------------- |
| `vehicle_models`                                                              | 控制台可选的车型清单；场景变体不在其中，写在每张图的 `scene.vehicle_scenes` |
| `depth`                                                                       | 深度语义与可视化范围                                                        |
| `motion` / `motion.continuous`                                                | 速度与加速度上限、控制频率、前瞻参数                                        |
| `sliding_window`                                                              | 队列容量、补点阈值、续规划周期、失败与时长上限                              |
| `free_navigation`                                                             | 碰撞倒车距离、最大恢复次数、恢复预算清零里程                                |
| `visual_goal_navigation`                                                      | 执行方式、完成规则文件、导航限制和执行参数                                  |
| `server`                                                                      | 监听地址、载具列表、默认载具、日志缓冲、CORS、视频帧率                      |
| `target_detection` / `trajectory_planner` / `task_completion` / `task_parser` | 四个模型端点                                                                |

### 视觉目标导航配置

下表列出 `vlm` 档使用的字段和值。字段路径以 `visual_goal_navigation` 为根。

| 字段                             | `vlm` 的值                          | 用途                                                                       |
| -------------------------------- | ----------------------------------- | -------------------------------------------------------------------------- |
| `execution`                      | `bare`                              | `staged` 表示规划器执行，也是省略此项时的默认值。`bare` 表示仿真器路径跟随 |
| `bare.speed_mps`                 | 1.0 米/秒                           | 交给仿真器的跟随速度                                                       |
| `bare.path_timeout_s`            | 60 秒                               | 仿真器跟随单批航点的超时                                                   |
| `limits.maximum_planning_rounds` | 30 次                               | 单阶段轨迹模型调用上限，复用上一批航点不计入                               |
| `limits.maximum_task_duration_s` | 600 秒                              | 单个视觉目标阶段的时间预算                                                 |
| `completion_rules_file`          | `tasks/visual_goal_navigation.yaml` | 仿真器路径跟随使用文件中的 `radius_m` 判定到达，本档为 7 米                |

使用仿真器路径跟随时，`config/app.yml` 中的 `server.planner` 必须设为 `vlm`。两种方式的流程见[视觉目标导航的执行方式](ARCHITECTURE.md#视觉目标导航的执行方式)。

## 7 `config/simulators.yaml`

每个仿真器一段。`projectairsim` 段的 `argv` 用 YAML 锚点定义，`airsim` 段引用同一份，
两套仿真器因此共用一套 xpra 会话参数，测试断言两处相等，改任何一处前先看那条测试。

| 字段                                                | 含义                                 | 注意                                                                        |
| --------------------------------------------------- | ------------------------------------ | --------------------------------------------------------------------------- |
| `driver`                                            | 进程驱动名，目前只有 `xpra_start`    | 写成别的会被配置校验拒绝                                                    |
| `runtime`                                           | 这套仿真器的 Python 包               | 由注册表按地图的 `simulator` 字段解析                                       |
| `ports`                                             | 这套仿真器监听的端口                 | 键名同时是 `{ports.<键>}` 占位符；UE4 的 `rpc` 必须等于引擎 `ApiServerPort` |
| `probes`                                            | 就绪判据：`window` / `ports` / `rpc` | UE5 用前两个，UE4 三个都要                                                  |
| `child_argv`                                        | 引擎命令行模板                       | 端口与分辨率走占位符                                                        |
| `argv` / `stop_argv`                                | xpra 起停参数                        | 两套仿真器共用一份                                                          |
| `settings_file`                                     | 引擎 settings 文件                   | UE5 没有这一项；它同时是 UE4 的归属指纹                                     |
| `runtime_python`                                    | 跑任务子进程的解释器                 | 只有 UE4 有；没有这一项的仿真器用线程 worker                                |
| `host` / `vehicle_name` / `camera_id` / `timeout_s` | 连接数据                             | `vehicle_name` 留空会让控制类 RPC 静默失效                                  |
| `camera`                                            | 这套仿真器产的图像怎么解释           | 分辨率、视场角、挂点、深度解码，无人机挂点也在里面                          |
| `process`（文件顶部）                               | 进程托管                             | 显示号、运行时目录、启停超时、地图目录与标记文件                            |
| `res_x` / `res_y`                                   | 画面尺寸                             | 同时决定 Xvfb 分辨率与引擎窗口参数                                          |
| `abslog_dir`                                        | 引擎日志目录模板                     | 只对会写日志的构建有意义                                                    |

能力位不在这里：写 `capabilities` 会被配置校验拒绝，能力由端口推导（见架构文档）。

`argv` 里是 `xpra` 的绝对路径。发行版把它装在别处时改模板，不要在代码里加分支。

## 8 `config/maps.yaml`

```yaml
# 包根目录：下面每张图的 binary 都相对它拼接，换机器只改这一行。
root: /roxy/vln

default_map: cityenviron
maps:
  - id: cityenviron # 控制台与 API 使用的地图标识
    simulator: projectairsim # 由哪个仿真器驱动，对应 simulators.yaml 的一段
    label: 城市 · 虚幻 5 # 控制台显示名，格式为「场景名 · 引擎」
    binary: projectairsim/CityEnviron/Binaries/Linux/CityEnviron-Linux-Shipping
    map_name: CityEnviron # 传给二进制的参数，窗口标题与它同字
    scene: # 这张图的场景身份，全部取自它加载的 scene JSONC
      file: scene_city_dual.jsonc
      config_path: config/scenes/city
      delay_after_load_s: 2.0
      rover_name: Rover1
      drone_name: Drone1
      front_camera: FrontCamera
      chase_camera: Chase
      down_camera: DownCamera
      uncapturable_views: [rover_chase]
      view_cycle_start: drone_front
      vehicle_scenes: # 非默认车型用哪份场景
        hexarotor: scene_city_dual_hexarotor.jsonc
```

- `root` 是仿真器包的根目录，`binary` 相对它拼接；写绝对路径则原样使用。换机器只改 `root` 一行。
- `binary` 同时是进程归属判据：停止时按 `/proc/<pid>/exe` 的真路径判断"这是不是我们的进程"。
- `map_name` 不一定等于目录名。Neighborhood 的目录叫 `Neighborhood`，交给引擎的地图名是 `AirSimNH`，这是引擎自己的命名。
- 地图引用的 `simulator` 必须在 `simulators.yaml` 里存在，否则配置加载报错。
- UE5 的图带 `scene` 段；UE4 的图只带 `binary` 与 `map_name`，场景与载具来自引擎自己的 settings。

## 9 `config/airsim_settings.json`

UE4 引擎读取的设置，规划器自己保存一份，不动引擎目录里的那份。

| 字段                             | 要求                                                           |
| -------------------------------- | -------------------------------------------------------------- |
| `SimMode`                        | `Multirotor`                                                   |
| `ApiServerPort`                  | 必须等于 `simulators.yaml` 的 `airsim.ports.rpc`；缺省是 41451 |
| `Vehicles.<名字>`                | 载具名必须等于 `simulators.yaml` 的 `airsim.vehicle_name`      |
| `Vehicles.<名字>.Cameras.<名字>` | 相机名必须等于 `airsim.camera_id`                              |
| `ImageType: 0`                   | RGB，分辨率与 `base.yaml` 的 `camera.width/height` 一致        |
| `ImageType: 2`                   | 深度，允许独立分辨率；几何用同一 FOV                           |

深度取 256×144 是传输决定的：`pixels_as_float=True` 把深度当 float 数组发回，纯 Python 的
msgpack 解包 36 864 个 float 约 0.24 秒，921 600 个要 25 秒以上。两个 ImageType 用同一个 FOV，
深度几何才能直接用 `depth_intrinsics` 反投射。

相机级覆盖写在 `Vehicles.<名字>.Cameras.<相机>.CaptureSettings`；写在 `CameraDefaults` 不会生效。

## 10 UE5 场景与机器人文件

`config/scenes/<map>/` 里每个场景一套：

| 文件                                             | 作用                                                             |
| ------------------------------------------------ | ---------------------------------------------------------------- |
| `scene_*_dual.jsonc`                             | 双载具场景：同时生成 Rover 与 Drone，时钟 `pause-on-start: true` |
| `robot_rover.jsonc` / `robot_drone_flight.jsonc` | 载具外观、相机安装、采集与流式开关                               |

约束：

- 两台载具出生点完全相同，同一时刻只显示一台。时钟在加载时暂停，无人机不会在加载等待里下坠。
- 恰好四个相机打开 `streaming-enabled`：无人车前视、无人车追尾、无人机前视、无人机追尾。
  服务端只能在它们之间循环。控制台不逐个列这四路，只区分第一人称与第三人称，
  按当前载具选对应的那一路。
- 无人车追尾相机 `capture-enabled: false`，标定工具不尝试对它取帧。
- 六旋翼需要单独的场景变体，写在该地图 `scene.vehicle_scenes` 的 `hexarotor` 项里。

## 11 模型端点与密钥

四段配置各有一个端点，都接受 OpenAI 兼容协议：

| 段                   | 用途                                                        | 超时 |
| -------------------- | ----------------------------------------------------------- | ---- |
| `task_parser`        | 自然语言 → 任务阶段                                         | 60 s |
| `target_detection`   | 视觉目标定位，给出框与表面点                                | 30 s |
| `trajectory_planner` | 规划器执行使用带航向的航点，仿真器路径跟随使用相对位移航点  | 60 s |
| `task_completion`    | 规划器执行时，到达后根据图像确认是否完成，只接受 true/false | 20 s |

两档在创建会话时都会创建 `task_completion` 客户端，因此都要填写该端点。仿真器路径跟随在运行时按距离判定到达。

- `request_format` 取 `openai`、`deepseek` 或 `anthropic`，决定请求字段与鉴权头。
  `anthropic` 只接受 `reasoning_effort: none`。
- `api_key` 直接写在配置里时优先于 `api_key_env`；留空则读环境变量，默认 `VLN_API_KEY`。
  填了真实密钥的配置文件不要提交到版本库。
- 端点不可用时任务被明确拒绝，服务本身照常启动、画面照常可看。拒绝信息里带状态码、 URL、
  该改哪个字段与 provider 原话。

## 12 启动、验证、关闭

### 启动

```bash
python scripts/serve_planner.py --host 0.0.0.0 --port 8000
```

- 服务启动时状态是 `disconnected`：不加载场景，不连仿真器，控制台没有画面。
- 启动时若发现上一轮由本项目拉起的仿真栈还在跑，会先停掉它。别人手工起的栈不动，这时画面可能还显示旧场景。
- 场景由控制台「启动场景」或 `POST /api/world` 拉起。
- 控制台的轨迹三维视图从 CDN 取 three.js r187，取库有 8 秒上限；取不到或浏览器没有 WebGL 时，
  整块隐藏，其余控件不受影响。

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

#### 普通 VLM 飞行探针

规划服务启动后，可在场景空闲或未启动时运行探针。脚本会按需启动场景，切换到 `vlm` 并提交指令。探针超时会请求停止任务，退出时会尝试关闭它启动的场景。

```bash
python scripts/probe_flight.py --host 127.0.0.1 --port 8000 \
  --instruction '飞到左前方那辆白色汽车旁边' --timeout 700 > run.jsonl
```

标准输出每秒记录一条 JSONL，摘要写入标准错误。位姿取自最近一次发布的轨迹快照，两个批次之间会重复。`distance` 是该快照的位姿到批次末航点的水平距离。任务是否到达以服务返回的阶段结果为准。

### 评测配置

命令行评测和控制台的评测仪表盘默认读取 `config/eval.yml`。换机器时按实际目录修改 `root`。`root` 的相对值以配置文件所在目录为基准。`root_path`、`dataset_root`、`metric_py` 分别指向场景包、数据集、评分脚本，相对值以 `root` 为基准。各字段也接受绝对路径。

`output_dir` 和 `saved_dir` 的相对值以仓库根为基准，默认使用 `eval/output` 和 `eval/saved`。主服务保存的评测记录与仪表盘读取的记录共用 `saved_dir`。

命令行可用 `--config` 指定另一份配置。`--root-path`、`--dataset-root` 和 `--output-dir` 优先于文件对应字段，相对值以调用时的工作目录为基准。仪表盘启动评测子进程时会传递配置文件和已解析的这三个路径。

从控制台启动评测时，主服务默认用 `simulators.yaml` 的 `airsim.runtime_python`。可用 `AVLN_EVAL_PYTHON` 覆盖解释器，用 `AVLN_EVAL_CONFIG` 指定配置文件，用 `AVLN_EVAL_PORT` 或 `AVLN_EVAL_SAVED` 覆盖端口或保存目录。

### 手动起停仿真栈

控制台之外的唯一入口，两套仿真器都是同一条命令：

```bash
PYTHONPATH=. python -c "
from pathlib import Path
from planner.config import load_config
from planner.simulator.router import FamilyRouter
cfg = load_config('config/core/default/base.yaml')
router = FamilyRouter(cfg.families, cfg.simulator, Path('.').resolve(), cfg.maps)
router.restart(cfg.maps.entry('cityenviron'))
print('running:', router.running())
"
```

停止把 `restart` 换成 `stop()`。已有图在跑时 `start` 会拒绝，这是有意的。画面地址是 `http://<主机>:10010/`。

### 关闭

`Ctrl-C` 停服务会带走本服务拉起的仿真栈：引擎、xpra 会话与 Xvfb 一起停。不属于本服务的栈只告警不动。

## 13 换机器要改什么

| 项                 | 位置                                                | 说明                                    |
| ------------------ | --------------------------------------------------- | --------------------------------------- |
| 仿真器包根目录     | `config/maps.yaml` 的 `root`                        | 一行改完下面所有 `binary`               |
| 单个引擎二进制     | `config/maps.yaml` 每条的 `binary`                  | 相对 root；`binary` 也是所有权判据      |
| UE4 子进程解释器   | `config/simulators.yaml` 的 `airsim.runtime_python` | 指向装了 `airsim` 的环境                |
| 引擎 settings      | `config/simulators.yaml` 的 `airsim.settings_file`  | 相对路径以仓库根为基准                  |
| 显示号             | `config/app.yml` 的 `process.display`               | 形如 `:104`                             |
| HTTP 监听          | `config/app.yml` 的 `server.host` / `server.port`   | 从别的机器访问时改 `0.0.0.0`            |
| 端口               | `config/simulators.yaml` 各套仿真器的 `ports`       | 改 UE4 的 `rpc` 时要同步改引擎 settings |
| xpra 路径          | `config/simulators.yaml` 的 `argv` / `stop_argv`    | 发行版装在别处才需要改                  |
| 模型端点与密钥     | `config/core/<profile>/base.yaml` 的四段            | 换 provider 时改 `request_format`       |
| UE4 载具名与相机名 | `airsim_settings.json` 与 `simulators.yaml`         | 两处必须一致                            |
| UE5 机位起点       | `config/maps.yaml` 每条的 `scene.view_cycle_start`  | 换了包或图之后需要重新确认              |
| GPU 选择           | 环境变量或引擎启动参数                              | 有坏卡时显式绑定健康卡                  |

`config/maps.yaml` 至少有 `default_map` 一张图；`default_map` 必须在列表里。

示例值（可直接照抄再替换）：

```yaml
# config/simulators.yaml · airsim.runtime_python
runtime_python: /home/<user>/miniconda3/envs/vln-ue4/bin/python

# config/maps.yaml
root: /roxy/vln
maps:
  - binary: projectairsim/CityEnviron/Binaries/Linux/CityEnviron-Linux-Shipping
  - binary: airsim/Blocks/LinuxNoEditor/Blocks/Binaries/Linux/Blocks
```

## 14 故障排查

| 现象                                                | 原因                                                                           | 处理                                                                                    |
| --------------------------------------------------- | ------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------- |
| 启动场景超时，且引擎一个字节日志都没有              | 显示号被残留的 X 锁占着，子进程根本没起来                                      | 检查 `/tmp/.X<n>-lock`；驱动只回收持有者已死的锁，别人的不删                            |
| `another ProjectAirSim connection is active`        | 同一进程里还有未关闭的连接                                                     | 会话关闭时用 `connection.close()`，不是 `client.disconnect()`                           |
| 取帧一直不返回                                      | 窗口没 map。seamless xpra 在客户端接上前不 map 窗口，引擎对未 map 窗口暂停渲染 | 驱动会 `xpra control map` + `focus`；不要用 `-RenderOffscreen` 绕，它把桌面画面一起去掉 |
| `TypeError: unexpected keyword argument 'encoding'` | `msgpack` 被 1.x 覆盖                                                          | 重建运行时环境，见 3.2                                                                  |
| 任务全被拒绝，日志提到 401/403                      | 模型端点凭据不对                                                               | 看日志里的状态码与字段提示；补 `VLN_API_KEY` 或配置 `api_key`                           |
| 换图返回 400                                        | 仿真器不是本服务拉起的，归属判据对不上                                         | 用 11 节的手动起停命令接管，或先停掉外部栈                                              |
| 控制台没有某些控件                                  | 当前地图的仿真器没有对应端口                                                   | 这是设计：能力由端口推导。`/api/status` 的 `capabilities` 可见                          |
| 三维块不出现                                        | CDN 取不到库或没有 WebGL                                                       | 只影响这块视图；需要离线时把库放本地，改 `scene3d.js` 里的地址                          |
| UE4 天气拖了没反应                                  | AirSim 特效默认关，参数要先启用才生效                                          | 适配器按状态开关；路面三项还要求地图带材质                                              |
| 接口返回 501 并带 `unsupported`                     | 当前仿真器没有实现这个动作背后的端口                                           | 同上，不是故障                                                                          |
| 无人机第二个任务贴地前飞                            | `landed_state` 落地后停在 Flying 不回 0                                        | 起飞判据已改为按地面高度比较；会话重建后仍复现时检查高度参考                            |
| 切换载具后车悬空                                    | 归位目标用了场景文件里的作者坐标                                               | 归位用暖机刹车沉降后实测的贴地姿态，日志里有 `Rover home pose measured`                 |
| 自由导航反复撞同一障碍                              | 没有车道参考，提示词只能靠画面判断                                             | 属已知限制，不是崩溃                                                                    |

## 15 运行目录与日志

| 路径                           | 内容                                                         |
| ------------------------------ | ------------------------------------------------------------ |
| `output/xpra/`                 | xpra 会话、pidfile、`current_map` 标记                       |
| `output/xpra/logs/`            | xpra 与引擎子进程的输出                                      |
| `output/logs/`                 | 引擎 `abslog` 目录                                           |
| `output/visual_goal/<run_id>/` | 由规划器执行的视觉任务记录，包括观测帧、深度、事件和控制遥测 |
| `output/calibration/`          | 机位标定截图                                                 |

服务日志进内存环形缓冲，通过 `GET /api/logs?cursor=N` 增量读取，进程退出即丢失。
Python 日志与 uvicorn 输出走 stderr。

`current_map` 只在拉起成功后写入。服务重启时读它恢复地图；内容不在目录里才回退到 `default_map`，
避免把城市地图的配置加载进郊区引擎。
