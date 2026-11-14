# 上机清单 · 两个引擎一套服务

分支 `feat/ue4-dual-runtime`。目标机：装了 UE4 AirSim 与 UE5 ProjectAirSim 打包产物的
那台 Linux。本文只写**在这台机器上跑通过**的步骤；与任何旧文档冲突时以本文为准。

## 0 现在能做到哪一步

| 能力 | 状态 |
|---|---|
| UE5 全套（现有任务、双载具、机位、模型、天气） | ok 完好，启动与客户端握手已在这台机器实测（见 4.1）；任务级流程未变 |
| 两族注册表、能力位、配置校验 | ok 已实现并测试 |
| 前端地图列表 | ok 9 张：2 张 UE5 加 7 张 UE4 |
| 两族共用一个进程驱动 | ok `planner/simulator/xpra_start.py`，shell 启动脚本已删除 |
| UE4 起图 / 切图 / 停止 | ok 真机实测：blocks 5s、campus 8s、stop 1.4s 无 X 锁残留 |
| UE4 取帧与几何 | ok 真机实测：RGB 1280x720 + 米制深度 256x144，单帧中位 **0.40s** |
| UE4 解锁、起飞、平移、降落 | ok 真机实测（载具名接通后 `prepare_vehicle` 拿到确认；完成与否看回读状态） |
| UE4 上飞任务 | no 缺会话装配：还没人构造 `AirSimConnection` 与观测源（见第 5 节） |
| UE4 俯视任务端到端 | no 本轮不做（用户 2026-11-15 定的顺序：先把整体程序跑起来） |

缺的每一处都会显式报错，没有静默降级的路径。

## 1 环境

装什么、装到哪、踩过哪些坑，全部写在 `docs/ENVIRONMENT.md`。要点是三个事实：

- 两个 conda 环境：`vln`（HTTP 主进程 + UE5）与 `vln-ue4`（UE4 客户端）。
  这不是谨慎，是 `msgpack` 模块名撞车逼出来的，见 `ENVIRONMENT.md` 第 1 节。
- `projectairsim` 在 PyPI 上，`pip install --no-deps projectairsim==1.0.2` 再补基础依赖。
- 装 `airsim` 必须先装 numpy，并且用 `--no-deps --no-build-isolation`。

```bash
conda activate vln
cd /roxy/kedior/Planner
python -m pytest -q                      # 期望 0 failed（当前 883 passed）
python -m pytest -q tests/test_architecture.py
test ! -e planner/ue4 && echo "复制树未复活"
```

`python scripts/probe_ue4_env.py` 按角色判（`--role auto|service|runtime`）：
它看哪个包能导入就认定你站在服务环境还是子进程环境，两边实测都是 `0 failed`。
**不要为了让它变绿把 airsim 装进 `vln`**（会触发 `msgpack` 撞车），也不要把 fastapi 装进
`vln-ue4`（子进程不导入它）。

## 2 配置

三处，各有归属，别在第四处重复。

### 2.1 `config/maps.yaml`——每张图是什么、在哪

```yaml
- id: blocks
  simulator: airsim                 # 驱动这张图的适配层包
  label: AirSim Blocks
  binary: /data/airsim/Blocks/LinuxNoEditor/Blocks/Binaries/Linux/Blocks
  map_name: Blocks                  # 交给二进制的 argv[1]，窗口标题与它同字
```

UE5 的两张图除 `binary`/`map_name` 外还带 `scene_file` 与 `sim_config_path`（World 加载
场景用的 JSONC 与目录）。换机器时只需要改 `binary` 一处。

`map_name` 不总是等于目录名：neighborhood 的目录是 `Neighborhood`，交给引擎的地图名却是
`AirSimNH`，窗口标题也是 `AirSimNH`。这是 ProjectAirSim 自己的命名，不是笔误。

### 2.2 `config/simulators.yaml`——怎么连、怎么起停

每个家族一段，两族的 `driver` 都是 `xpra_start`。字段：

| 字段 | 含义 | 要确认什么 |
|---|---|---|
| `ports` | 家族监听的端口，键名同时是 `{ports.<键>}` 占位符 | `airsim.ports.rpc` 必须等于 settings.json 的 `ApiServerPort`（本机没有该字段，即默认 41451） |
| `probes` | 就绪判据：`window` / `ports` / `rpc` | UE4 三个都要；UE5 用 `window` 与 `ports`，它没有 RPC |
| `child_argv` | 引擎命令行的 argv 模板 | 端口与分辨率走占位符，不要写死 |
| `argv` / `stop_argv` | xpra 会话参数 | 两族用 YAML 锚点共用**同一份**，由测试断言两处相等 |
| `settings_file` | 引擎 settings（AirSim 专用） | UE5 没有这个字段，驱动据此不生成所有权指纹 |
| `res_x` / `res_y` | 画面尺寸 | 同时决定 Xvfb 分辨率与 `-ResX/-ResY` |

能力位不在这里：写了 `capabilities` 会被配置校验拒绝，能力由
`planner/simulators/<name>/runtime.py` 的 `PORT_SLOTS` 推导。

这个文件缺失时配置加载直接报错。以前它会合成一个走 shell 脚本的家族，那等于在 Python
里再抄一份命令行；两族现在都从这个文件读命令。

### 2.3 `config/base.yaml`——`simulator` 段

只剩与进程无关的东西：`display`、`runtime_dir`、`map_catalog_file`、`map_marker_file`、
三个超时与 `restart_attempts`。**端口不在这个文件里**（以前这里是第二份真相，
和家族段互相校验），UE4 运行时的模型端点与密钥见 `ENVIRONMENT.md` 第 2 节。

要让别的机器的浏览器访问，启动时传 `--host 0.0.0.0`，不必改默认值。

## 3 手动拉起与停止仿真栈

这是全仓库唯一一份命令。冒烟脚本与 README 都指到这里。
两族同一条：驱动读 `config/simulators.yaml` 渲染 argv。

```bash
cd /roxy/kedior/Planner && conda activate vln
PYTHONPATH=. python -c "
from pathlib import Path
from planner.config import load_config
from planner.simulator.router import FamilyRouter
cfg = load_config('config/base.yaml')
router = FamilyRouter(cfg.families, cfg.simulator, Path('.').resolve(), cfg.maps)
router.restart(cfg.maps.entry('cityenviron'))     # 或 blocks / campus / airsimnh …
print('running:', router.running())
"
```

停止：

```bash
PYTHONPATH=. python -c "
from pathlib import Path
from planner.config import load_config
from planner.simulator.router import FamilyRouter
cfg = load_config('config/base.yaml')
FamilyRouter(cfg.families, cfg.simulator, Path('.').resolve(), cfg.maps).stop()
"
```

要点：

- 换图用 `restart`，已有图在跑时 `start` 会拒绝——那是有意的，别改成静默切换。
- UE4 的取图与飞行要放在 `vln-ue4` 里跑，主进程按架构规则不许加载 AirSim 客户端。
- 画面：`http://<host>:10010/`。seamless xpra 在客户端接上之前不 map 窗口，UE 对未 map
  的窗口暂停渲染；`window` 探针会 `xpra control map` + `focus` 处理这件事。
  **不要**用 `-RenderOffscreen` 绕它：它把桌面画面一起去掉了。

## 4 上机顺序

```bash
conda activate vln
python -m pytest -q
python scripts/serve_planner.py --host 0.0.0.0
```

浏览器开 `http://<机器IP>:8000/`，逐项确认：

1. 地图下拉里出现 9 项：2 张 ProjectAirSim 加 7 张 AirSim（`/data/airsim` 下的
   七张都已配好路径，含本机自己打包的 campus）。
2. 选 `cityenviron` 启动，UE5 全流程与改造前一致。
3. `GET /api/status` 的响应里出现 `simulator` 与 `capabilities`。
4. 选一张 AirSim 图（`blocks` 最快）启动：能起、能取帧、能停。UE4 任务还飞不起来，
   原因见 `docs/UE4_REMAINING_WORK.md` 7.3，不是进程驱动的问题。
5. UE5 地图下机位、模型、回起点、轨迹卡片都在。要验证门控真的在跑，从
   `planner/simulators/projectairsim/runtime.py` 的 `PORT_SLOTS` 里删掉 `models`
   再重启，那张卡片应消失。

### 4.1 UE5 实测记录

发布配置经 `FamilyRouter` 跑完整周期（`vln`，同一条 `GetBuildCommitHash` 请求）：

| 动作 | 结果 |
|---|---|
| `restart(cityenviron)` | 7.8s 就绪 |
| `restart(neighborhood)` | 7.0s 就绪（切图走通） |
| 客户端握手 | 两图都应答，commit `deff6aa1` |
| `stop()` | 1.8s，端口释放，无 X 锁残留 |

ProjectAirSim 的 topic socket 是单消费者：一个进程里要连第二张图，必须先
`connection.close()`，直接调 `client.disconnect()` 会留下活动连接、下一张图报
`another ProjectAirSim connection is active`。

ProjectAirSim 的 CityEnviron 与 AirSimNH 构建**不写任何日志文件**，加了 `-log`
也不往 stdout 打，退出时同样不刷（引擎的 fd/1 就指着 `output/xpra/logs/xpra-start.<map>.log`，
里面只有 xpra 自己的话）。所以 UE5 的 `child_argv` 里没有 `-abslog`/`-Stdout`，就绪判断
只能靠 `window` 与 `ports`；驱动的超时提示按实际 argv 生成，不会指一个不存在的路径。

## 5 剩下要写的东西

| 项 | 位置 |
|---|---|
| 标定降级 | `streaming-view` 四机位标定在本机四帧打分完全相同，于是回落内置顺序（见 5.2）。不阻断任务，但循环顺序在这台机器上没被实测标定过 |
| 俯视与目标导航 | UE4 的目标导航已真机跑通（见 5.5）；UE5 俯视被用户明确推迟，见第 6 节与 `docs/UE4_REMAINING_WORK.md` 第 4 节 |
| 旧分支心跳重连 | 不必原样搬。实测任务中途 SIGKILL 子进程后服务自己重启了 worker；真需要验的是长时间 RPC 卡住这种它原本负贡的场景 |

会话装配、天气、进程驱动、控制面四处冲突、子进程入口、跨族切换、UE4 目标导航都已落地并真机验证。
实测数字与控制面冲突的证据在 `docs/UE4_REMAINING_WORK.md` 7.3 末尾的表里，跨族切换在 5.3，
目标导航在 5.5。

## 5.1 脚本化验 HTTP 服务时会假卡死的一个坑

`server_api.py` 在 **lifespan** 里启动控制器的监控线程。`TestClient(app)` 不进上下文
管理器就不会跑 lifespan：事件没人排空，服务一直停在 `connecting`，而底下的仿真器是好的。
我在这上面白花了一轮，差点把好好的重构当 bug 查。正确写法：

```python
with TestClient(build_server(load_config("config/base.yaml"))) as client:
    ...
```

## 5.2 UE5 与 UE4 的任务级实测（本轮）

同一句「无人机向前飞三米，然后向右转90度」，`vehicle=drone`。

| | UE5 cityenviron（经 `/api/world` + `/api/tasks`） | UE4 blocks（直接构造会话） |
|---|---|---|
| 解析 | stage_1、stage_2 basic_motion | 同样两阶段 |
| 执行 | 两阶段 `completed; basis=local_measurement`，39s | 位移 0→3.084m、偏航 0→89.81°，25.4s |
| 收尾 | `POST /api/stop` 200，状态 idle | `Vehicle on the ground: z=0.68 against the ground record 0.67` |

一句否定的实测值记下来：「起飞并向上爬升两米」被解析器拒绝
（`Unsupported actions: takeoff and vertical climb ...`）。basic_motion 在这套提示词下只支持
前/后飞与偏航；垂直起飞是会话的准备动作，不是任务阶段。

`vln-ue4` 需要 `httpx`（任务解析与轨迹端点都在 worker 侧发 HTTP）。

## 5.3 跨族切换实测（本轮）

一个服务、一段脚本，`POST /api/world` 连切三次，每次切完立刻下发一个飞行任务：

| 段 | 目标 | 切换+就绪 | 任务 |
|---|---|---|---|
| 1 | cityenviron（UE5） | 40s | 向前飞两米，16.1s 完成 |
| 2 | blocks（UE4） | 8s | 向前飞两米，16.1s 完成 |
| 3 | cityenviron（UE5） | 42s | 向后飞两米，16.1s 完成 |

UE5 就绪慢是引擎自己起图的时间；UE4 约 8s。这条路径上撞出三个缺陷，都在真机上复现并修完：

1. **跨族停止被归属关口挡住**。`holders()` 用 `owned_map()` 判断"这是不是我们的进程"，
   而 `owned_map()` 要求 argv 里有本族的 `-settings=` 标记。UE5 引擎没有这个标记，UE4
   驱动因此把还在跑的 UE5 栈判成不可管理的进程，拒绝停它。停止问的是"能不能停"，
   二进制在自家 `maps.yaml` 里就够了；"是哪一族的哪张图"才需要标记。两个问题拆成
   `catalogued_map()` 与 `owned_map()`。
2. **`FamilyRouter.restart` 把整次切换交给目标那一族的驱动**。UE4 驱动只读自己那套端口，
   UE5 栈不占 41451，于是它认为机器空闲、跳过 stop 直接 launch，然后卡在显示锁上。
   标记记着在跑的是哪一族，路由现在先让那一族 stop。
3. **`xpra stop` 不收它自己起的 X 服务器**。实测引擎与会话都死、端口都放，
   `Xvfb-for-Xpra-104` 还活着并占着 `/tmp/.X104-lock`，下次 launch 报
   "display :104 is still held by PID"。停止收尾现在验三件事再动手：属同一 uid、
   argv 里是这个显示号、名字是 Xvfb；别人的 X 服务器照旧只拒绝。

路由层的第一份测试是 `tests/test_family_router.py`。`FamilyRouter` 因此多了
`driver_factory` 参数——不然"哪一族负责停"这条规则只能靠真机验。

## 5.4 停止与退出的实际语义（测量）

这几条是读代码和量出来的，文档此前没写清：

- `POST /api/stop` 只中止任务（`worker.stop()`）。会话与仿真栈按设计留着，控制台上一次
  场景仍然可看。任务飞行中按下它：实测 UE4 在 **2s** 内回到 `idle`，阶段报
  `cancelled: task was cancelled`，无人机保持最后一个设定点。
- 停止只命名当下这份活：在飞的任务立即置取消事件；已提交还没出队的任务也会被取消；
  没有东西在飞时它被丢弃并记一行 `stop requested with nothing flying; ignored`，
  **不会顺延给下一条指令**（顺延过一次，后果是中止后的第一个任务开局就被取消）。
- 服务进程退出会关掉会话：子进程 worker 在约 1s 内退出。仿真栈仍留着。
- 任务进行中把子进程 `SIGKILL`：服务自己重启 worker 并回到 `idle`，之后切图与再飞都正常。
  旧分支那套 `ConnectionMonitor` 不必搬回——至少这条崩溃路径已被现有的重启逻辑盖住。
- 残留的仿真栈由**下一次启动**负责收：控制器启动时 preflight，认得出的就停掉
  （"Leftover simulator stopped; the console starts blank"），认不出的只告警不动。
- 子进程 `shutdown()` 现在会等到 CLOSED 为止（上限 `land_timeout_s + 10`）。之前它写完
  命令就返回，而 `ServerController._drain()` 只翻队列里已有的事件，于是每次关闭都记一条
  "Worker did not confirm shutdown within the timeout"，其实子进程收得干干净净。

## 5.5 UE4 目标导航（本轮）

「向前飞到那个红色集装箱前面」在 blocks 上 60s 完成：
`Stage stage_1: reached; basis=model_assessment_with_local_checks`，9 次 grounding 调用
（每次 1155–1402 输入 token），诊断帧落在 `output/visual_goal/<run_id>/`。
`api.deepseek.com/anthropic` 能收图像输入——此前担心这台机器没有可用视觉模型，事实不是。

接通常撞了三个 AttributeError，病因是同一个：**抄旧分支的属性名，没对着装配好的机器核**。

| 现象 | 真因 | 处理 |
|---|---|---|
| `'AirSimVehicle' object has no attribute 'capture'` | 装配把 vehicle 当 observation_source 传给视觉工作流；UE5 载具自己会取图，所以这个约定从没露馅 | `build_vehicle_arm` 新增 `observation_source`，默认仍为 vehicle（UE5 一字未改），UE4 传 `AirSimObservationSource`（它既有 `capture` 也有 `capture_moving`，因此 moving 源也传它） |
| `'PositionHold' object has no attribute 'wait_for_observation'` | 共享服务里这个方法叫 `wait`，语义一致（tick 到整个窗口可观测） | 只改调用名，不另拄一套稳定判据 |
| `'PositionHold' object has no attribute 'observation_ready'` | 旧分支叫这个，现在叫 `observable` | 同上 |

这类错不该靠整机启动去发现。`tests/test_airsim_port_names.py` 用 ast 扫 UE4 适配层里对
`holding` 与 `observation_gate` 的属性访问，逐项比对两个服务的真实成员（含 `__init__` 赋值的
实例属性）。33 条，0.19 秒跑完。本轮三处都是真机跑一遍才看见的，下一轮不必再跑。

## 6 UE5 俯视

实测 `projectairsim` 有 `Drone.set_camera_pose`，所以"UE5 缺相机位姿写入"这个前提不成立。
但用户 2026-11-15 明确"先不做俯视"，`PORT_SLOTS` 保持现状。要开工时先改的是
`planner/simulators/projectairsim/runtime.py` 与提示词的能力过滤顺序，见
`docs/UE4_REMAINING_WORK.md` 第 4 节。
