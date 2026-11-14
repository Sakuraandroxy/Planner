# 上机清单 · 两个引擎一套服务

分支 `feat/ue4-dual-runtime`。目标机：装了 UE4 AirSim 与 UE5 ProjectAirSim 打包产物的
那台 Linux。本文只写**在这台机器上跑通过**的步骤；与任何旧文档冲突时以本文为准。

## 0 现在能做到哪一步

| 能力 | 状态 |
|---|---|
| UE5 全套（现有任务、双载具、机位、模型、天气） | ok 完好，并且已在这台机器上实测（见 3.4） |
| 两族注册表、能力位、配置校验 | ok 已实现并测试 |
| 前端地图列表 | ok 9 张：2 张 UE5 加 7 张 UE4 |
| 两族共用一个进程驱动 | ok `planner/simulator/xpra_start.py`，shell 启动脚本已删除 |
| UE4 起图 / 切图 / 停止 | ok 真机实测：blocks 5s、campus 8s、stop 1.4s 无 X 锁残留 |
| UE4 取帧与位姿 | ok 真机实测，观测已对齐主干 `Observation` 契约 |
| UE4 上飞任务 | no 卡在控制面三处冲突，见 `docs/UE4_REMAINING_WORK.md` 7.3 |
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

`python scripts/probe_ue4_env.py` 在 `vln` 里必然报 1 个 FAIL（`airsim` 装在另一个环境），
这是设计使然，不要为了跑绿把 airsim 装进 `vln`。

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
| UE4 控制面三处冲突（载具名、动作完成判据、深度分辨率） | `docs/UE4_REMAINING_WORK.md` 7.3，修法顺序 2 → 3 → 1 |
| AirSim 会话装配 | `planner/simulators/airsim/runtime.py`，补回时同一提交内必须带调用方 |
| 心跳重连与停止接管 | 在旧分支的 `ServerController` 里，不在 worker 协议中；装配时只看 worker 接口会漏掉 |

## 6 UE5 俯视

实测 `projectairsim` 有 `Drone.set_camera_pose`，所以"UE5 缺相机位姿写入"这个前提不成立。
但用户 2026-11-15 明确"先不做俯视"，`PORT_SLOTS` 保持现状。要开工时先改的是
`planner/simulators/projectairsim/runtime.py` 与提示词的能力过滤顺序，见
`docs/UE4_REMAINING_WORK.md` 第 4 节。
