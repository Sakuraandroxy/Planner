# UE4 上机清单

分支：`feat/ue4-dual-runtime`。目标机：装了 UE4 AirSim 与 UE5 ProjectAirSim 的那台 Linux。

## 0 现在能做到哪一步

| 能力 | 状态 |
|---|---|
| UE5 全套（现有功能） | ok 完好，847 个测试通过 |
| 两族注册表、能力位、配置校验 | ok 已实现并测试 |
| 前端地图列表出现 6 张 AirSim 图 | ok 已实现 |
| 前端按端口推导的能力位隐藏控件 | ok 已实现 |
| AirSim 适配层接上共享端口 | ok 14 个模块，Vehicle 等四协议零缺失 |
| 俯视的支持层进入共享层 | ok 13 个文件，不再属于任何仿真器目录 |
| 两族共用一套 xpra argv 渲染 | ok 模板在 config/simulators.yaml |
| 切到 AirSim 地图 | no 返回 501，xpra_start 进程驱动未实现 |
| UE4 上飞行 | no 还缺会话装配与任务类型接线 |

也就是说：这份代码不会破坏你现在的 UE5 用法，UE4 部分停在"配置与前端就绪、执行链路未接"的位置。第 6 节列出剩余要写的东西。

## 1 建环境

两个环境，不要合成一个。UE4 的 `msgpack-rpc-python` 会拉一个老 `tornado`，UE5 的 `projectairsim` 依赖 `pynng` 与新 `msgpack`。旧分支专门写了 `planner/ue4/adapters/airsim/rpc_loop.py` 绕开 msgpack-rpc 0.4.1 与 Tornado 4.5 的循环重启问题，说明这条依赖链本来就脆。

主进程只加载契约与注册表，永不 import 两族客户端，所以两个环境可以共存于同一台机器。这条约束由 `tests/test_import_boundary.py` 守着。

```bash
# UE5 侧，也是 HTTP 主进程
conda create -n vln python=3.11 -y
conda activate vln
pip install numpy pyyaml httpx fastapi "uvicorn[standard]" pydantic \
  opencv-python-headless pytest
pip install /path/to/projectairsim-1.0.2-py3-none-any.whl

# UE4 侧，只给 UE4 运行时子进程用
conda create -n vln-ue4 python=3.11 -y
conda activate vln-ue4
pip install "airsim>=1.8.1" "msgpack-rpc-python>=0.4.1" "Pillow>=10.0.0" \
  "requests>=2.31.0" "numpy>=1.24.0" "PyYAML>=6.0"
```

`projectairsim` 不在 PyPI 上，必须用 wheel。版本号看 `pyproject.toml` 的注释，是 1.0.2。

## 2 改配置

### 2.1 `config/maps.yaml`

6 张 UE4 图的二进制路径按 flight-console 的原表写好：

```yaml
binary: /data/airsim/AirSimNH/LinuxNoEditor/AirSimNH/Binaries/Linux/AirSimNH
```

改这一处：如果你的二进制不装在 `/data/airsim/` 下，改每张图的 `binary`。`map_name` 是传给二进制的 argv[1]，保持与目录名一致。

### 2.2 `config/simulators.yaml`

| 字段 | 现在写的值 | 要确认什么 |
|---|---|---|
| `ue4.settings_file` | `/data/airsim/settings.json` | AirSim 那份带大量启动参数的配置，用 `-settings=` 原样传入 |
| `ue4.ports.rpc` | `41451` | 必须等于 settings.json 的 `ApiServerPort` |
| `ue4.ports.xpra` | `10010` | 与 `ue5.ports.xpra` 相同，两族共用一套显示与画面端口 |
| `ue5.ports.topics/services` | `8991` / `8992` | 保持现有代码的默认值，与 `config/base.yaml` 的 `projectairsim.port_*` 一致 |
| `ue4.argv` | `/usr/bin/xpra` 开头 | xpra 不在这个路径就改 argv |
| `ue4.res_x/res_y` | `1280` / `720` | 与 UE5 画面尺寸一致，改这里会同时改 Xvfb 分辨率 |

两族端口不统一：UE4 单 RPC 41451，UE5 双 socket 8991 与 8992。这是你的决定。同一时刻只跑一个仿真器，所以 xpra 端口共用不冲突。

### 2.3 `config/base.yaml`

UE5 侧不用改。要让别的机器的浏览器访问，启动时传 `--host 0.0.0.0`，不必改默认值 `127.0.0.1`。

### 2.4 `planner/ue4/config/base.yaml`（容易漏）

UE4 运行时读的是这份旧分支配置，它与主配置有三处不同，务必核对：

| 项 | UE5 主配置 | UE4 配置 |
|---|---|---|
| `request_format` | `anthropic` | `deepseek` |
| 任务解析 `url` | `https://api.deepseek.com/anthropic` | `https://api.deepseek.com` |
| 密钥环境变量 | 全部读 `VLN_API_KEY` | 按角色分：`TASK_PARSER_API_KEY`、`TRAJECTORY_PLANNER_API_KEY`、`TARGET_DETECTION_API_KEY`、`TASK_COMPLETION_API_KEY` |

所以 UE4 任务要单独导出这几个变量，只给 `VLN_API_KEY` 会认证失败。

同一份文件里的 `airsim.host` 留空表示用客户端默认地址，`airsim.port` 必须是 41451 或与 settings.json 一致。`airsim.camera_id` 是 `front_center`，必须与 settings.json 里无人机实际相机名一致——自检脚本会两边都读并比对。

## 3 上机顺序

```bash
git checkout feat/ue4-dual-runtime
conda activate vln
pip install -e .          # 或按第 1 节手工装
python scripts/probe_ue4_env.py
```

期望：`0 failed`。有 FAIL 就按提示补，不要继续。

```bash
PYTHONPATH=. python -m pytest -q          # 期望 847 passed
python scripts/serve_planner.py --host 0.0.0.0
```

浏览器开 `http://<机器IP>:8000/`，逐项确认：

1. 地图下拉里出现 8 项：2 张 ProjectAirSim 加 6 张 AirSim。
2. 选 `cityenviron` 启动，UE5 全流程与今天一致。
3. `GET /api/status` 的响应里出现 `simulator` 与 `capabilities`。
4. 选一张 AirSim 图点启动，界面显示 501 与 `driver:xpra_start`。这是预期行为。
5. UE5 地图下机位、模型、回起点、轨迹卡片都在。能力位由端口推导，改 YAML 无效；
   要验证门控真的在跑，从 `planner/simulators/projectairsim/runtime.py` 的
   `PORT_SLOTS` 里删掉 `models` 再重启，那张卡片应消失。

## 4 剩下的实现工作

按依赖顺序，每条都能独立提交：

| 步 | 内容 | 现有依据 | 可本地验证 |
|---|---|---|---|
| 1 | `planner/simulator/` 的 `xpra_start` 驱动：渲染 argv 模板、起进程、跑探针、写 pidfile | `airsim-flight-console/scene_launcher.py:33-75`、`scene_processes.py:180-200`、`environment_service.py` 状态机 | 可以，用假 runner 与假端口检查器 |
| 2 | UE4 运行时子进程入口与父侧 `BackendWorker` 包装 | 旧分支 `SubprocessPlannerWorker` 与 `server_worker.py` 已在 `planner/ue4/` | 部分可以，真连不行 |
| 3 | 旧版停止与接管链路一并搬进运行时 | 旧分支的这套逻辑在它自己的 `ServerController` 里（`README.md:428` 描述 `rotateByYawRateAsync` 交接），不在 worker 协议内 | 不行 |
| 4 | UE4 天气：移植 `weather_service.py`，字段名从注册表读 | `airsim-flight-console/weather_service.py` | 可以，假 RPC 服务 |
| 5 | 俯视任务在 UE5 上的启用判断 | 见第 5 节 | 需要上机 |

第 3 条是新发现，改变了工作量估计：旧分支的"停止任务"不是 worker 收到一条命令，而是控制器发起模式切换与接管。UE4 桥接层必须把旧 `ServerController` 的这部分一起带上，不能只包 worker。

## 5 确认 UE5 是否有相机位姿 API

UE5 俯视任务（`visual_goal_above`）当前被关掉，因为 `simSetCameraPose` 在 ProjectAirSim 里没有对应物。装了 `projectairsim` 的机器上跑一行确认：

```bash
python -c "from projectairsim import Drone, World; \
print('drone:', [m for m in dir(Drone) if 'camera' in m.lower() or 'sensor' in m.lower()]); \
print('world:', [m for m in dir(World) if 'camera' in m.lower() or 'pose' in m.lower() or 'object' in m.lower()])"
```

判读：

- `drone` 里出现相机位姿写入 → 按 UE4 同样方式实现 UE5 俯视，注册表 `ue5.capabilities.task_kinds` 加 `visual_goal_above`。
- 只有 `world` 侧的 `spawn_object` 与 `set_object_pose` → 走场景相机方案。
- 都没有 → UE5 继续不做俯视。

注意 `switch_streaming_view` 在官方 API 文档里没有列出但代码可用，说明文档不是全集，以 `dir()` 为准。
