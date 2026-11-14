# 接手提示词 · Planner 双仿真器

把本文件整段粘给新 agent 作为首条指令。方括号里的内容按实际情况替换。

---

你在接手一个已经改造完结构、但执行链路没接通的任务。你有我没有的东西：**真实机器**。
所以你最重要的价值不是继续推理代码，而是把纸面上的未知改成实测结论，再把剩下的三段
代码接通。任何你没法验证的说法，不要写成事实。

## 第 0 步：先跑，别先读长文档

```bash
cd [仓库路径] && git branch --show-current        # 应为 feat/ue4-dual-runtime
conda create -n vln python=3.11 -y && conda activate vln
pip install numpy pyyaml httpx fastapi "uvicorn[standard]" pydantic opencv-python-headless pytest
pip install [projectairsim-1.0.2 wheel 路径]
python -m pytest -q
```

期望 **0 failed**。达不到就先修，别继续往下走。然后把这条命令的**原始输出**贴出来。
具体条数会随功能涨跌（本轮从 851 到 883），不要把某个条数当基线。
基线不绿的时候做的任何改动都无法归因，这是硬门槛。

接着读四份文档，只读这些，按顺序：

| 文件 | 拿什么 |
|---|---|
| `docs/HANDOVER.md` | 全局状态、边界、提交含义 |
| `docs/UE4_REMAINING_WORK.md` | 你的活清单，含精确文件与行号 |
| `docs/UE4_BRINGUP.md` | 配置字段与环境 |
| `docs/PORT_BASED_REFACTOR.md` | 为什么是这个形状。改结构前读 |

不要读 `docs/PLAN.md`，那是历史实施记录，路径已过时。

## 第 1 步：把三个未知改成已知（先做这个）

这一步不写产品代码，只产出结论，但优先级最高，因为结论会改后面的做法。

```bash
# 1) UE5 到底有没有相机位姿写入 API
python -c "from projectairsim import Drone, World; \
print('drone:', [m for m in dir(Drone) if 'camera' in m.lower() or 'sensor' in m.lower()]); \
print('world:', [m for m in dir(World) if 'camera' in m.lower() or 'pose' in m.lower() or 'object' in m.lower()])"

# 2) UE4 环境是否齐备
python scripts/probe_ue4_env.py

# 3) 两套客户端能否共存一个解释器
python -m venv /tmp/coexist && /tmp/coexist/bin/pip install "airsim>=1.8.1" "msgpack-rpc-python>=0.4.1" pynng msgpack
# 不要只拿 import 判共存，它会给假绿灯；要跑到真正发包那一步：
/tmp/coexist/bin/python -c "import airsim; airsim.MultirotorClient().ping()"
```

判读：

- 第 1 条出现相机位姿写入 → UE5 可以做俯视，回来告诉我，这会把 `PORT_SLOTS` 加一项。
  没有 → 维持现状，UE5 不做俯视。（实测：`Drone.set_camera_pose` 存在，结论已进第 7 节；
  用户 2026-11-15 定了本轮不做俯视，先把整体程序跑起来，不要自行开工）
- 第 2 条的 FAIL 逐条列出，每条给修复命令。
- 第 3 条：`import` 全部成功不代表能共存。实测不能——`msgpackrpc` 用
  `msgpack.Packer(encoding=...)`，该参数在 msgpack 1.0 已删；混装下 `ping()` 报
  `TypeError`，隔离环境报 `TransportError`（序列化链路是通的，只是没起仿真器）。
  所以 UE4 必须跑独立环境，详见 `docs/ENVIRONMENT.md` 第 1 节。

结论写进 `docs/UE4_REMAINING_WORK.md` 第 7 节，替换掉"无法本地验证"里对应条目。
**文档是这个任务的共享记忆，改代码不改文档等于把结论丢掉。**

## 第 2 步：写 `xpra_start` 进程驱动（**已完成**，两族现在都走它）

UE4 的 shell 启动脚本与 `driver: script` 已经整列删除。这一节的验收标准已经满足，
留在文档里是为了下一个改动知道边界在哪：生命周期那套必须继续照
`airsim-flight-console` 写，不要自己发明。

- 落点 `planner/simulator/`。argv 模板与探针已经在 `config/simulators.yaml` 的 `airsim` 段，
  不要新写脚本文件，也不要在代码里硬编码命令。
- 抄的对象：`airsim-flight-console` 的 `scene_launcher.py:33-75`（argv）、
  `scene_processes.py:180-200`（`/proc/<pid>/exe` 加 argv 地图名判所有权）、
  `environment_service.py`（phase 状态机、超时、`instance_id` 防串话）。
- 必须保留现有行为：只停自己拉起过的进程，否则拒绝并打印运维命令。
- 非自己拉起的进程、端口被占、`settings.json` 的 `ApiServerPort` 与配置不一致，三种情况
  都要给出可直接执行的修复建议，不能只抛栈。

验收：

```bash
python -m pytest -q                       # 全绿，含你新增的用例
python -m pytest -q tests/test_architecture.py
python -m pytest -q tests/test_xpra_start_driver.py
```

新增用例必须覆盖：模板渲染、未解析占位符报错、所有权拒绝、探针超时。用假 runner 与假
端口检查器，这些不需要真机。

## 第 3 步：UE4 会话装配与天气

- 补 `planner/simulators/airsim/runtime.py` 的 `build_ports`。**同一个提交里必须有调用方**，
  否则就是无人使用的桩，上一个 agent 删过一次自己写的这个函数。
- 天气从 `airsim-flight-console/weather_service.py` 移植。字段拼写差异读同包
  `WEATHER_FIELDS`，不要写 if 分支。
- 装配时注意：心跳重连与停止接管的语义在旧 `ServerController` 里，不在 worker 协议里。
  旧 worker 的 IPC 只认 `task` 和 `shutdown`。只看 worker 接口会漏掉这两块。

验收：真机上 `POST /api/tasks {"instruction":"[一条最短的悬停指令]"}` 能起无人机并落回。

## 第 4 步：俯视任务端到端

顺序写在 `docs/UE4_REMAINING_WORK.md` 第 4 节，六步，按它做。两个坑：

- 提示词必须先按能力位过滤再拼，否则 UE5 会解析出跑不了的任务类型。
- 这一步改的 `task_parser` 与 `domain/mission.py` 是 UE5 共用的。改完**两侧都要实机跑一遍**。

## 第 5 步：裁决 7 个共享文件

清单第 2 节，只有 7 个，不是 48 个。重点是
`application/server_controller.py`（旧有 `_monitor`、`_reconnect`、`_retry_preparation`）
和 `services/position_hold.py`（旧有 `_wait`、`observation_ready`、`wait_for_observation`）。

每个文件在文档里那一行后面写结论：并入 / 不并入 + 一句理由。

---

## 硬规则

1. **能力位只能由端口推导。** 往 `config/simulators.yaml` 写 `capabilities` 会被配置校验
   拒绝，这是有意的。要改能力就改 `planner/simulators/<name>/runtime.py` 的 `PORT_SLOTS`。
2. **`planner/ue4/` 不许复活。** 曾整树复制过 12044 行并删掉。两版控制层实测逐字相同，
   复制的代价全花在格式差异上。共享逻辑进共享层，只有需要 import 客户端库的文件才进
   `planner/simulators/`。
3. **俯视是任务，不是 UE4 资产。** 任务逻辑在共享层，只有 `CameraView` 端口属于 AirSim。
4. **不写无人调用的桩、开关、feature flag。** 未实现的路径显式报错，不许静默降级。
5. **一个事实只留一个归属。** 发现两处校验同一件事，删掉后加的那处。
6. **命名用软件名不用引擎版本号**：`airsim` / `projectairsim`；地图字段叫 `simulator`。
7. **UE5 留在服务进程内。** `tests/test_architecture.py` 只禁 AirSim 栈进主进程。把 UE5
   搬进子进程是可选优化，不是待修 bug。
8. **已定的决策不要重开。** 清单第 6 节那九条边界是用户拍过的。你觉得不对，就说"我建议
   改 X，理由是实测 Y"，不要默默改。
9. **估工作量先测再报。** 本任务里"48 个文件要读"按"UE4 会失去什么"重测后是 7 个。
   先写脚本量化，再给数字。

## 工作方式

- 每步一个提交。提交信息说清"为什么"，不只说改了什么。
- 每步结束贴 `pytest -q` 的原始输出。测试没跑就说没跑，不要用"应该能过"。
- 卡住两次以上就停下来报告：现象、你试了什么、你判断缺哪块信息。不要绕过去加兼容分支。
- 改任何共享层文件前，先确认 UE5 侧有没有等价测试覆盖它。没有的话，先补测试再改。
- 真机现象与文档冲突时，**以真机为准并改文档**。

## 每轮结束按这个格式汇报

```
做了什么：
实测结论（含原始命令与输出）：
pytest：N passed / M failed（贴尾部，不要只说全绿）
改了哪些文件：
下一步建议：
需要我决定的：（没有就写"无"）
```

最后一条最重要：**这个任务里最贵的错误是把没验证的东西当成已知往下做。**
