# 交接文档 · Planner 双仿真器改造

读者：在 Linux 真机上接手本任务的 agent。
写于分支 `feat/ue4-dual-runtime`，13 个提交，测试 851 passed。

## 0 先读哪几份

| 文件 | 内容 | 什么时候读 |
|---|---|---|
| 本文件 | 状态、边界、迁移、下一步 | 先读 |
| `docs/UE4_REMAINING_WORK.md` | 脚本生成的剩余工作清单与已定边界 | 动手前必读 |
| `docs/PORT_BASED_REFACTOR.md` | 端口化架构设计与理由 | 改结构前读 |
| `docs/UE4_BRINGUP.md` | 环境、配置、上机步骤 | 装机时读 |
| `docs/PLANNER_UE4_PLAN.md` | 早期方案与 12 项决策原始记录。第 4 节架构已被取代 | 查决策出处时读 |
| `README.md` 的 HTTP API 节 | 接口表 | 改 API 时同步 |

## 1 任务是什么

原状态：两个仓库。`Planner` 主干是新版规划器（FastAPI 前后端一体、驱动 UE5 ProjectAirSim、能起停仿真器）。`Planner` 的 `low-altitude-level-view-visual-goal-navigation` 分支是旧版（驱动 UE4 AirSim，纯无人机，有俯视任务，不管仿真器进程）。前端另有独立仓库 `airsim-flight-console`（Flask，负责起停 UE4、天气、xpra 画面）。

目标：废弃 `airsim-flight-console`，本仓库同时提供前后端，一套前端能随时在 UE4 与 UE5 场景之间切换，后端按地图选实现。

最终采用的架构不是"给 UE4 做兼容分支"，而是按端口分组：共享层一套，两个仿真器是同深度的并列包，能力位由实现了哪些端口推导。理由与细节见 `docs/PORT_BASED_REFACTOR.md`。

## 2 现在到哪一步

| | |
|---|---|
| UE5 全部原有功能 | ok 未破坏，816 个基线测试保持通过 |
| 端口层与能力推导 | ok 已实现并测试 |
| 结构改造（删复制树、两包并列） | ok 完成 |
| xpra argv 统一 | ok 完成，UE5 也走渲染 |
| AirSim 适配层接共享类型 | ok 14 模块，四个协议零缺失 |
| 俯视支持层进共享层 | ok 13 文件 |
| 切到 AirSim 地图 | no 返回 501 `driver:xpra_start`，驱动未实现 |
| UE4 上飞任何任务 | no 还缺进程驱动与会话装配 |
| UE4 俯视任务端到端 | no 缺 `TaskKind` 与 task_parser 接线 |

一句话：形状已经对了，UE4 的执行链路还差三段。缺的每一处都会显式报错，没有静默降级的路径。

## 3 接手前必须知道的边界

`docs/UE4_REMAINING_WORK.md` 第 6 节是完整列表。最容易踩错的四条：

1. 能力位只能推导。往 `config/simulators.yaml` 写 `capabilities` 会被配置校验拒绝。想改能力，改 `planner/simulators/<name>/runtime.py` 的 `PORT_SLOTS`。
2. 俯视是任务，不是 UE4 资产。任务逻辑在共享层，只有 `CameraView` 端口属于 AirSim。别把 `workflows/visual_goal_above/` 放进仿真器目录。
3. 不写无人调用的桩。前一个 agent 删过一次自己写的 `build_ports`，因为它没有调用方。要加就先加调用方，同一次提交内。
4. UE5 留在服务进程内。`tests/test_architecture.py` 只禁 AirSim 栈进主进程，不禁 `projectairsim`。把 UE5 搬进子进程是可选优化，不是待修的 bug。

## 4 在 Linux 上把测试跑起来

Python 要 3.10 以上，代码用 `str | None` 语法。系统自带的 3.9 会语法报错。

```bash
git clone git@github.com:Sakuraandroxy/Planner.git   # 或进入已有克隆
cd Planner
git fetch origin
git checkout feat/ue4-dual-runtime

conda create -n vln python=3.11 -y && conda activate vln
pip install numpy pyyaml httpx fastapi "uvicorn[standard]" pydantic \
  opencv-python-headless pytest
pip install /path/to/projectairsim-1.0.2-py3-none-any.whl   # 不在 PyPI

python -m pytest -q      # 期望 851 passed
```

`projectairsim` 是硬依赖：`planner/simulators/projectairsim/` 在模块级 import 它，装不上就连测试收集都过不去。前一个 agent 在 macOS 上用一个仓库外的替身包 `/tmp/pstub` 加 `PYTHONPATH` 顶过去，替身没有进仓库，你也不需要它。

UE4 侧的依赖（`airsim`、`msgpack-rpc-python`）不装也能跑测试，因为适配层把 `import airsim` 都放在函数体内。真要跑 UE4 才需要第二个环境，命令见 `docs/UE4_BRINGUP.md` 第 1 节。

跑完顺手确认结构没被破坏：

```bash
python -m pytest -q tests/test_architecture.py    # 三条规则
test ! -e planner/ue4 && echo "复制树未复活"
```

## 5 提交历史

```
0ebd534 docs: 端口化重构的落地状态与未做事项
7f77ff1 refactor(simulator): argv 模板改由 YAML 驱动
5aa3d81 refactor(simulator): 两族共用一套 argv 渲染
cf2309d refactor: 删除 planner/ue4 平行应用，共享任务层并入主干
09e32f6 refactor(simulator): 能力位改由端口推导，YAML 不再声明
7098298 refactor(simulators): 仿真器适配层改为并列目录，端口层就位
397377e docs: UE4 上机清单与实施进度
d03cf77 feat(api): 未实现的进程驱动一律拒绝切图
ca2ed71 feat(scripts): UE4 上机环境自检
df7f3f3 feat(console): 前端按能力位渲染可选控件
5cb0565 feat(runtime): 声明式仿真器族注册表与能力位
fed3373 chore: 复制旧分支规划器到 planner/ue4      ← 被 cf2309d 删除
```

`fed3373` 与后续删除对是有意保留的历史，别 rebase 掉：git 把那次删除识别为重命名，13 个共享文件的来源因此可追溯。想压成一个提交也可以，但会丢掉逐文件来源。

相对 `main` 的总量：90 文件、4918 行新增，其中 `planner/` 代码 3129 行。

## 6 下一步做什么

按依赖顺序，每步一个提交，每步之后 `pytest -q` 必须回绿。

1. `planner/simulator/` 实现 `xpra_start` 驱动。模板与探针声明已在 `config/simulators.yaml` 的 `airsim` 段就位。抄 `airsim-flight-console` 的 `scene_launcher.py:33-75`、`scene_processes.py:180-200`、`environment_service.py` 状态机。用假 runner 与假端口检查器写测试，这部分不需要真机。
2. 补回 `planner/simulators/airsim/runtime.py` 的 `build_ports`，同一提交内接上调用方。天气从 `airsim-flight-console/weather_service.py` 移植，字段拼写差异读同包的 `WEATHER_FIELDS`。
3. 俯视任务端到端接线，六步顺序见 `docs/UE4_REMAINING_WORK.md` 第 4 节。这一步会改 UE5 共用的 `task_parser`，两侧都要验。
4. 裁决第 2 节那 7 个共享文件。重点是 `application/server_controller.py`（旧版有 `_monitor`、`_reconnect`、`_retry_preparation`，即心跳断线重连）与 `services/position_hold.py`（旧版有 `_wait`、`observation_ready`、`wait_for_observation`）。
5. 真机联调，顺序见 `docs/UE4_BRINGUP.md`。先跑 `python scripts/probe_ue4_env.py`，FAIL 清零再继续。

## 7 上一个 agent 改过的判断，别当铁律

| 它原本的做法 | 改成了什么 | 为什么 |
|---|---|---|
| 整树复制旧分支，12044 行 | 端口化，只搬 14 个适配模块加 13 个共享文件 | 复制是拿"逻辑保真"当理由做的，实测两版控制层逐字相同，复制的代价花在格式差异上 |
| 能力位写 YAML | 由端口推导 | 配置能对现实说谎 |
| 估"48 个文件要逐个读" | 7 个 | 口径换成"UE4 会失去什么"，而不是"文本有无差异" |
| UE5 也换成 flight-console 起停 | UE5 保持现有脚本，但走同一套 argv 渲染 | 无法本地验证就不动已验证可用的东西。你要的"两族统一"落在渲染路径上，不在换脚本上 |

如果这些判断在你手上被推翻，更新 `docs/UE4_REMAINING_WORK.md` 第 6 节，别只改代码。

## 8 已知的仓库外事实

- 远端只有 `origin`，即 `git@github.com:Sakuraandroxy/Planner.git`。旧分支在远端存在（`f93bbb4`），取旧文件不需要本地分支。
- `main` 与旧分支无共同祖先，是两个独立初始提交。比差异用 `git diff A B -- path` 可以，`git merge-base` 为空属正常。
- `docs/` 在旧分支上被 `.gitignore` 忽略，在本分支不忽略。若你在旧分支工作树里写文档，切过来后记得 `git add`。
- 本机（macOS）无 UE、无 `projectairsim` wheel，所以所有仿真器侧行为都未验证。真机是唯一判据。
