# 开发约定

给在本仓库继续开发的 agent 和人类。读完这一份再动手，能避开上一轮踩过的所有结构性错误。

## 1 先读什么

| 顺序 | 文件 | 拿什么 |
|---|---|---|
| 1 | `docs/ARCHITECTURE.md` | 分层、端口、能力推导、进程模型。改结构前必读 |
| 2 | `docs/DEPLOYMENT.md` | 配置字段归属、环境、换机器要动什么。上机前必读 |
| 3 | 本文 | 约定与扩展步骤 |
| 4 | `docs/archive/` | 查历史：某个设计为什么变成现在这样、某个坑的根因 |

`docs/archive/` 里是历史记录，不是当前行为说明。与当前代码冲突时以代码为准。

## 2 不可违背的约定

每条都对应一次真实的返工。

1. **能力只能由端口推导。** 往 `config/simulators.yaml` 写 `capabilities` 会被配置校验拒绝。
   要加能力就实现端口，改 `planner/simulators/<name>/runtime.py` 的 `PORT_SLOTS`。
2. **不复活 `planner/ue4`。** 曾整树复制 12044 行，其中约 10600 行与主干逐字重复。
   共享逻辑进共享层，只有需要 import 仿真器客户端库的文件才进 `planner/simulators/`。
3. **不写无人调用的桩、开关、feature flag。** 未实现的路径显式报错，不静默降级。
   曾经有人写过一个没人调用的 `build_ports`，后来自己删掉了。
4. **一个事实只留一个归属。** 发现两处校验同一件事，删掉后加的那处。端口名、配置字段、
   文档描述都算。
5. **并列件不构造并列件。** 仿真器会话只交出端口，不构造检测器、规划器、遥测汇。
   装配只在 `application/mission_assembly.py` 里做。
6. **进程边界是部署事实，不是功能归属。** UE4 跑子进程是因为依赖冲突，不是因为它只能做更少的事。
7. **命名用软件名，不用引擎版本号。** `airsim` / `projectairsim`；地图字段叫 `simulator`。
8. **端口只回答"这台引擎能不能做"。** 遥测、日志、任务列表、指令卡是服务自己的功能，
   不进端口包。剩下的轨迹曾经被做成端口，结果前端在 UE4 上少了一个面板。
9. **前端门控按卡片 id 点名。** 曾用 `closest("section")` 按 DOM 嵌套找卡片，
   UE4 缺 `models` 能力时整张世界设置卡被藏起来。
10. **文档与代码冲突时，以代码为准并改文档。** 反过来用文档去改代码，等于用猜测覆盖实测。

## 3 扩展配方

### 3.1 新增任务类型

以 `free_navigation` 为模板，七步，缺一步就在运行期报错而不是编译期。

1. **领域类型。** `planner/domain/mission.py`：`TaskKind` 加成员，`TaskParameters` 联合加上
   新的参数类型；参数类是 `domain/` 里的 frozen dataclass。
2. **解码器。** `planner/adapters/task_parser/tasks/<kind>.py`：写提示词规则与 `decode_*`
   函数。规则文本进的是模型提示词，所以措辞要与实际可执行能力一致。
3. **注册解码器。** `planner/adapters/task_parser/tasks/__init__.py` 的
   `builtin_task_codecs` 加一条。地面与空中载荷不同就各给一个解码器。
4. **工作流。** `planner/workflows/<kind>/workflow.py`，实现 `prepare / run / close`。
   `run` 必须返回带确认完成证据的 `StageResult`，否则 runner 判失败。
5. **阶段校验器。** `planner/capabilities/task_validation/<kind>.py`，在
   `mission_assembly.build_vehicle_arm` 的 `stage_validators` 里注册。
6. **接线。** 同文件的 `workflows` 字典加一项；无界任务同时加进 `terminal_kinds`。
   提示词的"允许类型"来自 `router.supported_kinds`，接上工作流就自动生效。
7. **能力表。** `planner/simulator/ports.py` 的 `TASK_REQUIREMENTS` 加一行，写它必需哪些端口。
   未实测的仿真器在各自 `runtime.py` 的 `ENABLED_TASKS` 里单独控制。

测试：解码器（含非法载荷）、校验器、工作流、能力推导（缺端口时该类型消失）。

### 3.2 新增地图

**UE5（ProjectAirSim）**

1. `config/scenes/<目录>/` 放场景与机器人文件：`scene_*_dual.jsonc`、`robot_rover.jsonc`、
   `robot_drone_*.jsonc`。
2. 场景约束：两台载具出生点完全相同、`clock.pause-on-start: true`、恰好四个相机
   `streaming-enabled`、无人车追尾相机 `capture-enabled: false`。
3. `config/maps.yaml` 加一条：`simulator: projectairsim`、`binary`、`map_name`、
   `scene_file`、`sim_config_path`。
4. 六旋翼等外观模型需要场景变体时，在 `config/base.yaml` 的 `vehicle_models` 里为这张图
   指定 `scene_files`，否则换模型会失败。
5. 换包后跑一次 `scripts/calibrate_streaming_views.py`，把 `projectairsim.view_cycle_start`
   改成实测值。

**UE4（AirSim）**

1. `config/maps.yaml` 加一条：`simulator: airsim`、`binary`、`map_name`。UE4 没有场景文件，
   场景与载具来自引擎自己的 settings。
2. 确认 `config/airsim_settings.json` 的载具名、相机名、`ApiServerPort` 与
   `config/simulators.yaml` 的 `airsim` 段一致。

### 3.3 新增仿真器

1. **实现包。** `planner/simulators/<name>/`：`runtime.py` 提供 `PORT_SLOTS`、`describe()`、
   `build_ports()`、`build_session()`。会话实现 worker 协议：`start / run / stop / poll /
   shutdown / terminate`，可选命令 `home / activate / set_weather / set_view`。
2. **注册。** `planner/simulator/registry.py` 的 `RUNTIME_MODULES` 加一行，`labels()` 加显示名。
3. **家族配置。** `config/simulators.yaml` 加一段：`driver: xpra_start`、`ports`、`probes`、
   `argv` / `stop_argv`、`child_argv`、`res_x` / `res_y`。有 `runtime_python` 就走子进程。
4. **地图。** `config/maps.yaml` 里给这个仿真器的每条图填 `simulator: <name>`。
5. **测试。** `tests/test_architecture.py` 里 `registry.known()` 的断言要更新；再补
   `build_ports` 与 `PORT_SLOTS` 一致的用例。

只在 `planner/simulators/<name>/` 里 import 客户端库。共享层与 `planner/simulator/` 都不许。

### 3.4 新增能力端口

1. `planner/ports/capabilities.py` 写 Protocol，方法数控制在必要范围。
2. `planner/simulator/ports.py`：`SimulatorPorts` 加字段（默认 `None`），`from_slots` 认这个槽。
3. 需要暴露成控件就在 `ACTION_PORTS` 加映射；需要暴露成任务需求就加到 `TASK_REQUIREMENTS`。
4. 在仿真器包里实现，并把它加进该族的 `PORT_SLOTS`。
5. HTTP 端点用 `require("<action>")` 门控；控制台按 `capabilities` 决定控件。
6. 测试：`PORT_SLOTS` 与实际实现一致、缺端口时能力消失、501 的 `unsupported` 字段。

### 3.5 新增 HTTP 端点

1. `planner/adapters/http/server_api.py` 加处理器；引擎相关的动作用 `require(action)` 门控。
2. 状态字段加到 `domain/server.py` 的 `ServerStatus`，由 `ServerController.status` 填充。
3. 控制台加控件与轮询；按 `capabilities` 门控。
4. 补状态码测试（422 / 400 / 409 / 503 / 501）。
5. 把端点加进 `docs/ARCHITECTURE.md` 的接口表。

### 3.6 修改提示词

- 任务解析规则在 `planner/adapters/task_parser/tasks/*.py`，按载具区分地面与空中两套。
- 轨迹提示词在 `planner/adapters/trajectory_planner/prompts/`，`*_AERIAL_*` 是无人机版本。
  地面版本不谈升降，空中版本允许垂直运动。
- 执行侧检查在解码器里：地面载荷没有竖直字段，带 `up_m` 的包在解码期即拒。
  不要在送仿真器那一层事后发现。
- 改完跑 `tests/test_trajectory_prompts.py` 与 `tests/test_task_parser.py`。

## 4 测试与验收

```bash
python -m pytest -q                       # 底线是 0 failed，不是某个条数
python -m pytest -q tests/test_architecture.py
python scripts/probe_ue4_env.py
```

- **测试条数会涨。** 不要把某个数字当基线，会因此误判"少了几条就是坏了"。
- **结构改动必须有测试。** 一条规则不写下来就会被侵蚀，所以三条不变量都有对应测试。
- **实机行为必须实机验证。** 前端与画面要用真实浏览器截图判断，不许靠读代码猜。
  需求是"点一下机位，画面与标签一致"，验证方式就是点一下看截屏。
- **单次验证控制在几十秒。** 不要跑几分钟的大循环。
- **不要重启用户正在运行的服务或仿真器。** 需要新代码生效时先说明，由用户决定。
- **实机脚本只覆盖 UE5。** `scripts/smoke_projectairsim*.py`；UE4 的验证走控制台或直连会话。
- 一个仿真器实例只能有一个客户端。跑冒烟前确认规划服务没有连着。

## 5 提交与文档

- 提交信息格式：`type(scope): 中文短句`，正文 3 到 5 条要点，说清改了什么、原来是什么问题。
  `type` 用 `feat / fix / refactor / perf / docs / test / chore`。
- 一件事一个提交。回退单位是提交，不是开关。
- 每步结束跑测试，把结果贴出来。测试没跑就说没跑。
- 行为变了就同步文档，至少这三处之一：本文、`ARCHITECTURE.md`、`DEPLOYMENT.md`。
  文档是这个任务共享的记忆，改代码不改文档等于把结论丢掉。
- 发现历史结论被推翻，更新 `docs/archive/` 里对应的条目，并写清新结论的依据。

## 6 排错顺序

从便宜到贵，不要跳步：

1. `GET /api/status` 看 `state`、`map`、`sim_error`、`capabilities`。
2. `GET /api/logs?cursor=N` 看服务侧日志，换图与退出过程都在这里。
3. `output/xpra/logs/` 看 xpra 与引擎子进程输出。UE5 的两个构建不写日志文件，别指望有。
4. `python scripts/probe_ue4_env.py` 看环境与二进制、端口、settings 的一致性。
5. `python -m pytest -q` 看是不是行为被改坏了。

现象与文档冲突时，以真机现象为准，然后改文档。

## 7 当前缺口

按优先级排列，都是已确认的代码事实，不是猜测：

| 缺口 | 说明 |
|---|---|
| UE5 会话构造签名与新装配路径不一致 | `ProjectAirSimMissionSession.__init__` 仍要求 `detector`、`planner`、`geometry` 三个关键字参数，而 `runtime.build_session` 不再传它们，构造会抛 `TypeError`。装配改到 `mission_assembly` 后这三个参数已无用途，签名应删掉 |
| `visual_goal_above` 未接线 | UE4 的能力位宣告了它（`TASK_REQUIREMENTS` 与 `ENABLED_TASKS` 都有），但 `TaskKind` 没有这个成员，也没有解码器与工作流，实际不可执行。接线顺序见 `docs/archive/decisions.md` |
| UE4 没有 `/api/video` | 会话从不往帧汇写帧，端口缺 `frames`，端点返回 501 |
| UE4 没有 `/api/home` | 引擎支持（`moveToHome` / `reset`），代码没实现端口 |
| UE4 的 `free_navigation` 未实测 | 技术上可达，从没飞过，所以列在 `ENABLED_TASKS` 之外 |
| UE5 机位起点靠配置 | 运行时不做标定，换包或换图后需要重新确认 `view_cycle_start` |
| 「上升十米」端到端未验 | 解码与执行已改，缺一次带真实模型端点的完整链路验证 |
| 心跳重连未搬回 | 长时间 RPC 卡住未被覆盖；进程崩溃已由现有重启逻辑覆盖 |