# 开发约定

给在本仓库继续开发的 agent 和人类。读完这一份再动手。

## 1 先读什么

| 顺序 | 文件                   | 拿什么                                           |
| ---- | ---------------------- | ------------------------------------------------ |
| 1    | `docs/ARCHITECTURE.md` | 分层、模块职责、能力从哪来。改结构前必读         |
| 2    | `docs/DEPLOYMENT.md`   | 配置字段归属、环境、换机器要动什么。上机前必读   |
| 3    | 本文                   | 约定与扩展步骤                                   |
| 4    | `docs/TODO.md`         | 现在缺什么，动手前看一眼避免撞车                 |
| 5    | `docs/archive/`        | 查历史：某个设计为什么变成现在这样、某个坑的根因 |

`docs/archive/` 里是历史记录，不是当前行为说明。与当前代码冲突时以代码为准。

## 2 约定

下面是这个仓库长期成立的几条约束。它们不是偏好，改之前先看它们保护的是什么。

**能力由实现决定。** 一台引擎能做什么，取决于它实现了哪些接口。配置里不写能力，写了会被校验拒绝。
要新增能力就实现接口，而不是加开关。

**装配只有一处。** 任务管线与仿真器是并列的中间件，彼此不构造对方。谁需要什么，由装配层一次性接好。
少接一根线要在构造期就报错。

**一个事实只留一个归属。** 同一个事实被两处定义或校验时，删掉后加的那处。配置字段、接口名字、文档描述都算。

**未完成的路径显式失败。** 没有实现的功能直接报错并说明缺什么，不做静默降级，也不写没人调用的桩。

**扩展靠加数据，不靠加分支。** 新任务、新地图、新引擎各自对应一张表里的一行或一个包。
出现按名字分叉的 `if` 时，先想能不能变成表里的一行。

**以代码为准。** 文档与实现冲突时，先确认哪个是对的，然后改另一侧。不要用文档去猜行为。

## 3 扩展配方

### 3.1 新增任务类型

以 `free_navigation` 为模板，七步，缺一步就在运行期报错而不是编译期。

1. **领域类型。** `planner/domain/mission.py`：`TaskKind` 加成员，`TaskParameters` 联合加上新的参数类型；
   参数类是 `domain/` 里的 frozen dataclass。
2. **解码器。** `planner/adapters/task_parser/tasks/<kind>.py`：写提示词规则与 `decode_*` 函数。
   规则文本进的是模型提示词，所以措辞要与实际可执行能力一致。
3. **注册解码器。** `planner/adapters/task_parser/tasks/__init__.py` 的 `builtin_task_codecs` 加一条。
   地面与空中载荷不同就各给一个解码器。
4. **工作流。** `planner/workflows/<kind>/workflow.py`，实现 `prepare / run / close`。
   `run` 必须返回带确认完成证据的 `StageResult`，否则 runner 判失败。
5. **阶段校验器。** `planner/capabilities/task_validation/<kind>.py`，
   在 `mission_assembly.build_vehicle_arm` 的 `stage_validators` 里注册。
6. **接线。** 同文件的 `workflows` 字典加一项；无界任务同时加进 `terminal_kinds`。
   提示词的"允许类型"来自 `router.supported_kinds`，接上工作流就自动生效。
7. **能力表。** `planner/simulator/ports.py` 的 `TASK_REQUIREMENTS` 加一行，写它必需哪些端口。
   未实测的仿真器在各自 `runtime.py` 的 `ENABLED_TASKS` 里单独控制。

测试：解码器（含非法载荷）、校验器、工作流、能力推导（缺端口时该类型消失）。

### 3.2 新增地图

**UE5（ProjectAirSim）**

1. `config/scenes/<目录>/` 放场景与机器人文件：`scene_*_dual.jsonc`、`robot_rover.jsonc`、`robot_drone_*.jsonc`。
2. 场景约束：两台载具出生点完全相同、`clock.pause-on-start: true`、恰好四个相机 `streaming-enabled`、
   无人车追尾相机 `capture-enabled: false`。
3. `config/maps.yaml` 加一条：`simulator: projectairsim`、`binary`（相对顶部的 `root`）、`map_name`，
   以及 `scene:` 段：
   `file`、`config_path`、`delay_after_load_s`、载具名与相机 id、`uncapturable_views`、`view_cycle_start`。
4. 六旋翼等外观模型需要场景变体时，在这张图的 `scene.vehicle_scenes` 里加一行
   `hexarotor: scene_xxx_dual_hexarotor.jsonc`。车型清单在 `config/simulators.yaml` 对应的仿真器段里
   （`vehicle_models`，只有产出网格的仿真器才声明），场景文件属于地图。
5. 换包后跑一次 `scripts/calibrate_streaming_views.py`，把这张图的 `scene.view_cycle_start` 改成实测值。

**UE4（AirSim）**

1. `config/maps.yaml` 加一条：`simulator: airsim`、`binary`（相对 `root`）、`map_name`。UE4 没有场景文件，
   场景与载具来自引擎自己的 settings。
2. 确认 `config/airsim_settings.json` 的载具名、相机名、
   `ApiServerPort` 与 `config/simulators.yaml` 的 `airsim` 段一致。

### 3.3 新增仿真器

1. **实现包。** `planner/simulators/<name>/`：`runtime.py` 提供 `PORT_SLOTS`、`describe()`、`build_ports()`、
   `build_session()`。会话实现 worker 协议：`start / run / stop / poll / shutdown / terminate`，
   可选命令 `home / activate / set_weather / set_view`。
2. **注册。** `planner/simulator/registry.py` 的 `RUNTIME_MODULES` 加一行，`labels()` 加显示名。
3. **仿真器配置。** `config/simulators.yaml` 加一段：`driver: xpra_start`、`ports`、`probes`、
   `argv` / `stop_argv`、`child_argv`、`res_x` / `res_y`、`camera`（分辨率、视场角、挂点、深度解码）。
   有 `runtime_python` 就走子进程。显示号、运行时目录与起停超时写在文件顶部的 `process` 段。
4. **地图。** `config/maps.yaml` 里给这个仿真器的每条图填 `simulator: <name>`。
5. **测试。** `tests/test_architecture.py` 里 `registry.known()` 的断言要更新；
   再补 `build_ports` 与 `PORT_SLOTS` 一致的用例。

只在 `planner/simulators/<name>/` 里 import 客户端库。共享层与 `planner/simulator/` 都不许。

### 3.4 新增能力端口

1. `planner/ports/capabilities.py` 写 Protocol，方法数控制在必要范围。
2. `planner/simulator/ports.py`：`SimulatorPorts` 加字段（默认 `None`），`from_slots` 认这个槽。
3. 任务能力需求加到 `TASK_REQUIREMENTS`，各导航执行方式所需的端口加到 `PIPELINE_REQUIREMENTS`，控件映射加到 `ACTION_PORTS`。
4. 在仿真器包里实现，并加进那套仿真器的 `PORT_SLOTS`。
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
- 轨迹提示词在 `planner/adapters/trajectory_planner/prompts/`，`*_AERIAL_*` 是无人机版本。地面版本不谈升降，
  空中版本允许垂直运动。
- 仿真器路径跟随使用 `prompts/bare.py` 中的视觉目标轨迹提示词。`build_visual_goal_planner()` 装配提示词，`parse_translation_trajectory()` 解析 `[dx, dy, dz]` 航点。相关测试在 `tests/test_bare_prompts.py`、`tests/test_bare_planner_wiring.py` 和 `tests/test_trajectory_parser.py`。
- 地面任务指令带 `up_m` 时会在解码期被拒绝。仿真器路径跟随时，地面航点的 `dz = 0` 由提示词约定，解析器检查数组格式和数值是否有限。
- 改任务解析或规划器执行的轨迹提示词时，核对 `tests/test_trajectory_prompts.py` 与 `tests/test_task_parser.py`。

### 3.7 新增一份配置

- 一份配置 = `config/core/<目录名>/`，里面放完整的 `base.yaml` 和它自己的 `tasks/`。配置文件只写导航与控制
  参数与模型端点；程序级的 `server` / `process` 在 `config/app.yml`，仿真器相关的在
  `config/simulators.yaml`，地图与场景在 `config/maps.yaml`。写错文件会被配置校验拒绝；
  配置清单由目录枚举得到，加一份就是新建目录。
- `base.yaml` 顶部的 `profile` 段只有 `id`、`label`、`note`：`id` 等于目录名，`label` 是前端文案，
  `note` 给运维与日志用。
- 配置与配置只允许在导航与模型段上不同，`python scripts/check_profiles.py` 会检查这一点，
  测试里也有同一条断言。切换配置只重建 worker、重载场景，仿真器进程不动，所以程序级与
  仿真器级的设置本来就无法随配置生效。
- 共享的 `maps.yaml`、`simulators.yaml`、`scenes/` 只写一份在 `config/`；某份配置需要自己的副本时，
  放进它自己的目录里就优先。
- 新功能和新参数先进 `default`，演示对照组那份配置按需跟随。
- `visual_goal_navigation.execution` 选择视觉目标导航的执行方式。默认由规划器执行，「普通 VLM」档由仿真器跟随路径。具体取值见[部署文档](DEPLOYMENT.md#视觉目标导航配置)，装配测试见 `tests/test_pipeline_selection.py`。

## 4 测试与验收

```bash
python -m pytest -q                       # 底线是 0 failed，不是某个条数
python -m pytest -q tests/test_architecture.py
python scripts/probe_ue4_env.py
```

- **测试条数会涨。** 不要把某个数字当基线，会因此误判"少了几条就是坏了"。
- **结构改动必须有测试。** 一条规则不写下来就会被侵蚀，有测试守着的约定都写在 `tests/test_architecture.py`。
- **实机行为必须实机验证。** 前端与画面要用真实浏览器截图判断，不许靠读代码猜。需求是"点一下机位，
  画面与标签一致"，验证方式就是点一下看截屏。
- **单次验证控制在几十秒。** 不要跑几分钟的大循环。
- **不要重启用户正在运行的服务或仿真器。** 需要新代码生效时先说明，由用户决定。
- **实机验证。** `scripts/smoke_projectairsim*.py` 直连 UE5；`scripts/probe_flight.py` 通过规划器 API 验证 `vlm` 档，场景由服务管理。
- 一个仿真器实例只能有一个客户端。跑冒烟前确认规划服务没有连着。

## 5 提交与文档

- 提交信息格式：`type(scope): 中文短句`，正文 3 到 5 条要点，说清改了什么、原来是什么问题。
  `type` 用 `feat / fix / refactor / perf / docs / test / chore`。
- 一件事一个提交。回退单位是提交，不是开关。
- 每步结束跑测试，把结果贴出来。测试没跑就说没跑。
- 行为变了就同步文档，至少这三处之一：本文、`ARCHITECTURE.md`、`DEPLOYMENT.md`。文档是这个任务共享的记忆，
  改代码不改文档等于把结论丢掉。
- 发现历史结论被推翻，更新 `docs/archive/` 里对应的条目，并写清新结论的依据。

## 6 排错顺序

从便宜到贵，不要跳步：

1. `GET /api/status` 看 `state`、`map`、`sim_error`、`capabilities`。
2. `GET /api/logs?cursor=N` 看服务侧日志，换图与退出过程都在这里。
3. `output/xpra/logs/` 看 xpra 与引擎子进程输出。UE5 的两个构建不写日志文件，那里不会有。
4. `python scripts/probe_ue4_env.py` 看环境与二进制、端口、settings 的一致性。
5. `python -m pytest -q` 看是不是行为被改坏了。

现象与文档冲突时，以真机现象为准，然后改文档。

## 7 缺口

缺口只在 `docs/TODO.md` 维护，那里限定了条数和长度。本文不重复，避免两处各写一份、逐渐不一致。
