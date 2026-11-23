> 上级：[项目架构](../ARCHITECTURE.md)
> 同层兄弟：[domain](domain.md)、[ports](ports.md)、[algorithms](algorithms.md)、[runtime](runtime.md)、[workflows](workflows.md)、[assembly](assembly.md)、[simulator](simulator.md)、[simulators](simulators.md)

# adapters：HTTP 入口与模型客户端

## 1 一句话职责

本层是全项目唯一把 HTTP 请求、模型 wire 格式、提示词文本变成 `planner.domain` 契约对象的地方，也负责把装配层的拒绝翻译成状态码。

## 2 边界

管这些：

- 全部 FastAPI 路由、请求体模型、错误码映射、CORS 与缓存头、控制台静态文件挂载：`planner/adapters/http/server_api.py:41`、`planner/adapters/http/wiring.py:84`。
- 四个模型端口的实现：任务解析、轨迹规划、目标检测、任务完成判定。
- 模型 HTTP 传输与三种方言（`openai`/`deepseek`/`anthropic`）：`planner/adapters/multimodal_chat.py:45`、`planner/adapters/chat_request_options.py:6`。
- MJPEG 分帧：`planner/adapters/http/video.py:36`。
- 提示词模板加载与 `${name}` 占位符替换：`planner/adapters/prompt_render.py:14`、`planner/adapters/trajectory_planner/prompts/render.py:16`。
- 模型输入图像的校验：`planner/adapters/model_visual_inputs.py:32`。
- 落盘诊断 trace：`planner/adapters/diagnostics/visual_goal_trace.py:36`。

不管这些：

- 任务语义、运动律、控制权与控制率：见 [workflows](workflows.md)、[runtime](runtime.md)。
- 端口 Protocol 的定义：见 [ports](ports.md)。
- 能力位与任务需求表的推导：`planner/assembly/requirements.py:97`，见 [assembly](assembly.md)。
- 仿真器进程、xpra 会话：见 [simulator](simulator.md)。
- 引擎客户端调用：见 [simulators](simulators.md)。

## 3 输入与输出

### 入口

| 入口 | 位置 | 说明 |
| --- | --- | --- |
| `create_api(controller, settings, frames=None, trajectory_hub=None)` | `planner/adapters/http/server_api.py:41` | 唯一 HTTP 装配点，返回 `FastAPI` |
| `OpenAITaskParser.parse(instruction)` | `planner/adapters/task_parser/openai_task_parser.py:80` | 自然语言 → `MissionPlan` |
| `QwenVLPlanner.plan(observation, instruction, *, ...)` | `planner/adapters/trajectory_planner/qwen_vl_planner.py:36` | 观测 + 指令 → `RelativeTrajectory`（恰好 5 点） |
| `OpenAICompatibleTargetDetector.detect(request, *, timeout_s)` | `planner/adapters/target_detection/client.py:52` | 观测 → `TargetDetection` |
| `OpenAICompatibleTaskCompletionChecker.check(request, *, timeout_s)` | `planner/adapters/task_completion/client.py:21` | 完成证据 → `bool` |
| `MultimodalChatClient.request(system, text, images, *, timeout_s, operation, ...)` | `planner/adapters/multimodal_chat.py:94` | 裸文本响应，上面四个端口共用 |

### 契约类型

读：`planner.domain` 的 `Observation`、`WorldPose`、`ExecutionBudget`、`VisualGoalCompletionInput`、`VisualGoalPlanningContext`、`TargetDetectionInput`、`LocalizedGoal` 系列、`default_view_for`，以及 `planner.runtime` 的 `TrajectorySnapshot`。

写：`MissionPlan`/`MissionStage`、`RelativeTrajectory`、`TargetDetection`、`bool`、`SuccessResponse` 等 HTTP 响应体。

### 返回与异常

| 异常 | 抛出点 | 语义 |
| --- | --- | --- |
| `ProtocolError` | `multimodal_chat.py:97`、`openai_task_parser.py:104`、`response_schema.py:36`、`client.py:37`、`task_completion/client.py:26`、`trajectory_planner/response_parser.py:18` | 模型或线格式不合契约 |
| `BudgetExceeded` | `qwen_vl_planner.py:90-91`、`qwen_vl_planner.py:99-100` | 预算已耗尽 |
| `ValueError` | `image_encoding.py:15`、`chat_request_options.py:8`、`client.py:43`、`qwen_vl_planner.py:28-29` | 构造期参数非法 |
| `ObservationError` | `model_visual_inputs.py:20` | 深度图或内参不满足模型输入前提 |
| `KeyError` | `prompt_render.py:20` | 模板占位符没有取值 |
| `HTTPException` | `wiring.py:24`、`wiring.py:36`、`wiring.py:51`、`wiring.py:66` | 路由层状态码 |

### 端点清单（全部 17 条 + 静态页）

全部在 `planner/adapters/http/server_api.py`。请求体模型在 `planner/adapters/http/schemas.py`，全部 `extra="forbid"`。

| 方法 路径 | 行 | 请求体 | 返回 | 错误码 |
| --- | --- | --- | --- | --- |
| `POST /api/tasks` | :55 | `TaskRequest{instruction, vehicle?}` | 202 `{task_id, state, vehicle}` | 400 空指令、409/503 状态不允许、422 体非法 |
| `GET /api/status` | :64 | 无 | `controller.status` 全字段，有 caps 时加 `simulator`、`capabilities` | 无 |
| `GET /api/trajectory` | :73 | 无 | `TrajectorySnapshot`；`trajectory_hub is None` 时返回空快照 | 无 |
| `GET /api/logs?cursor=0` | :84 | 查询参数 `cursor>=0` | `controller.logs.read(cursor)` 全字段 | 422 `cursor<0` |
| `POST /api/stop` | :88 | 无 | 202 若 `STOPPING`，否则 200，体 `{state}` | 无 |
| `POST /api/home` | :94 | `HomeRequest{vehicle?}` 可空 | 202 `{state: "idle", vehicle}` | 501 缺 `home` 端口、409/503、400 |
| `POST /api/vehicle` | :101 | `VehicleRequest{vehicle}` | 202 `{state, vehicle, view_mode}` | 400 未知载具、409/503 |
| `GET /api/world/options` | :112 | 无 | `{maps, map_labels, default_map, selection, simulator_owned, map_simulators?, simulator_labels?, simulator?, capabilities?}` | 503 无地图目录 |
| `POST /api/world` | :139 | `WorldRequest{map}` | 202 若 `CONNECTING`，否则 200 | 400 未知地图、409/503 |
| `POST /api/world/stop` | :149 | 无 | 202 `{state}` | 400/409/503 |
| `GET /api/models` | :157 | 无 | `{models, options, switching}` | 501 缺 `models`、503 无车型目录 |
| `POST /api/models` | :176 | `ModelsRequest{models}` | 202/200 `{state, models}` | 501、400/409/503 |
| `GET /api/profile` | :189 | 无 | `{current, profiles[], switching}` | 无 |
| `POST /api/profile` | :209 | `ProfileRequest{profile}` | 202/200 `{state, profile}` | 400/409/503 |
| `GET /api/weather` | :217 | 无 | `{fields[], presets[]}` | 501 缺 `weather`、503 无目录 |
| `POST /api/weather` | :238 | 任意 JSON 对象（无模型校验） | 202 `{state, weather}` | 501、400/409/503 |
| `POST /api/view` | :247 | `ViewRequest{mode}`，`mode` 是 4 个字面量之一 | `{view_mode, deferred}` | 501 若控制器拒、422 `ValueError`、409/503 |
| `GET /api/video` | :255 | 无 | `multipart/x-mixed-replace` 流 | 仅在 `frames is not None` 时注册；缺 `video` 端口 501；未注册时 404 |
| `GET /` `/console/*` | `wiring.py:119` | 无 | `console/index.html` 与静态文件 | 目录不存在时整段不挂载 |

`POST /api/weather` 是唯一没有 Pydantic 模型的写端点（`server_api.py:239`），字段合法性全在控制器与引擎侧，见第 6 节。

## 4 对外名字表

| 名字 | 类型 | 一句话用途 | 层外调用点 |
| --- | --- | --- | --- |
| `create_api` | 函数 | 建 FastAPI 应用 | `planner/server_bootstrap.py:134` |
| `MultimodalChatClient` | 类 | 多模态 chat 传输 | `planner/assembly/factories/detectors.py:44`、`planner/assembly/factories/planners.py:48` |
| `OpenAITaskParser` | 类 | `TaskParser` 端口实现 | `planner/assembly/factories/parsers.py:34` |
| `QwenVLPlanner` | 类 | `TrajectoryPlanner` 端口实现 | `planner/assembly/factories/planners.py:57` |
| `parse_trajectory` | 函数 | 解析 `[[dx,dy,dz,dyaw]]` | `planner/adapters/trajectory_planner/__init__.py:4` 导出，测试直用 |
| `OpenAICompatibleTargetDetector` | 类 | `TargetDetector` 端口实现 | `planner/assembly/factories/detectors.py:53` |
| `ManualTargetDetector` | 类 | 人工框兜底检测器 | `planner/assembly/factories/detectors.py:36` |
| `parse_target_detection` | 函数 | 解析 `{bbox, surface_points}` | 同上文件导出 |
| `OpenAICompatibleTaskCompletionChecker` | 类 | `TaskCompletionChecker` 端口实现 | `planner/assembly/factories/completion.py:19` |
| `VisualGoalCompletionPrompt` | 类 | 完成判定的提示词 | `planner/assembly/factories/completion.py:24` |
| `VisualGoalTargetDetectionPrompt` | 类 | 检测的提示词 | `planner/adapters/target_detection/client.py:45` 默认值 |
| `VisualGoalTrace` | 类 | 落盘诊断 trace | `planner/assembly/factories/completion.py:40` |
| `FreeNavigationTrajectoryPrompt` | 类 | 自由导航提示词 | `planner/assembly/factories/planners.py` 提示词注入链 |
| `VisualGoalTrajectoryPrompt` | 类 | 视觉目标提示词 | 同上 |
| `BaseTrajectoryPrompt` | 类 | 默认提示词 | `planner/adapters/trajectory_planner/qwen_vl_planner.py:34` |
| `builtin_task_codecs` | 函数 | 载具对应的任务线格式集合 | `planner/adapters/task_parser/task_factory.py:12`、`prompt.py:61` |
| `generation_options` | 函数 | 生成参数按方言成型 | 本层内 |
| `describe_http_error` | 函数 | 把 httpx 失败压成一行可执行提示 | 本层内 |
| `mjpeg_chunks` | 异步生成器 | MJPEG 分帧 | `planner/adapters/http/server_api.py:261` |

## 5 关键流程

### 5.1 应用启动与关闭

1. `server_bootstrap.create_api` 传入 `controller`、`settings`、可选的 `frames`（帧 hub）与 `trajectory_hub`（`planner/server_bootstrap.py:134`）。
2. `build_app` 注册 lifespan：启动时 `controller.start()`，退出时 `run_in_threadpool(controller.close)`（`wiring.py:86-92`）。
3. 装 CORS（只允许 `GET`/`POST`，`wiring.py:96-102`）与缓存头中间件：`/api/*` 写 `no-store`，`/` 与 `/console/*` 写 `no-cache`（`wiring.py:107-115`）。
4. `capability_gate(controller)` 闭包 `require(action)`（`wiring.py:63-80`），后续路由用它拒绝不支持的端口。
5. `mount_console(app)` 在仓库根 `console/` 存在时挂 `/console` 与 `/`（`wiring.py:119-132`）。

### 5.2 一条指令变成 MissionPlan

1. `POST /api/tasks` 校验 `instruction` 非空（`schemas.py:24-30`），`invoke(controller.submit, ...)` 把校验后文本交给控制器（`server_api.py:57`）。
2. 解析器侧 `parse` 再校验一次非空（`openai_task_parser.py:80-81`）。
3. 组装请求头：`anthropic` 方言走 `anthropic_headers`，`openai`/`deepseek` 在 `api_key` 为空或 `"no-key"` 时不发 `Authorization`（`openai_task_parser.py:83-92`）。
4. `_request` 组 body：`{"model":…, **generation_options(...)}`，`anthropic` 用顶层 `system` 字段，其余用 `system` 角色消息（`openai_task_parser.py:107-120`）。
5. `httpx.post` 同步请求（`openai_task_parser.py:121-126`），`raise_for_status`，httpx 异常在 `parse` 里包成 `ProtocolError` 并附 `describe_http_error` 的一行（`openai_task_parser.py:100-106`）。
6. 取文本：`openai` 方言检查 `finish_reason` 与 `refusal`（`openai_task_parser.py:134-146`），`anthropic` 走 `anthropic_response_text`（`multimodal_chat.py:133-149`）。
7. `parse_json_object` 去代码围栏、拒绝重复键与非有限数（`response_schema.py:14-39`）。
8. `mission_from_dict` 校验信封与阶段字段、按 `codecs` 解码参数（`task_factory.py:8-54`）。

### 5.3 一次轨迹规划

1. `QwenVLPlanner.plan` 先校验 `pending_trajectory`（点数 ≤5、`|dyaw|≤180`，`qwen_vl_planner.py:52-58`）。
2. 有效超时 `min(self.timeout_s, timeout_s)`，再与预算取小（`qwen_vl_planner.py:59-61`、`:88-89`）。
3. `prompt_builder.system_prompt()` 与 `build_user_prompt` 生成两段正文（`qwen_vl_planner.py:69-84`）。
4. `aligned_depth_image` 校验 RGB/深度内参与尺寸（`model_visual_inputs.py:44-49`），再把 `(rgb, "JPEG")`、`(depth, "PNG")` 当图像序列（`qwen_vl_planner.py:85-86`）。
5. `transport.request(..., operation="trajectory planning")`，成功后若预算已过期则丢弃响应（`qwen_vl_planner.py:92-100`）。
6. `parse_trajectory` 用正则抓第一个 `[[...]]`，逐点校验 4 个数（`response_parser.py:15-40`）。
7. 点数必须恰好 5，否则 `ProtocolError`（`qwen_vl_planner.py:107-110`）。

### 5.4 提示词渲染

1. 轨迹类模板在**模块导入期**读取：`base.py:17`、`base.py:23`、`free_navigation.py:16-25`、`visual_goal_navigation.py:19-33`、`visual_goal_navigation.py:67-68`。
2. 检测类也在导入期读：`target_detection/prompts/system.py:4`、`visual_goal_target_detection.py:5`。
3. 完成判定类在**调用期**读：`visual_goal_completion.py:79-80` 才 `load_fragment("system")`。
4. 片段用 `load_fragment` 去掉文件末尾换行，分隔符由组合代码决定（`prompts/render.py:21-23`）。
5. 占位符只支持 `\$\{name\}`，缺值抛 `KeyError`（`prompt_render.py:11-23`）。

### 5.5 视频流

1. 只有构造时给了 `frames` 才注册 `/api/video`（`server_api.py:253`）。
2. `require("video")` 再按能力位拒一次（`server_api.py:258`）。
3. `mjpeg_chunks` 每轮 `frames.wait_next(sequence, timeout_s=0.0)`，没有新帧就 `asyncio.sleep(poll_s)`（`video.py:36-52`）。
4. 首帧一次性发出「分隔符 + 头 + JPEG + 尾分隔符 + 下一个头」，之后每帧发「JPEG + 尾分隔符 + 下一个头」（`video.py:57-70`）。
5. JPEG 编码在线程池里跑（`video.py:54-56`）。

## 6 失败模式

| 报错 | 触发条件 | 调用方怎么处理 |
| --- | --- | --- |
| `ProtocolError("task parser request failed: …")` | 模型端 4xx/5xx、连接失败、超时（`openai_task_parser.py:100-106`） | 任务失败并原样显示消息；这是运维唯一能读到的诊断 |
| `ProtocolError("task parser response was truncated or refused")` | `finish_reason` 非 `stop`/`None`，或 `message.refusal` 为真（`openai_task_parser.py:134-142`） | 同上，不重试 |
| `ProtocolError("mission rejected by task parser: …")` | 模型返回 `error` 且 `stages` 为空（`response_schema.py:59-61`） | 拒绝型结果，告诉用户需要澄清 |
| `ProtocolError("trajectory planner returned no JSON array")` | 响应里没有 `[[...]]`（`response_parser.py:17-18`） | 规划失败，工作流按预算决定是否重规划 |
| `BudgetExceeded("deadline before trajectory request")` | 预算在发请求前已耗尽（`qwen_vl_planner.py:90-91`） | 终止当前阶段，不再重试 |
| `BudgetExceeded("late trajectory response discarded")` | 响应回来时预算已过（`qwen_vl_planner.py:99-100`） | 丢弃本次结果 |
| `ProtocolError("task completion must return exactly true or false")` | 模型输出带标点、大小写不符、附解释（`task_completion/client.py:25-26`） | 判为失败，不回退到启发式 |
| `ValueError("model endpoint URL is empty")` | 配置里 url 为空（`multimodal_chat.py:21`、`:28`） | 装配期就该失败，见 `detectors.py:56-58` 的前置检查 |
| `ObservationError("model visual inputs require …")` | 深度图、深度内参、尺寸任一缺失或不符（`model_visual_inputs.py:40-49`） | 阶段失败，不发模型请求 |
| `KeyError("prompt template has no value for …")` | 模板有占位符但调用方没传（`prompt_render.py:19-20`） | 编程错误，本地测试应拦住 |
| `FileNotFoundError` | 模板文件缺失（`prompts/render.py:18`） | 轨迹类与检测类在导入期就炸，服务起不来；完成判定类在首次请求时才炸 |
| `HTTPException(503)` | 控制器拒绝且状态是 `CONNECTING`/`DISCONNECTED`（`wiring.py:26-33`） | 控制台提示「还没连上」 |
| `HTTPException(409)` | 控制器拒绝且状态可运行（`wiring.py:29-31`） | 控制台提示「当前状态不允许」 |
| `HTTPException(501)` | 活动仿真器 `Capabilities` 不含该 action（`wiring.py:69-79`） | 控制台按能力位隐藏按钮；直接调接口才看得到 |
| `HTTPException(503, "… catalog is unavailable")` | 控制器没带对应目录（`wiring.py:51-56`、`server_api.py:116`、`:162`、`:222`） | 视为部署缺件 |

`POST /api/weather` 没有请求体模型，非法字段名由 `planner.domain.validate_weather_values` 抛 `ValueError` → 400（`server_api.py:241`、`wiring.py:41-49`）。

### 6.1 已知缺口

同一个"不支持视频"有两种状态码。`/api/video` 只在构造参数 `frames` 非 `None` 时注册（`planner/adapters/http/server_api.py:253`），因此"这台仿真器不发帧"在 hub 缺席时是 404，在 hub 存在但能力位缺 `video` 时是 501（`planner/adapters/http/wiring.py:69-79`）。客户端不能只用一套分支判断，得先看路由表再看 `capabilities`。

能力位不可得时是**省略键**而不是 `null`：`/api/status` 只在 `caps is not None` 时写 `simulator` 与 `capabilities`（`server_api.py:66-71`），`/api/world/options` 同样（`:130-137`）。客户端必须区分"键不存在"与"值为空"，缺失只能解释为"当前地图还没有能力报告"。

`/api/trajectory` 在 `trajectory_hub is None` 时返回空 `TrajectorySnapshot` 而不是报错（`server_api.py:76-79`）。装配层漏传 hub 时控制台只剩空面板，没有任何错误，这个事故模式在子进程装配注释里有记录（`planner/simulator/mission_child.py:236-247`）。

提示词模板的读取时机有两种：轨迹类与检测类在**模块导入期**读（`planner/adapters/trajectory_planner/prompts/base.py:17`、`planner/adapters/target_detection/prompts/system.py:4`），缺文件则服务起不来；完成判定类在**首次请求**时读（`planner/adapters/task_completion/prompts/visual_goal_completion.py:79-80`），缺文件表现为运行期 500。文件缺失这一种故障因此有两种出现时机。

跨层依赖例外两条。本层 import 装配层的状态枚举与异常类（`planner/adapters/http/wiring.py:19`、`planner/adapters/http/server_api.py:19`），也 import 工作流层的校验函数（`planner/adapters/task_completion/prompts/visual_goal_completion.py:6`）。`tests/test_layering.py` 的检查器只把 `algorithms`（`:129-131`）与 `runtime`（`:140`）当来源层，`planner/adapters/` 从不在被检集合里，所以这条依赖目前没有任何测试拦得住。

## 7 测试位置

覆盖到的：

- `tests/test_server_api.py`（全 17 条路由，路由路径逐一断言在 293-1038 行区间）、`tests/test_capability_gating.py`（501 门与 `Capabilities` 组合）。
- `tests/test_video.py`（分帧格式、取消、编码失败）、`tests/test_http_errors.py`（401/404/503/连接失败）。
- `tests/test_batch3_chat.py`（三种方言的请求体、空 key、URL 补全）、`tests/test_batch3_services.py`（检测与完成判定的服务栈）。
- `tests/test_task_parser.py`、`tests/test_target_detection.py`、`tests/test_task_completion.py`、`tests/test_trajectory_parser.py`。
- `tests/test_trajectory_prompts.py`、`tests/test_prompt_render.py`（含 `prompt_render_baseline.json` 基线快照）。
- `tests/test_visual_goal_trace.py`、`tests/test_model_visual_inputs.py`、`tests/test_factories.py`。

没覆盖的空白：

- `/api/trajectory` 在 `trajectory_hub is None` 时的空快照分支（`server_api.py:76-79`）没有独立断言。
- `describe_http_error` 的 `_HINTS` 只测了 401/404/503，403/422/429 靠常量表读代码。
- `ManualTargetDetector` 在生产路径上的行为（`detectors.py:26-36`）只有单元测试，没有走 HTTP 的端到端断言。
- 模板文件缺失时的失败路径（`prompts/render.py:18`）没有测试。
- 模型端超时与 429 的重试行为没有测试，因为代码里也没有重试（见 Qad10）。

## 8 与其他层的联动

| 方向 | 对象 | 形态 |
| --- | --- | --- |
| 上游 | 装配层 `ServerController` | 通过 `capabilities()`、`active_simulator`、`simulator_labels`、`maps`、`profiles`、`weather_catalog`、`vehicle_models` 读状态；控制器方法返回值经 `invoke` 转状态码 |
| 上游 | `BackendUnavailable`、`ServerState` | 从 `planner.assembly` 直接 import（`wiring.py:19`、`server_api.py:19`） |
| 上游 | `planner.workflows.visual_goal_navigation` | `require_visual_completion_evidence` 直接 import（`task_completion/prompts/visual_goal_completion.py:6`） |
| 上游 | `planner.algorithms` | 提示词里算数值：`world_pose_delta_in_reference_frame`、`measure_free_space`、`point_in_horizontal_body`、`relative_to_world` |
| 下游 | `planner.domain` | 全部契约对象；本层不定义新契约类型 |
| 下游 | `planner.runtime.FrameHub` | 只读 `wait_next`/`closed`（`video.py:22`、`:38-45`） |
| 横切 | 模型 HTTP 端点 | 同步 httpx，无重试、无连接池配置 |

## 9 疑问与争议

- Qad1 层：adapters 结论：34 个轨迹与检测提示词模板在模块导入期读取，模板缺失时服务起不来但错误栈指向 import 而不是配置；完成判定的模板却是调用期读，同一层出现两种失败时机。依据：`planner/adapters/trajectory_planner/prompts/base.py:17`、`planner/adapters/target_detection/prompts/system.py:4`、`planner/adapters/task_completion/prompts/visual_goal_completion.py:79`。影响：否。
- Qad2 层：adapters 结论：本层 import 了上层 `planner.workflows`，`tests/test_layering.py` 的 D1-D5 没有约束 adapters 的向上依赖，所以这条依赖只靠人守。依据：`planner/adapters/task_completion/prompts/visual_goal_completion.py:6`。影响：是（workflows 文档需声明这是双向依赖）。
- Qad3 层：adapters 结论：HTTP 状态码判定直接依赖装配层的状态枚举与异常类，`adapters` 不能脱离 `assembly` 单独存在。依据：`planner/adapters/http/wiring.py:19`、`planner/adapters/http/server_api.py:19`。影响：是（assembly 文档需写清这是对外契约）。
- Qad4 层：adapters 结论：同一个「不支持视频」有两个状态码：`frames` 为 `None` 时路由不存在（404），能力位缺 `video` 时是 501，控制台无法用一套逻辑判断。依据：`planner/adapters/http/server_api.py:253`、`planner/adapters/http/wiring.py:69-79`。影响：是（assembly 的传参决定是 404 还是 501）。
- Qad5 层：adapters 结论：`ManualTargetDetector.detect` 完全忽略 `request`，任何任务都返回构造时那个框，能力位里也没有任何标记说明这是人工模式。依据：`planner/adapters/target_detection/manual.py:43-46`、`planner/assembly/factories/detectors.py:26-36`。影响：是（assembly 的 detector mode 需要暴露给状态接口）。
- Qad6 层：adapters 结论：`_chat_url` 与 `chat_completions_url` 重复实现，且空 URL 处理不一致：前者会拼出 `"/chat/completions"`，后者抛 `ValueError`。依据：`planner/adapters/task_parser/openai_task_parser.py:161-168`、`planner/adapters/multimodal_chat.py:19-25`。影响：否。
- Qad7 层：adapters 结论：两处 `generation_options(...)` 调用只为触发校验，返回值丢弃；非法 `request_format` 在构造期报错，运行时不会再检查。依据：`planner/adapters/task_parser/openai_task_parser.py:62`、`planner/adapters/multimodal_chat.py:53`。影响：否。
- Qad8 层：adapters 结论：地面与空中两套 base user prompt 是近 90 行逐行重复，唯一差异是模板名与 `vehicle="UAV"`。依据：`planner/adapters/trajectory_planner/prompts/base.py:37-124` 与 `:127-222`。影响：否。
- Qad9 层：adapters 结论：404 的运维提示假设用户 `url` 里带对了 path prefix，但 `chat_completions_url` 会自动补 `/chat/completions`，用 `anthropic` 方言配过 `url` 再切回 `openai` 时会真的 404，提示却不指向这条路。依据：`planner/adapters/http_errors.py:22-23`、`planner/adapters/multimodal_chat.py:19-33`。影响：否。
- Qad10 层：adapters 结论：四个模型端口全部单次请求、无重试、无退避、无超时重排；一次 429 或瞬时超时就是一次任务失败。依据：`planner/adapters/multimodal_chat.py:99-118`、`planner/adapters/task_completion/client.py:23-26`、`planner/adapters/trajectory_planner/qwen_vl_planner.py:92-98`。影响：否。
- Qad11 层：adapters 结论：`/api/status` 与 `/api/world/options` 在能力位不可得时是**省略键**而不是写 `null`，客户端必须区分「键不存在」与「值为空」。依据：`planner/adapters/http/server_api.py:66-71`、`:130-137`。影响：否。
- Qad12 层：adapters 结论：`/api/trajectory` 在装配层漏传 hub 时静默返回空快照，控制台只剩空面板且没有任何报错，这个事故模式在 `mission_child` 里已经发生过一次。依据：`planner/adapters/http/server_api.py:76-79`、`planner/simulator/mission_child.py:232-247`。影响：是（assembly 的 resources 构造）。
