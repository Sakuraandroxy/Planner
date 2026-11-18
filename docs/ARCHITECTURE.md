# 架构

一个 HTTP 服务，通过统一端口驱动两个仿真器，把自然语言指令变成载具动作。本文讲清谁负责什么、边界在哪、能力从哪来。

## 1 三条不变量

设计只有三条规则，其余都是它们的推论。每条都有测试守着。

1. **并列件不构造并列件。** 规划器（模型端点适配）与仿真器是并列中间件。仿真器会话只交出端口，不接收检测器、
   规划器或遥测汇。它报 `mission_inputs()`，装配层还它 `attach_arms()`。
2. **只有装配层知道全部。** `application/mission_assembly.py` 是唯一同时认识仿真器与任务管线的地方。
   少接一根线在构造期报错，不静默降级。
3. **能力门只描述引擎能不能做这个动作。** 遥测、日志、任务列表、指令卡是服务自己提供的，不进端口包。
   否则一个通用功能会变成按仿真器授权的开关。

进程边界不是功能归属：UE4 的任务循环跑在子进程里，是因为它的客户端库不能在服务进程里加载，
不是因为 UE4 只能做更少的事。

## 2 分层

```
浏览器
  console/                                   静态页面，按能力位渲染控件
──────────────── HTTP 边界 ────────────────
  adapters/http/server_api.py                路由、状态码、能力门 → 501
  application/server_controller.py           状态机、命令队列、事件处理、换图/换模型/退出
  server_bootstrap.py                        装配：模型适配、帧汇、轨迹汇、controller
──────────────── 任务编排 ────────────────
  application/mission_assembly.py            唯一同时认识两边的装配器
  application/{application,mission_runner,mission_validator,workflow_router}
  workflows/{basic_motion,visual_goal_navigation,free_navigation}
  services/{execution,motion,continuous_execution,planning_window,trajectory_hub,frame_hub}
  capabilities/{planning,targeting,completion,approach,sliding_window,viewing}
──────────────── 外部适配 ────────────────
  adapters/{task_parser,target_detection,trajectory_planner,task_completion}
──────────────── 仿真器 ──────────────────
  simulator/{registry,router,xpra_start,processes,argv,marker,
             worker,subprocess_worker,mission_child,telemetry_relay,ports}
  simulators/projectairsim/                   UE5：连接、双载具会话、坐标、观测、相机、天气、机位
  simulators/airsim/                          UE4：RPC 连接、载具、观测、俯视相机、天气、会话
──────────────── 契约 ────────────────────
  domain/    纯数据，不依赖任何人
  ports/     接口协议，不依赖任何人
```

依赖方向只有一个：上层依赖下层，`domain/` 与 `ports/` 谁都不依赖。

## 3 目录职责

| 路径 | 职责 |
|---|---|
| `planner/domain/` | 位姿、任务、观测、轨迹、天气、机位、状态等纯数据与不变量校验 |
| `planner/ports/` | 载具、观测源、流式载具、相机、天气、机位、外观模型、帧流、解析器、检测器、完成判定、轨迹规划等协议 |
| `planner/services/` | 执行服务（运动段、停止检查点、预算）、同步与连续运动、滑窗、帧汇、轨迹汇、日志缓冲 |
| `planner/capabilities/` | 不碰设备的算法：深度几何、目标关联、完成判定、几何规划器、滑窗调度 |
| `planner/workflows/` | 三类任务的阶段实现与生命周期（prepare/run） |
| `planner/application/` | 解析→校验→路由→执行，会话装配，服务控制器 |
| `planner/adapters/` | 与推理服务、浏览器打交道；`http/` 是路由与 MJPEG |
| `planner/simulator/` | 两族共用的进程栈、注册表、worker、地图标记 |
| `planner/simulators/<name>/` | 每族一个包，只有这里有仿真器客户端库的 import |
| `console/` | 单页控制台，无构建步骤 |
| `config/` | 服务配置、地图目录、家族配置、场景、任务规则 |
| `scripts/` | 启动、自检、机位标定、实机冒烟 |

`planner/simulator/` 与 `planner/simulators/` 是两个不同的东西：前者是进程与协议，后者是每个引擎的适配层。

## 4 端口与能力推导

一个仿真器实现哪些端口，决定它能做什么、控制台显示哪些控件。配置里没有能力开关，
写了 `capabilities` 会被配置校验拒绝。

### 4.1 端口清单

| 端口 | 语义 | 必需 |
|---|---|---|
| `Vehicle` | 位姿、执行运动段、取消、悬停、碰撞状态 | 是 |
| `ObservationSource` | `capture()` 取一帧 RGB + 深度 | 是 |
| `StreamingVehicle` | 速度流与状态流，连续执行用 | 否 |
| `ControlledObservationSource` | 暂停轨迹后按需采一帧 | 否 |
| `MovingObservationSource` | 运动中取帧 | 否 |
| `CameraView` | 读写机体相机位姿（俯视） | 否 |
| `WeatherControl` | 应用并描述天气参数 | 否 |
| `StreamingViews` | 在固定相机之间循环机位 | 否 |
| `AppearanceModelControl` | 切换载具外观网格 | 否 |
| `FramePublisher` | 往帧汇发帧，`/api/video` 的门 | 否 |
| `ReturnHome` | 回出生点 | 否 |

`SimulatorPorts` 里还有两个非端口字段：
`vehicles`（这台引擎真的生成哪些载具）与 `enabled_tasks`（实测跑过哪些任务）。

### 4.2 任务需求与控件映射

`planner/simulator/ports.py` 的两张表是唯一的分支来源：

```python
TASK_REQUIREMENTS = {
    "basic_motion":            ("vehicle",),
    "visual_goal_navigation":  ("vehicle", "observation"),
    "free_navigation":         ("vehicle", "observation", "streaming"),
    "visual_goal_above":       ("vehicle", "observation", "camera_view"),
}

ACTION_PORTS = {"home": "home", "models": "models", "video": "frames", "weather": "weather"}
```

加一个任务就是加一行，不是加一个分支。

### 4.3 两族实现

| 端口 | projectairsim (UE5) | airsim (UE4) |
|---|---|---|
| `vehicle` / `observation` | 有 | 有 |
| `streaming` | 有 | 有 |
| `controlled_capture` | 有 | — |
| `moving_capture` | 有 | 有 |
| `camera_view` | — | 有 |
| `weather` | 有 | 有 |
| `views` | 有（4 个机位） | — |
| `models` | 有 | — |
| `frames` | 有 | — |
| `home` | 有 | — |
| `vehicles` | `rover`, `drone` | `drone` |
| `enabled_tasks` | 空（全部可跑） | `basic_motion`、`visual_goal_navigation`、`visual_goal_above` |

`enabled_tasks` 是验证状态，不是引擎上限。UE4 的 `free_navigation` 技术上是通的，但从没飞过，所以不在名单里。
空表示"不需要额外限制"。

### 4.4 能力矩阵

由上面两张表推导，`/api/status` 与 `/api/world/options` 输出推导结果：

| 输出 | 依据 | UE5 | UE4 |
|---|---|---|---|
| `task_kinds` | `TASK_REQUIREMENTS` ∩ 已实现端口 ∩ `enabled_tasks` | 数值动作、视觉目标、自由导航 | 数值动作、视觉目标、俯视 |
| `actions` | `ACTION_PORTS` | home、models、video、weather | weather |
| `vehicles` | `vehicles` | rover、drone | drone |
| `view_modes` | `views.view_modes()` | 四个 | 空 |
| `weather_*` | `weather` | 8 参数 + 预设 | 8 参数 + 预设 |

两点代码事实：

- UE5 的 `visual_goal_above` 因缺 `camera_view` 自动消失，不需要开关。
- UE4 的能力位宣告了 `visual_goal_above`，但共享层还没有这个 `TaskKind`，也没有对应工作流与解码器，
  所以它当前不可执行。`basic_motion` 与 `visual_goal_navigation` 已实测跑通。

## 5 任务管线

```
POST /api/tasks
  → ServerController.submit          仅 idle 接受，置 running
  → worker.run(TaskSubmission)       线程 worker 或子进程
  → 会话按 vehicle 路由到 VehicleArm
  → PlannerApplication
       parse    task_parser 模型 → mission_plan_v1
       validate MissionValidator：阶段类型、参数、"必须最后一段"等
       prepare  只准备计划里用到的 workflow
       run      MissionRunner：按顺序跑，首个失败即停
  → Workflow.run(stage)
       basic_motion           直接构造运动段执行，不做观测
       visual_goal_navigation 检测→规划→滑窗执行→到达→语义确认
       free_navigation        无界连续跟随，碰撞触发倒车重规划
  → StageResult（必须带 confirmed 完成证据，否则判失败）
```

解析是严格的：解析失败整单拒绝，不执行部分阶段。没有确定性回退。

工作流内部通过服务访问设备：`ExecutionService` 管运动段与停止检查点，
`SynchronizedMotionPlanner` 生成五次多项式段，滑窗服务管队列续规划，`TrajectoryHub` 管剩余航点快照，
`FrameHub` 管 MJPEG。

## 6 进程与生命周期

### 6.1 状态

```
disconnected → connecting → idle ⇄ running → stopping
```

`disconnected` 是启动状态：没有场景、没有画面，任务返回 503。换图、换模型、
退出都会经过 `connecting` 或 `stopping`，期间的提交按状态返回 503 或 409。

### 6.2 worker 两种形态

| 形态 | 谁用 | 说明 |
|---|---|---|
| `ThreadedMissionWorker` | UE5 | 会话跑在本进程的线程里，`projectairsim` 因此会被加载 |
| `SubprocessMissionWorker` | UE4 | 会话跑在 `runtime_python` 拉起的子进程里，事件走专用 fd，stdout/stderr 只当日志转发 |

两种 worker 的接口相同：`start / submit / stop / poll / shutdown / terminate` 加四个可选命令。
`ServerController` 不关心会话在哪一侧。

子进程拿不到父进程的 `TrajectoryHub`，用 `simulator/telemetry_relay.py` 里同接口的实现，
按 0.4 秒合并后经事件管道送回，父进程 `adopt` 进真汇。`/api/trajectory` 读的一直是同一个对象。

### 6.3 仿真栈

两族共用一个驱动 `simulator/xpra_start.py`，
它渲染 `config/simulators.yaml` 里的 argv 并执行 `xpra start --start-child`：引擎是 xpra 的子进程，
Xvfb 由 xpra 自己拉起。

| 环节 | 判据 |
|---|---|
| 就绪 | 家族声明探针：`window`（窗口 map 且归属对上）、`ports`（端口有人听）、`rpc`（UE4 专属） |
| 归属 | `/proc/<pid>/exe` 真路径在 `config/maps.yaml` 里；UE4 另需 argv 里的 `-settings=` 标记 |
| 停止 | 只停自己拉起过的进程树；先校验后动作，别人的栈只告警 |
| 显示 | 启动前回收持有者已死的 X 锁；别人的显示号拒绝 |
| 记录 | 拉起成功后写 `current_map`，服务重启据此恢复 |

`FamilyRouter` 决定"哪一族负责停、哪一族负责起"：按 marker 找到在跑的那一族先 stop，再交给目标家族 launch。

### 6.4 会话握手

`registry.build_session(simulator, ...)` 是唯一造会话的入口，它按地图的 `simulator` 字段解析包。流程：

1. 会话建立连接、加载场景、起载具
2. 会话报 `mission_inputs()`：载具字典、执行服务、观测源、移动取帧工厂、`enabled_tasks`
3. 注册表调 `assemble_missions(...)` 造 `{vehicle: VehicleArm}`
4. 会话收 `attach_arms(arms)`，缺任何一台载具的 arm 即报错

## 7 HTTP 接口

服务默认 `127.0.0.1:8000`，`/docs` 有完整 schema。

| 方法 | 路径 | 语义 |
|---|---|---|
| POST | `/api/tasks` | `{"instruction", "vehicle"}`；仅 idle，202 |
| GET | `/api/status` | 状态、指令、最近结果、载具、地图、机位、天气、`capabilities` |
| GET | `/api/logs?cursor=N` | 增量日志 |
| GET | `/api/trajectory` | 剩余航点、队列占用、里程、位姿；不设能力门 |
| POST | `/api/stop` | 停止当前任务；执行中 202，空闲 200 |
| POST | `/api/home` | `{"vehicle"}`，缺省当前载具；仅 idle，202 |
| POST | `/api/vehicle` | 停放另一台、选中载具归位、切尾视角；仅 idle，202 |
| GET | `/api/world/options` | 地图目录、标签、默认、当前、`simulator_owned`、能力位 |
| POST | `/api/world` | 起场景或换图；202 已接受，200 无变化 |
| POST | `/api/world/stop` | 停止当前场景并断开会话；仅 idle，202 |
| GET | `/api/models` | 每台载具当前模型、可选模型、切换中 |
| POST | `/api/models` | 提交部分模型变更；仅 idle，202 / 200 |
| GET | `/api/weather` | 8 个字段定义与预设 |
| POST | `/api/weather` | 任意子集，值 0–1；202 返回合并结果 |
| POST | `/api/view` | 切机位；运行中接受但 `deferred=true` |
| GET | `/api/video` | MJPEG；没有帧汇时不注册该路径 |
| GET | `/` `/console/*` | 控制台静态文件 |

状态码约定：

| 情况 | 状态码 |
|---|---|
| 请求体校验失败 | 422 |
| 未知载具、未知地图、模型或地图非法 | 400 |
| 运行中或停止中提交 | 409 |
| 连接中或未连接时提交 | 503 |
| 动作背后的端口不存在 | 501，`detail.unsupported` 与 `detail.simulator` |

## 8 控制台

单页应用，`console/index.html` + `app.js` + `styles.css`，没有打包步骤。

- 控件由 `capabilities` 决定是否出现，代码里不出现仿真器名字。
- 门控按卡片 id 点名，不按 DOM 嵌套查找：曾经用 `closest("section")` 找卡片，
  UE4 缺 `models` 能力时整张世界设置卡被藏起来。
- 状态轮询驱动界面：`connecting` 或 `view_pending` 时用不透明遮罩盖住画面，机位到位才显示。
- 画面是嵌入的 xpra iframe，默认端口 10010，可用 `?xpra=host:port` 覆盖。

## 9 约束由什么守着

`tests/test_architecture.py` 用 AST 扫全仓，检查四条规则：

| 规则 | 检查方式 |
|---|---|
| `simulators/**` 之外不许 import 仿真器客户端 | 遍历 import 节点 |
| 共享层不许 import 具体仿真器包 | 同上 |
| 服务进程不加载 AirSim 客户端栈 | 新解释器里导入主进程模块，看 `sys.modules` |
| 声明的端口槽有真实实现 | 比对 `PORT_SLOTS` 与 `build_ports`、协议成员 |

另有几条针对性测试：`test_family_router.py`（哪一族负责停）、
`test_airsim_port_names.py` （属性名与真实成员一致，不用整机启动才发现）、
`test_console_gating.py`（前端门控）、`test_scene_spawns.py`（场景出生点）、
`test_runtime_catalog.py` 与 `test_airsim_session.py` （声明的端口槽与实际填的槽一致）。

## 10 能力边界

这些是设计选择，不是待修缺陷：

| 边界 | 原因 |
|---|---|
| 一个仿真器实例只接一个客户端 | topic 链路是单消费者 |
| 一个引擎进程只加载一张 cooked 地图 | 引擎限制，因此换图 = 重启栈 |
| 视角只能循环固定相机 | 服务端不报告当前机位，也没有自由观察者相机 |
| 非活动载具被抬到 1 km 并钉住 | 客户端没有运行时删除机器人的接口 |
| UE5 不做自由观察者、UE4 不做帧流 | 端口不同，能力位自动反映 |
| `free_navigation` 没有完成条件 | 任务定义就是无界的 |

## 11 运动与航向控制

航向律只有一份：`planner/services/motion/heading_servo.py`。输入是偏航误差、实测偏航角速度与几个参数，
输出一个角速度。它不限幅到具体载具，也不认识仿真器。

三处调用对应三种运动阶段：

| 阶段 | 调用点 | 前馈 |
|---|---|---|
| 连续跟进 | `services/motion/continuous.py` | 路径给出的参考角速度 |
| 段式运动 | `simulators/projectairsim/motion.py`、`simulators/airsim/motion_tracker.py` | 参考轨迹自带的角速度 |
| 位置保持 | `services/position_hold.py` | 无 |

误差大时用刹车曲线收缩上限，误差与角速度同时小于死区时输出零。阻尼项乘的是**实测**角速度，不是位姿差分：
差分噪声会被负反馈直接放大成指令（见 `archive/measurements.md`）。

载具差异只在参数，而参数现在只有一份：`config/core/<profile>/base.yaml` 的 `motion` 段。
`heading_hold_gain` 只作用于位置保持，与飞行路径的 `yaw_gain` 分开：执行链延迟越大，保持环的增益就必须越低，
两者对应的对象不同。

实测角速度取自引擎真值：UE5 读 `twist.angular.z`，UE4 读 `KinematicsState.angular_velocity`，
都经 `TimedVehicleState.yaw_rate_deg_s` 进入服务层。字段缺失时为 `None`，伺服退回只依赖刹车曲线。

停稳观测也要等真的停稳。UE5 的 `capture()` 在取帧前先让位置保持确认 `observable`，UE4 用观测闸门做同一件事；
没有这一步，滑行途中拍的图片会被锚点校验判为过期。
| 模型端点是硬依赖 | 解析、检测、规划、完成判定都没有确定性回退 |