> 配套：[项目说明](../README.md) · [部署文档](DEPLOYMENT.md) · [开发文档](DEVELOPMENT.md)
> 层文档：[domain](layers/domain.md) · [ports](layers/ports.md) · [algorithms](layers/algorithms.md) · [runtime](layers/runtime.md) · [workflows](layers/workflows.md) · [assembly](layers/assembly.md) · [adapters](layers/adapters.md) · [simulator](layers/simulator.md) · [simulators](layers/simulators.md)

# 项目架构

一句话：把「开到前方那辆红色汽车旁」这样的中文指令，变成仿真里无人机或无人车的一系列运动命令，并在执行过程中持续看画面纠偏。

## 1 三个不变量

这三条决定了代码怎么分层，改代码时优先保它们。

1. **一条指令一类任务，任务不编排安全。** 三类任务（数值动作、视觉目标接近、无终点巡航）各自只决定「去哪」，碰撞退避由执行层被动触发，任务层不写退避逻辑。
2. **引擎只执行命令，不算曲线。** 采样、限幅、反馈律在执行层与计算层，换引擎不重写控制。
3. **能力由端口交集推导，不用开关。** 某类任务能不能跑，看任务包声明的必需端口是否都被引擎填满，没有 `enabled_tasks` 这种名单。

## 2 分层

```
            ┌──────────────────────────────────────────┐
            │ adapters    HTTP、模型客户端、提示词      │  入口
            └───────────────┬──────────────────────────┘
                            │
            ┌───────────────▼──────────────────────────┐
            │ assembly    装配、路由、能力求交、控制器   │  装配
            └───────────────┬──────────────────────────┘
                            │
            ┌───────────────▼──────────────────────────┐
            │ workflows   三类任务包 + 公共编排         │  任务
            └───────────────┬──────────────────────────┘
                            │
            ┌───────────────▼──────────────────────────┐
            │ runtime     控制循环、队列、滑窗、退避、汇 │  执行
            └───────────────┬──────────────────────────┘
                            │
   ┌────────────────────────▼─────────────────────────┐
   │ algorithms   几何、深度、校验、运动律、补点        │  计算
   └────────────────────────┬─────────────────────────┘
                            │
   ┌────────────────────────▼─────────────────────────┐
   │ domain 契约类型      ports 13 个 Protocol         │  契约
   └────────────────────────▲─────────────────────────┘
                            │ 实现
   ┌────────────────────────┴─────────────────────────┐
   │ simulators   UE5 projectairsim / UE4 airsim       │  引擎
   │ simulator    进程栈、子进程监督、worker 协议       │  进程
   └──────────────────────────────────────────────────┘
```

约束：只能向下依赖。跨层只能 import 层入口 `planner.<层>`，指向层内普通模块会被守门测试拒绝。

| 层 | 职责 | 文档 |
| --- | --- | --- |
| domain | 契约类型、常量、引擎能力位、运动模型 | [layers/domain.md](layers/domain.md) |
| ports | 13 个 Protocol 文件，引擎要兑现的全部能力 | [layers/ports.md](layers/ports.md) |
| algorithms | 纯计算：几何、深度、校验、运动律、补点 | [layers/algorithms.md](layers/algorithms.md) |
| runtime | 控制循环、队列跟随、滑窗、碰撞退避、各类汇 | [layers/runtime.md](layers/runtime.md) |
| workflows | 三类任务包、公共编排 `common/` | [layers/workflows.md](layers/workflows.md) |
| assembly | 装配、路由、需求求交、控制器六协作者、配置 | [layers/assembly.md](layers/assembly.md) |
| adapters | HTTP 端点、四个模型客户端、提示词模板 | [layers/adapters.md](layers/adapters.md) |
| simulator | 进程栈、子进程监督、worker 协议、xpra 驱动 | [layers/simulator.md](layers/simulator.md) |
| simulators | UE5 与 UE4 两套引擎实现 | [layers/simulators.md](layers/simulators.md) |

## 3 一次任务的全链路

以「开到前方那辆红色汽车旁」为例。

| # | 步骤 | 位置 |
| --- | --- | --- |
| 1 | 控制台把指令与载具名 POST 到 `/api/tasks` | `console/src/` |
| 2 | 端点校验参数，交给控制器 | `planner/adapters/http/server_api.py` |
| 3 | 控制器检查状态与能力位，拒绝则直接返回原因 | `planner/assembly/controller/` |
| 4 | 选可跑任务：指令解析成阶段序列 | `planner/workflows/task_parser` 客户端 → `assembly/requirements.py` |
| 5 | 取一张停机观测 | `planner/runtime/observation.py`、引擎观测端口 |
| 6 | 视觉定位目标，得到一个世界坐标与置信依据 | `planner/adapters/target_detection/`、`planner/algorithms/targeting/` |
| 7 | 规划器给出五个航点 | `planner/adapters/trajectory_planner/` |
| 8 | 航点过校验后进队列 | `planner/algorithms/checks/`、`planner/runtime/queue_follower.py` |
| 9 | 控制循环逐段执行，每 30 ms 采一拍 | `planner/runtime/motion/loop.py` |
| 10 | 执行中后台线程请求后续航点，队列不断流 | `planner/runtime/window/` |
| 11 | 碰撞时执行层停、退、清队列、继续 | `planner/runtime/collision_recovery.py` |
| 12 | 到达后调完成判定，返回结果给控制器 | `planner/workflows/visual_goal_navigation/`、`planner/adapters/task_completion/` |
| 13 | 控制器写状态、日志、轨迹，控制台轮询取走 | `planner/runtime/log_buffer.py`、`/api/status` |

## 4 三条主干路径

| 路径 | 起点 | 走法 | 结束条件 |
| --- | --- | --- | --- |
| 数值动作 | 「向前五米再左转九十度」 | 解析成段序列，逐段执行，段内 30 ms 采样 | 全部段完成或某段失败 |
| 视觉目标接近 | 「开到红车旁」 | 观测、定位、规划五点、逐段执行、边执行边重规划 | 到达阈值满足且完成判定为真 |
| 无终点巡航 | 「沿路一直开」 | 20 Hz 速度流，滑窗续规划 | 用户叫停或本地失败 |

三条路径都走同一个控制循环，差别在目标源：数值动作与目标接近用段队列，巡航用速度流。

## 5 两条命令通道

| 通道 | 频率 | 谁用 | 契约 |
| --- | --- | --- | --- |
| 段执行 | 30 ms 采样 | 数值动作、目标接近 | `Vehicle.execute_segment`，内部走 `stream_command(command, duration_s)` |
| 速度流 | 20 Hz | 巡航、滑窗补点 | `StreamingVehicle.stream_command` |

两条通道共用同一个命令类型 `MotionCommand`，分型为 `HolonomicCommand`（全向）与 `CarCommand`（阿克曼）。引擎侧用 `require_supported(command, capability)` 校验，不支持的组合直接报错而不是悄悄降级。

## 6 载具与能力位

| 载具 | 运动模型 | 能否横移 | 出现引擎 |
| --- | --- | --- | --- |
| 无人机 | HOLONOMIC | 能 | UE5、UE4 |
| 阿克曼无人车 | ACKERMANN | 不能，最小转弯半径约 3.2 m | 仅 UE5 |

能力位从端口槽位求交：`planner/assembly/requirements.py` 拿三个任务包各自声明的 `REQUIRED_PORTS`，与引擎已填槽位求子集，得到该引擎可跑的任务清单。UE4 填了流式端口，所以巡航自动开放，不需要白名单。

载具可在同一场景内热切换，不重启引擎。切换流程是原子的：停住、隐藏、复位、换机位、恢复位置保持，由 `simulators` 层实现，控制器不参与。

## 7 进程与线程

| 进程或线程 | 干什么 | 谁管 |
| --- | --- | --- |
| HTTP 服务进程 | 端点、控制器、任务调度 | `scripts/serve_planner.py` |
| xpra + Xvfb + UE | 渲染与仿真 | `planner/simulator/xpra_start.py` |
| 任务执行线程 | 跑控制循环，不阻塞 HTTP | `planner/runtime/` |
| 后台规划线程 | 滑窗续规划、目标刷新 | `planner/runtime/window/` |
| 模型调用线程 | 长耗时 HTTP 与图像编码 | `planner/runtime/` 的 `BackgroundCall` |
| UE4 任务会话子进程 | UE4 客户端依赖 msgpack 0.5.6，与主环境冲突 | `planner/simulator/` 的 worker 协议 |

进程栈由本项目拉起时才有所有权；别人起的栈不动。归属靠引擎二进制真实路径与 xpra 会话目录判定。

## 8 错误与预算

三类失败分开表达：

| 类别 | 类型 | 处理 |
| --- | --- | --- |
| 输入与协议 | `ProtocolError`、`ValueError` | 任务直接拒绝，端点返回 4xx |
| 执行与预算 | `ExecutionError`、`BudgetExceeded` | 任务中止，写日志与状态，载具停稳 |
| 观测与模型 | `ObservationError`、模型客户端错误 | 重试有限次，超限中止并给出原因 |

每阶段有时间预算，每次规划有模型调用预算，碰撞恢复有次数预算。预算在 `runtime` 统一扣减，任务层只读结果，不自己算钱。

## 9 设计取舍

以下是本套结构里最容易引起争论的六条，逐条给结论与理由。

1. **碰撞退避放执行层，不放任务层。** 三类任务都要退避，写在任务里就是三份镜像。放进控制循环后，任务只有在需要时才感知一次事件。
2. **引擎不实现控制律。** UE5 与 UE4 的动力学不同，但采样节奏、限幅、航向伺服与引擎无关。放在引擎里意味着接第三套引擎时要重写一遍。
3. **段执行与速度流两条通道并存。** 数值动作需要精确的时长与停稳判定，巡航需要连续的短周期下发。合并成一条会让其中一类的语义含糊。
4. **能力由端口交集推导。** 开关列表会漂移：加了端口忘了改名单，任务就白跑。交集是纯函数，加端口自动生效。
5. **视觉目标的连续路径不并入巡航的公共编排。** 视觉目标每次取帧对应一次确认与重规划，节拍与巡航不同。硬并会让节拍与遥测无法验证，收益是省几十行。
6. **契约层保留较大的导出面。** 设计时估 44 个名字，实测有 116 个被跨层引用。为了凑数字把调用点改到深层路径，会破坏「跨层只走层入口」这条更重要的约束。

## 10 目录与文件规模

| 层 | 文件 | 行数 |
| --- | --- | --- |
| domain | 25 | 2692 |
| ports | 13 | 291 |
| algorithms | 20 | 993 |
| runtime | 35 | 3386 |
| workflows | 23 | 2687 |
| assembly | 20 | 2560 |
| adapters | 44 | 2947 |
| simulator | 13 | 2503 |
| simulators | 32 | 5679 |
| 合计 | 225 | 25942 |

## 11 测试与守门

三层守门测试把架构钉住：`test_layering.py` 查依赖方向，`test_layer_entries.py` 查跨层 import 是否走入口，`test_public_surface.py` 查层入口导出面与层文档是否一致。

改动这三条规则覆盖的东西时，同步改实现、层入口、层文档三处，缺一处就红。裁决与重构过程留在 `docs/refactor/`，与当前代码冲突时以代码与本套文档为准。
