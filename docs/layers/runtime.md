> 上级：[项目架构](../ARCHITECTURE.md)
> 同层兄弟：[计算 algorithms](./algorithms.md)、[契约 domain](./domain.md)、[契约 ports](./ports.md)、[任务 workflows](./workflows.md)、[装配 assembly](./assembly.md)、[入口 adapters](./adapters.md)、[进程栈 simulator](./simulator.md)、[引擎 simulators](./simulators.md)

# runtime（执行层）

## 1 一句话职责

执行层是唯一碰载具与墙钟的一层：用控制权、两条命令通道、授权队列、在途模型调用与观察汇，把任务层给的轨迹和目标变成按节拍下发的载具命令，并在失败时留下 receipt。

## 2 边界

管什么
- 控制权与预算：`ExecutionService` 的 owner 断言、飞行持有、阶段 deadline、停止请求、停止动作。
- 段执行：`ControlLoop.run_segment` 的五次曲线采样、限幅、停稳验证、暂停与碰撞中断后的 receipt。
- 流执行：`ControlLoop.run_stream` 一拍、`QueueFollower` / `ContinuousExecution` 的队列消费与连续命令生成。
- 授权队列与滑窗：`WorldWaypointQueue` 的修订与消费、`PlanningWindow` 的补点与在途结果收集、`SlowPlannerWorker` 的模型线程。
- 碰撞退避：`CollisionRecovery` 的接触去重、后退距离递增、预算与复位。
- 位置保持：`PositionHold` 的到位窗口与可见性判定。
- 后台调用：`ModelCallService`、`BackgroundOperation` 的单飞模型/数据调用与超时隔离。
- 汇与日志：`FrameHub`、`TrajectoryHub`、`LogBuffer`、`StreamTimeline`、`ControlTelemetry`。
- 剩余轨迹遥测：`telemetry.py` 的快照类型、`build_queue_snapshot` 与跨进程 payload 编解码。

不管什么
- 任务是否完成、何时暂停、补点授权与否：见 [workflows](./workflows.md)。
- 引擎 API 与坐标原点、碰撞读数的实际来源：见 [simulators](./simulators.md)、[adapters](./adapters.md)。
- 契约类型与端口：见 [domain](./domain.md)、[ports](./ports.md)。
- 装配与配置：见 [assembly](./assembly.md)。

依赖实测：`planner/runtime/` 无一条 import 指向 `workflows`、`assembly`、`adapters`、`simulators`。复核命令：

```bash
grep -rn "planner.workflows\|planner.assembly\|planner.adapters\|planner.simulators" planner/runtime/ || echo clean
```

## 3 输入与输出

入口 `planner/runtime/__init__.py`，导出 37 个名字（`__all__` 在 planner/runtime/__init__.py:40-77）。

典型输入：`MotionTrajectory` / `MotionSegment`（段执行）、`StreamGoalSource`（流执行的目标源）、`WorldTrajectory`（授权批量）、`PlanningSnapshot`（模型请求快照）、`MovingObservationSource`（移动帧源）、`CollisionState`、`ExecutionBudget`。

典型返回：`ExecutionReceipt`（完成段数、最终位姿、耗时、实测弦长、是否暂停、剩余轨迹、实测动作）、`ExecutionRecord`、`StopResult`、`TrajectorySnapshot`、`LogBatch`、`PlanningSnapshot` 对应的 `ContinuationCandidate`，或本层直接下发命令、返回 `None`。

异常一览（全部来自 `planner.errors`）：`ExecutionError`（owner 用错、重入、轨迹为空、不可达停止、超时）、`BudgetExceeded`（阶段或动作预算耗尽）、`ModelCallTimeout`（模型/后台调用超时，结果被隔离）、`ProtocolError`（队列、快照、时间线不一致）、`CollisionInterrupt`（控制循环内部信号，只在 `ControlLoop` 内部抛出并接住）、`MotionPaused`（内部信号）、`ObservationError`（移动帧连续失败）、`ResourceCleanupError`（关闭阶段聚合失败）。

## 4 对外名字表

| 名字 | 类型 | 一句话用途 | 层外调用点 |
| --- | --- | --- | --- |
| `BackgroundOperation` | 类（线程） | 一段只读后台调用：单飞、超时隔离、可关闭 | planner/workflows/visual_goal_navigation/targeting.py:52、planner/simulators/airsim/observation_source.py:107 |
| `ContinuousExecution` | 类 | 视觉目标连续路径的整拍执行：采样、消费、跟随、命令、遥测 | planner/workflows/visual_goal_navigation/continuous.py:39 |
| `ControlLoop` | 类 | 段执行与流执行的唯一控制循环 | planner/assembly/mission_assembly.py:216 |
| `ExecutionService` | 类 | 控制权、预算、deadline、停止；`Vehicle` 端口的停止侧兑现 | planner/assembly/mission_assembly.py:215 |
| `FrameHub` | 类（锁） | 最新相机帧的线程安全发布与等待 | planner/adapters/http/video.py:41、planner/server_bootstrap.py:30 |
| `GuidedVehicle` | 类 | `Vehicle` 端口包装：读侧转引擎，段执行转控制循环 | planner/assembly/mission_assembly.py:229 |
| `LogBuffer` | 类（锁） | 有界诊断日志与游标读取 | planner/assembly/controller/__init__.py:63 |
| `MODE_CONTINUOUS` | 常量 `str` | 连续执行写入的遥测模式串 | planner/workflows/free_navigation/continuous.py:22、planner/workflows/visual_goal_navigation/continuous.py:10 |
| `MODE_IDLE` / `MODE_NO_QUEUE` / `MODE_PENDING` | 常量 `str` | 空闲、无队列与待执行三种遥测模式串 | 无层外调用点，仅 telemetry.py 与 trajectory_hub.py |
| `MODE_STOPPED_BATCH` | 常量 `str` | 批次结束停稳后的遥测模式串 | planner/workflows/free_navigation/workflow.py:28、planner/workflows/visual_goal_navigation/workflow.py:23 |
| `ModelCallService` | 类（线程） | 单飞模型调用，超时后的在途结果隔离到线程返回，可换下一调用接管 | planner/workflows/visual_goal_navigation/workflow.py:89、planner/workflows/free_navigation/workflow.py:119 |
| `MotionPlanner` | Protocol | 世界轨迹到运动轨迹的策略接口 | planner/workflows/visual_goal_navigation/workflow.py:69 |
| `MovingObservations` | 类 | 移动帧的有界抓取泵与失败预算 | planner/workflows/visual_goal_navigation/continuous.py:41、planner/workflows/free_navigation/continuous.py:91 |
| `PlanningWindow` | 类 | 滑窗：授权队列、补点、在途候选收集 | planner/workflows/visual_goal_navigation/workflow.py:94 |
| `PositionHold` | 类 | owner 线程位置保持与到位窗口判定 | planner/simulators/projectairsim/drone.py:121 |
| `QueueFollower` | 类 | 连续路径的队列跟随器，只消费航点并给一拍命令 | planner/workflows/free_navigation/continuous.py:83 |
| `QueueSnapshot` | 冻结 dataclass | 快照里的队列长度与容量 | 仅 tests/test_trajectory_hub.py:150 |
| `SMOOTH_PROFILE` | 常量 `str` | 五次同步曲线的 profile 标识 | planner/workflows/basic_motion/workflow.py:86、planner/simulators/projectairsim/drone.py:203 |
| `SynchronizedMotionPlanner` | 类 | 世界航点转带时长的 `MotionTrajectory` | planner/assembly/mission_assembly.py:231 |
| `TrajectoryHub` | 类（锁） | 剩余轨迹面板快照的发布与读取 | planner/simulator/telemetry_relay.py:40、planner/server_bootstrap.py:19 |
| `TrajectoryPointSnapshot` | 冻结 dataclass | 快照里的一个航点：世界位姿、直线距离与段长 | 仅 tests/test_trajectory_hub.py:151 |
| `TrajectorySnapshot` | 冻结 dataclass | 控制台读的剩余路径快照 | planner/adapters/http/server_api.py:20、planner/simulator/telemetry_relay.py:20、planner/simulator/mission_child.py:36 |
| `VehicleSnapshot` | 冻结 dataclass | 快照里的车位姿 | 仅 tests/test_trajectory_hub.py:151 |
| `braking_speed` | 函数 | jerk 感知的刹车曲线速度上限 | 无层外调用点 |
| `build_queue_snapshot` | 函数 | 由队列、位姿与容量算出快照，直线距离与段长都算 | 同层 planner/runtime/trajectory_hub.py:106 |
| `close_resources` | 函数 | 逐项关闭并聚合失败 | planner/workflows/visual_goal_navigation/workflow.py:368、planner/workflows/free_navigation/workflow.py:189 |
| `duration_for` | 函数 | 按位移/偏航上限算最短时长（`segment_motion` 版） | 仅 planner/runtime/motion/__init__.py:2 转出的另一份同名实现被任务层使用 |
| `freeze_observation` | 函数 | 拷贝并按只读冻结 rgb 与深度 | planner/workflows/visual_goal_navigation/continuous.py:77、planner/workflows/visual_goal_navigation/workflow.py:286 |
| `freeze_reference` | 函数 | 拷贝冻结目标参考裁剪图 | planner/workflows/visual_goal_navigation/continuous.py:78、planner/workflows/visual_goal_navigation/targeting.py:112 |
| `heading_rate_deg_s` | 函数 | 航向误差到角速度命令的唯一律 | 无层外调用点 |
| `plan_segment` | 函数 | 按控制率采样一条五次段（`segment_motion` 版） | 仅 tests/test_motion.py:8 |
| `quintic` | 函数 | 归一化五次曲线 | 仅 tests/test_motion.py:8 |
| `snapshot_from_payload` | 函数 | 管道字典还原快照，缺键或类型不符抛 `ValueError` | planner/simulator/subprocess_worker.py:38 |
| `snapshot_to_payload` | 函数 | 快照转 JSON 安全字典，带上修订号 | planner/simulator/mission_child.py:36 |
| `vehicle_snapshot` | 函数 | `WorldPose` 转 `VehicleSnapshot` | 同层 planner/runtime/trajectory_hub.py |

二级入口 `planner.runtime.motion`（planner/runtime/motion/__init__.py:4）另转出 `duration_for`（synchronized 版），任务层从这里拿：planner/workflows/basic_motion/workflow.py:25。`planner.runtime.window`（planner/runtime/window/__init__.py:13）转出 `PlanningWindow`、`WorldWaypointQueue`、`SlowPlannerWorker`、`BackgroundContinuationPlanner`。

## 5 关键流程

段执行：一段一条五次曲线，30 ms 一拍
1. 入口两种。`ControlLoop.execute` 只做 checkpoint、取起点、调 `run_segment`、封装 `ExecutionRecord`，不传 budget（planner/runtime/motion/loop.py:191-200）；带预算的调用方直接用 `run_segment`（planner/workflows/visual_goal_navigation/flight.py:76）。
2. 前置断言。`execution.assert_owner()`、`_running` 防重入、空轨迹拒收（loop.py:203-208）。
3. 每段前：`execution.checkpoint()`、预算剩余大于 0、用 `resolve_yaw_delta_deg` 取原始段的偏航分支作为量测起点（loop.py:222-230）。
4. checkpoint 闭包：给 `pose_checkpoint` 时直接透传，否则每次调用先 `execution.check_safety()` 再问 `pause_when`（loop.py:232-251）。
5. 单段执行 `_drive_segment`（loop.py:383-520），顺序如下：
   1. 段必须带 `target_speed_mps`（loop.py:385-388）。
   2. `period = SEGMENT_PERIOD_S = 0.030`（loop.py:42,389），经验值、未标定，不读 `limits.control_hz`。
   3. `goal_yaw = start.yaw + 带符号偏航目标`，`duration = max(段上已有时长, duration_for(实测起点, 段终点, 速度, limits))`（loop.py:390-401）。
   4. 预算预检要求至少 `duration + settle_time_s` 可用，且不短于 `timeout_s`，否则 `BudgetExceeded` / `ExecutionError`（loop.py:402-406）。
   5. 用实测位姿重锚 `active = replace(segment, start=..., duration_s=..., yaw_delta_deg=剩余偏航)`，重置合成器与 `steps`（loop.py:408-416）。
   6. 每拍先过取帧门 `gate.step()`；代际变化说明段内做过一次取帧，从实测位姿重新锚定、重算时长、重置 `steps` 与停稳窗口（loop.py:419-443）。
   7. 每拍检查：预算剩余（loop.py:444）、碰撞接触（:445）、`elapsed = steps * period` 未超 `timeout_s`（:451-454）。
   8. 由位姿差分算实测速度与偏航角速度；停稳判据是 `elapsed >= duration` 且位置、偏航、速度、角速度同时落在容差内（loop.py:455-481）。
   9. 停稳持续 `settle_time_s` 后做最后复测，超差即 `ExecutionError("position/yaw drifted outside tolerance while hovering")`（loop.py:487-504）。
   10. 未停稳先问 `pose_checkpoint`，为真抛 `MotionPaused`；否则 `SegmentSynthesizer.command` 算期望速度与角速度，做有限性检查，交 `command_for` 转成该载具命令类型下发（loop.py:509-519）。
6. 段完成的记账：`completed += 1`，累计实测动作与弦长，再 `checkpoint()`、`_poll_contact()`、复查预算（loop.py:316-331）。
7. 暂停：`execution.stop()`，用暂停点与当前位姿折算实测动作，剩余轨迹从当前位姿重锚，返 `ExecutionReceipt(paused=True)`（loop.py:256-283）。
8. 碰撞中断：`_recover(contact, clear_queue=False)`，后退后从恢复位姿重锚剩余轨迹，同样返 receipt（loop.py:284-315）。
9. 其他异常：先 `execution.stop()`，尽力构造带 receipt 的异常；非 `ExecutionError` 包成 `ExecutionError` 并 `from exc`（loop.py:339-380）；`finally` 清 `_running`。

碰撞退避：被动技能，检查点触发
1. `CollisionRecovery.poll` 每拍读一次 `CollisionState`，恢复进行中或事件已处理时返 `None`，避免倒车途中反复上报被计成多次恢复（planner/runtime/collision_recovery.py:43-57）。
2. `begin` 累加次数，超过 `maximum_recoveries` 抛 `ExecutionError`，返回 `distance_m = backoff_m × attempts` 的计划（:59-73）。
3. `ControlLoop._back_off` 按 `SEGMENT_PERIOD_S` 倒车，速度按 `max_acceleration_mps2` 爬升，靠位姿差累计行程；行程达标下发一条零速命令后返回（loop.py:144-182）。步数上限是 `ceil(预计时长/period)*3 + 10`，走满仍未达标抛 `ExecutionError("collision backoff made no measurable progress")`（:152-155,181）。
4. `_recover` 收尾：`holding=True`、合成器与限幅器复位、流路径调 `window.interrupt()`、`recorder.reset(当前位姿)`（loop.py:124-142）。
5. `note_progress` 用恢复后累计路程作基线，超过 `reset_after_progress_m` 才把次数归零（collision_recovery.py:79-93）。

流执行：一拍一条命令，返回即交还任务层
1. `assert_owner()`，把 `deadline_monotonic` 包成 `ExecutionBudget`（loop.py:543-546,530-542）。
2. `_guard`：checkpoint、读接触（有接触抛 `CollisionInterrupt`）、预算剩余（loop.py:109-113）。
3. 接触路径：`_recover(contact, clear_queue=True)` 后正常返回，下一拍由任务层重新取帧补点（loop.py:545-547）。
4. `sample()` 读一次 `stream_state()` 并交给 `StreamRecorder` 记录，任务层的取帧与窗口都读这一次（loop.py:522-528）。
5. `goals.next(state, recorder.yaw)`：目标源返回 `None` 时停车一次后返回；有命令则过可选 `limiter` 并下发，周期是 `1 / limits.control_hz`（loop.py:548-558）。
6. 连续命令生成有两条实现：`QueueFollower.next`（planner/runtime/motion/follower.py:75-115）与 `ContinuousExecution.tick`（planner/runtime/continuous_execution.py:96-130）。两者都在队尾或原地转向之前先停稳，都有航点超时，都把世界系速度按运动模型投影成 `CarCommand` / `HolonomicCommand`。

滑窗：授权在 owner 线程，模型在后台线程
1. `begin` 校验 deadline（`None` 表示不限）与规划轮数，取 `min(调用方 deadline, now + max_task_duration_s)`，清空队列（planner/runtime/window/window.py:78-110）。
2. `stream_due` 一念判据：阶段活跃、没有在途 job 与候选、worker 空闲、队列不超过 5 点且加点后不超 `max_buffer_points`、过了重试时刻、距上次提交超过 `planning_interval_s`（window.py:168-174）。
3. `_submit`：冻结观测（rgb 拷贝、`depth_m` 只读）、把队列换成 `pending` 相对轨迹、算 `valid_until` 与 `request_deadline`、给 `on_request()` 打点、把 `_BoundPlanner` 绑上本次上下文后提交（window.py:117-166）。
4. `SlowPlannerWorker` 单线程串行跑模型，完成 future 时仍持锁，保证调用方看到 `done()` 后立即提交不会撞上旧任务（planner/runtime/window/worker.py:28-83）。
5. `BackgroundContinuationPlanner.poll` 收结果并校验：阶段 token、观测有效期、恰好 5 点、相对与世界两次校验；失败只累计 `consecutive_failures` 并记日志（planner/runtime/window/inflight.py:81-107）。
6. `poll_stream` 超时抛 `ModelCallTimeout`，连续失败达到上限抛 `ProtocolError`；追加成功返 `("queue_appended", request_id, world)`，追加被拒返 `("planning_discarded", request_id, reason)`（window.py:185-209）。
7. `next_world` 只在队列为空时用：先用候选，候选过期或锚点不符则丢弃重规划，重试上限是 `max_consecutive_plan_failures`（window.py:221-248）。
8. `executed` 把这次 receipt 并入 `record`；暂停则清队并按剩余轨迹重载前缀，否则消费队头（window.py:293-307）。
9. `interrupt`（碰撞后）与 `complete` / `end` / `close` 分别作废在途调用、清队、关 worker（window.py:314-335）。

线程与关闭
| 线程名 | 创建位置 | 谁关闭 | 关闭语义 |
| --- | --- | --- | --- |
| `planner-slow-model` | planner/runtime/window/worker.py:37 | `PlanningWindow.close` → `SlowPlannerWorker.close`（window.py:330-335、worker.py:69-77） | 置 `_closed`、取消 job，`stop_thread(…, "planner-slow-model")` |
| `planner-model-call` | planner/runtime/model_calls.py:93 | `ModelCallService.close`（model_calls.py:113-118） | 置 `_closed`，`stop_thread(…, "planner-model-call")` |
| `planner-data-operation` | planner/runtime/background_operation.py:57 | `BackgroundOperation.close`（background_operation.py:84-101） | `stop_thread(…, "planner-data-operation")`，再按需 `close_idle()` |
| `planner-read-cleanup` | planner/runtime/background_operation.py:99 | 自身，同样过 `stop_thread` | 等 `planner-data-operation` 以 1 s 为界；仍未收就放弃 `close_idle()` 并记 WARNING（background_operation.py:96-100） |

四条关闭走同一个 `stop_thread`（planner/runtime/resource_cleanup.py:35-51）：`join(CLOSE_TIMEOUT_S=1.0)`，线程未收就按名字打一条 WARNING，返回“仍在跑”。调用方在此之前已置 `_closed`，所有 `require_available` 随后一律拒绝新任务；线程是 daemon，随进程结束。

异常回到主循环的方式：全部经 `Future`。`SlowPlannerWorker._run` 用 `set_exception` 传回，`BackgroundContinuationPlanner.poll` 把它变成失败计数与日志（inflight.py:99-105）；`ModelCallService.run` 同样 `set_exception`，`call` 在 owner 线程用 `future.result()` 重新抛出（model_calls.py:58-111）；`BackgroundOperation.poll` 也是 `future.result()`（background_operation.py:63-77）。

任务收尾时 `close_resources` 逐项关闭并聚合失败，`KeyboardInterrupt` / `SystemExit` 优先重抛（planner/runtime/resource_cleanup.py:7-21）。

## 6 失败模式

| 报错类型 | 触发条件 | 调用方处理 |
| --- | --- | --- |
| `ExecutionError("flight execution may only run on its owner thread")` | 非创建线程调 owner 方法（execution.py:36-38）；队列、窗口、保持、模型调用同类断言见 queue.py:39-41、window.py:85、position_hold.py:38-40、model_calls.py:28-30 | 线程模型接线错误，修调用点 |
| `ExecutionError("nested motion execution is not allowed")` / `"cannot execute an empty motion trajectory"` | 段执行重入或空轨迹（loop.py:205-207） | 任务层串行化调用，空轨迹提前拦掉 |
| `ExecutionError("smooth motion requires a speed limit")` / `"… exceeds available execution timeout"` | 段缺 `target_speed_mps`，或段时长加停稳窗口超过 `timeout_s`（loop.py:385-406） | 检查 `SynchronizedMotionPlanner` 与 `drone.move_timeout_s` 配置 |
| `ExecutionError("synchronized motion timed out before position/yaw settled")` | 段内拍数超时（loop.py:451-454） | 视为载具未跟上曲线，按失败收尾 |
| `ExecutionError("position/yaw drifted outside tolerance while hovering")` | 停稳窗口末复测超差（loop.py:497-504） | 视为到位失败，不要当成功 |
| `BudgetExceeded` | `ExecutionBudget.remaining <= 0` 或阶段 deadline 到期（loop.py:112-113,187-189,223-224；execution.py:77-82；window.py:112-115） | 任务层决定收尾，执行层不重试 |
| `CollisionInterrupt` | `_poll_contact` 读到未处理接触（loop.py:115-122） | 只在 `ControlLoop` 内部处理，不外抛；预算耗尽时 `begin` 抛 `ExecutionError` |
| `MotionPaused` | `pose_checkpoint` 返回真（loop.py:509） | 内部信号，转成 `ExecutionReceipt(paused=True)` |
| `ValueError("unusable trajectory snapshot: ...")` | 管道消息缺 `active`、`queue`、`points` 等键，或数值不能转 float（telemetry.py:182） | `subprocess_worker` 视为事件损坏，丢掉该事件 |
| `ModelCallTimeout` | 模型调用超时或结果晚于 deadline（model_calls.py:76-81）；窗口/后台同语义（window.py:189-191,218、background_operation.py:71-77）；`_BoundPlanner` 记图超时（window.py:52-56） | 本阶段失败并隔离在途结果，不要等待 |
| `ModelCallTimeout` | 模型调用超时或结果晚于 deadline（model_calls.py:58-111）；窗口/后台同语义（window.py:189-191,218、background_operation.py:71-77）；`_BoundPlanner` 记图超时（window.py:52-56） | 本阶段失败；过了 deadline 下一调用可接管，旧结果丢弃。
| `ProtocolError` | 队列非空时授权新批量、点数不是 5、快照 token/修订不符、时间线倒退、连续失败超限（queue.py:60-62,88-90,119-126、stream_timeline.py:15-22、window.py:196-197,266-267） | 丢弃本轮授权与模型输出，重新取帧 |
| `ObservationError` | 移动帧连续失败达上限（moving_observations.py:24-35） | 任务层结束连续阶段 |
| `ResourceCleanupError` | 至少一个关闭回调抛异常（resource_cleanup.py:18-21） | 记录聚合失败，继续退出流程 |
| 后台线程关不掉（不抛异常） | 关闭时线程 1 s 内没退出：四份 close 都走 `stop_thread`（resource_cleanup.py:35-51），按名字打 WARNING 后放走（worker.py:69-77、model_calls.py:113-118、background_operation.py:84-101） | 进程继续退出，线程随进程结束；日志里已有线程名，不要假设调用点返回就已停 |
| 模型调用挂了很久（不抛异常） | 超时后 `_busy` 仍为真，但过了 `_deadline` 后 `require_available` 放行下一个调用；旧线程返回时按代号丢弃并打 WARNING（model_calls.py:38-56、67-95） | 阶段可以继续推进；`has_pending_call` 仍为真，别把它当“无在途调用” |

## 7 测试位置

- `tests/test_control_loop_segment.py`（260 行）：段执行节拍、限幅、停稳、`SEGMENT_PERIOD_S`、`limit_vector`、暂停与异常路径的 receipt。
- `tests/test_control_loop_stream.py`：`run_stream` 一拍、目标源返 `None` 后的停车、预算耗尽。
- `tests/test_collision_recovery.py`：恢复次数递增、预算耗尽、进度复位、接触去重、流路径清队、段路径返剩余轨迹。
- `tests/test_queue_follower.py`：队列消费、停稳边界、航点超时、命令类型。
- `tests/test_position_hold.py`：到位窗口、`observable`、`approach_target` 的两种返回。
- `tests/test_batch3_services.py`（`:78` 起）：`PlanningWindow` 的 begin 校验、无界阶段、前缀装载与比对、`next_world` 五点锚定、未清队拒收、`executed` 消费；`MovingObservations` 的失败预算；`close_resources` 聚合与中断重抛。
- `tests/test_frame_hub.py`、`tests/test_trajectory_hub.py`、`tests/test_log_buffer.py`：三个汇的并发与游标行为。
- `tests/test_motion.py`、`tests/test_heading_servo.py`、`tests/test_motion_contract.py`、`tests/test_passed_corridor.py`：`plan_segment`、`quintic`、刹车曲线、航向律与越点走廊的数值行为。
- `tests/test_visual_goal_trace.py`：`ControlTelemetry` 被诊断轨迹消费。
- `tests/test_runtime_thread_shutdown.py`：`stop_thread` 的 WARNING 与返回值、三份 close 的有界等待与日志、超时后的调用接管与迟到结果丢弃。

未覆盖的空白
- `ContinuousExecution`（183 行）没有任何测试文件 import，只经 visual_goal 的连续路径间接跑到；`tests/test_public_surface.py` 只断言名字在入口。
- `SlowPlannerWorker`、`BackgroundContinuationPlanner`、`WorldWaypointQueue`、`StreamTimeline` 无直接测试，覆盖来自 `PlanningWindow` 的间接路径。
- `ModelCallService` 的 takeover 只测了“过期后放行下一个调用”，没有接法测两个调用同时在途的时序。
- `ControlLoop._back_off` 的步数上限与"无可测进度"分支无测试。
- 段执行 30 ms 周期与引擎实际步长的一致性无测试。

## 8 与其他层的联动

- 上游 `domain`：`MotionLimits`、`MotionTrajectory`、`ExecutionReceipt`、`TimedVehicleState`、`PlanningSnapshot`、`QueueObservationAnchor`、`WorldPose`、`QueuedWaypoint`。
- 上游 `ports`：`Vehicle`（`GuidedVehicle` 兑现）、`StreamingVehicle`（控制循环的 `streaming`）、`PositionHoldVehicle`、`StreamGoalSource`（`run_stream` 的目标源）、`MovingObservationSource`、`TrajectoryPlanner`（`_BoundPlanner` 包装）。
- 上游 `algorithms`：`bounded`、`reached`、`world_to_relative`、`relative_to_world`、`require_execution_anchor`、`TrajectoryValidator`、`RefillScheduler`。
- 下游 `workflows`：`ControlLoop.execute` / `run_segment` / `run_stream`、`PlanningWindow` 全套、`ModelCallService`、`MovingObservations`、`freeze_observation`、`close_resources`。任务层持有编排，执行层只提供原语。
- 下游 `assembly`：`ExecutionService`、`ControlLoop`、`GuidedVehicle`、`SynchronizedMotionPlanner`、`TrajectoryValidator` 的接线在 planner/assembly/mission_assembly.py:215-236。
- 下游 `adapters` / `simulator`：`FrameHub`、`TrajectoryHub` 被 HTTP 与遥测中继读取。
- 数据形态：命令是 `CarCommand` 或 `HolonomicCommand` 加 `duration_s`；状态是 `TimedVehicleState`；跨层传递的都是不可变 dataclass，唯一的可变载体是队列与 receipt。

## 9 疑问与争议

Qb8 层：runtime 结论：`GuidedVehicle` 八个方法里只有 `current_pose` 有生产调用点，其余七个无人调用。依据：planner/runtime/guided_vehicle.py:16-45、planner/workflows/basic_motion/workflow.py:68,98 影响：否

Qb9 层：runtime 结论：`Vehicle.execute_segment` 端口方法与 `GuidedVehicle.execute_segment` 在生产路径无人调用，段执行实际走 `ControlLoop.execute`；引擎类已不再实现该方法。依据：planner/ports/vehicle.py:27、planner/runtime/guided_vehicle.py:37-45、planner/workflows/basic_motion/workflow.py:97 影响：是

Qb10 层：runtime 结论：`ControlLoop.sample()` 隐含要求 `StreamRecorder.begin()` 先被调用，否则抛 `AttributeError: 'NoneType' object has no attribute 'revision'`（已用探针脚本复现），docstring 未写这个前置条件。依据：planner/runtime/motion/loop.py:522-528、planner/runtime/motion/record.py:79、planner/runtime/stream_timeline.py:17 影响：否

Qb11 层：runtime 结论：`ExecutionService.collision_state()` 恒返 `False`，`check_safety()` 永不触发，docstring 描述的碰撞检查分支在代码里不存在。依据：planner/runtime/execution.py:87-90,104-111 影响：否

Qb12 层：runtime 结论：`ExecutionService.stop()` 恒返 `StopResult(False, ...)`，执行层的 `confirmed` 永远为假，调用方只能读 `errors`。依据：planner/runtime/execution.py:113-123、planner/domain/execution.py:44-46 影响：否

Qb13 层：runtime 结论：`ContinuousExecution` 与 `QueueFollower` 是两套几乎逐行相同的连续控制实现，职责边界只能靠调用方区分（视觉目标 vs 自由导航）。依据：planner/runtime/continuous_execution.py:96-142、planner/runtime/motion/follower.py:75-115 影响：是

Qb14 层：runtime 结论：`StreamRecorder.sample` 与 `ContinuousExecution.sample` 重复实现同一套动作记录与 200 条上限逻辑。依据：planner/runtime/motion/record.py:55-82、planner/runtime/continuous_execution.py:56-81 影响：否

Qb15 层：runtime 结论：`duration_for` 有两个同名不同签名的实现，包入口导出的是任务层并不使用的那份。依据：planner/runtime/segment_motion.py:26、planner/runtime/motion/synchronized.py:14、planner/runtime/__init__.py:21、planner/workflows/basic_motion/workflow.py:25 影响：否

Qb16 层：runtime 结论：`braking_speed`、`heading_rate_deg_s`、`plan_segment`、`quintic` 四个入口导出名没有层外生产调用点，`plan_segment` 与 `quintic` 只被 tests 使用。依据：planner/runtime/__init__.py:14-15,21,42-49、tests/test_motion.py:8、tests/test_heading_servo.py 影响：否


Qb18 层：runtime 结论：`PlanningWindow.executed` 在队列为空时直接 `self.queue.items[0]`，抛的是 `IndexError` 而不是本层统一的 `ProtocolError`。依据：planner/runtime/window/window.py:307 影响：否

Qb19 层：runtime 结论：`PlanningWindow._submit` 每次提交前改写共享字段 `background.planner`，请求与上下文的绑定只靠 worker 串行兜住，没有断言保护。依据：planner/runtime/window/window.py:165-166、planner/runtime/window/inflight.py:64-74 影响：否

Qb22 层：runtime 结论：`limit_vector` 与 `algorithms.bounded` 是同一套范数限幅的两份实现，只有非有限值的处理不同。依据：planner/runtime/motion/tracking.py:38-48、planner/algorithms/motion/limits.py:8-11 影响：否

Qb23 层：runtime 结论：`segment_sink` 与 `stream_sink` 方法体相同，`run_stream` 的 `sink` 参数在所有生产调用点都传 `loop.stream_sink()`。依据：planner/runtime/motion/loop.py:103-107,530、planner/workflows/common/streaming.py:87、planner/workflows/visual_goal_navigation/flight.py:76 影响：否

Qb24 层：runtime 结论：`run_segment` 只捕 `Exception` 与 `KeyboardInterrupt`，`SystemExit` 路径不会调用 `execution.stop()`，载具保持最后一条命令。依据：planner/runtime/motion/loop.py:339-380 影响：否

Qb25 层：runtime 结论：段执行周期固定 30 ms，与 `MotionLimits.control_hz` 无关，引擎是否按这个时长兑现没有测试。依据：planner/runtime/motion/loop.py:42,389,515-519 影响：否


Qb27 层：runtime 结论：`MovingObservations.begin` 里 discard 只在 job 空闲时执行，忙时直接进 `require_available` 并抛"unavailable or quarantined"，错误信息指向的能力状态与真实原因（上一次还在跑）不一致。依据：planner/runtime/moving_observations.py:18-22、planner/runtime/background_operation.py:26-30 影响：否

## 10 已知缺口

M4 契约适配层与生产路径分离
- 现状：`GuidedVehicle` 八个方法里只有 `current_pose` 有生产调用点（planner/workflows/basic_motion/workflow.py:68,98）；`GuidedVehicle.execute_segment`（planner/runtime/guided_vehicle.py:37）生产零调用，段执行实际走 `ControlLoop.execute`（planner/workflows/basic_motion/workflow.py:97）。三个引擎类不再实现 `execute_segment`，装配层也不再向载具注入控制循环。
- 风险：端口要求的方法与真实路径分叉，改控制循环不会带动 `GuidedVehicle` 的实现，反之亦然。
- 触发条件：按 `Vehicle` 端口签名改段执行时，改动落在无人调用的路径上，运行行为不变。

M5 重复实现
- 生产路径：`QueueFollower.next`（planner/runtime/motion/follower.py:75）服务自由导航；`ContinuousExecution.tick`（planner/runtime/continuous_execution.py:96）服务视觉目标。
- 死路：`planner/runtime/segment_motion.py` 整模块，`plan_segment`、`quintic`、`duration_for` 只被 tests 使用；生产时长统一走 `synchronized.duration_for`（planner/runtime/motion/loop.py:37）。
- 其他重复：`StreamRecorder.sample`（motion/record.py:55）与 `ContinuousExecution.sample`（continuous_execution.py:56）；`limit_vector`（motion/tracking.py:38）与 `algorithms.bounded`（planner/algorithms/motion/limits.py:8）；`segment_sink` 与 `stream_sink` 方法体相同（motion/loop.py:103,106）。
- 风险：改一处漏一处，两条通道行为漂移。
- 触发条件：改限幅、动作记录或队列消费逻辑时只改了一份。

M6 恒假分支
- 碰撞状态端口在引擎侧有真实实现：planner/simulators/projectairsim/drone.py:268 消费锁存的非地面接触，planner/simulators/airsim/vehicle.py:205 读 `simGetCollisionInfo`。
- 恒返 `False` 的是 `ExecutionService.collision_state`（planner/runtime/execution.py:104），因此 `ExecutionService.check_safety`（execution.py:87）永不触发，执行服务自检分支失效。
- 退避由 `ControlLoop._poll_contact` 与 `_guard` 独立完成（planner/runtime/motion/loop.py:115,109），不经过执行服务。
- 触发条件：任何指望 `check_safety` 拦碰撞的调用点都拿不到拦截。

M7 经验值未标定
- `SEGMENT_PERIOD_S = 0.030`（planner/runtime/motion/loop.py:42）固定，与 `MotionLimits.control_hz` 无关；流拍是 `1/control_hz`（loop.py:99）。风险：引擎步长不等于 30 ms 时，停稳判据与超时都按 30 ms 记账。触发条件：换仿真步长或引擎实时性不足。
- `waypoint_timeout_s` 默认 60.0（continuous_execution.py:23、motion/follower.py:32），同为经验值。

M12 状态机单点问题
- `PlanningWindow.executed` 在队列为空时直接 `self.queue.items[0]`，抛 `IndexError` 而不是 `ProtocolError`（planner/runtime/window/window.py:307）。触发条件：暂停清队之后又调一次 `executed`。
- `PlanningWindow._submit` 每次提交前改写共享字段 `background.planner`（window.py:165），请求与上下文的绑定只靠 worker 串行兜住。触发条件：两次提交之间换上下文。
- 本轮已删 `WorldWaypointQueue.load_batch` 与 `CollisionRecovery.reset`（原 Qb17、Qb26 随代码消失）。
