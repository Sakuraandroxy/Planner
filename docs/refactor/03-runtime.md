# 03 执行层 runtime

执行层持有每台载具在任何时刻唯一的控制权，按 30 ms 与 50 ms 两个周期把任务层给的目标变成限幅后的 `MotionCommand`，并在每个检查点处理停止、预算与碰撞退避。任务层不感知周期、线程、队列与退避。

## 1 职责与文件清单

### 1.1 判据与归属

按 `docs/TARGET_ARCHITECTURE.md` 的两个问题：有状态、读时钟、不认识具体任务。本层因此收下四类东西：

| 类别 | 本层内容 | 判据落点 |
| --- | --- | --- |
| 控制权 | `ExecutionService` 的 owner 线程、flight lease、停止事件、deadline | 任一时刻每台载具一个 owner |
| 运动执行 | `ControlLoop` 两条循环、命令限幅、位置保持、滑窗执行 | 谁在什么时候发指令 |
| 观测与遥测汇 | `FrameHub`、`TrajectoryHub`、`LogBuffer`、`StreamTimeline` | 有锁、有界、跨线程读 |
| 受限后台任务 | `SlowPlannerWorker`、`ModelCallService`、`BackgroundOperation`、`MovingObservations` | 起线程、隔离迟到结果 |

纯函数不进本层：`heading_rate_deg_s`、`braking_speed`、`planned_speed`、`corner_speed`、`reached`、`duration_for`、`sample_segment`、`translation_handoff`、`ContinuousPathReference`、`world_to_relative`、`require_execution_anchor` 由相邻的计算层 `algorithms/` 提供（见计算层方案）。本层只写持有状态的外壳。

### 1.2 文件清单

| 文件 | 类与函数签名 | 估计行数 | 从哪来 |
| --- | --- | --- | --- |
| `runtime/__init__.py` | 导出 `ControlLoop`、`ExecutionService`、`PlanningWindow`、`CollisionRecovery`、`PositionHold` | 14 | 新建 |
| `runtime/execution.py` | `class ExecutionService(vehicle: Vehicle, clock=time.monotonic)`；`acquire_flight(owner)`（裁决 R5：去掉 `collision_check` 参数）；`release_flight(owner, pending_cleanup=None)`；`stop_if_owner(owner)`；`set_deadline(deadline)`；`clear_deadline()`；`request_stop()`；`check_deadline()`；`checkpoint()`；`wait(seconds)`；`stop() -> StopResult`；`current_pose()` | 210 | `planner/services/execution.py` 262 行，`execute`/`execute_with_receipt` 移出到 `motion/loop.py` |
| `runtime/collision_recovery.py` | `class CollisionRecovery(settings: CollisionRecoverySettings, capability: MotionCapability, limits: MotionLimits)`；`poll(state: CollisionState) -> CollisionContact \| None`；`begin(contact, pose) -> BackoffPlan`；`finish()`；`note_progress(metres)`；`reset()`；`attempts: int` | 130 | 新建，吸收 `planner/workflows/free_navigation/continuous.py:312` 与 `planner/workflows/visual_goal_navigation/continuous.py:259` 两份镜像 |
| `runtime/motion/__init__.py` | 导出 `ControlLoop`、`make_control_loop(...)` | 18 | 新建 |
| `runtime/motion/loop.py` | `SEGMENT_PERIOD_S = 0.030`；`class ControlLoop(execution, *, streaming, synthesizer, limiter, recovery, window=None, clock=time.monotonic, sleep=time.sleep)`；`run_segment(segment: MotionSegment, sink: SegmentSink, *, budget: ExecutionBudget \| None) -> ExecutionReceipt`；`run_stream(goals: StreamGoalSource, sink: StreamSink, *, deadline_monotonic: float \| None) -> None` | 430 | 吸收 `planner/services/execution.py:127` 的逐段循环与 `planner/services/continuous_execution.py` 169 行 |
| `runtime/motion/tracking.py` | `class MotionSynthesizer(capability, limits, tracking)`；`reset(velocity_world)`；`desired(goals, state, unwrapped_yaw, goal_yaw, dt) -> MotionCommand`；`class HolonomicSynthesizer`；`class AckermannSynthesizer` | 260 | 拆分 `planner/services/motion/continuous.py:31` 的 `ContinuousFollower`；阿克曼纯跟踪新写 |
| `runtime/motion/limits.py` | `class CommandLimiter(limits, tracking)`；`reset(velocity_world)`；`limit(command, state, dt) -> MotionCommand`；`command_type` | 120 | `planner/services/motion/command_smoothing.py` 32 行的 `CommandSmoother` 加航向伺服包装 |
| `runtime/motion/guided.py` | `class GuidedVehicle(execution, loop, streaming)`，兑现 `Vehicle` 槽：`current_pose` / `motion_capability` / `collision_state` / `cancel` / `hover` 转发到 streaming，`execute_segment(segment, *, budget, pose_checkpoint)` 委派 `loop.run_segment` | 90 | 新建（裁决 R1）；取代 `planner/simulators/projectairsim/motion.py:47` 的 `SteppedMotionTracker` 与 `planner/simulators/airsim/motion_tracker.py` 的下发段 |
| `runtime/motion/follower.py` | `class QueueFollower(streaming, window, synthesizer, limits, clock)`，实现 `StreamGoalSource`：`next(state: TimedVehicleState, unwrapped_yaw_deg: float) -> MotionCommand \| None`，含队列消耗与 waypoint 超时（裁决 R4）；`consumed: tuple[int, ...]` | 160 | `planner/services/continuous_execution.py:94` 的 `tick` 去掉下发与记录 |
| `runtime/motion/record.py` | `class StreamRecorder(clock)`；`sample(state, yaw)`；`reset(at: WorldPose)`；`record: ExecutionRecord \| None`；`path_length_m: float`；`drain_telemetry()` | 100 | `planner/services/continuous_execution.py:54`、`:130`、`:143` |
| `runtime/window/__init__.py` | 导出 `PlanningWindow` | 12 | `planner/capabilities/sliding_window/__init__.py` 由空说明改为导出 |
| `runtime/window/window.py` | `class PlanningWindow(planner, validator, settings, motion_limits, clock)`；公开 `begin(deadline_monotonic, maximum_rounds)`、`submit(observation, context, instruction, planning)`、`poll_stream(measured, checkpoint)`、`pause_for_completion(checkpoint)`、`complete()`、`end()`；层内 `load(motion)`、`prefetch(...)`、`executed(before, receipt, remaining)`、`next_world(...)`、`interrupt()` | 330 | `planner/services/planning_window.py` 331 行 |
| `runtime/window/queue.py` | `class WorldWaypointQueue(position_tolerance_m, yaw_tolerance_deg, passed_lateral_tolerance_m)`；`load_prefix`、`load_batch`、`pending`、`consume`、`consume_stream`、`append_continuation`、`pause_stream`、`restore_stream`、`clear`、`matches`、`require_anchor`、`check_owner` | 150 | `planner/services/trajectory_queue.py` 148 行 |
| `runtime/window/worker.py` | `class SlowPlannerWorker`；`require_available()`、`has_pending_call`、`submit(operation) -> Future`、`close()` | 80 | `planner/capabilities/sliding_window/worker.py` 76 行 |
| `runtime/window/inflight.py` | `class BackgroundContinuationPlanner(planner, validator, worker, scheduler, clock)`；`begin(request_limit)`、`submit(snapshot)`、`poll()`、`wait(checkpoint)`、`discard()`、`end()` | 140 | `planner/capabilities/sliding_window/planning.py` 135 行 |
| `runtime/position_hold.py` | `class PositionHold(vehicle: PositionHoldVehicle, limits, tracking, speed_mps, timeout_s, *, clock, sleep)`；`start(target=None)`、`release()`、`tick()`、`wait(*, budget, checkpoint)`、`climb_by(metres)`、`observable`、`contains_pose(pose)` | 170 | `planner/services/position_hold.py` 同职责 |
| `runtime/frame_hub.py` | `class FrameHub(max_fps=10.0, clock)`；`publish(bgra, timestamp_s)`、`wait_next(after, timeout_s)`、`close()` | 70 | `planner/services/frame_hub.py` 同职责 |
| `runtime/trajectory_hub.py` | `class TrajectoryHub(clock)`；`read()`、`publish(snapshot)`、`adopt(snapshot)`、`start`、`clear`、`publish_no_queue`、`publish_queue` | 120 | `planner/services/trajectory_hub.py` 同职责 |
| `runtime/log_buffer.py` | `class LogBuffer(capacity=500)`；`append(message, level)`、`read(cursor)` | 45 | `planner/services/log_buffer.py` 同职责 |
| `runtime/model_calls.py` | `class ModelCallService(clock)`；`require_available()`、`has_pending_call`、`call(operation, timeout_s, checkpoint)`、`close()` | 95 | `planner/services/model_calls.py` 同职责 |
| `runtime/background_operation.py` | `class BackgroundOperation(clock)`；`busy`、`require_available()`、`start(operation, timeout_s)`、`poll()`、`discard()`、`close(close_idle=None)` | 96 | `planner/services/background_operation.py` 同职责 |
| `runtime/moving_observations.py` | `class MovingObservations(source, clock, timeout_s, maximum_failures=3)`；`begin()`、`poll()`、`request(budget)`、`discard()`、`close()` | 50 | `planner/services/moving_observations.py` 同职责 |
| `runtime/observation_snapshot.py` | `freeze_observation(observation)`；`freeze_reference(reference)` | 18 | `planner/services/observation_snapshot.py` 同职责 |
| `runtime/stream_timeline.py` | `class StreamTimeline(maximum_entries=4096)`；`clear()`、`record(timestamp_ns, stage_token, queue, history)`、`at(timestamp_ns, stage_token, maximum_gap_ns)` | 35 | `planner/services/stream_timeline.py` 同职责 |
| `runtime/resource_cleanup.py` | `close_resources(callbacks)` | 18 | `planner/services/resource_cleanup.py` 同职责 |

合计约 2960 行，与现 `planner/services/` 2710 行加 `capabilities/sliding_window/` 226 行基本持平。

### 1.3 本层删除或移出的东西

| 对象 | 处置 | 依据 |
| --- | --- | --- |
| `planner/services/segment_motion.py` 的 `MotionLimits` | 删；`to_tracking_limits()` 的等价转换进 `domain/motion.py` | 同名不同类，一个事实两处定义 |
| `planner/services/segment_motion.py` 的 `quintic`、`duration_for`、`plan_segment` | 移到 `algorithms/motion/quintic.py` | 纯函数 |
| `planner/services/motion/{heading_servo,continuous_speed,continuous_reference,hold_handoff,synchronized,base}.py` | 移到 `algorithms/motion/` | 纯函数与纯策略 |
| `planner/services/motion/continuous.py` 的 `reached` | 移到 `algorithms/motion/reached.py` | 纯判定 |
| `planner/services/trajectory_transform.py`、`trajectory_validation.py` | 移到 `algorithms/` | 纯几何与纯校验 |
| `planner/services/continuous_execution.py` | 拆进 `motion/loop.py`、`motion/follower.py`、`motion/record.py` | 一个实现同时做循环、跟随与记账 |
| `planner/capabilities/sliding_window/{worker,planning}.py` | 迁入 `runtime/window/` | 线程与在途请求属于执行层 |
| `planner/ports/collision_check.py` | 删（裁决 R5） | 碰撞状态由 `Vehicle.collision_state() -> CollisionState` 表达 |
| `simulators/projectairsim/motion.py`、`simulators/airsim/motion_tracker.py` 的律部分 | 删，引擎只剩 `stream_command`（裁决 R1、R2） | 三套跟踪律收敛成 `motion/tracking.py` 一份，采样循环在 `loop.py` |
| `ROVER_PASSED_LATERAL_TOLERANCE_M` 与 `rover_tracking_limits()` | 删（裁决 R6） | 走廊改由 `algorithms/motion/corridor.py` 按 `MotionModel` 取值 |

## 2 数据流与时序

### 2.1 两条循环的共用骨架

`ControlLoop` 只做一件事：以固定周期把"目标"变成"合法命令"并下发，每一步都过同一个关卡。两处差异被压到两个常量与两个目标来源上。

| 环节 | `run_segment` | `run_stream` | 共用实现 |
| --- | --- | --- | --- |
| 周期 | `SEGMENT_PERIOD_S = 0.030` | `1 / limits.control_hz`（20 Hz = 0.050） | `loop.py` 常量 |
| 目标来源 | 五次多项式曲线在 `t` 处的参考位姿 | `QueueFollower.next(state, unwrapped_yaw_deg)`（裁决 R4） | 都返回世界系期望 |
| 限幅 | `CommandLimiter.limit` | 同左 | `motion/limits.py` |
| 伺服 | `MotionSynthesizer`（按 `MotionCapability.model` 分型） | 同左 | `motion/tracking.py` |
| 偏航律 | `algorithms.heading_rate_deg_s` | 同左 | 一份 |
| 预算 | `budget.remaining(now) <= 0` | `deadline_monotonic` | `_guard` |
| 停止 | `execution.check_deadline()` | 同左 | `_guard` |
| 碰撞 | 接触即中断当前段 | 接触即清队列 | `CollisionRecovery` |
| 结束 | 全部段完成并停稳，返回 `ExecutionReceipt` | `deadline_monotonic` 到期抛 `BudgetExceeded`，任务层映射 TIMEOUT（裁决 R3） | — |

`_guard` 是两条循环唯一的检查点入口，本层所有"停止/超时/碰撞"都从这里出：

```python
def _guard(self, budget: ExecutionBudget | None) -> None:
    self.execution.checkpoint()          # 停止事件、hold 检查点、观测检查点
    if budget is not None and budget.remaining(self.clock()) <= 0:
        raise BudgetExceeded("stage time budget exhausted")
    contact = self.recovery.poll(self.vehicle.collision_state())
    if contact is not None and not self.recovering:
        raise CollisionInterrupt(contact)
```

`ExecutionService.checkpoint()` 保持现有语义：`_stop_requested` → `ExecutionError`，`_deadline` → `BudgetExceeded`，再跑 `bind_hold_checkpoint` 与 `bind_observation_checkpoint` 两个回调（`planner/services/execution.py:113`）。碰撞不进 `ExecutionService`，因为退避要点动载具，只有循环能点。这与 `docs/REFACTOR.md`「被动能力」一节里"`checkpoint()` 是碰撞退避的触发点"的字面表述有偏差，取舍见第 8 节第 2 条。

### 2.2 `run_segment` 伪代码

```python
def run_segment(self, segment, sink, *, budget=None) -> ExecutionReceipt:
    self.execution.assert_owner()
    if self._running:
        raise ExecutionError("nested motion execution is not allowed")
    self._running = True
    began = self.clock()
    actions, chords, completed = [], 0.0, 0
    active, previous, observed_yaw = None, None, None
    try:
        self._guard(budget)
        previous = self.vehicle.current_pose()
        for index, segment in enumerate(trajectory.segments):
            self._guard(budget)
            # 1. 以实测位姿重锚，保留世界目标与带符号偏航
            measured = self.streaming.stream_state()
            goal_yaw = segment.start.yaw_deg + resolve_yaw_delta_deg(
                segment.start.yaw_deg, segment.end.yaw_deg, segment.yaw_delta_deg)
            measured_yaw = segment.start.yaw_deg + resolve_yaw_delta_deg(
                segment.start.yaw_deg, measured.pose.yaw_deg)
            remaining_yaw = goal_yaw - measured_yaw
            duration = max(segment.duration_s or 0.0,
                algorithms.duration_for(measured.pose, segment.end,
                                        segment.target_speed_mps, self.limits,
                                        yaw_delta_deg=remaining_yaw))
            # 2. 预算预检：整段加停稳时间放不进预算就当场报
            if budget is not None and budget.remaining(self.clock()) <= duration + self.limits.settle_time_s:
                raise BudgetExceeded("deadline before segment")
            active = replace(segment, start=measured.pose,
                             duration_s=duration, yaw_delta_deg=remaining_yaw)
            self.synthesizer.reset(measured.velocity_world)
            self.limiter.reset(measured.velocity_world)
            # 3. 30 ms 一拍走完整段
            steps, elapsed = 0, 0.0
            while elapsed < duration:
                tick_at = self.clock()
                self._guard(budget)                       # 停止/预算/碰撞中断
                state = self.streaming.stream_state()
                if state.timestamp_ns < measured.timestamp_ns:
                    raise ExecutionError("simulator clock moved backwards during motion")
                sample = algorithms.sample_segment(active, elapsed)
                command = self.synthesizer.desired(
                    target=sample, state=state, yaw=unwrapped_yaw(state),
                    goal_yaw=goal_yaw, dt=SEGMENT_PERIOD_S)
                command = self.limiter.limit(command, state, SEGMENT_PERIOD_S)
                sink.stream_command(command, SEGMENT_PERIOD_S)  # 唯一数据面动作（裁决 R2）
                previous, measured = measured.pose, state
                steps += 1
                elapsed = steps * SEGMENT_PERIOD_S
                self._pace(tick_at, SEGMENT_PERIOD_S)
            # 4. 停稳验证：位置与偏航进容差并保持 settle_time_s
            self.execution.stop()
            self.hold.wait(budget=budget, checkpoint=self.execution.checkpoint)
            completed += 1
            actions.append(measured_action(previous, current_pose(), remaining_yaw))
            previous, active = current_pose(), None
            self._guard(budget)
        return ExecutionReceipt(completed, self.vehicle.current_pose(),
                                self.clock() - began, chords,
                                measured_actions=tuple(actions))
    except CollisionInterrupt as interrupt:
        return self._after_collision(interrupt, sink, budget,
                                     completed, previous, active, actions, began)
    except (Exception, KeyboardInterrupt) as exc:
        stop = self.execution.stop()
        receipt = self._partial_receipt(completed, active, previous, actions, began)
        exc.receipt, exc.stop_result = receipt, stop
        raise exc if isinstance(exc, (ExecutionError, KeyboardInterrupt)) \
              else ExecutionError(str(exc)) from exc
    finally:
        self._running = False
```

`run_segment` 的输入仍是 `MotionSegment`，但调用方按段循环：规划器一次给出五个航点时，任务层把这五个航点转成五段，逐段调用，语义与现在的 `execute_with_receipt` 相同。

### 2.3 `run_stream` 伪代码

```python
def run_stream(self, goals, sink, *, deadline_monotonic=None) -> None:
    self.execution.assert_owner()
    budget = None if deadline_monotonic is None else ExecutionBudget(deadline_monotonic)
    # 到期由 _guard 抛 BudgetExceeded，不静默返回（裁决 R3）
    began = self.clock()
    self.holding = True
    try:
        while True:
            tick_at = self.clock()
            self._guard(budget)                       # 停止/预算/碰撞中断
            state = self.streaming.stream_state()
            if self._last_ns is not None and state.timestamp_ns < self._last_ns:
                raise ExecutionError("simulator clock moved backwards during stream")
            self._last_ns = state.timestamp_ns
            unwrapped = self.recorder.sample(state)   # 位姿差分、动作序列、path_length
            if self.window is not None and not self.window.active:
                raise ExecutionError("streaming requires an active planning window")
            # 1. 目标 -> 期望命令；队列消耗与 waypoint 超时都在 QueueFollower 内（裁决 R4）
            command = goals.next(state, unwrapped)
            if command is None:                       # 目标源主动要求保持
                if not self.holding:
                    self.execution.stop()
                    self.holding = True
                self._pace(tick_at, self._period)
                continue
            self.require_command_type(command)        # 与 MotionCapability 不符即抛
            command = self.limiter.limit(command, state, self._period)
            sink.stream_command(command, self._period)
            self.holding = False
            self.recorder.record_control(state, command, self._head_id)
            self._pace(tick_at, self._period)
    except CollisionInterrupt as interrupt:
        self._recover_stream(interrupt, sink, budget)
        # 恢复后队列已清，目标源返回 None，循环继续等任务层补点
```

`_recover_stream` 之后不再进入 `except`，因为队列清空后循环靠 `goals.next` 返回 `None` 自然保持；`window.interrupt()` 把 revision 加一，任务层的 `poll_stream` 与 `submit` 都会看到新 revision。

### 2.4 逐拍时序：`run_segment`

```text
t=0.000  owner  _guard: 停止事件未置，deadline 未到，碰撞无接触
t=0.000  owner  current_pose -> 段重锚，duration 与剩余偏航重算
t=0.000  owner  预算预检：duration + settle <= remaining
t=0.000  owner  synthesizer.reset / limiter.reset(实测速度)
t=0.000  owner  sink.stream_command 第 0 拍（t=0，速度接近 0）
t=0.030  owner  _guard -> stream_state -> sample_segment(0.030)
t=0.030  owner  desired: 参考位姿 -> 世界速度 -> 按 capability 分型
t=0.030  owner  limit: 加速度/ jerk 球投影 + 偏航伺服
t=0.030  owner  sink.stream_command(cmd, 0.030)   # 裁决 R2
t=0.030  engine 执行一个 30 ms 周期，推进仿真时钟（采样循环在 loop.py，引擎不决定节奏）
...
t=T      owner  elapsed >= duration，退出采样循环
t=T      owner  execution.stop() -> cancel + hover
t=T      owner  hold.wait: 位置/偏航进容差并保持 settle_time_s
t=T+dt   owner  receipt.completed_segments += 1，处理下一段或返回
```

引擎侧只实现 `stream_command(command, duration_s)`（裁决 R1、R2）：`SetRoverControls` / `move_by_velocity` / `moveByVelocity` 三选一，外加把仿真推进一个周期（`planner/simulators/projectairsim/motion.py:67` 现在做的采样、反馈、限幅、预算检查全部上移到本层，tracker 整体删除）。

### 2.5 逐拍时序：`run_stream`（两个线程交错）

```text
owner 线程                                     planner-slow-model 线程
─────────────────────────────────────────────  ────────────────────────────────
t=0      _guard                                  （空闲，等 submit）
t=0      stream_state -> recorder.sample
t=0      goals.consume: 队头满足 reached -> pop
t=0      goals.next -> 期望命令
t=0      limiter -> sink.stream_command(cmd, 0.050)
t=0      recorder.record_control（10 Hz 采样）
t=0      pace: sleep(0.050 - 耗时)
t=0      window.poll_stream(pose, checkpoint)    
t=0      poll: future 未完成 -> 返回 None
t=0      window.submit(观测, 上下文, planning)      submit -> _model_call(snapshot)
t=0      pace 结束                                HTTP 在途（≤ request_deadline）
t=0.050  _guard -> stream_state                  future 完成，candidate 生成
t=0.050  goals.next -> 命令
t=0.050  poll_stream -> queue.append_continuation（revision+1）
t=0.050  sink.stream_command
...
```

一条硬性顺序：`queue.append_continuation` 与 `consume` 都只在 owner 线程，`SlowPlannerWorker` 与 `ModelCallService` 的线程从不接触队列、窗口与载具（`planner/capabilities/sliding_window/worker.py:1` 的模块说明已经如此，迁移后仍然如此）。

### 2.6 碰撞退避与两条循环的交互

批次中断当前段：

```text
owner  _guard -> recovery.poll(CollisionState(in_contact=True))
owner  抛 CollisionInterrupt(contact)
owner  execution.stop()：cancel + hover，确认停稳
owner  recovery.begin(contact, pose) -> BackoffPlan(distance = backoff_m * attempts)
owner  循环 stream_command(reverse command, 0.030)，直到累计位移 >= distance
owner  recovery.finish()：消费残留接触事件，attempts 已自增
owner  返回 ExecutionReceipt(completed_segments=k, paused=False,
                            remaining=从恢复位姿重锚的当前段 + 后续段)
任务层  拿到 receipt，自行决定重规划或报失败
```

连续清队列：

```text
owner  _guard -> recovery.poll -> CollisionInterrupt
owner  execution.stop()
owner  window.interrupt()：queue.clear()（revision+1）、inflight.discard()、record=None
owner  recovery.begin/finish：同样按 attempts 递增后退
owner  recorder.reset(恢复位姿)：历史从新位姿起算，path_length 累计不清
owner  limiter.reset / synthesizer.reset
任务层  下一拍 poll_stream 看到 revision 变化，重新取帧补点
```

退避预算按 `CollisionRecoverySettings.reset_after_progress_m` 复位：`note_progress` 在 `recorder.path_length_m` 超过基线加复位距离时把 `attempts` 归零（迁移自 `planner/workflows/free_navigation/continuous.py:264`）。`attempts > maximum_recoveries` 时抛 `ExecutionError("collision recovery budget exhausted")`，批次与连续都上抛到任务层。

## 3 跨层接口

### 3.1 我提供什么

```python
# runtime/motion/loop.py
class ControlLoop:
    def run_segment(self, segment: MotionSegment, sink: SegmentSink, *,
                    budget: ExecutionBudget | None) -> ExecutionReceipt: ...
    def run_stream(self, goals: StreamGoalSource, sink: StreamSink, *,
                   deadline_monotonic: float | None) -> None: ...

# runtime/window/window.py
class PlanningWindow:
    def begin(self, *, deadline_monotonic: float | None, maximum_rounds: int | None) -> None: ...
    def submit(self, observation, context, instruction, planning) -> None: ...
    def poll_stream(self, measured: WorldPose, checkpoint) -> tuple | None: ...
    def pause_for_completion(self, checkpoint) -> None: ...
    def complete(self) -> None: ...
    def end(self) -> None: ...

# runtime/execution.py
class ExecutionService:
    def acquire_flight(self, owner) -> None: ...   # 裁决 R5：无 collision_check
    def release_flight(self, owner, pending_cleanup=None) -> None: ...
    def request_stop(self) -> None: ...
    def set_deadline(self, deadline: float | None) -> None: ...
    def checkpoint(self) -> None: ...
    def stop(self) -> StopResult: ...

# runtime/collision_recovery.py
class CollisionRecovery:
    def poll(self, state: CollisionState) -> CollisionContact | None: ...
    def begin(self, contact: CollisionContact, pose: WorldPose) -> BackoffPlan: ...
    def finish(self) -> None: ...
```

三个协议由契约层声明，本层实现或消费：

```python
# ports/control_loop.py（请求 01 契约层收；裁决 R2：只有一个下发名字）
class CommandSink(Protocol):
    def stream_command(self, command: MotionCommand, duration_s: float) -> None: ...

SegmentSink = CommandSink      # runtime 内部别名，不出现 apply
StreamSink = CommandSink

class StreamGoalSource(Protocol):
    def next(self, state: TimedVehicleState, unwrapped_yaw_deg: float) -> MotionCommand | None: ...
```

`StreamGoalSource.next` 的第二个参数是在契约签名上多出来的：契约层给的是 `next(state)`，而偏航必须用不折返的连续角，`WorldPose.yaw_deg` 会跨 ±180 跳变。请求 01 契约层把 `next` 定为 `next(state, unwrapped_yaw_deg)`，或把连续偏航放进 `TimedVehicleState`。

`SegmentSink` 与 `StreamSink` 是同一签名的两个名字，区别只在下发周期与是否允许 `None` 目标，没有方法差异；保持空闲用零速度命令表达，停稳用 `ExecutionService.stop()`（裁决 R2）。

### 3.2 我要求相邻层提供什么

| 相邻层 | 我要求的东西 | 形状 |
| --- | --- | --- |
| `domain/` | `MotionModel`、`MotionCapability`、`HolonomicCommand`、`CarCommand`、`MotionCommand`、`CollisionContact`、`CollisionState`、`CollisionRecoverySettings`、`ExecutionBudget`、`ExecutionReceipt`、`ExecutionRecord`、`StopResult`、`MotionSegment`、`MotionTrajectory`、`WorldPose`、`resolve_yaw_delta_deg`、`wrap_yaw_deg`、`TimedVehicleState`、`QueuedWaypoint`、`PlanningSnapshot`、`PlanningWindowInput`、`SlidingWindowSettings`、`ContinuousTrackingSettings`、`MotionLimits`（只留一份） | `00-boundaries.md` 第四节与契约层方案 |
| `ports/` | `Vehicle.current_pose/motion_capability/execute_segment/cancel/hover/collision_state -> CollisionState`（裁决 R1：槽最终由 runtime 的 `GuidedVehicle(streaming, loop)` 兑现，引擎侧只提供 `current_pose`/`motion_capability`/`collision_state`/`cancel`/`hover` 与 `stream_command`）；`StreamingVehicle.stream_state/stream_command`；`PositionHoldVehicle.stream_state/cancel/hold_velocity`（裁决 R7：`hold_velocity` 内部转调 `stream_command`）；`TrajectoryPlanner.plan`；`ObservationSource.capture`；`MovingObservationSource.capture_moving` | 现有 14 个端口，`collision_check.py` 删除（裁决 R5） |
| `algorithms/` | `corridor_for(model, limits)`（裁决 R6：`ACKERMANN` 0.6 m，`HOLONOMIC` 取 `MotionLimits.position_tolerance_m`）、`heading_rate_deg_s`、`braking_speed`、`planned_speed`、`corner_speed`、`reached`、`duration_for`、`sample_segment`、`translation_handoff`、`ContinuousPathReference`、`world_to_relative`、`relative_to_world`、`require_execution_anchor`、`RefillScheduler` | 现 `services/motion/` 与 `capabilities/sliding_window/scheduler.py` 的纯函数部分，加 `algorithms/motion/corridor.py` |
| `workflows/` | 实现或组装 `StreamGoalSource`（默认直接用 `QueueFollower`，只补暂停策略钩子，裁决 R4）；每阶段一个 `PlanningWindowInput`；调 `window` 六个方法；把 `delay`、`maximum_rounds` 交给 `window.begin` | 见任务层方案 |
| `assembly/` | 按载具造一套 `ControlLoop`：`capability = vehicle.motion_capability()`、`limits` 从 `config.motion` 一次构造、`tracking` 从 `config.continuous`、`recovery` 从 `config.motion.collision`（开关只有 `motion.collision.enabled`，默认 true；删 `visual_goal_navigation.collision_check_enabled`，裁决 R8）、注入 `clock`/`sleep`；用 `GuidedVehicle(streaming, loop)` 填 `Vehicle` 槽（裁决 R1） | 见装配层方案 |
| `simulators/` | 只实现执行器：`stream_command(command, duration_s)` 按 `CarCommand` / `HolonomicCommand` 落地并推进一个周期；收到与 `MotionCapability` 不符的命令抛错，不许静默丢分量 | 现 `drive.py` 的投影必须改成校验 |

### 3.3 时钟与节拍的注入点

| 位置 | 现在 | 目标 |
| --- | --- | --- |
| `ExecutionService(clock)` | `planner/services/execution.py:29` | 不变，仍是 `time.monotonic` 默认 |
| `ControlLoop(clock, sleep)` | 无 | 新增；`sleep` 与 `clock` 都可替换，测试用假时钟 |
| `PlanningWindow(clock)` | `planner/services/planning_window.py:66` | 不变；与 `ControlLoop` 共用同一个实例 |
| `PositionHold(clock, sleep)` | `planner/services/position_hold.py:21` | 不变 |
| `StreamRecorder(clock)` | 新 | 由 `ControlLoop` 注入，测试断言节拍用 |
| 周期 | `1/limits.control_hz` 散落各处（`planner/services/continuous_execution.py:125`、`planner/services/position_hold.py:68`） | 只有两处：`SEGMENT_PERIOD_S` 与 `1/MotionLimits.control_hz`，都在 `loop.py`，不进配置（裁决 R9） |

## 4 错误与失败路径

### 4.1 抛出与捕获矩阵

| 异常 | 谁抛 | 谁捕获 | 恢复动作 |
| --- | --- | --- | --- |
| `ExecutionError("flight stop requested")` | `ExecutionService.check_deadline`（`planner/services/execution.py:100`） | 任务层 `run` | `stop()` 已执行；`StageResult(False, reason)` |
| `BudgetExceeded("stage time budget exhausted")` | `_guard` / `check_deadline`（`:103`） | 任务层 | 同上；剩余运动写进 `StageResult` |
| `BudgetExceeded("stream deadline exhausted")` | `run_stream` 的 `_guard` | 任务层 | 不静默返回，任务层映射 TIMEOUT（裁决 R3） |
| `BudgetExceeded("deadline before segment")` | `run_segment` 预算预检（现 `planner/services/execution.py:148`） | 任务层 | receipt 带 `completed_segments` 与 `remaining` |
| `CollisionInterrupt` | `_guard` | `ControlLoop` 自己 | 退避后批次返回 partial receipt，连续清队列继续 |
| `ExecutionError("collision recovery budget exhausted")` | `CollisionRecovery.begin` | 任务层 | 停止并报失败，理由含对方名称与次数 |
| `ExecutionError("nested motion execution is not allowed")` | `run_segment` 重入检查（现 `:129`） | 调用方 bug，直达任务层 | 不恢复 |
| `ExecutionError("simulator clock moved backwards")` | `run_stream` 采样（现 `planner/services/continuous_execution.py:62`） | 任务层 | 停止；不自动重试 |
| `ExecutionError("continuous waypoint timed out before position/yaw progress")` | `QueueFollower`（现 `:121`；裁决 R4） | 任务层 | 停止；任务层决定重规划或失败 |
| `ExecutionError("non-finite yaw control command")` | `algorithms.heading_rate_deg_s` 的校验（`planner/services/motion/heading_servo.py:85`） | 循环 | 停止；该拍命令不下发 |
| `ExecutionError("world queue may only be accessed by its execution owner")` | `WorldWaypointQueue.check_owner`（`planner/services/trajectory_queue.py:28`） | 违约线程 | 不恢复，属于实现缺陷 |
| `ProtocolError("continuation belongs to an invalidated queue or batch")` | `append_continuation`（`:105`） | `poll_stream` | 记 `planning_discarded`，`stream_retry_at = now + 1s`，队列不变 |
| `ModelCallTimeout` | `inflight.poll_stream` 请求截止（现 `planner/services/planning_window.py:189` 之后的截止分支） | 任务层 | 迟到结果隔离，不允许自动重试同一个 future |
| `ProtocolError("repeated stream planning failures")` | `poll_stream` 失败计数达 `max_consecutive_plan_failures` | 任务层 | 阶段失败 |
| `ValueError("command type does not match MotionCapability")` | `ControlLoop.__init__` 与 `require_command_type` | 构造期或第一拍 | 构造期即报，不进运行期（`00-boundaries.md` 第五节） |
| `ResourceCleanupError` | `close_resources`（`planner/services/resource_cleanup.py:7`） | 装配层 | 每个资源尝试一次，聚合上报 |
| `KeyboardInterrupt` | 任意 | 循环 | `stop()` 后原样上抛，receipt 挂到异常上 |

### 4.2 三条硬规则

1. 任何异常离开 `run_segment`／`run_stream` 之前必须先 `execution.stop()`。现在 `planner/services/execution.py:196` 已经是这个结构，迁移时保留。
2. 队列在任何失败后处于两种状态之一：`pause_stream` 快照有效（可在同一阶段恢复）或已 `clear`（revision 变化，推测全部作废）。不允许"部分消费但 revision 未变"。
3. 迟到结果只隔离，不重试同一个请求。`BackgroundContinuationPlanner.discard()`（`planner/capabilities/sliding_window/planning.py:128`）取消 future 并把 job 置空，`ModelCallService` 的 `_busy` 在真实返回前保持真（`planner/services/model_calls.py:36`）。

### 4.3 撞到上限时的收尾顺序

```text
1  execution.checkpoint() 已抛（停止或预算）
2  ControlLoop finally: self._running = False
3  execution.stop() -> cancel() + hover()，各自 try/except 聚合进 StopResult.errors
4  window.pause_for_completion(checkpoint) 已被调用方执行，或 window.end()
5  position_hold.start() 接管，保持到下一次任务
6  release_flight(owner) 清 deadline 并放开飞行租约
```

第 3 步任一个动作失败时 `StopResult.errors` 非空，任务层据 `errors` 判失败，不把"API 调用成功"当成"物理停稳"（现 `planner/services/execution.py:253` 的注释已如此约定）。

## 5 并发与所有权

### 5.1 线程清单

| 线程 | 名字 | 谁起 | 碰什么 | 不许碰 |
| --- | --- | --- | --- | --- |
| owner | worker 线程（`ThreadedMissionWorker`）或 UE4 子进程主线程 | `simulator/worker.py` / `mission_child.py` | 载具端口、队列、窗口、`PositionHold`、`ExecutionService` | — |
| 模型 worker | `planner-slow-model` | `SlowPlannerWorker.submit`（`planner/capabilities/sliding_window/worker.py:28`） | 只跑 `TrajectoryPlanner.plan` | 队列、载具、窗口 |
| 模型调用 | `planner-model-call` | `ModelCallService.call`（`planner/services/model_calls.py:66`） | 只跑模型 I/O | 队列、载具 |
| 数据操作 | `planner-data-operation` | `BackgroundOperation.start`（`planner/services/background_operation.py:54`） | 只跑取帧/观察 | 队列、载具 |
| 读清理 | `planner-read-cleanup` | `BackgroundOperation.close`（`:96`） | 等读线程结束后关传输 | 载具 |
| HTTP 读 | FastAPI 请求线程 | 服务框架 | `FrameHub.read`、`TrajectoryHub.read`、`LogBuffer.read` | 队列、载具 |
| 相机回调 | 引擎侧回调线程 | 引擎包 | `FrameHub.publish` | 其余全部 |

owner 身份在四个对象里各记一次：`ExecutionService._owner`（`planner/services/execution.py:32`）、`WorldWaypointQueue._owner`（`planner/services/trajectory_queue.py:23`）、`BackgroundContinuationPlanner._owner`（`planner/capabilities/sliding_window/planning.py:45`）、`PositionHold.owner`（`planner/services/position_hold.py:29`）。四者必须在同一线程构造。装配层断言一次即可，四个对象不必互相知道。

### 5.2 锁与阻塞

- `SlowPlannerWorker` 的 `threading.Condition`（`:12`）只在 `submit`、`_run`、`close` 三处进出，持锁时不调任何外部代码。
- `ModelCallService`、`BackgroundOperation` 的 `_lock` 只保护 `_busy` 与 future 完成。future 的 `set_result` 在锁内，保证"看到 `done()` 就能立刻提交下一个"。
- owner 线程等结果一律用 `checkpoint()` 轮询加 20 ms 事件等待（`planner/capabilities/sliding_window/planning.py:124`），不持任何锁。
- 汇类对象各有一把独立的锁，owner 线程只是写入方之一，不进 `checkpoint`，不阻塞载具调用。

### 5.3 控制权交接：三种情况

切换载具（`/api/vehicle`，仅 idle）：

```text
controller  idle 才接受；置 connecting
controller  arm_old.execution.request_stop()        # 置事件，不等
owner       下一个检查点（≤ 50 ms）抛 ExecutionError
owner       ControlLoop finally: stop() -> cancel + hover
owner       position_hold.release()
owner       execution.release_flight(arm_old)
controller  旧载具 park()：抬到 1 km 并钉住
controller  session.activate(new_vehicle)           # 不重启引擎
controller  新 arm 的 owner 线程身份不变，ExecutionService 重新绑定 owner
owner       position_hold.start() 在新生效位姿上接管
controller  idle
```

暂停（`/api/stop` 或阶段内 `pause_when`）：

```text
controller  request_stop()：只置 threading.Event，不收任何锁
owner       下一个 checkpoint 抛 ExecutionError("flight stop requested")
owner       stop()：cancel + hover，校验 hover 生效
owner       window.pause_for_completion(checkpoint)：等待并丢弃在途候选，
            保留已授权队列（queue.pause_stream 快照）
owner       position_hold.start(target=当前位姿)
owner       release_flight；任务层把 receipt 写进 StageResult
恢复       同一阶段内 restore_stream(snapshot, stage_token) 校验 revision 与端点一致才继续
```

完成确认（目标接近的到达判定）：

```text
owner       队列清空且 last command 已停 -> execution.stop()
owner       position_hold.wait()：位置与偏航进容差并保持 settle_time_s（现
            planner/services/position_hold.py:142）
owner       window.pause_for_completion(checkpoint)：丢弃推测规划，保留授权队列
owner       取一帧停稳观测（ControlledObservationSource 或 ObservationSource.capture）
owner       任务层语义确认（完成判定端口）
owner       window.complete()：end 掉 inflight，清队列
owner       release_flight；position_hold 继续持有直到下一任务
```

三种情况的共同约束：交接期间只有 owner 线程发指令；`request_stop` 与 `release_flight` 之间不允许第三方 `acquire_flight`，`_pending_cleanup` 检查（`planner/services/execution.py:61`）挡的是"上一笔请求还在跑"。

## 6 测试点

| # | 测试 | 断言对象 | 负例 |
| --- | --- | --- | --- |
| 1 | `test_control_loop_segment.py`：单段执行 | 假 sink 收到的 `stream_command` 间隔 0.030 s，次数 = `ceil(duration/0.030)`，最后一个命令速度范数 ≤ `stopping` 阈值 | 空 `MotionTrajectory` → `ExecutionError` |
| 2 | 同上：段内预算耗尽 | `receipt.completed_segments == k`，`receipt.remaining.segments[0].start` 等于实测位姿 | 异常未带 receipt → 失败 |
| 3 | 同上：重入 | 嵌套 `run_segment` 抛 `ExecutionError` | 第二次调用不得发出任何 `stream_command` |
| 4 | `test_control_loop_stream.py`：节拍 | 假时钟下 20 拍耗时 1.0 s，`stream_command` 第二参数恒为 0.05；deadline 到期抛 `BudgetExceeded`（裁决 R3） | 目标源返回 `None` 时不发非零命令，`stop()` 调用一次 |
| 5 | 同上：命令类型 | `capability.model == ACKERMANN` 时给 `HolonomicCommand` → 构造期 `ValueError` | 无人机收 `CarCommand` 同样报错 |
| 6 | `test_motion_model_commands.py`（扩展 `tests/test_motion.py` 与 `tests/test_passed_corridor.py`） | `corridor_for(ACKERMANN)` 返回 0.6 m、`corridor_for(HOLONOMIC)` 返回 `position_tolerance_m`（裁决 R6）；阿克曼在"冲过端点、残差侧向"状态下 `CarCommand.forward_speed_mps >= 0`，且永不产生横向分量；无人机对照允许横向 | 阿克曼出现 `reverse=True` 且未越点 → 失败 |
| 7 | `test_command_limiter.py`（扩展 `tests/test_heading_servo.py`） | 阶跃目标下加速度范数 ≤ `max_acceleration_mps2`、jerk ≤ `max_jerk_mps3`，偏航角速度 ≤ `max_yaw_rate_deg_s` | `NaN` 输入 → `ExecutionError`，且整段零下发 |
| 8 | `test_collision_recovery.py`（用 `tests/test_visual_goal_collision_recovery.py` 的假载具） | 第 n 次接触后退距离 = `backoff_m * n`；`attempts` 超上限抛 `ExecutionError`；批次 receipt 段数不变 | 恢复期间再接触不得连乘两次退避 |
| 9 | 同上：预算复位 | 前进 `reset_after_progress_m` 后 `attempts == 0` | 未达阈值时 `attempts` 保持 |
| 10 | `test_runtime_ownership.py` | 非 owner 线程调 `queue.load_prefix`、`window.poll_stream`、`execution.checkpoint` 各抛 `ExecutionError` | owner 线程同一调用成功 |
| 11 | `test_planning_window.py`（沿用 `tests/test_batch3_services.py` 的假规划器） | `poll_stream` 只返回候选，不写队列；`append_continuation` 后 revision +1；`pause_for_completion` 期间在途 future 被 discard | 候选晚于 `request_deadline` → `ModelCallTimeout`，队列不变 |
| 12 | 同上：授权边界 | `next_world` 在队列非空时抛 `ProtocolError`；`load_prefix` 只接受 1..5 点 | 六点前缀 → `ProtocolError` |
| 13 | `test_stream_timeline.py` | 同一 `timestamp_ns` 覆盖写、时间倒退抛 `ProtocolError`、超过 `maximum_gap_ns` 拒绝锚点 | 阶段 token 不符 → `ProtocolError` |
| 14 | `test_position_hold.py`（已有，迁移导入） | `observable` 要求整窗进容差；`wait` 超时抛 `ExecutionError` | 只在一瞬间对齐 → `False` |
| 15 | `test_frame_hub.py`、`test_trajectory_hub.py`、`test_log_buffer.py`（已有） | 限流丢帧、`adopt` 覆盖 revision、游标截断标记 | — |
| 16 | `test_layering.py`（`docs/REFACTOR.md` 验收项） | `runtime/` 不 import `workflows/`、`assembly/`、`simulators/`；`algorithms/` 不 import `runtime/` | 任一反向 import → 失败 |
| 17 | `test_resource_cleanup.py` | 每个回调都执行一次，失败聚合；`KeyboardInterrupt` 优先上抛 | 只抛第一条失败 → 失败 |

测试用假件统一放 `tests/support.py`：`FakeClock`、`RecordingSink`、`ScriptedGoalSource`、`FakeVehicle`（可注入 `CollisionState` 序列）。现有 `tests/support.py` 已有假载具骨架，扩展即可。

## 7 迁移步骤

每步独立可发布、可验证；前四步不改对外行为。

| 步 | 做什么 | 验证 |
| --- | --- | --- |
| 1 | 纯搬迁：`services/` → `runtime/`；`services/planning_window.py` → `runtime/window/window.py`；`trajectory_queue.py` → `runtime/window/queue.py`；`capabilities/sliding_window/{worker,planning}.py` → `runtime/window/`。只改 import 与包名 | `pytest tests/ -q` 全绿；新增 `test_layering.py` 通过 |
| 2 | 纯函数下沉：`services/motion/{heading_servo,continuous_speed,continuous_reference,hold_handoff,synchronized,base}.py`、`trajectory_transform.py`、`trajectory_validation.py`、`reached` 移到 `algorithms/`；`runtime` 改 import | 同上；`tests/test_motion.py`、`tests/test_heading_servo.py` 不改断言 |
| 3 | 合并 `MotionLimits`：删 `services/segment_motion.py` 的同名类，`to_tracking_limits` 进 `domain/motion.py`；`runtime/__init__.py` 不再导出 `MotionLimits` | `tests/test_config.py`、`tests/test_motion.py`、`tests/test_passed_corridor.py` |
| 4 | 端口与引擎加 `motion_capability()` 与 `stream_command`；`drive.py` 的横向投影改成类型校验；删 `ROVER_PASSED_LATERAL_TOLERANCE_M` 与 `rover_tracking_limits()`，走廊改取 `algorithms/motion/corridor.py`（裁决 R6）。本层只消费能力位，不写引擎 | `tests/test_projectairsim_drive.py`、`tests/test_projectairsim_streaming.py`；给车喂横向命令必须抛错 |
| 5 | 引入 `ControlLoop` 外壳：`run_segment` 内部先调现有 `execute_with_receipt`，`run_stream` 内部先调现有 `ContinuousExecution.tick`。对外已经是新签名 | 三类任务各跑一次，`telemetry` 与重构前逐行比对 |
| 6 | 把律搬进 `motion/tracking.py` 与 `motion/limits.py`，`run_segment` 自己采样五次曲线并 `sink.stream_command`；删 `SteppedMotionTracker` 整体，引擎只剩 `stream_command`（裁决 R1、R2）；过渡期允许引擎保留三行委派，随本步一起删 | `tests/test_projectairsim_motion.py` 改造成假 sink 断言；UE5 数值动作位置误差不劣于现状 |
| 7 | 加 `CollisionRecovery`，先与 `free_navigation` 现逻辑并存、日志对比后删两份镜像与 `visual_goal` 的碰撞预检分支 | `tests/test_visual_goal_collision_recovery.py`、`tests/test_free_navigation.py`；接触后退、清队列、继续 |
| 8 | 收敛 `PlanningWindow` 到六个公开方法：`submit_stream`→`submit`，`stream_due`/`prefetch`/`next_world`/`executed` 只由 `ControlLoop` 调用；删 `continuous_execution.py` | 两类任务端到端；`/api/trajectory` 输出逐字段一致 |

第 5 步是关键分界：在此之前行为等价，在此之后引擎不再决定节奏。

## 8 未决问题

### 1 越点走廊的取值（裁决 R6）

- 裁决：常量放 `algorithms/motion/corridor.py`，按 `MotionModel` 取值：`ACKERMANN` 0.6 m，`HOLONOMIC` 取 `MotionLimits.position_tolerance_m`；`config.motion.passed_lateral_tolerance_m` 可覆盖；不从 `min_turn_radius_m` 推导。`MotionCapability` 不加走廊字段。
- 落点：删 `planner/simulators/projectairsim/rover.py:48` 的 `ROVER_PASSED_LATERAL_TOLERANCE_M` 与 `rover_tracking_limits()`（`planner/simulators/projectairsim/rover.py:56`）。
- 不采纳：按 `min_turn_radius_m` 推导（原默认，标定值与真实越点走廊脱钩）；走廊留在引擎适配层（两个引擎各写一份）。
- 残余不确定：0.6 m 与车宽的对应关系没有实测记录，直接取 `rover.py:48` 的现值。

### 2 碰撞检查的挂载点（裁决 R5）

- 裁决：`ControlLoop._guard` 调 `CollisionRecovery.poll`，接触抛 `CollisionInterrupt` 由循环退避；`ExecutionService.checkpoint()` 只管停止与预算；`ports/collision_check.py` 删除，`acquire_flight` 去掉 `collision_check` 参数。
- 不采纳：`ExecutionService` 持 recovery 并在 `checkpoint()` 内 raise（职责变宽，与“只管停止与预算”冲突）；检查点内同步跑完退避（递归进运动循环）。
- 残余：任务层在循环外调 `checkpoint()` 时不会触发退避，`docs/REFACTOR.md` 里“`checkpoint()` 是碰撞退避触发点”的说法要随本次裁决改口径。

### 3 队列清空的责任人

- 默认：`ControlLoop` 调 `PlanningWindow.interrupt()`（层内方法：`queue.clear()`、`inflight.discard()`、`record=None`），窗口仍是队列唯一门面。
- 选项 B：`ControlLoop` 直接持 `WorldWaypointQueue`，跳过窗口。
- 选项 C：`CollisionRecovery` 通过回调清队列，任务层注入回调。
- 取舍：默认保封装，代价是窗口多一个对外不可见的 `interrupt()`；B 打穿窗口，任务层与执行层的队列视图可能不一致；C 把被动技能外泄到任务层，违反 `00-boundaries.md` 第一节的决定。

### 4 `StreamGoalSource` 的组装位置（裁决 R4）

- 裁决：`runtime/motion/follower.py` 的 `QueueFollower` 实现 `StreamGoalSource`，含队列消耗与 waypoint 超时；`PlanningWindow` 只提供队列视图、`consume` 与六个公开方法，不实现目标源、不承担伺服；任务层把 `QueueFollower` 传给 `run_stream`。
- 不采纳：目标源完全由任务层实现（两份队列跟随）；`ControlLoop` 内置窗口驱动（与 `run_stream(goals, sink, deadline)` 签名不符）。
- 残余：`QueueFollower` 何时返回 `None`（暂停策略）仍由任务层决定，需要一个策略钩子，形状待任务层方案确认。

### 5 两个周期（裁决 R9）

- 裁决：`SEGMENT_PERIOD_S = 0.030` 与 `1/MotionLimits.control_hz` 都写在 `runtime/motion/loop.py`，不进配置。
- 不采纳：配置新增 `segment_period_s`/`stream_period_s`（第二处事实）；周期由引擎报告（引擎重新掌握节奏，与 `00-boundaries.md` 相反）。

### 6 `PositionHold` 的归属

- 默认：`runtime/position_hold.py`，由 `ControlLoop` 在停稳验证与 `pause_for_completion` 之前驱动，任务层通过 `execution.checkpoint` 间接使用。
- 选项 B：并进 `loop.py`，成为 `ControlLoop` 的内部状态。
- 选项 C：留在任务层，由各任务的循环自己调（现状 `visual_goal_navigation` 的做法）。
- 取舍：默认保住"一个 owner 一份实现"，文件不致过大；B 减少一层间接，但 `loop.py` 会到 600 行以上；C 三类任务各写一遍停稳等待，且停稳期间的控制权归属重新变模糊。需 04 任务层确认没有阶段内自定义 hold 需求。
