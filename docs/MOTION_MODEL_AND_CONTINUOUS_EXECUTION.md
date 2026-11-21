# 连续执行、运动模型与载具端口

本文回答四个问题：

1. 连续执行（`continuous`）的逻辑，无人机和无人车是不是共用一份？
2. 不同仿真器对规划器暴露的接口是什么、具体执行写在哪？
3. "越过航点→后退→原地空转卡死"的根因因果链到底是什么？
4. 怎样保证不会再越过航点、不会后退，以及接口该怎样设计，从结构上杜绝"无人机能横移、无人车不能"这类错误？

> 与 `docs/ARCHITECTURE.md` 第 4 节（端口与能力推导）、第 11 节（运动与航向控制）配套阅读。本文只讲**运动执行**这条线，把这条线上"共享/分叉"的边界讲清楚。

---

## 1 结论速览

| 问题 | 结论 |
| --- | --- |
| 连续跟随逻辑是否共用？ | **共用一份。** `ContinuousExecution` + `ContinuousFollower` 同时服务无人机、无人车，也同时服务 `visual_goal_navigation` 与 `free_navigation` 两个工作流。 |
| 不同仿真器接口是否相同？ | **相同。** 规划器只认 `StreamingVehicle` 端口（`stream_state` / `stream_velocity`）。具体"把速度变成轮胎/桨的动作"写在每个仿真器的载具类里。 |
| 差异写在哪？ | 每个仿真器包里的 `stream_velocity` 实现。无人车走 `drive.py`（阿克曼），无人机走 `move_by_velocity_async(..., MaxDegreeOfFreedom)`（全向）。 |
| 后退是不是 VLM 给的？ | **不是。** VLM 只给前进航点。后退来自连续跟随器：目标点落到车身后方时，`desired = 误差方向 × 速度` 指向后方，执行器前向投影为负→倒车。 |
| 为什么会空转卡死？ | 倒车后残差几乎全在**侧向**。跟随器是全向的，命令一个纯横向速度；无人车执行器只取前向投影、丢掉横向→油门≈0→既不横移也不前进→航点的 0.3 m 判据永远满足不了→超时。 |
| 结构上缺什么？ | 端口 `stream_velocity(world_velocity, yaw_rate)` 是**全向速度契约**，但没有声明载具的**运动模型**，无人车适配层把横向分量**静默丢弃**。这是"能横移/不能横移"错误能潜伏的根。 |

---

## 2 连续执行是共用的

两个工作流用**同一个** `ContinuousExecution` 构造调用：

- `planner/workflows/visual_goal_navigation/continuous.py:38`
  ```python
  self.stream = ContinuousExecution(execution, execution.vehicle, motion_limits, speed_mps, clock, ...)
  ```
- `planner/workflows/free_navigation/continuous.py:100`
  ```python
  self.stream = ContinuousExecution(execution, execution.vehicle, motion_limits, speed_mps, clock, ...)
  ```

`ContinuousExecution`（`planner/services/continuous_execution.py:19`）内部持有 `ContinuousFollower`（`…:24`），而 `ContinuousFollower` 是纯算法（`planner/services/motion/continuous.py`），不认识仿真器、也不区分载具。它每拍产出 `(velocity_world, yaw_rate)`，交给 `self.vehicle.stream_velocity(velocity, rate, dt)`（`continuous_execution.py:124-125`）。

所以"规划器→轨迹队列→跟随→下发"这整条链，**无人机与无人车、两种任务完全一致**。载具差异全部收敛到 `self.vehicle` 这个端口对象上。

装配层确认这一点：`planner/application/mission_assembly.py:220`
```python
execution = ExecutionService(vehicle) if execution is None else execution
```
每个载具一套 `VehicleArm`（`mission_assembly.py:196`），`execution.vehicle` 就是当前活动载具的 `StreamingVehicle`。切载具即切 `vehicle`，上层代码不动。

---

## 3 仿真器对规划器暴露的接口

### 3.1 一个端口，两套实现

端口只有两个方法（`planner/ports/streaming_vehicle.py`）：

```python
class StreamingVehicle(Protocol):
    def stream_state(self) -> TimedVehicleState: ...
    def stream_velocity(self, velocity_world: tuple[float, float, float],
                        yaw_rate_deg_s: float, duration_s: float) -> None: ...
```

三种实现（都由 `stream_velocity` 落地）：

| 实现 | 文件 | 底层驱动 | 运动约束 |
| --- | --- | --- | --- |
| ProjectAirSim 无人车 | `planner/simulators/projectairsim/rover.py:235` | `controls_from_reference`（`drive.py`）→ `SetRoverControls(engine, steering_angle, brake)` | **阿克曼：只能前进/后退 + 转向，不能横移** |
| ProjectAirSim 无人机 | `planner/simulators/projectairsim/drone.py:556` | `move_by_velocity_async(vx,vy,vz, MaxDegreeOfFreedom)` | **全向：任意方向速度** |
| AirSim 无人机 | `planner/simulators/airsim/vehicle.py:208` | `moveByVelocityAsync(..., MaxDegreeOfFreedom)` | **全向：任意方向速度** |

### 3.2 无人车的落地：横向被"投影掉"

`planner/simulators/projectairsim/drive.py:156-161`：

```python
north, east = heading_vector(yaw_deg)              # 车头方向
command = velocity_world[0]*north + velocity_world[1]*east   # 只取世界速度在车头上的投影
measured = measured_velocity_world[0]*north + measured_velocity_world[1]*east
engine = _clamp(command / model.speed_per_engine_mps + correction, -1.0, 1.0)
```

- `command` 是**期望速度沿车头方向的标量分量**。世界速度里垂直于车头的横向部分**根本没进 `engine`**——它被直接忽略了。
- 当期望速度指向车头**反方向**，`command < 0`，`engine < 0` → 倒车（`drive.py:163-168` 区分"带刹车的减速"与"从静止倒车"）。
- 转向来自 `steering_for(yaw_rate, speed)`（`drive.py:80-86`），**只由 `yaw_rate` 驱动**，与位置误差无关。

### 3.3 无人机的落地：全向照单全收

`planner/simulators/projectairsim/drone.py:605-612`：

```python
task = await self._robot.move_by_velocity_async(
    velocity_world[0], velocity_world[1], velocity_world[2],
    yaw_control_mode=YawControlMode.MaxDegreeOfFreedom,
    yaw_is_rate=True, yaw=math.radians(yaw_rate_deg_s))
```

无人机把 `velocity_world` 的**三个分量全部实现**——包括横向、包括倒飞。

> **这就是分叉点**：同一个 `stream_velocity(velocity_world, yaw_rate)` 契约，无人机完整兑现，无人车只兑现前向投影。而端口**没有声明**"这台载具能不能横移"，所以这个差异是**隐式**的，上层无从得知。

---

## 4 越过航点→后退→空转卡死的因果链

### 4.1 跟随器是全向的

`planner/services/motion/continuous.py:70`：
```python
error = tuple(v - p for v, p in zip(reference.position, (pose.x, pose.y, pose.z)))
```
`planner/services/motion/continuous.py:87`：
```python
desired = tuple(v/norm*speed for v in error) if norm else (0.0, 0.0, 0.0)
```

- `reference.position` 是"胡萝卜点"：正常时是沿路径前视一段的采样点（`continuous_reference.py:49-54`），临近终点/对齐时是端点 `b`。
- `desired` 的方向 = **从当前位置指向胡萝卜点的单位向量**，大小 = 规划速度 `speed`（`planned_speed` 返回 `speed ≥ 0`）。
- 也就是说：**期望速度朝哪，只取决于目标点在哪，与车头朝向无关。** 这是"全向载具"才成立的假设。

### 4.2 冲过航点 → 目标落到身后 → `desired` 指向后方

车以约 3 m/s 逼近 waypoint 3，刹车距离不够（后轮无摩擦进一步削弱了制动，见 §6），冲过航点。此时端点 `b` **落到车身后方**，向量 `error` 指向后方，`desired` 变成指向后方的速度。

- 无人机：`move_by_velocity` 朝后飞，飞回点附近即可收敛，**不会卡**。
- 无人车：`command = desired · 车头 < 0` → `engine < 0` → **物理倒车**（实测峰值约 −1.5 m/s）。

> 所以"后退"不是 VLM 给的，也不是谁显式下了倒车指令；它是**全向跟随器生成了一个指向后方的速度**，执行器忠实地把它的负前向投影变成了倒车 throttle。

### 4.3 倒回来以后：侧向残差 + 全向命令 → 死锁

倒回到离 waypoint 约 0.36 m 时，残差方向几乎是**垂直于车头**的：

```
车在 (12.10, −16.79)  yaw ≈ −6°
航点在 (11.99, −16.42)
残差 ≈ 横向 +0.37 m，前后 −0.11 m
```

因为车头 yaw 已经对齐航段朝向（`yaw_error ≈ 0.02°`，见 `continuous.py:67-68` 只跟 `goal_yaw`），跟随器只会：

- 命令一个几乎纯横向的世界速度：`vx ≈ 0.036、vy ≈ 0.340`（横向 0.34 m/s，前向分量 ≈ 0）。
- 无人车执行器取前向投影：`0.036·cos(−6°) + 0.340·sin(−6°) ≈ 0.000` → **engine ≈ 0**。
- 车不会横移（阿克曼），也没油门前进 → **原地不动**。

同时，航点弹出判据 `reached()`（`planner/services/motion/continuous.py:12-23`）要求"距端点 ≤ `position_tolerance`（默认 0.3 m，`planner/domain/motion.py:47`）**或** 已驶过端点且横向 ≤ 0.3 m"，且朝向对齐。残差 0.388 m、横向 0.37 m，**两条都过不了**，队列头永远弹不掉。

结果：靠 `planner/services/continuous_execution.py:121-122` 的到达超时（`waypoint_timeout_s`，默认 60 s，本任务日志里累计约 70 s）抛出 `execution_error`。

### 4.4 空转的视觉来源

卡死阶段的 `engine` 已经≈0，轮子本身不该有动力空转。你看到的"轮胎一直转"更可能来自：

1. **后轮无摩擦（`coeff-of-friction: 0.0`，且 `engine: true`）**：后轮是驱动轮却没有抓地力，任何给油时段（前进/倒车/刹车）都只会打滑空转，不产生地面推力；无摩擦也意味着没有滑动阻力，轮子转起来后久久不停。这与本项目此前在**交流中心**修过的缺陷是同一个（交流中心后轮已改 `1.0`，**郊区当时仍是 0.0**，已同步修正）。
2. 真正的"冻死"来自 §4.3 的横向死锁（engine≈0）。

> 结论：**后退是控制器航向/速度设计问题；卡死是"全向命令投到阿克曼执行器"问题；空转是后轮摩擦为 0 的建模缺陷。** 三者叠加成一次任务失败。

---

## 5 怎样保证不越过航点、不后退

分两层：**载具/场景层**（仿真器物理）与**跟随算法层**（规划到控制）。

### 5.1 载具/场景层（避免冲过头）

越过航点的直接原因是**实际制动强于规划的弱、实际减速度达不到 `planned_speed` 的假设**。要点：

- **四个轮子都要有摩擦**（已修郊区后轮 `0.0→1.0`）。只有前轮有抓地力时，制动距离远大于模型中 `max_acceleration_mps2` 的假设，冲过头就发生在这里。
- 模型参数（`drive.py` 顶部的标定：`speed_per_engine_mps`、`brake_gain` 等）必须以**实际刹车能力**标定；否则规划出来的刹车曲线不可达。
- 巡航速度与航点间距要匹配：低速+短间距时，留给刹车的距离更充足。这是配置项（`config` 的 `motion` 段），不是代码改动。

### 5.2 跟随算法层（避免"目标落到身后就倒车"）

**根本做法是把"全向跟随"换成"运动模型感知的跟随"。** 对非全向车，正确的路径跟踪是**纯跟踪（pure pursuit）**式的，而不是"朝胡萝卜点直接拉一个速度向量"：

- 在路径上取前视点（已有前视逻辑，`continuous_reference.py:49-78`）；
- 由"车头与到前视点连线的夹角"算**曲率** `κ = 2·sin(α)/Ld`；
- 输出 **有符号前向速度 + 曲率/偏航率**，`前向速度 ≥ 0`（除非任务明确要求倒车）；
- **转向去贴合路径，而不是让速度向量拐到身后**。

这样：
- 目标不会变成"后方一个点需要倒着去够"——它始终在车头前方的前视弧上，**从结构上不会触发后退**；
- 横向误差靠**转向**自然收掉（车头朝向前视点时，前进就同时减小横向误差），**不再出现"只差横向、前向为零"的死锁**；
- 无人机继续用全向速度模式（它本来就该能横移/悬停对位）。

配套的最小判据修正：**对非全向车，"已驶过航点"的横向走廊应放宽到车宽量级**（或直接沿用分段投影 `passed` 判据），使"冲过一点但在走廊内"时**推进下一个航点**，而不是倒回去精确命中一个点。当前 `reached()` 把"到达"和"驶过"都用 0.3 m，是造成"差 0.09 m 就倒车"的直接推手。

---

## 6 接口与设计：从结构上杜绝"能横移/不能横移"

### 6.1 现状的问题（为什么错误能潜伏）

1. 端口 `stream_velocity(world_velocity, yaw_rate)` 是**全向速度契约**，但**没有声明载具的运动模型**。
2. 无人车适配层（`drive.py:157`）把横向分量**静默投影掉**——不是报错，不是降级日志，是无声丢弃。
3. 上层跟随器**无从得知**当前载具是不是全向，只能假设全向。
4. `prepare()` 的能力检查（`planner/workflows/visual_goal_navigation/continuous.py:48-50`）只确认 `stream_state`/`stream_velocity` 两个方法**存在**，不校验运动模型是否匹配。

于是"无人机能横移、无人车不能"这种差异**没有任何地方显式表达**，只能在跑飞后靠日志反推。

### 6.2 建议的设计

**核心原则：端口要表达"这台载具真正能执行什么动作"，跟随器按运动模型合成合法命令，非法组合在构造期报错，而不是运行期静默丢分量。**

#### (a) 载具声明运动能力

在 `StreamingVehicle` 端口旁加一个**能力描述**（属性或方法二选一）：

```python
class MotionModel(Enum):
    HOLONOMIC     = "holonomic"      # 全向：无人机
    ACKERMANN     = "ackermann"      # 阿克曼：本项目无人车
    DIFFERENTIAL  = "differential"   # 差速（预留）

@dataclass(frozen=True)
class MotionCapability:
    model: MotionModel
    max_lateral_speed_mps: float     # 阿克曼 = 0
    min_turn_radius_m: float | None  # 阿克曼有下限
    allows_reverse: bool
```

由每个仿真器的载具类给出。无人车 `ACKERMANN, max_lateral_speed=0`；无人机 `HOLONOMIC`。

#### (b) 命令按模型分型（tagged union）

```python
@dataclass(frozen=True)
class HolonomicCommand:      # 给 HOLONOMIC
    vx: float; vy: float; vz: float; yaw_rate_deg_s: float

@dataclass(frozen=True)
class CarCommand:            # 给 ACKERMANN / DIFFERENTIAL
    forward_speed_mps: float     # 有符号
    curvature_per_m: float       # 或 yaw_rate_deg_s
    reverse: bool
```

- `ContinuousFollower` 读 `capability.model`：
  - `HOLONOMIC` → 走现有全向逻辑，产出 `HolonomicCommand`；
  - `ACKERMANN` → 走纯跟踪，产出 `CarCommand`（前向速度 + 曲率/偏航率，绝不产出横向速度）。

#### (c) 适配层拒绝非法命令

- 无人车 `stream_velocity/stream_command`：若收到带横向分量的命令（超出 `max_lateral_speed` 的 epsilon），**抛错**而不是丢分量。
- 无人机适配层只接受 `HolonomicCommand`。

把现在的"静默投影"（`drive.py:157`）改成"契约校验"。原来能跑，是因为上层错误地给了横向速度；改后这种错误在**第一拍**就炸，不会再跑到超时。

#### (d) 构造期一致性检查

在 `prepare()`（`visual_goal_navigation/continuous.py:47-50`）加：

- 跟随器模式 == `vehicle.motion_capability.model`；
- 适配层接受的命令类型 == 该模型对应的类型；
- 不一致 → 构造期报错（沿用项目"少接一根线在构造期报错，不静默降级"的不变量，`docs/ARCHITECTURE.md` §1）。

#### (e) 用测试钉住

- **阿克曼跟随器永不产出横向分量**：在多种场景（直行、急弯、冲过点、残差为侧向）下断言 `|lateral| ≤ eps`。
- **全向跟随器允许横向**：对照组。
- **适配层拒收非法命令**：给无人车喂横向速度，断言抛错。
- **"目标在身后"时对阿克曼是推进航点或转向，而不是倒车**：构造一个"已冲过端点且横向在走廊内"的状态，断言队列头被消费 / 不产生 `reverse=True`。

这几条把本次的 bug 变成回归测试。

### 6.3 为什么不把非全向逻辑全塞进适配层

一种替代方案是"端口契约改成'到达这个参考位姿'，各适配层自己跑局部跟踪"。缺点是**每个仿真器都要重写一遍路径跟踪**（本项目三套实现就是三份重复），难测试、难统一。**推荐保留共享的高层路径跟踪，但让它运动模型感知**（上面 6.2），差异只落在"命令类型 + 适配层落地"这一小层——这与现有分层（§2/§3）一致，改动面可控。

---

## 7 无人机 vs 无人车：现在到底哪些不同、哪些相同

| 维度 | 相同 | 不同 |
| --- | --- | --- |
| VLM 任务解析 / 阶段 | 同流程 | 提示词按 `aerial` 分叉（`factories.py:126-146`、`mission_assembly.py:316`） |
| 轨迹规划出的航点格式 | `[dx,dy,dz,dyaw]` 同一协议 | 提示词 aerial/ground 不同 |
| 滑窗队列 / 连续执行 | **完全同一份** `ContinuousExecution` | — |
| `stream_velocity` 契约 | **同一签名** | 落地不同：阿克曼投影 vs 全向速度 |
| 垂直运动 | — | `vertical_motion=is_drone`（`mission_assembly.py:260/308`） |
| 观测源 / 跟踪限制 | — | 各载具端口对象不同（`vehicle.tracking_limits`） |

一句话：**"规划到下发"共享，"下发到动作"按运动模型分叉。** 现在缺的正是"分叉点上的类型约束"。

---

## 8 落地清单（按优先级）

1. **[已做] 郊区后轮摩擦 `0.0→1.0`**（`config/scenes/neighborhood/robot_rover.jsonc`，与交流中心一致）。改善制动与循迹，降低冲过航点概率。
2. **[建议] 放宽非全向车的"驶过"横向走廊**，让"冲过一点但在车宽内"推进下一个航点，而非倒回精确命中（`reached()` / `consume_stream`）。
3. **[建议·核心] 跟随器运动模型感知**：给载具加 `MotionCapability`，`ContinuousFollower` 对阿克曼产出 `CarCommand(前向速度, 曲率)` 的纯跟踪，取代"朝胡萝卜点拉速度向量"。
4. **[建议·核心] 端口校验**：适配层拒收带横向分量的无人车命令；`prepare()` 校验跟随器模式与载具模型一致，不一致构造期报错。
5. **[建议] 回归测试**：§6.2(e) 的 four 条断言。

前两条能快速止血；第三、四条是根治"越过航点→后退→空转卡死"这一类问题的结构性改动，建议单独评估、分提交落地。
</content>
