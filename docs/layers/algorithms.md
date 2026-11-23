> 上级：[项目架构](../ARCHITECTURE.md)
> 同层兄弟：[执行 runtime](./runtime.md)、[契约 domain](./domain.md)、[契约 ports](./ports.md)、[任务 workflows](./workflows.md)、[装配 assembly](./assembly.md)、[入口 adapters](./adapters.md)、[进程栈 simulator](./simulator.md)、[引擎 simulators](./simulators.md)

# algorithms（计算层）

## 1 一句话职责

计算层只用纯计算把契约类型换成契约类型：三维几何、坐标变换、轨迹协议校验、深度测距、目标绑定与数值动作换算；不持有时钟、线程、车辆、队列或网络。

## 2 边界

管什么
- 模长限幅、五次曲线时长与采样相关的纯函数（`bounded`、越点判据、走廊默认值）。
- 相对/世界轨迹互转与预执行协议校验。
- 从标定深度图估计目标表面点、扇面自由空间剖面、目标位置绑定与阶段内冻结。
- 无模型端点时的确定性几何规划器（`GeometricTargetPlanner`）。
- 滑窗补点判据（`RefillScheduler`），纯策略，不含 HTTP。

不管什么
- 命令节拍、停稳判定、碰撞退避、控制权：见 [runtime](./runtime.md)。
- 队列修订、在途模型调用、窗口状态：见 [runtime](./runtime.md)。
- 端口定义与契约类型：见 [domain](./domain.md)、[ports](./ports.md)。
- 视觉模型与检测器实现、prompt 渲染：见 [adapters](./adapters.md)。

依赖实测：`planner/algorithms/` 只 import `planner.domain`、`planner.errors` 与本层内部模块，无一条指向 `runtime`、`workflows`、`assembly`、`adapters`、`simulators`。用命令复核：

```bash
grep -rn "^from \.\|^from planner\." planner/algorithms/ | grep -v "planner.domain\|planner.errors\|planner.algorithms"
```

## 3 输入与输出

入口 `planner/algorithms/__init__.py`，导出 13 个名字（`__all__` 在 planner/algorithms/__init__.py:18）。二级入口三个：`planner.algorithms.motion`（planner/algorithms/motion/__init__.py:7）、`planner.algorithms.depth`（planner/algorithms/depth/__init__.py:8）、`planner.algorithms.targeting`（planner/algorithms/targeting/__init__.py:12）。

输入是契约类型：`Observation`、`GoalObservation`、`GoalGeometry`、`TargetPositionState`、`WorldPose`、`RelativeTrajectory`、`WorldTrajectory`、`MotionLimits`、`CameraIntrinsics`、`VisualGoalPlanningContext`、`DepthForegroundSettings`、`ContinuousTrackingSettings`。

返回分四类
- 数值或布尔：`bounded` 返 tuple，`reached` 返 bool，`corridor_default` 返 float。
- 契约结构：`relative_to_world` 返 `WorldTrajectory`，`world_to_relative` 返 `RelativeTrajectory`，`accept_position` 返 `(TargetPositionState | None, str | None)`，`measure_free_space` 返 `FreeSpaceProfile`，`DepthGeometry.estimate` 返 `GoalGeometry`。
- 校验通过返 `None`：`TrajectoryValidator.validate_relative` / `validate_world`、`require_execution_anchor`。
- 异常：见第 6 节。

## 4 对外名字表

| 名字 | 类型 | 一句话用途 | 层外调用点 |
| --- | --- | --- | --- |
| `RefillScheduler` | 类（带状态） | 按待执行点数与剩余时长判断是否补点，并滑动平均模型延迟 | planner/runtime/window/window.py:69、planner/runtime/window/inflight.py:38 |
| `TrajectoryValidator` | 类 | 相对/世界轨迹点数、单步位移与单步偏航上限校验 | planner/runtime/window/window.py:62、planner/assembly/mission_assembly.py:236、planner/workflows/visual_goal_navigation/workflow.py:68 |
| `bounded` | 函数 | 按欧氏范数限幅，唯一的速度/加速度/jerk 限幅实现 | planner/runtime/motion/command_smoothing.py:21、planner/runtime/position_hold.py:91 |
| `corridor_default` | 函数 | 按运动模型给出越点走廊默认值 | 无层外调用点，仅 tests/test_algorithms_motion.py:20 |
| `passed_lateral_corridor` | 函数 | 配置走廊抬到模型默认值后的结果 | planner/simulators/projectairsim/rover.py:66 |
| `horizontal_distance` | 函数 | 两点水平距离 | planner/workflows/visual_goal_navigation/completion.py:43 |
| `point_in_horizontal_body` | 函数 | 世界点转 yaw-only 车体系 (forward, right, down) | planner/workflows/visual_goal_navigation/continuous.py:211、planner/adapters/trajectory_planner/prompts/visual_goal_navigation.py:111 |
| `reached` | 函数 | 越点判据：距离圈或越点走廊成立且偏航对齐 | planner/runtime/window/queue.py:142 |
| `relative_to_world` | 函数 | 车体系增量序列转世界航点，保留带符号偏航 | planner/runtime/window/inflight.py:98 |
| `require_execution_anchor` | 函数 | 实测位姿偏离锚点超差即抛 `ExecutionError` | planner/runtime/window/queue.py:114 |
| `world_pose_delta_in_reference_frame` | 函数 | 世界位姿差转到起点水平系 | planner/adapters/trajectory_planner/prompts/base.py:61 |
| `render_depth` | 函数 | 米制深度渲成三通道显示图，不改输入，按映射协议选灰度曲线（planner/algorithms/depth/render.py:18） | planner/simulators/airsim/observation_source.py:15、planner/simulators/projectairsim/observation.py:24、scripts/smoke_projectairsim_drone.py:43 |
| `world_to_relative` | 函数 | 世界航点转车体系增量，偏航用元数据不重新测量 | planner/runtime/window/queue.py:82、planner/runtime/window/window.py:131、planner/workflows/visual_goal_navigation/approach.py:25 |

二级入口额外对外（不在顶层 13 名里）：`measure_free_space`、`FreeSpaceProfile`、`FreeSpaceProfileSettings`、`DepthGeometry`、`DepthForegroundFilter`、`sample_surface_depth`（depth 组）；`BasicMotionConverter`、`GeometricTargetPlanner`、`GeometricPlannerSettings`、`accept_position`、`bound_target_geometry`、`bound_target_goal`、`make_reference`（targeting 组）。调用点：planner/adapters/trajectory_planner/prompts/visual_goal_navigation.py:6、planner/assembly/factories/detectors.py:60、planner/workflows/basic_motion/workflow.py:14、planner/workflows/visual_goal_navigation/targeting.py:12。

## 5 关键流程

视觉目标：从深度图到五个新航点
1. 标定与形状校验。`DepthGeometry.validate` 检查 `depth_kind`、两张图与各自内参的尺寸、焦距、主点、旋转矩阵正交性与行列式、相机原点有限性，任一不合格抛 `ValueError`（planner/algorithms/targeting/depth_geometry.py:26-48）。
2. 逐点采样。对每个目标表面点先按 `intrinsics` 求射线，再用 `depth_intrinsics` 投影到像素，`sample_surface_depth` 只读一个像素并校验落在 bbox 内（depth_geometry.py:90-94、planner/algorithms/targeting/depth_surface_sampling.py:17-39）。
3. 前景分层。`DepthForegroundFilter.filter` 在半径窗口内找原生深度断点，把落后于 cutoff 的样本标为 `filtered_background` 并置 `valid=False`（planner/algorithms/targeting/depth_foreground.py:18-38）。
4. 世界点。光轴深度换算成沿射线的距离，再经 `rotation_camera_to_world` 落回世界系，输出 `SurfacePointEstimate`；取最近的有效样本作为 `GoalGeometry.world_point`（depth_geometry.py:98-103、:84-86）。
5. 绑定到阶段。`accept_position` 首次绑定时把所有有效表面点冻结成锚点集合，之后只允许锚点半径内的最近点刷新同名表面，不新增表面；时间戳或单调钟倒退的观测返 `stale_target_observation`（planner/algorithms/targeting/association.py:12-35、:36-50）。
6. 规划。`GeometricTargetPlanner.plan` 把目标点转到车体系，水平剩余距离减掉 `arrival_buffer_ratio × arrival_radius_m` 后切五段，每段偏航步长受 `max_yaw_step_deg` 限制，返回 5 点 `RelativeTrajectory`（planner/algorithms/planning/geometric_planner.py:71-110）。

轨迹协议校验与坐标变换
1. `TrajectoryValidator.__init__` 校验点数 1..5、步长与偏航上限有限为正、偏航上限不超过 180（planner/algorithms/trajectory_validation.py:21-32）。
2. `validate_relative` 逐点查 `hypot(dx,dy,dz) <= max_step_m` 与 `|dyaw_deg| <= max_yaw_step_deg`（:34-42）。
3. `relative_to_world` 以上一段预测位姿为参考逐点累加，偏航显式取模但保留小角命令元数据（planner/algorithms/trajectory_transform.py:26-40）。
4. `validate_world` 用 `resolve_yaw_delta_deg` 消歧带符号偏航后逐段复核，容差加 1e-6 吸收浮点误差（trajectory_validation.py:44-56）。
5. 执行前锚点核对：`require_execution_anchor` 同时查位置与偏航，超差抛 `ExecutionError`（trajectory_validation.py:10-15）。

越点与补点
1. `reached` 计算投影与横向偏差，`projection >= 1` 且横向在走廊内算"冲过"，或距离在容差圈内；两者任一成立还要偏航对齐（planner/algorithms/motion/reached.py:19-33）。
2. 队列侧每拍用 `consume_stream` 判断是否消费队头（planner/runtime/window/queue.py:140-147）。
3. 补点判据 `should_refill(count, duration_s)`：点数 1..5 且（低于阈值点数 或 剩余时长不超过滑动延迟加 margin）（planner/algorithms/sliding_window/scheduler.py:10-13）。
4. 每次模型返回后 `record_latency` 做 0.75/0.25 指数平均（:16）。

## 6 失败模式

| 报错类型 | 触发条件 | 调用方处理 |
| --- | --- | --- |
| `ValueError` | `corridor_default` 收到非 `MotionModel`（corridor.py:18）；`point_in_horizontal_body` 收到非三个有限数（target_coordinates.py:26-27）；`FreeSpaceProfileSettings` / `TrajectoryValidator` 参数非法；`DepthGeometry.validate` 标定不成立 | 属于接线错误，修正构造参数，不重试 |
| `ProtocolError` | 轨迹点数越界、单步位移或偏航超限（trajectory_validation.py:36,40,42,46,51,55）；`GeometricTargetPlanner.plan` 缺类型化目标上下文或无可靠目标点（geometric_planner.py:61-64） | 丢弃本次模型输出，重新取帧补点 |
| `ExecutionError` | `require_execution_anchor` 锚点漂移超差（trajectory_validation.py:15） | 任务层结束本阶段，按漂移复核后续点 |
| `ValueError('target state belongs to another stage')` | `accept_position` 收到属于别的阶段的 `TargetPositionState`（association.py:17-18） | 阶段隔离被破坏，清空绑定重新开始 |
| 不抛异常，返回不可靠几何 | `DepthGeometry.estimate` 捕获 `ValueError/TypeError/IndexError` 后返 `GoalGeometry(False, str(exc))`（depth_geometry.py:87-88） | 读 `geometry.reliable` 与 `geometry.reason`，不要靠异常 |

## 7 测试位置

- `tests/test_algorithms_motion.py`（36 行）：`bounded`、`corridor_default`、`passed_lateral_corridor` 的模型分支与非法模型。
- `tests/test_passed_corridor.py`：越点走廊在队列层面被抬高的行为。
- `tests/test_capability_targeting.py`（284 行）：`accept_position` 的首次绑定、陈旧观测、跨阶段拒收、`make_reference` 裁剪、`sample_surface_depth` 越界与过近、`DepthForegroundFilter` 关断与均匀深度。
- `tests/test_depth_geometry.py`（155 行）：标定校验失败分支与世界点换算。
- `tests/test_free_space.py`（188 行）：`measure_free_space` 的剖面、`clear_window_deg`、`nearest_ahead_m`、参数校验。
- `tests/test_geometric_planner.py`（160 行）：五段切分、到达缓冲、类型化上下文缺失。
- `tests/test_basic_motion.py:132`：`BasicMotionConverter` 轴向到世界位姿的换算。
- `tests/test_trajectory_transform.py`（新增，15 个用例、22 个测试项）：`world_pose_delta_in_reference_frame` 的航向系与回绕、`relative_to_world` 的累加与带符号偏航元数据、`world_to_relative` 的往返与元数据优先、`TrajectoryValidator` 两个校验入口的点数边界、恰好等容差、1e-6 容差带与构造期参数拒收。
- `tests/test_public_surface.py`：断言 12 个名字仍在入口。

未覆盖的空白
- `planner/algorithms/trajectory_validation.py` 的 `require_execution_anchor` 仍无直接单测；`TrajectoryValidator` 的执行层集成边界（锚点与队列抵叠）只在 `tests/test_batch3_services.py` 间接跑到。
- `FreeSpaceProfileSettings` 的十个阈值只在 tests/test_free_space.py:189 处做参数校验测试，没有针对真实深度图的标定回归。
- `DepthForegroundFilter` 的断点判定只在构造样例上验证，没有针对不同内参组合（`intrinsics` ≠ `depth_intrinsics`）的固定样例。

## 8 与其他层的联动

- 上游：`domain` 提供全部契约类型与 `resolve_yaw_delta_deg`、`wrap_yaw_deg`；`errors` 提供 `ProtocolError`、`ExecutionError`。
- 下游 `runtime`：限幅（command_smoothing.py:21、position_hold.py:91）、越点（window/queue.py:142）、锚点（queue.py:114）、轨迹互转（window/window.py:131、window/inflight.py:98）、补点判据（window/window.py:69,273）。
- 下游 `workflows`：距离测量（visual_goal_navigation/completion.py:43）、目标绑定（visual_goal_navigation/targeting.py:71,89）、几何上下文（visual_goal_navigation/continuous.py:149,190）。
- 下游 `adapters`：prompt 里的车体系坐标（trajectory_planner/prompts/visual_goal_navigation.py:111,124）、自由空间剖面（prompts/free_navigation.py:83）。
- 下游 `simulators`：走廊值落到载具 `tracking_limits`（projectairsim/rover.py:66）。
- 数据形态：进出都是不可变 dataclass 与 numpy 只读数组；本层不缓存跨调用状态，`RefillScheduler` 的 `latency_s` 是唯一例外。

## 9 疑问与争议

Qb1 层：algorithms 结论：包入口自称"纯函数与纯策略"，但唯一带状态的 `RefillScheduler` 在 `record_latency` 里改写 `latency_s`。依据：planner/algorithms/__init__.py:1、planner/algorithms/sliding_window/scheduler.py:8,16 影响：否

Qb2 层：algorithms 结论：`relative_to_world` / `world_to_relative` 在契约层与计算层各有一份同名不同签名的实现，两个运行时调用点分别指向不同层的同名函数。依据：planner/domain/pose.py:54、planner/algorithms/trajectory_transform.py:26,43、planner/runtime/segment_motion.py:58、planner/runtime/window/inflight.py:11 影响：是

Qb3 层：algorithms 结论：`ACKERMANN_PASSED_LATERAL_TOLERANCE_M = 1.0` 没有标定出处，注释只说明是"量级依据"，而测试把这个值钉住了。依据：planner/algorithms/motion/corridor.py:7-9、tests/test_algorithms_motion.py:20 影响：否

Qb4 层：algorithms 结论：轨迹变换与校验四个对外名字原本无直接单测，本轮新增 tests/test_trajectory_transform.py 后覆盖到 1e-6 容差带与构造期拒收；`require_execution_anchor` 仍只在层外间接跑到。依据：planner/algorithms/trajectory_transform.py:9,26,43、planner/algorithms/trajectory_validation.py:10,14,50,54 影响：否

Qb5 层：algorithms 结论：`reached` 的 `passed_lateral_tolerance=None` 分支在真实路径不可达，队列构造时已把 `None` 换成位置容差。依据：planner/algorithms/motion/reached.py:27、planner/runtime/window/queue.py:30-33,142-144 影响：否

Qb6 层：algorithms 结论：`DepthGeometry.estimate` 把标定错误和"没有有效像素"压成同一种返回值，调用方无法按原因分流。依据：planner/algorithms/targeting/depth_geometry.py:26-29,82-88 影响：否

Qb7 层：algorithms 结论：`FreeSpaceProfileSettings` 十个阈值的合法性有校验，出处与标定过程没有记录。依据：planner/algorithms/free_space.py:25-34,36-74 影响：否

## 10 已知缺口

M5 同名变换两份
- 生产名 `planner.algorithms.trajectory_transform.relative_to_world(RelativeTrajectory, WorldPose)` 把整条相对轨迹落到世界，逐点保留带符号偏航（planner/algorithms/trajectory_transform.py:26）。调用方：planner/runtime/window/inflight.py:98、planner/workflows/visual_goal_navigation/approach.py:25。
- 契约层另有一份同名不同签名的 `planner.domain.pose.relative_to_world(yaw_deg, dx, dy, dz) -> tuple`（planner/domain/pose.py:54），只把单个车体系偏移转到世界。调用方：planner/runtime/segment_motion.py:58。
- 风险：按名字检索会混两条路径，改一处漏另一处；两份都叫 `relative_to_world`，没有 `_trajectory` 式的区分后缀。

M7 经验值未标定

| 常量 | 取值 | 位置 | 影响 |
| --- | --- | --- | --- |
| `ACKERMANN_PASSED_LATERAL_TOLERANCE_M` | 1.0 m | planner/algorithms/motion/corridor.py:9 | 阿克曼越点走廊下限，配置只能抬高 |
| `sector_width_deg` | 10.0 度 | planner/algorithms/free_space.py:26 | 扇面分辨率与剖面点数 |
| `max_range_m` | 40.0 m | free_space.py:27 | 剖面报告距离上限 |
| `distance_bin_m` | 1.0 m | free_space.py:28 | 地面线分箱粒度 |
| `min_points_per_bin` | 8 | free_space.py:29 | 分箱有效所需点数 |
| `ground_height_percentile` | 90.0 | free_space.py:30 | 地面高度取分位 |
| `min_height_above_ground_m` | 0.15 m | free_space.py:31 | 障碍高度带下沿 |
| `max_height_above_ground_m` | 2.5 m | free_space.py:32 | 障碍高度带上沿 |
| `obstacle_distance_percentile` | 5.0 | free_space.py:33 | 障碍距离取分位 |
| `min_obstacle_pixels` | 3 | free_space.py:34 | 判定为障碍所需像素数 |
| `max_advance_m` / `max_vertical_m` / `max_yaw_step_deg` | 6.0 m / 2.0 m / 20.0 度 | planner/algorithms/planning/geometric_planner.py:27-29 | 几何规划器单批推进上限 |
| `arrival_buffer_ratio` | 0.8 | geometric_planner.py:31 | 停在到达半径的 80% 处 |
| `RefillScheduler.latency_s` | 5.5 s | planner/algorithms/sliding_window/scheduler.py:8 | 补点提前量，注释自述实验初值 |

上表全部为经验值，未标定。改这些数只改本层计算结果，仓库里没有标定回归可对照。

M13 剩余测试空白
- `require_execution_anchor`（planner/algorithms/trajectory_validation.py:10）无直接单测。
- 阈值改动没有回归基线，改 `free_space.py` 或 `corridor.py` 的常量不会让任何测试失败，除非同时改 tests 里的期望值。
- `DepthForegroundFilter` 在 `intrinsics` 与 `depth_intrinsics` 不相同的场景没有固定样例，像素映射只靠一次手工修复的注释保证。
