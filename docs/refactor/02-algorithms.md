# 计算层 `planner/algorithms/`：文件清单与内容

`algorithms/` 只做变换与判定：不读时钟、不开线程、不碰设备与网络、不认识任务名。全部现有算式按四组归位（几何与变换、深度、序列校验、运动律），`ContinuousFollower` 与两份 tracker 里重复的算式各有唯一落点，执行层只留状态壳、节拍与下发。越点走廊默认值、纯跟踪曲率和偏航律在本层定型，按载具运动模型分叉的只有一处：越点走廊。（裁决 R1：引擎侧不再有 tracker 与段采样循环，这些算式的宿主是 `runtime/motion/loop.py`；裁决 R6：走廊常量与按模型的默认值在 `algorithms/motion/corridor.py`。）

## 一、本层职责与文件清单

职责一句话：给定输入得到输出，同输入同输出；跨拍状态只以显式对象持有，状态转移是 (state, 输入) 的纯函数，重建或 `reset()` 由执行层负责。

### 1.1 包结构

```
planner/algorithms/
  __init__.py                          本层唯一公开入口，`__all__` 见 §3.5（裁决 R19a）
  target_coordinates.py                几何：水平距离、最近表面、机体系投影
  trajectory_transform.py              变换：机体/世界互转，参考系增量
  free_space.py                        深度：扇面测距与地面高度带过滤
  trajectory_validation.py             校验：执行锚点、航点数与单步上下限
  basic_motion_conversion.py           数值动作：参数 → 目标位姿
  depth/__init__.py
  depth/surface_sampling.py            单像素深度读数
  depth/foreground.py                  逐点深度不连续过滤
  depth/geometry.py                    相机基、反投影、目标几何估计
  motion/__init__.py
  motion/base.py                       MotionPlanner 声明（见 §8 第 4 条）
  motion/limits.py                     模长限幅、jerk/加速度限幅步、带宽
  motion/quintic.py                    五次多项式段规划
  motion/synchronized.py               同步平移+偏航 profile 的采样与时长
  motion/speed.py                      刹车距离、转角速度、整队列速度包络、接近速度
  motion/reference.py                  有界路径预览与锁存对齐
  motion/corridor.py                   越点走廊常量与按模型的默认值（裁决 R6）
  motion/reached.py                    越点判据
  motion/settle.py                     位姿差测量与停稳判据
  motion/heading.py                    偏航伺服
  motion/pursuit.py                    纯跟踪：前视、曲率、阿克曼命令
  motion/handoff.py                    停稳前纯平移交接
  planning/__init__.py
  planning/geometric_planner.py        确定性几何规划器（清单内，细则不在本方案四组）
  targeting/__init__.py
  targeting/association.py             目标绑定（清单内，细则不在本方案四组）
  sliding_window/__init__.py
  sliding_window/scheduler.py          补点策略
```

依赖白名单：`domain/`、`planner/errors.py`、标准库与 `numpy`。本层不 import `ports/`（没有端口调用，连类型标注也不需要），不 import `runtime/`、`workflows/`、`assembly/`、`adapters/`、`simulator/`、`simulators/`。跨层 import 只写 `from planner.algorithms import <名字>`，`algorithms/motion/*`、`algorithms/depth/*` 等子模块路径不出本层，同层内部可以深层 import（裁决 R19a）；子包的 `__init__.py`（`motion/`、`depth/`、`planning/`、`targeting/`、`sliding_window/`）只是同层内的聚合，不对外。四份下游文档要求的名字与路径逐一对应：`01-contracts.md:154` 的 `algorithms/motion/quintic.py`、`03-runtime.md:57-60` 的 `algorithms/motion/{...}` 与 `algorithms/motion/reached.py`、`03-runtime.md:363` 的 `RefillScheduler`、`04-workflows.md:197` 的 `BasicMotionConverter` 与 `TrajectoryValidator`；裁决 R6 把走廊拆到 `algorithms/motion/corridor.py`，比 `03-runtime.md` 的清单多一个文件。

### 1.2 文件清单

| 目标文件 | 估计行数 | 从哪来 | 类与函数 |
| --- | --- | --- | --- |
| `target_coordinates.py` | 40（现 34） | `planner/services/target_coordinates.py` 全文 | `horizontal_distance`、`nearest_surface`、`point_in_horizontal_body` |
| `trajectory_transform.py` | 62（现 52） | `planner/services/trajectory_transform.py` 全文 | `horizontal_body_offset`（新）、`world_pose_delta_in_reference_frame`、`relative_to_world`、`world_to_relative` |
| `free_space.py` | 285（现 281） | `planner/services/free_space.py` 全文 | `FreeSpaceProfileSettings`、`FreeSpaceSector`、`FreeSpaceProfile`、`measure_free_space`、`sector_centers`、`measure_sector` |
| `trajectory_validation.py` | 62（现 56） | `planner/services/trajectory_validation.py` 全文 | `require_execution_anchor`、`TrajectoryValidator` |
| `basic_motion_conversion.py` | 20（现 19） | `planner/capabilities/basic_motion_conversion.py` 全文 | `BasicMotionConverter.create_target` |
| `depth/surface_sampling.py` | 40（现 39） | `planner/capabilities/targeting/depth_surface_sampling.py` 全文 | `SurfaceDepthSample`、`sample_surface_depth` |
| `depth/foreground.py` | 80（现 74） | `planner/capabilities/targeting/depth_foreground.py` 全文 | `DepthForegroundFilter.filter`、`_layer` |
| `depth/geometry.py` | 115（现 107） | `planner/capabilities/targeting/depth_geometry.py` 全文 | `DepthGeometry.validate/ray/project_camera/estimate` |
| `motion/base.py` | 8（现 7） | `planner/services/motion/base.py` 全文 | `MotionPlanner.create_motion` |
| `motion/limits.py` | 55（现 33 中的算术 + 两份重复 17） | `planner/services/motion/command_smoothing.py` 的算式 + `simulators/projectairsim/motion.py:35-44` 与 `simulators/airsim/motion_tracker.py:20-27` 的 `limit_vector`；状态壳留在 `runtime/motion/limits.py::CommandLimiter`（`03-runtime.md:30`） | `bounded`、`limit_vector`、`tracking_velocity`、`slew_limited`、`command_bandwidth`、`smoothed_step` |
| `motion/quintic.py` | 80（现 154 中的 60） | `planner/services/segment_motion.py:95-155` | `SegmentSetpoint`、`quintic`、`duration_for`、`plan_segment`、`_QUINTIC_PEAK_VELOCITY`、`_QUINTIC_PEAK_ACCELERATION` |
| `motion/synchronized.py` | 82（现 75） | `planner/services/motion/synchronized.py` 全文 | `SMOOTH_PROFILE`、`duration_for_poses`、`MotionSample`、`sample_segment`、`SynchronizedMotionPlanner` |
| `motion/speed.py` | 110（现 96） | `planner/services/motion/continuous_speed.py` 全文 + `continuous.py:83-91` 的接近速度 | `stopping_distance`、`braking_speed`、`angular_duration`、`corner_speed`、`corner_entry_speed`、`damped_approach_speed`、`planned_speed` |
| `motion/reference.py` | 85（现 81） | `planner/services/motion/continuous_reference.py` 全文 | `xyz`、`segment_length`、`is_rotation`、`distance_to_segment`、`ContinuousPathReference` |
| `motion/corridor.py` | 25（现 1 个常量） | `simulators/projectairsim/rover.py:48` 的常量 + `MotionLimits.position_tolerance_m`；裁决 R6 | `ACKERMANN_PASSED_LATERAL_TOLERANCE_M`、`corridor_default`、`passed_lateral_corridor` |
| `motion/reached.py` | 35（现 17） | `planner/services/motion/continuous.py:12-28` | `reached` |
| `motion/settle.py` | 45（现两份 12 + 12） | `simulators/projectairsim/motion.py:121-171` 与 `simulators/airsim/motion_tracker.py:100-123` 的重复部分；裁决 R1 后两份文件删除，宿主是 `runtime/motion/loop.py` | `pose_delta_measurement`、`is_settled`、`within_tolerance` |
| `motion/heading.py` | 100（现 97） | `planner/services/motion/heading_servo.py` 全文 | `heading_rate_deg_s`、`tracked_heading_rate_deg_s` |
| `motion/pursuit.py` | 95（新增） | 无；`docs/MOTION_MODEL_AND_CONTINUOUS_EXECUTION.md` §5.2 的纯跟踪 | `PursuitSolution`、`lookahead_distance`、`pure_pursuit_curvature`、`max_curvature`、`lateral_speed_limit`、`holonomic_velocity`、`pursuit_solution`、`ackermann_command` |
| `motion/handoff.py` | 30（现 25） | `planner/services/motion/hold_handoff.py` 全文 | `translation_handoff` |
| `planning/geometric_planner.py` | 110 | `planner/capabilities/planning/geometric_planner.py` 全文 | `GeometricPlannerSettings`、`GeometricTargetPlanner.plan` |
| `targeting/association.py` | 80 | `planner/capabilities/targeting/association.py` 全文 | `accept_position`、`bound_target_geometry`、`bound_target_goal`、`make_reference` |
| `sliding_window/scheduler.py` | 22（现 16） | `planner/capabilities/sliding_window/scheduler.py` 全文 | `RefillScheduler` |

合计约 1 600 行，现状对应文件约 1 475 行。增量集中在两处新增：`motion/pursuit.py` 95 行、`motion/settle.py` 45 行；`motion/limits.py`、`motion/corridor.py` 与 `motion/reached.py` 各增 15–30 行。重复的 `limit_vector` 两份、停稳判据两份被合并后抵消一部分。

`planning/`、`targeting/` 两个文件的细则不在本方案四组内，按 `docs/REFACTOR.md` 原样搬迁，签名不变；`basic_motion_conversion.py` 只有 `create_target(parameters: BasicMotionParameters, start: WorldPose) -> WorldPose` 一个方法（`forward_m` 沿机头、`up_m` 抬高 z 减小、`yaw_right_deg` 顺时针加），纯函数，归本层。

### 1.3 几何与变换

**`horizontal_distance(point: Point3, pose: WorldPose) -> float`**
- 输入输出：世界点与位姿 → 水平（x-y）距离，忽略 z。
- 边界条件：不做类型校验（现状如此）；`math.hypot(dx, dy)` 对 ±inf 返回 inf，不抛错，调用方负责传入有限值。
- 数值稳定性：用 `hypot` 而不是 `sqrt(dx*dx+dy*dy)`，避免中间量溢出；不引入 epsilon，0.3 m 判据由调用方给。
- 单测点：z 变化不影响结果；东西向与南北向各一个已知值；与 `math.dist` 在 z 相等时一致。

**`nearest_surface(points: tuple[Point3, ...], pose: WorldPose) -> Point3`**
- 输入输出：非空表面点集合 → 水平距离最小的那个点。
- 边界条件：空 tuple 抛 `ValueError`（`min()` 的默认行为，不另加校验）；平局取先出现者（`min` 的稳定性，保证同输入同输出）。
- 数值稳定性：只比较水平距离，不做平均；等价于 `min(points, key=...)`，O(n)。
- 单测点：两点等距时返回第一个；含重复点时返回重复值；z 差很大但水平更近者胜出。

**`point_in_horizontal_body(point: Point3, pose: WorldPose) -> tuple[float, float, float]`**
- 输入输出：世界点 → 机体系 (forward, right, down)，只按 yaw 旋转。
- 边界条件：`len(point) != 3` 或任一坐标非有限 → `ValueError("target point must contain three finite coordinates")`；`pose.yaw_deg` 不做 wrap（旋转对 360° 周期不敏感）。
- 数值稳定性：内联的逆旋转改为调用 `trajectory_transform.horizontal_body_offset`，两处算式合并成一份（现 `planner/services/target_coordinates.py:30-34` 与 `planner/services/trajectory_transform.py:17-19` 是同一段算术的两次实现）；`dz` 直接取世界差，z 轴向下不需要取反。
- 单测点：`yaw=0` 时 (f,r,d) == (dx,dy,dz)；`yaw=90` 时前方点落到 right 轴；`yaw=170` 与 `yaw=-190` 输入结果相同；前后向与右向两个符号约定各一个用例。

**`horizontal_body_offset(yaw_deg: float, dx: float, dy: float) -> tuple[float, float]`（新增）**
- 输入输出：yaw 与世界系偏移 → 机体系 (forward, right)。
- 边界条件：无；调用方已保证有限值。
- 数值稳定性：一次 `cos/sin`；两次乘法加法，无除法。这是 `point_in_horizontal_body`、`world_pose_delta_in_reference_frame`、纯跟踪车体系分解三处共用的唯一实现。
- 单测点：与旧 `world_pose_delta_in_reference_frame` 的 dx/dy 逐位相等（等价性回归）；`yaw=0/90/±180` 三个方向的符号约定。

**`world_pose_delta_in_reference_frame(start: WorldPose, end: WorldPose) -> RelativePoseDelta`**
- 输入输出：两个世界位姿 → (dx, dy, dz, dyaw_deg)，平移在 `start` 的水平机体系，`dyaw_deg` 为最短转并做 `wrap_yaw_deg`。
- 边界条件：`dyaw_deg` 用 `wrap_yaw_deg`，小于 0.5 毫度会吸附成 0；这是合同约定，不能换成 `wrap_deg`。
- 数值稳定性：平移经 `horizontal_body_offset`；`wrap_yaw_deg` 用取模而不是 `math.fmod`，负角正确。
- 单测点：`start == end` 时四项全 0；180° 差的符号取 `wrap_yaw_deg` 的值；跨 ±180 的一小步（179.9 → -179.9）得到 +0.2°。

**`relative_to_world(trajectory: RelativeTrajectory, start: WorldPose) -> WorldTrajectory`**
- 输入输出：机体相对增量序列与起点 → 世界位姿序列，并把 `yaw_deltas_deg` 原样带出。
- 边界条件：每一点相对**前一个预测位姿**展开，不是相对起点；空 `points` 合法，返回 `WorldTrajectory(start, ())`。
- 数值稳定性：yaw 累积用 `(yaw + dyaw + 180) % 360 - 180`，明确不用 `wrap_yaw_deg`（小角吸附会丢掉指令元数据，见现文件 35-37 行的注释）；平移用 `domain.pose.relative_to_world`（`planner/domain/pose.py:54`），不再内联。
- 单测点：单点 `dx=1, dyaw=0, yaw=0` → x+1；`dyaw=+30` 累积两段与手算一致；与 `world_to_relative` 往返在 1e-9 内相等。

**`world_to_relative(trajectory: WorldTrajectory) -> RelativeTrajectory`**
- 输入输出：世界位姿序列 → 机体相对增量序列，`dyaw_deg` 优先取 `yaw_deltas_deg`。
- 边界条件：`yaw_deltas_deg` 为 `None` 时用最短转；长度不一致由 `WorldTrajectory.__post_init__`（`planner/domain/trajectory.py:31-43`）在构造时拦住，本函数不重复校验。
- 数值稳定性：逐段调用 `world_pose_delta_in_reference_frame`，无累积误差放大（每段独立）。
- 单测点：往返恒等（属性测试，10 组随机位姿）；`yaw_deltas_deg=(+179.9, ...)` 时保留符号而不是折成 -180。

### 1.4 深度

**`SurfaceDepthSample`（frozen dataclass）**：`valid: bool`、`reason: str`、`depth_m: float | None`。`reason` 是给诊断轨迹与提示词看的机器串（`mode=direct_pixel; pixel=[u,v]; ...`），改文案会改 `tests/test_capability_targeting.py:179` 之后的断言与 `adapters/diagnostics` 的留档内容。

**`sample_surface_depth(depth, pixel, bbox, rgb_intrinsics, depth_intrinsics, minimum_m) -> SurfaceDepthSample`**
- 输入输出：原始深度图、深度网格上的浮点像素、归一化 bbox、两套内参、最小深度 → 一个像素读数。
- 边界条件：像素非有限或越界 `[0,width) × [0,height)` → 无效；`int(u), int(v)` 向零截断；把该像素映射回 rgb 归一化坐标后必须仍在 bbox 内，否则无效；深度值需有限、`> 0`、`>= minimum_m`。任何一条不满足都返回 `valid=False` 与原因，不抛异常。
- 数值稳定性：单像素直读，无邻域、无插值、无显示上限；`depth[y, x]` 用整数索引避免浮点下标；阈值比较是 `value > 0` 与 `value >= minimum_m`，恰好等于 `minimum_m` 算有效。
- 单测点：中心像素读出原值；越界像素返回 `outside depth coverage`；映射后落在 bbox 外返回 `outside target bbox`；`minimum_m` 相等/小 1e-9 两个方向；`int()` 截断而不是四舍五入（0.99 → 0）。

**`DepthForegroundFilter.filter(observation, bbox, depth, minimum_m, samples, pixels) -> tuple[tuple[SurfacePointEstimate, ...], DepthForegroundResult]`**
- 输入输出：观测、bbox、深度图、最小深度、逐点样本与像素 → 逐点样本（可能被标为无效）与过滤证据。
- 边界条件：`settings.enabled is False` → 原样返回并给 `applied=False, reason="disabled"`；局部窗口 `radius = settings.pixel_radius`（1..32，`planner/domain/depth_foreground.py:15`）夹到图边；有效样本少于 2 个或空间差分集为空 → 该点不产生层证据。
- 数值稳定性：`forward = depth / sqrt(1 + rx² + ry²)`，分母 ≥ 1，不会除零；排序后取最大间隙，阈值 `max(数值噪声, ratio × median(|spatial diff|))`，数值噪声 `eps × max(1, max_forward) × 64`；两侧都必须有相邻像素才认定层，避免单点孤立层。窗口一律取深度网格，只有 u/v 归一化映射回 rgb 网格，这是 AirSim 两种分辨率共存时的正确坐标系（现文件 42-45 行注释记录过一次搞错）。
- 单测点：禁用时逐点样本不变；完全平坦窗口不产生层；`depth_gap_ratio` 边界（间隙恰等于阈值）不产生层；rgb 与 depth 不同分辨率时窗口仍在深度网格上（用 2 倍分辨率差构造）。

**`DepthGeometry.validate(obs) -> np.ndarray`**
- 输入输出：观测 → 深度数组（校验通过才返回），失败抛 `ValueError`。
- 边界条件：`verified` 为假、`depth_kind` 与 `depth_is_range` 不匹配、`depth_intrinsics is None`、深度形状不等于 depth 内参、rgb 形状不等于 rgb 内参、`fx/fy` 非有限或非正、宽高非正、`cx/cy` 非有限、旋转不是 3×3 或非正交或行列式不为 1、相机原点非有限，逐条抛带原因的 `ValueError`。
- 数值稳定性：正交性用 `allclose(RᵀR, I, atol=1e-3)`、行列式 `atol=1e-3`，吸收浮点与四元数转矩阵误差；`np.isfinite` 全量检查而不是抽样。
- 单测点：`verified=False` 抛（`tests/test_depth_geometry.py:135`）；`depth_kind="ray_range"` 配 `depth_is_range=False` 抛（`:142`）；旋转矩阵加 2e-3 扰动抛，加 5e-4 扰动通过；深度形状差一行抛。

**`DepthGeometry.ray(obs, point) -> np.ndarray`（静态）** 与 **`DepthGeometry.project_camera(obs, camera_point) -> tuple[float, float] | None`（静态）**
- 输入输出：归一化图像点 → 单位方向向量；相机系点 → 深度网格像素。
- 边界条件：`project_camera` 在 `camera_point[0] <= 0` 时返回 `None`（相机背后的点不投影，否则符号翻转会得到一个看似合法的像素）。
- 数值稳定性：`ray` 先构造 `(1, rx, ry)` 再归一，`dir[0] = 1/‖·‖ ≥ 1/√(1+rx²+ry²)` 恒正，后续 `depth/ray[0]` 不可能除零；`project_camera` 用 depth 内参，`ray` 用 rgb 内参，两套内参不得混用。
- 单测点：图像中心点的 `ray` 为 `(1,0,0)`；`project_camera(ray(p))` 回到 `p` 的像素（往返）；`camera_point[0]=0` 与负值都返回 `None`。

**`DepthGeometry.estimate(obs, goal) -> GoalGeometry`**
- 输入输出：观测与模型给的框/表面点 → 可靠性与世界点、逐点样本、过滤证据。不抛异常，失败折成 `reliable=False` 与原因串。
- 边界条件：模型给点就只用模型点（空 `surface_points` 不退化成 bbox 中心）；缺 bbox 或缺点 → `reliable=False`；全部样本无效 → 把各点原因用 `"; "` 连起来；多份有效样本取 `distance_m` 最小者；捕获 `ValueError, TypeError, IndexError` 三类，其余异常（含 `numpy` 的 `LinAlgError`）上抛。
- 数值稳定性：光轴深度换算成射程 `range_m = depth_m / ray[0]`，`ray[0] > 0` 保证单调且不除零；`depth_is_range=True` 时直接用 `depth_m`；位置由 `R @ (ray × range) + origin` 得到，不经过欧拉角拼装。
- 单测点：轴心样本反投影落在光轴上（`tests/test_depth_geometry.py:55`）；离轴样本横向偏移增大（`:67`）；最近有效样本被选中（`:105`）；`ray_range` 深度得到径向距离（`:149`）；空点集不退化成中心点（`:115`）。

**`measure_free_space(observation, settings: FreeSpaceProfileSettings | None = None) -> FreeSpaceProfile`**
- 输入输出：观测与设置 → 每个方位扇区最近的高于地面障碍，加 `max_range_m` 与高度带。
- 边界条件：`settings` 省略时用 `FreeSpaceProfileSettings()` 默认值；深度形状不等于 depth 内参抛 `ValueError`；`depth_kind` 不在 `{optical_depth, ray_range}` 抛 `ValueError`；像素无效判据是「非有限、`<= 0`、`>= 65500`（`planner/domain/depth.py:6` 的哨兵）」；水平距离超过 `max_range_m` 丢弃；整帧无有效像素时所有扇区 `ground_available=False`（未知，不是畅通）；某扇区没有点或没有够点数的距离 bin 时同样 `ground_available=False`；障碍像素少于 `min_obstacle_pixels` 时 `distance_m=None` 且 `ground_available=True`（实测畅通）。
- 数值稳定性：每 bin 地面高度取 90 百分位（抗 outlier），空 bin 用 `np.interp` 在已知 bin 上插值、两端钳住；`ray_range` 先除以 `sqrt(1+rx²+ry²)` 换成光轴深度；两处取整（方位归扇区用 `np.rint`、扇区中心回索引用 `round`）都是银行家舍入，扇区中心恒为 `width` 的整数倍，映射精确且与现状一致；障碍距离取排序后第 `floor(p/100 × n)` 个并夹到 `n-1`，不用均值。
- 单测点：平地无遮挡 → `nearest_ahead_m is None` 且 `clear_window_deg == (-40, 40)`（`tests/test_free_space.py:82-87`）；正前方墙 → 10 m ± 0.2（`:90-97`）；右侧护栏 → 正方位角（`:101`）；斜坡自标定（`:112`）；全无效深度 → `nearest_ahead_m is None` 且 `clear_window_deg is None`（`:132-138`）；`ray_range` 与 `optical_depth` 同几何（`:141`）。

**`sector_centers(observation, settings) -> tuple[float, ...]`（现 `_sector_centers`，改公开）** 与 **`measure_sector(center, forward, right, down, horizontal, settings) -> FreeSpaceSector`（现 `_measure_sector`）**
- 输入输出：观测/设置 → 扇区中心列表；单扇区数组 → `FreeSpaceSector`。
- 边界条件：半视场角除不尽扇区宽时取整到内侧（`ceil(-half/width)`、`floor(half/width)`）；一个扇区都取不到时退回 `(0.0,)`；`forward.size == 0` 或没有地面 bin 时 `ground_available=False`。
- 数值稳定性：`±1e-9` 的取整偏移保证半视场恰为扇区宽整数倍时边界只算一次；`np.percentile` 在 `min_points_per_bin` 之上才调用。
- 单测点：视场 100°、宽 10° → 中心 `(-40…40)`；视场 95° → 仍是 `(-40…40)`；视场 4°、宽 10° → `(0.0,)`；这两个函数改为公开后 `tests/test_free_space.py` 直接调用，不再借 `measure_free_space` 间接测。

**`FreeSpaceProfile.nearest_ahead_m` / `clear_window_deg`**
- 输入输出：扇区列表 → 正前方最近距离；最长畅通窗口的两端方位。
- 边界条件：无扇区时 `None`；窗口平局取离正前方更近的一段（现实现用 `-abs(中线方位)` 作次键）。
- 数值稳定性：整数索引求窗口，不插值；连续畅通段用一次扫描。
- 单测点：两侧等长窗口取靠近正前方的一段；全部未知时不报畅通（`clear_window_deg is None`）。
- 现文件 `_measured()`（`planner/services/free_space.py:99-104`）无调用方，迁移时删除。

**`FreeSpaceProfileSettings.__post_init__`**
- 输入输出：9 个构造参数 → 校验或抛 `ValueError`。
- 边界条件：`sector_width_deg`、`max_range_m`、`distance_bin_m` 必须有限正；`min_points_per_bin`、`min_obstacle_pixels` 必须是正整数（拒 bool）；两个百分位必须在 0–100；高度带必须 `0 < min < max`。
- 数值稳定性：拒绝 bool 与非有限值，避免 `int(True)` 混进点数阈值；百分位用闭区间。
- 单测点：参数化负例见 `tests/test_free_space.py:176`（现有 9 组），迁移后保持断言不动。

### 1.5 序列与轨迹校验

**`require_execution_anchor(measured, expected, position_tolerance_m, yaw_tolerance_deg) -> None`**
- 输入输出：实测位姿、期望位姿（队首端点）、两个容差 → 通过或抛 `ExecutionError`。
- 边界条件：三维距离 `> position_tolerance_m + 1e-6` 或偏航差 `> yaw_tolerance_deg + 1e-6` 即失败。
- 数值稳定性：`math.dist` 走三维；偏航差经 `wrap_yaw_deg`（含 0.5 毫度吸附，与合同一致）；`1e-6` 吸收浮点累加，不是精度声明。
- 单测点：距离恰好等于容差通过、容差 + 2e-6 失败；偏航恰好等于容差通过；跨 ±180 的 0.1° 差不误判。

**`TrajectoryValidator(expected_points: int, max_step_m: float, max_yaw_step_deg: float)`**
- 输入输出：构造参数 → 可复用的校验器（不可变配置，无状态）。
- 边界条件：`type(expected_points) is not int` 或不在 1–5 → `ValueError`（`type(...) is int` 拒 bool 与 float）；`max_step_m`、`max_yaw_step_deg` 必须有限正；`max_yaw_step_deg > 180` → `ValueError`。
- 数值稳定性：不做浮点换算，比较用原值；上界 180 与 `WorldTrajectory.__post_init__` 的同一上界对齐。
- 单测点：`(5, 2.0, 45.0)` 通过；`(True, 2.0, 45.0)`、`(0, ...)`、`(6, ...)`、`(5, 0.0, 45.0)`、`(5, 2.0, 181.0)` 各抛（`tests/test_batch3_services.py:70` 现有正例保留）。

**`TrajectoryValidator.validate_relative(trajectory) -> None`**
- 输入输出：相对航迹 → 通过或抛 `ProtocolError`。
- 边界条件：点数必须在 `1..expected_points`；每点三维位移 `hypot(dx,dy,dz) > max_step_m`、`|dyaw_deg| > max_yaw_step_deg` 即拒。模型一侧另行要求恰好 5 点，由调用方（`planner/capabilities/sliding_window/planning.py:99`）判，本函数只做上限。
- 数值稳定性：三维模长一次 `hypot`；不含容差（相对量是模型原值，不需要吸收累积误差）。
- 单测点：1 点与 5 点通过，0 点与 6 点拒；位移恰好等于上限通过，超 1e-9 拒；`|dyaw_deg|` 恰好等于 `max_yaw_step_deg` 通过、超 1e-9 拒；`max_yaw_step_deg=180` 时 ±180 通过。

**`TrajectoryValidator.validate_world(trajectory) -> None`**
- 输入输出：世界航迹 → 通过或抛 `ProtocolError`。
- 边界条件：点数同 `validate_relative`；逐段三维距离 `> max_step_m + 1e-6`；偏航步长经 `resolve_yaw_delta_deg(previous.yaw, pose.yaw, explicit)` 解析，显式增量与端点矛盾时 `resolve_yaw_delta_deg` 先抛 `ValueError`（`planner/domain/pose.py:48-50`）。
- 数值稳定性：世界坐标由累加得到，因此这里保留 `1e-6` 容差；偏航有显式增量时以增量为准，避免 180° 边界翻向。
- 单测点：相对→世界→校验整条链在合法航迹上通过；篡改末点使单步超限被拒；显式 `yaw_deltas_deg` 与端点矛盾时抛 `ValueError` 而不是 `ProtocolError`（区分合同违约与预算违约）。

### 1.6 运动律

**`limit_vector(values: tuple[float, ...], maximum: float) -> tuple[float, ...]`**
- 输入输出：任意维向量与上限 → 按模长等比缩放后的向量。
- 边界条件：`maximum` 非有限或 `<= 0`、任一分量非有限 → `ExecutionError("non-finite motion vector or invalid limit")`；模长为零返回原向量（零向量）。
- 数值稳定性：`hypot(*values)` 抗溢出，`scale = min(1, maximum/norm)` 只在超限时缩放，不放大。
- 单测点：`(3,4)` 上限 2 → `(1.2,1.6)`；零向量原样返回；`nan` 分量抛 `ExecutionError`；与 `simulators/projectairsim/motion.py:35` 的旧实现逐值一致（等价性回归）。

**`tracking_velocity(reference_velocity, error, gain, maximum) -> tuple[float, ...]`**
- 输入输出：前馈速度、位置误差、比例增益、上限 → 限幅后的期望速度。等价于现两份 tracker 的 `limit_vector(tuple(v + gain*e ...), speed)`。
- 边界条件：维度一致（同长度 tuple），否则 `zip` 静默截断，故长度不同直接抛 `ValueError`；上限校验交给 `limit_vector`。
- 数值稳定性：先加后限，与现状同序（先限后加会放大误差项）。
- 单测点：误差为零时等于前馈限幅结果；误差使模长超限时方向不变、模长等于上限；长度不等抛 `ValueError`。

**`slew_limited(previous_values, desired_values, maximum_change) -> tuple[float, ...]`**
- 输入输出：上一拍值、本拍目标、单拍最大变化量 → 按变化量限幅后的本拍值。等价于现 tracker 的 `limit_vector(tuple(v - p ...), max_accel*period)` 再加回去。
- 边界条件：`maximum_change` 非有限或非正 → `ExecutionError`；长度不等 → `ValueError`。
- 数值稳定性：作用在差值上再相加，保证 `|new - previous| <= maximum_change`（范数意义），不会因坐标轴分别裁剪而改变方向。
- 单测点：目标在允许变化内时直通；超限时模长等于 `maximum_change`；连续两拍的差值范数不超过上限。

**`command_bandwidth(acceleration: float, jerk: float) -> float`**
- 输入输出：加速度与 jerk 上限 → 临界阻尼带宽 `min(4, 2*sqrt(jerk/acceleration))`。
- 边界条件：任一参数非有限或非正 → `ValueError`（现状由 `MotionLimits`／`ContinuousTrackingSettings` 保证）。
- 数值稳定性：`sqrt(jerk/acceleration)` 先除后开，不放大溢出；上限 4 是稳定性的来源（见下一条）。
- 单测点：`acceleration=1.5, jerk=3.0` → `2*sqrt(2) = 2.828`；`jerk` 极大时被钳到 4.0；`acceleration=0` 抛。

**`smoothed_step(desired, velocity, acceleration, *, acceleration_limit, jerk_limit, bandwidth, dt) -> tuple[tuple[float, ...], tuple[float, ...]]`**
- 输入输出：期望速度、上一拍速度与加速度、两个上限、带宽、周期 → (新速度, 新加速度)。纯函数，状态由 `runtime/motion/limits.py::CommandLimiter` 持有（`03-runtime.md:30`）。
- 边界条件：`dt <= 0` → `ValueError`；`acceleration_limit`、`jerk_limit` 非正 → `ValueError`；`desired` 与 `velocity` 长度不等 → `ValueError`；`dt` 由执行层夹到 `[0.001, 0.25]`（现 `planner/services/continuous_execution.py:123`）。
- 数值稳定性：jerk 先按模长限度、加速度再积分为球投影、速度用中点法 `v += (a_old + a_new)/2·dt`；`bandwidth × dt ≤ 4 × 0.25 = 1 < 2` 是显式欧拉在临界阻尼下的稳定域；投影到加速度球非扩张，不会把已可行的状态放大（现文件 29-30 行注释的同一理由）；`bounded` 独立导出，`position_hold` 用它做无状态的模长钳位（`planner/services/position_hold.py:90`）。
- 单测点：静止起步的第一拍速度增量 `<= acceleration_limit·dt`；`desired` 恒定且远大于上限时速度收敛到与上限同模长；`dt=0.25` 与 `dt=0.01` 分多步走完同一时间的终值差 < 1%；`dt=0` 抛 `ValueError`；与现 `CommandSmoother.step` 在同一状态序列上逐值一致（等价性回测）。

**`quintic(tau: float) -> float`**
- 输入输出：归一化时间 → 归一化位移 `10τ³ - 15τ⁴ + 6τ⁵`。
- 边界条件：`τ` 不裁剪；调用方保证 `τ ∈ [0,1]`；`τ<0` 或 `>1` 会得到非单调值，属调用方违约。
- 数值稳定性：多项式直算，无除法；端点 `quintic(0)=0`、`quintic(1)=1` 精确（整数系数在双精度下可精确表示）。
- 单测点：0/0.5/1 三个点（`tests/test_motion.py:13`）；51 点单调（`:19`）；`quintic(1-1e-12) <= 1`。

**`duration_for(delta: RelativePoseDelta, limits: MotionLimits) -> float`**
- 输入输出：相对增量与合并后的 `MotionLimits` → 段时长。
- 边界条件：平移为零只看偏航，偏航为零只看平移，两者都为零返回 `limits.min_duration_s`；`max_speed_mps`、`max_acceleration_mps2`、`max_yaw_rate_deg_s`、`max_yaw_acceleration_deg_s2` 由 `domain.motion.MotionLimits.__post_init__` 保证为正（`docs/refactor/01-contracts.md` 的 18 字段表）。
- 数值稳定性：四个约束取最大值，峰值常数用解析值 `1.875 = 30·(1/2)⁴`、`10/√3`（`s''` 极点），不做数值求导；`sqrt(peak_accel·distance/max_accel)` 的被开方数非负。
- 单测点：0.01 m 位移被 `min_duration_s` 兜住（`tests/test_motion.py:27`）；位移翻倍时长翻倍；纯偏航不受平移上限影响；`max_accel_mps2` 减半时长乘 `√2`。

**`plan_segment(start: WorldPose, delta: RelativePoseDelta, limits: MotionLimits) -> list[SegmentSetpoint]`**
- 输入输出：起点与相对增量 → 按 `limits.control_hz` 采样的位姿序列。
- 边界条件：步数 `max(2, ceil(duration*control_hz)+1)`；首点等于 `start`；末点等于 `start.offset(delta)`；偏航用 `wrap_deg`（`(-180,180]` 语义）而不是 `wrap_yaw_deg`。
- 数值稳定性：`τ = index/(steps-1)`，`steps >= 2` 保证不除零；端点值由 `quintic` 精确给出，不靠采样逼近。
- 单测点：端点（`tests/test_motion.py:32`）；`170°+30° → -160°`（`:43`）；步数等于 `max(2, ceil(duration*20)+1)`（`:50`）。
- 现状事实：`plan_segment` 没有生产调用方，只有 `tests/test_motion.py` 与 `planner/services/__init__.py:7,15` 的导出；去留见 §8 第 7 条。

**`duration_for_poses(start, end, speed, limits, *, yaw_delta_deg=None) -> float`（现 `synchronized.duration_for`，改名）**
- 输入输出：两个世界位姿、巡航速度、`MotionLimits`、可选显式偏航增量 → 同步 profile 的最短时长。
- 边界条件：`speed` 非有限或非正 → `ValueError`；五项取最大（`min_duration_s`、速度、偏航速度、加速度、偏航加速度）。
- 数值稳定性：峰值加速度用 `10/math.sqrt(3)` 而不是字面小数；偏航用 `resolve_yaw_delta_deg` 保号。
- 单测点：零位移零偏航返回 `min_duration_s`；位移与速度成正比；显式 `yaw_delta_deg=-180` 与端点一致时走全半转时长。
- 命名的理由：本层同时有 `quintic.duration_for(delta, limits)`，两者入参语义不同（一个吃相对增量、一个吃两个位姿加巡航速度），保留同名会让一次误调用静默算错时长。改名点只有两处调用：`simulators/projectairsim/motion.py:88`、`simulators/airsim/motion_tracker.py:57`。

**`sample_segment(segment: MotionSegment, elapsed: float) -> MotionSample`**
- 输入输出：运动段与已用时间 → (位姿, 世界速度, 偏航角速度)，`u = clamp(elapsed/duration, 0, 1)`。
- 边界条件：`duration_s` 为 `None`、非有限或非正 → `ValueError`；`elapsed` 超出时长时停在终点，不外推。
- 数值稳定性：`rate = 30u²(1-u)²/duration`，峰值 `1.875/duration`，与 `duration_for_poses` 的速度约束互为逆运算；速度用世界差乘 `rate`，不重新求导位姿。
- 单测点：`elapsed=0` 返回起点与零速度；`elapsed=duration` 返回终点；`u=0.5` 的速度等于 `1.875·distance/duration`；`elapsed=2×duration` 与 `elapsed=duration` 相同。

**`SynchronizedMotionPlanner(speed_mps, limits).create_motion(trajectory) -> MotionTrajectory`**
- 输入输出：世界航迹 → 逐段 `MotionSegment`，每段在航点停稳。
- 边界条件：`speed_mps` 由装配层从 `config.motion.max_speed_mps` 传入；`yaw_deltas_deg` 为 `None` 时每段留 `None`。
- 数值稳定性：逐段独立计算时长，不跨段累积。
- 单测点：段数等于航点数；每段 `start` 等于上一段 `end`；`profile == SMOOTH_PROFILE`；`target_speed_mps` 全等于入参。

**`stopping_distance(speed, acceleration, jerk, current_acceleration=0., response_time=0.) -> float`**
- 输入输出：速度、加速度上限、jerk 上限、当前加速度、执行链响应时间 → 停车所需距离。
- 边界条件：低速走三角 profile（`v <= a²/j`），高速走梯形；`response_time` 直接乘速度加到距离上；负速度取绝对值处理。新增校验：`acceleration <= 0` 或 `jerk <= 0` → `ValueError`（现实现会除零或返回 inf）。
- 数值稳定性：先抵消当前加速度（`neutral_time = a/jerk`、`neutral_distance = v·a/jerk + a³/(3j²)`），再按 `v' = speed + a²/(2j)` 算停车段，全程闭式，无迭代。
- 单测点：`speed=0` 返回 `0`；`jerk→0` 抛 `ValueError`；当前加速度与运动同向时距离单调不减；`response_time=0.1` 恰好多 `0.1·speed`。

**`braking_speed(distance, end_speed, maximum_speed, acceleration, jerk, current_acceleration=0., response_time=0.) -> float`**
- 输入输出：剩余距离、末端速度、速度上限、加速度/jerk 上限 → 该距离下允许的最大速度。
- 边界条件：`end_speed >= maximum_speed` 直接返回 `maximum_speed`；`distance <= 0` 返回 0；其余用 24 次二分求解 `stopping_distance(v) - stopping_distance(end_speed) <= distance`。
- 数值稳定性：二分不用牛顿（`stopping_distance` 在三角/梯形切换处分段但单调连续）；24 次迭代分辨率 `maximum_speed/2²⁴`，在 5 m/s 下约 3e-7 m/s，远小于任何执行器分辨率；`max(0., distance)` 防负距离破坏单调性。
- 单测点：距离为 0 返回 0；距离很大返回上限；`end_speed == maximum_speed` 返回上限；结果对 `distance` 单调不减（属性测试）；与 `stopping_distance` 互为逆（往返误差 < 1e-3）。

**`angular_duration(angle, limits, settings) -> float`**
- 输入输出：偏航角（取绝对值）与两套参数 → 静止到静止的偏航时间预算。
- 边界条件：`angle == 0` 返回 `0.0`；三项取最大（速度、加速度、jerk 的立方根项）。
- 数值稳定性：`(60·angle/jerk)^(1/3)` 的被开方数为非负；jerk 项只在 `angle > 0` 时参与。
- 单测点：0° 返回 0；90° 由速度项支配（`1.875·90/30 = 5.625 s`）；把 `max_yaw_rate_deg_s` 抬到 300 后改由 jerk 项支配（`(60·90/60)^(1/3) = 4.48 s`）。

**`corner_speed(before, after, limits, settings, cruise, response_time) -> float`**
- 输入输出：相邻两段航点与参数 → 过弯速度。
- 边界条件：任一段长度 `<= 1e-9` 返回 0；夹角一半 `< 1e-6`（共线）返回 `cruise`；`cos(half) < 1e-6`（掉头）返回 0。
- 数值稳定性：`cosine` 夹到 `[-1, 1]` 再 `acos`；半径取 `min(position_tolerance/(1/cos(half)-1), 0.45·min(len)/tan(half))`，两条几何界取小；速度取 `min(cruise, sqrt(max_accel·radius), transition)`，`transition` 用 `braking_speed(position_tolerance/max(2·sin(half),1e-9), ...)`，`1e-9` 防掉头时除零。
- 单测点：共线两段返回 `cruise`；90° 转弯的速度小于 `cruise` 且大于 0；掉头返回 0；零长段返回 0；夹角变大时速度单调不增。

**`corner_entry_speed(previous: QueuedWaypoint | None, head: QueuedWaypoint, limits, settings, cruise, response_time) -> float`**
- 输入输出：上一段、当前队首与参数 → 进入当前段的入口速度（现 `planner/services/motion/continuous.py:60-71` 的内联分支）。
- 边界条件：`previous is None` 或 `previous.waypoint_id == head.waypoint_id` 返回 `cruise`；任一段长度 `<= 1e-9` 返回 `cruise`；方向余弦 `|cos| >= 1-1e-9` 视为直行返回 `cruise`；否则返回 `corner_speed(...)`。
- 数值稳定性：先算两段单位方向余弦，再决定是否进几何分支，避免零长段除零；`|cos| < 1-1e-9` 的严格不等号与现状一致（`1e-9` 吸收同向但长度不同的浮点差）。
- 单测点：队首未切换返回 `cruise`；直行接续返回 `cruise`；90° 转角返回小于 `cruise` 的值；`previous` 的 `waypoint_id` 与 `head` 相同返回 `cruise`（等价于现状）。

**`damped_approach_speed(norm, cruise, limits, settings, current_acceleration=0., response_time=0.) -> float`**
- 输入输出：到参考点的水平距离、巡航速度、参数 → 终点对齐阶段的阻尼速度（现 `continuous.py:83-91`）。
- 边界条件：`norm == 0` 返回 0；结果为 `min(cruise, position_gain·norm, braking_speed(norm, 0, cruise, a, j, |a_cur|, response))`。
- 数值稳定性：三项取最小保证不会以巡航速度冲过端点；`braking_speed` 已处理 `distance <= 0`。
- 单测点：`norm` 很小时与 `position_gain·norm` 相等（线性段）；`norm` 很大时等于 `cruise`；中间段与手算 `braking_speed` 一致；`norm=0` 返回 0。

**`planned_speed(items, reference: ContinuousReference, limits, settings, cruise, *, current_acceleration: float, bandwidth: float) -> float`**
- 输入输出：队列（队首在前）、当前参考、参数 → 本拍允许速度（整条已授权队列的包络）。
- 边界条件：跳过队首之外长度为零且非旋转的段；`duration == 0` 的段用 `cruise`；最后一段保留 `position_tolerance_m/2` 余量；`0 < boundary < cruise` 时再留一个前视距离的刹车余量。
- 数值稳定性：逐段用 `braking_speed` 取最小，全程单调收紧；`response = 2/bandwidth`、`current_acceleration` 由调用方（执行层）传入上一拍状态，本函数不持有状态。签名从 `smoother` 对象改成两个数值，理由是本层不收有状态对象；传错状态只会改变刹车余量，不会破坏单调性。
- 单测点：单段短队列速度由终段刹车支配；接近 90° 转角时速度在入弯前降到 `corner_speed`；零长段的 padding 不影响结果；`items` 只有队首时结果等于 `braking_speed(remaining_head_m - position_tolerance/2, 0, ...)`；与现实现逐值一致（等价性回归）。

**`xyz(pose)`、`segment_length(item)`、`is_rotation(item)`、`distance_to_segment(point, start, end)`**
- 输入输出：位姿 → 三元组；航点段 → 长度；航点段 → 是否纯旋转（长度 ≤ 1e-9 且 `|yaw_delta| > 1e-9`）；点到线段 → 距离。
- 边界条件：`distance_to_segment` 在 `length_sq == 0` 时 `fraction = 0`；点在线段外时钳到端点。
- 数值稳定性：全部 `hypot`/`dist`；`1e-9` 是零长判据，与 `reached` 的 `1e-12` 用途不同（前者比长度、后者比平方），保留两个常量并各自注明。
- 单测点：零长段 `is_rotation` 只有偏航非零时为真；线段外点的距离等于到端点距离；中点距离为 0。

**`ContinuousPathReference(settings, limits)`**
- `reset() -> None`：清 `head_id`、`progress`、`aligning`。
- `sample(items, pose, yaw_error, speed) -> ContinuousReference`：队首切换归零、沿弧长前视、跨段前视（切角受限）、锁存对齐。
- 边界条件：`length <= 1e-9` 时 `progress` 不动、`pure_rotation=True`；`aligning` 一旦置位就锁存（漂出位置圈不解除，避免刹车/加速振荡）；纯旋转段是预览屏障，不跳过；零长零偏航 padding 跳过；切角二分固定 16 次。
- 数值稳定性：`progress` 用 `max(progress, min(length, projected))` 单调不回溯；前视距离 `clamp(speed·lookahead_time_s, min_lookahead_m, max_lookahead_m)`；`0.45`/`0.5` 的几何系数保持现值；`position_tolerance_m·0.5` 是切角走廊，改动会改路线形状。
- 单测点：队首切换后 `progress == 0`；`progress` 不随测量抖动回退；`aligning` 置位后在位置圈外仍为真；预览不跨纯旋转段；切角结果与 `distance_to_segment <= position_tolerance/2` 一致。

**`corridor_default(model: MotionModel, limits: MotionLimits) -> float` 与 `passed_lateral_corridor(model: MotionModel, limits: MotionLimits) -> float`**（裁决 R6、R17）
- 输入输出：载具运动模型与合并后的 `MotionLimits` → 越点横向走廊；`corridor_default` 只给按模型的默认值。
- 边界条件：`corridor_default` 对 `ACKERMANN` 返回 `ACKERMANN_PASSED_LATERAL_TOLERANCE_M = 1.0`（裁决 R17），对 `HOLONOMIC` 返回 `limits.position_tolerance_m`；`passed_lateral_corridor` 返回 `max(limits.passed_lateral_tolerance_m, corridor_default(...))`，即配置只能抬高不能压低（裁决 R17）；`model` 不是 `MotionModel` 成员 → `ValueError`；配置值非有限或非正 → `ValueError`。
- 数值稳定性：无算术，只有 `max` 与分支；两个默认值都是米级常量，不存在量纲混用。`min_turn_radius_m` 不参与推导（裁决 R6 明确排除这条路）。
- 单测点：阿克曼默认 1.0；全向默认等于 `position_tolerance_m`（默认 0.3）；配置 0.2 时阿克曼仍得 1.0、全向仍得 0.3；配置 1.5 时两者都得 1.5（裁决 R17）。`tests/test_passed_corridor.py:56-61` 的「只能抬高」断言保留，期望值从 0.6 改成 1.0。
- 这个常量来自实车宽度量级，不是标定值；裁决 R17 把它从 0.6 抬到 1.0，理由是 0.6 为手工挑值、实测失败率偏高。

**`reached(waypoint, pose, unwrapped_yaw, goal_yaw, position_tolerance, yaw_tolerance, passed_lateral_tolerance=None) -> bool`**
- 输入输出：队首航点、实测位姿、连续偏航、目标连续偏航、两个容差、可选走廊 → 是否推进队列。
- 边界条件：距离判据（三维距离 `<= position_tolerance`）与越点判据（投影 `>= 1` 且横向 `<= corridor`）任一成立，且同时满足 `|goal_yaw - unwrapped_yaw| <= yaw_tolerance`；线段长度平方 `<= 1e-12` 时只靠距离判据；`passed_lateral_tolerance is None` 时用 `position_tolerance`。
- 数值稳定性：投影用 `dot(offset, delta)/|delta|²`；横向用 `offset - projection·delta` 的模长；`1e-12` 是平方量级，10 cm 级的段不会误判。
- 单测点：冲过 4.0 m 端点 0.14 m、横向 0.355 m 时窄走廊不判过（`tests/test_passed_corridor.py:41`），显式传 0.6 的用例仍判过（`:47`）；默认值改 1.0 后该用例参数不动，只加一条默认值断言（裁决 R17）；偏航未对齐时宽走廊也不判过（`:52`）；零长段只按距离。

**`pose_delta_measurement(previous: WorldPose, actual: WorldPose, dt: float) -> tuple[tuple[float,float,float], float, float, float]`**
- 输入输出：相邻两拍位姿与时间差 → (测量速度向量, 速度标量, 偏航角速度 dps, 偏航增量)。等价于现两份 tracker 各自的差值算术。
- 边界条件：`dt <= 0` → `ValueError`（现实现返回 0，属静默降级；改为显式拒绝，调用方在 `dt=0` 时不该问速率）。
- 数值稳定性：偏航增量先经 `wrap_yaw_deg` 再除 `dt`；速度用三维差除以 `dt`，与 `math.dist` 同源，保证「速度」与「位移」一致。
- 单测点：静止两拍返回零速度零角速度；跨 ±180 的一小步角速度为正；`dt=0` 抛 `ValueError`；UE5 用固定 `period`、UE4 用墙钟同构（同一输入同一输出）。

**`is_settled(elapsed, duration, distance, yaw_error, measured_speed, measured_yaw_rate, limits) -> bool`**
- 输入输出：七个量 → 是否停稳；`settled_since` 的锁存逻辑留在执行层。
- 边界条件：五个条件全满足才为真：`elapsed >= duration`、`distance <= position_tolerance_m`、`yaw_error <= yaw_tolerance_deg`、`measured_speed <= stopped_speed_mps`、`measured_yaw_rate <= stopped_yaw_rate_deg_s`。
- 数值稳定性：全部闭区间比较，无 epsilon；阈值全来自 `MotionLimits`，不在函数内写常数。
- 单测点：五个条件逐条放宽/收紧各一个用例；`elapsed < duration` 时即便位置到位也为假；两个 tracker 的现行判据逐值等价（等价性回归）。

**`within_tolerance(pose, goal, yaw_deg, limits) -> bool`**
- 输入输出：终态位姿、目标位姿、连续目标偏航、limits → 停稳后漂移检查。
- 边界条件：三维距离与偏航差任一超限即假。
- 数值稳定性：与 `is_settled` 用同一组阈值。
- 单测点：恰好等于容差为真；超 1e-9 为假；偏航超限为假。

**`heading_rate_deg_s(error_deg, measured_rate_dps=None, *, gain, maximum_rate_dps, acceleration_dps2, jerk_dps3, damping=0.0, deadband_deg=0.0, rate_deadband_dps=0.0, current_acceleration_dps2=0.0, response_time_s=0.0) -> float`**
- 输入输出：偏航误差（连续角）、实测偏航角速度 → 角速度指令（dps）。
- 边界条件：误差与实测角速度都落在死区内返回 0；`measured_rate_dps is None` 关掉阻尼项（唯一没有实测角速度的调用点）；结果钳到 `±maximum_rate_dps`。非有限输入 → `ValueError`（`_require_finite` 拒 bool）。
- 数值稳定性：`shaped = copysign(min(gain·|e|, braking_speed(|e|, ...)), e)`，刹车曲线只能压低比例项，大误差不会像裸比例项那样超调；阻尼用实测角速度而不是位姿差分（差分把抖动放大成指令，注释见现文件 8-11 行）；`response_time_s = 2/bandwidth` 由调用方算。
- 单测点：小误差斜率等于 `gain`；90° 误差被限到 `maximum_rate_dps`（`tests/test_heading_servo.py:29`）；阻尼减/加号（`:40`、`:46`）；死区（`:52`）；`None` 关阻尼（`:61`）；极端增益下不超上限（`:66`）；参数化 7 个误差角与手算一致（`:76`）。

**`tracked_heading_rate_deg_s(reference_rate_dps, error_deg, *, last_rate_dps, period_s, gain, maximum_rate_dps, acceleration_dps2, jerk_dps3) -> float`**
- 输入输出：参考角速度、偏航误差、上一拍指令、周期与四个增益 → 本拍角速度指令。
- 边界条件：结果先钳 `±maximum_rate_dps`，再按 `acceleration_dps2 · period_s` 限斜率；合成值非有限 → `ExecutionError("non-finite yaw control command")`。
- 数值稳定性：前馈与反馈相加后再钳位，避免前馈把反馈挤到饱和之外；斜率限制用上一拍指令而不是测量值。
- 单测点：误差为 0 时输出等于前馈；前馈超上限时被钳住；单拍变化不超过 `acceleration·period`；`period_s` 很小时输出接近目标值；`inf` 前馈抛 `ExecutionError`。

**`lookahead_distance(speed_mps: float, settings: ContinuousTrackingSettings) -> float`**
- 输入输出：当前速度与连续跟踪设置 → 前视距离。
- 边界条件：`clamp(speed·lookahead_time_s, min_lookahead_m, max_lookahead_m)`；`speed` 为 0 时取 `min_lookahead_m`。
- 数值稳定性：三参数均经 `ContinuousTrackingSettings.__post_init__` 校验为正且 `min <= max`（`planner/domain/motion.py:22`），`clamp` 不会反转。
- 单测点：`speed=0` 取 0.3；`speed=1.0` 取 0.6（`lookahead_time_s=0.6`）；`speed=10` 被压到 1.5；`min > max` 由 `ContinuousTrackingSettings.__post_init__` 拒，本函数不重复校验。

**`pure_pursuit_curvature(lateral_m: float, distance_m: float) -> float`**
- 输入输出：前视点在车体系的横向偏移与到前视点的水平距离 → 曲率（1/m）。
- 公式：`κ = 2·sin(α)/d`，`α = atan2(lateral, forward)`，`d = hypot(forward, lateral)`。因 `sin α = lateral/d`，等价形式 `κ = 2·lateral/d²`，实现取后者（少一次三角函数，且 `d` 已算）。
- 边界条件：`d <= 1e-6` 返回 0（前视点与车重合，曲率无定义）；`lateral` 符号决定左右转。
- 数值稳定性：`d` 取 `max(d, 1e-6)` 防除零；结果不在此处钳位，钳位在 `pursuit_solution` 用 `max_curvature` 做，便于单测分别验证。
- 单测点：`lateral=0` 返回 0；`lateral=d/2` 时 `κ = 1/d`（等价于 30° 前视角）；两种形式在 100 组随机点上相等（属性测试）；`d=0` 返回 0。

**`max_curvature(min_turn_radius_m: float) -> float`**
- 输入输出：最小转弯半径 → 允许曲率上限 `1/R_min`。
- 边界条件：`R_min` 非有限或 `<= 0` → `ValueError`。
- 数值稳定性：无。
- 单测点：`R=3.2` → 0.3125±1e-4；`R=0` 抛；`R=inf` 抛。

**`lateral_speed_limit(curvature_per_m: float, max_acceleration_mps2: float) -> float`**
- 输入输出：曲率与加速度上限 → 该曲率下允许速度 `sqrt(a/|κ|)`。
- 边界条件：`κ == 0` 返回 `inf`（直线不限制，调用方与 `cruise` 取小）；`max_acceleration_mps2 <= 0` → `ValueError`。
- 数值稳定性：用 `sqrt(a/|κ|)` 而不是 `sqrt(a·R)`，避免两次倒数累积误差；`κ` 极小（< 1e-9）按 0 处理。
- 单测点：`κ=1/3.2, a=1.5` → 2.19±0.01；`κ=0` 返回 `inf`；`κ` 翻倍速度乘 `1/√2`。

**`pursuit_solution(reference_position, pose: WorldPose, yaw_deg, speed_mps, limits, settings, capability: MotionCapability) -> PursuitSolution`**
- 输入输出：前视点、实测位姿、连续偏航、包络限速、两套参数、载具能力 → `PursuitSolution(curvature_per_m, yaw_rate_deg_s, forward_speed_mps)`。
- 边界条件：`capability.model is not MotionModel.ACKERMANN` 或 `min_turn_radius_m is None` → `ValueError`（构造期就该发现，`docs/refactor/01-contracts.md` 的 `MotionCapability.__post_init__` 要求阿克曼必有半径）；`speed_mps <= 0` 返回全零解（车不能原地转）。
- 数值稳定性：`(forward, lateral) = horizontal_body_offset(yaw_deg, dx, dy)`；`κ` 钳到 `±max_curvature(R_min)`；`forward_speed = min(speed_mps, lateral_speed_limit(κ, max_acceleration_mps2))`；`yaw_rate = degrees(forward_speed·κ)` 再钳到 `±max_yaw_rate_deg_s`；钳位顺序固定（先曲率、后速度、再角速度），保证同一 κ 下输出确定。
- 单测点：直线（`lateral=0`）输出 `κ=0`、`yaw_rate=0`；左偏 0.3 m、距离 0.3 m → κ 被钳到 `max_curvature(R_min)`（默认 ≈0.3124）；`R` 变小则钳位更紧；低速满舵时 `forward_speed` 被横向加速度压到 2.19 m/s 以下；非阿克曼能力抛 `ValueError`。

**`ackermann_command(solution: PursuitSolution, *, reverse: bool = False) -> CarCommand`**
- 输入输出：纯跟踪解与是否倒车 → `CarCommand(forward_speed_mps, yaw_rate_deg_s, reverse)`。
- 边界条件：`reverse=False` 时 `forward_speed_mps >= 0`；`reverse=True` 时取负；`CarCommand.__post_init__`（01 契约）校验符号与 `reverse` 一致，`allows_reverse=False` 由适配层 `require_supported` 拒。
- 数值稳定性：无算术，只做符号装配；不倒车的默认分支永不产生负速度，从结构上取消「目标落到身后 → 倒车」的路径（`docs/MOTION_MODEL_AND_CONTINUOUS_EXECUTION.md` §5.2）。
- 单测点：正速度正角速度 → `CarCommand(0.5, 12.3, False)`；`reverse=True` 得负速度；`solution.forward_speed_mps < 0` 抛 `ValueError`（解本身不该带符号）。

**`holonomic_velocity(error: tuple[float, float, float], speed: float) -> tuple[float, float, float]`**
- 输入输出：位置误差向量与速度 → 期望世界速度（现 `continuous.py:92` 的一行算式）。
- 边界条件：`‖error‖ == 0` 返回零向量；`speed <= 0` 返回零向量。
- 数值稳定性：`hypot(*error)` 一次；单位化后乘 `speed`，不放大误差。
- 单测点：方向与误差同向、模长等于 `speed`；零误差返回零向量；`speed=0` 返回零向量；与现算式逐值一致。

**`translation_handoff(start, target, speed, acceleration, jerk) -> WorldPose | None`**
- 输入输出：当前位置、停放目标、速度与两个上限 → 停稳前后的交接位姿；距离不足刹车距离时返回 `None`。
- 边界条件：`distance <= braking_distance` → `None`（短距离只靠端点保持）；`jerk <= 0` 或 `acceleration <= 0` → `ValueError`（新增显式校验）。
- 数值稳定性：三角/梯形刹车距离与 `stopping_distance` 同构（`speed·sqrt(speed/j)` 与 `.5·speed·(speed/a + a/j)`），两处常数必须一致，单测交叉验证。
- 单测点：短路返回 `None`；长距离返回点位于起点到目标连线上且距目标恰为刹车距离；`jerk` 极小走梯形分支；与 `stopping_distance(speed, a, j)` 相等。

**`RefillScheduler(pending_threshold: int, margin_s: float)`**
- `should_refill(count: int, duration_s: float) -> bool`：`1 <= count <= 5` 且（`count <= pending_threshold` 或 `duration_s <= latency_s + margin_s`）。
- `record_latency(latency_s: float) -> None`：`latency_s = 0.75·latency_s + 0.25·实测`。
- 边界条件：`count` 为 0（队列空）或大于 5（越界）一律不补；`pending_threshold` 与 `margin_s` 来自 `SlidingWindowSettings.refill_pending_count`（默认 3）与 `latency_margin_s`（默认 1.5），校验在 `planner/domain/sliding_window.py:26` 一侧。
- 数值稳定性：`latency_s` 初值 5.5 s 是实验估计（现注释已声明不是上界）；EMA 系数固定 0.75/0.25，等价于 4 拍时间常数，不会因单次长尾立即触发补点。新增校验：`record_latency` 的入参非有限或为负 → `ValueError`（现实现会把负值写进状态，之后 `should_refill` 永久偏向补点）。
- 单测点：`count=0` 不补；`count=2, threshold=2` 补；`count=4, duration=8.0` 且 `latency+margin=7.0` 时不补；`record_latency(1.0)` 后 `latency_s == 0.75·5.5+0.25`；两次 `record_latency` 收敛到观测值；负延迟抛 `ValueError`（`tests/test_batch3_services.py:165` 现有用例保留，新增两条）。

## 二、数据流与时序

### 2.1 目标接近：一次测量到一次下发

```text
ObservationSource.capture()                                    引擎，停稳后一帧
 → DepthGeometry.validate(obs)                                 校验并取深度
 → DepthGeometry.ray(obs, point)                               归一化图像点 → 单位方向
 → DepthGeometry.project_camera(obs, ray)                      rgb 点 → depth 网格像素
 → sample_surface_depth(depth, pixel, bbox, ...)               单像素读数（米）
 → DepthForegroundFilter.filter(...)                           逐点不连续过滤
 → DepthGeometry.estimate(...)                                 反投影 → 世界点 + 到车距离
 → nearest_surface(points, pose)                               首帧绑定最近的表面点
 → horizontal_distance / point_in_horizontal_body              提示词里的米制向量
 → 模型端点返回 5 个相对航点
 → TrajectoryValidator.validate_relative(...)                  点数、单步位移与偏航
 → relative_to_world(...) → validate_world(...)                世界航迹并复核
 → WorldWaypointQueue.load_batch(...)                          授权 5 点
 → 每 50 ms 一拍：planned_speed → reached → 命令               见 2.4
```

一帧观测只进一次 `DepthGeometry`；`estimate` 内部对每个模型点各跑一次 `sample_surface_depth`，点数上限 5（提示词协议）。`GoalGeometry.distance_m` 是最近有效表面点到车体的直线距离，不是光轴深度。

### 2.2 巡航：一次补点到一拍命令

```text
ObservationSource.capture()                                    引擎
 → measure_free_space(obs, settings)                           逐扇区最近高于地面障碍
 → FreeSpaceProfile.clear_window_deg / nearest_ahead_m          提示词里的米制扇面
 → 模型端点返回 5 个相对航点
 → TrajectoryValidator → relative_to_world                      同 2.1
 → RefillScheduler.should_refill(count, duration)               是否提前补点
 → 每 50 ms：ContinuousPathReference.sample → planned_speed
            → reached → heading_rate_deg_s → 命令              见 2.4
```

`measure_free_space` 只读这一帧深度，不需要相机高度常数；地面线逐距离 bin 自估，因此坡道不需要额外标定。扇面是证据不是地图：看不见的空间是未知（`ground_available=False`），不是畅通。

### 2.3 段执行：一次数值动作

```text
BasicMotionConverter.create_target(params, start)              参数 → 目标位姿
 → WorldTrajectory(start, (target,)) → TrajectoryValidator
 → SynchronizedMotionPlanner.create_motion(trajectory)          每段 duration_for_poses
 → Vehicle.execute_segment(segment, budget, checkpoint)         适配层
 → `runtime/motion/loop.py::ControlLoop.run_segment` 每个采样周期（`SEGMENT_PERIOD_S = 0.030`，裁决 R1/R9）：
      duration_for_poses(实测起点, 目标, speed, limits, yaw_delta)
      sample_segment(segment, elapsed)                          参考位姿/速度/角速度
      tracking_velocity(ref.velocity, error, position_gain, speed)
      slew_limited(last_velocity, desired, max_acceleration·period)
      tracked_heading_rate_deg_s(ref.yaw_rate, yaw_error, ...)
      pose_delta_measurement → is_settled → within_tolerance     停稳判定
```

引擎侧只有 `stream_command(command, duration_s)` 一个数据面动作，没有采样循环，也没有 tracker（裁决 R1）。`SEGMENT_PERIOD_S = 0.030` 与 `1/limits.control_hz` 两个周期常量都写在 `runtime/motion/loop.py`，不进配置（裁决 R9）。

本层只给纯函数；周期、预算、取消、暂停全在 `runtime/motion/loop.py` 与 `runtime/execution.py`。`sample_segment` 的 `u` 已经钳在 `[0,1]`，所以采样周期比时长更细或更粗都不会外推。

### 2.4 每拍时序（速度流，20 Hz）

| 顺序 | 调用 | 输入来源 | 典型值 |
| --- | --- | --- | --- |
| 1 | `pose_delta_measurement(previous, actual, dt)` | 引擎状态流 | `dt` 夹到 `[0.001, 0.25]`（`planner/services/continuous_execution.py:123`） |
| 2 | `corner_entry_speed(previous_waypoint, head, ...)` | 队列 + 上一拍队首 | 方向余弦 `\|cos\| < 1-1e-9` 才换算式 |
| 3 | `ContinuousPathReference.sample(items, pose, yaw_error, speed)` | 队列、位姿、连续偏航、上一拍速度 | 前视 0.3–1.5 m |
| 4 | `planned_speed(items, reference, ...)` | 整条队列 | 结果 ≤ 巡航速度 |
| 5 | `damped_approach_speed(...)` 或纯跟踪 | 对齐标志 | 对齐后速度线性收敛 |
| 6 | `holonomic_velocity` / `pursuit_solution` | 误差或车体系分解 | 见 §3.4 |
| 7 | `smoothed_step(desired, velocity, acceleration, ...)` | 上一拍速度与加速度（执行层 `CommandLimiter` 持有） | `bandwidth ≤ 4`，`w·dt ≤ 1` |
| 8 | `heading_rate_deg_s` / `pursuit_solution` 的角速度 | 偏航误差、实测角速度 | 0–30 dps |
| 9 | `reached(...)` | 队首、位姿、两个连续偏航 | 走廊按 `corridor.py`：全向 `position_tolerance_m`、阿克曼 1.0 m，配置只能抬高（裁决 R17） |

段执行与速度流共用第 7、8 步的两个限幅器与同一条偏航律，差别只在第 6 步的目标生成方式。速度流的每拍调用者与队列消耗者是 `runtime/motion/follower.py::QueueFollower`（裁决 R4）。

### 2.5 等价性论证：现有算式逐段对应

`ContinuousFollower`（`planner/services/motion/continuous.py`）与两份 tracker（`planner/simulators/projectairsim/motion.py`、`planner/simulators/airsim/motion_tracker.py`）的每一段算术，在新结构里有唯一落点；执行层只留状态、节拍与下发。裁决 R1 后两份 tracker 连同引擎侧采样循环删除：下表「留在执行层」的行全部落在 `runtime/motion/loop.py`（段）或 `runtime/motion/follower.py`（速度流）。

`ContinuousFollower.sample` 逐行：

| 现位置（`planner/services/motion/continuous.py`） | 现算式/职责 | 新落点 |
| --- | --- | --- |
| `continuous.py:60-62` | 队首切换检测（`waypoint_id` 变化） | 留在 `runtime/motion/follower.py`：只比较 id，属状态壳 |
| `continuous.py:63-70` | 重置 `entry_speed`，算两段方向余弦，必要时取 `corner_speed` | `algorithms/motion/speed.py::corner_entry_speed`；`entry_speed` 变量留在执行层 |
| `continuous.py:71` | `previous_waypoint = head` | 执行层 |
| `continuous.py:72` | `yaw_error = goal_yaw - yaw`（连续角，保留半转/全转符号） | 执行层一行减法；连续偏航由执行层维护（`ContinuousExecution` 现已在做，`planner/services/continuous_execution.py:63-64`） |
| `continuous.py:73-74` | `self.path.sample(...)` 并保存 `self.reference` | `algorithms/motion/reference.py::ContinuousPathReference.sample`；对象由执行层持有并在 `reset()`/`resume()` 时清 |
| `continuous.py:75-76` | `error = reference.position - pose`，`norm = hypot(*error)` | 全向分支：执行层两行（三减一 `hypot`）；阿克曼分支改用 `horizontal_body_offset` + `pursuit_solution` |
| `continuous.py:77` | `planned_speed(items, reference, ...)` | `algorithms/motion/speed.py::planned_speed`（签名改成收 `current_acceleration` 与 `bandwidth` 两个数值） |
| `continuous.py:78-82` | 入口段速度保持 `min(speed, entry_speed)` | 执行层一行 `min`；判据比较留在执行层（依赖 `entry_speed` 状态） |
| `continuous.py:83-89` | 对齐/纯旋转/尾部时用阻尼接近速度 | `algorithms/motion/speed.py::damped_approach_speed` |
| `continuous.py:90-91` | 纯旋转且误差 ≤ `position_tolerance/2` 时速度归零 | 执行层两行（依赖 `reference.pure_rotation` 与 `norm`） |
| `continuous.py:92` | `desired = error/norm·speed` | `algorithms/motion/pursuit.py::holonomic_velocity`；阿克曼分支换成 `pursuit_solution` + `ackermann_command` |
| `continuous.py:93` | `self.translation.step(desired, dt)` | `runtime/motion/limits.py::CommandLimiter` 持有状态，算术是本层的 `smoothed_step` |
| `continuous.py:95-103` | 偏航律（`gain=yaw_gain`、上限、加速度、jerk、`response=2/bandwidth`） | `algorithms/motion/heading.py::heading_rate_deg_s`；参数装配留在执行层 |
| `continuous.py:104` | `self.rotation.step(...)`，返回 `(velocity, yaw_rate)` | 同上（第二个 `CommandLimiter` 实例）+ 执行层组装 `HolonomicCommand` |
| `continuous.py:12-28` | `reached(...)` 与走廊默认值 | `algorithms/motion/reached.py::reached`、`algorithms/motion/corridor.py::passed_lateral_corridor`（裁决 R6） |

两份 tracker 的重复算术：

| 现位置（`planner/simulators/projectairsim/motion.py` / `planner/simulators/airsim/motion_tracker.py`） | 现算式 | 新落点 |
| --- | --- | --- |
| `motion.py:35-44` / `motion_tracker.py:20-27` | `limit_vector`（两份逐字相同） | `algorithms/motion/limits.py::limit_vector`，两份删除 |
| `motion.py:86-95` / `motion_tracker.py:57-59` | `max(segment.duration_s, duration_for(start, end, speed, limits, yaw_delta))` | `algorithms/motion/synchronized.py::duration_for_poses` |
| `motion.py:102-104` / `motion_tracker.py:64` | `replace(segment, start=实测, duration_s, yaw_delta_deg=剩余)` | `runtime/motion/loop.py`（重锚属执行流程，裁决 R1） |
| `motion.py:120-139` / `motion_tracker.py:99-106` | 连续偏航累积、测量速度、位移、偏航角速度 | `algorithms/motion/settle.py::pose_delta_measurement`；连续偏航累积留在 `runtime/motion/loop.py` |
| `motion.py:143-171` / `motion_tracker.py:110-123` | `settled` 五条件、`settled_since` 锁存、终态漂移复核 | `settle.py::is_settled`、`settle.py::within_tolerance`；`settled_since` 时间锁存留在 `runtime/motion/loop.py` |
| `motion.py:174` / `motion_tracker.py:126` | `sample_segment(segment, elapsed)` | `algorithms/motion/synchronized.py::sample_segment` |
| `motion.py:180-186` / `motion_tracker.py:128` | `limit_vector(ref.velocity + position_gain·error, speed)` | `algorithms/motion/limits.py::tracking_velocity` |
| `motion.py:187-191` / `motion_tracker.py:129-131` | `limit_vector(desired - last, max_accel·period)`，再相加 | `algorithms/motion/limits.py::slew_limited`；`last_velocity` 状态留在 `runtime/motion/loop.py` |
| `motion.py:194-203` / `motion_tracker.py:133-142` | `tracked_heading_rate_deg_s(...)` | `algorithms/motion/heading.py::tracked_heading_rate_deg_s`（签名不变） |
| `motion.py:204-207` / `motion_tracker.py:145-146` | 非有限检查 | 执行层（`ExecutionError`），`limit_vector` 内部另有校验 |
| `motion.py:210-212` / `motion_tracker.py:147-151` | `drive(...)` / `moveByVelocityAsync(...)` | 引擎适配层，不在本层 |
| `motion.py:96-119` / `motion_tracker.py:60-98` | 预算、超时、取消、`MotionPaused`、观测闸门、resume 重算 | 执行层，全部落在 `runtime/motion/loop.py`（裁决 R1） |

留在执行层的完整清单（不进 `algorithms/`）：所有状态变量（`previous_waypoint`、`entry_speed` 在 `runtime/motion/follower.py`；`last_velocity`、`last_yaw_rate`、`settled_since`、`measured_yaw` 在 `runtime/motion/loop.py`；`head_id`、`paused`、`holding` 在 `runtime/window/` 与 `runtime/continuous_execution.py`）、所有时间（`clock`、`period`、`dt` 夹取、`waypoint_timeout_s`）、所有中断（`budget`、`cancelled`、`pose_checkpoint`、`MotionPaused`）、队列消费顺序、遥测与记录、以及两个限幅器实例（`runtime/motion/limits.py::CommandLimiter`，各持速度与加速度两向量）。

等价性由三条验证保证：算术逐值一致（同一输入、同一 `dt`、同一状态下，新旧实现给出的 `velocity` 与 `yaw_rate` 位相同）；钳位顺序一致（曲率 → 速度 → 角速度，以及 jerk → 加速度 → 速度）；异常集合不扩大（`limit_vector` 的 `ExecutionError`、`trajectory_validation` 的 `ProtocolError`/`ExecutionError`、`estimate` 捕获的三类异常，与现状一致；新增的 `ValueError` 只在构造期与参数非法时出现）。

## 三、跨层接口

### 3.1 我提供什么

| 符号 | 签名 | 调用方 |
| --- | --- | --- |
| `horizontal_distance` | `(Point3, WorldPose) -> float` | `workflows/visual_goal_navigation/policy.py`、`algorithms/targeting/association.py` |
| `nearest_surface` | `(tuple[Point3, ...], WorldPose) -> Point3` | `algorithms/targeting/association.py` |
| `point_in_horizontal_body` | `(Point3, WorldPose) -> tuple[float, float, float]` | `adapters/trajectory_planner/prompts/*`、`workflows/visual_goal_navigation/targeting.py`、`algorithms/planning/geometric_planner.py` |
| `horizontal_body_offset` | `(yaw_deg, dx, dy) -> tuple[float, float]` | 本层三处 |
| `world_pose_delta_in_reference_frame` | `(WorldPose, WorldPose) -> RelativePoseDelta` | `planner/adapters/trajectory_planner/prompts/base.py` |
| `relative_to_world` | `(RelativeTrajectory, WorldPose) -> WorldTrajectory` | `runtime/window/`, `workflows/*`, `algorithms/planning/geometric_planner.py` |
| `world_to_relative` | `(WorldTrajectory) -> RelativeTrajectory` | `runtime/window/queue.py`, `runtime/window/window.py` |
| `measure_free_space` | `(Observation, FreeSpaceProfileSettings \| None) -> FreeSpaceProfile` | `adapters/trajectory_planner/prompts/{free_navigation,visual_goal_navigation}.py` |
| `DepthGeometry.estimate` | `(Observation, GoalObservation) -> GoalGeometry` | `workflows/visual_goal_navigation/workflow.py` |
| `sample_surface_depth` | `(depth, pixel, bbox, rgb_intr, depth_intr, minimum_m) -> SurfaceDepthSample` | `DepthGeometry` |
| `DepthForegroundFilter.filter` | `(obs, bbox, depth, minimum_m, samples, pixels) -> (samples, DepthForegroundResult)` | `DepthGeometry` |
| `require_execution_anchor` | `(measured, expected, pos_tol, yaw_tol) -> None` | `runtime/window/queue.py` |
| `TrajectoryValidator.validate_relative/.validate_world` | `(RelativeTrajectory) -> None` / `(WorldTrajectory) -> None` | `runtime/window/inflight.py`、`workflows/*` |
| `quintic`、`duration_for`、`plan_segment` | 见 §1.6 | `tests/test_motion.py`；迁移后无生产调用方（§8 第 7 条） |
| `duration_for_poses`、`sample_segment` | 见 §1.6 | `runtime/motion/loop.py`（段执行采样，裁决 R1） |
| `bounded`、`limit_vector`、`tracking_velocity`、`slew_limited`、`command_bandwidth`、`smoothed_step` | 见 §1.6 | `runtime/motion/limits.py::CommandLimiter`、`runtime/motion/loop.py`、`runtime/position_hold.py` |
| `planned_speed`、`braking_speed`、`corner_speed`、`corner_entry_speed`、`damped_approach_speed`、`angular_duration` | 见 §1.6 | `runtime/motion/follower.py`（`QueueFollower`，裁决 R4）、`runtime/position_hold.py` |
| `ContinuousPathReference` | `(settings, limits)`；`reset()`、`sample(...)` | `runtime/motion/follower.py` |
| `reached`、`corridor_default`、`passed_lateral_corridor` | 见 §1.6 | `runtime/motion/follower.py`（队列消耗判据）、`runtime/window/queue.py`（队列视图） |
| `pose_delta_measurement`、`is_settled`、`within_tolerance` | 见 §1.6 | `runtime/motion/loop.py`、`runtime/motion/follower.py` |
| `heading_rate_deg_s`、`tracked_heading_rate_deg_s` | 见 §1.6 | `runtime/motion/follower.py`、`runtime/motion/loop.py`、`runtime/position_hold.py` |
| `lookahead_distance`、`pure_pursuit_curvature`、`max_curvature`、`lateral_speed_limit`、`pursuit_solution`、`ackermann_command`、`holonomic_velocity` | 见 §1.6 | `runtime/motion/follower.py`（按 `MotionCapability.model` 选支） |
| `translation_handoff` | `(WorldPose, WorldPose, float, float, float) -> WorldPose \| None` | `runtime/position_hold.py` |
| `RefillScheduler` | `(pending_threshold, margin_s)`；`should_refill`、`record_latency` | `runtime/window/inflight.py`（现 `planner/capabilities/sliding_window/planning.py:42-44` 的持有者） |
| `BasicMotionConverter.create_target` | `(BasicMotionParameters, WorldPose) -> WorldPose` | `workflows/basic_motion/workflow.py` |
| `GeometricTargetPlanner.plan`、`GeometricPlannerSettings` | 签名不变 | `assembly/factories/planners.py` |
| `accept_position`、`bound_target_geometry`、`bound_target_goal`、`make_reference` | 签名不变 | `workflows/visual_goal_navigation/targeting.py` |

### 3.2 我要求相邻层提供什么

| 相邻层 | 要求 | 用在哪 |
| --- | --- | --- |
| `domain/` | `MotionLimits` 18 字段齐全，含 `max_speed_mps`、`position_gain`、`yaw_gain`、`passed_lateral_tolerance_m`、`position_tolerance_m`、`yaw_tolerance_deg`、`stopped_speed_mps`、`stopped_yaw_rate_deg_s`、`settle_time_s`、四个 heading 参数；`passed_lateral_tolerance_m` 保持 `float`（裁决 R17 改成 `max` 语义，不再需要区分「未配置」） | 全部运动律 |
| `domain/` | `MotionModel`、`MotionCapability`（`model`、`min_turn_radius_m`、`max_lateral_speed_mps`、`allows_reverse`）；`ACKERMANN` 的 `min_turn_radius_m` 必非 `None` | `passed_lateral_corridor`、`pursuit_solution` |
| `domain/` | `ContinuousTrackingSettings`（`lookahead_time_s`、`min/max_lookahead_m`、`max_jerk_mps3`、`max_yaw_jerk_deg_s3`）与 `ContinuousReference`（`position`、`distance_m`、`remaining_head_m`、`segment_length_m`、`progress_m`、`aligning`、`pure_rotation`） | `reference.py`、`speed.py`、`pursuit.py` |
| `domain/` | `QueuedWaypoint`（`waypoint_id`、`start`、`end`、`yaw_delta_deg`）是队列元素的唯一类型；`reached`/`planned_speed`/`corner_*` 只认这个类型，不认 list | `reached.py`、`speed.py`、`reference.py` |
| `domain/` | `WorldPose`、`RelativePoseDelta`、`RelativeTrajectory`、`WorldTrajectory`、`MotionSegment`、`MotionTrajectory`、`wrap_yaw_deg`、`wrap_deg`、`resolve_yaw_delta_deg`、`relative_to_world`（标量版） | 全部 |
| `domain/` | `HolonomicCommand`、`CarCommand`（`forward_speed_mps` 有符号且与 `reverse` 一致）、`MotionCommand` | `ackermann_command`、`pursuit_solution` 的返回值 |
| `domain/` | `Observation` 必带 `depth_kind`、`depth_intrinsics`、`rotation_camera_to_world`、`camera_position_world`、`pose`、`intrinsics`；`CameraIntrinsics.horizontal_fov_deg` | `depth/*`、`free_space.py` |
| `domain/` | `GoalObservation`（`bbox`、`surface_point`、`surface_points`）、`SurfacePointEstimate`、`GoalGeometry`、`DepthForegroundSettings`、`DepthForegroundResult`、`DepthPointLayer`、`TargetPositionState`、`GoalReference`、`INVALID_DEPTH_SENTINEL_M` | `depth/*`、`free_space.py`、`association.py` |
| `domain/` | `BasicMotionParameters`（`forward_m`、`up_m`、`yaw_right_deg`） | `basic_motion_conversion.py` |
| `planner/errors.py` | `ProtocolError`、`ExecutionError` 留在顶层模块，在本层的 import 白名单内 | `trajectory_validation.py`、`heading.py`、`limits.py` |
| `runtime/` | 按 owner 线程逐个持有状态对象；切载具或重规划时对 `ContinuousPathReference` 调 `reset()`、对限幅器调 `reset(velocity_world)` 而不是新建；`dt` 在 `[0.001, 0.25]` 内传入 `smoothed_step` | `ContinuousPathReference`、`smoothed_step` |
| `runtime/` | 传连续偏航（不折返），不传 `WorldPose.yaw_deg`；`goal_yaw` 同 | `reached`、`heading_rate_deg_s`、`pursuit_solution` |
| `runtime/` | `items` 以 `QueuedWaypoint` 的 tuple 传入，队首在前；不得在调用期间修改队列。队列视图与 `consume` 由 `runtime/window/` 提供，`StreamGoalSource` 由 `runtime/motion/follower.py::QueueFollower` 实现，`PlanningWindow` 不做伺服（裁决 R4） | `planned_speed`、`ContinuousPathReference.sample` |
| `workflows/` | 提供巡航速度（`speed_mps`）与 `MotionCapability`；不自行算曲率、不自行钳偏航 | `pursuit_solution` |
| `assembly/` | 由 `config.motion`、`config.motion.continuous`、`config.motion.collision` 各构造一次，传给运行时；不在本层读配置 | 全部 |
| `adapters/` | `MotionCapability.min_turn_radius_m` 由转向标定反算（`RoverDriveModel`），`Observation` 的深度字段在停稳帧上齐备 | `pursuit_solution`、`depth/*` |

本层对 `ports/` 零依赖：没有端口调用，也没有端口类型标注。

### 3.3 与 `00-boundaries.md` 第四节的对应

| 契约名 | 本层用法 |
| --- | --- |
| `MotionModel` | `passed_lateral_corridor` 的唯一分叉依据；`pursuit_solution` 的模型前置条件 |
| `MotionCapability` | `min_turn_radius_m` → 曲率上限；`allows_reverse` → 是否允许 `reverse=True`（校验在适配层） |
| `HolonomicCommand` | `holonomic_velocity` 的期望速度经执行层限幅后组装 |
| `CarCommand` | `ackermann_command` 的返回值 |
| `MotionCommand` | 不构造联合类型，只产出两支之一 |
| `CollisionState` / `CollisionContact` / `CollisionRecoverySettings` | 本层不读；退避速度由执行层调用 `ackermann_command(reverse=True)` 或 `holonomic_velocity` 生成。碰撞检查挂在 `ControlLoop._guard`，由 `CollisionRecovery.poll` 触发，`ExecutionService.checkpoint()` 只管停止与预算（裁决 R5） |
| `MotionLimits` | 全部运动律的包络来源，只此一份 |

### 3.4 纯跟踪的曲率与参数来源

曲率解算（`pursuit_solution`）：

```text
(f, l) = horizontal_body_offset(yaw_deg, x_ref - x, y_ref - y)      车体系前向/横向
d      = hypot(f, l)
alpha  = atan2(l, f)
kappa  = 2*sin(alpha)/d = 2*l/d^2                                   |d| > 1e-6, 否则 0
kappa  = clamp(kappa, -1/R_min, +1/R_min)                            R_min 来自 MotionCapability
v_lat  = sqrt(max_acceleration_mps2/|kappa|)                         横向加速度上限
v      = min(speed, v_lat)                                            speed 来自 planned_speed
omega  = clamp(degrees(v*kappa), -max_yaw_rate_deg_s, +max_yaw_rate_deg_s)
```

参数来源：

| 参数 | 来源 | 现值 |
| --- | --- | --- |
| 前视距离 `d` | `ContinuousTrackingSettings.lookahead_time_s`、`min_lookahead_m`、`max_lookahead_m`，由 `ContinuousPathReference.sample` 沿弧长取得 | 0.6 s、0.3 m、1.5 m（`config/core/default/base.yaml:47-49`）；`vlm` 档 `0.3/0.3/1.5`（`config/core/vlm/base.yaml:49-53`） |
| `R_min` | `MotionCapability.min_turn_radius_m`，由 `RoverDriveModel` 的三个标定常数反算 | `180/(yaw_rate_per_steer_mps·π·max_steering) = 180/(17.9·π·1.0) ≈ 3.20 m`（`planner/simulators/projectairsim/drive.py:40-41`） |
| 曲率上限 | `1/R_min` | ≈0.3124 1/m |
| 横向加速度上限 | 合并后的 `MotionLimits.max_acceleration_mps2`（与 `corner_speed` 的向心上限同一个量，现实现已在用） | 1.5 m/s² → 满舵时 `v ≤ 2.19 m/s` |
| 偏航角速度上限 | `MotionLimits.max_yaw_rate_deg_s` | 30 dps |
| 冻结转向速度 | `RoverDriveModel.min_steering_speed_mps`（引擎侧标定，不进契约） | 0.5 m/s |

`R_min` 从标定反算的理由：转向角 1.0 时 `yaw_rate = 17.9·speed` dps，即 `ω/v = 17.9°/s` 每 m/s = 0.3124 rad/m，曲率上限即 0.3124 1/m。改标定（`speed_per_engine_mps`、`yaw_rate_per_steer_mps`）时 `R_min` 跟着变，`min_turn_radius_m` 由适配层在构造期算一次。低于 0.5 m/s 时引擎把转向比按 0.5 m/s 冻结（`drive.py:83-85`），曲率指令在该速度以下只会按冻结比实现，速度不会因此增加。

### 3.5 输入面与输出面（裁决 R19）

本层对外的唯一入口是 `planner/algorithms/__init__.py`，`__all__` 与下面两张表一一对应；表中名字就是跨层可见的全部。表里的深层文件路径只表示实现位置，跨层 import 一律写 `from planner.algorithms import <名字>`（裁决 R19a）。

输入面：全部是契约层类型，本层没有要求任何其他层提供类型或具体对象（裁决 R19b）。

| 名字 | 形状 | 由谁提供 | 是否可选 |
| --- | --- | --- | --- |
| `MotionLimits`、`ContinuousTrackingSettings`、`ContinuousReference` | frozen dataclass | `domain.motion` | 必需 |
| `MotionModel`、`MotionCapability` | enum、frozen dataclass | `domain.motion` | 必需 |
| `QueuedWaypoint` | frozen dataclass（`waypoint_id`、`start`、`end`、`yaw_delta_deg`） | `domain.sliding_window` | 必需 |
| `WorldPose`、`RelativePoseDelta`、`RelativeTrajectory`、`WorldTrajectory`、`MotionSegment`、`MotionTrajectory` | frozen dataclass | `domain.pose`、`domain.trajectory` | 必需 |
| `HolonomicCommand`、`CarCommand`、`MotionCommand` | frozen dataclass、类型别名 | `domain.motion` | 必需（`ackermann_command` 的返回值） |
| `Observation`、`CameraIntrinsics` | frozen dataclass（含 numpy 数组字段） | `domain.observation` | 深度组必需 |
| `GoalObservation`、`GoalGeometry`、`SurfacePointEstimate` | frozen dataclass | `domain.visual_goal` | 深度组必需 |
| `DepthForegroundSettings`、`DepthForegroundResult`、`DepthPointLayer` | frozen dataclass | `domain.depth_foreground` | 可选（`DepthGeometry(foreground_filter=None)` 时不需） |
| `TargetPositionState`、`GoalReference` | frozen dataclass | `domain.target_state`、`domain.visual_goal` | 仅 `association.py` 需要 |
| `BasicMotionParameters` | frozen dataclass | `domain.basic_motion` | 仅 `basic_motion_conversion.py` 需要 |
| `INVALID_DEPTH_SENTINEL_M` | `float` 常量 | `domain.depth` | 必需（`free_space.py`） |
| `ProtocolError`、`ExecutionError` | 异常类 | `planner/errors.py`（顶层模块，不属任何层） | 必需 |

输出面：返回值只有数据类型（数字、tuple、契约类型）与本层的纯函数或纯策略对象；当消费方收的是本层的有状态对象时，消费方在自己的模块里声明 Protocol，构造参数与返回类型写那个 Protocol，本层只保证结构上满足（裁决 R19b）。

| 名字 | 形状 | 由谁消费 | 稳定性 |
| --- | --- | --- | --- |
| `horizontal_distance`、`nearest_surface`、`point_in_horizontal_body`、`horizontal_body_offset` | 纯函数，入出均为契约类型或 tuple | `workflows/`、`adapters/trajectory_planner/` | 稳定 |
| `world_pose_delta_in_reference_frame`、`relative_to_world`、`world_to_relative` | 纯函数 | `runtime/window/`、`workflows/`、`adapters/trajectory_planner/` | 稳定 |
| `FreeSpaceProfileSettings`、`FreeSpaceSector`、`FreeSpaceProfile`、`measure_free_space` | frozen dataclass、纯函数 | `adapters/trajectory_planner/prompts/` | 稳定 |
| `sector_centers`、`measure_sector` | 纯函数 | 仅 `tests/test_free_space.py` | 内部（不进 `__all__`） |
| `DepthGeometry`、`DepthForegroundFilter`、`SurfaceDepthSample`、`sample_surface_depth` | 纯计算类（无可变状态）、frozen dataclass、纯函数 | `workflows/`、`assembly/` | 稳定 |
| `TrajectoryValidator`、`require_execution_anchor` | 不可变配置对象、纯函数 | `runtime/window/`、`workflows/` | 稳定 |
| `BasicMotionConverter`、`GeometricTargetPlanner`、`GeometricPlannerSettings` | 纯计算类（无可变状态） | `workflows/basic_motion/`、`assembly/factories/` | 稳定 |
| `accept_position`、`bound_target_geometry`、`bound_target_goal`、`make_reference` | 纯函数 | `workflows/visual_goal_navigation/` | 稳定（细则不在四组） |
| `quintic`、`duration_for`、`SegmentSetpoint` | 纯函数、frozen dataclass | `tests/test_motion.py` | 内部 |
| `plan_segment` | 纯函数 | 无生产调用方（§8 第 7 条） | 内部 |
| `duration_for_poses`、`sample_segment`、`MotionSample`、`SynchronizedMotionPlanner`、`MotionPlanner` | 纯函数、frozen dataclass、无状态类、`Protocol` | `runtime/motion/loop.py`、`assembly/`、`workflows/` | 稳定；`MotionPlanner` 与 R19b 冲突，见 §8 第 4 条 |
| `bounded`、`limit_vector`、`tracking_velocity`、`slew_limited`、`command_bandwidth`、`smoothed_step` | 纯函数，返回 tuple | `runtime/motion/limits.py`、`runtime/motion/loop.py`、`runtime/position_hold.py` | 稳定 |
| `stopping_distance`、`braking_speed`、`angular_duration`、`corner_speed`、`corner_entry_speed`、`damped_approach_speed`、`planned_speed` | 纯函数，返 `float` | `runtime/motion/follower.py`、`runtime/position_hold.py` | 稳定 |
| `ContinuousPathReference` | 状态对象（`reset()` + `sample()`） | `runtime/motion/follower.py` | 稳定（消费方按自己 Protocol 接） |
| `xyz`、`segment_length`、`is_rotation`、`distance_to_segment` | 纯函数 | 本层内部 | 内部 |
| `reached`、`corridor_default`、`passed_lateral_corridor`、`ACKERMANN_PASSED_LATERAL_TOLERANCE_M` | 纯函数、`float` 常量 | `runtime/motion/follower.py`、`runtime/window/queue.py` | 稳定 |
| `pose_delta_measurement`、`is_settled`、`within_tolerance` | 纯函数，入出均为标量或契约类型 | `runtime/motion/loop.py`、`runtime/motion/follower.py` | 稳定 |
| `heading_rate_deg_s`、`tracked_heading_rate_deg_s` | 纯函数，返 `float` | `runtime/motion/`、`runtime/position_hold.py` | 稳定 |
| `lookahead_distance`、`pure_pursuit_curvature`、`max_curvature`、`lateral_speed_limit`、`pursuit_solution`、`PursuitSolution`、`ackermann_command`、`holonomic_velocity` | 纯函数、frozen dataclass，返契约命令或 tuple | `runtime/motion/follower.py` | 稳定 |
| `translation_handoff` | 纯函数，返 `WorldPose \| None` | `runtime/position_hold.py` | 稳定 |
| `RefillScheduler` | 状态对象（`should_refill` + `record_latency`） | `runtime/window/inflight.py` | 稳定（消费方按自己 Protocol 接） |

不暴露什么：

- 子模块路径。`algorithms/motion/speed.py`、`algorithms/motion/settle.py` 这类文件划分属实现细节，跨层看不到，也不承诺稳定（裁决 R19a）。
- 内部辅助函数与方法。`_layer`、`_sample_point`、`_sector_centers`（现名）等私有名，以及只给测试用的 `sector_centers`、`measure_sector`。
- 限幅器与队列的内部状态。`CommandLimiter` 在 runtime，本层只有 `smoothed_step`；`WorldWaypointQueue`、`PlanningWindow`、`QueueFollower` 都是消费方自己的对象，不列为本层输出（反例：把 `WorldWaypointQueue` 当输出）。
- 配置读取与默认值拼装。本层不收原始配置字典，`MotionLimits`、`ContinuousTrackingSettings`、`passed_lateral_tolerance_m` 都由装配层构造后传入。
- 诊断串格式。`SurfaceDepthSample.reason`、`GoalGeometry.reason` 的文案是留档内容，不是接口承诺。

## 四、错误与失败路径

| 触发 | 谁抛 | 异常类型与消息 | 谁捕获 | 恢复动作 |
| --- | --- | --- | --- | --- |
| `point_in_horizontal_body` 入参不是三个有限数 | 本层 | `ValueError("target point must contain three finite coordinates")` | 调用方任务层 | 目标当帧无效，下帧重取 |
| `nearest_surface` 收到空点集 | 本层（`min()`） | `ValueError` | `association.accept_position` 的上游 | 该帧不绑定目标 |
| `DepthGeometry.validate` 任一条校准不成立 | 本层 | `ValueError`（带具体字段） | `DepthGeometry.estimate` 捕获并折成 `GoalGeometry(False, reason)`；`measure_free_space` 同类校验不捕获 | 本帧不可靠，任务层重取观测 |
| 深度/图像形状与内参不符 | 本层 | `ValueError("image dimensions do not match calibrated intrinsics")` | `estimate` | 同上 |
| `sample_surface_depth` 越界/越框/深度无效 | 本层 | 不抛，返回 `valid=False` | — | 该点在证据里留原因串 |
| `TrajectoryValidator` 构造参数非法 | 本层 | `ValueError` | `assembly` 构造期（不捕获） | 服务启动失败，带字段名 |
| 轨迹点数或单步超限 | 本层 | `ProtocolError` | `runtime/window/inflight.py` 的 `poll`（现 `planning.py:109-111` 的 `except Exception`） | 记一次规划失败，队列不变，按 `stream_retry_at` 重试 |
| 世界航迹的显式偏航增量与端点矛盾 | `domain.pose.resolve_yaw_delta_deg` | `ValueError("signed yaw delta disagrees with endpoint yaw")` | `runtime/window/inflight.py` | 同上 |
| 执行锚点漂出容差 | 本层 | `ExecutionError("continuation anchor drifted outside position/yaw tolerance")` | `runtime/window/window.py` 的 `poll_stream`（现 `planner/services/planning_window.py:210-212` 的 `except (ExecutionError, ProtocolError)`） | 返回 `planning_discarded`，队列不变，1 s 后重试；该错误表示新点接不上已授权的队列尾 |
| 限幅器收到非有限向量或非法上限 | 本层 | `ExecutionError("non-finite motion vector or invalid limit")` | `runtime/motion/loop.py` 与 `runtime/motion/follower.py` → `runtime/execution.py` | 本拍不下发，停止 |
| 偏航合成值非有限 | 本层 | `ExecutionError("non-finite yaw control command")` | 同上 | 同上 |
| 偏航/误差入参非有限 | 本层 | `ValueError`（`_require_finite`） | 调用方 | 属实现缺陷，不恢复 |
| `sample_segment` 的段时长缺失或非正 | 本层 | `ValueError("smooth segment requires a positive finite duration")` | `runtime/motion/loop.py` → `runtime/execution.py` 的兜底（裁决 R1：引擎侧没有 tracker） | 包成 `ExecutionError`，任务层 `StageResult(False)` |
| `stopping_distance`/`translation_handoff`/`max_curvature`/`pursuit_solution` 的上限非法或缺半径 | 本层 | `ValueError` | 装配期或第一拍 | 构造期失败（契约层已要求阿克曼必有半径）；运行期只可能是配置错 |
| `pose_delta_measurement` 的 `dt <= 0` | 本层 | `ValueError("dt must be positive")` | `runtime/motion/loop.py` 或 `runtime/motion/follower.py` | 属实现缺陷：`dt` 由执行层夹取后传入 |
| `record_latency` 收到负值或非有限值 | 本层 | `ValueError` | `runtime/window/inflight.py` 的 `poll` 兜底 `except Exception` | 记一次规划失败；`latency_s` 状态不变 |
| `FreeSpaceProfileSettings` 参数非法 | 本层 | `ValueError` | `assembly` 构造期 | 服务启动失败 |
| `measure_free_space` 形状或 `depth_kind` 不符 | 本层 | `ValueError` | `adapters/trajectory_planner/prompts/*` 的调用方 | 该次规划请求失败，任务层计入调用预算 |
| `numpy` 内部异常（`LinAlgError` 等） | 本层不捕获 | 原样上抛 | 任务层 | 阶段失败，异常进 `StageResult.error` |

三条规则：

1. 只有 `DepthGeometry.estimate` 捕获异常，捕获集合是 `(ValueError, TypeError, IndexError)`，与现状一致；扩到 `OverflowError` 会让非有限坐标静默降级成「不可靠」，见 §8 第 6 条。
2. 契约违约用 `ProtocolError`（模型输出不该这样），执行违约用 `ExecutionError`（已授权的东西做不到），参数非法用 `ValueError`（构造期就该发现）。三类不互相替代。
3. 本层不吞异常后返回默认值。唯一例外是 `sample_surface_depth` 与 `DepthForegroundFilter` 的逐点证据，它们的返回值本身就是「无效 + 原因」的形状。

## 五、并发与所有权

- 线程：`algorithms/` 不创建线程，不持锁，不读时钟，不 sleep。模块级只有不可变常量（`ACKERMANN_PASSED_LATERAL_TOLERANCE_M`、`_QUINTIC_PEAK_*`、`INVALID_DEPTH_SENTINEL_M` 的引用）。
- 无状态函数可以被任意线程并发调用：几何、变换、校验、五次多项式、速度包络、偏航律、纯跟踪全部属于这一类。
- 跨拍状态只出现在三处，都只允许单一所有者线程访问：
  - 限幅器状态（`runtime/motion/limits.py::CommandLimiter` 的两个向量）：由 `runtime/motion/loop.py` 与 `runtime/position_hold.py` 各持实例，owner 线程独占；本层只给无状态的 `smoothed_step`。
  - `ContinuousPathReference`：由 `runtime/motion/follower.py::QueueFollower` 持有，owner 线程独占。
  - `RefillScheduler`：由 `runtime/window/inflight.py` 持有（现 `planner/capabilities/sliding_window/planning.py:45` 的 owner 校验），`should_refill`、`record_latency` 都在 owner 线程（现 `planning.py:86` 的 `poll` 先 `check_owner`）。
- `TrajectoryValidator` 是不可变配置加纯方法，可跨线程共享；现调用点仍全在 owner 线程（`planning.py:101-103`）。
- 切换载具：不重建算法对象，`MotionLimits`、`ContinuousTrackingSettings`、`MotionCapability` 随载具 `VehicleArm` 一起换引用；执行层在换引用后对限幅器调 `reset(velocity_world)`、对 `ContinuousPathReference` 调 `reset()`。理由：同一族载具的参数可能不同（阿克曼的走廊 1.0、全向 0.3，裁决 R17），状态必须按新参数从头积分，不能把旧速度带过去。
- 暂停/恢复：`pause()` 停稳后 `reset(measured_velocity)`；`resume()` 再次 `reset()` 并重建 `goal_yaw`（现 `planner/services/continuous_execution.py:159`、`:170` 已是这个顺序）。`ContinuousPathReference` 在 `reset()` 后 `progress` 归零，队首未变时下一次 `sample` 重新投影，不会沿用暂停前的进度。
- 顺序约束：`planned_speed` 与 `heading_rate_deg_s` 的 `current_acceleration` 都来自限幅器的上一拍状态，同一拍里必须先读状态、后 `smoothed_step`，否则用的是本拍中间值。这个顺序由执行层的调用次序保证，本层不记忆。

## 六、测试点

### 6.1 单测清单

| 文件 | 断言对象 | 负例 |
| --- | --- | --- |
| `tests/test_algorithms_geometry.py`（新） | `horizontal_distance` 忽略 z；`nearest_surface` 平局取首个；`point_in_horizontal_body` 与 `horizontal_body_offset` 逐位一致；`relative_to_world`/`world_to_relative` 往返恒等（10 组随机位姿） | 非有限坐标抛 `ValueError`；空点集抛 `ValueError`；`yaw_deltas_deg` 长度不符在 `WorldTrajectory` 构造期抛 |
| `tests/test_depth_geometry.py`（现有 12 例，改 import 路径） | 反投影、ray/像素往返、最近样本、`ray_range` 换算 | `verified=False`、`depth_kind` 不符、旋转非正交抛 `ValueError`；背后点 `project_camera` 返回 `None` |
| `tests/test_capability_targeting.py`（现有，改 import 路径） | 单像素直读、过滤禁用与无分离、首帧绑定 | 越界像素、越框像素、深度哨兵值、`minimum_m` 边界 |
| `tests/test_free_space.py`（现有 10 例 + 新增 4 例） | 平地/墙/护栏/斜坡/`ray_range` 等价 | 全无效深度返回未知而非畅通；`sector_centers` 视场 95°/4° 两种取整；`measure_sector` 空数组；设置参数负例 |
| `tests/test_batch3_services.py`（现有，拆出调度部分到 `tests/test_algorithms_scheduler.py`） | `should_refill` 四种组合；`record_latency` 的 EMA 系数 | `count=0`/`count=6` 不补；负延迟抛 `ValueError`；`latency_s` 状态在异常后不变 |
| `tests/test_motion.py`（现有 8 例，改 import） | `quintic` 端点与单调；`duration_for` 四项约束；`plan_segment` 端点/偏航回绕/步数 | `speed<=0` 抛；`min_duration_s=0` 合法 |
| `tests/test_heading_servo.py`（现有 11 例，不动断言） | 比例项斜率、刹车曲线压低大误差、阻尼符号、死区、钳位、`tracked_*` 的斜率限制 | `nan`/`inf`/`True` 抛 `ValueError`；`inf` 前馈抛 `ExecutionError` |
| `tests/test_passed_corridor.py`（现有 5 例，改默认值断言） | `corridor_default` 按模型取；阿克曼 1.0（裁决 R17）、全向 `position_tolerance_m`；配置只能抬高 | 偏航未对齐时宽走廊不判过；零长段只按距离；旧 0.6 断言改 1.0，配置 0.2 仍得 1.0 |
| `tests/test_algorithms_limits.py`（新） | `limit_vector` 缩放与零向量；`tracking_velocity` 先加后限；`slew_limited` 差值范数；`command_bandwidth` 上限 4；`smoothed_step` 首拍增量与 `dt` 稳定性 | 非有限向量抛 `ExecutionError`；`maximum<=0` 抛；`dt=0` 抛；长度不等抛 `ValueError` |
| `tests/test_algorithms_settle.py`（新） | `pose_delta_measurement` 三返回值；`is_settled` 五条件逐条；`within_tolerance` 边界 | `dt=0` 抛 `ValueError`；`elapsed<duration` 时即便到位也为假 |
| `tests/test_algorithms_pursuit.py`（新） | 直线 `κ=0`；`κ=2l/d²` 与 `2sinα/d` 在 100 组随机点相等；曲率钳到 `1/3.2`；`v_lat` 与满舵 2.19 m/s；`ackermann_command` 的符号 | 非阿克曼能力抛 `ValueError`；`min_turn_radius_m=None` 抛；`d=0` 返回 0；`forward_speed<0` 抛 |
| `tests/test_motion_equivalence.py`（新） | 同一输入、同一状态序列下，新纯函数组合与迁移期旧实现快照（`ContinuousFollower.sample` 与两份 tracker 的当前代码）输出逐值一致（`velocity`、`yaw_rate` 位相同）；裁决 R1 删掉旧文件后，测试对照仓库内保留的快照常量 | 旧实现的边界输入（零长段、纯旋转、掉头 180°、队列只剩一点）逐条对照 |

### 6.2 断言对象与数值

- 角度比较统一 `pytest.approx(..., abs=1e-9)`；长度比较 `abs=1e-9`；曲率比较 `rel=1e-6`。
- `2.19 m/s` 由 `sqrt(1.5×3.2009)` 得到，断言写 `pytest.approx(2.190, abs=0.01)`，并注明它随 `max_acceleration_mps2` 与 `min_turn_radius_m` 变化。
- `0.3124 1/m` 写成由标定常数算出的表达式 `1/(180/(17.9*math.pi*1.0))`，改标定时断言跟着变（与 `docs/refactor/06-adapters-engines.md:287` 的口径一致）。

### 6.3 分层测试

`tests/test_layering.py` 增加五条针对本层与 R19 的检查：

1. `planner/algorithms/**` 不 import `planner.runtime`、`planner.workflows`、`planner.assembly`、`planner.adapters`、`planner.simulator`、`planner.simulators`、`planner.ports`。
2. `planner/algorithms/**` 不 import `time`、`threading`、`asyncio`、`socket`、`subprocess`；AST 扫描 import 节点。
3. `planner/algorithms/**` 只有 `depth/` 与 `free_space.py` 可以 import `numpy`（其余模块保持纯 Python，便于在无 numpy 环境跑单测）。
4. 跨层 import 只指向包入口（裁决 R19a）：`planner/algorithms/` 之外的包不得出现 `from planner.algorithms.<子模块>` 或 `import planner.algorithms.<子模块>`；反向同理，本层不得写 `planner.runtime.motion...` 一类深层路径。同层内部深层 import 不算违反。
5. 每个层包的 `__init__.py` 有显式 `__all__`，且 `__all__` 非空、无下划线名、每个名字都能 `getattr` 到；`planner/algorithms/__init__.py` 的 `__all__` 与 §3.5 输出表中标为「稳定」的名字集合相等（裁决 R19a）。构造参数与返回注解不得出现其他层的具体类（裁决 R19b）：扫描 `planner/runtime/` 与 `planner/workflows/` 的函数签名注解，只允许 `planner.domain.*`、`planner.ports.*`、本层 Protocol、内置类型与本层名字，出现 `planner.algorithms.<类名>` 即失败。

第三条是可执行的形式化约束：`motion/*`、`trajectory_*`、`target_coordinates`、`trajectory_validation`、`sliding_window/scheduler` 全部只用标准库。第五条里「本层名字」指消费层自己声明的 Protocol：runtime 接收 `TrajectoryValidator`、`RefillScheduler`、`ContinuousPathReference` 时用的那几个 Protocol 由 `03-runtime.md` 命名；`MotionPlanner` 的归属见 §8 第 4 条（与 R19b 冲突，待裁）。

## 七、迁移步骤

| 步 | 做什么 | 验收 |
| --- | --- | --- |
| 1 | 建 `algorithms/` 目录，原样搬运 `target_coordinates`、`trajectory_transform`（新增 `horizontal_body_offset` 并让三处调用它）、`free_space`（删 `_measured`、公开 `sector_centers`/`measure_sector`）、`trajectory_validation`、`depth/*`、`sliding_window/scheduler`、`planning/geometric_planner`、`targeting/association`、`basic_motion_conversion`；同批改 import（含 `capabilities` → `algorithms` 的机械替换） | `pytest tests/test_depth_geometry.py tests/test_free_space.py tests/test_capability_targeting.py tests/test_motion.py tests/test_batch3_services.py tests/test_workflow.py tests/test_geometric_planner.py tests/test_trajectory_prompts.py -q` 全绿；`tests/test_architecture.py` 全绿 |
| 2 | 合并 `MotionLimits`（依赖 `docs/refactor/01-contracts.md` 的契约落地），把 `segment_motion.py` 剩余四个符号搬到 `motion/quintic.py`，删 `to_tracking_limits` 与 `services/__init__.py` 的再导出 | `pytest tests/test_motion.py tests/test_passed_corridor.py tests/test_projectairsim_motion.py -q`；`grep -rn "segment_motion" planner --include=*.py` 只剩 `motion/quintic.py` 的历史注释 |
| 3 | 搬 `motion/{synchronized,speed,reference,limits,heading,handoff,base}.py`；`limits.py` 只取 `command_smoothing.py` 的算术，状态壳按 `03-runtime.md:30` 落 `runtime/motion/limits.py::CommandLimiter`；两份 tracker 的 `limit_vector` 合并进 `motion/limits.py`；`synchronized.duration_for` 改名 `duration_for_poses` 并改两处调用（裁决 R1：这两处调用随采样循环一起进 `runtime/motion/loop.py`） | `pytest tests/test_motion.py tests/test_heading_servo.py tests/test_projectairsim_motion.py tests/test_position_hold.py tests/test_passed_corridor.py -q`；UE5 数值动作实机一次 |
| 4 | 新建 `motion/corridor.py`：搬常量、加 `corridor_default`/`passed_lateral_corridor`（裁决 R6）；`motion/reached.py` 只留 `reached`；删 `simulators/projectairsim/rover.py` 的 `ROVER_PASSED_LATERAL_TOLERANCE_M` 与 `rover_tracking_limits`，`WorldWaypointQueue` 与 `QueueFollower` 改从新位置取走廊 | `pytest tests/test_passed_corridor.py -q`（断言按裁决 R6 语义改写）；`grep -rn "ROVER_PASSED_LATERAL_TOLERANCE_M" planner --include=*.py` 只在 `algorithms/` |
| 5 | 新建 `motion/settle.py`；`pose_delta_measurement`、`is_settled`、`within_tolerance`、`tracking_velocity`、`slew_limited` 接进 `runtime/motion/loop.py` 的段循环（裁决 R1：不再接 tracker） | `pytest tests/test_algorithms_settle.py tests/test_algorithms_limits.py tests/test_projectairsim_motion.py tests/test_heading_servo.py tests/test_position_hold.py -q`；`tests/test_motion_equivalence.py` 逐值一致；UE5 与 UE4 各跑一次数值动作 |
| 6 | 新建 `motion/pursuit.py`，不接线；`holonomic_velocity` 抽自 `continuous.py:92` 并让 `ContinuousFollower` 改用它 | `pytest tests/test_algorithms_pursuit.py tests/test_motion_equivalence.py -q`；`grep -rn "import time\|threading" planner/algorithms/motion/pursuit.py` 为空 |
| 7 | 接线阿克曼律：`runtime/motion/follower.py::QueueFollower` 按 `MotionCapability.model` 选 `pursuit_solution` 或 `holonomic_velocity`（依赖 `docs/refactor/03-runtime.md` 的 `ControlLoop` 与 `QueueFollower` 落地，本层只交函数；裁决 R1/R4） | UE5 车跑一次目标接近，`/api/trajectory` 与日志中不出现 `continuous waypoint timed out`，`reverse` 命令计数为 0 |
| 8 | 清理：删 `services/motion/continuous.py`、`simulators/projectairsim/motion.py`、`simulators/airsim/motion_tracker.py`（裁决 R1：tracker 随采样循环一起删）、`services/segment_motion.py`、`capabilities/targeting/{depth_geometry,depth_foreground,depth_surface_sampling}.py`、`capabilities/sliding_window/scheduler.py`、`capabilities/basic_motion_conversion.py`、空目录 `capabilities/viewing` | `pytest tests/ -q` 全绿且用例数不减少（`tests/test_projectairsim_motion.py` 的断言迁到 runtime 侧循环测试）；`python -c "import planner.algorithms, planner.algorithms.motion.pursuit"` 成功；`ruff check planner/algorithms` 通过 |

第 1、2 步不改任何算式，第 3–5 步只搬位置与合重复，第 6、7 步引入新律。每步可单独提交，回退只回退一步。

## 八、未决问题

**1.（裁决 R6、R17 已关闭，不再是未决项）越点横向走廊**
裁决内容：常量放 `algorithms/motion/corridor.py`；`ACKERMANN` 取 1.0 m（裁决 R17 从 0.6 抬到 1.0），`HOLONOMIC` 取 `MotionLimits.position_tolerance_m`；配置 `motion.passed_lateral_tolerance_m` 只能抬高不能压低（取 `max`）；不从 `min_turn_radius_m` 推导；引擎侧的 `ROVER_PASSED_LATERAL_TOLERANCE_M` 与 `rover_tracking_limits()` 删除。实现细节见 §1.6；`max` 语义下 `MotionLimits.passed_lateral_tolerance_m` 保持 `float`，不再需要 `None`。

**2. 扇区网格的视场来源**
`_sector_centers` 用 rgb 内参的 `horizontal_fov_deg`（`planner/services/free_space.py:219`），而方位角由深度内参的 `fx/cx` 算出（`:155-156`）。两套内参分辨率不同时（`planner/domain/observation.py:94-99` 记录过 AirSim 的 rgb 与 depth 不是同一网格），扇区网格与实测方位可能错开。
- 默认：保持现状（rgb 视场），迁移期不改行为；加一条单测钉住当前网格，并在 `free_space.py` 的模块注释里写明「扇区网格取 rgb 视场、方位角取 depth 内参」这个事实。代价：AirSim 上的扇面标签可能与实际方位偏几个度。
- 选项 B：改用 `observation.depth_intrinsics.horizontal_fov_deg`。代价：两套内参的视场角本应一致，改了会让「扇区数与视野」随深度分辨率变化。
- 选项 C：由深度像素几何反算 `2·atan(width/(2·fx))`，不依赖任何 `horizontal_fov_deg` 字段。代价：与其余代码的 FOV 来源不一致，且要确认两种引擎的 `fx` 都可信。

**3. 巡航速度上限取哪一处**
`MotionLimits.max_speed_mps`（`config` 的 `motion.max_speed_mps`，默认档 5.0）、`MotionCapability.max_lateral_speed_mps`（无人机取 `drone.speed_mps`，默认 2.0）、任务配置里的 `speed_mps` 三个数都存在。
- 默认：任务层给 `speed_mps`，`MotionLimits.max_speed_mps` 是绝对上界，`MotionCapability` 只用于校验命令合法性（横向分量不超过它）。本层只做 `min`。
- 选项 B：一律以 `MotionCapability.max_lateral_speed_mps` 为上限。代价：车的该值为 0，语义直接崩掉，需要再加「前向速度上限」字段。
- 选项 C：删 `motion.max_speed_mps`，只留载具能力。代价：配置里失去一个与物理包络无关的调试上限，段规划时长会跟着载具变。

**4. `MotionPlanner` 声明的归属**（本条默认与裁决 R19b 冲突，需裁：裁决 R19b 只允许把「本层自己声明的 Protocol」当输入，因此工作流用本层的 `MotionPlanner` 作构造参数不符；下面三个选项按 R19b 重新排序时，选项 B 与选项 C 合规，选项 A 不再合规）
`planner/services/motion/base.py:6` 只有一个 `Protocol`，实现只有 `SynchronizedMotionPlanner` 一份，使用方是 `workflows/visual_goal_navigation/workflow.py:61,79` 的类型标注。
- 默认：留在 `algorithms/motion/base.py`，当作内部策略声明，不当作端口。代价：与「`Protocol` 声明集中在 `ports/`」的风格约定不一致。
- 选项 B：搬到 `ports/motion_planner.py`。代价：需要 `01-contracts.md` 加一个端口文件，而它没有对应的外部实现方（两个引擎都不实现它）。
- 选项 C：删掉，工作流直接标注 `SynchronizedMotionPlanner`。代价：工作流从此认识具体实现类，换规划器要改任务包。

**5. 阿克曼的偏航指令组成**
纯跟踪的曲率指令与现有偏航伺服解决的是同一件事的两端：前者消横向偏差，后者消航向偏差。
- 默认：二选一，按 `reference.aligning or remaining_head_m <= 1e-9` 切换。正常跟踪用 `pursuit_solution` 的角速度，终端对齐与纯旋转用 `heading_rate_deg_s`。与现有结构（`continuous.py:83-104`）同形，切换点只有一个。
- 选项 B：两者叠加（曲率前馈 + `heading_rate_deg_s` 修正）。代价：在大横向误差时两路的期望方向不一致，叠加会双计，需要重新调 `yaw_gain`。
- 选项 C：曲率前馈 + 横向误差 PI，去掉 `goal_yaw` 概念。代价：`goal_yaw` 是队列协议的一部分（`resolve_yaw_delta_deg`、`pause_stream` 的 `first_goal_yaw_deg` 都依赖它），删除面很大。

**6. `DepthGeometry.estimate` 的异常捕获集合**
现状只捕获 `(ValueError, TypeError, IndexError)`（`planner/capabilities/targeting/depth_geometry.py:91`）。`float('nan')` 不会触发其中任何一个，而 `numpy` 的某些形状不匹配会抛别的类型。
- 默认：保持三类，理由是非有限坐标已经在 `validate` 与 `point_in_horizontal_body` 里被拦。代价：`numpy` 抛 `ValueError` 之外的异常时整帧上抛，任务失败而不是降级。
- 选项 B：加 `OverflowError`。代价：真正的溢出会被静默改成「不可靠」，日志里看不出是坐标问题。
- 选项 C：改成 `except Exception`。代价：违反「契约违约要有明确类型」这条规则，调试时无法区分校准失败与编程错误。

**7. `plan_segment` 的去留**
现状没有生产调用方，只有 `planner/services/__init__.py:7,15` 的导出与 `tests/test_motion.py` 的 4 个用例。
- 默认：保留在 `motion/quintic.py`，签名不变。理由：它是五次多项式采样步数的唯一测试锚点，删掉会少一处对 `ceil(duration·control_hz)+1` 的约束。
- 选项 B：删除连同 4 个用例。代价：本层少约 35 行无调用方的代码，但失去对采样步数的断言时间线。
- 选项 C：保留并接进段执行（让引擎按 `plan_segment` 的采样下发）。代价：与 `00-boundaries.md` 第二节「引擎侧按 30 ms 采样曲线」冲突，且引入第二套采样率。

**8. 冻结转向速度是否进契约**
`RoverDriveModel.min_steering_speed_mps = 0.5`（`planner/simulators/projectairsim/drive.py:44`）意味着低于 0.5 m/s 时转向被按 0.5 m/s 的比例实现，曲率指令打了折扣。
- 默认：不进契约。引擎适配层已经处理（`drive.py:83-85` 冻结比例），本层只保证 `forward_speed_mps >= 0`，不引入最小前进速度。代价：纯跟踪在极低速微调阶段实际转向不足，收敛靠时间而不是精度。
- 选项 B：`MotionCapability` 加 `min_steering_speed_mps`，`pursuit_solution` 在 `|κ| > 0` 时把 `forward_speed_mps` 抬到该值。代价：契约层多一个标定量，停车对位时车会持续缓慢前进。
- 选项 C：`pursuit_solution` 在 `forward_speed` 低于常量 0.5 m/s 时把曲率按速度比缩减，让指令与实现一致。代价：常量进了本层，换引擎要改代码。
