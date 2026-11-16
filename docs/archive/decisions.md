# 被取代的设计与被推翻的结论

记录"当时怎么做、为什么改、现在是什么"。改结构前先看这里，避免把已经付过代价的方案再做一遍。

## 1 双仿真器支持的三次迭代

| 版本 | 做法 | 结局 |
|---|---|---|
| v1 | 给 UE4 加兼容分支，主流程按族名 if | 否决：分支散落在 HTTP、控制器、前端、提示词各处 |
| v2 | 声明式族注册表，能力位写 YAML，旧分支整树复制为 `planner/ue4` | 形状对了一半，能力位仍是配置；复制树 12044 行 |
| 现在 | 端口化：两个并列的仿真器包，能力由实现的端口推导 | 采用 |

v2 被推翻的两条 v1 结论：

- "生命周期以 main 为准" → 改为照 `airsim-flight-console`，那是调试过的程序。
- "UE5 保留 `--sim-only` 换图不断流" → 代价是失去它，见第 4 节。

## 2 复制树改成端口化

测量结果：

| 量 | 值 |
|---|---|
| main 的 `planner/` 总量 | 19393 行 / 178 文件 |
| 其中真正仿真器专有 | 4382 行，22% |
| 复制进来的量 | 12044 行，其中约 10600 行是纯重复 |
| 两边同名文件 | 137 个，70 个只差格式、导入写法与 docstring |
| 真正 import 客户端库的文件 | 11 个 |

结论：仿真器差异集中在 11 个文件，不在 206 个文件。删除复制树后 `git` 记为重命名，旧分支仍可按文件取回。

顺带修正了工作量口径：早期估"48 个共享文件要逐个读"，改成按"UE4 会失去什么"重测后是 7 个。

## 3 能力位从配置改到代码

原设计在 `config/simulators.yaml` 写 `capabilities:`，前端与后端读它。

问题：配置文件能对现实说谎。仿真器包加了端口而配置没改，控制台就多一个点不动的按钮；
反过来少了端口而配置写着有，接口返回成功但什么也没发生。

现在：`SimulatorPorts` 与 `TASK_REQUIREMENTS` / `ACTION_PORTS` 推导，写 `capabilities` 会被配置校验拒绝。
仿真器包自报 `PORT_SLOTS`，测试断言它等于 `build_ports` 真实填的槽。

## 4 进程驱动从三个版本收敛到一个

| 阶段 | 形态 |
|---|---|
| 最初 | 两个 shell 脚本 `launch_*_xpra.sh` / `stop_*_xpra.sh`，UE5 路径写死成 `$PROJECT_DIR/sim/` |
| 中间 | `driver: script` 与 `driver: xpra_start` 并存，两族各走一边 |
| 现在 | 只有 `xpra_start`：argv 全在 `config/simulators.yaml`，引擎是 `xpra start --start-child` 的子进程 |

删除脚本时一并删掉 1782 行。代价是失去 UE5 的"只重启引擎、保留 Xvfb 与 xpra"：

- 旧行为：换图画面不断流
- 现在：会话与 Xvfb 一起重启，iframe 断开一次再自动重连

前端的恢复路径仍在：worker 就绪后重建 xpra 帧，断流期间显示未连接。

`argv` / `stop_argv` 用 YAML 锚点写一份，两族引用同一列表，由测试断言两处相等。

## 5 装配从会话构造函数搬到装配层

原状：`ProjectAirSimMissionSession` 在构造函数里建立检测器、规划器、几何与遥测汇，再把它们交给工作流。
UE4 的子进程自己建 resources 时漏了 `trajectory`，工作流遇到 `None` 就静默不发，
控制台的剩余轨迹面板在 UE4 上直接消失，没有任何报错。

现在：

- 会话只报 `mission_inputs()`，收 `attach_arms(arms)`
- 装配在 `application/mission_assembly.py`，缺轨迹汇在构造期报 `ProtocolError`
- 子进程用 `simulator/telemetry_relay.py` 转发，0.4 秒合并

这就是"少接一根线在构造期报错"的来源。

## 6 剩余轨迹不再是仿真器能力

`action` 里曾有 `trajectory`，`SimulatorPorts` 里曾有 `telemetry` 槽，UE4 因此被判定不支持。

问题：那个槽里装的是服务自己的 `TrajectoryHub`，端点直接读 hub，除了门之外没有任何生产代码读这个端口。
门是空转的，UE4 的 `/api/trajectory` 却返回 501。

现在：删掉 `ACTION_PORTS["trajectory"]`、`telemetry` 槽、`TrajectoryTelemetry` 协议与前端门控。
两个家族都返回剩余航点。

## 7 基础动作解码器按载具分开

原状：一个解码器，无人机提示词明写 "cannot change altitude in this runtime"，所以升降被拒。

事实：底层一直支持。轨迹插值带 dz，UE5 用 `move_to_position_async`，UE4 用 `moveToPositionAsync(x, y, z)`。

现在：两族各一个解码器，解码结果仍是同一个领域对象。车的线上格式里没有竖直字段，带 `up_m` 的包在解码期即被拒。
合理性判断落在"模型输出变成我们的数据"这个边界上，不在送仿真器那一层事后发现。

## 8 机位的三版做法

| 版本 | 做法 | 问题 |
|---|---|---|
| 1 | 写死顺序表 + 相对步数 | 服务端循环锚点不跨进程稳定，四个按钮整体错位 |
| 2 | 运行时标定（截图与参考帧相关匹配） | 起图从 13.6 s 拖到 60 到 90 s |
| 现在 | 起点写进 `projectairsim.view_cycle_start`，按"起点 + 切换次数"记账 | 换包或换图后需要手动重测 |

标定代码没有删，留在 `streaming_views.py` 与 `scripts/calibrate_streaming_views.py`，作为手动确认工具。
它解出的整圈排列、影子帧稳定等待、不可取帧的排除法，仍然是这方面的最佳可用方案，俯视接线时可能还要用。

## 9 隐藏非活动载具的三次调整

| 做法 | 结果 |
|---|---|
| 水平传送到 300 m 外 | 无人机嵌进墙里，物理求解器每小时报约 700 次接触，4 小时 93970 条，自由导航把每次接触当碰撞 |
| 垂直抬高 1000 m，每次时钟推进后钉住 | 采用 |
| 标定前先悬停同点上方 15 m，标定后再隐藏 | 已回退：城市地图地面机位匹配从 0.99 掉到 0.39，标定失败 |

配套改动：两台载具出生点改为完全相同，场景 `pause-on-start: true`，标定前先把非活动载具抬走。

## 10 UE5 俯视：结论反转过，但决定没变

时间线：

1. 早期查 API 文档，结论是"UE5 没有相机位姿读写"，于是 UE5 不做俯视，UE4 保留。
2. 装好 `projectairsim` 后实测 `dir()`，
   发现 `Drone.set_camera_pose` / `reset_camera_pose` / `camera_look_at_object` 都存在，
   `World` 另有 `spawn_object` / `set_object_pose`。前提不成立，文档里因此写了"建议重开这条边界"。
3. 用户 2026-11-15 明确：先不做俯视，先把整体程序跑起来。`PORT_SLOTS` 保持现状。

所以现在的事实是：UE5 有 API，但能力位没有宣告俯视；UE4 的能力位宣告了俯视，但共享层还没接线，实际不可执行。
两族都跑不了俯视任务。

### 接线顺序（要做时按这个来）

1. `planner/domain/mission.py`：`TaskKind` 加 `VISUAL_GOAL_ABOVE`，
   `TaskParameters` 并上 `visual_goal_above` 的参数类型。
2. `planner/adapters/task_parser/tasks/` 加解码器并注册；规则文本加进提示词。
3. `planner/workflows/visual_goal_above/` 从旧分支取回（共享层，不放进仿真器目录），导入指向共享层。
4. `planner/application/workflow_router.py` 与 `mission_assembly.py` 注册工作流与阶段校验器。
5. `planner/simulators/projectairsim/runtime.py` 实现 `camera_view` 端口后加进 `PORT_SLOTS`。
6. 两侧都实机验证，因为第 1、2 步改的是共享层。

## 11 文档的三代

| 文档 | 状态 |
|---|---|
| `HANDOVER.md` | 写于 851 passed / 13 提交时，自述过时 |
| `HANDOVER-2026-11-15.md` | 换机前快照，被本套文档取代 |
| `README` + `DEPLOYMENT` + `ARCHITECTURE` + `AGENT` + 本文档目录 | 当前 |

期间历史被 `filter-branch` 重写过一次：删除误提交的 696 KB bundle。
旧哈希（`fed3373` 到 `0ebd534`）作为悬挂对象仍在库里，树与重写后逐条一致。文档里若出现旧哈希，指的就是同一批提交。

`PLAN.md` 记录过 M1 到 M8 与十五条补充。它被本文件取代，因为它按时间追加，读起来像流水账；
真正的结论已经按主题归到 `pitfalls.md`、`measurements.md` 与本文。

## 12 航向律从四份收成一份

早先按运动阶段各写一份：连续跟进带刹车曲线，UE5 与 UE4 的段式运动各一份数学（同一段代码抄两遍），
位置保持是前者的简化版。四份里没有两份服务于不同的控制目标，航向律只有一个。

现在只剩 `services/motion/heading_servo.py` 一份，载具差异在参数，仿真器差异在传输。四条参数结论是踩出来的：

| 结论 | 怎么来的 |
|---|---|
| 阻尼必须用实测角速度 | 用位姿差分的那一版摆动更明显，30 Hz 下差分角速度峰峰 42 °/s |
| 延迟超过 90° 相位时阻尼是助推 | UE5 加阻尼后更晃，UE4 变好：两侧执行链延迟差一个量级 |
| 保持环的增益必须单独一个 | 用飞行路径的 `yaw_gain` 取 2.0 时，在 3 s 延迟下必然自激 |
| 增益不进适配器 | 同机型两套引擎应该用同一套增益，不一致说明模型对不上，要记录而不是盖住 |

## 13 时钟从锁步改成自由跑

UE5 原来靠 planner 一步一步推 `ContinueForSimTime`，好处是每个控制周期恰好一拍仿真时间，
代价是画面刷新率等于推时钟的频率：模型请求、日志、编码期间世界就冻住，GPU 一忙就掉到 10 帧。

改成加载阶段仍暂停（挡加载瞬态），载具就位后交给引擎自己跑，`advance_for_sim_time` 变成等时钟。
`resume_clock()` 实测时钟是否真的自己走，不走就回落到步进，不靠配置开关。代价是失去逐周期可复现性，
控制节奏变成墙钟；这也意味着凡是以“世界冻着”为前提的判据（例如停稳观测、锚点校验）都是第一次真正生效，
它们自己的毛病会在这时暴露。

## 14 不要再重开的边界

| 边界 | 内容 |
|---|---|
| 命名 | 用软件名 `airsim` / `projectairsim`，不用引擎版本号；地图字段叫 `simulator` |
| 能力位 | 只能由端口推导，配置里写会被拒绝 |
| UE4 不提供 | 机位切换、外观模型、帧流、回起点；`free_navigation` 未实测前不提供 |
| UE5 进程 | 留在服务进程内，不搬进子进程 |
| 进程驱动 | 只有一个 `xpra_start`，命令在 YAML 里 |
| 桩 | 不写无人调用的桩与开关，未实现的路径显式报错 |
| 前端门控 | 按卡片 id 点名，不按 DOM 嵌套查找；拆卡不拆门 |
| 复制 | `planner/ue4` 不复活 |
| 兼容层 | 不为"某个引擎的怪癖"在共享层加 if；怪癖留在适配层，并写进该层文档字符串 |