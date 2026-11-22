> 上级：[项目架构](ARCHITECTURE.md) · [开发文档](DEVELOPMENT.md) · [部署文档](DEPLOYMENT.md)
> 层文档：[domain](layers/domain.md) · [ports](layers/ports.md) · [algorithms](layers/algorithms.md) · [runtime](layers/runtime.md) · [workflows](layers/workflows.md) · [assembly](layers/assembly.md) · [adapters](layers/adapters.md) · [simulator](layers/simulator.md) · [simulators](layers/simulators.md)

# 待办与已知缺口

规则：这里只放**跨层**或**需要一次结构改动**的事项。只在一层内的问题写在该层文档的「已知缺口」小节，不重复登记。一条超过二百字说明它该拆成独立文档。

| # | 事项 | 现状 | 下一步 |
| --- | --- | --- | --- |
| 1 | 契约层混入实现 | `domain/depth_visualization.py` 有 34 行 numpy 渲染，`trajectory_telemetry.py` 有约 90 行快照编解码，`engine_family.py` 是 `config/simulators.yaml` 的 schema | 三块分别搬去 `runtime`、`runtime`、`config`。搬迁会牵动 `config/loader.py` 与 `adapters/http/schemas.py`，单独立项 |
| 2 | 端口签名漂移 | `ReturnHome.return_home`、`StreamingViews.set_view_mode`、`Vehicle` 的 `tracking_limits` 与 `attach_loop` 三处声明与实现不一致；`tests/test_architecture.py:152` 只查 `hasattr` 不查签名 | 先把签名收成一份，再给端口加签名比对测试。顺序不能反，否则契约测试先红 |
| 3 | 过渡委派残留 | 三个引擎类的 `execute_segment` 与 `attach_loop` 在生产路径零调用；`GuidedVehicle` 八个方法只有 `current_pose` 被调用 | 等第 2 项完成后删委派，把 `GuidedVehicle` 收敛成真正的契约适配层 |
| 4 | 重复实现 | `ContinuousExecution` 与 `QueueFollower` 近乎逐行相同；`segment_motion` 的 `plan_segment`、`quintic`、`duration_for` 只被测试使用；`limit_vector` 与 `algorithms.bounded` 两份；`relative_to_world` 在契约层与计算层各一份 | 每处先判哪份进生产路径，再删另一份。`segment_motion` 整块可以直接删 |
| 5 | 执行服务自检失效 | `ExecutionService.collision_state()` 恒返 `False`，`check_safety()` 永不触发；真实碰撞状态在引擎端口里有实现，退避由 `ControlLoop._guard` 独立完成 | 要么让 `collision_state` 转发端口，要么删掉这条自检分支。现状是两套语义并存 |
| 6 | 配置阈值无标定出处 | 越点走廊容差 1.0 m、`FreeSpaceProfileSettings` 十个阈值、三套停滞判据、30 ms 段周期与 `control_hz` 无关 | 逐条加标定脚本与出处注释；段周期要么服从 `control_hz`，要么在配置里写死并注明 |
| 7 | 跨层依赖例外未被测试约束 | `adapters` 依赖 `workflows` 的提示词常量；`assembly` 与 `simulator` 双向依赖；`tests/test_layering.py` 没把 `adapters` 当来源层检查 | 扩来源集并加显式白名单，白名单外新增依赖即红 |
| 8 | HTTP 契约不一致 | 视频不支持在 404 与 501 两种状态；能力位不可得时省略键而非 `null`；缺 hub 时 `/api/trajectory` 静默返回空快照 | 统一成一种状态与一种空值表达，前端同步；静默返回空要改成显式错误 |
| 9 | 进程与线程可靠性 | 子进程协议 fd 先读到 EOF 再读 `stdout`，有 64 KiB 死锁风险；线程版 worker 无强杀；后台线程 `join` 超时即放走；`ModelCallService` 挂死时 `_busy` 常真 | 子进程读循环改双路并发；线程版补强制中止；挂死路径加外部看门狗 |
| 10 | 槽位校验缺失 | 引擎构造会话后不比对其声明槽位与实物槽位；UE5 的端口在会话构造里内联填，没有 `build_ports` | 先给 UE4 加 `describe` 与 `build_ports` 的比对，再给 UE5 抽 `build_ports` |

## 已确认不改的

以下几项讨论过，决定保持现状，理由记在这里防止反复。

| 项 | 理由 |
| --- | --- |
| 契约层导出面 116 个名字 | 其中 112 个确有跨层调用点。为凑设计时估的 44 而把调用点改到深层路径，会破坏「跨层只走层入口」 |
| 视觉目标连续路径不并入 `StreamRunner` | 每次取帧对应一次确认与重规划，节拍与巡航不同；硬并会让节拍与遥测无法验证 |
| 引擎 `execute_segment` 保留在端口上 | 契约需要这个方法表达「按段执行」；生产路径暂走 `ControlLoop` 是第 3 项要解决的实现问题，不是契约问题 |
| `basic_motion` 只有两个结束状态 | 数值动作任务本身没有中间态可言；补预算属于第 6 项的扩展，不是缺陷 |
