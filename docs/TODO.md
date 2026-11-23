> 上级：[项目架构](ARCHITECTURE.md) · [开发文档](DEVELOPMENT.md) · [部署文档](DEPLOYMENT.md)
> 层文档：[domain](layers/domain.md) · [ports](layers/ports.md) · [algorithms](layers/algorithms.md) · [runtime](layers/runtime.md) · [workflows](layers/workflows.md) · [assembly](layers/assembly.md) · [adapters](layers/adapters.md) · [simulator](layers/simulator.md) · [simulators](layers/simulators.md)

# 待办与已知缺口

跨层待办目前为空。原先登记的十条已全部处置，处置记录见本节末尾。层内问题写在对应层文档的「已知缺口」小节，不在这里重复。

## 登记规则

只放跨层或需要一次结构改动的事项。一条超过二百字说明它该拆成独立文档。写清三件事：现状（带文件行号）、为什么现在不改、下一步动作。

## 已确认不改的取舍

| 项 | 理由 |
| --- | --- |
| 契约层导出面 118 个名字 | 其中绝大多数确有跨层调用点。为凑设计时估的数字而把调用点改到深层路径，会破坏「跨层只走层入口」这条更重要的约束 |
| 视觉目标连续路径不并入 `StreamRunner` | 每次取帧对应一次确认与重规划，节拍与巡航不同；硬并会让节拍与遥测无法验证 |
| 段拍与流拍是两个配置项 | 段执行每拍要判停稳与剩余距离，采样要细；流执行是命令下发节拍，频率由 `control_hz` 决定。合成一个值会牺牲其中一边 |
| `basic_motion` 只有两个结束状态 | 数值动作任务没有中间态可言；补中间态不是缺陷修复，是需求变化 |

## 处置记录

| 原条目 | 处置 |
| --- | --- |
| 契约层混入实现 | `render_depth` 迁到 `algorithms/depth/render.py`；轨迹快照编解码迁到 `runtime/telemetry.py`；引擎族 schema 迁到 `config/engine_family.py`，`Capabilities` 留在契约层 |
| 端口签名漂移 | `ReturnHome`、`StreamingViews`、`Vehicle`、`hover` 四处签名收成一份，并新增签名一致性测试 |
| 过渡委派残留 | 三个引擎类的 `execute_segment`、`attach_loop` 删除，`execute_segment` 只剩 `GuidedVehicle` 一处实现 |
| 重复实现 | `segment_motion` 与 `ContinuousExecution` 删除，`limit_vector`、`relative_to_world` 各保留一份 |
| 执行服务自检失效 | 恒假的 `collision_state` 与 `check_safety` 删除，碰撞退避只由控制循环消费真实端口状态 |
| 阈值无出处 | 段拍进配置 `motion.segment_period_s`，各阈值注释写明出处或标「未标定」 |
| 跨层依赖未被测试约束 | 九层全部纳入 `tests/test_layering.py`，现存例外写成带理由的白名单 |
| HTTP 契约不一致 | 视频统一 501、能力位缺失写 `null`、轨迹缺 `hub` 返回 503，前端同步 |
| 进程线程可靠性 | 子进程读循环双路并发、线程关闭超时后显式作废、模型调用超时可被接管 |
| 槽位校验缺失 | 两套引擎构造期比对声明槽位与实物，缺项报错并打出槽位名 |
