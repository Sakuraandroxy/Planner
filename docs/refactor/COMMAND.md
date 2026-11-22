# 指挥记录

给压缩后的自己看。读完这一份就能继续，不用重建上下文。

## 一、最短恢复路径

1. 读本文件。
2. 读 `docs/refactor/README.md`（契约、裁决、迁移阶段）。
3. 跑 `git log --oneline main..refactor/layered-architecture` 看主干进度。
4. 跑 `git worktree list` 看并行流。
5. 跑 `herdr agent list` 看哪些 agent 还活着。
6. 跑 `~/miniconda3/envs/vln/bin/python -m pytest tests/ -q` 确认基线。

## 二、运行方式

| 项 | 值 |
| --- | --- |
| 主干分支 | `refactor/layered-architecture`，主工作树 `/roxy/kedior/Planner` |
| 并行方式 | 每条流一个 worktree、一个分支、一个 agent，目录 `/roxy/kedior/Planner-wt/<流>` |
| 并发上限 | 同时最多 4 个 agent（6 个会撞模型限速） |
| 提交自检 | `~/miniconda3/envs/vln/bin/python scripts/check_commit_msg.py <文件>`，退出 0 才提交 |
| 看门狗 | `/tmp/orchestrator-watchdog.sh`，盯 `/tmp/orchestrator-heartbeat`，停 900 秒就唤醒主控 |
| 验收顺序 | 分支绿 → 主干跑全量测试 → `git merge --no-ff` |
| 历史形状 | 每个功能流一条侧枝，流内只提交自己的东西，不合主干；收尾一次 `--no-ff` 合并，信息写 `<流名> finished` |
| 禁止 rebase | 不再用 rebase 把功能流压成直线，也不再在流内写 `merge master` |
| 禁止 | 不 push、不改 `config/` 下的场景二进制、不跑模拟器 |

## 三、当前进度

主干已合并：改名、纯函数搬家、滑窗子包、删车道先验、标定工具移出、提示词模板化、配置拆包。

| 流 | 分支 | 状态 | 产物 |
| --- | --- | --- | --- |
| stage1 | `impl/stage1` | 进行中 | 三包改名、纯函数进 algorithms、MotionLimits 合一份、`runtime/window/`、删 lane |
| controller | `impl/controller` | 已合并 | `assembly/controller/` 六个协作者，1132 行的 server_controller 删除 |
| tests | `impl/tests` | 已合并 | 三条守门测试，9 条 xfail 指向后续阶段 |
| sec | `impl/sec` | 已合并 | `scripts/verify_stream.sh`、`docs/refactor/PROGRESS.md` |
| req | `impl/req` | 已合并 | 任务包导出 `TASK_KIND`/`REQUIRED_PORTS`，`assembly/requirements.py` 汇总 |
| contract | `impl/contract` | 已合并 | 阶段 2：`MotionModel`、命令分型、`stream_command`、碰撞状态 |
| tasks | `impl/tasks` | 已合并 | 阶段 5a：`workflows/common/{evidence,budget}.py` |
| assembly | `impl/assembly` | 已合并 | `assembly/factories/` 四文件 |
| docs | `impl/docs` | 已合并 | ARCHITECTURE、AGENT、DEPLOYMENT、README、运动模型文档的路径对齐 |
| loop | `impl/loop` | 进行中 | 阶段 3：`ControlLoop`、`GuidedVehicle`、`QueueFollower` |
| config | `impl/config` | 已合并 | `planner/config/` 八个文件，`config.py` 删除 |
| prompts | `impl/prompts` | 已合并 | 提示词模板外置，渲染逐字一致 |
| tools | `impl/tools` | 已合并 | `streaming_views.py` 移到 `scripts/` |
| lint | `impl/lint` | 已合并 | 提交信息检查脚本与用例 |

测试基线：主干 1168 通过、9 条 xfail。xfail 是未来阶段的守门条目，每个都写了等哪个阶段修。

| xfail | 等哪个阶段 |
| --- | --- |
| workflows 1 条深层 import | 阶段 1g 收尾 |
| assembly 16 条深层 import | 阶段 6 |
| simulators 4 条深层 import | 阶段 7 |
| domain、ports 入口导出面 | 阶段 2 |
| algorithms 二级入口与削面 | 阶段 3 |
| runtime 的 ControlLoop 一族 | 阶段 3、4、5 |
| workflows 的任务包声明 | 阶段 5 |
| assembly 的 12 个名字收口 | 阶段 6 |

合并记录：controller 以 `4218de6 controller finished` 合入，冲突三处（测试 import、bootstrap import、config.py 与 server_controller 的删除），按分支侧或主干侧分别解掉后跑绿。

## 四、阶段 1 剩余

| 步 | 内容 | 归属 |
| --- | --- | --- |
| 1e | 任务判定从 `algorithms/` 搬进各任务包（准入、完成判定、到达评估、目标刷新） | stage1，进行中 |
| 1g | 跨层深层 import 改走包入口（实测 460 条、162 个文件） | stage1，最后做 |
| 阶段 2 | 加 `MotionModel`、`MotionCapability`、两个命令类型、`CollisionState`、`stream_command` | 待开工 |
| 阶段 3 | `ControlLoop` 上移到 runtime，删两个引擎的 tracker 律段 | 待开工 |
| 阶段 4 | 碰撞退避接检查点，删 `enabled_tasks` | 待开工 |
| 阶段 5 | 任务层公共编排与三任务包收口 | 待开工 |
| 阶段 6 | 装配层与控制器收口 | controller 流部分覆盖 |
| 阶段 7 | 引擎会话拆分、提示词收口 | 待开工 |

## 五、债务清单

| 债务 | 位置 | 处理时机 |
| --- | --- | --- |
| 两处反向依赖 | `algorithms/targeting/visual_goal_refresh.py`、`algorithms/completion/visual_goal_verification.py` import `runtime` | 1e 搬走这两个文件后自然消失 |
| 配置包延迟 import | `planner/config/vehicles.py` 里的 `from ..simulator import registry` | 阶段 6 改判据后删除 |
| 深层 import | 460 条指向 `planner.domain.<模块>` 与 `planner.ports.<模块>` | 1g 统一改走包入口 |
| 引擎 tracker | `simulators/projectairsim/motion.py`、`simulators/airsim/motion_tracker.py` | 阶段 3、阶段 7 |
| `enabled_tasks` | `simulator/ports.py` 与两个引擎的 `ENABLED_TASKS` | 阶段 4 删除 |

## 六、裁决增量

前 27 条在 `README.md` 第十节。之后新增：

| 编号 | 争点 | 结论 |
| --- | --- | --- |
| R28 | 提交信息检查脚本动词表过窄 | 扩到 65 个动词，新增「标题不许逗号并列」 |
| R29 | 阶段 1 是否并行 | 按 worktree 分流，同时最多 4 个，验收后 rebase 再 ff 合并 |
| R30 | 配置包对 `simulator.registry` 的延迟 import | 本阶段保留，阶段 6 改判据后删除 |

## 六之二、合并做法

```bash
# 流内：只提交，不 rebase，不合主干
git checkout impl/<流> && git commit ...

# 收尾：主干上合并，一次 merge 提交
git checkout refactor/layered-architecture
git merge --no-ff impl/<流> -m "<流> finished"
~/miniconda3/envs/vln/bin/python -m pytest tests/ -q
```

冲突在合并提交里解，解完把结果跑绿再收工。

## 七、恢复时别忘

- 心跳过期会被看门狗叫醒，`echo "$(date +%s) 当前阶段" > /tmp/orchestrator-heartbeat`。
- 收工要关掉对应 herdr 标签，用户不喜欢闲置窗口。
- 每次合并前先看提交信息是否过了检查脚本，不合格的用 `git filter-branch --msg-filter` 重写。
- 合并完成后在主干跑一次全量测试，再 `git worktree remove` 清掉已合并的 worktree。
