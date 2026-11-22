# 重构流进度台账

每条流的最后一次验收由 `scripts/verify_stream.sh <工作区路径>` 产出。该脚本依次跑全量 `pytest`、`scripts/check_commit_msg.py` 逐条检查主干以来的提交、`git grep` 查冲突标记，任一步失败退出 1。

主干取 `origin/HEAD`（当前指向 `origin/main`），没有远端时取 `main`。检查范围是 `主干..HEAD`。

## 台账

| 流 | 状态 | 内容 |
| --- | --- | --- |
| lint | 已合并 | 提交信息检查脚本与 8 个用例 |
| config | 已合并 | planner/config/ 八文件 |
| controller | 已合并 | assembly/controller/ 六协作者 |
| prompts | 已合并 | 提示词模板化，另一次收口外置剩余片段 |
| tools | 已合并 | streaming_views 归位 |
| stage1 | 已合并 | 三包改名、纯函数搬家、460 条 import 走入口 |
| tests | 已合并 | 分层、入口、公开面三条守门测试 |
| sec | 已合并 | verify_stream.sh 与本台账 |
| req | 已合并 | TASK_KIND 与 REQUIRED_PORTS 常量、assembly/requirements.py |
| contract | 已合并 | 运动模型、命令分型、stream_command、碰撞状态 |
| tasks | 已合并 | workflows/common/{evidence,budget}.py |
| tasks2 | 已合并 | workflows/common/streaming.py |
| assembly | 已合并 | assembly/factories/ 四文件 |
| docs | 已合并 | 五份文档路径对齐 |
| loop | 已合并 | ControlLoop、GuidedVehicle、QueueFollower |
| collide | 已合并 | CollisionRecovery 下沉、删任务层镜像、删白名单 |
| session | 已合并 | projectairsim 会话拆四文件 |
| surfacea | 已合并 | 契约层与端口层入口校准 |
| surfaceb | 已合并 | 算法层三个二级入口、执行层入口收口 |
| surfacec | 已合并 | 任务层与装配层入口收口、跨层深层 import 清零 |
| ports | 已合并 | worker 协议迁 simulator、端口 13 文件 |
| entry2 | 已合并 | 任务层与装配层入口按实际收口 |
| domainslim | 已合并 | HTTP、接近、遥测三类类型搬出契约层 |
| drone | 已合并 | projectairsim 无人机拆 flight 与 commands |
| serverapi | 已合并 | HTTP 入口拆 schemas 与 wiring |
| shared | 已合并 | 算法层补齐走廊与限幅四个入口名 |

## 收尾

主干 `561f3be`：1217 条用例通过，112 条提交信息通过检查，无冲突标记，无未合并分支，无残留工作树。

遗留项见 `COMMAND.md` 第八节。
