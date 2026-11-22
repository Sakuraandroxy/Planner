# 重构流进度台账

每条流的最后一次验收由 `scripts/verify_stream.sh <工作区路径>` 产出。该脚本依次跑全量 `pytest`、`scripts/check_commit_msg.py` 逐条检查主干以来的提交、`git grep` 查冲突标记，任一步失败退出 1。

主干取 `origin/HEAD`（当前指向 `origin/main`），没有远端时取 `main`。检查范围是 `主干..HEAD`。

## 台账

| 流 | 分支 | 工作区 | 当前状态 | 最近一次验收结果 |
| --- | --- | --- | --- | --- |
| lint | `impl/lint` | 无 | 分支已建，无新增提交 | 未执行 |
| config | `impl/config` | 无 | 分支已建，无新增提交 | 未执行 |
| controller | `impl/controller` | 无 | 分支已建，无新增提交 | 未执行 |
| prompts | `impl/prompts` | 无 | 分支已建，无新增提交 | 未执行 |
| tools | `impl/tools` | 无 | 分支已建，无新增提交 | 未执行 |
| docs | `impl/docs` | `/roxy/kedior/Planner-wt/docs` | 工作区已建，无新增提交 | 未执行 |
| sec | `impl/sec` | `/roxy/kedior/Planner-wt/sec` | 工作区已建，1 条新增提交（`e96e422`） | 2026-11-22 通过（1105 例，29 条提交信息，无冲突标记） |
| stage1 | `impl/stage1` | `/roxy/kedior/Planner-wt/stage1` | 工作区已建，无新增提交 | 未执行 |

三条带工作区的分支原本都在 `c8a273d`，与 `refactor/layered-architecture` 的 `6903a13` 相差 14 条提交。
