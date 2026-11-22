> 上级：[项目架构](ARCHITECTURE.md) · [项目说明](../README.md) · [部署文档](DEPLOYMENT.md)
> 层文档：[domain](layers/domain.md) · [ports](layers/ports.md) · [algorithms](layers/algorithms.md) · [runtime](layers/runtime.md) · [workflows](layers/workflows.md) · [assembly](layers/assembly.md) · [adapters](layers/adapters.md) · [simulator](layers/simulator.md) · [simulators](layers/simulators.md)

# 开发文档

写给两类读者：接手这个仓库的工程师，以及只靠文档就要动代码的 agent。先读「先读什么」，再按任务类型挑一节。

## 1 先读什么

| 你要做的事 | 先读 |
| --- | --- |
| 跑起来、改配置、换机器 | [部署文档](DEPLOYMENT.md) |
| 搞清一次任务从指令到轮子怎么走 | [项目架构](ARCHITECTURE.md) 第 3 节 |
| 改某层的实现 | 该层的层文档，见文首链接 |
| 新增一类任务 | 第 5 节 |
| 接一套新仿真器 | 第 6 节 |
| 加一个 HTTP 端点 | 第 7 节 |
| 改契约类型或端口 | 第 8 节 |

## 2 仓库布局

```
planner/
  domain/        契约：类型、常量、引擎能力位、运动模型
  ports/         契约：13 个 Protocol 文件
  algorithms/    计算：几何、深度、校验、运动律、补点策略
  runtime/       执行：控制循环、队列、滑窗、碰撞退避、汇
  workflows/     任务：三类任务包与公共编排
  assembly/      装配：装配、路由、需求求交、控制器、配置
  adapters/      入口：HTTP、模型客户端、提示词
  simulator/     进程栈：子进程监督、worker 协议、xpra
  simulators/    引擎实现：UE5 的 projectairsim、UE4 的 airsim
console/         静态控制台，无需构建
config/          四层配置，见部署文档第 3 节
scripts/         启动、标定、冒烟、提交检查、验收
tests/           87 个测试文件
docs/            本套文档与重构档案
```

依赖方向只能从下往上：`domain`、`ports` 被所有层引用，`algorithms` 只依赖契约，`runtime` 依赖契约与算法，`workflows` 依赖前三层，`assembly` 依赖前四层，`adapters` 与 `simulator` 依赖装配与契约，`simulators` 实现端口。

## 3 硬性规矩

这些是测试守着的不变量，不是风格建议。

| 规矩 | 谁在守 |
| --- | --- |
| 跨层只能 import 层入口 `planner.<层>` 或已存在的二级入口，不能指向层内普通模块 | `tests/test_layer_entries.py` |
| 每层只能依赖允许的下游层 | `tests/test_layering.py` |
| 层入口 `__init__.py` 的 `__all__` 必须与层文档的名字表逐名一致 | `tests/test_public_surface.py` |
| 提交标题动词开头、三十字内、不并列 | `scripts/check_commit_msg.py`、`tests/test_commit_msg.py` |
| 可跑任务只由「任务包声明的必需端口 ⊆ 引擎已填槽位」推导 | `tests/test_requirements.py`、`tests/test_capability_gating.py` |

新增对外名字时，要同时改三处：实现所在模块、层入口 `__all__`、层文档的名字表。少一处测试就红。

## 4 日常命令

```bash
conda activate vln
cd /roxy/kedior/Planner

python -m pytest tests/ -q                      # 全量
python -m pytest tests/test_control_loop_segment.py -q   # 单文件
python -m pytest tests/ -q -k collision         # 按名字筛

ruff check planner tests scripts                # 静态检查
bash scripts/verify_stream.sh /roxy/kedior/Planner      # 提交前总验收

python scripts/serve_planner.py                 # 起服务
python scripts/check_commit_msg.py <消息文件>    # 提交信息自检
```

`verify_stream.sh` 依次跑全量用例、逐条检查主干以来的提交信息、查冲突标记，任一步失败即退出非零。合并前跑它，不要只看 pytest。

## 5 新增一类任务

1. 在 `planner/workflows/<新包>/` 建包，`__init__.py` 声明 `TASK_KIND` 与 `REQUIRED_PORTS`，见 `docs/layers/workflows.md`。
2. 实现工作流类，兑现 `planner.workflows.Workflow` 契约：`prepare`、`run`、`close`，返回 `StageResult`。
3. 判定与结果校验放到该包的 `policy.py`，用 `workflows.common.evidence` 里的校验，不要另写一套。
4. 在 `planner/assembly/requirements.py` 的需求表里登记，让能力位自动出现。
5. 补三类测试：准入与拒绝、正常路径、失败路径。
6. 检查两个引擎的能力位：任务在新引擎上若缺必需端口，会被自动判定为不可跑，不需要额外开关。

不要给任务加开关字段。可跑与否由端口交集决定，删掉开关是设计目标。

## 6 接一套新仿真器

1. 在 `config/simulators.yaml` 加一段：驱动、端口、相机、场景约定、就绪探针。
2. 在 `config/maps.yaml` 加地图，`simulator` 指向新段。
3. 在 `planner/simulators/<新引擎>/` 实现端口：
   - `planner.ports.Vehicle`：`execute_segment`、`stream_state`、`stream_command`
   - `motion_capability()`：返回 `MotionCapability`，无人机为全向、阿克曼车不能横移
   - `collision_state()`：返回 `CollisionState`
   - 观测源、天气、机位按能力位声明为可选
4. 在 `planner/simulator/` 的注册表里登记运行时类。
5. 补 `tests/test_<引擎>_*.py`，至少覆盖连接、观测、命令下发、能力矩阵。

引擎实现只做三件事：下发一条命令、推进一个周期、返回一次状态。采样曲线、限幅、反馈律都在 `runtime` 与 `algorithms`，不要写进引擎。

## 7 加一个 HTTP 端点

1. 路由写 `planner/adapters/http/server_api.py`。
2. 请求与响应模型写 `planner/adapters/http/schemas.py`。
3. 需要装配的东西写 `planner/adapters/http/wiring.py`，不要直接 import 装配层内部模块。
4. 错误映射写 `wiring.py` 的错误表，保持状态码与既有端点一致。
5. 补用例：正常、参数非法、状态不允许三类。

控制台调用新端点时，同时考虑该能力在 UE4 上缺失的情形，前端要有兜底。

## 8 改契约或端口

契约是跨层公共面，改动成本最高。

```bash
grep -rn "planner.domain" planner tests scripts --include=*.py | wc -l   # 有多少引用点
grep -rn "motion_capability\|stream_command" planner tests --include=*.py  # 端口实现与调用
```

规矩：

- 新类型先放 `domain`，只在某一层使用的类型不进契约层。
- 端口方法只增不改；改签名要同时改三个引擎实现与所有调用点。
- 端口新增可选能力时，用能力位表达，不要用布尔开关散落在任务里。
- 删名字前先证明没有层外调用点，删完同步层入口与层文档。

## 9 测试策略

| 类型 | 例子 | 要求 |
| --- | --- | --- |
| 契约与校验 | `test_motion_contract.py`、`test_intrinsics_validation.py` | 正例一条、反例覆盖每个拒绝分支 |
| 纯计算 | `test_depth_geometry.py`、`test_passed_corridor.py` | 固定输入给固定输出，不依赖时钟与网络 |
| 执行循环 | `test_control_loop_segment.py`、`test_control_loop_stream.py` | 假时钟与假载具，断言节拍与停止条件 |
| 任务编排 | `test_mission.py`、`test_stream_runner.py` | 假模型与假窗口，断言调用顺序与失败传播 |
| 控制器 | `test_server_controller.py` | 按端点逐字段断言 |
| 分层守门 | `test_layering.py`、`test_layer_entries.py`、`test_public_surface.py` | 不许放宽，它们是架构的守门人 |

写新测试时优先用 `tests/support.py` 里的假件，别自己造第二套。

涉及时间的测试一律注入 `clock` 与 `sleep`，不要 `time.sleep`。

## 10 调试

| 想知道什么 | 怎么做 |
| --- | --- |
| 一次任务怎么走的 | `GET /api/logs?cursor=0`，日志按阶段打点 |
| 航点从哪来 | `GET /api/trajectory`，含已授权队列与来源 |
| 当前能力位 | `GET /api/status` 的 `capabilities` 字段 |
| 引擎有没有起来 | `output/logs/` 下该次启动的子进程日志 |
| 配置是否被正确加载 | `python scripts/check_profiles.py` |
| 模型端点通不通 | `python scripts/smoke_projectairsim_adapter.py` |

命中计划外的行为时，先确认三件事：能力位是否允许、命令是否被校验拒绝、预算是否耗尽。这三处覆盖了大多数「没动静」的报告。

## 11 提交

```bash
git add <逐路径>
~/miniconda3/envs/vln/bin/python scripts/check_commit_msg.py /tmp/msg.txt
git commit -F /tmp/msg.txt
```

标题格式 `<type>(<scope>): <动词开头的描述>`，三十字内，不用逗号并列。正文说清动机与影响，不写「以及」「同时」这类拼接词。

不 push，不 rebase 别人的分支。每个改动流在自己的分支上累计，合并由主干用 `--no-ff` 一次做完。

## 12 常见陷阱

- **跨层深层 import**：`from planner.runtime.motion.loop import ControlLoop` 会被守门测试拒绝，只能 `from planner.runtime import ControlLoop`。同层内部随便。
- **层入口漏改**：加实现不改 `__all__` 与层文档，`test_public_surface.py` 立刻红。
- **引擎里写采样**：引擎只下发命令与推进周期，采样进 `runtime`。
- **任务里编排碰撞退避**：退避是执行层的被动能力，任务层只读事件。
- **用开关控制可跑任务**：`enabled_tasks` 已删除，改用端口交集推导。
- **改配置场景二进制**：`config/` 下的场景文件由引擎产出，改代码不改它们。
- **在测试里等真实时间**：注入时钟，或用 `tests/support.py` 的假件。

## 13 文档维护

改动落到哪一层，就更新那一层的文档，并检查架构文档里的链路描述是否仍然成立。

三处要一起改的地方：层入口 `__all__`、层文档的名字表、`tests/test_public_surface.py` 的期望集。
