# 任务Workflow扩展接口

当前只实现navigation，但退出规则与模型Prompt已经从Workflow和模型客户端中解耦。新增任务时应提供自己的Workflow、强类型退出上下文、退出策略和Prompt构建器，再在bootstrap中显式组装。

## 退出策略

共用接口位于`planner/workflows/termination/base.py`：

```python
class ExitPolicy(Protocol[ContextT]):
    def begin(self) -> None: ...
    def evaluate(self, context: ContextT) -> ExitDecision: ...
```

`ContextT`是泛型。每类任务定义自己的不可变上下文，不需要把所有任务字段塞进万能字典。`begin()`在每个Stage开始时调用，用于清理该策略的任务内状态；`evaluate()`返回共用的类型化状态及原因。

当前navigation实现位于`planner/workflows/termination/navigation.py`，负责模型复核结果、停滞和轮数上限。未来inspection或tracking应新增各自策略，不能继续向NavigationExitPolicy加入无关分支。安全限制仍由执行层负责，不应被任务退出策略绕过。

## Prompt策略

Prompt接口位于`planner/adapters/trajectory_planner/prompts/base.py`：

- `TrajectoryPromptBuilder`生成轨迹模型文本输入。
- `ProgressPromptBuilder`生成任务进度复核文本输入。

navigation实现位于`prompts/navigation.py`。模型HTTP客户端只调用注入的builder，不判断TaskKind。未来新增任务时，建立对应的Prompt类，并在bootstrap构建该Workflow专用的规划器/复核器：

```python
planner = QwenVLPlanner(..., prompt_builder=InspectionTrajectoryPrompt())
reviewer = VisualProgressReviewer(planner, InspectionProgressPrompt())
workflow = InspectionWorkflow(..., planner, reviewer, exit_policy=InspectionExitPolicy(...))
```

一个Prompt类应只描述一种任务协议。服务器微调模型若要求固定航点数，应使用与训练协议一致的builder；通用API模型可以使用另一实现。执行端对航点数量的兼容范围不代表所有模型都应共享相同提示词。

## 进度复核边界

类型化结果位于`planner/domain/progress.py`；Workflow通过`planner/ports/progress_reviewer.py`依赖复核能力。外部模型请求仍属于Adapter。

当前共用状态为：

```text
ProgressStatus: complete / continue / blocked
ExitStatus: continue / completed / blocked / stuck / limit_reached
```

新增任务可以复用状态枚举，也可以在自己的强类型上下文中加入所需证据。不要让Workflow导入具体复核Adapter，也不要依靠异常字符串判断正常退出分支。

## 新任务接入顺序

1. 在Domain定义TaskKind及专用参数。
2. 定义任务Workflow及其退出上下文、ExitPolicy。
3. 定义任务所需的轨迹Prompt和复核Prompt。
4. 实现或注入所需Capability与Port。
5. 更新任务解析Schema和Factory白名单。
6. 在bootstrap显式组装并注册`TaskKind → Workflow`。
7. 测试单独执行、复合Stage、退出状态、缺失能力和资源清理。

不要根据模型返回动态导入Prompt、ExitPolicy或Workflow。所有实现必须由本地代码显式注册。
