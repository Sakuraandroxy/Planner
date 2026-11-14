# Base Planner

这是一个面向 AirSim 的基础无人机规划器。当前版本保留任务解析、视觉轨迹规划、相机观测、轨迹校验与 AirSim 飞行控制等最小执行链路；暂不包含目标检测与目标绑定、记忆、语义完成判定、避障与恢复、候选轨迹评分、滑动窗口队列、Web UI，以及旧版快慢速运行模式。

## 当前运行流程

```text
自然语言指令
  → TaskParser 解析为 MissionPlan
  → MissionValidator 校验
  → MissionRunner / WorkflowRouter 选择任务工作流
  → 获取 RGB 与对齐深度观测
  → 轨迹模型输出 1～5 个相对航点 [dx, dy, dz, dyaw_deg]
  → 转换为世界坐标并校验
  → MotionPlanner 生成平滑运动段
  → ExecutionService / AirSim Vehicle 执行
```

每个航点的平移量和偏航量，都相对于前一个预测航点表示。AirSim 执行时将位置与偏航结合控制，因此无人机的移动方向可以与机头/相机朝向不同。相机适配器会读取图像返回的相机位姿，并获取相机 FOV；不会假设相机一定水平朝前。

当前没有语义层面的任务完成检测。因此，`Mission executed` 只表示已验证的轨迹阶段执行完成，期间没有检测到碰撞或 API 错误；它不代表已经确认到达用户描述的目标。

## 配置和启动

先检查并按本机环境修改 `config/base.yaml`，再安装依赖：

```bash
pip install -r requirements.txt
```

交互式启动：

```bash
python run_airsim_cli.py
```

直接执行单条指令：

```bash
python run_airsim_cli.py --instruction "Follow the road"
```

连接 AirSim 但不自动起飞：

```bash
python run_airsim_cli.py --no-takeoff
```

通过 Camera API 连续保存相机实际画面（包括配置了 pitch/yaw/roll 的相机）：

```bash
python run_airsim_cli.py --record-camera-api --camera-record-mode frames
```

录制图像、深度和元数据默认保存到 `output/camera_api/<时间戳>/` 下。录制器使用独立的 AirSim RPC 连接，避免主连接等待飞行指令时停止采集。

### 使用 DeepSeek 轨迹规划

将 `config/deepseek.example.yaml` 复制为本地配置 `config/deepseek.yaml`，填写 API 凭据和需要的模型参数，然后启动：

```powershell
python run_airsim_cli.py --config config/deepseek.yaml --record-camera-api --camera-record-mode frames
```

真实 API key 不要提交到 Git。`config/deepseek.yaml` 是本地配置文件；提交共享配置时只提交不含密钥的示例文件。

## 项目目录和职责

```text
run_airsim_cli.py             命令行入口
config/                       配置 schema、加载器和 YAML 配置
  schema.py
  loader.py
  base.yaml                   默认配置
  deepseek.example.yaml       不含密钥的 DeepSeek 配置模板
planner/
  domain/                     任务、观测、位姿、轨迹和结果等领域数据类型
  ports/                      可替换组件的类型化接口
  application/                计划校验、阶段运行和工作流路由
  workflows/                  面向具体任务的编排流程
    termination/              工作流退出策略接口及导航退出策略
  capabilities/               多个工作流可复用的任务能力接口
  services/                   共享的轨迹变换、校验和执行协调逻辑
    motion/                   从世界轨迹生成运动段与运动曲线
  adapters/                    模型、仿真器和外部服务的具体实现
    airsim/                   AirSim 连接、观测、车辆、运动跟踪和录制
    task_parser/              任务解析提示词、HTTP 调用和响应解析
    trajectory_planner/       轨迹规划/复核客户端、提示词和响应解析
      prompts/                针对具体任务的规划提示词构造
  extensions/                  显式白名单注册骨架；当前不是动态插件系统
  bootstrap.py                 创建并组装运行时依赖
  errors.py                    规划器异常类型
tests/
  unit/                        单元测试
  contract/                    接口契约测试
  integration/                 多组件集成测试（默认使用 fake）
  fakes/                       测试用替身组件
output/                        本地运行输出，例如相机画面和日志
```

### 主要目录细分

- `run_airsim_cli.py` 负责命令行参数、连接与关闭、用户输入和结果展示。任务决策、模型请求和 AirSim 业务逻辑不应堆进 CLI。
- `config/schema.py` 定义有类型的配置结构；`config/loader.py` 负责读取并校验 YAML。环境路径、凭据等本机参数应留在本地配置中。
- `planner/domain/` 存放 `MissionPlan`、`Observation`、`WorldPose`、轨迹、进度状态和结果等核心数据。领域对象不应直接调用 API 或读取 YAML。
- `planner/ports/` 声明调用方需要的接口，例如 `TaskParser`、`ObservationSource`、`TrajectoryPlanner`、`ProgressReviewer` 和 `Vehicle`。接口描述“要做什么”，具体方式由适配器实现。
- `planner/application/` 校验解析结果、按顺序运行阶段，并通过 `TaskKind` 路由到工作流；不负责图像处理或 AirSim 细节。
- `planner/workflows/` 编排某类任务的观测、规划、执行、复核和退出流程。不同任务应有各自的工作流，不要不断扩展一个万能工作流。
- `planner/workflows/termination/` 放置退出策略接口及具体任务策略。后续任务可以定义自己的停止/完成判定，不必把规则塞入导航工作流。
- `planner/capabilities/` 用于多个工作流共享的任务能力。目前这里只有接口骨架，尚未实现目标检测或记忆系统。
- `planner/services/` 放置跨适配器复用的流程和转换逻辑；运动段生成及运动曲线位于 `services/motion/`。
- `planner/adapters/airsim/` 封装 AirSim RPC、车辆控制、相机观测、平滑运动跟踪和录制等仿真器相关实现。
- `planner/adapters/task_parser/` 实现任务解析模型协议；`planner/adapters/trajectory_planner/` 实现轨迹规划/进度复核模型协议。对应提示词和响应解析应留在各自功能边界内。
- `planner/adapters/trajectory_planner/prompts/` 管理规划任务提示词构造。提示词策略与网络传输、响应解析分开，便于不同任务选择不同 prompt。
- `planner/extensions/` 当前只是显式白名单注册骨架，不是自动扫描或动态加载机制。禁止根据模型输出导入或执行任意模块。
- `planner/bootstrap.py` 是组合根：创建具体适配器，通过接口注入依赖，并注册 `TaskKind → Workflow` 映射。
- `tests/` 按测试边界组织。普通自动化测试优先使用 `fakes/`，不依赖 API key、在线模型服务或正在运行的 AirSim；需要仿真器的测试应明确标出前置条件。
- `output/` 仅用于本机运行产物，不放源码或共享配置模板。

## 如何添加一种新任务

例如后续增加巡检、搜索或目标跟随时，按显式链路扩展：

1. 在 `planner/domain/` 中增加 `TaskKind` 成员和专属的不可变参数类型，并扩展 `MissionStage` 的参数类型，避免用无约束的通用字典承载所有任务参数。
2. 更新任务解析提示词、响应解析/工厂和 `MissionValidator`。对未知任务类型、缺失参数和错误参数明确拒绝。
3. 在 `planner/workflows/<任务名>.py` 实现独立工作流。为循环设置配置化或明确的上限，并返回可解释的完成、停止和失败结果。
4. 只为该任务接入需要的能力。可复用的任务逻辑放在 `capabilities/`；需要替换实现的外部依赖先在 `ports/` 定义窄接口，再放入合适的 `adapters/` 实现。
5. 在 `planner/bootstrap.py` 装配该工作流，并注册 `TaskKind → Workflow`。由 `WorkflowRouter` 做路由，不要把任务判断分支堆入 `MissionRunner`。
6. 补充输入合法性、错误处理、路由、成功/失败传递及与其他阶段交互的单元、契约和集成测试。

不同任务也可以使用不同的提示词与退出策略：提示词放在对应 adapter 的 `prompts/` 中，由任务工作流/规划器选择；退出策略放在 `workflows/termination/`，通过接口注入相应工作流。避免在一个 prompt 或退出逻辑中混入所有任务的特殊情况。

## 如何添加新模块或外部集成

先判断模块的职责，再决定落点：

- 任务步骤编排：`planner/workflows/`
- 多任务共用的任务能力：`planner/capabilities/`
- 共享的数据转换或执行协调：`planner/services/`
- 模型、仿真器、硬件或网络服务实现：`planner/adapters/`
- 核心数据类型：`planner/domain/`
- 可替换依赖的接口：`planner/ports/`

如果需要更换实现，在 `ports/` 定义足够小且有类型的接口，并通过构造函数注入实现。不要要求所有模块继承一个万能父类：只有确实需要可替换性时才定义 `Protocol`，具体实现使用普通类即可。新增配置时更新 `config/schema.py` 和 `config/loader.py`，再在 `planner/bootstrap.py` 装配。不要使用隐藏的全局注册、导入时副作用或模型控制的动态导入。对于会访问外部服务或仿真器的模块，记录输入输出、异常、超时、状态生命周期和资源所有权。

## 如何适配一种新仿真器

例如增加其他仿真器时，仿真器 SDK、RPC 和坐标系差异都应封装在对应的适配器目录中；工作流和领域层不应直接导入仿真器 SDK。

1. 在 `planner/adapters/<仿真器名>/` 新建适配器包，分别实现 `Vehicle` 和 `ObservationSource`：前者提供当前位置/姿态、执行运动段、取消、悬停及碰撞状态；后者采集并返回统一的 `Observation`。仿真器特有的连接、传感器和运动控制辅助类也放在此包内。
2. 在适配器边界把仿真器数据转换为项目统一的领域语义。`WorldPose`、`MotionSegment`、相机位姿、内参、RGB 和以米为单位的深度都必须语义一致；尤其要核对世界坐标方向、单位、yaw 正方向、角度单位、机体/相机坐标变换及深度定义。若仿真器采用不同坐标系，应在适配器中显式转换并增加正反向转换测试，不要把坐标系特例散落到 workflow 或 prompt。
3. 核对 `Vehicle.execute_segment()` 对 `MotionSegment` 的执行语义：从起点到终点、平移和 yaw 同步、速度/时长限制、悬停与取消行为，以及运行中碰撞检查。仿真器 API 不同可以使用不同控制实现，但要满足同一个 `Vehicle` 契约；低层跟踪器放在该仿真器自己的适配器包中。
4. 处理仿真器生命周期。当前 `run_airsim_cli.py` 直接通过 `Runtime.connection` 调用 AirSim 的 `connect()`、`prepare_vehicle()`、`takeoff()` 和 `shutdown()`；`bootstrap.py` 也直接创建 `AirSimConnection`。所以新增仿真器时，不能只替换 `Vehicle`：应先在 `planner/ports/` 定义通用的会话/生命周期接口（例如 `connect()`、`prepare_vehicle()`、`takeoff()`、`shutdown()`），由各仿真器适配器分别实现；再在 `bootstrap.py` 选择并装配对应组件，让 CLI 依赖该接口而非 AirSim 专属方法。不要把生命周期控制代码复制成多个 CLI 分支。
5. 在 `config/schema.py`、`config/loader.py` 和 YAML 配置中加入必要的仿真器选择与连接、相机、速度、超时等参数。若需要在同一代码版本中选择多种仿真器，应显式配置仿真器类型并在 bootstrap 中白名单分派；避免散落的字符串判断和隐式自动发现。凭据和机器专属地址继续留在本地配置。
6. 检查通用逻辑是否真的与仿真器无关。可复用的 `ExecutionService`、轨迹变换、校验、运动规划和 `ObservationGate` 应继续依赖 `Vehicle` 等接口；如果它们出现具体 SDK 类型或仿真器方法名，应将相关细节移回适配器，或先定义必要的窄接口。
7. 为新适配器增加契约测试：统一的姿态/坐标转换、观测字段和 RGB-深度配对、执行运动段、纯 yaw 旋转、取消/悬停、碰撞上报、连接失败和关闭清理。用 fake 仿真器客户端测试常规路径；需要真实仿真器的测试单独标记并说明启动条件。

完成后，至少确认原有 AirSim 路径仍可运行，并运行 `python -m pytest -q`。如果生命周期接口或领域坐标语义需要调整，应先同步更新对应 Port、CLI/bootstrap 和契约测试，再接入新仿真器；不要让某个仿真器的特殊假设悄悄改变所有现有任务的行为。

## 从入口到 AirSim 的调用关系

```text
CLI
 → PlannerApplication
 → TaskParser
 → MissionValidator
 → MissionRunner
 → WorkflowRouter
 → 对应任务 Workflow
 → 观测 / 轨迹规划 / 校验与坐标变换
 → MotionPlanner
 → ExecutionService
 → AirSim Vehicle 适配器
 → AirSim
```

任务工作流负责任务层的观测、重规划和退出决策；运动跟踪器负责单个运动段中的低层反馈控制。轨迹 API 正常返回或飞完轨迹，不等于已经证明语义任务完成。

## 测试

运行全部自动化测试：

```bash
python -m pytest -q
```

根目录 `docs/` 当前被 Git 忽略，其中的本地文档不会随仓库共享。如需共享项目规范，请放入本 README，或在明确评估后调整忽略规则。
