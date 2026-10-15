# Base Planner

当前完整架构与逐步流程见 [CURRENT_ARCHITECTURE_AND_FLOW.md](docs/CURRENT_ARCHITECTURE_AND_FLOW.md)。

这是一个面向 AirSim 的模块化无人机规划器。本分支只注册明确数值动作 `basic_motion`，以及可选的低空平视静态目标接近任务 `visual_goal_navigation`。旧示例 `navigation` 已删除；`base_planner` 分支不受影响。

`basic_motion` 使用任务解析器提取动作参数，本地代码计算航点并以实际位姿和停稳观测确认完成，不调用轨迹模型。`visual_goal_navigation` 显式调用可插拔目标观察能力：当前使用 DeepSeek V4.1 Flash 返回角点，本地深度定位和关联后调用轨迹模型，并在目标附近悬停进行独立完成判断。两种已实现任务默认均可被解析和调度；执行前仍需通过各自的准备检查。

已支持后台续规划滑窗，航段终点和正式采图仍停稳。当前不包含 GroundingDINO、全局地图、完整避障/绕行、持久记忆、动态目标跟随或候选轨迹评分。目标定位、关联及几何完成判断依赖观测质量和人工校准，不能保证所有天气或任意未知环境下成功。

## 当前运行流程

```text
启动CLI → 连接AirSim → 获取控制权、解锁 → 自动起飞并悬停（--no-takeoff可跳过起飞）
  → 等待输入自然语言指令（后续任务复用连接，不重复起飞）
  → TaskParser 解析为 MissionPlan
  → MissionValidator 校验并准备本次使用的Workflow（通过后才执行任务动作）
  → MissionRunner / WorkflowRouter 选择任务工作流
  ├─ basic_motion：停稳观测 → 明确动作转为一个相对航点 → 安全检查
  └─ visual_goal_navigation：停稳观测 → 目标定位/深度几何 → 模型规划 → 协议校验 → 直接执行
       ↓ 共用坐标变换、轨迹校验、平滑运动和执行服务
  MotionPlanner → ExecutionService → AirSim Vehicle / MotionTracker
       ↓ 实测反馈与任务专用完成判定
  StageResult → 下一阶段或停止 → MissionResult
```

每个航点的平移量和偏航量，都相对于前一个预测航点表示。AirSim 执行时将位置与偏航结合控制，因此无人机的移动方向可以与机头/相机朝向不同。相机适配器会读取图像返回的相机位姿，并获取相机 FOV；不会假设相机一定水平朝前。

`basic_motion` 仅在实测位置、yaw、停稳状态及所启用的检查满足时返回 `completed`；单纯发完 API 不能算完成。`visual_goal_navigation` 只有通过目标关联、框中心深度半径上限、停稳观测确认和独立模型完成判定，才返回 `reached`；其依据是本地估计证据结合模型评估，不是仿真器目标位置真值。

## 配置和启动

先检查并按本机环境修改 `config/base.yaml`，再安装依赖：

任务解析、目标检测、完成判断默认分别配置为DeepSeek官方 `https://api.deepseek.com`、`deepseek-flash`（V4.1 Flash）。
三个角色各自独立保留URL、model、api_key、api_key_env和request_format，不合并配置，轨迹规划仍使用原Qwen后端。
运行前分别设置 `TASK_PARSER_API_KEY`、`TARGET_DETECTION_API_KEY`、`TASK_COMPLETION_API_KEY` 为官方密钥；可填同一个实际key，但读取项相互独立。
旧中转密钥未带入官方配置，原配置已完整备份到Git忽略的 `.local-config-archive/`，不参与运行。
主配置的 `request_format: deepseek` 使用官方请求字段：`max_tokens` 与 `thinking: {type: disabled}`。
本地仍以 `max_completion_tokens` 配置令牌预算（解析/完成2048，检测64）；`reasoning_effort: none` 在该格式下转换为关闭思考。
选择 `request_format: openai` 可保留原 `max_completion_tokens` / `reasoning_effort` 格式；旧解析构造参数不带这些设置时仍使用原max_tokens/temperature格式。
协议选择由配置显式注入，不按URL或模型名猜测，不静默改参数重试；提示词、mission_plan_v1、检测框和布尔完成协议保持。
官方说明：[模型与端点](https://api-docs.deepseek.com/quick_start)、[请求字段与思考开关](https://api-docs.deepseek.com/api/create-chat-completion/)。

```bash
pip install -r requirements.txt
```

交互式启动：

```bash
python run_airsim_cli.py
```

直接执行单条指令：

```bash
python run_airsim_cli.py --instruction "上升12米"
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

### 唯一主配置与模型切换

所有运行参数统一维护在 `config/base.yaml`；各任务的完成判定规则放在 `config/tasks/`。
无需复制模板或选择另一份主配置。更换模型时编辑对应角色的 URL/model/api_key_env 等字段，
解析、规划、检测、完成四个角色保持独立，模型输出仍须满足对应协议。
真实密钥使用 TASK_PARSER_API_KEY、TRAJECTORY_PLANNER_API_KEY、TARGET_DETECTION_API_KEY、
TASK_COMPLETION_API_KEY；程序读取进程环境变量，当前不会自动加载 .env 文件。

旧本地后端配置已原样移入 Git 忽略的 `.local-config-archive/`，不参与运行。
`--config` 仍可指定合法文件；指定不存在的旧路径会明确失败，不自动回退到主配置。
目录整理及验证见 [CONFIG_CLEANUP_2026-10-15.md](docs/CONFIG_CLEANUP_2026-10-15.md)。

## 项目目录和职责

```text
run_airsim_cli.py             命令行入口
config/                       配置 schema、加载器和 YAML 配置
  schema.py
  loader.py
  basic_motion.py             基础动作安全空间配置和校验
  base.yaml                   唯一主配置，含独立模型角色及任务配置节
  tasks/visual_goal_navigation.yaml  当前任务的完成判定规则（4米半径上限）
  visual_goal.py              视觉目标任务的类型配置和校验
planner/
  domain/                     任务、观测、位姿、轨迹和结果等领域数据类型
  ports/                      可替换组件的类型化接口
  application/                计划校验、阶段运行和工作流路由
  workflows/                  面向具体任务的编排流程
    basic_motion.py           明确数值动作工作流
    visual_goal_navigation.py 静态目标接近工作流
    termination/              通用退出接口及视觉目标任务预算/搜索策略
  capabilities/               可复用任务能力：目标关联、深度估计与完成条件
  services/                   共享的轨迹变换、校验和执行协调逻辑
    motion/                   从世界轨迹生成运动段与运动曲线
  adapters/                    模型、仿真器和外部服务的具体实现
    airsim/                   AirSim 连接、观测、车辆、运动跟踪和录制
    task_parser/              任务解析提示词、HTTP 调用和响应解析
    target_detection/         可插拔角点检测、独立提示词与严格解析
    diagnostics/              实现 TaskTrace 的本地文件诊断输出
    trajectory_planner/       轨迹规划客户端、提示词和响应解析
      prompts/                针对具体任务的规划提示词构造
  extensions/                  显式白名单注册骨架；当前不是动态插件系统
  bootstrap.py                 创建并组装运行时依赖
  visual_goal_bootstrap.py     新任务依赖装配，不包含执行循环
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
- `planner/ports/` 声明调用方需要的接口，例如 `TaskParser`、`ObservationSource`、`TrajectoryPlanner` 和 `Vehicle`。接口描述“要做什么”，具体方式由适配器实现。
- `planner/application/` 校验解析结果、按顺序运行阶段，并通过 `TaskKind` 路由到工作流；不负责图像处理或 AirSim 细节。
- `planner/workflows/` 编排某类任务的观测、规划、执行、复核和退出流程。不同任务应有各自的工作流，不要不断扩展一个万能工作流。
- `planner/workflows/termination/` 放置退出策略接口及具体任务策略。后续任务可以定义自己的停止/完成判定，不必把规则塞入导航工作流。
- `planner/capabilities/` 放置多个工作流可以共享的任务能力，当前实现静态目标关联、深度估计和 near 完成条件；尚无持久记忆系统。
- `planner/capabilities/basic_motion_conversion.py` 只将动作参数转为相对轨迹；`basic_motion_safety.py` 检查人工确认的空间范围；`completion/basic_motion.py` 进行实测完成判定。它们都不直接调用 AirSim。
- `planner/capabilities/targeting/` 实现目标关联及 RGB 射线到深度的几何映射；`completion/` 实现 near 到达条件。这里不发送模型 HTTP 请求。
- `planner/adapters/target_detection/` 实现独立角点检测后端；本地定位和关联在 `capabilities/targeting/`，不需要启动 GroundingDINO。
- `planner/workflows/visual_goal_navigation.py` 编排新任务；`visual_goal_session.py` 保存一次小任务的局部状态，`termination/visual_goal_navigation.py` 管理搜索、预算及退出策略。
- 视觉工作流直接执行有效Qwen轨迹；`planner/adapters/diagnostics/visual_goal_trace.py` 实现 TaskTrace 接口，保存诊断证据。安全检查是单前视深度支持的采样检查，不是全向避障保证。
- `planner/visual_goal_bootstrap.py` 只装配新任务依赖，由共同 bootstrap 显式调用，不处理任务运行循环。
- `planner/services/` 放置跨适配器复用的流程和转换逻辑；运动段生成及运动曲线位于 `services/motion/`。
- `planner/adapters/airsim/` 封装 AirSim RPC、车辆控制、相机观测、平滑运动跟踪和录制等仿真器相关实现。
- `planner/adapters/task_parser/` 实现任务解析模型协议；`planner/adapters/trajectory_planner/` 实现轨迹规划模型协议。对应提示词和响应解析应留在各自功能边界内。
- `planner/adapters/trajectory_planner/prompts/` 管理规划任务提示词构造。提示词策略与网络传输、响应解析分开，便于不同任务选择不同 prompt。
- `planner/extensions/` 当前只是显式白名单注册骨架，不是自动扫描或动态加载机制。禁止根据模型输出导入或执行任意模块。
- `planner/bootstrap.py` 是组合根：创建具体适配器，通过接口注入依赖，并注册 `TaskKind → Workflow` 映射。
- `tests/` 按测试边界组织。普通自动化测试优先使用 `fakes/`，不依赖 API key、在线模型服务或正在运行的 AirSim；需要仿真器的测试应明确标出前置条件。
- `output/` 仅用于本机运行产物，不放源码或共享配置模板。

## 如何添加一种新任务

例如后续增加巡检、独立搜索或道路跟随时，按显式链路扩展：

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

## 基础数值动作：basic_motion

本分支使用独立的 `BasicMotionWorkflow`，不再保留示例 `navigation` 工作流。任务解析模型将明确的数值命令转换为动作参数，基础动作流程**不请求轨迹规划模型或目标定位模型**，直接生成相对轨迹，复用世界坐标转换、平滑运动和统一执行服务。

例如“上升12米”的解析结果为：

```json
{
  "protocol_version": "mission_plan_v1",
  "stages": [{
    "stage_id": "stage_1",
    "kind": "basic_motion",
    "parameters": {
      "instruction": "上升12米",
      "forward_m": 0,
      "right_m": 0,
      "up_m": 12,
      "yaw_right_deg": 0
    }
  }]
}
```

参数为相对**该阶段起始实际姿态**的动作：`forward_m` 正向前、负向后；`right_m` 正向右、负向左；`up_m` 正上升、负下降；`yaw_right_deg` 正右转、负左转。转换为运动协议时，上升12米对应 `[0,0,-12,0]`，NED 的正 Z 向下约定不变。

- “左转90度，然后前进5米”拆为两个阶段；第二阶段以前一阶段执行后的实际朝向为基准。
- “向前8米、向右5米，同时右转15度”是一个同步动作阶段。
- 支持单步左右180°掉头，内部保留旋转方向；旧的仅起终点构造仍默认最短转向。外部航点仍为四维数组，不改变训练或服务器输出协议。
- 不猜测缺失距离/角度，不支持绝对高度或世界坐标命令。道路跟随、绕行、穿越、动态目标等未实现任务明确拒绝；若长任务含不支持部分，解析器应拒绝整条任务，不执行一个省略了剩余动作的计划。

### 安全前置条件及配置迁移

主配置中的 `basic_motion.free_space.confirmed` 默认为 `false`，此时任务校验会拒绝包含基础动作的整份计划。实验前需人工确认一个**固定世界 NED 坐标的静态无障碍盒**，在本地配置填写三个轴的最小/最大边界，并核实机体和跟踪误差余量后才允许设置 `confirmed: true`：

```yaml
basic_motion:
  free_space:
    confirmed: false
    minimum_world_ned_m: null  # 核实后填 [最小x, 最小y, 最小z]
    maximum_world_ned_m: null  # 核实后填 [最大x, 最大y, 最大z]
    clearance_radius_m: 1.0   # 初值；须包含机体尺寸及位置/跟踪误差余量
```

此空间不会随无人机移动，不是单张前视深度生成的全向避障地图。流程只允许完整直线航段的起终点及球形安全包络位于盒内；纯旋转也要满足净空。场景、障碍或边界有效性变化后，应撤销确认并重建运行时。基础动作使用自己的自由空间检查；视觉任务的相对高度边界可单独配置。

执行前会停稳观测、检查轨迹/空间、复核观测起点是否漂移；仅当本工作流开启碰撞检查时读取碰撞状态。执行后再次停稳观测，按 `motion.position_tolerance_m`、`motion.yaw_tolerance_deg` 校验实际终点。只有测量确认到达并停稳才为 `completed`。执行失败或未到达时取消/悬停，不自动重复原始相对动作；停止 API 成功本身不算停稳确认。

旧本地配置如含 `navigation:` 块，需删除，并加入以上 `basic_motion:` 配置；否则加载器会明确报迁移错误。现有运行配置已合并到唯一主配置，旧本地后端参数及凭据保存在忽略的本地归档；其他外部旧配置仍需按此规则迁移。基础动作仍需要任务解析服务；关闭视觉目标任务时不会启动或调用轨迹规划、目标定位后端，也不要求未使用的轨迹后端 API key。

**启动注意：**默认CLI启动后先连接、获取控制权、解锁、起飞并悬停，再等待输入任务；`--instruction`也在起飞完成后解析执行。各阶段仍在任务动作执行前完成校验与准备；任务被拒绝时不执行其动作，交互模式继续等待输入。起飞不受后续任务净空确认项阻止，启动前须确认起飞空间；`free_space.confirmed: false`仍阻止基础动作执行。已在空中且准备好任务起点时可用`--no-takeoff`跳过自动起飞。先只读检查配置，再在仿真器和起飞条件就绪后启动：

```powershell
python -c "from config import load_config; c=load_config('config/base.yaml'); print('basic_motion_space_confirmed:', c.basic_motion.free_space.confirmed)"
python run_airsim_cli.py --config config/base.yaml --no-takeoff --record-camera-api --camera-record-mode frames --camera-record-fps 5
```

动作阶段总预算使用 `airsim.move_timeout_s`，包括观测、执行及完成复核；RPC 阻塞仍可能造成超时滞后，不承诺硬实时。此次验证使用离线替身，尚未验证真实任务解析服务或 AirSim 飞行。

## 低空平视视觉目标导航：当前实现

本任务显式调用目标检测模型，只返回两个归一化bbox角点；本地使用框中心对应的原始深度估计目标位置。
首次按任务描述找目标，随后用参考裁剪图识别同一目标；每次重新估计中心位置，不匹配固定物理表面点。
目标不可见时有限原地搜索；无需确认起点或搜索净空。不使用邻域比例、点云或路径包络审核。

Qwen每次返回5个NEW四维点，基本协议/数值和锚点有效性校验通过后，整批依序执行。
不再执行场景空间审核、6m/8s批准前缀或转角裁剪。速度/加速度/yaw控制、到位、停止和总预算保持。
滑窗预取不改变控制权；规划次数用完只停止申请新点，已有点继续执行。

进入当前框中心目标位置4米半径后暂停并保留队列。停稳重新检测、测距，按0.3m余量确认上限，
连续两次新鲜观测后调用独立完成判断。false保留并恢复原世界目标，true清空后进入下一阶段。
碰撞检查按工作流默认关闭；基础动作free_space及BasicMotionSafety保持。

主配置仅使用config/base.yaml，视觉完成条件在config/tasks/visual_goal_navigation.yaml。
检测预算64，解析/检测/完成各自配置DeepSeek官方服务并关闭思考；全部URL/model/key彼此独立。
旧审核、净空和多表面候选参数已删除，旧配置会明确拒绝。重启CLI加载新协议与配置。

详细模块与完整流程见[当前架构](docs/CURRENT_ARCHITECTURE_AND_FLOW.md)。
方案、自审、完整检测Prompt、接口迁移和验证见[直接执行与框中心定位](docs/DIRECT_QWEN_BBOX_CENTER_2026-10-15.md)。
新增任务按[Workflow扩展指南](docs/WORKFLOW_EXTENSION.md)接入。

本次559项离线测试通过；未请求真实模型、连接AirSim或触发飞行，实际模型定位和飞行效果仍待实验确认。
