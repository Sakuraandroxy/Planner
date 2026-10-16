# Base Planner

当前完整架构与逐步流程见 [CURRENT_ARCHITECTURE_AND_FLOW.md](docs/CURRENT_ARCHITECTURE_AND_FLOW.md)。

这是一个面向 AirSim 的模块化无人机规划器。本分支只注册明确数值动作 `basic_motion`，以及可选的低空平视静态目标接近任务 `visual_goal_navigation`。旧示例 `navigation` 已删除；`base_planner` 分支不受影响。

`basic_motion` 使用任务解析器提取数值动作，按阶段起点实际位姿换算目标后直接调用 AirSim 运动 API，不采图或进行飞行条件与到位审核。`visual_goal_navigation` 显式调用可插拔目标观察能力：当前使用 DeepSeek V4.1 Flash 返回角点，本地深度定位和关联后调用轨迹模型，并在目标附近悬停进行独立完成判断。两种已实现任务默认均可被解析和调度；视觉任务保留其专属检查。

目标检测独立提示词要求仅返回归一化角点 `[[x1,y1],[x2,y2]]` 或未识别时的 `[]`，并提供两种正确格式示例。首次定位和绑定后定位的 User Prompt 共用末尾格式要求与具体框示例；坐标始终相对当前整张图，原点在左上角，x向右、y向下，不相对参考裁剪图。示例仅说明格式；提示词不能保证模型每次遵守，原有解析与错误处理保持不变。

视觉任务开启滑窗时使用连续速度跟踪：普通航点经过时不停稳，运动中独立采图并在线补点；进入任务半径时才暂停并采集停稳证据。队列耗尽允许悬停等待。当前不包含 GroundingDINO、全局地图、完整避障/绕行、持久记忆、动态目标跟随或候选轨迹评分。目标定位、关联及几何完成判断依赖观测质量和人工校准，不能保证所有天气或任意未知环境下成功。

视觉导航的规划 System Prompt 明确要求仅输出一个含5个四维新增航点的完整 JSON 数组；User Prompt 最末尾再次强调格式，并附不带 Markdown 代码围栏的正确输出示例。示例仅说明格式，实际数值仍由模型依据当前任务与观测生成。这是提示词约束，不保证模型每次遵守；实际返回仍由现有解析器检查。请求参数、坐标约定及其他任务提示词保持原样。

视觉导航在输出要求之前集中给出“新增轨迹规划参考”：pending 非空时，使用其预测末端机体系中的目标向量；为空时使用当前观测机体系。目标水平偏角由本地 `atan2(right, forward)` 计算并转换为度，正值向右、负值向左；水平距离接近零时标为未定义。该角度仅描述新增轨迹起点的目标方向，不是强制转角，也不覆盖模型 yaw。提示词说明转角逐点累计，并要求随预测平移和转向更新目标方向；原始“左前方”等文字用于识别任务起点的目标。共享滑窗和飞行执行保持原样，不新增轨迹拒绝规则。

## 当前运行流程

```text
启动CLI → 连接AirSim → 获取控制权、解锁 → 自动起飞并悬停（--no-takeoff可跳过起飞）
  → 等待输入自然语言指令（后续任务复用连接，不重复起飞）
  → TaskParser 解析为 MissionPlan
  → MissionValidator 校验并准备本次使用的Workflow（通过后才执行任务动作）
  → MissionRunner / WorkflowRouter 选择任务工作流
  ├─ basic_motion：读取当前位姿 → 数值动作换算目标 → AirSim直接执行 → API执行结束
  └─ visual_goal_navigation：停稳观测 → 目标定位/深度几何 → 模型规划 → 协议校验 → 直接执行
       ↓ 共用坐标变换、轨迹校验、平滑运动和执行服务
  MotionPlanner → ExecutionService → AirSim Vehicle / MotionTracker
       ↓ 实测反馈与任务专用完成判定
  StageResult → 下一阶段或停止 → MissionResult
```

每个航点的平移量和偏航量，都相对于前一个预测航点表示。AirSim 执行时将位置与偏航结合控制，因此无人机的移动方向可以与机头/相机朝向不同。相机适配器会读取图像返回的相机位姿，并获取相机 FOV；不会假设相机一定水平朝前。

`basic_motion` 在运动 API 执行结束后返回 `completed`，完成依据为 `api_execution`，不声称实测到位或已核实停稳。`visual_goal_navigation` 首次定位后保存阶段目标记忆，只有通过记忆点半径、停稳观测确认和独立模型完成判定，才返回 `reached`；其依据是位置估计结合当前图像评估，不是仿真器目标位置真值。

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
  tasks/visual_goal_navigation.yaml  当前任务的急停半径（6.5米）和完成距离上限（7米）
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
- `planner/capabilities/basic_motion_conversion.py` 将数值动作换算为世界目标；`ports/basic_motion_vehicle.py` 定义直接执行接口，AirSim适配器实现原生运动API调用。基础动作没有净空、轨迹审核或到位复核模块。
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

本分支使用独立的 `BasicMotionWorkflow`，不再保留示例 `navigation` 工作流。任务解析模型将明确的数值命令转换为动作参数，基础动作流程**不请求轨迹规划模型、目标定位模型或完成判断模型**，经坐标换算后直接执行，共用飞行控制权的协调机制。

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
- 支持有符号偏航动作及完整多圈转动；基础动作不套用视觉轨迹的步长和180°转角上限。视觉轨迹的四维数组及其协议保持原样。
- 不猜测缺失距离/角度，不支持绝对高度或世界坐标命令。道路跟随、绕行、穿越、动态目标等未实现任务明确拒绝；若长任务含不支持部分，解析器应拒绝整条任务，不执行一个省略了剩余动作的计划。

### 直接执行及配置迁移

基础动作仅要求任务解析提供完整、有限的数值参数；不进行净空确认、空间边界、动作距离/角度上限、轨迹审核、起点漂移、碰撞或实测到位检查。旧 `basic_motion:` 配置块及对应的净空/碰撞配置已删除；外部旧配置需移除此块，未知配置仍按通用加载规则报告错误。

单纯平移直接调用 `moveToPositionAsync`；单纯转向调用 `rotateByYawRateAsync`；同步平移与转向使用世界坐标速度及有符号偏航速率的 `moveByVelocityAsync`，时长由距离、速度及转角计算。同步动作不做闭环位置补偿，实际位移和转角可能存在误差。API结束后发送悬停指令，但不等待本地停稳核验。速度使用 `airsim.speed_mps`，转动时间参考 `motion.max_yaw_rate_deg_s`；它们用于执行换算，不限制任务总距离或总转角。

API正常结束即作为本阶段完成依据，结果明确标为 `api_execution`，不冒充实测到位。最终位姿读取仅作诊断，读不到也不否定API执行结果。API异常或用户中断时取消/悬停，不自动重复相对动作。原有共享飞行控制权协调保持，防止与未结束的旧请求并发抢占。

默认CLI启动后先连接、获取控制权、解锁、起飞并悬停，再接受任务；后续基础动作相对各阶段起点计算，例如起飞后“升高10米”使当前NED z减10。已在空中可用 `--no-takeoff`。基础动作不需要独立配置块；真实API及AirSim效果仍需实验验证。

## 低空平视视觉目标导航：当前实现

本任务显式调用目标检测模型，只返回两个归一化bbox角点；本地使用框中心对应的原始深度估计目标位置。
首次按任务描述找到目标并得到有效中心深度后，立即锁定首次位置和参考图。
后续候选只允许在首次点0.5米范围内更新最新可信坐标；远处错车、空框、无效中心深度或检测请求失败不覆盖记忆，继续按原记忆点导航。
仅未绑定时进行有限原地搜索；绑定后不自动重选目标。不使用邻域比例、点云或路径包络审核。

Qwen每次返回5个NEW四维点，基本协议/数值和锚点有效性校验通过后，整批依序执行。
不再执行场景空间审核、6m/8s批准前缀或转角裁剪。速度/加速度/yaw控制、到位、停止和总预算保持。
滑窗预取不改变控制权；规划次数用完只停止申请新点，已有点继续执行。

`sliding_window.enabled: true` 时由 `ContinuousExecution` 连续跟踪队列，普通航点不发送停稳指令。
连续跟踪通过`motion.continuous`配置前瞻距离与平移/偏航jerk。速度预算查看整个授权队列，提前协调短航段的转向时间、转弯及队尾停车；中间点和批次切换延续速度与加速度状态。模型有符号yaw仍逐点核实，原地旋转保持位置，零位移零yaw点不额外产生停留。普通航点不必停稳，短段大转角和急弯会提前降速；这些参数不影响基础动作和串行停稳运动。
飞行期间约10Hz采样控制诊断到有界内存缓冲，阶段退出并处理停止请求后保存为实验目录下的`control_samples.csv`，包含控制周期、指令/实测速度、指令加速度、yaw误差及AirSim实测俯仰/横滚。该CSV用于分析点头，不代表姿态已验证平稳。具体算法与验收边界见[连续运动跟踪](docs/CONTINUOUS_TRACKING_2026-10-16.md)。
`planning_interval_s: 5.0` 以实际提交时间为基准；首次立即请求，正常请求至少间隔5秒。
最多一个Qwen请求在途，pending最多5点、本地缓冲最多10点；队列大于5点时延后提交，
队列耗尽时允许提前补点，并受失败退避、请求次数和阶段预算约束。5秒不保证恰好剩3点。
只读连接在运动中采图，通过图像相机位姿和人工配置的安装外参恢复图像时刻机体位姿，
按AirSim时间戳对齐pending与实测历史；不把旧预测平移到新位置。

目标检测独立按5秒最小间隔刷新。Qwen收到目标相对图像时刻无人机及预测pending末端的两组
`[前,右,下]` 坐标，以及来源和年龄；不再接收目标bbox或目标世界坐标。
当前无人机世界位姿、相机几何、原始双语指令和执行上下文仍保留。
当`keep_target_in_view=true`时，视觉导航专属User Prompt追加转向引导：先考虑每个新增点的平移，再依据该位置的目标方向调整增量yaw，尽量保持目标靠近画面水平中央；有pending时从预测队尾姿态考虑新增动作，后续点使用更新后的机体系。大角度调整时可减少平移或原地转向，水平距离接近零时不强行推导转角；yaw不保证垂直构图或目标可见。这是模型规划引导，不增加执行拒绝或失败条件。
每次规划根据本次观测位姿和记忆世界坐标重算相对向量，不复用旧相对坐标。
记忆坐标在当前静态目标阶段内有效，当前图像和规划锚点仍须满足新鲜度要求；记录真实来源与年龄，不声称当前目标可见。

进入记忆目标点6.5米半径（`pause_radius_m`）后暂停并保留队列。距离工程余量由`distance_uncertainty_margin_m`配置，当前主配置为0米，停稳后的记忆距离不超过7米（`radius_m`）即可通过距离准入；规划接近提示使用6.5米，避免提前停在两条边界之间。旧配置省略`pause_radius_m`时继续与`radius_m`共用半径。当前距离计算仍为三维距离。
连续两次新鲜观测后调用独立完成判断，无需先获取新的检测框或目标中心深度。
完成器接收当前RGB/深度展示与首次参考图，必须确认原目标及任务要求；记忆距离不是当前实测目标表面距离。
false保留并恢复原世界目标，true清空后进入下一阶段。
连续与串行视觉导航均不再因短期目标距离改善不足而判失败，允许先横向调整或绕行后再接近；任务总时限、规划次数等现有退出条件仍生效。旧配置中的`progress_window_rounds`和`minimum_progress_m`已删除，需移除后再加载。
视觉导航碰撞检查默认关闭；基础动作直接执行，不调用碰撞接口。

主配置仅使用config/base.yaml，视觉完成条件在config/tasks/visual_goal_navigation.yaml。
检测预算64，解析/检测/完成各自配置DeepSeek官方服务并关闭思考；全部URL/model/key彼此独立。
旧审核、净空和多表面候选参数已删除，旧配置会明确拒绝。重启CLI加载新协议与配置。

详细模块与完整流程见[当前架构](docs/CURRENT_ARCHITECTURE_AND_FLOW.md)。
方案、自审、完整检测Prompt、接口迁移和验证见[直接执行与框中心定位](docs/DIRECT_QWEN_BBOX_CENTER_2026-10-15.md)。
本次连续执行、五秒调度、相对目标协议及验证边界见[连续滑窗迁移](docs/CONTINUOUS_WINDOW_2026-10-16.md)。
首次目标记忆、配置迁移、完成判定证据和本次验证见[静态目标记忆迁移](docs/STATIC_TARGET_MEMORY_2026-10-16.md)。
新增任务按[Workflow扩展指南](docs/WORKFLOW_EXTENSION.md)接入。

测试默认使用离线替身；最新测试数量和结果以本次交付记录为准。没有连接AirSim或真实模型时，不声称实飞通过；实际模型定位、姿态平稳程度和连续飞行效果仍待实验确认。首次位置估计可能错误，记忆更新约束不构成严格身份识别，也不保证规划模型实际选择正确方向。
